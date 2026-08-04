#!/usr/bin/env python3
import json
import os
import sys
import threading
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError as exc:
    raise SystemExit("Missing dependency: pyserial. Install with: pip3 install pyserial") from exc

try:
    from PyQt5 import QtCore, QtWidgets
except ImportError as exc:
    raise SystemExit("Missing dependency: PyQt5. Install with: pip3 install PyQt5") from exc


GIMBAL_TOOL_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "Gimbal_update_tool"))
if GIMBAL_TOOL_DIR not in sys.path:
    sys.path.insert(0, GIMBAL_TOOL_DIR)

try:
    import gimbal_py
    from gimbal_py import MAVLink
except ImportError as exc:
    raise SystemExit("Cannot import gimbal_py.py") from exc


SERIAL_CONFIG = {
    "baudrate": 500000,
    "bytesize": serial.EIGHTBITS,
    "parity": serial.PARITY_NONE,
    "stopbits": serial.STOPBITS_ONE,
    "timeout": 0.10,
    "xonxoff": False,
    "rtscts": False,
    "dsrdtr": False,
}
HEARTBEAT_PERIOD_S = 1.0
ATTITUDE_CONTROL_PERIOD_S = 0.10
GCS_SYSTEM_ID = 255
GCS_COMPONENT_ID = 190
GIMBAL_TARGET_SYSTEM = 1
GIMBAL_TARGET_COMPONENT = 154


def debug_data_to_ascii(debug_data):
    raw_bytes = bytearray()
    for value in debug_data:
        raw_bytes.append(value & 0xFF)
    return raw_bytes.split(b"\x00", 1)[0].decode("ascii", errors="replace").rstrip()


def format_mavlink_message(mavmsg):
    data = mavmsg.to_dict()
    if mavmsg.get_type() == "GIMBAL_DEBUGDATA" and "debug_data" in data:
        data["debug_data"] = debug_data_to_ascii(data["debug_data"])
    return json.dumps(data, ensure_ascii=False)


class SerialMavlinkWorker(QtCore.QThread):
    message_received = QtCore.pyqtSignal(str)
    mount_orientation_received = QtCore.pyqtSignal(dict)
    mavlink_source_received = QtCore.pyqtSignal(int, int, str)
    heartbeat_sent = QtCore.pyqtSignal(int, str)
    status_changed = QtCore.pyqtSignal(str)
    error_received = QtCore.pyqtSignal(str)
    stopped = QtCore.pyqtSignal()

    def __init__(self, config, parent=None):
        super().__init__(parent)
        self.config = config
        self._running = True
        self._port = None
        self._mavlink = None
        self._control_lock = threading.Lock()
        self._yaw_rate_dps = 0.0
        self._pitch_rate_dps = 0.0
        self._target_yaw_deg = 0.0
        self._target_pitch_deg = 0.0
        self._have_target = False
        self._heartbeat_count = 0
        self._mount_config_sent = False
        self._center_requested = False

    def stop(self):
        self._running = False
        if self._port is not None:
            try:
                self._port.cancel_read()
            except (AttributeError, serial.SerialException, OSError):
                pass

    def run(self):
        try:
            self._port = serial.Serial(**self.config)
            self._mavlink = MAVLink(self._port, GCS_SYSTEM_ID, GCS_COMPONENT_ID)
            self._mavlink.set_callback(self._handle_message)
            next_heartbeat_time = 0.0
            next_attitude_time = time.monotonic() + ATTITUDE_CONTROL_PERIOD_S
            last_attitude_time = time.monotonic()
            self.status_changed.emit("Connected: %s" % self.config["port"])

            while self._running:
                now = time.monotonic()
                if now >= next_heartbeat_time:
                    self._send_heartbeat()
                    next_heartbeat_time = now + HEARTBEAT_PERIOD_S
                if now >= next_attitude_time:
                    dt = max(0.0, now - last_attitude_time)
                    self._send_attitude_control(dt)
                    last_attitude_time = now
                    next_attitude_time = now + ATTITUDE_CONTROL_PERIOD_S

                read_size = max(1, self._mavlink.bytes_needed())
                chunk = self._port.read(read_size)
                if not chunk:
                    continue
                try:
                    self._mavlink.parse_char(chunk)
                except gimbal_py.MAVError as exc:
                    self.error_received.emit(str(exc))
                except Exception as exc:
                    self.error_received.emit("Parse error: %s" % exc)
        except serial.SerialException as exc:
            self.error_received.emit("Serial error: %s" % exc)
        except OSError as exc:
            self.error_received.emit("I/O error: %s" % exc)
        finally:
            self._close()
            self.stopped.emit()

    def _handle_message(self, mavmsg, *args, **kwargs):
        timestamp = time.strftime("%H:%M:%S")
        self.mavlink_source_received.emit(mavmsg.get_srcSystem(), mavmsg.get_srcComponent(), mavmsg.get_type())
        self.message_received.emit("[%s] %s" % (timestamp, format_mavlink_message(mavmsg)))
        if mavmsg.get_type() == "MOUNT_ORIENTATION":
            data = mavmsg.to_dict()
            self._sync_target_from_mount_orientation(data)
            self.mount_orientation_received.emit(data)

    @QtCore.pyqtSlot(float, float)
    def set_control_rates(self, yaw_rate_dps, pitch_rate_dps):
        with self._control_lock:
            self._yaw_rate_dps = yaw_rate_dps
            self._pitch_rate_dps = pitch_rate_dps

    @QtCore.pyqtSlot()
    def request_center(self):
        with self._control_lock:
            self._yaw_rate_dps = 0.0
            self._pitch_rate_dps = 0.0
            self._target_yaw_deg = 0.0
            self._target_pitch_deg = 0.0
            self._have_target = True
            self._center_requested = True

    def _sync_target_from_mount_orientation(self, data):
        with self._control_lock:
            if self._have_target:
                return
            yaw = data.get("yaw")
            pitch = data.get("pitch")
            if yaw is None or pitch is None:
                return
            self._target_yaw_deg = float(yaw)
            self._target_pitch_deg = float(pitch)
            self._have_target = True

    def _send_heartbeat(self):
        try:
            self._mavlink.heartbeat_send(
                gimbal_py.MAV_TYPE_GCS,
                gimbal_py.MAV_AUTOPILOT_INVALID,
                0,
                0,
                gimbal_py.MAV_STATE_ACTIVE,
            )
            if self._port is not None:
                self._port.flush()
            self._heartbeat_count += 1
            self.heartbeat_sent.emit(self._heartbeat_count, time.strftime("%H:%M:%S"))
        except (serial.SerialException, OSError) as exc:
            self.error_received.emit("Heartbeat send error: %s" % exc)
            self._running = False

    def _send_attitude_control(self, dt):
        with self._control_lock:
            yaw_rate = self._yaw_rate_dps
            pitch_rate = self._pitch_rate_dps
            center_requested = self._center_requested
            self._center_requested = False
            if yaw_rate == 0.0 and pitch_rate == 0.0 and not center_requested:
                return
            if not center_requested:
                self._target_yaw_deg += yaw_rate * dt
                self._target_pitch_deg += pitch_rate * dt
                self._target_pitch_deg = max(-90.0, min(90.0, self._target_pitch_deg))
            target_yaw = self._target_yaw_deg
            target_pitch = self._target_pitch_deg

        try:
            if not self._mount_config_sent:
                self._send_mount_configure()
                self._mount_config_sent = True
            self._mavlink.command_long_send(
                GIMBAL_TARGET_SYSTEM,
                GIMBAL_TARGET_COMPONENT,
                gimbal_py.MAV_CMD_DO_MOUNT_CONTROL,
                0,
                target_pitch,
                0,
                target_yaw,
                0,
                0,
                0,
                2,
            )
        except (serial.SerialException, OSError) as exc:
            self.error_received.emit("Gimbal control send error: %s" % exc)
            self._running = False

    def _send_mount_configure(self):
        self._mavlink.command_long_send(
            GIMBAL_TARGET_SYSTEM,
            GIMBAL_TARGET_COMPONENT,
            gimbal_py.MAV_CMD_DO_MOUNT_CONFIGURE,
            0,
            2,
            0,
            0,
            0,
            0,
            0,
            0,
        )

    def _close(self):
        if self._port is not None:
            try:
                self._port.close()
            except (serial.SerialException, OSError):
                pass
            self._port = None


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.worker = None
        self.yaw_rate_dps = 0.0
        self.pitch_rate_dps = 0.0
        self.setWindowTitle("Gimbal MAVLink Serial Reader")
        self.resize(920, 640)
        self._build_ui()
        self.refresh_ports()

    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)

        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(10)

        config = QtWidgets.QGroupBox("Serial Port")
        config_layout = QtWidgets.QGridLayout(config)
        config_layout.setHorizontalSpacing(8)
        config_layout.setVerticalSpacing(8)

        self.port_box = QtWidgets.QComboBox()
        self.refresh_button = QtWidgets.QPushButton("Refresh")
        self.connect_button = QtWidgets.QPushButton("Connect")
        self.clear_button = QtWidgets.QPushButton("Clear")
        self.autoscroll_box = QtWidgets.QCheckBox("Auto scroll")
        self.autoscroll_box.setChecked(True)

        config_layout.addWidget(QtWidgets.QLabel("Port"), 0, 0)
        config_layout.addWidget(self.port_box, 0, 1)
        config_layout.addWidget(self.refresh_button, 0, 2)
        config_layout.addWidget(self.connect_button, 0, 3)
        config_layout.setColumnStretch(1, 1)

        root.addWidget(config)

        link_status = QtWidgets.QGroupBox("MAVLink Link")
        link_layout = QtWidgets.QGridLayout(link_status)
        link_layout.setHorizontalSpacing(12)
        link_layout.setVerticalSpacing(8)

        self.gcs_id_value = self._make_value_label()
        self.gimbal_id_value = self._make_value_label()
        self.last_message_value = self._make_value_label()
        self.heartbeat_value = self._make_value_label()

        self.gcs_id_value.setText("%d / %d" % (GCS_SYSTEM_ID, GCS_COMPONENT_ID))
        self.gimbal_id_value.setText("-- / --")
        self.last_message_value.setText("--")
        self.heartbeat_value.setText("0")

        link_layout.addWidget(QtWidgets.QLabel("GCS sys/comp"), 0, 0)
        link_layout.addWidget(self.gcs_id_value, 0, 1)
        link_layout.addWidget(QtWidgets.QLabel("Gimbal sys/comp"), 0, 2)
        link_layout.addWidget(self.gimbal_id_value, 0, 3)
        link_layout.addWidget(QtWidgets.QLabel("Last message"), 1, 0)
        link_layout.addWidget(self.last_message_value, 1, 1)
        link_layout.addWidget(QtWidgets.QLabel("Heartbeats sent"), 1, 2)
        link_layout.addWidget(self.heartbeat_value, 1, 3)
        link_layout.setColumnStretch(1, 1)
        link_layout.setColumnStretch(3, 1)

        root.addWidget(link_status)

        values = QtWidgets.QGroupBox("Mount Orientation")
        values_layout = QtWidgets.QGridLayout(values)
        values_layout.setHorizontalSpacing(12)
        values_layout.setVerticalSpacing(8)

        self.boot_time_value = self._make_value_label()
        self.roll_value = self._make_value_label()
        self.pitch_value = self._make_value_label()
        self.yaw_value = self._make_value_label()

        values_layout.addWidget(QtWidgets.QLabel("Boot time ms"), 0, 0)
        values_layout.addWidget(self.boot_time_value, 0, 1)
        values_layout.addWidget(QtWidgets.QLabel("Roll"), 0, 2)
        values_layout.addWidget(self.roll_value, 0, 3)
        values_layout.addWidget(QtWidgets.QLabel("Pitch"), 1, 0)
        values_layout.addWidget(self.pitch_value, 1, 1)
        values_layout.addWidget(QtWidgets.QLabel("Yaw"), 1, 2)
        values_layout.addWidget(self.yaw_value, 1, 3)
        values_layout.setColumnStretch(1, 1)
        values_layout.setColumnStretch(3, 1)

        root.addWidget(values)

        controls = QtWidgets.QGroupBox("Gimbal Control")
        controls_layout = QtWidgets.QGridLayout(controls)
        controls_layout.setHorizontalSpacing(12)
        controls_layout.setVerticalSpacing(8)

        self.yaw_slider = self._make_center_slider()
        self.pitch_slider = self._make_center_slider()
        self.yaw_rate_value = QtWidgets.QLabel("0 deg/s")
        self.pitch_rate_value = QtWidgets.QLabel("0 deg/s")
        self.center_button = QtWidgets.QPushButton("Center")

        controls_layout.addWidget(QtWidgets.QLabel("Yaw"), 0, 0)
        controls_layout.addWidget(self.yaw_slider, 0, 1)
        controls_layout.addWidget(self.yaw_rate_value, 0, 2)
        controls_layout.addWidget(QtWidgets.QLabel("Pitch"), 1, 0)
        controls_layout.addWidget(self.pitch_slider, 1, 1)
        controls_layout.addWidget(self.pitch_rate_value, 1, 2)
        controls_layout.addWidget(self.center_button, 0, 3, 2, 1)
        controls_layout.setColumnStretch(1, 1)

        root.addWidget(controls)

        toolbar = QtWidgets.QHBoxLayout()
        toolbar.addWidget(self.clear_button)
        toolbar.addWidget(self.autoscroll_box)
        toolbar.addStretch(1)
        root.addLayout(toolbar)

        self.log_view = QtWidgets.QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        font = self.log_view.font()
        font.setFamily("Monospace")
        self.log_view.setFont(font)
        root.addWidget(self.log_view, 1)

        self.statusBar().showMessage("Disconnected")

        self.refresh_button.clicked.connect(self.refresh_ports)
        self.connect_button.clicked.connect(self.toggle_connection)
        self.clear_button.clicked.connect(self.log_view.clear)
        self.yaw_slider.valueChanged.connect(self.yaw_slider_changed)
        self.pitch_slider.valueChanged.connect(self.pitch_slider_changed)
        self.yaw_slider.sliderReleased.connect(self.center_yaw_slider)
        self.pitch_slider.sliderReleased.connect(self.center_pitch_slider)
        self.center_button.clicked.connect(self.reset_gimbal_center)

    def _make_center_slider(self):
        slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        slider.setRange(-10, 10)
        slider.setValue(0)
        slider.setTickInterval(1)
        slider.setTickPosition(QtWidgets.QSlider.TicksBelow)
        slider.setPageStep(1)
        return slider

    def _make_value_label(self):
        label = QtWidgets.QLabel("--")
        label.setMinimumWidth(140)
        label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        font = label.font()
        font.setPointSize(font.pointSize() + 2)
        font.setBold(True)
        label.setFont(font)
        return label

    def refresh_ports(self):
        current = self.port_box.currentData()
        self.port_box.clear()
        for port in serial.tools.list_ports.comports():
            label = "%s  %s" % (port.device, port.description)
            self.port_box.addItem(label, port.device)
        if self.port_box.count() == 0:
            self.port_box.addItem("No serial ports found", "")
        elif current:
            index = self.port_box.findData(current)
            if index >= 0:
                self.port_box.setCurrentIndex(index)

    def toggle_connection(self):
        if self.worker is not None:
            self.disconnect_serial()
            return
        self.connect_serial()

    def connect_serial(self):
        port = self.port_box.currentData()
        if not port:
            self.statusBar().showMessage("No serial port selected")
            return

        config = dict(SERIAL_CONFIG)
        config["port"] = port

        self.worker = SerialMavlinkWorker(config, self)
        self.worker.message_received.connect(self.append_log)
        self.worker.mount_orientation_received.connect(self.update_mount_orientation)
        self.worker.mavlink_source_received.connect(self.update_mavlink_source)
        self.worker.heartbeat_sent.connect(self.update_heartbeat_status)
        self.worker.error_received.connect(self.append_error)
        self.worker.status_changed.connect(self.statusBar().showMessage)
        self.worker.stopped.connect(self.serial_stopped)
        self.worker.start()
        self._publish_control_rates()

        self._set_config_enabled(False)
        self.connect_button.setText("Disconnect")

    def disconnect_serial(self):
        self.statusBar().showMessage("Disconnecting...")
        self.connect_button.setEnabled(False)
        if self.worker is not None:
            self.worker.stop()

    def serial_stopped(self):
        if self.worker is not None:
            self.worker.wait(100)
            self.worker = None
        self._set_config_enabled(True)
        self.connect_button.setEnabled(True)
        self.connect_button.setText("Connect")
        self.statusBar().showMessage("Disconnected")

    def _set_config_enabled(self, enabled):
        for widget in (
            self.port_box,
            self.refresh_button,
        ):
            widget.setEnabled(enabled)

    def append_log(self, text):
        self.log_view.appendPlainText(text)
        if self.autoscroll_box.isChecked():
            self.log_view.verticalScrollBar().setValue(self.log_view.verticalScrollBar().maximum())

    def append_error(self, text):
        self.append_log("[ERROR] %s" % text)

    def update_mount_orientation(self, data):
        self.boot_time_value.setText(str(data.get("time_boot_ms", "--")))
        self.roll_value.setText(self._format_degrees(data.get("roll")))
        self.pitch_value.setText(self._format_degrees(data.get("pitch")))
        self.yaw_value.setText(self._format_degrees(data.get("yaw")))

    def update_mavlink_source(self, system_id, component_id, message_type):
        self.gimbal_id_value.setText("%d / %d" % (system_id, component_id))
        self.last_message_value.setText(message_type)

    def update_heartbeat_status(self, count, timestamp):
        self.heartbeat_value.setText("%d  last %s" % (count, timestamp))

    def _format_degrees(self, value):
        if value is None:
            return "--"
        return "%.3f deg" % value

    def yaw_slider_changed(self, value):
        self.yaw_rate_dps = float(value)
        self.yaw_rate_value.setText("%+d deg/s" % value)
        self._publish_control_rates()

    def pitch_slider_changed(self, value):
        self.pitch_rate_dps = float(value)
        self.pitch_rate_value.setText("%+d deg/s" % value)
        self._publish_control_rates()

    def center_yaw_slider(self):
        self.yaw_slider.setValue(0)

    def center_pitch_slider(self):
        self.pitch_slider.setValue(0)

    def _publish_control_rates(self):
        if self.worker is not None:
            self.worker.set_control_rates(self.yaw_rate_dps, self.pitch_rate_dps)

    def reset_gimbal_center(self):
        self.yaw_slider.setValue(0)
        self.pitch_slider.setValue(0)
        self.yaw_rate_dps = 0.0
        self.pitch_rate_dps = 0.0
        self.yaw_rate_value.setText("0 deg/s")
        self.pitch_rate_value.setText("0 deg/s")
        if self.worker is not None:
            self.worker.request_center()
            self.statusBar().showMessage("Center command sent")

    def closeEvent(self, event):
        if self.worker is not None:
            self.worker.stop()
            self.worker.wait(1500)
        event.accept()


def main():
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
