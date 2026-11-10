import os
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt5 import QtWidgets

from serial_mavlink_ui import MainWindow, SerialMavlinkWorker


class FakeMavlink:
    def __init__(self):
        self.commands = []

    def command_long_send(self, *args):
        self.commands.append(args)


class SerialMavlinkWorkerControlTests(unittest.TestCase):
    def setUp(self):
        self.worker = SerialMavlinkWorker({"port": "test"})
        self.worker._mavlink = FakeMavlink()

    def test_mount_configuration_uses_angular_rate_for_pitch_and_yaw(self):
        self.worker._send_mount_configure()

        command = self.worker._mavlink.commands[-1]
        self.assertEqual(command[-2:], (1, 1))

    def test_speed_command_uses_current_control_values_without_angle_integration(self):
        self.worker.set_control_percentages(25.0, -40.0)
        self.worker._send_attitude_control(0.1)

        command = self.worker._mavlink.commands[-1]
        self.assertEqual(command[4], -40.0)
        self.assertEqual(command[6], 25.0)

    def test_stopping_after_motion_sends_one_zero_speed_command(self):
        self.worker.set_control_percentages(25.0, 0.0)
        self.worker._send_attitude_control(0.1)
        self.worker.set_control_percentages(0.0, 0.0)
        self.worker._send_attitude_control(0.1)

        self.assertEqual(len(self.worker._mavlink.commands), 3)
        stop_command = self.worker._mavlink.commands[-1]
        self.assertEqual(stop_command[4], 0.0)
        self.assertEqual(stop_command[6], 0.0)

        self.worker._send_attitude_control(0.1)
        self.assertEqual(len(self.worker._mavlink.commands), 3)

    def test_pending_stop_is_sent_before_a_new_speed_command(self):
        self.worker.set_control_percentages(25.0, 0.0)
        self.worker._send_attitude_control(0.1)
        self.worker.set_control_percentages(0.0, 0.0)
        self.worker.set_control_percentages(30.0, 0.0)
        self.worker._send_attitude_control(0.1)

        stop_command = self.worker._mavlink.commands[-1]
        self.assertEqual(stop_command[4], 0.0)
        self.assertEqual(stop_command[6], 0.0)

        self.worker._send_attitude_control(0.1)
        resumed_command = self.worker._mavlink.commands[-1]
        self.assertEqual(resumed_command[6], 30.0)

    def test_control_panel_has_no_center_button(self):
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        window = MainWindow()

        self.assertNotIn("Center", [button.text() for button in window.findChildren(QtWidgets.QPushButton)])
        self.assertEqual((window.yaw_slider.minimum(), window.yaw_slider.maximum()), (-100, 100))
        self.assertEqual((window.pitch_slider.minimum(), window.pitch_slider.maximum()), (-100, 100))
        self.assertEqual(window.yaw_control_value.text(), "0%")
        self.assertEqual(window.pitch_control_value.text(), "0%")


if __name__ == "__main__":
    unittest.main()
