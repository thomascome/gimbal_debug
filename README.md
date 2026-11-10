# Gimbal MAVLink Serial Reader

Small PyQt5 UI for reading MAVLink messages from a serial port.

The parser is imported from `../Gimbal_update_tool/gimbal_py.py`, so it uses the same generated Goertek/Aerora MAVLink dialect as the update tool.

Select the serial port and baud rate in the UI. The baud-rate list includes common values and also accepts a custom positive integer; its default is `500000`.

Other serial settings are fixed in code:

- Format: `8N1`
- Flow control: disabled
- Timeout: `0.10s`

## Install

```bash
pip3 install PyQt5 pyserial
```

## Run

```bash
python3 serial_mavlink_ui.py
```

## Windows package

The GitHub Actions **Build Windows package** workflow runs manually, on pushes to `main`, or when a tag beginning with `v` is pushed. It creates `gimbal-debug.exe` and uploads it as the `gimbal-debug-windows` workflow artifact.

Select the serial port and baud rate, then press `Connect`. Decoded MAVLink messages are shown as JSON. `GIMBAL_DEBUGDATA.debug_data` is converted to ASCII text like the update tool monitor code.

After connecting, the tool also sends one MAVLink `HEARTBEAT` per second as a ground control station:

- Type: `MAV_TYPE_GCS`
- Autopilot: `MAV_AUTOPILOT_INVALID`
- System id: `255`
- Component id: `190` (`MAV_COMP_ID_MISSIONPLANNER`)

The `MAVLink Link` panel shows:

- The GCS source system/component id used for outgoing messages
- The last incoming gimbal system/component id
- The last incoming MAVLink message type
- The heartbeat transmit count and last transmit time

The yaw and pitch sliders send `COMMAND_LONG / MAV_CMD_DO_MOUNT_CONTROL` commands, matching the firmware path in `../Gimbal_APP/CODE/communication/ComTask.c` and `../Gimbal_APP/CODE/Instruction/FCDataConvert.c`:

- Slider range: `-100%` to `+100%`
- Release behavior: slider returns to center and sends an explicit zero-speed command once both axes are centered
- The sliders control speed proportion only; no angle or degrees-per-second value is displayed
- Send rate: `10 Hz` while either slider is off center
- Target system/component: `1/154`
- Before the first control command, the tool sends `MAV_CMD_DO_MOUNT_CONFIGURE` with pitch/yaw input mode `1` so both axes use angular-rate control
