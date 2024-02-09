# Volvo Power Steering Control

ESP32 based using the TWAI CANbus controller.

## CAN

The volvo pump communicates at 500kbps and uses extended CAN IDs.

| CAN ID | DLC | Period | Description |
| ------ | ------------------ | ------ | ----------- |
| 0x1ae0092c | 8 | 2000 ms | "keep alive" message, possibly from ECU/ignition/security/BCM? <br><br>Byte 1 appears to be a counter, and is the only byte that seems to matter. Byte 1 changes each time the keep alive message is sent. Byte 1 only seems to rotate through [0x00, 0x40, 0x80, 0xC0] indefinitely<br><br>Sample Frame:<br>00 00 22 E0 41 90 00 00|
| 0x02104136 | 8 | 72 ms | Bytes 7 and 8 of this frame are likely vehicle speed and varies from 1 - 6000 decimal or 00 01 - 17 70 hex. A hex value of 00 01 turns the pump on full speed<br><br>Sample Frame (full speed):<br>BB 00 3F FF 06 E0 00 01|

All credits to:

https://github.com/jssteinke/Volvo-EHPS-Pump-Control

https://gist.github.com/mck1117

https://github.com/rusefi/rusefi/blob/master/firmware/controllers/lua/examples/Volvo-electric-power-steering-pump.txt

https://github.com/NMSTEC/Volvo_EPS_FREE
