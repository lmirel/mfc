# mfc
motion feedback controller

this tool uses the native in-game telemetry shared via UDP/TCP from games to control a motion platform.
the next thing it does, it uses the USB HID data exchanged between the steering wheel and the gaming console to 'guesstimate' how a motion platform SHOULD move as if it had telemetry available. it does its best and most of the times it manages well. keep in mind however that it guesses that and it relies heavily on the FFB and wheel data to do it.

supported wheels so far:
- Thrustmaster T300RS for PS4
- Logitech G92 for PS4
- Fanatec Elite CSL for PS4

how this works:
the SERVER it uses a server to control the motion platform drivers (so far works with SCN6 but Arduino controller support is in the works).
the CLIENTS are native programs that receive native telemetry data and sends it to the servers.
the USBXTRACTOR is used to proxy between the steering wheel and the console to extract USB HID data.
the XTRACTOR uses the USB HID data to generate game telemetry.
the two extractors are only needed if the games you play don't provide in-game telemetry via UDP/TCP.

the end result using the xtractors on a PS4 system looks something like this:
https://www.youtube.com/watch?v=uBPW2BS_ysU&t=1s and https://www.youtube.com/watch?v=jvZpMXiD8k4&t=1s

TODO
- Wiki
- add motion support for Arduino-based controllers

Tested on a system using Raspberry PI2/3.

USE AT YOUR OWN RISK and Enjoy!
