# mfc
motion feedback controller

this tool uses the native in-game telemetry shared via UDP/TCP from games to control a motion platform.
the next thing it does, it uses the USB HID data exchanged between the steering wheel and the gaming console to 'guesstimate' how a motion platform SHOULD move as if it had telemetry available. it does its best and most of the times it manages well. keep in mind however that it guesses that and it relies heavily on the FFB and wheel data to do it.
<br>
<br>supported wheels so far:
- Thrustmaster T300RS for PS4
- Logitech G92 for PS4
- Fanatec Elite CSL for PS4
<br>
<br>how this works:<br>
the SERVER it uses a server to control the motion platform drivers (so far works with SCN6 but Arduino controller support is in the works).<br>
the CLIENTS are native programs that receive native telemetry data and sends it to the servers.<br>
the USBXTRACTOR is used to proxy between the steering wheel and the console to extract USB HID data.<br>
the XTRACTOR uses the USB HID data to generate game telemetry.<br>
the two extractors are only needed if the games you play don't provide in-game telemetry via UDP/TCP. they also require an Arduino/Teensy atmega32u4 with custom firmware. this solution is based on Matlo's and is documented on http://gimx.fr<br>
<br>
the end result using the xtractors on a PS4 system looks something like this:<br>
https://www.youtube.com/watch?v=uBPW2BS_ysU&t=1s and https://www.youtube.com/watch?v=jvZpMXiD8k4&t=1s
<br>
<br>TODO
<br>- Wiki
<br>- add motion support for Arduino-based controllers
<br>
<br>Tested on a system using Raspberry PI2/3.
<br>
USE AT YOUR OWN RISK and Enjoy!
