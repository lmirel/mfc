# mfc - motion feedback controller

this tool uses the native in-game telemetry shared via UDP/TCP from games to control a motion platform.<br>
the following games are supported so far:

- Assetto Corsa (PS4)<br>
- Codemaster's F1 (PS4) and Dirt Rally (PC)<br>
- Project Cars 2 (PS4)
<br>
the next thing it does, it uses the USB HID data exchanged between the steering wheel and the gaming console to 'guesstimate' how a motion platform SHOULD move as if it had telemetry available. it does its best and most of the times it manages well. keep in mind however that it guesses that and it relies heavily on the FFB and wheel data to do it.
<br>supported wheels so far:

- Thrustmaster T300RS for PS4
- Logitech G92 for PS4
- Fanatec Elite CSL for PS4

# how this works

the MFC SERVER controls the motion platform drivers.<br>
the MFC CLIENTS are programs that receive native telemetry data and sends it to the server. clients developed so far:<br>
the USB XTRACTOR is used to proxy between the steering wheel and the console to extract USB HID data.<br>
the MFC XTRACTOR uses the USB HID data to generate game telemetry.<br>
the two extractors are only needed if the games you play don't provide in-game telemetry via UDP/TCP. the USBXTRACTOR also requires an Arduino/Teensy atmega32u4 with custom firmware. this solution is based on Matlo's https://github.com/matlo/serialusb<br>
<br>
the end result using the xtractors on a PS4 system looks something like this:<br>
[side view](https://www.youtube.com/watch?v=uBPW2BS_ysU&t=1s) and [back view](https://www.youtube.com/watch?v=jvZpMXiD8k4&t=1s)

# WiKi

- <a href="https://github.com/lmirel/mfc/wiki">Wiki</a>

<br>Tested on a system using Raspberry PI2/3.
<br><br>
USE AT YOUR OWN RISK and Enjoy!
