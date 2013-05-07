hidmedia
========

A USB HID Media Controller (for Parrot Asteroid)

The one thing that really bugs me about the Parrot Asteroid SMART is the lack of physical
volume control buttons. The roadster doesn't have steering wheel audio controls, so UNIKA
is not a possibility. Parrot make a bluetooth steering wheel remote for the Asteroid
tablet, but it won't work on the SMART (they say it will later this year).

It is Android and that means the Parrot supports USB keyboards using the HID protocol.

So: https://github.com/markwj/hidmedia

Using a PIC18F2550 (US$6 @mouser) usb-capable micro controller, I threw together a
workable hand built prototype. It has a d-pad, two function keys and a mode switch.
At the moment, I've got it programmed as mode 0 directional control and mode 1 media
control. In directional control mode, it behaves like up, down, left, right, enter,
esc, menu, etc - useful for navigating around the Android screens. In media control
mode, the two top buttons are volume decease/increase, and the others will be for
previous song, next song, play/pause, stop, mute, etc. There is a little led that
currently blinks when you press a key.

Still some tuning to do, finding the correct media control codes, but not bad for
a few hours work at the weekend - with my six year old daughter as a little helper
she told me she wants to be me when she grows up 

At the moment, volume control and directional control is perfect, but media pause/play
and song navigation still a work-in-progress (I need to find the correct HID codes).
Last thing would be to 3D print a nice box.

CODE
====

Project is here: https://github.com/markwj/hidmedia/
Microchip C18 project is here: https://github.com/markwj/hidmedia/tree/master/hidmedia.X
Hardware schematic is here: https://github.com/markwj/hidmedia/tree/master/hardware

LINKS
=====

Write-up and discussion on Parrot Asteroid SMART in Tesla Roadster:
  http://www.teslamotorsclub.com/showthread.php/15980-Parrot-Asteroid-SMART-2DIN-Head-Unit/

Some pics of my approach:
  http://www.teslamotorsclub.com/attachment.php?attachmentid=21531&d=1367747866
  http://www.teslamotorsclub.com/attachment.php?attachmentid=21532&d=1367747880

Intro video of Parrot Asteroid SMART in a Tesla Roadster:
  http://www.youtube.com/watch?v=1gYApR6PfyY

An alternative volume-only approach:
  http://www.amazon.com/Fusion-Control-Powered-Controlled-Rotary/dp/B00ANRRZCG

An Arduino approach:
  http://atomic-cactus.com/2013/03/26/parrot-asteroid-smart-steering-wheel-controls-using-arduino-part-1/
  http://atomic-cactus.com/2013/03/29/resistor-ladder-steering-wheel-control-interpreter-using-arduino/
