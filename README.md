# 3-Axis Slider
A stepper motor driven, 3D printed and Arduino Nano controlled 3-axis slider.

Project Overview:
This is the 3-axis slider I designed for my DSLR Canon EOS 250D (EOS Rebel SL3). I designed it for video motion control, time-lapses and panoramic shots.

Here is a quick overview of the components and how the project works. There is an Arduino Nano as the brain controlling everything and monitoring the inputs. The three Nima 17 stepper motors are controlled by TMC2208 stepper driver boards. When combining the stepper motor’s step angle, the microstepping mode of the driver boards and the gear ratios of the pan and tilt axis you get a precision of 0.0133° and 0.0369° respectively (Stepper motors move 1.8° per full step, microstepping divides this by 16, the pan axis has a gear ratio of 144:17 and the tilt has a ratio of 64:21). 
The slider carriage moves using a 36 tooth timing pulley with a 2GT timing belt on 2040 V-slot aluminium extrusion. This gives the slider has a positional precision of approximately 0.0225mm.

In 16th microstepping mode, axis speeds are limited to about 20 degrees per second for the pan, 15 degrees per second for the tilt and 20mm per second for the slider. This limit is due to the software/microcontroller speed which is only able to produce ~4000 step pulses per second for the 3 stepper motors. In half stepping mode speeds about eight times higher should be possible depending on acceleration profiles and camera weight.

The pan, tilt and slider are almost completely silent in 16th microstepping mode. In half stepping mode when moving at full speed the noise is audible.

The maximum weight I have tested it with is just over 1kg. This was with my Canon EOS 250D with the heaviest lens I have (Canon EF-S 18-135mm f/3.5-5.6) and a Rode Video Mic Pro+. This worked without any issues although some shaking does occur when rapidly changing directions.

There are Hall effect sensors and magnets embedded in the pan, tilt and slider axis to allow the stepper motors to home and zero themselves. The circuit can be powered by a 12V DC input or 3 cell LiPo battery for portability. With a 1000mAh battery the 3-axis slider can operate for abount one hour. The Arduino will monitors the battery level and print it out. The Arduino can also trigger the camera’s shutter using an NPN transistor and 2.5mm 3 pole jack. 

Communicating with the Arduino is simply over a serial connection provided by the USB or JDY-31 serial pass-through Bluetooth module.

The 3-Axis slider project video/demo: https://youtu.be/1FfB7cLkUyQ

Some picture, videos and development updates are available on my Instagram: https://www.instagram.com/isaac879/

The CAD STL and STEP files are available here: https://www.thingiverse.com/thing:4547074 (with some files unchanged since earlier releases: https://www.thingiverse.com/thing:4512714 and https://www.thingiverse.com/thing:4316563)
