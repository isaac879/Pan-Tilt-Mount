# Pan-Tilt-Mount
A stepper motor driven, 3D printed and Arduino Nano controlled pan tilt mount.

Project Overview:
This is the pan tilt camera mount I designed for my DSLR Canon EOS 250D. I designed it for video motion control, time-lapses and panoramic shots.

Here is a quick overview of the components and how the project works. There is an Arduino Nano as the brain controlling everything and monitoring the inputs. The two Nima 17 stepper motors are controlled by A4988 stepper driver boards. When combining the stepper motor’s step angle, the microstepping mode of the driver boards and the gear ratios of the pan and tilt axis you get a precision of 0.0133° and 0.0369° respectively (Stepper motors move 1.8° per full step, microstepping divides this by 16, the pan axis has a gear ratio of 144:17 and the tilt has a ratio of 64:21). 

There are Hall effect sensors embedded in the pan and tilt axis to allow the stepper motors to home and zero themselves when powered on. The circuit can be powered by a 12V DC input or 3 cell LiPo battery for portability. The Arduino monitors the battery level and indicates its charge on the RGB status LED. The Arduino can also trigger the camera’s shutter using an NPN transistor and 2.5mm 3 pole jack. 

Communicating with the Arduino is simply over a serial connection provided by the USB or JDY-31 serial pass-through Bluetooth module.

The project video/demo: https://youtu.be/uJO7mv4-0PY

Some picture, videos and development updates are available here: https://www.instagram.com/p/B-KXCTlj_CY/

The CAD STL files are available here: https://www.thingiverse.com/thing:4316563
