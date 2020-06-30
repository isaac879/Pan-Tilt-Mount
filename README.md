# Pan-Tilt-Mount
A stepper motor driven, 3D printed and Arduino Nano controlled pan tilt mount.

Project Overview:
This is the pan tilt camera mount I designed for my DSLR Canon EOS 250D. I designed it for video motion control, time-lapses and panoramic shots.

Here is a quick overview of the components and how the project works. There is an Arduino Nano as the brain controlling everything and monitoring the inputs. The two Nima 17 stepper motors are controlled by A4988 stepper driver boards. When combining the stepper motor’s step angle, the microstepping mode of the driver boards and the gear ratios of the pan and tilt axis you get a precision of 0.0133° and 0.0369° respectively (Stepper motors move 1.8° per full step, microstepping divides this by 16, the pan axis has a gear ratio of 144:17 and the tilt has a ratio of 64:21). 

There are Hall effect sensors embedded in the pan and tilt axis to allow the stepper motors to home and zero themselves when powered on. The circuit can be powered by a 12V DC input or 3 cell LiPo battery for portability. The Arduino monitors the battery level and indicates its charge on the RGB status LED. The Arduino can also trigger the camera’s shutter using an NPN transistor and 2.5mm 3 pole jack. 

Communicating with the Arduino is simply over a serial connection provided by the USB or JDY-31 serial pass-through Bluetooth module.

As the continued development of this project I have now designed a slider for the pan tilt mount.

The slider is made from mostly 3d printed parts, M3 nuts and bolts and 2020 V-slot aluminium extrusion. It uses the same PCB I created for my pan tilt mount which is why I included an extra stepper driver and breakout for another Hall effect sensor.

The 2GT timing belt, 20 tooth timing pulley and 16th microstepping allow the slider to move with a precision of 0.0125mm. The movement speed can be set extremely slow for time-lapses or up to about 200mm/s (in full step mode).

The pan tilt mount project video/demo: https://youtu.be/uJO7mv4-0PY
The slider project video/demo: https://youtu.be/v1b7Wvu87-U

Some picture, videos and development updates are available on my Instagram: https://www.instagram.com/isaac879/?hl=en

The CAD STL and STEP files for the pan tilt mount are available here: https://www.thingiverse.com/thing:4316563
The CAD STL and STEP files for the slider are available here: https://www.thingiverse.com/thing:4512714
