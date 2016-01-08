# finding-big-zebro
Motion tracking ROS node for Raspberry Pi + camera.

This project was tested on the raspbian distribution (Jessie) on the Raspberry Pi 2.
Make sure the camera is enabled with

raspi-config


In order for this project to work, you'll need OpenCV and the raspicam library:

   • Download the latest version of raspicam:
     http://sourceforge.net/projects/raspicam/

   • unpack the .zip file

   • in the extracted raspicam directory:

     	- mkdir build

	- cd build

	- cmake ..

	- sudo make install

   • for jessie, install the package libgl1-mesa-dri, otherwise you won't be able to open a window for openCV (GdkGLExt-WARNING **: Window system doesn't support OpenGL.)

   • To disable the camera LED, add the line
     disable_camera_led=1
     to the file /boot/config.txt (reboot neccessary)
     

   
   