fabmouse
========
The fabmouse is a project which would enable your to create your own 3D mouse by hacking a normal mouse.
It currently uses a accelerometer-gyroscope fusion board to get the orientation in 3D space.
To enable normal 2D use of the mouse, the bluetooth module is to be set as a combo device(mouse+keyboard).

The arduino code controls the mouse movement as well use the keyboard as a hack to send the 3D orientation data to the computer. We use the FreeIMU library(http://www.varesano.net/projects/hardware/FreeIMU) for intrpreting the sensor values and getting the orientation.

The accompanying processing library scans and decodes the data back and gives methods to control the 3D orientaion.
There is also a small processing example which illustrates its use and how it could be used with personal processing projects.

Instructions on building your own 3D mouse can be found at -
http://www.instructables.com/id/3D-Mouse/

Further details on the project and accompanying documentation at http://hci.rwth-aachen.de/fabmouse
