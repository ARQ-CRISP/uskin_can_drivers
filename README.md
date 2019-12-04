# uskin_can_drivers
Drivers for communicating with uSkin sensor using the SocketCAN protocol

The libraries work "as is" and can be simply copied inside any project and start being used. The following libraries can be found:
- can_communication.h: Implements the 'low-level' methods that include the CAN communication protocol between machine and sensor.
- uskinCanDriver: Implements the 'high-level' methods to operate with the sensor (CAN protocol is hidden to the user), *e.g.* start and stop sensor, retrieve data, calibrate sensor, *etc*.

**Make sure SocketCan is installed in your machine** 
https://github.com/gribot-robotics/documentation/wiki/Installing-SocketCAN

You might be able to check if SocketCan is already installed by running  
`ls /sys/class/net/`.  
When there is an entry "can0" everything should be fine (be sure that the CAN interface is connected to the computer).

**In your machine, start by setting up the 'can0' network necessary to communicate with the CAN interface**

`sudo ip link set can0 up type can bitrate 1000000`

**Include the contents of "include" and "src" directory into your code**

And start working the 'UskinCanDriver' object class

**Additional reading - Websites of interest:**

To connect the sensor to the computer and extract data you might need a USB converter interface. For this we use the ESD CAN-USB 2.0 Interface:
- https://esd.eu/en/products/can-usb2.

To use this interface with Linux it is necessary to install the NTCAN plugin for CAN-USB protocol conversion:
- https://esd.eu/en/software-downloads/27071 (file ntcansckplugin64-2.3.1-ntcan-4.0.1.tgz for the 64bit version)
Follow the README file instructions for installation.


Linux Networking Communication - SocketCan
- https://www.kernel.org/doc/html/latest/networking/can.html
