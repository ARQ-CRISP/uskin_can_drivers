# uskin_can_drivers
Drivers for communicating with uSkin sensor using socketCAN

- **In your machine, start by setting up the 'can0' network necessary to communicate with ESD CAN/USB converter**

sudo ip link set can0 up type can bitrate 1000000

- **Include the contents of "include" directory into your code**

And start working the 'UskinCanDriver' object class
