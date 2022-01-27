# Rigid Wing Trim Tab Controller Firmware

This repository contains the firmware that runs on the microcontroller used to control the trim tab. The trim tab is an essential part of the rigid wing assembly and is used to adjust the main wing's angle of attack relative to the wind.

The trim tab controller communicates with the main control system located in the hull over a Bluetooth Low Energy (BLE) connection. BLE was chosen because it is easy to use, reliable, and power efficient.


## Dependencies
- ArduinoBLE
- arduino-timer
- Battery Sense

Before you can compile the firmware, you'll need to install the libraries listed above. You can do this from within the Arduino IDE, by going to `Tools` > `Manage Libraries...`.


## Connecting to BLE
Once the firmware has been uploaded and the microcontroller has power it will begin advertising over Bluetooth. To connect to the trim tab controller from inside the boat, use the ROS node provided in [wpisailbot/sailbot21-22](https://github.com/wpisailbot/sailbot21-22).

### Getting the BLE Address
If you need the BLE address for the trim tab controller, open up a serial monitor, then restart the microcontroller. Once the microcontroller restarts it will print out the address. You should only need to do this once.


## Additional Resources
- [Bluetooth Specifications - Assigned Numbers](https://www.bluetooth.com/specifications/assigned-numbers/)
