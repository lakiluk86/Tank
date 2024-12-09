# Soviet Tank KV-1
This project realizes a RC tank, controlled by a PS4-Controller. As the controller in the tank, a ESP32-NodeMCU board is used.
## 3D Print
All the parts for 3D printing can be obtained on Thingiverse, https://www.thingiverse.com/thing:5069466
## Bluetooth
For working bluetooth one has to set the master MAC address of the PS4-Controller to the one specified in the code. This can be done by the tool "SixAxisPairTool" for example.
Furthermore all BT connections of the ESP have to be resettet before pairing is possible to the ESP. This is done automatically in the code.
## Electronics
- ESP32-NodeMCU
- l298n motor driver board
- 5-12V battery
