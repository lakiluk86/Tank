# Soviet Tank KV-1
This project realizes a RC tank, controlled by a PS4-Controller. As the controller in the tank, a ESP32-NodeMCU board is used. A 3D-print project from Thingiverse was modified to enable the tank to shoot.
## 3D Print
- All the parts for 3D printing can be obtained on Thingiverse, https://www.thingiverse.com/thing:5069466
- If one wants to enable the tank to shoot the files for printing the adapter for the shooting unit is included in the doc directory
  - see pictures in doc directory for where to put the prints, should be self explanatory
  - Closure.3mf is for closing the hole you drill in the hull on top of the tank for inserting the BB bullets
  - for attaching the adapter 2x M2x5 screws are needed
## Bluetooth
- For BT functionality the lib https://github.com/aed3/PS4-esp32 is used. But watch out, it is not working with newer versions of the arduino framework. So there is a local copy in the project which was modified according to https://www.youtube.com/watch?v=dRysvxQfVDw
- For working bluetooth one has to set the master MAC address of the PS4-Controller to the one specified in the code. This can be done by the tool "SixAxisPairTool" for example.
- Furthermore all BT connections of the ESP have to be resettet before pairing is possible to the ESP. This is done automatically in the code.
## Electronics
- ESP32-NodeMCU
- 2x BTS7960 Dual H-Bridge Motor Driver
- 2x LM2596S Step down converter
- LiPo 11.2V battery
- Relais for driving the shooting unit
- The servo for rotating the turrent was modified so that it can turn endless (see https://youtu.be/JhHSXCLsN4k?si=Jbtp7p6A8w-mFTv1 for example)
- Shooting unit for HengLong tanks V6 with barrel recoil, see also picture in doc directory
  - the shaft at the rear end of the unit has to be removed, so that the unit fits in the turrent when barrel is pointing downwards
