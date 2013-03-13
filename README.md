ROSPoMoCo
=========

Port of PoMoCo to a ROS Node.

Current Features
-------------------
* Ability to script specify moves based on ROS processes by publishing to \moves
* A voice operation demo using CMU's PocketSphinx.

Notable Known Bugs
-------------------------
* Cannot relax servos from within ROS (may cause servo burnout if left running).
* Cannot calibrate servos.

The TODO includes
-----------------------
* Providing Access in ROS to all features under normal PoMoCo
* Integration or creation of gait generation for arbitry paths
* An optional simple GUI for monitoring and controlling hexy manually.
* Complete ROS .launch files for various modes.

Custom ROS messages
---------------------------
* leg_pose defines the pose of a single leg. Primative uint8s are used for angles, and should be converted into the 0 to 180 range before use.
* pose defines the pose of the entire robot, and is composed of many leg_pose objects.
