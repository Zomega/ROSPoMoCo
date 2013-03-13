ROSPoMoCo
=========

Port of PoMoCo to a ROS Node.

Current Features
-------------------
* Ability to script specify moves based on ROS processes by publishing to \moves
* A voice operation demo using CMU's PocketSphinx (working .launch).

Notable Known Bugs
-------------------------
* Cannot relax servos from within ROS (may cause servo burnout if left running).
* Cannot calibrate servos.

TODO includes
-----------------------
* Providing Access in ROS to all features under normal PoMoCo (and more!)
* Integration or creation of gait generation for arbitrary paths (move_base compatible ).
* An optional simple GUI for monitoring and controlling hexy manually.
* Complete ROS .launch files for various modes.

Custom ROS messages
---------------------------
* leg_pose defines the pose of a single leg. Primitive uint8s are used for angles, and should be converted into the 0 to 180 range before use.
* pose defines the pose of the entire robot, and is composed of many leg_pose objects.

License(s) and Warranty
-----------------------------

Informally, it is my intent to ensure you are completely free to use this software as you see fit for personal projects. I appreciate feedback ( especially in the form of well documented bug reports ), but once you've downloaded, you're on your own legally. If you're having problems that can't be solved by answers.ros.org, let me know. If you make derivatives or use this in a larger project, I always appreciate (but don't really require) attribution.

If you're going to use this for a commercial project, that's fine too. I ask only that you please consider giving back to the open source community in some way.

I also implicitly assume that if you're pull requesting me, then you accept that your changes will be subject these agreements. Pull requests that explicitly request otherwise won't be integrated unless you have a really good reason.

Copyright 2013 Will Oursler

Licensed under your choice of the Apache License, Version 2.0 (the "License"); the MIT Liscence; or LGPL3.

You may not use this work except in compliance with one of these Licenses. You may obtain a copy of the relevant License at:

http://opensource.org/licenses/Apache-2.0
http://opensource.org/licenses/MIT
http://opensource.org/licenses/lgpl-3.0.html

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
