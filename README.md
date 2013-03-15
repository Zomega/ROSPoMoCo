ROSPoMoCo
==========

Port of PoMoCo to a ROS Node. The original code is a stand alone application and can be found at http://github.com/ArcBotics/PoMoCo .

Install and Set Up
---------------------
_DO NOT_ just clone and `make` this repository as normal. It is not a stand alone application, but a set of scripts and programs for use in a larger system.

I developed this under Ubuntu 12.10 and ROS groovy. Other operating systems may work, but you're on your own.

* Go to http://ros.org and download ROS.
* (Optional) Familiarize yourself with the basics of ROS.
* Clone the repository (or your fork of it) into a directory on your `ROS_PACKAGE_PATH` (e.g. your workspace).
* (Optional) If you plan on using the voice demo, install PocketSphinx.
* From a terminal, run `rospack profile`.
* `roscd ROSPoMoCo` to ensure things are profiled correctly. If they are, you should cd into the cloned repository. Otherwise, check your `ROS_PACKAGE_PATH`.
* Build all the code by running `make`.
* To start the voice demo run (from anywhere) `roslaunch ROSPoMoCo voice-control.launch`. It will take some time to start (and output a lot to the console) even if all goes well. You can get a better feel for what's going on using `rxgraph` and `rxconsole`.

Current Features
-------------------
* Ability to initiate moves based from ROS processes by publishing to \moves
* A new Kill All Servos move ( does what it says on the tin ).
* Less python threading through use of ROS.
* A voice operation demo using CMU's PocketSphinx (working .launch).
* More Object Oriented approach to servos and servo control (Allows for virtual servos, using seamlessly with simulators, etc.).

Notable Known Bugs / Missing Features / Issues
--------------------------------------------------------
* Servotor32 object raises an exception (ROSWarn) when a motor detach is attempted.
* Cannot calibrate servos / calibrations not loaded.
* Neck 

TODO includes
------------------
* Providing Access in ROS to all features under normal PoMoCo (and more!)
* Clean code base.
* Integration or creation of gait generation for arbitrary paths (move_base compatible ).
* Optional simple GUIs for monitoring and controlling Hexy manually.
* Complete ROS .launch files for various modes.
* (Possible) C++ port of essentials for speed.

Custom ROS messages
---------------------------
* `leg_pose` defines the pose of a single leg. Primitive `uint8`s are used for angles, and should be converted into the 0 to 180 range before use.
* `pose` defines the pose of the entire robot, and is composed of many `leg_pose` objects. The neck is currently not included in this description.

License(s) and Warranty
-----------------------------

Informally, it is my intent to ensure you are completely free to use this software as you see fit for personal projects. I appreciate feedback ( especially in the form of well documented bug reports ), but once you've downloaded, you're on your own legally. If you're having setup problems, try http://answers.ros.org. If you make derivatives or use this in a larger project, I always appreciate (but don't really require) attribution.

If you're going to use this for a commercial project, that's fine too. I ask only that you please consider giving back to the open source community in some way.

I also implicitly assume that if you're pull requesting me, then you accept that your changes will be subject these agreements. Pull requests that explicitly request otherwise won't be integrated unless you have a really good reason.

Copyright 2013 Will Oursler

Licensed under your choice of the Apache License, Version 2.0; the MIT License; or LGPL3 (the "Licenses").

You may not use this work except in compliance with one of these Licenses. You may obtain a copy of the relevant License at:

*http://opensource.org/licenses/Apache-2.0

*http://opensource.org/licenses/MIT

*http://opensource.org/licenses/lgpl-3.0.html

Unless required by applicable law or agreed to in writing, software distributed under any and all of the Licenses is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
