## :star2: A Simple Open Loop Robotic Motion (SCARA robot) Library

* This Arduino [library](RobotMotion%20Library) will move the SCARA robot's end effector from one point to another in a smooth motion.

## :robot: Brief Description

<div align="center"><img src="Demo/motion.gif" alt="motion" width="350" height="auto" class="center"></div>

<div align="center">(Please wait for the GIF to load)</div>

This library is designed to move all the robotic links gracefully according to the time specified by the user.

Such motion is achieved by controlling the time intervals to move each step of the stepper motors using **trapeziodal velocity profile** ([working principle in Section 4](Documentation/Working%20Principle.pdf)).

## 	:toolbox: Getting Started

### :bangbang: Prerequisites

* Designed for this type of [SCARA robot](https://www.thingiverse.com/thing:1241491) with this [specifications (Section 2)](https://github.com/CK-Explorer/A-Simple-Open-Loop-Robotic-Motion--SCARA-robot--Library/blob/main/Documentation/Working%20Principle.pdf).
* Requires Arduino microcontrollers. (Tested on Arduino Uno and Mega)
* Designed for stepper motors which actuate the robotic links.
* Requires stepper motor drivers like [DRV8825](https://www.pololu.com/product/2133), [A4988](https://www.pololu.com/product/1182), or any similar drivers which utilize square wave signals to drive the motors.

### :running: Using the library

[This page](Brief%20explanation.md)  consists of the steps required to use this library with its details in the [header file](RobotMotion%20Library/RobotMotion.h).

### :test_tube: Examples

* [simple example.ino](Examples/simple%20example.ino) is just a simplified code to showcase the use of this library.

* [demo.ino](Examples/demo.ino) is the project in this [video](https://youtu.be/1WI9T8hou2I), and its schematic diagram is [here](Schematic/schematic.png).

## :gem: Credits

Big thanks to 
* My university coursework's partners in building the hardware.
* [Idegraaf](https://www.thingiverse.com/idegraaf/designs), the creator of this SCARA robot, for open-sourcing it.
* [Sw-tcNet](https://www.youtube.com/watch?v=Z4vRkZ8kcTU) for the gripper design.

## :scroll: Version History

* 1.0.0
    * Initial Release

## :warning: License

This project is licensed under the MIT license - see the [LICENSE.md](LICENSE.md) file for details.
