# easyEEZYbotARM
 A python controller (3 dimensions inverse and forward kinematics) for the EEZYbotARM (MK1,MK2,MK3) movement

## Contents

- Introduction
- Installation (python)
- Installation (arduino)
- Configuration (of the physical arduino setup)
- Guidance on use
- Help and updates

## Introduction

This code library provides function for the EEZYbotARM (Mk1, Mk2) to undertake: 

1. 3-D Forward kinematics
2. 3-D Inverse kinematics
3. Plotting of the (simulated) robot arm
4. Plotting of the (simulated) workspace of the robot arm
5. Communicating with a (arduino) microcontroller
6. Using the (arduino) microcontroller to control the movement of the robot arm servo motors 

The code library allows the user to easily convert x,y,z co-ordinates to joint angles, so that the EEZYbotARM can be controlled in 3-D space.

![inverseKinematics](images/inverseKinematics.png)

## Installation (python)

This code is bundelled as a python package. A setup file is included and so dependent libraries should install with the package.

To install the package on Windows, OS X or Linux, download the zip file or clone the repository. Then open a terminal window, cd to the setup directory and install the package using pip command:

```sh
$pip install .
```

To use with a virtual environment

(1) create a virtual environment (e.g. using conda or venv). 
(2) Then run the following command from a terminal window in the local directory of dobot_tactile_toolbox setup.py: 

```sh
$pip install -e . 
```

A future aim is to distribute this module using pip

## Installation (arduino)

TBC

## Configuration (of the physical arduino setup)

![fritzingDiagram](images/fritzingDiagram.png)

## Guidance on use

- For examples of basic usage see the 'examples' folder

## Bug list

- Currently no bugs are listed here

## Help and updates

If you need help using this code libray, please in the first instance try googling your problem to see if you can find a solution. If that doesn't help then please do feel free to initiate a pull request on github.

## Contibuting

Anyone is welcome to contribute - please let me know

## Thanks

-	Thanks to you for using this code library 
-	A big thanks to Carlo Franciscone for open sourcing the EEZYbotARM
-   A big thanks to ArminJo for creating the fantastic arduino ServoEasing library: https://github.com/ArminJo/ServoEasing
-	Thanks to Antonia Tzemanaki on the University of West England / University of Bristol Robotics course, what an amazing teacher.
-	Thanks to [HOLD] and [HOLD] for putting their courses on inverse and forward kinematics online
