# Kinematics

## Introduction

Kinematics is the mapping of joint angles to the end effector position. It allows us to calculate how to control the robot arm.

## Generalisation

The method of these kinematic calculations should generalise to other similar geometry (4 axis) robot arms. 

For example:

* [Dobot Magician](https://www.dobot-robots.com/products/education/magician.html)
* [Dobot MG400](https://www.dobot-robots.com/products/education/magician.html)
* [MyPalletizer 260 M5Stack](https://shop.elephantrobotics.com/en-gb/products/mypalletizer)

## Forward kinematic calculations

The [forward kinematic calculations](Forward_kinematics_EEZYbotARM.pdf) are implemented in the kinematic_model.py python module, in the forwardKinematics function.

![forwardKinematics](../../images/forwardKinematics.png)

## Inverse kinematic calculations

The [inverse kinematic calculations](Inverse_kinematics_EEZYbotARM.pdf) are implemented in the kinematic_model.py python module, in the inverseKinematics function.

![inverseKinematics](../../images/inverseKinematics.png)




