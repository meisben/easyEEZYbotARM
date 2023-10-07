# Kinematics

## Introduction

Kinematics is the mapping of joint angles to the end effector position. It allows us to calculate how to control the robot arm.

## Generalisation

The method of these kinematic calculations should generalise to other similar geometry robot arms. For example the [Dobot Magician](https://www.dobot-robots.com/products/education/magician.html)

## Forward Kinematics

![forwardKinematics](../../images/forwardKinematics.png)

The [forward kinematics](Forward_kinematics_EEZYbotARM.pdf) calculations are implemented in the kinematic_model.py python module, in the forwardKinematics function.

## Inverse Kinematics

![inverseKinematics](../../images/inverseKinematics.png)

The [inverse kinematics](Inverse_kinematics_EEZYbotARM.pdf) calculations are implemented in the kinematic_model.py python module, in the inverseKinematics function.


