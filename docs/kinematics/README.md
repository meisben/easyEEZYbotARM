# Kinematics

## Introduction

Kinematics is the mapping of joint angles to the end effector position. It allows us to calculate how to control the robot arm.

## Generalisation

The method of these kinematic calculations should generalise to other similar geometry robot arms. For example the [Dobot Magician](https://www.dobot-robots.com/products/education/magician.html)

## Forward Kinematics

The [forward kinematic calculations](Forward_kinematics_EEZYbotARM.pdf) are implemented in the kinematic_model.py python module, in the forwardKinematics function.

![forwardKinematics](../../images/forwardKinematics.png)

## Inverse Kinematics

The [inverse kinematic calculations](Inverse_kinematics_EEZYbotARM.pdf) are implemented in the kinematic_model.py python module, in the inverseKinematics function.

![inverseKinematics](../../images/inverseKinematics.png)




