from EEZYbotARM_controller.controller_movement import EEzybotArmMk2

print("")
print("program starting")
print("")

myEEzybot = EEzybotArmMk2(0, 70, -100)

myEEzybot.q3CalcLimits(q2=-12)

myEEzybot.checkErrorJointLimits()

print("Servo angles = ", myEEzybot.map_kinematicsToServoAngles())

myEEzybot.updateJointAngles(30, 70, -100)

myfig, myax = myEEzybot.plot_EEZYbotARM()

one, two, three = myEEzybot.FK_EEzybotArm(q1=30, q2=70, q3=-100)

print("FK (mm) -->", one, two, three)

myEEzybot.IK_EEzybotArm(one, two, three)

# ---------------------

print("")
print("program ending")
print("")
