import pybullet as p
import time
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy
import math
import random
import constants as c

class MOTOR:
    
    def __init__(self, jointName, maxStep):
        self.jointName = jointName
        self.maxStep = maxStep      

    def Set_Value(self, robotId, desiredAngle):
        return pyrosim.Set_Motor_For_Joint(bodyIndex=robotId, jointName=self.jointName, controlMode=p.POSITION_CONTROL, targetPosition=desiredAngle, maxForce=500)

