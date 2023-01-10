import pybullet as p
import time
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy
import math
import random
import constants as c

class MOTOR:
    
    def __init__(self, jointName):
        self.jointName = jointName
        self.Prepare_To_Act()

    def Prepare_To_Act(self):
        self.amplitude = c.amplitude
        self.frequency = c.frequency
        self.offset = c.phaseOffset

        self.oldTargetAngles = numpy.linspace(0, 2*numpy.pi, 1000)
        self.motorValues = numpy.zeros(1000)

        for i in range(0, 1000):
            self.motorValues[i] = numpy.sin(self.oldTargetAngles[i]) * numpy.pi/4
            self.motorValues[i] = self.amplitude * numpy.sin(self.frequency * (self.oldTargetAngles[i] + self.offset))
        

    def Set_Value(self, robotId, t):
        return pyrosim.Set_Motor_For_Joint(bodyIndex=robotId, jointName=self.jointName, controlMode=p.POSITION_CONTROL, targetPosition=self.motorValues[t], maxForce=500)
        # pyrosim.Set_Motor_For_Joint(bodyIndex=robotId, jointName=b'Torso_FrontLeg', controlMode=p.POSITION_CONTROL, targetPosition=flTargetAngles[i], maxForce=500)



# # back leg
# blOldTargetAngles = numpy.linspace(0, 2*numpy.pi, 1000)
# blTargetAngles = numpy.zeros(1000)

# c.blAmplitude = numpy.pi/4
# c.blFrequency = 10
# c.blPhaseOffset = 0

# for i in range(0, 1000):
#     blTargetAngles[i] = numpy.sin(blOldTargetAngles[i]) * numpy.pi/4
#     blTargetAngles[i] = c.blAmplitude * numpy.sin(c.blFrequency * (blOldTargetAngles[i] + c.blPhaseOffset))
#     print(blTargetAngles[i]) 