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
        self.Prepare_To_Act()

    def Prepare_To_Act(self):
        self.amplitude = c.amplitude
        self.frequency = c.frequency
        self.offset = c.phaseOffset

        self.oldTargetAngles = numpy.linspace(0, 2*numpy.pi, c.maxStep)
        self.motorValues = numpy.zeros(c.maxStep)

        for i in range(0, c.maxStep):
            self.motorValues[i] = numpy.sin(self.oldTargetAngles[i]) * numpy.pi/4
            self.motorValues[i] = self.amplitude * numpy.sin(self.frequency * (self.oldTargetAngles[i] + self.offset))
        

    def Set_Value(self, robotId, t):
        return pyrosim.Set_Motor_For_Joint(bodyIndex=robotId, jointName=self.jointName, controlMode=p.POSITION_CONTROL, targetPosition=self.motorValues[t], maxForce=500)

    def Save_Values(self):
        numpy.save(f'data/{self.joinName}MotorValues.npy', self.motorValues)

