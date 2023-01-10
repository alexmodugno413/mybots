import pybullet as p
import time
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy
import math
import random
import constants as c
from sensor import SENSOR
from motor import MOTOR

class ROBOT:
    
    def __init__(self):
        self.robotId = p.loadURDF("body.urdf") # indicates which robot you want prepared for simulation

    def Prepare_To_Sense(self):
        self.sensors = {}

        for linkName in pyrosim.linkNamesToIndices:
            self.sensors[linkName] = SENSOR(linkName)

    def Sense(self, t, maxStep):
        for key, value in self.sensors.items():
            self.sensors[key].values[t] = self.sensors[key].Get_Value()

    def Prepare_To_Act(self, maxStep):
        self.motors = {}

        for jointName in pyrosim.jointNamesToIndices:
            self.motors[jointName] = MOTOR(jointName, maxStep)

    def Act(self, t):
        for key, value in self.motors.items():
            self.motors[key].Set_Value(self.robotId, t)

