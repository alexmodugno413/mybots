import pybullet as p
import time
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy
import math
import random
import constants as c
from sensor import SENSOR

class ROBOT:
    
    def __init__(self):
        self.motors = {}

        self.robotId = p.loadURDF("body.urdf") # indicates which robot you want prepared for simulation

    def Prepare_To_Sense(self):
        self.sensors = {}

        for linkName in pyrosim.linkNamesToIndices:
            self.sensors[linkName] = SENSOR(linkName)

    def Sense(self, t, maxStep):
        for key, value in self.sensors.items():
            print(key)
            self.sensors[key].values[t] = self.sensors[key].Get_Value(t, maxStep)