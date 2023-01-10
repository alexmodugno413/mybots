import pybullet as p
import time
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy
import math
import random
import constants as c

class SENSOR:
    
    def __init__(self, linkName):
        self.linkName = linkName
        self.values = numpy.zeros(c.maxStep) # change number

    def Get_Value(self):
        return pyrosim.Get_Touch_Sensor_Value_For_Link(self.linkName)

    def Save_Values(self):
        numpy.save(f'data/{self.linkName}SensorValues.npy', self.values)