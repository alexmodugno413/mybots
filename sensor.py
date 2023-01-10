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
        self.values = numpy.zeros(1000) # change number

    def Get_Value(self, t, maxStep):
        if t == maxStep-1:
            print(self.values)
        return pyrosim.Get_Touch_Sensor_Value_For_Link(self.linkName)