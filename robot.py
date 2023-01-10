import pybullet as p
import time
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy
import math
import random
import constants as c

class ROBOT:
    
    def __init__(self):
        self.sensors = {}
        self.motors = {}

        self.robotId = p.loadURDF("body.urdf") # indicates which robot you want prepared for simulation