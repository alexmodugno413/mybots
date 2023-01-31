import pybullet as p
import time
import pybullet_data
import pyrosim.pyrosim as pyrosim
from pyrosim.neuralNetwork import NEURAL_NETWORK
import numpy
import math
import random
import constants as c
from sensor import SENSOR
from motor import MOTOR
import codecs
import os

class ROBOT:
    
    def __init__(self, solutionID):
        self.solutionID = solutionID
        self.robotId = p.loadURDF("body.urdf") # indicates which robot you want prepared for simulation
        self.nn = NEURAL_NETWORK(f"brain{self.solutionID}.nndf")
        os.system(f"rm brain{str(self.solutionID)}.nndf")
        # os.system(f"rm brain{solutionID}.nndf")

    def Prepare_To_Sense(self):
        self.sensors = {}

        for linkName in pyrosim.linkNamesToIndices:
            self.sensors[linkName] = SENSOR(linkName)

    def Sense(self, t, maxStep):
        switch = True
        for key, value in self.sensors.items():
            # if switch == True:
            #     # overwrite value of one touch sensor with sin(xt)
            #     # low x value means slow-stepping gait, high x value means high-stepping gait
            #     self.sensors[key].values[t] = math.sin(4.0*t)
            #     switch = False
            #     continue
            self.sensors[key].values[t] = self.sensors[key].Get_Value()
            # print(f't: {t}')

    def Prepare_To_Act(self, maxStep):
        self.motors = {}

        for jointName in pyrosim.jointNamesToIndices:
            self.motors[jointName] = MOTOR(jointName, maxStep)

    def Act(self, t):
        for neuronName in self.nn.Get_Neuron_Names():
            if self.nn.Is_Motor_Neuron(neuronName):
                jointName = self.nn.Get_Motor_Neurons_Joint(neuronName)
                jointName = bytes(jointName, 'utf-8')
                desiredAngle = self.nn.Get_Value_Of(neuronName)
                self.motors[jointName].Set_Value(self.robotId, desiredAngle*c.motorJointRange)
                # print(f'neuronName = {neuronName}, jointName = {jointName}, desiredAngle = {desiredAngle}')

    def Think(self):
        self.nn.Update()
        # self.nn.Print()

    def Get_Fitness(self):
        # stateOfLinkZero = p.getLinkState(self.robotId, 0)
        # positionOfLinkZero = stateOfLinkZero[0]
        # xCoordinateOfLinkZero = positionOfLinkZero[0]

        basePositionAndOrientation = p.getBasePositionAndOrientation(self.robotId)
        basePosition = basePositionAndOrientation[0]
        zPosition = basePosition[2]

        # position = basePositionAndOrientation[0]
        # height = position[2]

        fitnessFile = open(f"fitness{str(self.solutionID)}.txt", "w")
        # fitnessFile.write(str(xCoordinateOfLinkZero))
        fitnessFile.write(str(zPosition))
        fitnessFile.close()
        

