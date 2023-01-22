import numpy
import pyrosim.pyrosim as pyrosim
import random
import os
import time
import constants as c
import math

class SOLUTION:
    
    def __init__(self, nextAvailableId):
        self.myID = nextAvailableId
        self.weights = numpy.random.rand(c.numSensorNeurons, c.numMotorNeurons)
        self.weights = self.weights * 2 - 1

    def Start_Simulation(self, directOrGUI):
        self.Create_World()
        self.Create_Body()
        self.Create_Brain()
        os.system(f'python3 simulate.py {directOrGUI} {str(self.myID)} 2&>1 &')

    def Wait_For_Simulation_To_End(self, directOrGUI):
        while not os.path.exists(f"fitness{str(self.myID)}.txt"):
            time.sleep(0.01)
        # print("Second")
        fitnessFile = open(f"fitness{str(self.myID)}.txt", "r")
        fitnessFileRead = fitnessFile.read()
        print(f'fitnessFileRead: {fitnessFileRead}')
        if len(fitnessFileRead) > 0:
            self.fitness = float(fitnessFileRead)
        # print(f'self.fitness{self.myID}: {self.fitness}')
        fitnessFile.close()
        os.system(f"rm fitness{str(self.myID)}.txt")

    def Mutate(self):
        randomRow = random.randint(0, len(self.weights)-1)
        randomColumn = random.randint(0, len(self.weights[0])-1)
        self.weights[randomRow, randomColumn] = random.random() * 2 - 1

    def Set_ID(self, nextAvailableID):
        self.myID = nextAvailableID

    def Create_World(self):
        x = -3
        y = 3
        z = 0.5
        length = 1
        width = 1
        height = 1 

        pyrosim.Start_SDF("world.sdf")
        pyrosim.Send_Cube(name="Box", pos=[x,y,z], size=[length, width, height])
        pyrosim.End()

    def Create_Body(self):
        length = 1
        width = 1
        height = 1 

        pyrosim.Start_URDF("body.urdf")

        pyrosim.Send_Cube(name="Torso", pos=[0,0,1], size=[length, width, height])
        pyrosim.Send_Joint(name="Torso_BackLeg", parent="Torso", child="BackLeg", type="revolute", position=[0,-0.5,1], jointAxis="1 0 0")

        pyrosim.Send_Cube(name="BackLeg", pos=[0,-0.5,0], size=[0.2, 1, 0.2])

        pyrosim.Send_Joint(name="Torso_FrontLeg", parent="Torso", child="FrontLeg", type="revolute", position=[0,0.5,1], jointAxis="1 0 0")
        pyrosim.Send_Cube(name="FrontLeg", pos=[0,0.5,0], size=[0.2, 1, 0.2])

        pyrosim.Send_Joint(name="Torso_LeftLeg", parent="Torso", child="LeftLeg", type="revolute", position=[-0.5, 0, 1], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="LeftLeg", pos=[-0.5, 0, 0], size=[1, 0.2, 0.2])

        pyrosim.Send_Joint(name="Torso_RightLeg", parent="Torso", child="RightLeg", type="revolute", position=[0.5, 0, 1], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="RightLeg", pos=[0.5, 0, 0], size=[1, 0.2, 0.2])

        pyrosim.Send_Joint(name="FrontLeg_FrontLowerLeg", parent="FrontLeg", child="FrontLowerLeg", type="revolute", position=[0, 1, 0], jointAxis="1 0 0")
        pyrosim.Send_Cube(name="FrontLowerLeg", pos=[0, 0, -0.5], size=[0.2, 0.2, 1])

        pyrosim.Send_Joint(name="BackLeg_BackLowerLeg", parent="BackLeg", child="BackLowerLeg", type="revolute", position=[0, -1, 0], jointAxis="1 0 0")
        pyrosim.Send_Cube(name="BackLowerLeg", pos=[0, 0, -0.5], size=[0.2, 0.2, 1])

        pyrosim.Send_Joint(name="LeftLeg_LeftLowerLeg", parent="LeftLeg", child="LeftLowerLeg", type="revolute", position=[-1, 0, 0], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="LeftLowerLeg", pos=[0, 0, -0.5], size=[0.2, 0.2, 1])

        pyrosim.Send_Joint(name="RightLeg_RightLowerLeg", parent="RightLeg", child="RightLowerLeg", type="revolute", position=[1, 0, 0], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="RightLowerLeg", pos=[0, 0, -0.5], size=[0.2, 0.2, 1])

        # pyrosim.Send_Joint(name="Torso_RightLeg2", parent="Torso", child="RightLeg2", type="revolute", position=[0.5, 0.4, 1], jointAxis="0 1 0")
        # pyrosim.Send_Cube(name="RightLeg2", pos=[0.5, 0, 0], size=[1.0, 0.2, 0.2])

        # pyrosim.Send_Joint(name="Torso_LeftLeg2", parent="Torso", child="LeftLeg2", type="revolute", position=[-0.5, -0.4, 1], jointAxis="0 1 0")
        # pyrosim.Send_Cube(name="LeftLeg2", pos=[-0.5, 0, 0], size=[1.0, 0.2, 0.2])

        # pyrosim.Send_Joint(name="LeftLeg2_LeftLowerLeg2", parent="LeftLeg2", child="LeftLowerLeg2", type="revolute", position=[-1, 0, 0], jointAxis="0 1 0")
        # pyrosim.Send_Cube(name="LeftLowerLeg2", pos=[0, 0, -0.5], size=[0.2, 0.2, 1])

        # pyrosim.Send_Joint(name="RightLeg2_RightLowerLeg2", parent="RightLeg2", child="RightLowerLeg2", type="revolute", position=[1, 0, 0], jointAxis="0 1 0")
        # pyrosim.Send_Cube(name="RightLowerLeg2", pos=[0, 0, -0.5], size=[0.2, 0.2, 1])


        pyrosim.End()

    def Create_Brain(self):
        pyrosim.Start_NeuralNetwork(f"brain{self.myID}.nndf")

        pyrosim.Send_Sensor_Neuron(name = 0, linkName = "Torso")
        pyrosim.Send_Sensor_Neuron(name = 1, linkName = "BackLeg")
        pyrosim.Send_Sensor_Neuron(name = 2, linkName = "FrontLeg")
        pyrosim.Send_Sensor_Neuron(name = 3, linkName = "LeftLeg")
        pyrosim.Send_Sensor_Neuron(name = 4, linkName = "RightLeg")
        pyrosim.Send_Sensor_Neuron(name = 5, linkName = "FrontLowerLeg")
        pyrosim.Send_Sensor_Neuron(name = 6, linkName = "BackLowerLeg")
        pyrosim.Send_Sensor_Neuron(name = 7, linkName = "LeftLowerLeg")
        pyrosim.Send_Sensor_Neuron(name = 8, linkName = "RightLowerLeg")
        # pyrosim.Send_Sensor_Neuron(name = 9, linkName = "RightLeg2")
        # pyrosim.Send_Sensor_Neuron(name = 10, linkName = "LeftLeg2")
        # pyrosim.Send_Sensor_Neuron(name = 9, linkName = "RightLowerLeg2")
        # pyrosim.Send_Sensor_Neuron(name = 10, linkName = "LeftLowerLeg2")

        pyrosim.Send_Motor_Neuron(name = c.numMotorNeurons+1, jointName = "Torso_BackLeg")
        pyrosim.Send_Motor_Neuron(name = c.numMotorNeurons+2, jointName = "Torso_FrontLeg")
        pyrosim.Send_Motor_Neuron(name = c.numMotorNeurons+3, jointName = "Torso_LeftLeg")
        pyrosim.Send_Motor_Neuron(name = c.numMotorNeurons+4, jointName = "Torso_RightLeg")
        pyrosim.Send_Motor_Neuron(name = c.numMotorNeurons+5, jointName = "FrontLeg_FrontLowerLeg")
        pyrosim.Send_Motor_Neuron(name = c.numMotorNeurons+6, jointName = "BackLeg_BackLowerLeg")
        pyrosim.Send_Motor_Neuron(name = c.numMotorNeurons+7, jointName = "LeftLeg_LeftLowerLeg")
        pyrosim.Send_Motor_Neuron(name = c.numMotorNeurons+8, jointName = "RightLeg_RightLowerLeg")
        # pyrosim.Send_Motor_Neuron(name = c.numMotorNeurons+9, jointName = "Torso_RightLeg2")
        # pyrosim.Send_Motor_Neuron(name = c.numMotorNeurons+10, jointName = "Torso_LeftLeg2")
        # pyrosim.Send_Motor_Neuron(name = c.numMotorNeurons+12, jointName = "LeftLeg2_LeftLowerLeg2")
        # pyrosim.Send_Motor_Neuron(name = c.numMotorNeurons+11, jointName = "RightLeg2_RightLowerLeg2")

        for currentRow in range(c.numSensorNeurons):
            for currentColumn in range(c.numMotorNeurons):
                pyrosim.Send_Synapse(sourceNeuronName = currentRow, targetNeuronName = currentColumn+c.numSensorNeurons, weight = self.weights[currentRow][currentColumn])

        pyrosim.End()