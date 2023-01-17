import numpy
import pyrosim.pyrosim as pyrosim
import random
import os
import time

class SOLUTION:
    
    def __init__(self, nextAvailableId):
        self.myID = nextAvailableId
        self.weights = numpy.random.rand(3, 2)
        self.weights = self.weights * 2 - 1

    def Start_Simulation(self, directOrGUI):
        self.Create_World()
        self.Create_Body()
        self.Create_Brain()
        os.system(f'python3 simulate.py {directOrGUI} {str(self.myID)} &')

    def Wait_For_Simulation_To_End(self, directOrGUI):
        fitnessFile = open(f"fitness{str(self.myID)}.txt", "r")
        while not os.path.exists(f"fitness{str(self.myID)}.txt"):
            time.sleep(0.01)
        self.fitness = float(fitnessFile.read())
        print(f'self.fitness{self.myID}: {self.fitness}')
        fitnessFile.close()
        os.system(f"rm fitness{str(self.myID)}.txt")

    # def Evaluate(self, directOrGUI):
    #     self.Create_World()
    #     self.Create_Body()
    #     self.Create_Brain()
    #     os.system(f'python3 simulate.py {directOrGUI} {str(self.myID)} &')
    #     fitnessFile = open(f"fitness{str(self.myID)}.txt", "r")
    #     while not os.path.exists(fitnessFile):
    #         time.sleep(0.01)
    #     self.fitness = float(fitnessFile.read())
    #     print(f'self.fitness{self.myID}: {self.fitness}')
    #     fitnessFile.close()

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

        pyrosim.Send_Cube(name="Torso", pos=[1.5,0,1.5], size=[length, width, height])
        pyrosim.Send_Joint(name="Torso_BackLeg", parent="Torso", child="BackLeg", type="revolute", position=[1,0,1])

        pyrosim.Send_Cube(name="BackLeg", pos=[-0.5,0,-0.5], size=[length, width, height])

        pyrosim.Send_Joint(name="Torso_FrontLeg", parent="Torso", child="FrontLeg", type="revolute", position=[2,0,1])
        pyrosim.Send_Cube(name="FrontLeg", pos=[0.5,0,-0.5], size=[length, width, height])

        pyrosim.End()

    def Create_Brain(self):
        pyrosim.Start_NeuralNetwork(f"brain{self.myID}.nndf")

        pyrosim.Send_Sensor_Neuron(name = 0, linkName = "Torso")
        pyrosim.Send_Sensor_Neuron(name = 1, linkName = "BackLeg")
        pyrosim.Send_Sensor_Neuron(name = 2, linkName = "FrontLeg")
        sensorNeurons = [0, 1, 2]

        pyrosim.Send_Motor_Neuron(name = 3, jointName = "Torso_BackLeg")
        pyrosim.Send_Motor_Neuron(name = 4, jointName = "Torso_FrontLeg")
        motorNeurons = [0, 1]

        for currentRow in sensorNeurons:
            for currentColumn in motorNeurons:
                pyrosim.Send_Synapse(sourceNeuronName = currentRow, targetNeuronName = currentColumn+3, weight = self.weights[currentRow][currentColumn])

        pyrosim.End()