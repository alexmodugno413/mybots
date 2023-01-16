import pyrosim.pyrosim as pyrosim
import random

def Create_World():
    x = -3
    y = 3
    z = 0.5
    length = 1
    width = 1
    height = 1 

    pyrosim.Start_SDF("world.sdf")
    pyrosim.Send_Cube(name="Box", pos=[x,y,z], size=[length, width, height])
    pyrosim.End()

# def Create_Robot():
#     length = 1
#     width = 1
#     height = 1 

#     pyrosim.Start_URDF("body.urdf")

#     pyrosim.Send_Cube(name="Torso", pos=[1.5,0,1.5], size=[length, width, height])
#     pyrosim.Send_Joint(name="Torso_BackLeg", parent="Torso", child="BackLeg", type="revolute", position=[1,0,1])

#     pyrosim.Send_Cube(name="BackLeg", pos=[-0.5,0,-0.5], size=[length, width, height])

#     pyrosim.Send_Joint(name="Torso_FrontLeg", parent="Torso", child="FrontLeg", type="revolute", position=[2,0,1])
#     pyrosim.Send_Cube(name="FrontLeg", pos=[0.5,0,-0.5], size=[length, width, height])

#     pyrosim.End()

def Generate_Body():
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

def Generate_Brain():
    pyrosim.Start_NeuralNetwork("brain.nndf")

    pyrosim.Send_Sensor_Neuron(name = 0, linkName = "Torso")
    pyrosim.Send_Sensor_Neuron(name = 1, linkName = "BackLeg")
    pyrosim.Send_Sensor_Neuron(name = 2, linkName = "FrontLeg")
    sensorNeurons = [0, 1, 2]

    pyrosim.Send_Motor_Neuron(name = 3, jointName = "Torso_BackLeg")
    pyrosim.Send_Motor_Neuron(name = 4, jointName = "Torso_FrontLeg")
    motorNeurons = [3, 4]

    # pyrosim.Send_Synapse(sourceNeuronName = 0, targetNeuronName = 3, weight = 1.0)
    # pyrosim.Send_Synapse(sourceNeuronName = 1, targetNeuronName = 3, weight = 1.0)

    # pyrosim.Send_Synapse(sourceNeuronName = 0, targetNeuronName = 4, weight = 1.0)
    # pyrosim.Send_Synapse(sourceNeuronName = 2, targetNeuronName = 4, weight = 1.0)

    # pyrosim.Send_Synapse(sourceNeuronName = 1, targetNeuronName = 4, weight = 1.0)
    # pyrosim.Send_Synapse(sourceNeuronName = 2, targetNeuronName = 3, weight = 1.0)

    for i in sensorNeurons:
        for j in motorNeurons:
            pyrosim.Send_Synapse(sourceNeuronName = i, targetNeuronName = j, weight = random.uniform(-1, 1))

    pyrosim.End()


Create_World()
# Create_Robot()
Generate_Body()
Generate_Brain()



# pyrosim.Send_Cube(name="Link0", pos=[0,0,0.5], size=[length, width, height])
    # pyrosim.Send_Joint(name="Link0_Link1", parent="Link0", child="Link1", type="revolute", position=[0,0,1])

    # pyrosim.Send_Cube(name="Link1", pos=[0,0,0.5], size=[length, width, height])
    # pyrosim.Send_Joint(name="Link1_Link2", parent="Link1", child="Link2", type="revolute", position=[0,0,1])

    # pyrosim.Send_Cube(name="Link2", pos=[0,0,0.5], size=[length, width, height])
    # pyrosim.Send_Joint(name="Link2_Link3", parent="Link2", child="Link3", type="revolute", position=[0,0.5,0.5])

    # pyrosim.Send_Cube(name="Link3", pos=[0,0.5,0], size=[length, width, height])
    # pyrosim.Send_Joint(name="Link3_Link4", parent="Link3", child="Link4", type="revolute", position=[0,1,0])

    # pyrosim.Send_Cube(name="Link4", pos=[0,0.5,0], size=[length, width, height])
    # pyrosim.Send_Joint(name="Link4_Link5", parent="Link4", child="Link5", type="revolute", position=[0,0.5,-0.5])

    # pyrosim.Send_Cube(name="Link5", pos=[0,0,-0.5], size=[length, width, height])
    # pyrosim.Send_Joint(name="Link5_Link6", parent="Link5", child="Link6", type="revolute", position=[0,0,-0.5])

    # pyrosim.Send_Cube(name="Link6", pos=[0,0,-0.5], size=[length, width, height])





