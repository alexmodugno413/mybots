# import pybullet as p
# import time
# import pybullet_data
# import pyrosim.pyrosim as pyrosim
# import numpy
# import math
# import random
# import constants as c

# physicsClient = p.connect(p.GUI)
# p.setAdditionalSearchPath(pybullet_data.getDataPath())

# p.setGravity(0,0,-9.8)
# planeId = p.loadURDF("plane.urdf")
# robotId = p.loadURDF("body.urdf") # indicates which robot you want prepared for simulation
# p.loadSDF("world.sdf")

# pyrosim.Prepare_To_Simulate(robotId)
# backLegSensorValues = numpy.zeros(1000)
# frontLegSensorValues = numpy.zeros(1000)

# # back leg
# blOldTargetAngles = numpy.linspace(0, 2*numpy.pi, 1000)
# blTargetAngles = numpy.zeros(1000)

# c.blAmplitude = numpy.pi/4
# c.blFrequency = 10
# c.blPhaseOffset = 0

# for i in range(0, 1000):
#     blTargetAngles[i] = numpy.sin(blOldTargetAngles[i]) * numpy.pi/4
#     blTargetAngles[i] = c.blAmplitude * numpy.sin(c.blFrequency * (blOldTargetAngles[i] + c.blPhaseOffset))
#     print(blTargetAngles[i]) 

# # front leg
# flOldTargetAngles = numpy.linspace(0, 2*numpy.pi, 1000)
# flTargetAngles = numpy.zeros(1000)

# c.flAmplitude = numpy.pi/4
# c.flFrequency = 10
# c.flPhaseOffset = 0

# for i in range(0, 1000):
#     flTargetAngles[i] = numpy.sin(flOldTargetAngles[i]) * numpy.pi/4
#     flTargetAngles[i] = c.flAmplitude * numpy.sin(c.flFrequency * (flOldTargetAngles[i] + c.flPhaseOffset))
#     print(flTargetAngles[i]) 

# numpy.save('data/blTargetAngles.npy', blTargetAngles)
# numpy.save('data/flTargetAngles.npy', flTargetAngles)



# for i in range(0, 1000):
#     p.stepSimulation()
    
#     # store sensor values
#     backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
#     frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")

#     # store motor values
#     pyrosim.Set_Motor_For_Joint(bodyIndex=robotId, jointName=b'Torso_BackLeg', controlMode=p.POSITION_CONTROL, targetPosition=blTargetAngles[i], maxForce=500)
#     pyrosim.Set_Motor_For_Joint(bodyIndex=robotId, jointName=b'Torso_FrontLeg', controlMode=p.POSITION_CONTROL, targetPosition=flTargetAngles[i], maxForce=500)

#     time.sleep(1/60)

# numpy.save('data/backLegSensorValues.npy', backLegSensorValues)
# numpy.save('data/frontLegSensorValues.npy', frontLegSensorValues)

# p.disconnect()

from simulation import SIMULATION

simulation = SIMULATION()
simulation.Run()
