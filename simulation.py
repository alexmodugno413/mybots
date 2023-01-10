from world import WORLD
from robot import ROBOT
import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import time

class SIMULATION:
    
    def __init__(self):
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,-9.8)

        self.world = WORLD()
        self.robot = ROBOT()
        pyrosim.Prepare_To_Simulate(self.robot.robotId)

    def Run(self):
        for i in range(0, 1000):
            p.stepSimulation()
            print(i)
            
            # # store sensor values
            # backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
            # frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")

            # # store motor values
            # pyrosim.Set_Motor_For_Joint(bodyIndex=robotId, jointName=b'Torso_BackLeg', controlMode=p.POSITION_CONTROL, targetPosition=blTargetAngles[i], maxForce=500)
            # pyrosim.Set_Motor_For_Joint(bodyIndex=robotId, jointName=b'Torso_FrontLeg', controlMode=p.POSITION_CONTROL, targetPosition=flTargetAngles[i], maxForce=500)

            time.sleep(1/60)