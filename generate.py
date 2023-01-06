import pyrosim.pyrosim as pyrosim

pyrosim.Start_SDF("boxes.sdf")

x = 0
y = 0
z = 0.5

length = 1
width = 1
height = 1 

for i in range(0, 5):
    x = i
    for j in range(0, 5):
        y = j
        length = 1
        width = 1
        height = 1
        z = 0.5
        for k in range(0, 10):
            pyrosim.Send_Cube(name="Box", pos=[x,y,z], size=[length, width, height])
            z += 1
            length *= 0.90
            width *= 0.90
            height *= 0.90 


pyrosim.End()