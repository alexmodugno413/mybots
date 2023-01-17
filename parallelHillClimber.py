from solution import SOLUTION
import constants as c
import copy
import os

class PARALLEL_HILL_CLIMBER:
    
    def __init__(self):
        # self.parent = SOLUTION()
        for i in range(0, c.populationSize):
            os.system(f"rm brain{i}.nndf")
            os.system(f"rm fitness{i}.nndf")
        self.nextAvailableID = 0
        self.parents = {}
        for i in range(0, c.populationSize):
            self.parents[i] = SOLUTION(self.nextAvailableID)
            self.nextAvailableID += 1

    def Evolve(self):
        # for i in range(0, c.populationSize):
        #     self.parents[i].Start_Simulation("DIRECT")
        # for i in range(0, c.populationSize):
        #     self.parents[i].Wait_For_Simulation_To_End("DIRECT")
        self.Evaluate(self.parents)
        for currentGeneration in range(c.numberOfGenerations):
            self.Evolve_For_One_Generation()

    def Evolve_For_One_Generation(self):
        self.Spawn()
        self.Mutate()
        self.Evaluate(self.children)
        self.Print()
        # print("\n")
        # print(f'self.parent.fitness: {self.parent.fitness}, self.child.fitness: {self.child.fitness}')
        # self.Select()

    def Spawn(self):
        self.children = {}
        for i in self.parents.keys():
            self.children[i] = copy.deepcopy(self.parents[i])
            self.children[i].Set_ID(self.nextAvailableID)
            self.nextAvailableID += 1

    def Mutate(self):
        for i in self.children.keys():
            self.children[i].Mutate()

    def Evaluate(self, solutions):
        for i in range(0, c.populationSize):
            solutions[i].Start_Simulation("DIRECT")
        for i in range(0, c.populationSize):
            solutions[i].Wait_For_Simulation_To_End("DIRECT")

    def Print(self):
        print("\n")
        for i in self.parents.keys():
            print(f'self.parents{i}.fitness: {self.parents[i].fitness}, self.children{i}.fitness: {self.children[i].fitness}')
        print("\n")

    def Select(self):
        if self.parent.fitness > self.child.fitness:
            self.parent = self.child

    def Show_Best(self):
        # self.parent.Evaluate("GUI")
        pass