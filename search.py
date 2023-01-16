import os
from parallelHillClimber import PARALLEL_HILL_CLIMBER
import constants as c
import pybullet as p

phc = PARALLEL_HILL_CLIMBER()
phc.Evolve()
phc.Show_Best()
