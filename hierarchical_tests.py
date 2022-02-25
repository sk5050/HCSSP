#!/usr/bin/env python

import sys
from utils import import_models
import_models()

from graph import Node, Graph
from LAOStar import LAOStar
from ILAOStar import ILAOStar
from value_iteration import VI
from CSSPSolver import CSSPSolver
from grid_model import GRIDModel
from RHCPSolver import *
import numpy as np
import time
import random
import cProfile
import json
import pprint


def evacuation_test():

    t = time.time()

    episode_dict = {'episode_1': {'file_name' : "models/evacuation_scenario/episode_1.txt",
                                  'likelihood': 1.0},
                    'episode_2': {'file_name' : "models/evacuation_scenario/episode_2.txt",
                                  'likelihood': 1.0},
                    'episode_3': {'file_name' : "models/evacuation_scenario/episode_3.txt",
                                  'likelihood': 1.0},
                    'episode_4': {'file_name' : "models/evacuation_scenario/episode_4.txt",
                                  'likelihood': 1.0},
                    'episode_5': {'file_name' : "models/evacuation_scenario/episode_5.txt",
                                  'likelihood': 1.0},
                    'episode_6': {'file_name' : "models/evacuation_scenario/episode_6.txt",
                                  'likelihood': 1.0},
                    'episode_7': {'file_name' : "models/evacuation_scenario/episode_7.txt",
                                  'likelihood': 1.0},
                    'episode_8': {'file_name' : "models/evacuation_scenario/episode_8.txt",
                                  'likelihood': 1.0},
                    'episode_9': {'file_name' : "models/evacuation_scenario/episode_9.txt",
                                  'likelihood': 0.99},
                    'episode_10': {'file_name' : "models/evacuation_scenario/episode_10.txt",
                                  'likelihood': 0.01},
                    'episode_11': {'file_name' : "models/evacuation_scenario/episode_11.txt",
                                   'likelihood': 0.99},
                    'episode_12': {'file_name' : "models/evacuation_scenario/episode_12.txt",
                                   'likelihood': 0.99},
                    'episode_13': {'file_name' : "models/evacuation_scenario/episode_13.txt",
                                   'likelihood': 0.99*0.01},
                    'episode_14': {'file_name' : "models/evacuation_scenario/episode_14.txt",
                                   'likelihood': 0.99*0.99}
                    }
                    

    
    solver = RHCPSolver(episode_dict, bound=1)
    try:
        solver.solve()
    except:
        print(solver.global_lb_list)
        print(solver.global_ub_list)
        print(solver.count)

    print(time.time() - t)




evacuation_test()
