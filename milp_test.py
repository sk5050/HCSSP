#!/usr/bin/env python

import sys
from utils import import_models
import_models()

from graph import Node, Graph
from LAOStar import LAOStar
from ILAOStar import ILAOStar
from value_iteration import VI
from CSSPSolver import CSSPSolver
from MILPSolver import MILPSolver
from aggregated_model import AGGModel
import numpy as np
import time
import random
import cProfile
import json




def agg_model():

    model = AGGModel()

    bounds = [1]

    solver = MILPSolver(model, bounds)

    t = time.time()
    solver.solve_opt_MILP()
    print("elapsed time: "+str(time.time()-t))
    print("number of states explored: "+str(len(solver.algo.graph.nodes)))


agg_model()
