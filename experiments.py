#!/usr/bin/env python

import sys
from utils import import_models
import_models()

from graph import Node, Graph
from LAOStar import LAOStar
from ILAOStar import ILAOStar
from value_iteration import VI
from CSSPSolver import CSSPSolver
from simple_grid_model import SIMPLEGRIDModel
from simple_grid_model_2 import SIMPLEGRIDModel2
from grid_model import GRIDModel
from grid_model_multiple_bounds import GRIDModel_multiple_bounds
from racetrack_model import RaceTrackModel
from LAO_paper_model import LAOModel
from manual_model import MANUALModel
from manual_model_2 import MANUALModel2
from manual_model_3 import MANUALModel3
from elevator_2011_2 import ELEVATORModel_2011_2
from elevator_2 import ELEVATORModel_2
from routing_model import ROUTINGModel
from cc_routing_model import CCROUTINGModel

# from grid import Grid
# # import functools

# from matplotlib.collections import LineCollection, PolyCollection
# from matplotlib.patches import Ellipse

# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
import numpy as np

import time
import random
import cProfile

import json

def linspace(start, stop, n):
    if n == 1:
        yield stop
        return
    h = (stop - start) / (n - 1)
    for i in range(n):
        yield start + h * i



def racetrack_small():


    sys.setrecursionlimit(8000)

    map_file = "models/racetrack1.txt"
    traj_check_dict_file = "models/racetrack1_traj_check_dict.json"
    heuristic_file = "models/racetrack1_heuristic.json"

    init_state = (1,5,0,0)
    model = RaceTrackModel(map_file, init_state=init_state, traj_check_dict_file=traj_check_dict_file, heuristic_file=heuristic_file, slip_prob=0.1)

    bound = 1

    cssp_solver = CSSPSolver(model, bounds=[bound],VI_epsilon=1e-1,convergence_epsilon=1e-10)

    t = time.time()

    cssp_solver.solve([[0,1.0]])

    policy = cssp_solver.algo.extract_policy()

    cssp_solver.candidate_pruning = True

    try:
        cssp_solver.incremental_update(20)
    except:

        k_best_solution_set = cssp_solver.k_best_solution_set
        for solution in k_best_solution_set:
            print("-"*20)
            print(solution[0])
            print(solution[1])

        print(time.time() - t)

        print(cssp_solver.anytime_solutions)


    k_best_solution_set = cssp_solver.k_best_solution_set
    for solution in k_best_solution_set:
        print("-"*20)
        print(solution[0])
        print(solution[1])

    print(time.time() - t)

    print(cssp_solver.anytime_solutions)



def racetrack_large():


    sys.setrecursionlimit(8000)

    map_file = "models/racetrack_hard_2.txt"
    traj_check_dict_file = "models/racetrack_hard_traj_check_dict.json"
    heuristic_file = "models/racetrack_hard_heuristic.json"

    init_state = (3,1,0,0)
    model = RaceTrackModel(map_file, init_state=init_state, traj_check_dict_file=traj_check_dict_file, heuristic_file=heuristic_file, slip_prob=0.1)

    bound = 1

    cssp_solver = CSSPSolver(model, bounds=[bound],VI_epsilon=1e-1,convergence_epsilon=1e-10)

    t = time.time()

    cssp_solver.solve([[0,1.0]])

    policy = cssp_solver.algo.extract_policy()

    cssp_solver.candidate_pruning = False

    try:
        cssp_solver.incremental_update(2)
    except:

        k_best_solution_set = cssp_solver.k_best_solution_set
        for solution in k_best_solution_set:
            print("-"*20)
            print(solution[0])
            print(solution[1])

        print(time.time() - t)

        print(cssp_solver.anytime_solutions)


    k_best_solution_set = cssp_solver.k_best_solution_set
    for solution in k_best_solution_set:
        print("-"*20)
        print(solution[0])
        print(solution[1])

    print(time.time() - t)

    print(cssp_solver.anytime_solutions)




def racetrack_ring():


    sys.setrecursionlimit(8000)

    map_file = "models/racetrack_ring_2.txt"
    traj_check_dict_file = "models/racetrack_ring_traj_check_dict.json"
    heuristic_file = "models/racetrack_ring_heuristic.json"

    init_state = (1,23,0,0)
    model = RaceTrackModel(map_file, init_state=init_state, traj_check_dict_file=traj_check_dict_file, heuristic_file=heuristic_file, slip_prob=0.1)

    bound = 1

    cssp_solver = CSSPSolver(model, bounds=[bound],VI_epsilon=1e-1,convergence_epsilon=1e-10)

    t = time.time()

    cssp_solver.solve([[0,1.0]])

    policy = cssp_solver.algo.extract_policy()

    cssp_solver.candidate_pruning = False

    try:
        cssp_solver.incremental_update(2)
    except:

        k_best_solution_set = cssp_solver.k_best_solution_set
        for solution in k_best_solution_set:
            print("-"*20)
            print(solution[0])
            print(solution[1])

        print(time.time() - t)

        print(cssp_solver.anytime_solutions)


    k_best_solution_set = cssp_solver.k_best_solution_set
    for solution in k_best_solution_set:
        print("-"*20)
        print(solution[0])
        print(solution[1])

    print(time.time() - t)

    print(cssp_solver.anytime_solutions)
            








def test_single_elevator():

    model = ELEVATORModel_2011_2(n=20, w=2, h=1, prob=0.75,
                                 init_state=((random.randint(1,20),random.randint(1,20)),(0,),random.randint(1,20), 0),
                                 px_dest=(random.randint(1,20),random.randint(1,20)),
                                 hidden_dest=(random.randint(1,20),),
                                 hidden_origin=(random.randint(1,20),))

    print(model.init_state)
    print(model.px_dest)
    print(model.hidden_dest)
    print(model.hidden_origin)


    # init_state = ((1, 19), (0,), 18, 0)
    # px_dest = (8, 13)
    # hidden_dest = (7,)
    # hidden_origin = (8,)
    
    # model = ELEVATORModel_2011_2(n=20, w=2, h=1, prob=0.75,
    #                              init_state=init_state,
    #                              px_dest=px_dest,
    #                              hidden_dest=hidden_dest,
    #                              hidden_origin=hidden_origin)



    bounds = [15,21]

    
    cssp_solver = CSSPSolver(model, bounds=bounds,VI_epsilon=1e-1,convergence_epsilon=1e-10)


    t = time.time()
    # try:
    cssp_solver.find_dual_line_search([[0,10],[0,10]])


    # except:
    print("elapsed time: "+str(time.time()-t))
    print("number of states explored: "+str(len(cssp_solver.graph.nodes)))
    print(cssp_solver.anytime_solutions)




def test_two_elevator():


    # init_state = ((4, 20), (0,), (17, 0), (1, 0))
    # px_dest = (4, 5)
    # hidden_dest = (4,)
    # hidden_origin = (12,)

    # Optimal solution found (tolerance 1.00e-04)
    # Best objective 2.109722011806e+01, best bound 2.109722011806e+01, gap 0.0000%
    # Obj: 21.0972
    # elapsed time: 356.8521845340729
    # number of states explored: 109075



    # init_state = ((6, 5), (0,), (5, 0), (2, 0))
    # px_dest = (15, 4)
    # hidden_dest = (20,)
    # hidden_origin = (19,)

    #################################################################################################


    # init_state = ((15, 11), (0,), (2, 0), (19, 0))
    # px_dest = (16, 13)
    # hidden_dest = (8,)
    # hidden_origin = (9,) 

    # k=7, alpha all bounded by 0.5

    # Obj: 15
    # elapsed time: 79.04550623893738
    # number of states explored: 113439
    # [(15.007812531789144, 9.56572151184082), (15.000000508626291, 20.58628273010254)]
    

    # init_state = ((7, 16), (0,), (18, 0), (15, 0))
    # px_dest = (20, 15)
    # hidden_dest = (20,)
    # hidden_origin = (7,) 

    # # k=7, alpha all bounded by 0.5

    # Obj: 24.0001
    # elapsed time: 113.16026258468628
    # number of states explored: 95973
    # [(24.00003260572751, 59.20481538772583), (24.000032549103103, 67.17310690879822), (24.00003250439962, 130.39673233032227)]

    # init_state = ((7, 13), (0,), (5, 0), (1, 0))
    # px_dest = (13, 19)
    # hidden_dest = (6,)
    # hidden_origin = (20,)

    # k=7, alpha all bounded by 0.5

    # Obj: 32
    # elapsed time: 438.9723823070526
    # number of states explored: 106225
    # [(32.00000000124176, 73.4345850944519)]


    # init_state = ((5, 16), (0,), (12, 0), (16, 0))
    # px_dest = (2, 2)
    # hidden_dest = (16,)
    # hidden_origin = (2,)

    # k=7, alpha all bounded by 0.5

    # Obj: 27
    # elapsed time: 602.465234041214
    # number of states explored: 96134
    # [(29.00000012697031, 67.85555839538574), (27.000002034493566, 83.79212689399719)]



    # init_state = ((15, 11), (0,), (14, 0), (2, 0))
    # px_dest = (2, 14)
    # hidden_dest = (16,)
    # hidden_origin = (20,)

    # k=7, alpha all bounded by 0.5

    # Obj: 15
    # elapsed time: 113.73814916610718
    # number of states explored: 113439
    # [(15.000032552083994, 39.3174729347229), (15.000032552083333, 40.77874803543091), (15.00003180106484, 46.28679370880127)]


    # init_state = ((14, 9), (0,), (15, 0), (1, 0))
    # px_dest = (13, 16)
    # hidden_dest = (2,)
    # hidden_origin = (15,)

    # k=7, alpha all bounded by 0.2

    # Obj: 19.1244
    # elapsed time: 528.1952228546143
    # number of states explored: 113439
    # [(22.00833333330423, 120.79154634475708), (21.883333333333333, 136.8465931415558), (21.508333333333333, 166.21979641914368), (20.50833333330423, 309.6624732017517)]

    # epsilon=0.35
    # [(22.00833333330423, 134.8155038356781), (21.883333333333333, 152.55902552604675), (21.508333333333333, 183.96188974380493), (20.50833333330423, 333.48996686935425), (19.75833333330423, 409.24730587005615), (19.733333333333334, 409.5753426551819), (19.691927083333333, 415.51637268066406)]


    # init_state = ((7, 2), (0,), (17, 0), (12, 0))
    # px_dest = (6, 7)
    # hidden_dest = (8,)
    # hidden_origin = (11,) 

    # # k=7, alpha all bounded by 0.5
    
    # Obj: 15
    # elapsed time: 124.07104086875916
    # number of states explored: 106225
    # [(15.015627034505206, 23.98805546760559), (15.000017293294269, 24.118666887283325)]



    # init_state = ((5, 8), (0,), (16, 0), (17, 0))
    # px_dest = (11, 3)
    # hidden_dest = (8,)
    # hidden_origin = (1,)    

    #     H    0     0                      24.9999967   25.00000  0.00%     -  331s
    #      0     0   25.00000    0    7   25.00000   25.00000  0.00%     -  331s

    # Cutting planes:
    #   MIR: 3
    #   Flow cover: 6

    # Explored 1 nodes (83086 simplex iterations) in 331.85 seconds
    # Thread count was 4 (of 4 available processors)

    # Solution count 1: 25 

    # Optimal solution found (tolerance 1.00e-04)
    # Best objective 2.499999666215e+01, best bound 2.499999666215e+01, gap 0.0000%
    # Obj: 25
    # elapsed time: 370.06756496429443
    # number of states explored: 107743

    # k=7 but canceled at 1, epsilon=0.4
    # [(26.000000000310443, 45.48210835456848), (25.000015259099506, 71.04595589637756)]





    # init_state = ((10, 3), (0,), (11, 0), (4, 0))
    # px_dest = (18, 20)
    # hidden_dest = (9,)
    # hidden_origin = (1,)

    # # k=7, alpha all bounded by 0.5
    
    # Obj: 16.0013
    # elapsed time: 101.37857222557068
    # number of states explored: 113439
    # [(16.03333333333333, 21.71832585334778), (16.002083321412407, 22.48470139503479)]



    init_state = ((10, 11), (0,), (15, 0), (1, 0))
    px_dest = (20, 9)
    hidden_dest = (16,)
    hidden_origin = (3,)

    # # # k=7, alpha all bounded by 0.5

    # Obj: 18.1333
    # elapsed time: 177.80239033699036
    # number of states explored: 113439
    # [(20.125520833333333, 37.19683313369751), (20.008333333333333, 61.02282118797302)]

    # k=4, epsilon=0.3
    # [(20.125520833333333, 34.174766063690186), (20.008333333333333, 55.9601948261261), (19.258333333333333, 110.79541301727295), (18.508333333333333, 119.45532584190369), (18.133333333333333, 139.16023015975952)]

    ###################################################################################################

    # init_state = ((19, 8), (0,), (9, 0), (6, 0))
    # px_dest = (1, 20)
    # hidden_dest = (8,)
    # hidden_origin = (14,)

    # init_state = ((10, 11), (0,), (15, 0), (1, 0))
    # px_dest = (20, 9)
    # hidden_dest = (16,)
    # hidden_origin = (3,)

    # init_state = ((4, 13), (0,), (19, 0), (10, 0))
    # px_dest = (2, 19)
    # hidden_dest = (6,)
    # hidden_origin = (17,)


    # init_state = ((7, 16), (0,), (18, 0), (15, 0))
    # px_dest = (20, 15)
    # hidden_dest = (20,)
    # hidden_origin = (7,)

    # init_state = ((5, 16), (0,), (12, 0), (16, 0))
    # px_dest = (2, 2)
    # hidden_dest = (16,)
    # hidden_origin = (2,)


    # init_state = ((14, 9), (0,), (15, 0), (1, 0))
    # px_dest = (13, 16)
    # hidden_dest = (2,)
    # hidden_origin = (15,)

    # init_state = ((7, 15), (0,), (5, 0), (11, 0))
    # px_dest = (6, 12)
    # hidden_dest = (20,)
    # hidden_origin = (6,)

    # init_state = ((10, 11), (0,), (15, 0), (1, 0))
    # px_dest = (20, 9)
    # hidden_dest = (16,)
    # hidden_origin = (3,)

    

    # init_state = ((7, 10), (0,), (20, 0), (20, 0))
    # px_dest = (9, 13)
    # hidden_dest = (3,)
    # hidden_origin = (17,)

    #     0     0   20.63643    0    5          -   20.63643      -     -  243s
    #      0     0   20.63643    0    5          -   20.63643      -     -  305s
    # H    0     0                      22.0020833   20.63643  6.21%     -  481s
    #      0     2   20.63643    0    5   22.00208   20.63643  6.21%     -  496s
    #      1     4   21.29182    1    5   22.00208   20.63643  6.21%  2154  547s
    #      3     6   21.29182    2    3   22.00208   20.63880  6.20%   729  601s
    #      5     8   21.29182    3    3   22.00208   20.63880  6.20%   441  663s
    #      7    10   21.60521    3    6   22.00208   20.67917  6.01%   611  822s
    #      9    12   21.60521    4    5   22.00208   20.76380  5.63%   791  906s
    #     11    15   21.50208    4    3   22.00208   20.76380  5.63%   664  939s
    #     14    18   21.50208    5    2   22.00208   20.76380  5.63%   524  988s
    #     17    21   21.50208    6    2   22.00208   20.76380  5.63%   434 1022s
    #     20    23   21.75208    7    2   22.00208   20.76380  5.63%   398 1042s
    #     22    24 infeasible    7        22.00208   20.76380  5.63%   381 1064s
    #     25    27   21.75208    8    2   22.00208   20.76380  5.63%   369 1107s
    #     28    29   21.75208    9    2   22.00208   20.76380  5.63%   330 1157s
    #     30    33   21.75208   10    2   22.00208   20.76380  5.63%   308 1197s
    #     34    36   21.75208   11    2   22.00208   20.76380  5.63%   272 1244s
    #     37    37   21.75208   12    2   22.00208   20.76380  5.63%   278 1320s
    #     40    36   21.75208   13    2   22.00208   20.76380  5.63%   280 1408s
    #     45    35     cutoff   13        22.00208   20.97999  4.65%   285 1631s
    #     50    39   22.00208    4    4   22.00208   20.97999  4.65%   486 1807s
    # H   54    42                      22.0020827   20.97999  4.65%   450 1884s
    #     58    43     cutoff    7        22.00208   21.00208  4.55%   440 1924s
    #     65    47   21.00208    6    2   22.00208   21.00208  4.55%   395 2002s
    #     69    55   21.00208    8    2   22.00208   21.00208  4.55%   372 2037s
    #     77    69   21.00208   15    2   22.00208   21.00208  4.55%   334 2247s
    #     91    85 infeasible   19        22.00208   21.00208  4.55%   401 2487s
    #    141    80     cutoff   19        22.00208   21.00208  4.55%   428 2826s
    #    163    89   21.78333   16    3   22.00208   21.00208  4.55%   410 3007s
    # H  201    89                      22.0020825   21.00208  4.55%   364 3007s
    #    207    86     cutoff   18        22.00208   21.00208  4.55%   376 3162s
    #    221    86   21.78333   14    2   22.00208   21.00208  4.55%   392 3272s
    #    256    86     cutoff   14        22.00208   21.00208  4.55%   369 3359s
    #    273    79   22.00208   10    3   22.00208   21.13049  3.96%   368 3434s
    #    323    88     cutoff   15        22.00208   21.19383  3.67%   332 3510s
    #    354    82   22.00208   22    3   22.00208   21.21865  3.56%   318 3619s
    #    375    92   22.00208   10    3   22.00208   21.26581  3.35%   322 3717s
    #    397   108   22.00208    9    3   22.00208   21.29182  3.23%   317 3791s
    # H  423   108                      21.9895833   21.33906  2.96%   307 3792s
    #    445   118     cutoff   19        21.98958   21.33906  2.96%   294 3883s
    #    467   114     cutoff   11        21.98958   21.33906  2.96%   297 3994s
    #    479   106   21.75208   16    2   21.98958   21.33906  2.96%   302 4299s
    #    485   104     cutoff   15        21.98958   21.33906  2.96%   352 4395s
    #    492   106     cutoff    8        21.98958   21.38388  2.75%   358 4492s
    # Killed
    



    model = ELEVATORModel_2(n=20, w=2, h=1, prob=0.75, init_state=init_state, \
                                 px_dest=px_dest, \
                                 hidden_dest=hidden_dest, \
                                 hidden_origin=hidden_origin)
    

    

    # model = ELEVATORModel_2(n=20, w=2, h=1, prob=0.75, init_state=((random.randint(1,20),random.randint(1,20)),(0,),(random.randint(1,20), 0), (random.randint(1,20),0)), \
    #                              px_dest=(random.randint(1,20),random.randint(1,20)), \
    #                              hidden_dest=(random.randint(1,20),), \
    #                              hidden_origin=(random.randint(1,20),))


    # print(model.init_state)
    # print(model.px_dest)
    # print(model.hidden_dest)
    # print(model.hidden_origin)
    
    

    alpha = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    bounds = [15,21,15,21,15,21]


    cssp_solver = CSSPSolver(model, bounds=bounds,VI_epsilon=1e-1,convergence_epsilon=1e-100)


    t = time.time()

    try:
        cssp_solver.find_dual_multiple_bounds_generalized([[0,.5],[0,.5],[0,.5],[0,.5],[0,.5],[0,.5]])
    except:
        print(time.time() - t)
        print(cssp_solver.anytime_solutions)


    cssp_solver.candidate_pruning = True

    # try:
    cssp_solver.incremental_update(600)
    # except:

    #     k_best_solution_set = cssp_solver.k_best_solution_set
    #     for solution in k_best_solution_set:
    #         print("-"*20)
    #         print(solution[0])
    #         print(solution[1])

    #     print(time.time() - t)

    #     print(cssp_solver.anytime_solutions)


    k_best_solution_set = cssp_solver.k_best_solution_set
    for solution in k_best_solution_set:
        print("-"*20)
        print(solution[0])
        print(solution[1])

    print("elapsed time: "+str(time.time()-t))
    print("number of states explored: "+str(len(cssp_solver.graph.nodes)))

    print(cssp_solver.anytime_solutions)










def test_dual_alg_routing():


    init_state = ((0,0),(5,5))
    size = (10,10)
    goal = (9,9)
    model = ROUTINGModel(size, init_state, goal, prob_right_transition=0.8)

    bound = 1 



    cssp_solver = CSSPSolver(model, bounds=[bound],VI_epsilon=1e-1,convergence_epsilon=1e-10)

    t = time.time()

    cssp_solver.solve([[0,10.0]])

    # policy = cssp_solver.algo.extract_policy()

    # cssp_solver.candidate_pruning = True

    # try:
    #     cssp_solver.incremental_update(2)
    # except:

    #     k_best_solution_set = cssp_solver.k_best_solution_set
    #     for solution in k_best_solution_set:
    #         print("-"*20)
    #         print(solution[0])
    #         print(solution[1])

    #     print(time.time() - t)

    #     print(cssp_solver.anytime_solutions)


    # k_best_solution_set = cssp_solver.k_best_solution_set
    # for solution in k_best_solution_set:
    #     print("-"*20)
    #     print(solution[0])
    #     print(solution[1])

    print(time.time() - t)

    print(cssp_solver.anytime_solutions)



def cc_routing():

    # init_state = ((0,0),(2,0))
    # size = (3,2)
    # goal = (2,1)
    # model = CCROUTINGModel(size, obs_num=1, obs_dir='U',obs_boundary=[(0,0)], init_state=init_state, goal=goal, prob_right_transition=0.99)

    # bounds = [0.1]

    # init_state = ((0,0),(7,1))
    # size = (15,15)
    # goal = (5,13)
    # model = CCROUTINGModel(size, obs_num=1, obs_dir='U',obs_boundary=[(1,2)], init_state=init_state, goal=goal, prob_right_transition=0.9)

    # bounds = [0.02]


    # init_state = ((0,0),(5,1), (8,9))
    # size = (10,10)
    # goal = (5,7)
    # model = CCROUTINGModel(size, obs_num=2, obs_dir=['U','L'],obs_boundary=[(1,2), (2,2)], init_state=init_state, goal=goal, prob_right_transition=0.8)

    # bounds = [0.2]


    #########################################################################################################


    # # # obj 1 scenario 1

    # init_state = ((0,0),(1,5))
    # size = (10,10)
    # goal = (5,7)
    # model = CCROUTINGModel(size, obs_num=1, obs_dir=['R'],obs_boundary=[(2,1)], init_state=init_state, goal=goal, prob_right_transition=0.8)

    # bounds = [0.001]


    # obj 1 scenario 2

    # init_state = ((0,0),(6,1))
    # size = (10,10)
    # goal = (8,8)
    # model = CCROUTINGModel(size, obs_num=1, obs_dir=['U'],obs_boundary=[(1,2)], init_state=init_state, goal=goal, prob_right_transition=0.8)

    # bounds = [0.001]

    # use only batch



    # # # # obj 1 scenario 3

    # init_state = ((0,0),(2,2))
    # size = (10,15)
    # goal = (5,6)
    # model = CCROUTINGModel(size, obs_num=1, obs_dir=['R'],obs_boundary=[(1,2)], init_state=init_state, goal=goal, prob_right_transition=0.8)

    # bounds = [0.001]   

    # use only batch


    # # # # obj 1 scenario 4

    # init_state = ((0,0),(7,5))
    # size = (10,15)
    # goal = (8,12)
    # model = CCROUTINGModel(size, obs_num=1, obs_dir=['L'],obs_boundary=[(1,1)], init_state=init_state, goal=goal, prob_right_transition=0.8)

    # bounds = [0.001]


    # # # obj 1 scenario 5

    # init_state = ((0,0),(12,5))
    # size = (15,15)
    # goal = (6,5)
    # model = CCROUTINGModel(size, obs_num=1, obs_dir=['L'],obs_boundary=[(2,1)], init_state=init_state, goal=goal, prob_right_transition=0.8)

    # bounds = [0.001]    


    
    # # # obj 1 scenario 6

    # init_state = ((0,0),(7,7))
    # size = (15,15)
    # goal = (13,13)
    # model = CCROUTINGModel(size, obs_num=1, obs_dir=['L'],obs_boundary=[(2,1)], init_state=init_state, goal=goal, prob_right_transition=0.8)

    # bounds = [0.001]


    

    # # obj 2 scenario 1 (goal is close, so i-dual performs well)
    # init_state = ((0,0),(7,1),(1,8))
    # size = (10,10)
    # goal = (5,2)
    # model = CCROUTINGModel(size, obs_num=2, obs_dir=['U','R'],obs_boundary=[(1,2),(1,1)], init_state=init_state, goal=goal, prob_right_transition=0.9)

    # bounds = [0.001]

    # obj 2 scenario 2 
    # init_state = ((0,0),(7,1),(1,8))
    # size = (10,10)
    # goal = (8,5)
    # model = CCROUTINGModel(size, obs_num=2, obs_dir=['U','R'],obs_boundary=[(1,2),(1,1)], init_state=init_state, goal=goal, prob_right_transition=0.9)

    # bounds = [0.002]


    # obj 2 scenario 3
    init_state = ((1,1),(7,5),(5,10))
    size = (15,10)
    goal = (13,8)
    model = CCROUTINGModel(size, obs_num=2, obs_dir=['R','D'],obs_boundary=[(2,1),(1,2)], init_state=init_state, goal=goal, prob_right_transition=0.6)

    bounds = [0.002]


    cssp_solver = CSSPSolver(model, bounds=bounds,VI_epsilon=1e-100,convergence_epsilon=1e-300)

    t = time.time()

    cssp_solver.solve([[0,1000.0]])

    policy = cssp_solver.algo.extract_policy()

    cssp_solver.candidate_pruning = False

    # try:
    cssp_solver.incremental_update(300)
    # except:

    #     k_best_solution_set = cssp_solver.k_best_solution_set
    #     for solution in k_best_solution_set:
    #         print("-"*20)
    #         print(solution[0])
    #         print(solution[1])

    #     print(time.time() - t)

    #     print(cssp_solver.anytime_solutions)


    k_best_solution_set = cssp_solver.k_best_solution_set
    for solution in k_best_solution_set:
        print("-"*20)
        print(solution[0])
        print(solution[1])

    print(time.time() - t)

    print(cssp_solver.anytime_solutions)


    
# racetrack_small()
# racetrack_large()
racetrack_ring()

# test_two_elevator()


# cc_routing()
