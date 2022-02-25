#!/usr/bin/env python

import sys
from utils import import_models
import_models()

import time
import math
from utils import *
from graph import Node, Graph
from LAOStar import LAOStar
from ILAOStar import ILAOStar
from CSSPSolver import CSSPSolver
from procedural_model import PROCEDURALModel
from grid_model import GRIDModel

import copy
from heapq import *


import numpy as np
import pprint

class RHCPSolver(object):

    def __init__(self, episode_dict, bound=0.05):

        self.episode_set = dict()

        self.episode_history = dict()

        for episode_name, episode_info in episode_dict.items():

            self.episode_set[episode_name] = self.generate_episode(episode_name,
                                                                   episode_info['file_name'],
                                                                   episode_info['likelihood'])

            self.episode_history[episode_name] = dict()
        
        self.bound = bound
        self.compute_episodes_bounds()
        self.partition = Partition(self.episode_set)
        self.global_lb = np.inf
        self.global_ub = np.inf
        self.t = time.time()
        self.global_lb_list = []
        self.global_ub_list = []

        self.count = 0


    def solve(self):

        episodes_values = dict()

        results = []

        t = time.time()

        while True:

            refined_partitions = self.partition.refine_partition()


            for refined_partition in refined_partitions:

                for episode_name, bound_dict in refined_partition['bounds'].items():

                    if bound_dict['ub'] in self.episode_history[episode_name]:
                        episodes_values[episode_name] = self.episode_history[episode_name][bound_dict['ub']]

                    else:
                        self.count += 1
                        self.episode_set[episode_name].solve(bound_dict['ub'])

                        episodes_values[episode_name] = [self.episode_set[episode_name].value,
                                                         self.episode_set[episode_name].risk,
                                                         self.episode_set[episode_name].lagrangian_value,
                                                         self.episode_set[episode_name].lb]

                        self.episode_history[episode_name][bound_dict['ub']] = episodes_values[episode_name]


                ### Upper bound computation

                self.model = PROCEDURALModel(episodes_values)
                self.procedural_solver = CSSPSolver(self.model, bounds=[self.bound],VI_epsilon=1e-1,convergence_epsilon=1e-10)


                # try:
                is_feasible = self.procedural_solver.solve([[0,3]])

                # self.procedural_solver.incremental_update(2)

                # if is_feasible==False:
                #     print("hi")
                #     pprint.pprint(refined_partition)
                #     pprint.pprint(episodes_values)
                #     continue

                try:
                    policy = self.procedural_solver.algo.extract_policy()

                    self.value = self.procedural_solver.anytime_solutions[-1][0]
                    self.risk = self.procedural_solver.anytime_solutions[-1][1] + self.bound

                    results.append((self.value, self.risk, time.time() - t))

                    self.compute_upper_bound(refined_partition)

                except:
                    print("hello")

                ### Lower bound computation

                self.model = PROCEDURALModel(episodes_values, lagrangian=True)
                self.procedural_solver = CSSPSolver(self.model, bounds=[self.bound],VI_epsilon=1e-1,convergence_epsilon=1e-10)


                # try:
                is_feasible = self.procedural_solver.solve([[0,3]])

                # self.procedural_solver.incremental_update(2)

                # if is_feasible==False:
                #     print("hi")
                #     pprint.pprint(refined_partition)
                #     pprint.pprint(episodes_values)
                #     continue

                try:

                    policy = self.procedural_solver.algo.extract_policy()

                    self.value = self.procedural_solver.k_best_solution_set[-1][0]
                    self.risk = self.procedural_solver.anytime_solutions[-1][1] + self.bound


                    self.compute_lower_bound(refined_partition)

                except:
                    print("infeasible.")

                self.update_global_upper_bound()
                self.update_global_lower_bound()

            self.update_global_upper_bound()
            self.update_global_lower_bound()
                
            pprint.pprint(results)


        lower_bounds = []
        upper_bounds = []

        for partition in self.partition.partition:
            upper_bounds.append(partition['alpha'])
            lower_bounds.append(partition['beta'])

        pprint.pprint(upper_bounds)
        pprint.pprint(lower_bounds)
            

        print("global lb: "+str(self.global_lb))
        print("global ub: "+str(self.global_ub))

        print(len(self.partition.partition))
        time.sleep(1000)



    def generate_episode(self, name, map_file, likelihood=1.0):

        init_state = None
        size = None
        goal = None
        model = GRIDModel(init_state, size, goal, map_file=map_file)

        return Episode(name, model, likelihood)



    def compute_episodes_bounds(self):
        for episode_name, episode in self.episode_set.items():
            episode.lb = 0
            episode.ub = self.bound * (1/episode.likelihood)



    def compute_upper_bound(self,partition):

        if self.risk <= self.bound:
            upper_bound = self.value
            partition['alpha'] = upper_bound
            

    def compute_lower_bound(self,partition):

        lower_bound = self.value
        partition['beta'] = lower_bound

        

    def update_global_upper_bound(self):

        for partition in self.partition.partition:

            if partition['alpha'] < self.global_ub:
                self.global_ub = partition['alpha']

                self.global_ub_list.append((self.global_ub, time.time() - self.t))
            

    def update_global_lower_bound(self):

        for partition in self.partition.partition:
            
            if partition['beta'] < self.global_lb:
                self.global_lb = partition['beta']

                self.global_lb_list.append((self.global_lb, time.time() - self.t))
        
            

class Episode(object):

    def __init__(self, name, model, likelihood=1.0):

        self.name = name
        self.model = model
        self.value = None
        self.risk = None
        self.lagrangian_value = None

        self.likelihood = likelihood
        self.lb = -np.inf
        self.ub = np.inf

        self.solver = CSSPSolver(model, bounds=[5.0], VI_epsilon=1e-1,convergence_epsilon=1e-10)



    def solve(self, bound, only_first_stage=True):

        self.solver = CSSPSolver(self.model, bounds=[bound], VI_epsilon=1e-1,convergence_epsilon=1e-10)

        try:
            self.solver.solve([[0,5]])
        except:
            print("canceled")
        if not only_first_stage:
            self.solver.candidate_pruning = False
            try:
                self.solver.incremental_update(2)
            except:
                print("continue")

        self.value = self.solver.anytime_solutions[-1][0]
        self.risk = self.solver.anytime_solutions[-1][1] + bound
        self.lagrangian_value = self.solver.k_best_solution_set[-1][0]

        print(self.name + " has been planned!")


class Partition(object):

    def __init__(self, episode_set):

        self.episode_set = episode_set

        initial_subset = {'alpha' : np.inf,
                          'beta'  : -np.inf,
                          'bounds': dict()} 

        for episode_name, episode in episode_set.items():
            initial_subset['bounds'][episode_name] = {'lb' : episode.lb,
                                                      'ub' : episode.ub}

        self.partition = [initial_subset]


    def refine_partition(self):

        refining_subset = self.choose_subset()
        refined_subsets = self.refine_subset(refining_subset)

        self.partition.remove(refining_subset)
        self.partition.extend(refined_subsets)

        return refined_subsets


    def choose_subset(self):

        min_val = np.inf
        min_val_subset_idx = None
        k = 0
        for subset in self.partition:
            if subset['alpha'] < min_val:
                min_val = subset['alpha']
                min_val_subset_idx = k

            k += 1
                

        if min_val_subset_idx == None:
            return self.partition[0]
        else:
            return self.partition[min_val_subset_idx]


    def refine_subset(self, subset):

        bounds = subset['bounds']

        new_bounds = []

        for episode_name, bound in bounds.items():

            bound_lower = bound['lb']
            bound_mid   = (bound['lb']+bound['ub'])/2
            bound_upper = bound['ub']


            if len(new_bounds)==0:

                if bound_mid >= self.episode_set[episode_name].lb:
                    new_bounds.append({episode_name: {'lb': bound_lower,
                                                      'ub': bound_mid }})
                else:
                    print("hi")
                    time.sleep(10000)


                if bound_upper >= self.episode_set[episode_name].lb:
                    new_bounds.append({episode_name: {'lb': bound_mid,
                                                      'ub': bound_upper }})
                else:
                    print("hi")
                    time.sleep(10000)

            else:
                new_bounds_prev = new_bounds
                new_bounds = []
                
                for new_bound in new_bounds_prev:
                    new_bound_lower = new_bound.copy()
                    new_bound_upper = new_bound.copy()

                    new_bound_lower[episode_name] = {'lb': bound_lower,
                                                     'ub': bound_mid}
                    new_bounds.append(new_bound_lower)
                    new_bound_upper[episode_name] = {'lb': bound_mid,
                                                     'ub': bound_upper}
                    new_bounds.append(new_bound_upper)


        refined_subsets = []
        for new_bound in new_bounds:

            refined_subsets.append( {'alpha' : np.inf,
                                     'beta'  : np.inf,
                                     'bounds': new_bound} )

        return refined_subsets
