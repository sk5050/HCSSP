#!/usr/bin/env python

# author: Sungkweon Hong
# email: sk5050@mit.edu
# Grid model as simple model to test rao star

import math
# import numpy as np
import time

class PROCEDURALModel(object):

    def __init__(self, episodes_values, lagrangian=False):

        events = ['s',
                  't1', 't2,' 't3', 't4', 't5', 't6',
                  't7', 't8', 't9', 't10', 't11', 't12',
                  'g']
        self.init_state='s'

        self.episodes_values = episodes_values

        self.lagrangian = lagrangian


    def actions(self, state):
        if state=='s':
            return ['episode_1', 'episode_8']
        elif state=='t1':
            return ['episode_2']
        elif state=='t2':
            return ['episode_3', 'episode_4']
        elif state=='t3':
            return ['episode_7']
        elif state=='t4':
            return ['episode_5']
        elif state=='t5':
            return ['episode_6']
        elif state=='t6':
            return ['open_door_1']
        elif state=='t7':
            return ['episode_9', 'episode_11']
        elif state=='t8':
            return ['episode_12']
        elif state=='t9':
            return ['open_door_2']
        elif state=='t10':
            return ['episode_14']
        elif state=='t11':
            return ['episode_13']
        elif state=='t12':
            return ['episode_10']
        

    
    def is_terminal(self, state):
        return state == "g"

    
    def state_transitions(self, state, action):
        if action=='episode_1':
            return [['t1',1.0]]
        elif action=='episode_2':
            return [['t2',1.0]]
        elif action=='episode_3':
            return [['t3',1.0]]
        elif action=='episode_4':
            return [['t4',1.0]]
        elif action=='episode_5':
            return [['t5',1.0]]
        elif action=='episode_6':
            return [['g',1.0]]
        elif action=='episode_7':
            return [['t9',1.0]]
        elif action=='episode_8':
            return [['t6',1.0]]
        elif action=='episode_9':
            return [['t8',1.0]]
        elif action=='episode_10':
            return [['t1',1.0]]
        elif action=='episode_11':
            return [['t9',1.0]]
        elif action=='episode_12':
            return [['t4',1.0]]
        elif action=='episode_13':
            return [['t8',1.0]]
        elif action=='episode_14':
            return [['g',1.0]]
        elif action=='open_door_1':
            return [['t7',0.99],['t12',0.01]]
        elif action=='open_door_2':
            return [['t10',0.99],['t11',0.01]]




    def cost(self,state,action):  # cost function should return vector of costs, even though there is a single cost function.
        if action in self.episodes_values:
            if self.lagrangian == True:
                return self.episodes_values[action][2], self.episodes_values[action][3]
            else:
                return self.episodes_values[action][0], self.episodes_values[action][1]
        else:
            return 0.0, 0.0

    
    def heuristic(self, state,depth=None):
        return 0.0, 0.0

