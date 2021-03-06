#!/usr/bin/env python

# author: Sungkweon Hong
# email: sk5050@mit.edu
# Grid model as simple model to test rao star

import math
# import numpy as np
import time
from grid_model import GRIDModel


class AGGModel(object):

    def __init__(self):
        self.init_state='s'

        self.model_map = {'episode_1': GRIDModel(init_state=None, size=None, goal=None,
                                                 map_file="models/evacuation_scenario/episode_1.txt"),
                          'episode_2': GRIDModel(init_state=None, size=None, goal=None,
                                                 map_file="models/evacuation_scenario/episode_2.txt"),
                          'episode_3': GRIDModel(init_state=None, size=None, goal=None,
                                                 map_file="models/evacuation_scenario/episode_3.txt"),
                          'episode_4': GRIDModel(init_state=None, size=None, goal=None,
                                                 map_file="models/evacuation_scenario/episode_4.txt"),
                          'episode_5': GRIDModel(init_state=None, size=None, goal=None,
                                                 map_file="models/evacuation_scenario/episode_5.txt"),
                          'episode_6': GRIDModel(init_state=None, size=None, goal=None,
                                                 map_file="models/evacuation_scenario/episode_6.txt"),
                          'episode_7': GRIDModel(init_state=None, size=None, goal=None,
                                                 map_file="models/evacuation_scenario/episode_7.txt"),
                          'episode_8': GRIDModel(init_state=None, size=None, goal=None,
                                                 map_file="models/evacuation_scenario/episode_8.txt"),
                          'episode_9': GRIDModel(init_state=None, size=None, goal=None,
                                                 map_file="models/evacuation_scenario/episode_9.txt"),
                          'episode_10': GRIDModel(init_state=None, size=None, goal=None,
                                                 map_file="models/evacuation_scenario/episode_10.txt"),
                          'episode_11': GRIDModel(init_state=None, size=None, goal=None,
                                                 map_file="models/evacuation_scenario/episode_11.txt"),
                          'episode_12': GRIDModel(init_state=None, size=None, goal=None,
                                                 map_file="models/evacuation_scenario/episode_12.txt"),
                          'episode_13': GRIDModel(init_state=None, size=None, goal=None,
                                                 map_file="models/evacuation_scenario/episode_13.txt"),
                          'episode_14': GRIDModel(init_state=None, size=None, goal=None,
                                                 map_file="models/evacuation_scenario/episode_14.txt")}

        self.goal_to_event_map = {'episode_1': 't1',
                                  'episode_2': 't2',
                                  'episode_3': 't3',
                                  'episode_4': 't4',
                                  'episode_5': 't5',
                                  'episode_6': 'g',
                                  'episode_7': 't9',
                                  'episode_8': 't6',
                                  'episode_9': 't8',
                                  'episode_10': 't1',
                                  'episode_11': 't9',
                                  'episode_12': 't4',
                                  'episode_13': 't8',
                                  'episode_14': 'g'}
                          
                          

        
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
        elif state=='g':
            return ['stay']
        else:
            sub_model = self.find_model(state)
            return sub_model.actions(state[1])

    
    def is_terminal(self, state):
        return state == "g"

    
    def state_transitions(self, state, action):
        if action=='episode_1':
            return [[('episode_1', self.model_map['episode_1'].init_state),1.0]]
        elif action=='episode_2':
            return [[('episode_2', self.model_map['episode_2'].init_state),1.0]]
        elif action=='episode_3':
            return [[('episode_3', self.model_map['episode_3'].init_state),1.0]]
        elif action=='episode_4':
            return [[('episode_4', self.model_map['episode_4'].init_state),1.0]]
        elif action=='episode_5':
            return [[('episode_5', self.model_map['episode_5'].init_state),1.0]]
        elif action=='episode_6':
            return [[('episode_6', self.model_map['episode_6'].init_state),1.0]]
        elif action=='episode_7':
            return [[('episode_7', self.model_map['episode_7'].init_state),1.0]]
        elif action=='episode_8':
            return [[('episode_8', self.model_map['episode_8'].init_state),1.0]]
        elif action=='episode_9':
            return [[('episode_9', self.model_map['episode_9'].init_state),1.0]]
        elif action=='episode_10':
            return [[('episode_10', self.model_map['episode_10'].init_state),1.0]]
        elif action=='episode_11':
            return [[('episode_11', self.model_map['episode_11'].init_state),1.0]]
        elif action=='episode_12':
            return [[('episode_12', self.model_map['episode_12'].init_state),1.0]]
        elif action=='episode_13':
            return [[('episode_13', self.model_map['episode_13'].init_state),1.0]]
        elif action=='episode_14':
            return [[('episode_14', self.model_map['episode_14'].init_state),1.0]]
        elif action=='open_door_1':
            return [['t7',0.99],['t12',0.01]]
        elif action=='open_door_2':
            return [['t10',0.99],['t11',0.01]]
        else:
            sub_model = self.find_model(state)
            new_state_list = sub_model.state_transitions(state[1], action)

            for new_state in new_state_list:
                new_state[0] = (state[0], new_state[0])
                if sub_model.is_terminal(new_state[0][1]):
                    new_state[0] = self.goal_to_event_map[new_state[0][0]]

            return new_state_list


    def cost(self,state,action):  # cost function should return vector of costs, even though there is a single cost function.
        if action not in ['U','D','L','R']:
            return 0.0, 0.0
        else:
            sub_model = self.find_model(state)
            return sub_model.cost(state[1],action)
            
    
    def heuristic(self, state,depth=None):
        return 0.0, 0.0
        # if action not in ['U','D','L','R']:
        #     return 0.0, 0.0
        # else:
        #     sub_model = self.find_model(state)
        #     return sub_model.heuristic(state[1])


    def find_model(self, state):
        return self.model_map[state[0]]

        
        
