#!/usr/bin/env python

# author: Sungkweon Hong
# email: sk5050@mit.edu
# Grid model as simple model to test rao star

import math
# import numpy as np
import time

class GRIDModel(object):

    def __init__(self, size=(3,3),init_state=(0,0), goal=(4,4), prob_right_transition=0.9, map_file=None):

        # grid with size (x,y): width is x and height is y. index start from 0.
        # example of (3,3)
        # _____________
        # |0,2|1,2|2,2|
        # |___|___|___|
        # |0,1|1,1|2,1|
        # |___|___|___|
        # |0,0|1,0|2,0|
        # |___|___|___|
        #

        if map_file != None:
            self.initialize_model(map_file)

        else:
            self.init_state=init_state
            self.size = size
            self.state_list = []
            self.goal = goal

            for i in range(size[0]):
                for j in range(size[1]):
                    self.state_list.append((i,j))


        self.prob_right_transition = prob_right_transition                
        self.action_list = ["U","L","R","D"]


    def initialize_model(self, map_file):

        self.state_list = []
        self.constrained_state_list = []

        map_text = open(map_file, 'r')
        lines = map_text.readlines()

        y = 0
        for line in reversed(lines):
            x = 0
            for char in line:
                if char==" ":
                    self.state_list.append((x,y))
                    
                elif char=="s":
                    self.init_state = (x,y)
                    self.state_list.append((x,y))
                    
                elif char=="g":
                    self.goal = (x,y)
                    self.state_list.append((x,y))
                    
                elif char=="x":
                    self.constrained_state_list.append((x,y))
                    self.state_list.append((x,y))

                x += 1

            y += 1        

        
    def actions(self, state):
        return self.action_list

    
    def is_terminal(self, state):
        return state == self.goal

    
    def state_transitions(self, state, action):
        if action=="U":
            new_states_temp = [[(state[0],state[1]+1), self.prob_right_transition],
                               [(state[0]+1,state[1]), (1-self.prob_right_transition)/2],
                               [(state[0]-1,state[1]), (1-self.prob_right_transition)/2]]

        elif action=="D":
            new_states_temp = [[(state[0],state[1]-1), self.prob_right_transition],
                               [(state[0]+1,state[1]), (1-self.prob_right_transition)/2],
                               [(state[0]-1,state[1]), (1-self.prob_right_transition)/2]]

        elif action=="L":
            new_states_temp = [[(state[0]-1,state[1]), self.prob_right_transition],
                               [(state[0],state[1]+1), (1-self.prob_right_transition)/2],
                               [(state[0],state[1]-1), (1-self.prob_right_transition)/2]]

        elif action=="R":
            new_states_temp = [[(state[0]+1,state[1]), self.prob_right_transition],
                               [(state[0],state[1]+1), (1-self.prob_right_transition)/2],
                               [(state[0],state[1]-1), (1-self.prob_right_transition)/2]]

        prob_stay = 0
        new_states = []
        for new_state in new_states_temp:
            if new_state[0] not in self.state_list:
                prob_stay = prob_stay + new_state[1]
            else:
                new_states.append(new_state)

        if prob_stay > 0:
            new_states.append([state, prob_stay])

        return new_states



    def cost(self,state,action):  # cost function should return vector of costs, even though there is a single cost function. 
        cost1 = 1.0
        # if state==(0,4) or state==(2,2) or state==(3,4):
        # if state==(0,4) or state==(2,2) or state==(3,4) or state==(4,1) or state==(4,2) or state==(3,4):
        if state in self.constrained_state_list:
            cost2 = 50.0
        else:
            cost2 = 0.0

        return cost1, cost2

    
    def heuristic(self, state,depth=None):
        heuristic1 = math.sqrt((state[0]-self.goal[0])**2 + (state[1]-self.goal[1])**2)

        # if state==(0,4) or state==(2,2) or state==(3,4):
        if state in self.constrained_state_list:
            heuristic2 = 50.0
        else:
            heuristic2 = 0

        return heuristic1, heuristic2



    def print_policy(self,policy):

        print("-"*41)
        for j in [12,11,10,9,8,7,6,5,4,3,2,1,0]:
            string = ""
            for i in [0,1,2,3,4,5,6,7,8,9,10,11,12]:
                string += "|"
                if (i,j) in policy:
                    if policy[(i,j)]=="Terminal":
                        string += " T "
                    else:
                        string += " "+policy[(i,j)]+" "
                else:
                    string += "   "

            string += "|"
            print(string)
            print("-"*41)
