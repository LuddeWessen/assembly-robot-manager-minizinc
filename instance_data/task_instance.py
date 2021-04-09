#!/usr/bin/python

import os
import numpy as np
import math
import os.path as path
import random
import csv

from random import seed
from random import randint


class Task:
    pick_duration = -1
    camera_duration = -1
    place_duration = -1
    press_duration = -1

    def __init__(self, id, work_type, location_type, no_agents=2):
        self.id = id
        self.no_agents = no_agents
        self.work_type = work_type #0 pick, 1 camera, 2 place, 3 press, 4 airgun
        self.location_type = location_type #0 tray, 1 camera, 2 fixture, 3 airgun, 4 output
        self.duration = self.GetTaskDuration()*np.ones((no_agents+1), dtype=int)

    # Set default values, based on average of original data
    def GetTaskDuration(self):
        dur = -1
        if self.work_type == 0:
            dur = Task.pick_duration
        elif self.work_type == 1:
            dur = Task.camera_duration
        elif self.work_type == 2:
            dur = Task.place_duration
        elif self.work_type == 3:
            dur = Task.press_duration
        else:
            dur = -2
            print("At undefined work type no " + str(self.work_type) + "!")

        #return [dur,dur,dur] # last is "base"
        return dur # last is "base"

    # Pick up _original_(!) task durations and change to value in uniform discrete distribution.
    # The idea is to simulate different characteristics, and slight variation per arm
    # Thus, first we vary the task, then within arms
    # Provided are task deviation from template, awhich becomes the this task's baseline,
    # Then we perturb each arm deviation from this baseline.
    # This way tasks get different times, and the same arms get different times.
    # We round away from template value, to make sure we get some differentiation
    def RandomizeTaskDuration(self, rnd_seed, variation_lb = 1.0 - 0.12, variation_ub = 1.0 + 0.12, arm_variation_lb=1.0 - 0.12 , arm_variation_ub=1.0 + 0.12):

        # Without proper initialization of a template duration, we will not proceed
        if self.duration[-1] < 0:
            return None

        seed(rnd_seed)

        low_int = max(1,np.floor(variation_lb*self.duration[-1]))
        high_int = max(1,np.ceil(variation_ub*self.duration[-1]))

        new_baseline = randint(low_int, high_int)

        low_int = max(1, np.floor(arm_variation_lb*new_baseline))
        high_int = max(1, np.ceil(arm_variation_ub*new_baseline))

        for i in range(len(self.duration) - 1):
            self.duration[i] = randint(low_int, high_int)


        #self.duration[0] = randint(low_int, high_int)
        #self.duration[1] = randint(low_int, high_int)
