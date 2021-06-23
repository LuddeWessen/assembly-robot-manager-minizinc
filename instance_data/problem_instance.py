# MIT License
#
# Copyright (c) 2021 Johan Ludde Wessén
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
    
import numpy as np

from task_instance import Task
from pnp_instance import PicknPlaceSequence


class Problem:

    #def __init__(self, fixture_order_raw, template_pick_duration=270, template_camera_duration=300, template_place_duration=510, template_press_duration=50, **kwargs):
    def __init__(self, fixture_order_raw, **kwargs):

        # It looks "volontary to provide the elements of kwargs, but without
        # these elements these durations become negative, and the model becomes UNSAT
        if "template_pick_duration" in kwargs:
            Task.pick_duration = kwargs["template_pick_duration"]

        if "template_camera_duration" in kwargs:
            Task.camera_duration = kwargs["template_camera_duration"]

        if "template_place_duration" in kwargs:
            Task.place_duration = kwargs["template_place_duration"]

        if "template_press_duration" in kwargs:
            Task.press_duration = kwargs["template_press_duration"]


        #Task.pick_duration = template_pick_duration
        #Task.camera_duration = template_camera_duration
        #Task.place_duration = template_place_duration
        #Task.press_duration = template_press_duration

        # Tasks will be created, and grip orders and suction orders will be created
        # Warning: no error handling, we assume the user does nothing outside specifications :-o

        # Here we have order (with/without # of each pick method, which is redundant anyway)

        n_g = 0
        n_s = 0
        for value in fixture_order_raw:
            if value == 0:
                n_g += 1
            elif value == 1:
                n_s += 1

        self.no_grip = n_g
        self.no_suction = n_s

        self.no_camera = n_s

        self.pick_n_place_list = []

        # Translating the fixture order matrix from the input
        self.fixture_component_order = -np.ones((2,len(fixture_order_raw)+2), dtype="int_")

        # We create the order of COMPONENTS on fixture (not tasks), using the input list as blueprint
        # The actual fixture tasks are corresponding place and press tasks, and comes later
        g_id = 0
        s_id = self.no_grip
        f_ind = 0
        f_order = 0
        for i in range(len(fixture_order_raw)):
            if fixture_order_raw[i] == 0:
                self.fixture_component_order[f_ind, f_order] = g_id
                g_id += 1
                f_order += 1
            elif fixture_order_raw[i] == 1:
                self.fixture_component_order[f_ind, f_order] = s_id
                s_id += 1
                f_order += 1
            elif fixture_order_raw[i] == -1:
                f_ind += 1
                f_order = 0
            else:
                print("Should not be here! fixture_order_raw contains other than -1,0,1")
                print( fixture_order_raw[i])

        # Generate task in a prescibed order
        # Grip picks, suction picks, fixture to fixture pick, fixture to output pick
        task_id = 1

        #Grip pick
        for i in range(self.no_grip):
            self.pick_n_place_list.append(PicknPlaceSequence(task_id, 0))
            task_id += 1

        #Suction pick
        for i in range(self.no_suction):
            self.pick_n_place_list.append(PicknPlaceSequence(task_id, 1))
            task_id += 1

        # Fixture to fixture pnp
        self.pick_n_place_list.append(PicknPlaceSequence(task_id, 2))
        task_id += 1

        # Fixture to out pnp
        self.pick_n_place_list.append(PicknPlaceSequence(task_id, 3))
        task_id += 1


        # Continue to generate task in a prescibed order by adding to sequence:
        # Grip place, suction place, camera, press

        # Fill the pnp-objects
        no_objs = len(self.pick_n_place_list)

        # Place tasks:
        # Note: due to order of pick generation above, this will first create
        # place task for components handled by gripper, then for suction
        # Sub assembly or assembly is not included, it comes later
        for i in range(no_objs):
            if (self.pick_n_place_list[i].GenPlaceTask(task_id)):
                task_id += 1

        # Camera tasks: (only adds if this return true, which happens when pick was done with suction cup)
        for i in range(no_objs):
            if (self.pick_n_place_list[i].GenCameraTask(task_id)):
                task_id += 1

        # Press tasks:
        for i in range(no_objs):
            if (self.pick_n_place_list[i].GenPressTask(task_id)):
                task_id += 1

        #for i in range(no_objs):
        #    print(self.pick_n_place_list[i])
        #    print("")

        self.no_tasks = task_id - 1



    def TrayTasksToString(self):
        return self.GetSetString(self.GetTrayTasks())

    def CameraTasksToString(self):
        return self.GetSetString(self.GetCameraTasks())

    def PressTasksToString(self):
        return self.GetSetString(self.GetPressTasks())

    def OutputTasksToString(self):
        return self.GetSetString(self.GetOutputTasks())


    def GetTrayTasks(self):
        #tasks = set([])
        tasks = []
        for pnp in self.pick_n_place_list:
            if pnp.type == 0 or pnp.type == 1 : #pick tray (grip or suction)
                #tasks.add(pnp.pick_task)
                tasks.append(pnp.pick_task)

        return tasks

    def GetCameraTasks(self):
        #tasks = set([])
        tasks = []
        for pnp in self.pick_n_place_list:
            if pnp.type == 1 : #pick tray (grip or suction)
                #tasks.add(pnp.camera_task)
                tasks.append(pnp.camera_task)

        return tasks

    def GetPressTasks(self):
        #tasks = set([])
        tasks = []
        for pnp in self.pick_n_place_list:
            if not (pnp.press_task is None) : #press task
                #tasks.add(pnp.press_task)
                tasks.append(pnp.press_task)

        return tasks

    def GetOutputTasks(self):
        #tasks = set([])
        tasks = []
        for pnp in self.pick_n_place_list:
            if not (pnp.place_task is None) : # place task
                if pnp.place_task.location_type == 4: # output location
                    #tasks.add(pnp.place_task)
                    tasks.append(pnp.place_task)


        return tasks


    def GetItemizedSetString(self, the_collection):
        s = "{ "
        if len(the_collection) > 0:
            for value in the_collection:
                s = s + str(int(value.id))
                s = s + ", "

            s = s[:-2] #remove last ", "
        s = s + " }"

        return s

    # Check if set is contigous
    def IsContiguous(self, the_collection):
        min_val = 100000
        max_val = -1
        for item in the_collection:
            min_val = min(min_val, item.id)
            max_val = max(min_val, item.id)

        if len(the_collection) > 1: # empty and with 1 element is not considered contiguous
            for i in range(len(the_collection)):
                if the_collection[i].id != min_val + i:
                    return [False, -1,-1]
        else:
            return [False, -1,-1]

        return [True, min_val, max_val]

    # Get string representation of minizinc set .
    # For readability, if possible we will use contiguous set
    def GetSetString(self, the_collection):
        isc_min_max = self.IsContiguous(the_collection)
        if isc_min_max[0]:
            return str(isc_min_max[1]) + ".." + str(isc_min_max[2])
        else:
            return self.GetItemizedSetString(the_collection)


    def GetSetDurations(self, the_collection, ):
        s = "{ "
        if len(the_collection) > 0:
            for value in the_collection:
                s = s + str(int(value.duration))
                s = s + ", "

            s = s[:-2] #remove last ", "
        s = s + " }"

        return s

    def GetDurationsOfTasksString(self, no_agents = 2, str_offset=0):

        lot = self.GetListOfTasks()

        max_val_len = 0
        for t in range(len(lot)):
            max_val_len = max(max_val_len, max(len(str(lot[t].duration[0])), len(str(lot[t].duration[1]))))

        code_str = '{:>'+str(max_val_len+2)+'s}'

        s = "["
        for a in range(no_agents):
            s = s + "|"
            for t in range(len(lot)):
                s = s + code_str.format(str(lot[t].duration[a]) + ", ")

            s = s[:-2] + "\n"
            for i in range(str_offset):
                s = s + " "


        s = s + "|];"
        return s

    # Pick up _original_(!) task durations and change to value in uniform discrete distribution.
    # This becomes the new baseline, which is then randomized to differentiate between arms (max +/- 25%)
    def RandomizeTaskDurations(self, rnd_seed):

        lot = self.GetListOfTasks()

        # Go through each task to randomize per task and agent
        for t in range(len(lot)):
            lot[t].RandomizeTaskDuration(rnd_seed)
            rnd_seed += 10

        return rnd_seed


    # This method returns the tasks of the problem instance, in the order of index id
    def GetListOfTasks(self):

        l = np.empty((self.no_tasks), dtype=object)
        for i in range(self.no_tasks):
            l[i] = None

        for pnp in self.pick_n_place_list:
            for item in pnp.GetTaskSet():
                if l[item.id-1] is None:
                    l[item.id-1] = item
                else:
                    print("Should not be here - item appears more than once. Tasks:")
                    print(l[item.id-1])
                    print(item)
                    print("--------------------------------------------------------")

        for i in range(self.no_tasks):
            if l[i] is None:
                print("Should not be here - index lacks task:")
                print(i)
                print("--------------------------------------------------------")

        return l


    def GetTask(self, id):
        l = self.GetListOfTasks()
        return l[id-1]

    # This method returns the tasks of pick and pick-and-place tasks , and pads with -1
    def GetPickTaskOrderString(self, tool):

        max_length = 0
        has_elem = False
        for item in self.pick_n_place_list:
            if item.tool == tool:
                max_length = max(max_length, item.GetPickLength())
                has_elem = True

        if not has_elem:
            return "[||]"
        else:
            s = "["

            for item in self.pick_n_place_list:
                if item.tool == tool:
                    s = s + "|" + item.GetPickTaskOrderString(max_length + 1) + "\n"

            s = s + "|]"
            return s

    # This method returns the tasks of fixture orders, and pads with -1
    def GetFixtureTaskOrderString(self):
        fco_shape = np.shape(self.fixture_component_order)

        # Calculate number of components per fixture
        counter = np.zeros(fco_shape[0], dtype="int_")
        for a in range(fco_shape[0]):
            for t in range(fco_shape[1]):
                if self.fixture_component_order[a,t] >= 0:
                    counter[a] +=1

        # Since each component means place + press task,
        # we double the number of components to get the number of tasks per fixture
        counter *= 2

        # We pick the subassembly from the first fixture (+1 task)
        counter[0] += 1

        # We place and press the subassembly from the second fixture,
        # and
        # we pick the assembly from the second fixture (tot +3 tasks)
        counter[1] += 3
        max_no = max(counter)+1 # pad with a -1

        s = "[|"

        for a in range(fco_shape[0]):
            for t in range(fco_shape[1]):
                pnp_ind = self.fixture_component_order[a,t]
                if pnp_ind >= 0:
                    s = s + self.pick_n_place_list[pnp_ind].GetFixtureTaskOrderSubString() + ", "

            # Add the pick-of-subassembly-from-fixture1, or the place of said subassembly on fixture 2, followed by press and pick out
            if a == 0:
                s = s + str(int(self.pick_n_place_list[-2].pick_task.id)) + ", "
            else:
                s = s + str(int(self.pick_n_place_list[-2].place_task.id)) + ", "
                s = s + str(int(self.pick_n_place_list[-2].press_task.id)) + ", "
                s = s + str(int(self.pick_n_place_list[-1].pick_task.id)) + ", "

            for t in range(counter[a], max_no):
                s = s + "-1, "

            s = s[:-2] + "\n|"

        s = s + "]"
        return s
