# MIT License
#
# Copyright (c) 2021 Johan Ludde Wessén
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

    from task_instance import Task

# This class holds a pick-and-place sequence, and is initialized by creating the pick task
# Subsequently, the place sequence
class PicknPlaceSequence:

    #object_id is unique int identifier
    # Type: 0 grip component pick, 1 suction component pick 2 fixture-to-fixture 3 fixture to output
    def __init__(self, object_id, type):
        self.id = object_id # pick task id
        self.type = type

        if type == 0 or type == 2 or type == 3:
            self.tool = 0
        elif type == 1:
            self.tool = 1
        else:
            self.tool = -1

        # Create the pick task at either tray (0) or fixture (2)
        if type == 0 or type == 1:
            self.pick_task = Task(object_id, 0, 0)
        elif type == 2:
            self.pick_task = Task(object_id, 0, 2)
        elif type == 3:
            self.pick_task = Task(object_id, 0, 2)

        # Initialize extra tasks as None
        self.camera_task = None
        self.intermediate_holding_task = None
        self.place_task = None
        self.press_task = None
        self.final_post_processing = None


    def GenPlaceTask(self, id):
        #0 tray, 1 camera, 2 fixture, 3 airgun, 4 output
        if self.type == 0 or self.type == 1 or self.type == 2:
            self.place_task = Task(id, 2, 2)
            return True
        elif self.type == 3: #fixture-to-output
            self.place_task = Task(id, 2, 4)
            return True

        return False


    def GenCameraTask(self, id):
        #0 tray, 1 camera, 2 fixture, 3 airgun, 4 output
        if self.type == 1: #suction pick
            self.camera_task = Task(id, 1, 1)
            return True

        return False

    def GenPressTask(self, id):
        #0 tray, 1 camera, 2 fixture, 3 airgun, 4 output
        if self.type == 0 or self.type == 1 or self.type == 2 or self.type == 4:
            self.press_task = Task(id, 3, 2)
            return True

        return False

    def GetTaskSet(self):
        ts = set([])
        if self.NotNone(self.pick_task):
            ts.add(self.pick_task)

        if self.NotNone(self.camera_task):
            ts.add(self.camera_task)

        if self.NotNone(self.intermediate_holding_task):
            ts.add(self.intermediate_holding_task)

        if self.NotNone(self.place_task):
            ts.add(self.place_task)

        if self.NotNone(self.press_task):
            ts.add(self.press_task)

        if self.NotNone(self.final_post_processing):
            ts.add(self.final_post_processing)

        return ts


    def GetPickLength(self):
        n = 0
        if self.NotNone(self.pick_task):
            n += 1

        if self.NotNone(self.camera_task):
            n += 1

        if self.NotNone(self.intermediate_holding_task):
            n += 1

        if self.NotNone(self.place_task):
            n += 1

        return n


    def GetFixtureTaskOrderSubString(self):
        s = ""

        if self.NotNone(self.place_task):
            s = s + str(int(self.place_task.id)) + ", "

        if self.NotNone(self.press_task):
            s = s + str(int(self.press_task.id)) + ", "

        if self.NotNone(self.final_post_processing):
            s = s + str(int(self.final_post_processing.id)) + ", "

        return s[:-2]


    def GetPickTaskOrderStringBase(self):
        s = ""
        if self.NotNone(self.pick_task):
            s = s + str(int(self.pick_task.id)) + ", "

        if self.NotNone(self.camera_task):
            s = s + str(int(self.camera_task.id)) + ", "

        if self.NotNone(self.intermediate_holding_task):
            s = s + str(int(self.intermediate_holding_task.id)) + ", "

        if self.NotNone(self.place_task):
            s = s + str(int(self.place_task.id)) + ", "

        return s[:-2] # return everything except trailing ", "

    # Return the task sequence, filled up with -1 to comply with what the flatzinc compiler wants
    def GetPickTaskOrderString(self, tot_length):
        s = self.GetPickTaskOrderStringBase()

        for i in range(self.GetPickLength(),tot_length):
            s = s + ", -1"

        return s


    def NotNone(self, item):
        if not (item is None):
            return True
        else:
            return False
