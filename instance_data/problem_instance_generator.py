# MIT License
#
# Copyright (c) 2021 Johan Ludde Wessén
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

from problem_printer import ProblemPrinter

# This script generates data for multi capacity dual arm robot assembly application
# There built in assumptions are given in the paper XYZ.
# In short, if an object is picked with tool it has to be put down using the same tool
# Every pick task that requires suction cup, generates a camera task and a place task, and a press task (to be done in that order)
# Every pick task that requires gripper, generates a place task, and a press task (to be done in that order)
# Press tasks requires empty gripper, since the gripper will be used to "press"
# Components are stacked sequentially on fixture 1 and fixture 2,
# then the sub-assembly of fixture 1 is picked up and merged with the sub-assembly of fixture 2.
# It is then picked up, and moved to output location.

# To fully define a problem one needs to give the tasks, which tool they are handled with,
# which order they are assembled on the two fixtures.
# This is done by providing two lists of numbers.
# One list represents the order of assembly of components on one fixture,
# the value of each number represents which tool handled that component.

# Here, we concatenate these two lists, by intersecting the two lists with a -1.
# Given that 0 is gripper (G), and 1 is suction cup (1), the following string:
# [G,S,-1,S,G] represents a problem,
# where component 1 and 4 are picked by gripper,
# and component 2 amd 3 are handled by suckion cup.
# component 2 is placed atop component 1 on fixture 1,
# and component 4 is placed atop component 3 on fixture 2,
# (followed by the picking of sub-assembly of fixture 1, and placing it on fixture 2, as this is always the case)

# Running this script will generate the problem instances of the paper XYZ

durations = {}
durations["template_pick_duration"] = 27
durations["template_camera_duration"] = 30
durations["template_place_duration"] = 51
durations["template_press_duration"] = 6

file_prefix = "p_"
G = 0
S = 1

fixture_def = []


#4 components
fixture_def.append([G,G,-1,G,G])
fixture_def.append([G,S,-1,S,G])
fixture_def.append([S,G,-1,S,G])
fixture_def.append([S,S,-1,S,S])

#5 components
fixture_def.append([G,G,-1,G,G,G])
fixture_def.append([G,S,-1,S,S,G])
fixture_def.append([G,S,S,-1,S,G])
fixture_def.append([S,S,S,-1,S,S])

#6 components
fixture_def.append([G,G,G,-1,G,G,G])
fixture_def.append([G,S,S,G,-1,S,G])
fixture_def.append([G,S,G,-1,S,S,G])
fixture_def.append([S,S,S,S,-1,S,S])

#7 components
fixture_def.append([G,G,G,-1,G,G,G,G])
fixture_def.append([G,S,G,-1,S,G,S,G])
fixture_def.append([S,G,S,G,-1,G,S,G])
fixture_def.append([S,S,S,S,-1,S,S,S])

#8 components
fixture_def.append([G,G,G,G,-1,G,G,G,G])
fixture_def.append([G,S,G,S,G,-1,S,G,S])
fixture_def.append([S,G,S,G,-1,S,G,S,G])
fixture_def.append([S,S,S,S,S,-1,S,S,S])

#9 components
fixture_def.append([G,G,G,G,-1,G,G,G,G,G])
fixture_def.append([G,S,G,S,-1,G,S,G,S,G])
fixture_def.append([S,G,S,G,S,-1,G,S,G,S])
fixture_def.append([S,S,S,S,S,-1,S,S,S,S])

#10 components
fixture_def.append([G,G,G,G,G,-1,G,G,G,G,G])
fixture_def.append([G,S,G,S,G,S,-1,G,S,G,S])
fixture_def.append([S,G,S,G,S,-1,G,S,G,S,G])
fixture_def.append([S,S,S,S,S,S,-1,S,S,S,S])


# Give template durations
ppinstance = ProblemPrinter(fixture_def[0], 0, **durations)
ppinstance.FilePrint(file_prefix)

for i in range(1,len(fixture_def)):
    ppinstance = ProblemPrinter(fixture_def[i], i*100)
    print("Generating model file: ",ppinstance.FilePrint(file_prefix))
