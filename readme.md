<!-- PROJECT HEADLINE -->
<br />
<p align="left">

  <h1 align="left">YuMi Scheduler+ - A Dual Arm Robot Assembly Scheduler and Workspace Designer </h1>

</p>

A scheduler to manage a multi tool dual arm robot.

Due to workspace data and assumptions on tool configuration it is a ABB YuMi that is used, but the model is otherwise not bound to any specific robot.
It
* Assigns tasks agents,
* Orders task on agents,
* Assigns start/wait/end times,
* Assigns task locations

This model requires the user to install the <a href="https://www.minizinc.org/software.html">MiniZinc</a> compiler, which provides the Constraint Programming solver Gecode, which is the default solver for this project.
MiniZinc is developed at Monash University in collaboration with Data61 Decision Sciences.

<!-- TABLE OF CONTENTS -->
## Table of Contents

* [About the Project](#about-the-project)
* [Regarding MiniZinc Challenge](#regarding-minizinc-challenge)
* [Getting Started](#getting-started)
  * [Installation](#installation-of-solver)
  * [Running Example](#running-example)
  * [Generating Assembly Instance Data Files Using Python](#generating-assembly-instance-data-files-using-python)
* [General Usage](#general-usage)
* [Running Tests](#running-tests)
* [Contact](#contact)


<!-- ABOUT THE PROJECT -->
## About The Project

It is developed as a part of Johan Ludde Wessén's PhD project at KTH and ABB.

You can use the code by providing it with data files on the appropriate format.
It is also possible to extend the models with new and/or alternative model files.

The choice of modeling language allows for use of many different solvers distributed over many solving technologies, such as Constraint Programming, (Mixed) Integer Programming, SAT, and of course combinations thereof.

The model have been tested using the default solver "Gecode".
To use any other solver backend than Gecode, one must remove the "include "gecode.mzn";" line in the YuMiScheduler.mzn file.

<!-- Regarding MiniZinc Challenge -->
## Regarding MiniZinc Challenge

If you would like this model properly annotated for MiniZinc challenge, and unnecessary python scripts removed, please use the sub-folder [minizinc-challenge](https://github.com/LuddeWessen/assembly-robot-manager-minizinc/tree/main/minizinc-challenge)


<!-- GETTING STARTED -->
## Getting Started

To use the YuMiScheduler follow these simple steps.

### Installation of Solver

The recommended way to install _MiniZinc_ is by the use of the bundled binary
packages. These packages are available for machines running Linux, Mac, and
Windows.

The latest release can be found on [the MiniZinc
website](http://www.minizinc.org/software.html).

Once the MiniZinc bundle is installed on your machine, you can start using it.
For the FleetManager to work you need to have the `minizinc` executable available on your path.

### Running Example

Static collision avoidance:
 - With the `minizinc` executable available on your path: run `minizinc --solver Gecode YuMiScheduler.mzn collision_avoidance_resource_zones.mzn instance_data/example_instance_4_GS_SG.dzn robot_cell_data/yumi_grid_setup_3_3.dzn robot_cell_data/yumi_grid_setup_3_3_zones.dzn -s -a
`

Dynamic (spatio-temporal) collision avoidance:
- With the `minizinc` executable available on your path: run `minizinc --solver Gecode YuMiScheduler.mzn collision_avoidance_static_linear.mzn instance_data/example_instance_4_GS_SG.dzn robot_cell_data/yumi_grid_setup_3_3.dzn -s -a
`

### Generating Assembly Instance Data Files Using Python

To generate data files use the "problem_instance_generator.py" script in the "instance_data" folder as a template. Or simply run the script as-is to get the data files encoded within.

## General Usage

Any MiniZinc model needs model file (ends with .mzn). Typically we separate the data instances and put that into data files (ends with .dzn).

The combination `--solver Gecode` tells minizinc to use Gecode as solver.
The flags `-s` and `-a` tells the solver to output statistics after finishing the computations, and output all solutions, respectively.

There are many options to be found in the MiniZinc handbok, but a particularly useful command is `-t time-in-milliseconds` for instances when we cannot wait for all solutions.

### Model Files, or Model Modules

We separate the model into separate files, as we see these as separable modules. We hope to create alternatives to the last modules depending on assumptions during development.
The next to last is a separable module defining constraint on the locations, as there will be _some_ constraints on locations depending on the tasks.

In this case, we have separated the model into the following files, or modules:
* `YuMiScheduler.mzn` is the core model
* `collision_avoidance_static_linear.mzn` models collision avoidance by statically dividing the workspace, assigning each division to an arm
* `collision_avoidance_resource_zones.mzn` models collision avoidance as spatio-temporally booking (scheduling) of user provided zones, by the arms.

### Data Files

In the example we have separated the data into several files:
* `instance_data/*.dzn` - the assembly instance, i.e. definition of what to be done
* `workspace_data/XYZ.dzn` and `workspace_data/XYZ_zones.dzn`, where XYZ is the workspace generated separately.
* `workspace_data/XYZ_zones.dzn`, collision information of zones of the workspace generated separately. This is only needed if using the spatio-temporal collision avoidance model.


<!-- Tests -->
## Running Tests

We have not designed tests for the python code. This work for the future.


<!-- CONTACT -->
## Contact

🏛 Website: [https://www.kth.se/profile/jlwessen](https://www.kth.se/profile/jlwessen)
