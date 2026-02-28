# MSc thesis code

This repo contains the code that supports the development of a master thesis on the development of a **real-time nonlinear model predictive control based method to safely guide and control a wheeled mobile robot on rough, uneven terrain**. **Optimal path planning** methods are also explored. It is intended to provide a fast review of the usefulness of this work to researchers.

The [acados](https://github.com/acados/acados) library provided key methods to effectively deliver control algorithms that meet stringent real-time requirements. On the other hand, [CasADi](https://github.com/casadi/casadi) provided efficient symbolic nonlinear constrained optimization methods to generate optimal path planning algorithms.

Python3.8 is the code based language.

##	Authors
1. Francisco Castro
2. Prof. Rodrigo Ventura (supervisor)

##	Content
The structure of this repo is similar to a ROS package as the first intention of this work was to make use of ROS-Noetic and Gazebo to output results. Due to infeasibililty of the Gazebo physics-engine to appropriately simulate the traversal of a wheeled mobile robot on rough, rigid terrain, the author resorted to Python simulation in order to achieve the work's main goal. Then, the main code files of the repo are outlined as follows:

1. [Configuration files](https://github.com/fmccastro/nmpc_code/tree/0f28a1b68c4998d8ebb971e4df233e82e0493463/nmpc_applications/config):
   *These files include the robot specs, the cost function weights, model constraints and solver specs. The obstacles shape file is also defined here.*
   
2. [Simulation scripts](https://github.com/fmccastro/nmpc_code/tree/c45579955c95297d7a88256a15d47549140c6684/nmpc_applications/scripts): *This folder contains the scripts that yielded the results presented in the thesis. The files with no folder attached generated the optimal path planning results. Then, the folder [Real-time one stage OCP on rough terrain](https://github.com/fmccastro/nmpc_code/tree/c45579955c95297d7a88256a15d47549140c6684/nmpc_applications/scripts/Real-time%20one%20stage%20OCP%20on%20rough%20terrain) contains the scripts that simulate the controller reference tracking on rough terrain with and without obstacles. The folder [Obstacle avoidance](https://github.com/fmccastro/nmpc_code/tree/c45579955c95297d7a88256a15d47549140c6684/nmpc_applications/scripts/Obstacle%20avoidance) contains the test files that generate the artificial potential field and check the proper implementation of the CIAO algorithm.*

##  Main results (to be updated)
1. Generation of traversability maps
![Example of traversability map generated with optimization methods (namely ipopt).](https://github.com/fmccastro/mscthesis_pdf/blob/7b91f8b9ee3b3ccbd17daf7faa5a1f8c68620eb5/Figures/mapRefinement%2BPoints%2B0.9%2B0.2%2B1.png)

2. Generation of potential flows by applying the Eikonal equation (solution computed with ![skfmm](https://github.com/scikit-fmm/scikit-fmm.git))

	-![Example of potential flow generated with the Eikonal equation.](https://github.com/fmccastro/mscthesis_pdf/blob/7b91f8b9ee3b3ccbd17daf7faa5a1f8c68620eb5/Figures/potentialFlow%2BmapRefinement%2BPoints%2B0.9%2B0.2.pdf)

	-![Comparison among paths generated with different types of maps.](https://github.com/fmccastro/mscthesis_pdf/blob/7b91f8b9ee3b3ccbd17daf7faa5a1f8c68620eb5/Figures/comparisonOfPaths.pdf)

##	Teleoperation node
A teleoperation node for the [mobile robot](https://github.com/fmccastro/nmpc_code/blob/ab60baa7eb3b822d2609c6c4235bec53f61a5c24/nmpc_description/robots/pioneer3at/urdf/pioneer3at.urdf) is coded [here](https://github.com/fmccastro/nmpc_code/blob/913261a1084ce2b29de2d6007a0622224d8becd1/nmpc_applications/src/mouse_joy_wheelTorques.py). You can take the code and adjust to your needs. This approach allows you to easily teleoperate a wheeled mobile robot from a PyGame GUI by mouse control, resembling a joystick.
This first teleoperation node controls wheel torques, whereas this second [teleoperation node](https://github.com/fmccastro/nmpc_code/blob/913261a1084ce2b29de2d6007a0622224d8becd1/nmpc_applications/src/mouse_joy_wheelRates.py) controls wheel rates.

[![Wheel torque teleoperation video demonstration](https://raw.githubusercontent.com/fmccastro/nmpc_code/master/assets/thumbnail.jpg)](https://youtu.be/Sgt95OHfLIY)
