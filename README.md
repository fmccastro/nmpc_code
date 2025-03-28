# MSc thesis code

This repo contains the code that supports the development of a master thesis on the development of a NMPC based method to control a wheeled mobile robot on unknown environments. The code is based on the intercomunication between ROS (Noetic) nodes. Each node runs a specific task. Python3.8 is the code based language.

##	Authors
1. Francisco Castro
2. Rodrigo Ventura (supervisor)

##	Content
1. [nmpc_applications](https://github.com/fmccastro/nmpc_code/tree/06ed9bd8a37130ad27dbac493a8a8cd28a3f696c/nmpc_applications): location of ROS nodes and python classes
2. [nmpc_bringup](https://github.com/fmccastro/nmpc_code/tree/8ab37a864867297aaf7e6ecfd55b73b3ce023acb/nmpc_bringup): location of launch files
3. [nmpc_description](https://github.com/fmccastro/nmpc_code/tree/8ab37a864867297aaf7e6ecfd55b73b3ce023acb/nmpc_description): location of URDF files that define the mobile robot
4. [nmpc_gazebo](https://github.com/fmccastro/nmpc_code/tree/8ab37a864867297aaf7e6ecfd55b73b3ce023acb/nmpc_gazebo): location of heightmap files to render in Gazebo 11.

##	Teleoperation node
A teleoperation node for the [mobile robot](https://github.com/fmccastro/nmpc_code/blob/ab60baa7eb3b822d2609c6c4235bec53f61a5c24/nmpc_description/robots/pioneer3at/urdf/pioneer3at.urdf) is coded [here](https://github.com/fmccastro/nmpc_code/blob/913261a1084ce2b29de2d6007a0622224d8becd1/nmpc_applications/src/mouse_joy_wheelTorques.py). You can take the code and adjust to your needs. This approach allow you to easily teleoperate a wheeled mobile robot from a pyGame Gui by mouse control, resembling a joystick.
This first teleoperation node controls wheel torques, whereas this second teleoperation node is coded [here](https://github.com/fmccastro/nmpc_code/blob/913261a1084ce2b29de2d6007a0622224d8becd1/nmpc_applications/src/mouse_joy_wheelRates.py).

##  Main results (to be updated)
1. Generation of traversability maps
![Example of traversability map generated with optimization methods (namely ipopt).](https://github.com/fmccastro/mscthesis_pdf/blob/7b91f8b9ee3b3ccbd17daf7faa5a1f8c68620eb5/Figures/mapRefinement%2BPoints%2B0.9%2B0.2%2B1.png)

2. Generation of potential flows by applying the Eikonal equation (solution computed with ![skfmm](https://github.com/scikit-fmm/scikit-fmm.git))

	-![Example of potential flow generated with the Eikonal equation.](https://github.com/fmccastro/mscthesis_pdf/blob/7b91f8b9ee3b3ccbd17daf7faa5a1f8c68620eb5/Figures/potentialFlow%2BmapRefinement%2BPoints%2B0.9%2B0.2.pdf)
)

	-![Comparison among paths generated with different types of maps.](https://github.com/fmccastro/mscthesis_pdf/blob/7b91f8b9ee3b3ccbd17daf7faa5a1f8c68620eb5/Figures/comparisonOfPaths.pdf)
