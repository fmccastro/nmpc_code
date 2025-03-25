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

##  Main results (to be updated)
![Example of traversability map generated with optimization methods (namely ipopt).](https://github.com/fmccastro/mscthesis_pdf/blob/7b91f8b9ee3b3ccbd17daf7faa5a1f8c68620eb5/Figures/mapRefinement%2BPoints%2B0.9%2B0.2%2B1.png)

![Example of potential flow generated with the Eikonal equation.](https://github.com/fmccastro/mscthesis_pdf/blob/7b91f8b9ee3b3ccbd17daf7faa5a1f8c68620eb5/Figures/potentialFlow%2BmapRefinement%2BPoints%2B0.9%2B0.2.pdf)
)

![Comparison among paths generated with different types of maps.](https://github.com/fmccastro/mscthesis_pdf/blob/7b91f8b9ee3b3ccbd17daf7faa5a1f8c68620eb5/Figures/comparisonOfPaths.pdf)
