# MSc thesis code

This repo contains the code that supports the development of a master thesis on the development of a NMPC based method to control a wheeled mobile robot on unknown environments. The code is based on the intercomunication between ROS (Noetic) nodes. Each node runs a specific task. Python3.8 is the code based language.

##	Authors
1. Francisco Castro
2. Rodrigo Ventura (supervisor)

##	Content
1. [nmpc_applications] (https://github.com/fmccastro/nmpc_code/tree/e8f5333dbb2cd314b467cad49bbd68ebdd4cfb32/nmpc_applications) : location of ROS nodes and python classes
2. nmpc_bringup : location of launch files
3. nmpc_description : location of URDF files that define the mobile robot
4. nmpc_gazebo : location of heightmap files to render in Gazebo 11.

##  Main results (to be updated)
![Example of traversability map generated with optimization methods (namely ipopt).](https://github.com/fmccastro/mscthesis_pdf/blob/7b91f8b9ee3b3ccbd17daf7faa5a1f8c68620eb5/Figures/mapRefinement%2BPoints%2B0.9%2B0.2%2B1.png
)

![Example of potential flow generated with the Eikonal equation.](https://github.com/fmccastro/mscthesis_pdf/blob/7b91f8b9ee3b3ccbd17daf7faa5a1f8c68620eb5/Figures/potentialFlow%2BmapRefinement%2BPoints%2B0.9%2B0.2.pdf
)

![Comparison among paths generated with different types of maps.](https://github.com/fmccastro/mscthesis_pdf/blob/7b91f8b9ee3b3ccbd17daf7faa5a1f8c68620eb5/Figures/comparisonOfPaths.pdf)
