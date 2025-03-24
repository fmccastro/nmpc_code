#!~/.venv/bin/python3
from __future__ import absolute_import
from __future__ import division

import matplotlib.pyplot as plt
import matplotlib.image as mpimg

import matplotlib, skfmm, math, os, pickle

import casadi as ca
import numpy as np

from matplotlib import cm
from scipy.optimize import minimize

from mpl_toolkits.mplot3d import Axes3D

from PIL import Image

import sys
sys.path.insert(0, "/media/fmccastro/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/src")

from classes.common_class import *
from classes.planner_class import *

"""
    This Python script delivers the following contributions:
        -   It converts raw digital elevation maps data into .npy matrix
        -   It publishes the elevation maps on .png format for publication
        -   It generates traversability maps of several types under the guidelines presented in the master thesis document
        -   It publishes each traversability map on .png format for publication

    This script only works under the framework of a Common() and Planner() class. An independent script shall be created to enable a global use.
"""

if __name__ == '__main__':

    #   Call instance of Common class to use robot kinematic and geometric data
    common = Common()

    #   Call instance of Planner class to call cost map generation and path planning algorithms
    planner = Planner()

    """
        Option 1 simulation options

        option1:    HeightMin   ;   maxTerrainRoughness     ;   HeightMin_maxInclination
    """
    option1 = "maxTerrainRoughness"

    """
        Option 2 simulation options

        option2:    NO_InitialGuess ; With_InitialGuess_X0 ; With_InitialGuess_X0_LAM_X_LAM_G
    """
    option2 = "With_InitialGuess_X0"

    """
        Option 3 model robot with points or surface
        
        option3:    Points  ;   Surface
    """
    option3 = "Points"

    """
        Option 4 simulation options
        
        option4:    Map1    ;   Map2
    """
    option4 = "Map1"

    """
        Load heightmap from original file
    """
    fromOrigin = True
    
    print("[" + os.path.basename(__file__) + "]" + "Create, scale, save and export elevation map for publication.\n")
    planner._createElevationMap(fromOrigin)
    
    #   Compute terrain traversability according to specs
    start = time.time()
    
    print("[" + os.path.basename(__file__) + "]" + " Build traversability map with options: " + "( " + option1 + " ; " + option2 + " ; " + option3 + " ; " + str(common.mapFolder) + " )")
    results = planner._terrainTraversability(option1, option2, option3, 0, thetaDiv = 5, yDiv = 1)
    
    end = time.time() - start

    results["totalTime"] = end

    print("[" + os.path.basename(__file__) + "]" +  "Computation time of traversability map: " + str(end))

    #   Save results to file
    if( option1 == "maxTerrainRoughness" ):
        f = json.dumps(results)
        with open( common.results_folder + "Traversability_maps/" + option1 + "+" + str(common.mapFolder) + '.json', 'w') as handle:
            handle.write(f)
            handle.close()

    else:
        f = json.dumps(results)
        with open( common.results_folder + "Traversability_maps/" + common.optSolver + "+" + option1 + "+" + option2 + "+" + option3 + "+" + str(common.mapFolder) + '.json', 'w') as handle:
            handle.write(f)
            handle.close()