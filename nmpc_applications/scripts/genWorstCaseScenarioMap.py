#!/usr/bin/python3
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
        -   It generates worst-case maps and its refinement of the maps generated with the /genTraversabilityMaps.py script under the guidelines presented in the master thesis document
        -   It publishes each traversability map on .png format for publication

    This script only works under the framework of a Common() and Planner() class. An independent script shall be created to enable a global use.
"""

if __name__ == '__main__':

    #   Call instance of Common class to use robot kinematic and geometric data
    common = Common()
    
    #   Call instance of Planner class to call cost map generation and path planning algorithms
    planner = Planner()

    """
        Option 3 model robot with points or surface

        option3:    Points  ;   Surface
    """
    option3 = "Points"

    """
        Parameters of map refinement

        alpha:  alpha
        radius:   radius
    """
    alpha = 0.9
    radius = 0.3

    """
        Load heightmap from original file
    """
    fromOrigin = False
    
    print("[" + os.path.basename(__file__) + "]" + "Create, scale, save and export elevation map for publication.\n")
    planner._createElevationMap(fromOrigin)

    start = time.time()

    print("[" + os.path.basename(__file__) + "]" + " Generate worst-case scenario map.\n")
    _, resultsWorst = planner._worstCaseScenarioMap(option3)

    end = time.time() - start

    resultsWorst["totalTime"] = end

    print("[" + os.path.basename(__file__) + "]" + " Computation time of worst case map: ", end)
    
    start = time.time()

    print("[" + os.path.basename(__file__) + "]" + " Refine worst-case scenario map.\n")
    _, resultsRefine = planner._refineCostMap(alpha, radius, option3, alpha, radius)

    end = time.time() - start

    resultsRefine["totalTime"] = end

    print("[" + os.path.basename(__file__) + "]" + " Computation time of refined map: ", end)
    
    f = json.dumps(resultsWorst)
    with open( common.results_folder + "Traversability_maps/" + "WorstCaseMap" + "+" + option3 + "+" + str(common.mapFolder) + '.json', 'w') as handle:
        handle.write(f)
        handle.close()

    f = json.dumps(resultsRefine)
    with open( common.results_folder + "Traversability_maps/" + "RefinedMap" + "+" + str(alpha) + "+" + str(radius) + "+" + option3 + "+" + str(common.mapFolder) + '.json', 'w') as handle:
        handle.write(f)
        handle.close()