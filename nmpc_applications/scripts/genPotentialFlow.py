#!~/.venv/bin/python3

import sys
sys.path.insert(0, "/media/fmccastro/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/src")

from classes.common_class import *
from classes.planner_class import *

"""
    This Python script delivers the following contributions:
        -   It generates potential flow for each traversability map generated on the /genWorstCaseScenarioMap.py script
        -   It applies the Eikonal equation to generate each potential flow

    This script only works under the framework of a Common() and Planner() class. An independent script shall be created to enable a global use.
"""

if __name__ == '__main__':

    #   Call instance of Common class to use robot kinematic and geometric data
    common = Common()

    #   Call instance of Planner class to call cost map generation and path planning algorithms
    planner = Planner()

    #   Load functions from Planner class defined translated to C for real-time use
    planner_c = ctypes.CDLL("./src/nmpc_applications/src/classes/planner.so")

    #   define struct arguments in ctypes
    class SETTINGS(ctypes.Structure):
        _fields_ = [("nrows", ctypes.c_int), ("ncols", ctypes.c_int), ("maxCycles", ctypes.c_int),\
                    ("startingPoint", ctypes.c_double * 6), ("goalPoint", ctypes.c_double * 2),\
                    ("maskValue", ctypes.c_double), ("pathGap", ctypes.c_double), ("goalCheck", ctypes.c_double)]

    planner_c._getPath2Follow.argtypes = [ctypes.POINTER(ctypes.c_double), SETTINGS]

    #   Define class Position from planner.c in Python
    class POSITION(ctypes.Structure):
        pass
    
    POSITION._fields_ = [ ("x", ctypes.c_double), ("y", ctypes.c_double), ("z", ctypes.c_double),\
                          ("roll", ctypes.c_double), ("pitch", ctypes.c_double), ("yaw", ctypes.c_double),\
                          ("next", ctypes.POINTER(POSITION)) ]

    planner_c._getPath2Follow.restype = ctypes.POINTER(POSITION)

    planner_c._freePath.argtypes = [ctypes.POINTER(POSITION)]
    #############################################################################################################################################

    """
        Option 1 simulation options

        option1:    worstCase   ;   mapRefinement
    """
    option1 = "mapRefinement"

    """
        Option 3 model robot with points or surface
        
        option3:    Points  ;   Surface
    """
    option3 = "Points"

    """
        Option 4 simulation options
        
        option4:    1   ;   2
    """
    option4 = "1"

    """
        Parameters of map refinement

        alpha:  alpha
        radius:   radius
    """
    alpha = 0.9
    radius = 0.2

    """
        Load heightmap from original file
    """
    fromOrigin = True

    elevationMap = planner._loadHeightmap(False)

    #   Set parameters of speed map computation function ( e^( ln(a) / b) * x ) )
    b = 0.4
    a = 0.2
    t1 = 0.6                        #   Set minimum untraversable cost
    maskValue = 0.0

    """
        Plot speed map for the chosen specs (comment if needed)
    """

    #   + Start
    maskedSpeedMap, maskedPotentialFlow = planner._getSpeedMap(option1, option3, radius, alpha, maskValue, t1, a, b)
    planner._plotMap(maskedPotentialFlow, verLabel = "Time to goal [s]")
    #planner._export2Publish(maskedSpeedMap, common.exportPlot2PubDir + "speedMap" + "+" + option1 + "+" + option3 + ".png", "Speed [m/s]")
    #   - End

    """
        Plot and export potential flow (comment if needed)
    """

    #   + Start
    """###   Plot levels   ##############################################################
    gradx, grady = np.gradient(maskedPotentialFlow.filled(np.nan))
    magnitude = np.sqrt(gradx**2 + grady**2)

    gradx = np.where(magnitude != 0, gradx / magnitude, 0)
    grady = np.where(magnitude != 0, grady / magnitude, 0)

    #   Plot and export potential flow with gradient vectors
    matplotlib.rcParams.update({
        'font.family': 'sans-serif',
        'font.sans-serif': ["Helvetica"]
    })

    # Define levels in z-axis where we want lines to appear
    levels = np.linspace(5.0, 110, 20)

    xx, yy = planner._getRealMeshgrid(maskedPotentialFlow)

    z_max = maskedPotentialFlow.max()
    z_min = maskedPotentialFlow.min()

    norm = matplotlib.colors.Normalize(vmin = z_min, vmax = z_max)

    fig, ax1 = plt.subplots(layout="constrained")
    m = cm.ScalarMappable(cmap = cm.YlOrBr)
    m.set_array(maskedPotentialFlow)
    cpf = ax1.contourf(xx, yy, maskedPotentialFlow, len(levels), cmap = cm.YlOrBr, norm = norm)

    # Set all level lines to black
    line_colors = ['black' for l in cpf.levels]
    cp = ax1.contour(xx, yy, maskedPotentialFlow, levels=levels, colors=line_colors)
    ax1.clabel(cp, fontsize=10, colors=line_colors)
    clb = fig.colorbar(m, ax = ax1, label="Time to goal [s]")
    ax1.quiver(xx[::10, ::10], yy[::10, ::10], -grady[::10, ::10], gradx[::10, ::10], units='xy', scale=1.2, width = 0.1, color='blue')
    ax1.set_xlabel("x [m]", fontname="Arial")
    ax1.set_ylabel("y [m]", fontname="Arial")
    ax1.set_xticklabels(ax1.get_xticks(), fontname='Arial', fontsize=12)
    ax1.set_yticklabels(ax1.get_yticks(), fontname='Arial', fontsize=12)
    ax1.scatter(common.goalPoint[0], common.goalPoint[1], s=5.0, c='r', marker='o')

    if( option1 == "worstCase" ):
        print("Check if file exists.")
        if os.path.isfile(common.exportPlot2PubDir + "potentialFlow" + "+" + option1 + "+" + option3 + "+" + str(common.mapFolder) + ".pdf"):
            
            print("File exists. File was removed.")
            os.remove(common.exportPlot2PubDir + "potentialFlow" + "+" + option1 + "+" + option3 + "+" + str(common.mapFolder) + ".pdf")
        
        else:
            print("File does not exist.")

        print("Save new file.")
        plt.savefig(common.exportPlot2PubDir + "potentialFlow" + "+" + option1 + "+" + option3 + ".pdf", dpi=50, bbox_inches='tight',transparent=True)

    elif( option1 == "mapRefinement" ):
        print("Check if file exists.")
        if os.path.isfile(common.exportPlot2PubDir + "potentialFlow" + "+" + option1 + "+" + option3 + "+" + str(alpha) + "+" + str(radius) + "+" + str(common.mapFolder) + ".pdf"):
            
            print("File exists. File was removed.")
            os.remove(common.exportPlot2PubDir + "potentialFlow" + "+" + option1 + "+" + option3 + "+" + str(alpha) + "+" + str(radius) + "+" + str(common.mapFolder) + ".pdf")
        
        else:
            print("File does not exist.")

        print("Save new file.")
        plt.savefig(common.exportPlot2PubDir + "potentialFlow" + "+" + option1 + "+" + option3 + "+" + str(alpha) + "+" + str(radius) + "+" + str(common.mapFolder) + ".pdf", dpi=50, bbox_inches='tight',transparent=True)

    print("\n")
    
    #plt.show()
    ###"""
    #   - End

    """
        Generate, plot and export a fixed potential flow with several optimal paths starting from different points
        and ending at the same goal point (comment if needed to save time)
    """

    #   + Start
    """#   Run function _getPath2Follow in C
    potentialFlow = maskedPotentialFlow.filled(maskValue)

    #   Define path settings
    nrows = maskedPotentialFlow.shape[0]
    ncols = maskedPotentialFlow.shape[1]
    goalPoint = (common.goalPoint[0], common.goalPoint[1])
    maxCycles = 800
    pathGap = common.pathGap
    goalCheck = common.goalCheck

    potentialFlow_flat = potentialFlow.ravel()  #   Flatten matrix by row order
    potentialFlow_c = (ctypes.c_double * potentialFlow_flat.size)(*potentialFlow_flat)

    #   Plot and export potential flow with several optimal paths
    paths = list()

    for _ in range(5):

        ref_e_c = None

        while(not ref_e_c):
            print("****************************************************")
            startingpoint_x, startingpoint_y = input("| Enter new starting point: ").split()
            startingpoint_x = float(startingpoint_x)
            startingpoint_y = float(startingpoint_y)
            startingPoint = (startingpoint_x, startingpoint_y)

            path_settings = SETTINGS(nrows, ncols, maxCycles, startingPoint, goalPoint, maskValue, pathGap, goalCheck)

            print(f"| Run _getPath2Follow in Python.")
            start = time.time()
            ref_e_c = planner_c._getPath2Follow(potentialFlow_c, path_settings)
            print("| Time: ", time.time() - start)
            print("****************************************************\n")
        
        ref_e_c_numpy = planner._turnResult2Numpy(ref_e_c)

        paths += [ref_e_c_numpy]

    #planner._plotMap(maskedPotentialFlow, "Time to goal [s]", includePath = paths)
    planner._export2Publish(maskedPotentialFlow, common.exportPlot2PubDir + "potentialFlow+paths" + "+" + option1 + "+" + option3 + ".pdf", "Time to goal [s]", includePath = paths)"""
    #   - End

    """
        Plot and export elevation map with optimals pats generated with respect to worst-case and refined maps
        under different conditions (comment if needed to save time)
    """

    #   + Start
    """#   Define path settings
    nrows = elevationMap.shape[0]
    ncols = elevationMap.shape[1]
    goalPoint = (common.goalPoint[0], common.goalPoint[1])
    maxCycles = 800
    pathGap = common.pathGap
    goalCheck = common.goalCheck

    startingPoint = (-20.0, -12.0)
    path_settings = SETTINGS(nrows, ncols, maxCycles, startingPoint, goalPoint, maskValue, pathGap, goalCheck)

    paths = []
    radius = 0.2
    
    #planner._updateTransformationParameters(elevationMap)

    potentialFlow_c = {}

    index = 0

    for option1 in ["worstCase", "mapRefinement"]:
        for option3 in ["Points", "Surface"]:
            ref_e_c = None

            #   Compute speed map under chosen specs
            maskedSpeedMap, maskedPotentialFlow = planner._getSpeedMap(option1, option3, radius, alpha, maskValue, t1, a, b)

            #   Run function _getPath2Follow in C
            potentialFlow = maskedPotentialFlow.filled(maskValue)

            potentialFlow_flat = potentialFlow.ravel()  #   Flatten matrix by row order
            potentialFlow_c = (ctypes.c_double * potentialFlow_flat.size)(*potentialFlow_flat)
            
            print("****************************************************")
            print(f"| Run _getPath2Follow in Python.")
            print(f"| Parameters: ", option1, option3, radius)
            start = time.time()
            ref_e_c = planner_c._getPath2Follow(potentialFlow_c, path_settings)
            print("| Time: ", time.time() - start)
            print("****************************************************\n")
            
            ref_e_c_numpy = planner._turnResult2Float32MultiArray(ref_e_c)

            planner_c._freePath(ref_e_c)

            paths += [ref_e_c_numpy]

            index += 1

    planner._plotMap(elevationMap, "Height [m]", includePath = paths)"""
    #planner._export2Publish(elevationMap, common.exportPlot2PubDir + "comparisonOfPaths" + ".pdf", "Height [m]", includePath = paths)
    #   - End

    """
        Run Monte-Carlo simulations to test function _getPath2Follow under different conditions.
        This test intends to record execution times, spot and fix found issues as well as to gather results for publication

        This function will run for different types of speed maps and starting positions. The goal point will remain fixed.
        Regarding the type of speed maps, only the type of map (worstCase or mapRefinment) and case (Points or Surface) change. The map 
        refinement parameters remain fixed.
        (comment if needed to save)
    """

    #   + Start
    """#   Define path settings
    nrows = elevationMap.shape[0]
    ncols = elevationMap.shape[1]
    goalPoint = (common.goalPoint[0], common.goalPoint[1])
    maxCycles = 800
    pathGap = common.pathGap
    goalCheck = common.goalCheck

    paths = []
    radius = 0.2

    planner._updateTransformationParameters(elevationMap)

    potentialFlow_c = {}

    index = 0

    times = []
    distance = []

    for option1 in ["worstCase", "mapRefinement"]:
        for option3 in ["Points", "Surface"]:

            #   Compute speed map under chosen specs
            maskedSpeedMap, maskedPotentialFlow = planner._getSpeedMap(option1, option3, radius, alpha, maskValue, t1, a, b)

            #   Run function _getPath2Follow in C
            potentialFlow = maskedPotentialFlow.filled(maskValue)

            potentialFlow_flat = potentialFlow.ravel()  #   Flatten matrix by row order
            potentialFlow_c = (ctypes.c_double * potentialFlow_flat.size)(*potentialFlow_flat)

            for startingPoint_x in np.linspace(-common.mapDimx/2.0, common.mapDimx/2.0, 100):
                for startingPoint_y in np.linspace(-common.mapDimy/2.0, common.mapDimy/2.0, 100):

                    startingPoint = (startingPoint_x, startingPoint_y)
                    path_settings = SETTINGS(nrows, ncols, maxCycles, startingPoint, goalPoint, maskValue, pathGap, goalCheck)

                    start = time.time()
                    ref_e_c = planner_c._getPath2Follow(potentialFlow_c, path_settings)
                    end = time.time() - start

                    #   Check if NULL was returned (valid pointer is interpreted as 1 and NULL as 0 by Python)
                    #   Only consider for evaluation calls that didn't return NULL
                    if( ref_e_c ):
                        print("Save.\n")
                        distance += [math.sqrt( math.pow(startingPoint_x - common.goalPoint[0], 2) + math.pow(startingPoint_y - common.goalPoint[1], 2) )]
                        times += [end]
                        index += 1

                    planner_c._freePath(ref_e_c)
    
    print("Number of iterations: ", index)
    print("(Times mean, std, max, min): ", np.mean(times), np.std(times), max(times), min(times))

    _a = round(np.mean(times) * math.pow(10, 3), 3)
    _b = round(np.std(times) * math.pow(10, 3), 3)

    plt.figure()
    plt.scatter( np.array(distance), np.array(times), c = 'red', s = 3.0 )
    plt.show()

    plt.figure()
    plt.violinplot( np.array(times) * math.pow(10, 3), showmeans=True, showmedians=False )
    plt.ylabel('Time [ms]')
    plt.title(r'$\nu\left(t \right)= {_a}$ [ms], $\sigma\left( t \right)={_b}$ [ms]'.format(_a=_a, _b=_b))

    print("Check if file exists.")
    if os.path.isfile(common.exportPlot2PubDir + "_getPath2Follow+Times" + ".pdf"):
        
        print("File exists. File was removed.")
        os.remove(common.exportPlot2PubDir + "_getPath2Follow+Times" + ".pdf")
    
    else:
        print("File does not exist.")

    print("Save new file.")
    plt.savefig(common.exportPlot2PubDir + "_getPath2Follow+Times" + ".pdf", bbox_inches='tight',transparent=True)
    plt.show()"""
    #   - End