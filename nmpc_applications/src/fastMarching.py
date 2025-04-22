#!/usr/bin/python3

"""
    fastMarching.py

    Node to deliver reference path to ros system through fast marching algorithm
"""

from pathlib import Path

classes_path = Path(__file__)

from sys import path
path.append( str(classes_path) )

from classes.common_class import *
from classes.planner_class import *

if __name__ == '__main__':

    common = Common()

    planner = Planner()

    script_path = Path(__file__)
    one_level_up = script_path.parent

    #   Load functions from Planner class defined translated to C for real-time use
    planner_c = ctypes.CDLL(str(one_level_up) + "/classes/planner.so")

    #   define struct arguments in ctypes
    class SETTINGS(ctypes.Structure):
        _fields_ = [("nrows", ctypes.c_int), ("ncols", ctypes.c_int), ("maxCycles", ctypes.c_int),\
                    ("startingPoint", ctypes.c_double * 2), ("goalPoint", ctypes.c_double * 2),\
                    ("maskValue", ctypes.c_double), ("pathGap", ctypes.c_double), ("goalCheck", ctypes.c_double)]

    planner_c._getPath2Follow.argtypes = [ctypes.POINTER(ctypes.c_double), SETTINGS]

    #   Define class Position from planner.c in Python
    class POSITION(ctypes.Structure):
        pass
    
    POSITION._fields_ = [ ("x", ctypes.c_double), ("y", ctypes.c_double), ("yaw", ctypes.c_double),\
                          ("next", ctypes.POINTER(POSITION)) ]

    planner_c._getPath2Follow.restype = ctypes.POINTER(POSITION)

    #   Define path settings
    elevationMap = planner._loadHeightmap(False)
    planner._updateTransformationParameters(elevationMap)

    nrows = elevationMap.shape[0]
    ncols = elevationMap.shape[1]
    goalPoint = (common.goalPoint[0], common.goalPoint[1])
    maxCycles = common.N
    pathGap = common.pathGap
    goalCheck = common.goalCheck

    """
        Parameters of map refinement

        alpha:  alpha
        radius:   radius
    """
    alpha = 0.9
    radius = 0.3

    t1 = 0.6
    a = 0.2
    b = 0.4
    
    #   Get potential flow
    maskedSpeedMap, maskedPotentialFlow = planner._getSpeedMap(common.option1, common.option3, radius, alpha, 0, t1, a, b)

    #   Run function _getPath2Follow in C
    potentialFlow = maskedPotentialFlow.filled(0)

    potentialFlow_flat = potentialFlow.ravel()  #   Flatten matrix by row order
    potentialFlow_c = (ctypes.c_double * potentialFlow_flat.size)(*potentialFlow_flat)

    rospy.init_node('pathPlanning', anonymous = True)

    #   Ground truth data
    if( common.poseType == 0 ):
        rospy.Subscriber( '/gazebo/link_states', LinkStates, common._callback, 0 )                                           #   '/gazebo/link_states' -> topic which collects ground truth
        rospy.Subscriber( '/vehicle/true_pose3D', pose3DStamped, common._callback, 3 )                                       #   '/vehicle/truePose' -> topic which collects robot perfect pose
        rospy.Subscriber( '/vehicle/true_velocity_bodyFrame', wheelTrueVelocitiesBodyFrame, common._callback, 6 )            #   '/vehicle/trueVelocity_bodyFrame' -> topic which collect robot links perfect velocity

        rospy.wait_for_message( '/vehicle/true_velocity_bodyFrame', wheelTrueVelocitiesBodyFrame )
        rospy.wait_for_message( '/gazebo/link_states', LinkStates )
        rospy.wait_for_message( '/vehicle/true_pose3D', pose3DStamped )

    #   Fused data (with noise)
    if( common.poseType == 1 ):
        rospy.Subscriber( '/vehicle/true_pose3D', pose3DStamped, common._callback, 3 )                                       #   '/vehicle/truePose' -> topic which collects robot perfect pose
        rospy.Subscriber( '/vehicle/noisy_pose3D', pose3DStamped, common._callback, 4 )                                      #   '/vehicle/noisy_pose3D' -> topic which collects robot perfect pose
        rospy.Subscriber( '/vehicle/true_velocity_bodyFrame', wheelTrueVelocitiesBodyFrame, common._callback, 6 )            #   '/vehicle/trueVelocity_bodyFrame' -> topic which collect robot links perfect velocity

        rospy.wait_for_message( '/vehicle/true_velocity_bodyFrame', wheelTrueVelocitiesBodyFrame )
        rospy.wait_for_message( '/vehicle/true_pose3D', pose3DStamped )
        rospy.wait_for_message( '/vehicle/noisy_pose3D', pose3DStamped )

    pub_nodePeriod = rospy.Publisher( '/node/fastMarching/period', Float32, queue_size = 1 )

    pub_ref = rospy.Publisher('/vehicle/reference', referencePath, queue_size = 1)

    gt_baseLinkIndex, _, _, _, _ = common._getLinksIndex( common.gazeboLinkStates )

    print("[fastMarching.py] Simulation cycle is running!")

    msg = referencePath()

    while not rospy.is_shutdown():
        try:
            start = time.time()
            
            currentPose = common.true_pose3D.pose
            currentVelocity = common.true_velocity_bodyFrame.velocity[0]

            startingPoint = (currentPose.x, currentPose.y)
            path_settings = SETTINGS(nrows, ncols, maxCycles + 1, startingPoint, goalPoint, 0, pathGap, goalCheck)
            
            #   Get optimal path from selected potential flow
            ref_e_c = planner_c._getPath2Follow(potentialFlow_c, path_settings)

            #   Smooth path
            smoothPath = planner._smoothPath(ref_e_c, 0.3, maxCycles)

            msg.reference = smoothPath
            msg.startingPose = currentPose
            msg.startingVelocity = currentVelocity

            planner_c._freePath(ref_e_c)

            timeDiff = time.time() - start

            pub_ref.publish(msg)
            pub_nodePeriod.publish( Float32(timeDiff) )

        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
            print( "[nmpc.py] Something went wrong!" )
    
    rospy.spin()