#!/usr/bin/python3
from pathlib import Path

classes_path = Path(__file__)

from sys import path
path.append( str(classes_path) )

from classes.common_class import *

if __name__ == '__main__':

    common = Common()

    rospy.init_node( 'markers', anonymous = True )

    pub_referencePath = rospy.Publisher('/markers/referencePath', Marker, queue_size = 1)
    pub_kinHorizonPath = rospy.Publisher('/markers/kinematicsHorizonPath', Marker, queue_size = 1)
    pub_dynHorizonPath = rospy.Publisher('/markers/dynamicsHorizonPath', Marker, queue_size = 1)

    rospy.Subscriber( '/vehicle/reference', referencePath, common._multiArrayCallback, 0 )                                    #   '/vehicle/reference' -> topic which collects the reference path                                                   
    rospy.wait_for_message( '/vehicle/reference', referencePath )
    
    if( common.simulationType == 0 ):
        rospy.Subscriber( '/vehicle/nmpc_kinematics/horizonPath', Float32MultiArray, common._multiArrayCallback, 1 )
        rospy.wait_for_message( '/vehicle/nmpc_kinematics/horizonPath', Float32MultiArray )
    
    elif( common.simulationType == 1):
        rospy.Subscriber( '/vehicle/nmpc_kinematics/horizonPath', Float32MultiArray, common._multiArrayCallback, 1 )
        rospy.wait_for_message( '/vehicle/nmpc_kinematics/horizonPath', Float32MultiArray )

        rospy.Subscriber('/vehicle/nmpc_dynamics/horizonState', Float32MultiArray, common._multiArrayCallback, 2 )
        #rospy.wait_for_message( '/vehicle/nmpc_dynamics/horizonState', Float32MultiArray )

    #   Reference path marker (yellow cubes)
    markerReferencePath = Marker()
    
    markerReferencePath.header.stamp = rospy.Time.now()
    markerReferencePath.ns = "true_pose"
    markerReferencePath.action = 0
    markerReferencePath.id = 2
    markerReferencePath.type = 6
    markerReferencePath.scale = Vector3(0.05, 0.05, 0.05)
    markerReferencePath.color = ColorRGBA(1.0, 1.0, 0.0, 1.0)
    markerReferencePath.header.frame_id = "base_link"
    ###

    #   Path horizon returned by kinematics optimization problem (white spheres)
    markerKinematicsHorizonPath = Marker()

    markerKinematicsHorizonPath.header.stamp = rospy.Time.now()
    markerKinematicsHorizonPath.ns = "kinematics_horizon_path"
    markerKinematicsHorizonPath.action = 0
    markerKinematicsHorizonPath.id = 1
    markerKinematicsHorizonPath.type = 8
    markerKinematicsHorizonPath.scale = Vector3(0.1, 0.1, 0.1)
    markerKinematicsHorizonPath.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
    ###

    ###   Path horizon returned by dynamics optimization problem (red spheres)
    markerDynamicsHorizonPath = Marker()

    markerDynamicsHorizonPath.header.stamp = rospy.Time.now()
    markerDynamicsHorizonPath.ns = "dynamics_horizon_path"
    markerDynamicsHorizonPath.action = 0
    markerDynamicsHorizonPath.id = 4
    markerDynamicsHorizonPath.type = 8
    markerDynamicsHorizonPath.scale = Vector3(0.1, 0.1, 0.1)
    markerDynamicsHorizonPath.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)

    markerDynamicsHorizonPath.points = []
    ###

    if( common.poseType == 0 ):
        rospy.Subscriber( '/vehicle/true_pose3D', pose3DStamped, common._callback, 3 )                                    #   '/vehicle/true_pose3D' -> topic which collects robot perfect pose
        rospy.wait_for_message( '/vehicle/true_pose3D', pose3DStamped )

        truePosePub = rospy.Publisher( '/markers/true_robotPose', Marker, queue_size = 1 )                            #   '/markers/viz/true_robotPose' -> topic for ground truth pose visualization

    ###   True robot pose (green line)      #################
    markerTruePoseRobot = Marker()

    markerTruePoseRobot.header.stamp = rospy.Time.now()
    markerTruePoseRobot.ns = "true_pose"
    markerTruePoseRobot.action = 0
    markerTruePoseRobot.id = 3
    markerTruePoseRobot.type = 4
    markerTruePoseRobot.scale = Vector3(0.02, 0.0, 0.0)
    markerTruePoseRobot.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)

    markerTruePoseRobot.points = []
    ###

    #   Set markers reference frames
    if( common.poseType == 0 ):
        markerTruePoseRobot.header.frame_id = "base_link"
        markerKinematicsHorizonPath.header.frame_id = "base_link"
        markerDynamicsHorizonPath.header.frame_id = "base_link"
        markerReferencePath.header.frame_id ="base_link"
    ###
            
    index = 0

    print( "[markers_path.py] Simulation loop is running!" )

    while( not rospy.is_shutdown() ):
        try:
            path2Follow = common.referencePath

            initialPose = path2Follow.startingPose
            reference = path2Follow.reference

            if( common.simulationType == 0 ):
                kinematicsHorizonPath = list(common.horizonPath.data)
            
            elif( common.simulationType == 1 ):
                kinematicsHorizonPath = list(common.horizonPath.data)
                #dynamicsHorizonPath = list(common.horizonVelocity.data)
            
            markerReferencePath.points = []
            markerKinematicsHorizonPath.points = []
            markerDynamicsHorizonPath.points = []

            #   Collect raw reference path(point only without orientation) from fast marching algorithm
            for _index in range(common.N + 1):
                markerReferencePath.header.stamp = rospy.Time.now()
                markerReferencePath.points += [ Point(reference[_index * 3],\
                                                      reference[_index * 3 + 1], 0) ]

                markerKinematicsHorizonPath.header.stamp = rospy.Time.now()

                if( common.simulationType == 0 ):
                    markerKinematicsHorizonPath.points += [ Point( kinematicsHorizonPath[_index * 6],\
                                                                    kinematicsHorizonPath[_index * 6 + 1],\
                                                                    kinematicsHorizonPath[_index * 6 + 2] ) ]
                
                elif( common.simulationType == 1 ):
                    markerKinematicsHorizonPath.points += [ Point( kinematicsHorizonPath[_index * 12],\
                                                                    kinematicsHorizonPath[_index * 12 + 1],\
                                                                    kinematicsHorizonPath[_index * 12 + 2] ) ]
                    
                    """markerDynamicsHorizonPath.points += [ Point( dynamicsHorizonPath[_index * 12],\
                                                                 dynamicsHorizonPath[_index * 12 + 1],\
                                                                 dynamicsHorizonPath[_index * 12 + 2] ) ]"""

            true_pose = common.true_pose3D
            markerTruePoseRobot.points += [ Point( true_pose.pose.x, true_pose.pose.y, true_pose.pose.z ) ]
            
            truePosePub.publish( markerTruePoseRobot )
            pub_referencePath.publish(markerReferencePath)
            pub_kinHorizonPath.publish(markerKinematicsHorizonPath)
            pub_dynHorizonPath.publish(markerDynamicsHorizonPath)

            index += 1

        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
            print( "[markers.py] Something went wrong!" )

    rospy.spin()