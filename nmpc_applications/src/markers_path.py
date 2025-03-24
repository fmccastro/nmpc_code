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
    pub_horizonPath = rospy.Publisher('/markers/horizonPath', Marker, queue_size = 1)

    rospy.Subscriber( '/vehicle/reference', Float32MultiArray, common._multiArrayCallback, 0 )                                       #   '/vehicle/reference' -> topic which collects the reference path                                                   
    rospy.Subscriber( '/vehicle/nmpc_kinematics/horizonPath', Float32MultiArray, common._multiArrayCallback, 1 )

    rospy.wait_for_message( '/vehicle/reference', Float32MultiArray )
    rospy.wait_for_message( '/vehicle/nmpc_kinematics/horizonPath', Float32MultiArray )

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

    #   Path horizon returned by optimization problem (white spheres)
    markerHorizonPath = Marker()

    markerHorizonPath.header.stamp = rospy.Time.now()
    markerHorizonPath.ns = "horizon_path"
    markerHorizonPath.action = 0
    markerHorizonPath.id = 1
    markerHorizonPath.type = 8
    markerHorizonPath.scale = Vector3(0.1, 0.1, 0.1)
    markerHorizonPath.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
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

    ###   Local odometry marker (blue line)     #################
    markerLocalLocRobot = Marker()

    markerLocalLocRobot.header.stamp = rospy.Time.now()
    markerLocalLocRobot.ns = "local_loc"
    markerLocalLocRobot.action = 0
    markerLocalLocRobot.id = 4
    markerLocalLocRobot.type = 4
    markerLocalLocRobot.scale = Vector3(0.02, 0.0, 0.0)
    markerLocalLocRobot.color = ColorRGBA(0.0, 0.0, 1.0, 1.0)

    markerLocalLocRobot.points = []
    ###

    ###   Global odometry marker (red line)     #################
    markerGlobalLocRobot = Marker()

    markerGlobalLocRobot.header.stamp = rospy.Time.now()
    markerGlobalLocRobot.ns = "global_loc"
    markerGlobalLocRobot.action = 0
    markerGlobalLocRobot.id = 5
    markerGlobalLocRobot.type = 4
    markerGlobalLocRobot.scale = Vector3(0.02, 0.0, 0.0)
    markerGlobalLocRobot.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)

    markerGlobalLocRobot.points = []
    ###

    #   Set markers reference frames
    if( common.poseType == 0 ):
        markerTruePoseRobot.header.frame_id = "base_link"
        markerHorizonPath.header.frame_id = "base_link"
        markerReferencePath.header.frame_id ="base_link"
    ###
            
    index = 0

    print( "[markers_path.py] Simulation loop is running!" )

    while( not rospy.is_shutdown() ):
        try:
            referencePath = list(common.referencePath.data)
            horizonPath = list(common.horizonPath.data)
            
            markerReferencePath.points = []
            markerHorizonPath.points = []

            #   Collect raw reference path(point only without orientation) from fast marching algorithm
            for _index in range(common.N + 1):
                markerReferencePath.header.stamp = rospy.Time.now()
                markerReferencePath.points += [ Point(referencePath[_index * 3],\
                                                      referencePath[_index * 3 + 1], 0) ]

                markerHorizonPath.header.stamp = rospy.Time.now()
                markerHorizonPath.points += [ Point( horizonPath[_index * (common.NbPosition + common.NbOrientation)],\
                                                     horizonPath[_index * (common.NbPosition + common.NbOrientation) + 1],\
                                                     horizonPath[_index * (common.NbPosition + common.NbOrientation) + 2] ) ]

            if( common.poseType == 0 ):
                true_pose = common.true_pose3D
                markerTruePoseRobot.points += [ Point( true_pose.pose.x, true_pose.pose.y, true_pose.pose.z ) ]
            
            if( common.poseType == 0 ):
                truePosePub.publish( markerTruePoseRobot )

            pub_referencePath.publish(markerReferencePath)
            pub_horizonPath.publish(markerHorizonPath)

            index += 1

        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
            print( "[markers.py] Something went wrong!" )

    rospy.spin()