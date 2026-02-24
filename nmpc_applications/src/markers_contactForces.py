#!/usr/bin/python3
from pathlib import Path

classes_path = Path(__file__)

from sys import path
path.append( str(classes_path) )

from classes.common_class import *

if __name__ == '__main__':

    common = Common()

    if( common.simulationType < 2 ):
        rospy.signal_shutdown("[" + rospy.get_name() + "] Shut down node.")

    rospy.init_node( 'markers', anonymous = True )

    rospy.Subscriber( '/contactForces', contactForces, common._callback, 32)
    rospy.wait_for_message('/contactForces', contactForces)

    pub_back_left_wheel_normal = rospy.Publisher('/markers/back_left_wheel/normal', Marker, queue_size = 1)
    pub_front_left_wheel_normal = rospy.Publisher('/markers/front_left_wheel/normal', Marker, queue_size = 1)
    pub_back_right_wheel_normal = rospy.Publisher('/markers/back_right_wheel/normal', Marker, queue_size = 1)
    pub_front_right_wheel_normal = rospy.Publisher('/markers/front_right_wheel/normal', Marker, queue_size = 1)

    pub_back_left_wheel_traction = rospy.Publisher('/markers/back_left_wheel/traction', Marker, queue_size = 1)
    pub_front_left_wheel_traction = rospy.Publisher('/markers/front_left_wheel/traction', Marker, queue_size = 1)
    pub_back_right_wheel_traction = rospy.Publisher('/markers/back_right_wheel/traction', Marker, queue_size = 1)
    pub_front_right_wheel_traction = rospy.Publisher('/markers/front_right_wheel/traction', Marker, queue_size = 1)

    #   Marker back left wheel (red arrows)
    markerBackLeftWheelNormalForce = Marker()
    
    markerBackLeftWheelNormalForce.header.stamp = rospy.Time.now()
    markerBackLeftWheelNormalForce.ns = "arrow"
    markerBackLeftWheelNormalForce.action = 0
    markerBackLeftWheelNormalForce.id = 0
    markerBackLeftWheelNormalForce.type = 0
    markerBackLeftWheelNormalForce.scale = Vector3(0.05, 0.1, 0.2)
    markerBackLeftWheelNormalForce.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)

    if( common.simulationType == 2 ):
        markerBackLeftWheelNormalForce.header.frame_id = "back_left_hub"

    else:
        markerBackLeftWheelNormalForce.header.frame_id = "contact_back_left"

    markerBackLeftWheelTractionForce = Marker()
    
    markerBackLeftWheelTractionForce.header.stamp = rospy.Time.now()
    markerBackLeftWheelTractionForce.ns = "arrow"
    markerBackLeftWheelTractionForce.action = 0
    markerBackLeftWheelTractionForce.id = 1
    markerBackLeftWheelTractionForce.type = 0
    markerBackLeftWheelTractionForce.scale = Vector3(0.05, 0.1, 0.2)
    markerBackLeftWheelTractionForce.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)

    if( common.simulationType == 2 ):
        markerBackLeftWheelTractionForce.header.frame_id = "back_left_hub"

    else:
        markerBackLeftWheelTractionForce.header.frame_id = "contact_back_left"
    ###

    #   Marker front left wheel (red arrows)
    markerFrontLeftWheelNormalForce = Marker()
    
    markerFrontLeftWheelNormalForce.header.stamp = rospy.Time.now()
    markerFrontLeftWheelNormalForce.ns = "arrow"
    markerFrontLeftWheelNormalForce.action = 0
    markerFrontLeftWheelNormalForce.id = 2
    markerFrontLeftWheelNormalForce.type = 0
    markerFrontLeftWheelNormalForce.scale = Vector3(0.05, 0.1, 0.2)
    markerFrontLeftWheelNormalForce.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)

    if( common.simulationType == 2 ):
        markerFrontLeftWheelNormalForce.header.frame_id = "front_left_hub"

    else:
        markerFrontLeftWheelNormalForce.header.frame_id = "contact_front_left"

    markerFrontLeftWheelTractionForce = Marker()
    
    markerFrontLeftWheelTractionForce.header.stamp = rospy.Time.now()
    markerFrontLeftWheelTractionForce.ns = "arrow"
    markerFrontLeftWheelTractionForce.action = 0
    markerFrontLeftWheelTractionForce.id = 3
    markerFrontLeftWheelTractionForce.type = 0
    markerFrontLeftWheelTractionForce.scale = Vector3(0.05, 0.1, 0.2)
    markerFrontLeftWheelTractionForce.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)

    if( common.simulationType == 2 ):
        markerFrontLeftWheelTractionForce.header.frame_id = "front_left_hub"

    else:
        markerFrontLeftWheelTractionForce.header.frame_id = "contact_front_left"
    ###

    #   Marker back right wheel (red arrows)
    markerBackRightWheelNormalForce = Marker()
    
    markerBackRightWheelNormalForce.header.stamp = rospy.Time.now()
    markerBackRightWheelNormalForce.ns = "arrow"
    markerBackRightWheelNormalForce.action = 0
    markerBackRightWheelNormalForce.id = 4
    markerBackRightWheelNormalForce.type = 0
    markerBackRightWheelNormalForce.scale = Vector3(0.05, 0.1, 0.2)
    markerBackRightWheelNormalForce.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)

    if( common.simulationType == 2 ):
        markerBackRightWheelNormalForce.header.frame_id = "back_right_hub"

    else:
        markerBackRightWheelNormalForce.header.frame_id = "contact_back_right"

    markerBackRightWheelTractionForce = Marker()
    
    markerBackRightWheelTractionForce.header.stamp = rospy.Time.now()
    markerBackRightWheelTractionForce.ns = "arrow"
    markerBackRightWheelTractionForce.action = 0
    markerBackRightWheelTractionForce.id = 5
    markerBackRightWheelTractionForce.type = 0
    markerBackRightWheelTractionForce.scale = Vector3(0.05, 0.1, 0.2)
    markerBackRightWheelTractionForce.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)

    if( common.simulationType == 2 ):
        markerBackRightWheelTractionForce.header.frame_id = "back_right_hub"

    else:
        markerBackRightWheelTractionForce.header.frame_id = "contact_back_right"
    ###

    #   Marker front right wheel (red arrows)
    markerFrontRightWheelNormalForce = Marker()
    
    markerFrontRightWheelNormalForce.header.stamp = rospy.Time.now()
    markerFrontRightWheelNormalForce.ns = "arrow"
    markerFrontRightWheelNormalForce.action = 0
    markerFrontRightWheelNormalForce.id = 6
    markerFrontRightWheelNormalForce.type = 0
    markerFrontRightWheelNormalForce.scale = Vector3(0.05, 0.1, 0.2)
    markerFrontRightWheelNormalForce.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)

    if( common.simulationType == 2 ):
        markerFrontRightWheelNormalForce.header.frame_id = "front_right_hub"

    else:
        markerFrontRightWheelNormalForce.header.frame_id = "contact_front_right"

    markerFrontRightWheelTractionForce = Marker()
    
    markerFrontRightWheelTractionForce.header.stamp = rospy.Time.now()
    markerFrontRightWheelTractionForce.ns = "arrow"
    markerFrontRightWheelTractionForce.action = 0
    markerFrontRightWheelTractionForce.id = 7
    markerFrontRightWheelTractionForce.type = 0
    markerFrontRightWheelTractionForce.scale = Vector3(0.05, 0.1, 0.2)
    markerFrontRightWheelTractionForce.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)

    if( common.simulationType == 2 ):
        markerFrontRightWheelTractionForce.header.frame_id = "front_right_hub"

    else:
        markerFrontRightWheelTractionForce.header.frame_id = "contact_front_right"
    ###

    print( "[" + rospy.get_name() + "] Simulation loop is running!" )

    while( not rospy.is_shutdown() ):
        try:
            forces = common.wheelForces

            normal_bl = forces.normalForce[0]
            normal_fl = forces.normalForce[1]
            normal_br = forces.normalForce[2]
            normal_fr = forces.normalForce[3]

            traction_bl = forces.tractionForce[0]
            traction_fl = forces.tractionForce[1]
            traction_br = forces.tractionForce[2]
            traction_fr = forces.tractionForce[3]

            #   Back left wheel
            if( normal_bl > 1e-3 ):
                markerBackLeftWheelNormalForce.action = 0
                markerBackLeftWheelTractionForce.action = 0

                markerBackLeftWheelNormalForce.header.stamp = rospy.Time.now()
                markerBackLeftWheelTractionForce.header.stamp = rospy.Time.now()

                markerBackLeftWheelTractionForce.points = [ Point(0, 0, 0), Point(traction_bl / (common.niu * normal_bl), 0, 0) ]
                markerBackLeftWheelNormalForce.points = [ Point(0, 0, 0), Point(0, 0, 1) ]

            else:
                markerBackLeftWheelNormalForce.action = 2
                markerBackLeftWheelTractionForce.action = 2

                markerBackLeftWheelTractionForce.points = []
                markerBackLeftWheelNormalForce.points = []
            
            #   Front left wheel
            if( normal_fl > 1e-3 ):
                markerFrontLeftWheelNormalForce.action = 0
                markerFrontLeftWheelTractionForce.action = 0

                markerFrontLeftWheelNormalForce.header.stamp = rospy.Time.now()
                markerFrontLeftWheelTractionForce.header.stamp = rospy.Time.now()

                markerFrontLeftWheelTractionForce.points = [ Point(0, 0, 0), Point(traction_fl / (common.niu * normal_fl), 0, 0) ]
                markerFrontLeftWheelNormalForce.points = [ Point(0, 0, 0), Point(0, 0, 1) ]

            else:
                markerFrontLeftWheelNormalForce.action = 2
                markerFrontLeftWheelTractionForce.action = 2

                markerFrontLeftWheelTractionForce.points = []
                markerFrontLeftWheelNormalForce.points = []
            
            #   Back right wheel
            if( normal_br > 1e-3 ):
                markerBackRightWheelTractionForce.action = 0
                markerBackRightWheelNormalForce.action = 0

                markerBackRightWheelNormalForce.header.stamp = rospy.Time.now()
                markerBackRightWheelTractionForce.header.stamp = rospy.Time.now()

                markerBackRightWheelTractionForce.points = [ Point(0, 0, 0), Point(traction_br / (common.niu * normal_br), 0, 0) ]
                markerBackRightWheelNormalForce.points = [ Point(0, 0, 0), Point(0, 0, 1) ]

            else:
                markerBackRightWheelTractionForce.action = 2
                markerBackRightWheelNormalForce.action = 2

                markerBackRightWheelTractionForce.points = []
                markerBackRightWheelNormalForce.points = []
                
            if( normal_fr > 1e-3 ):
                markerFrontRightWheelTractionForce.action = 0
                markerFrontRightWheelNormalForce.action = 0

                markerFrontRightWheelTractionForce.header.stamp = rospy.Time.now()
                markerFrontRightWheelNormalForce.header.stamp = rospy.Time.now()

                markerFrontRightWheelTractionForce.points = [ Point(0, 0, 0), Point(traction_fr / (common.niu * normal_fr), 0, 0) ]
                markerFrontRightWheelNormalForce.points = [ Point(0, 0, 0), Point(0, 0, 1) ]

            else:
                markerFrontRightWheelTractionForce.action = 2
                markerFrontRightWheelNormalForce.action = 2

                markerFrontRightWheelNormalForce.points = []
                markerFrontRightWheelTractionForce.points = []

            pub_back_left_wheel_traction.publish(markerBackLeftWheelTractionForce)
            pub_front_left_wheel_traction.publish(markerFrontLeftWheelTractionForce)
            pub_back_right_wheel_traction.publish(markerBackRightWheelTractionForce)
            pub_front_right_wheel_traction.publish(markerFrontRightWheelTractionForce)

            pub_back_left_wheel_normal.publish(markerBackLeftWheelNormalForce)
            pub_front_left_wheel_normal.publish(markerFrontLeftWheelNormalForce)
            pub_back_right_wheel_normal.publish(markerBackRightWheelNormalForce)
            pub_front_right_wheel_normal.publish(markerFrontRightWheelNormalForce)

        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
            print( "[" + rospy.get_name() + "] Something went wrong!" )

    rospy.spin()