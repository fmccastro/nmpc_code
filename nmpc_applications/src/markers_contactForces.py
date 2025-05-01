#!/usr/bin/python3
from pathlib import Path

classes_path = Path(__file__)

from sys import path
path.append( str(classes_path) )

from classes.common_class import *

if __name__ == '__main__':

    common = Common()

    if( common.simulationType < 1 ):
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
    markerBackLeftWheelNormalForce.header.frame_id = "back_left_hub"

    markerBackLeftWheelTractionForce = Marker()
    
    markerBackLeftWheelTractionForce.header.stamp = rospy.Time.now()
    markerBackLeftWheelTractionForce.ns = "arrow"
    markerBackLeftWheelTractionForce.action = 0
    markerBackLeftWheelTractionForce.id = 1
    markerBackLeftWheelTractionForce.type = 0
    markerBackLeftWheelTractionForce.scale = Vector3(0.05, 0.1, 0.2)
    markerBackLeftWheelTractionForce.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)
    markerBackLeftWheelTractionForce.header.frame_id = "back_left_hub"
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
    markerFrontLeftWheelNormalForce.header.frame_id = "front_left_hub"

    markerFrontLeftWheelTractionForce = Marker()
    
    markerFrontLeftWheelTractionForce.header.stamp = rospy.Time.now()
    markerFrontLeftWheelTractionForce.ns = "arrow"
    markerFrontLeftWheelTractionForce.action = 0
    markerFrontLeftWheelTractionForce.id = 3
    markerFrontLeftWheelTractionForce.type = 0
    markerFrontLeftWheelTractionForce.scale = Vector3(0.05, 0.1, 0.2)
    markerFrontLeftWheelTractionForce.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)
    markerFrontLeftWheelTractionForce.header.frame_id = "front_left_hub"
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
    markerBackRightWheelNormalForce.header.frame_id = "back_right_hub"

    markerBackRightWheelTractionForce = Marker()
    
    markerBackRightWheelTractionForce.header.stamp = rospy.Time.now()
    markerBackRightWheelTractionForce.ns = "arrow"
    markerBackRightWheelTractionForce.action = 0
    markerBackRightWheelTractionForce.id = 5
    markerBackRightWheelTractionForce.type = 0
    markerBackRightWheelTractionForce.scale = Vector3(0.05, 0.1, 0.2)
    markerBackRightWheelTractionForce.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)
    markerBackRightWheelTractionForce.header.frame_id = "back_right_hub"
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
    markerFrontRightWheelNormalForce.header.frame_id = "front_right_hub"

    markerFrontRightWheelTractionForce = Marker()
    
    markerFrontRightWheelTractionForce.header.stamp = rospy.Time.now()
    markerFrontRightWheelTractionForce.ns = "arrow"
    markerFrontRightWheelTractionForce.action = 0
    markerFrontRightWheelTractionForce.id = 7
    markerFrontRightWheelTractionForce.type = 0
    markerFrontRightWheelTractionForce.scale = Vector3(0.05, 0.1, 0.2)
    markerFrontRightWheelTractionForce.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)
    markerFrontRightWheelTractionForce.header.frame_id = "front_right_hub"
    ###

    print( "[markers_path.py] Simulation loop is running!" )

    while( not rospy.is_shutdown() ):
        try:
            forces = common.wheelForces

            alpha_bl = forces.contactAngle[0]
            alpha_fl = forces.contactAngle[1]
            alpha_br = forces.contactAngle[2]
            alpha_fr = forces.contactAngle[3]

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
                contactPoint_bl_x = common.wheelRadius * math.sin(alpha_bl)
                contactPoint_bl_z = -common.wheelRadius * math.cos(alpha_bl)

                end_traction_bl_x = contactPoint_bl_x + math.cos(alpha_bl) * traction_bl / (common.niu * normal_bl)
                end_traction_bl_z = contactPoint_bl_z + math.sin(alpha_bl) * traction_bl / (common.niu * normal_bl)

                end_normal_bl_x = contactPoint_bl_x - math.sin(alpha_bl)
                end_normal_bl_z = contactPoint_bl_z + math.cos(alpha_bl)

                markerBackLeftWheelNormalForce.action = 0
                markerBackLeftWheelTractionForce.action = 0

                markerBackLeftWheelTractionForce.points = [ Point(contactPoint_bl_x, 0, contactPoint_bl_z), Point(end_traction_bl_x, 0, end_traction_bl_z) ]
                markerBackLeftWheelNormalForce.points = [ Point(contactPoint_bl_x, 0, contactPoint_bl_z), Point(end_normal_bl_x, 0, end_normal_bl_z) ]

            else:
                markerBackLeftWheelNormalForce.action = 2
                markerBackLeftWheelTractionForce.action = 2

                markerBackLeftWheelTractionForce.points = []
                markerBackLeftWheelNormalForce.points = []
            
            #   Front left wheel
            if( normal_fl > 1e-3 ):
                contactPoint_fl_x = common.wheelRadius * math.sin(alpha_fl)
                contactPoint_fl_z = -common.wheelRadius * math.cos(alpha_fl)

                end_traction_fl_x = contactPoint_fl_x + math.cos(alpha_fl) * traction_fl / (common.niu * normal_fl)
                end_traction_fl_z = contactPoint_fl_z + math.sin(alpha_fl) * traction_fl / (common.niu * normal_fl)

                end_normal_fl_x = contactPoint_fl_x - math.sin(alpha_fl)
                end_normal_fl_z = contactPoint_fl_z + math.cos(alpha_fl)

                markerFrontLeftWheelTractionForce.action = 0
                markerFrontLeftWheelNormalForce.action = 0

                markerFrontLeftWheelTractionForce.points = [ Point(contactPoint_fl_x, 0, contactPoint_fl_z), Point(end_traction_fl_x, 0, end_traction_fl_z) ]
                markerFrontLeftWheelNormalForce.points = [ Point(contactPoint_fl_x, 0, contactPoint_fl_z), Point(end_normal_fl_x, 0, end_normal_fl_z) ]

            else:
                markerFrontLeftWheelNormalForce.action = 2
                markerFrontLeftWheelTractionForce.action = 2

                markerFrontLeftWheelTractionForce.points = []
                markerFrontLeftWheelNormalForce.points = []
            
            #   Back right wheel
            if( normal_br > 1e-3 ):
                contactPoint_br_x = common.wheelRadius * math.sin(alpha_br)
                contactPoint_br_z = -common.wheelRadius * math.cos(alpha_br)
            
                end_traction_br_x = contactPoint_br_x + math.cos(alpha_br) * traction_br / (common.niu * normal_br)
                end_traction_br_z = contactPoint_br_z + math.sin(alpha_br) * traction_br / (common.niu * normal_br)

                end_normal_br_x = contactPoint_br_x - math.sin(alpha_br)
                end_normal_br_z = contactPoint_br_z + math.cos(alpha_br)

                markerBackRightWheelTractionForce.action = 0
                markerBackRightWheelNormalForce.action = 0

                markerBackRightWheelTractionForce.points = [ Point(contactPoint_br_x, 0, contactPoint_br_z), Point(end_traction_br_x, 0, end_traction_br_z) ]
                markerBackRightWheelNormalForce.points = [ Point(contactPoint_br_x, 0, contactPoint_br_z), Point(end_normal_br_x, 0, end_normal_br_z) ]

            else:
                markerBackRightWheelTractionForce.action = 2
                markerBackRightWheelNormalForce.action = 2

                markerBackRightWheelTractionForce.points = []
                markerBackRightWheelNormalForce.points = []
                
            if( normal_fr > 1e-3 ):
                contactPoint_fr_x = common.wheelRadius * math.sin(alpha_fr)
                contactPoint_fr_z = -common.wheelRadius * math.cos(alpha_fr)

                end_traction_fr_x = contactPoint_fr_x + math.cos(alpha_fr) * traction_fr / (common.niu * normal_fr)
                end_traction_fr_z = contactPoint_fr_z + math.sin(alpha_fr) * traction_fr / (common.niu * normal_fr)

                end_normal_fr_x = contactPoint_fr_x - math.sin(alpha_fr)
                end_normal_fr_z = contactPoint_fr_z + math.cos(alpha_fr)

                markerFrontRightWheelTractionForce.action = 0
                markerFrontRightWheelNormalForce.action = 0

                markerFrontRightWheelTractionForce.points = [ Point(contactPoint_fr_x, 0, contactPoint_fr_z), Point(end_traction_fr_x, 0, end_traction_fr_z) ]
                markerFrontRightWheelNormalForce.points = [ Point(contactPoint_fr_x, 0, contactPoint_fr_z), Point(end_normal_fr_x, 0, end_normal_fr_z) ]

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
            print( "[markers.py] Something went wrong!" )

    rospy.spin()