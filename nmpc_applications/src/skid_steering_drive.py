#!/usr/bin/python3.8

from pathlib import Path

classes_path = Path(__file__)

from sys import path
path.append( str(classes_path) )

from classes.model_class_acados import *
from classes.common_class import *

if __name__ == '__main__':

    #   Start node
    rospy.init_node('skid_steering_drive', anonymous = True, disable_signals = True)

    common = Common()

    rospy.Subscriber( '/mouse/cmd_vel', Twist, common._callback, 5 )
    rospy.wait_for_message( '/mouse/cmd_vel', Twist )

    pub_bl_wheelRates = rospy.Publisher( '/back_left_wheel_plant/command', Float64, queue_size = 1 )
    pub_fl_wheelRates = rospy.Publisher( '/front_left_wheel_plant/command', Float64, queue_size = 1 )
    pub_br_wheelRates = rospy.Publisher( '/back_right_wheel_plant/command', Float64, queue_size = 1 )
    pub_fr_wheelRates = rospy.Publisher( '/front_right_wheel_plant/command', Float64, queue_size = 1 )

    pub_nodePeriod = rospy.Publisher( '/vehicle/node/skid_steering_drive/period', Float64, queue_size = 1 )

    print("[" + rospy.get_name() + "] Loop is running.")

    while not rospy.is_shutdown():
        try:
            start = time.time()

            vel = common.velocityActuation

            w_r = ( 2 * vel.linear.x + vel.angular.z * common.wheelLatSeparation ) / ( 2 * common.wheelRadius )
            w_l = ( 2 * vel.linear.x - vel.angular.z * common.wheelLatSeparation ) / ( 2 * common.wheelRadius )

            pub_bl_wheelRates.publish( Float64(w_l) )
            pub_fl_wheelRates.publish( Float64(w_l) )
            pub_br_wheelRates.publish( Float64(w_r) )
            pub_fr_wheelRates.publish( Float64(w_r) )

            end = time.time()

            timeDiff = end - start

            pub_nodePeriod.publish(timeDiff)

        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print( "[" + rospy.get_name() + "] Something went wrong!" )
    
    pub_bl_wheelRates.publish(0.0)
    pub_fl_wheelRates.publish(0.0)
    pub_br_wheelRates.publish(0.0)
    pub_fr_wheelRates.publish(0.0)

    rospy.spin()