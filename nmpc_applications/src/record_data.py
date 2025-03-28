#!/usr/bin/python3
from pathlib import Path

classes_path = Path(__file__)

upper_directory = classes_path.parent
upper_directory = upper_directory.parent
lower_directory = upper_directory / "data" / "modelIdentification"

#   Change bag file name according to needs
bag_file_dir = lower_directory / "niu_1.bag"

from sys import path
path.append( str(classes_path) )

from classes.common_class import *

if __name__ == '__main__':

    common = Common()

    rospy.init_node('record_data', anonymous = True)

    bag = rosbag.Bag(bag_file_dir, 'w')

    rospy.Subscriber( '/back_left_wheel_plant/command', Float64, common._wheelTorqueInputCallback, 0 )                             
    rospy.Subscriber( '/front_left_wheel_plant/command', Float64, common._wheelTorqueInputCallback, 1 )
    rospy.Subscriber( '/back_right_wheel_plant/command', Float64, common._wheelTorqueInputCallback, 2 )
    rospy.Subscriber( '/front_right_wheel_plant/command', Float64, common._wheelTorqueInputCallback, 3 )
    rospy.Subscriber( '/vehicle/true_velocity_bodyFrame', wheelTrueVelocitiesBodyFrame, common._callback, 6 )
    rospy.Subscriber( '/joint_states', JointState, common._callback, 8 )

    rospy.wait_for_message( '/back_left_wheel_plant/command', Float64 )
    rospy.wait_for_message( '/front_left_wheel_plant/command', Float64 )
    rospy.wait_for_message( '/back_right_wheel_plant/command', Float64 )
    rospy.wait_for_message( '/front_right_wheel_plant/command', Float64 )
    rospy.wait_for_message( '/vehicle/true_velocity_bodyFrame', wheelTrueVelocitiesBodyFrame )
    rospy.wait_for_message( '/joint_states', JointState )
    
    loop_freq_rate = Float64(0)

    print("[" + rospy.get_name() + "] Loop is running.")

    while not rospy.is_shutdown():
        try:
            start = time.time()

            tau_bl = common.backLeftWheelTorque
            tau_fl = common.frontLeftWheelTorque
            tau_br = common.backRightWheelTorque
            tau_fr = common.frontRightWheelTorque
            jointStates = common.jointStates
            velocity_bodyFrame = common.true_velocity_bodyFrame

            bag.write('/record_data/torque_bl', tau_bl)
            bag.write('/record_data/torque_fl', tau_fl)
            bag.write('/record_data/torque_br', tau_br)
            bag.write('/record_data/torque_fr', tau_fr)
            bag.write('/record_data/jointState', jointStates)
            bag.write('/record_data/velocity_bodyFrame', velocity_bodyFrame)
        
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print( "[" + rospy.get_name() + "] Something went wrong!" )
            continue
    
    bag.close()
    
    rospy.spin()