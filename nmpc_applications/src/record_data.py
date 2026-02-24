#!/usr/bin/python3
from pathlib import Path

classes_path = Path(__file__)

upper_directory = classes_path.parent
upper_directory = upper_directory.parent
lower_directory = upper_directory / "data" / "modelIdentification"

#   Change bag file name according to needs
bag_file_dir = lower_directory / "modelDynamics.bag"

from sys import path
path.append( str(classes_path) )

from classes.common_class import *

if __name__ == '__main__':

    common = Common()

    rospy.init_node('record_data', anonymous = True)
    
    bag = rosbag.Bag(bag_file_dir, 'w')

    rospy.Subscriber( '/vehicle/true_pose3D', pose3DStamped, common._callback, 3)
    #rospy.Subscriber( '/back_left_wheel_wrench', WrenchStamped, common._linkWrenchCallback, 0 )
    #rospy.Subscriber( '/front_left_wheel_wrench', WrenchStamped, common._linkWrenchCallback, 1 )
    #rospy.Subscriber( '/back_right_wheel_wrench', WrenchStamped, common._linkWrenchCallback, 2 )
    #rospy.Subscriber( '/front_right_wheel_wrench', WrenchStamped, common._linkWrenchCallback, 3 )
    rospy.Subscriber( '/base_wrench', WrenchStamped, common._linkWrenchCallback, 0 )                             
    rospy.Subscriber( '/back_left_wheel_wrench', WrenchStamped, common._linkWrenchCallback, 1 )
    rospy.Subscriber( '/front_left_wheel_wrench', WrenchStamped, common._linkWrenchCallback, 2 )
    rospy.Subscriber( '/back_right_wheel_wrench', WrenchStamped, common._linkWrenchCallback, 3 )
    rospy.Subscriber( '/front_right_wheel_wrench', WrenchStamped, common._linkWrenchCallback, 4 )
    #rospy.Subscriber( '/base_wrench/rel_pose', Vector3,  )
    rospy.Subscriber( '/back_left_wheel_wrench/rel_pose', Vector3, common._wheelRate, 0 )
    rospy.Subscriber( '/front_left_wheel_wrench/rel_pose', Vector3, common._wheelRate, 1 )
    rospy.Subscriber( '/back_right_wheel_wrench/rel_pose', Vector3, common._wheelRate, 2 )
    rospy.Subscriber( '/front_right_wheel_wrench/rel_pose', Vector3, common._wheelRate, 3 )
    rospy.Subscriber( '/vehicle/true_velocity_bodyFrame', wheelTrueVelocitiesBodyFrame, common._callback, 6 )            #   '/vehicle/trueVelocity_bodyFrame' -> topic which collect robot links perfect velocity
    rospy.Subscriber( '/joint_states', JointState, common._callback, 8 )

    rospy.wait_for_message( '/vehicle/true_pose3D', pose3DStamped)
    #rospy.wait_for_message( '/back_left_wheel_wrench', WrenchStamped )
    #rospy.wait_for_message( '/front_left_wheel_wrench', WrenchStamped )
    #rospy.wait_for_message( '/back_right_wheel_wrench', WrenchStamped )
    #rospy.wait_for_message( '/front_right_wheel_wrench', WrenchStamped )
    rospy.wait_for_message( '/base_wrench', WrenchStamped )
    rospy.wait_for_message( '/back_left_wheel_wrench', WrenchStamped )
    rospy.wait_for_message( '/front_left_wheel_wrench', WrenchStamped )
    rospy.wait_for_message( '/back_right_wheel_wrench', WrenchStamped )
    rospy.wait_for_message( '/front_right_wheel_wrench', WrenchStamped )
    rospy.wait_for_message( '/back_left_wheel_wrench/rel_pose', Vector3 )
    rospy.wait_for_message( '/front_left_wheel_wrench/rel_pose', Vector3 )
    rospy.wait_for_message( '/back_right_wheel_wrench/rel_pose', Vector3 )
    rospy.wait_for_message( '/front_right_wheel_wrench/rel_pose', Vector3 )
    rospy.wait_for_message( '/vehicle/true_velocity_bodyFrame', wheelTrueVelocitiesBodyFrame )
    rospy.wait_for_message( '/joint_states', JointState )
    
    loop_freq_rate = Float64(0)

    print("[" + rospy.get_name() + "] Loop is running.")
    
    while not rospy.is_shutdown():
        try:
            start = time.time()
            
            true_pose = common.true_pose3D
            #contact_bl = common.backLeftWheelContact
            #contact_fl = common.frontLeftWheelContact
            #contact_br = common.backRightWheelContact
            #contact_fr = common.frontRightWheelContact
            wrench_base_link = common.base_link_wrench
            wrench_back_left = common.back_left_wrench
            wrench_front_left = common.front_left_wrench
            wrench_back_right = common.back_right_wrench
            wrench_front_right = common.front_right_wrench
            velocity_bodyFrame = common.true_velocity_bodyFrame
            bl_or = common.rate_bl
            fl_or = common.rate_fl
            br_or = common.rate_br
            fr_or = common.rate_fr
            joint_states = common.jointStates

            bag.write( '/record_data/true_pose3D', true_pose)
            #bag.write( '/record_data/back_left_wheel_wrench', contact_bl)
            #bag.write( '/record_data/front_left_wheel_wrench', contact_fl)
            #bag.write( '/record_data/back_right_wheel_wrench', contact_br)
            #bag.write( '/record_data/front_right_wheel_wrench', contact_fr)
            bag.write('/record_data/wrench_base_link', wrench_base_link)
            bag.write('/record_data/wrench_back_left', wrench_back_left)
            bag.write('/record_data/wrench_front_left', wrench_front_left)
            bag.write('/record_data/wrench_back_right', wrench_back_right)
            bag.write('/record_data/wrench_front_right', wrench_front_right)
            bag.write('/record_data/velocity_bodyFrame', velocity_bodyFrame)

            bag.write('/record_data/bl_rel_pose', bl_or)
            bag.write('/record_data/fl_rel_pose', fl_or)
            bag.write('/record_data/br_rel_pose', br_or)
            bag.write('/record_data/fr_rel_pose', fr_or)

            bag.write('/record_data/joint_states', joint_states)
            
            now = rospy.Time.now()
            
            now_time = now.secs + now.nsecs * math.pow(10, -9)

            bag.write('/record_data/clock', Float64(now_time))
        
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print( "[" + rospy.get_name() + "] Something went wrong!" )
            continue
    
    bag.close()
    
    rospy.spin()