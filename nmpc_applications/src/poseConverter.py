#!/usr/bin/python3

from sys import path
path.append(r"/media/fmccastro/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/src")

from classes.common_class import *
    
def _turn_pose_to_pose3D( _pose, rpy ):

    """
        Turn robot link pose into a (x, y, z, roll, pitch, yaw) set
    """

    pose = pose3D()
    
    pose.x = _pose.position.x
    pose.y = _pose.position.y
    pose.z = _pose.position.z

    pose.roll = rpy[0]
    pose.pitch = rpy[1]
    pose.yaw = rpy[2]
    
    return pose

def _quaternionToEuler( quaternion ):

    """
        Turn quaternion into a (roll, pitch, yaw) angle set
    """

    r = R.from_quat( [ quaternion.x, quaternion.y, quaternion.z, quaternion.w ] )

    return r.as_euler('xyz')

if __name__ == '__main__':

    common = Common()

    rospy.init_node('poseConverter', anonymous = True)

    rospy.Subscriber( '/gazebo/link_states', LinkStates, common._callback, 0 )                             #   '/gazebo/link_states' -> topic which collects perfect sensors data
    rospy.wait_for_message( '/gazebo/link_states', LinkStates )

    pub_true_vehiclePose = rospy.Publisher( '/vehicle/true_pose3D', pose3DStamped, queue_size = 1 )                                                     #   '/vehicle/true_pose3D' -> topic for 3D pose
    
    ###
    tfBuffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tfBuffer)

    pub_loop_freq = rospy.Publisher( '/rosnode/poseConverter/loop_freq_rate', Float64, queue_size = 1 )

    ###

    baseLinkIndex, _, _, _, _ = common._getLinksIndex( common.gazeboLinkStates )

    ###
    true_pose_stamped = pose3DStamped()
    
    loop_freq_rate = Float64(0)

    print("[poseConverter.py] Simulation cycle is running!")

    while not rospy.is_shutdown():
        try:
            start = time.time()

            ###
            linkStates = common.gazeboLinkStates

            rpy = _quaternionToEuler( linkStates.pose[baseLinkIndex].orientation )

            true_pose_stamped.header.frame_id = "base_link"
            true_pose_stamped.header.stamp = rospy.Time.now()
            true_pose_stamped.pose = _turn_pose_to_pose3D( linkStates.pose[baseLinkIndex], rpy )                #   Turn received pose type into a pose3D type
            
            try:
                loop_freq_rate.data = 1.0 / ( time.time() - start )
                pub_loop_freq.publish( loop_freq_rate )

            except( ZeroDivisionError ):
                loop_freq_rate.data = 0.0
                pub_loop_freq.publish( loop_freq_rate )

            pub_true_vehiclePose.publish( true_pose_stamped )
        
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print( "[poseConverter.py] Something went wrong!" )
            continue
    
    rospy.spin()