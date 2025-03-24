#!/usr/bin/python3

from sys import path
path.append(r"/media/fmccastro/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/src")

from classes.common_class import *

def _quaternionToRotationMatrix( quaternion, rpy ):

    if( tuple( map( int, str( scipy.__version__ ).split('.') ) ) < ( 1, 4, 0 ) ):
        roll = rpy[0]
        pitch = rpy[1]
        yaw = rpy[2]

        yawMatrix = np.matrix( [ [math.cos(yaw), -math.sin(yaw), 0],
                                    [math.sin(yaw), math.cos(yaw), 0],
                                    [0, 0, 1] ] )
            
        pitchMatrix = np.matrix( [ [math.cos(pitch), 0, math.sin(pitch)],
                                      [0, 1, 0],
                                      [-math.sin(pitch), 0, math.cos(pitch)] ])
        
        rollMatrix = np.matrix( [ [1, 0, 0],
                                     [0, math.cos(roll), -math.sin(roll)],
                                     [0, math.sin(roll), math.cos(roll)] ])
        
        return yawMatrix @ pitchMatrix @ rollMatrix
    
    else:
        r = R.from_quat( [ quaternion.x, quaternion.y, quaternion.z, quaternion.w ] )

        return r.as_matrix()

def _quaternionToEuler( quaternion ):

    """
        Turn quaternion into a (roll, pitch, yaw) angle set
    """

    r = R.from_quat( [ quaternion.x, quaternion.y, quaternion.z, quaternion.w ] )

    return r.as_euler('xyz')

def _turn_velocity_into_bodyFrame( _velocity, rotationMatrix, rpy ):

    velocity = Twist()
        
    roll = rpy[0]
    pitch = rpy[1]
    yaw = rpy[2]
        
    pitchMatrix = np.matrix( [ [math.cos(pitch), 0, math.sin(pitch)],
                                    [0, 1, 0],
                                    [-math.sin(pitch), 0, math.cos(pitch)] ])
    
    rollMatrix = np.matrix( [ [1, 0, 0],
                                    [0, math.cos(roll), -math.sin(roll)],
                                    [0, math.sin(roll), math.cos(roll)] ])

    linVelocity_bodyFrame = rotationMatrix.T @ np.array( [ _velocity.linear.x, _velocity.linear.y, _velocity.linear.z ] ).reshape(-1, 1)
    angVelocity_bodyFrame = rotationMatrix.T @ np.array( [ _velocity.angular.x, _velocity.angular.y, _velocity.angular.z ] ).reshape(-1, 1)

    angVelocity_bodyFrame = np.array( [_velocity.angular.x, 0, 0] ).reshape(-1, 1) +\
                            rollMatrix @ np.array( [0, _velocity.angular.y, 0] ).reshape(-1, 1) +\
                            rollMatrix @ pitchMatrix @ np.array( [0, 0, _velocity.angular.z] ).reshape(-1, 1)

    velocity.angular.x = angVelocity_bodyFrame[0, 0]
    velocity.angular.y = angVelocity_bodyFrame[1, 0]
    velocity.angular.z = angVelocity_bodyFrame[2, 0]
    
    velocity.linear.x = linVelocity_bodyFrame[0, 0]
    velocity.linear.y = linVelocity_bodyFrame[1, 0]
    velocity.linear.z = linVelocity_bodyFrame[2, 0]

    return velocity
    
if __name__ == '__main__':

    common = Common()

    rospy.init_node('wheelKinematics', anonymous = True)

    #   Load vectors from com to each wheel center
    with open( common.results_folder + "com2wheels_" + common.robot + '.pickle', 'rb') as handle:
        com2wheels = pickle.load(handle)

    rospy.Subscriber( '/gazebo/link_states', LinkStates, common._callback, 0 )                             #   '/gazebo/link_states' -> topic which collects perfect sensors data
    rospy.wait_for_message( '/gazebo/link_states', LinkStates )

    pub_true_velocity_bodyFrame = rospy.Publisher( '/vehicle/true_velocity_bodyFrame', wheelTrueVelocitiesBodyFrame, queue_size = 1 )                   #   '/vehicle/true_velocity_bodyFrame' -> topic for true velocity

    ###

    true_velocity_bodyFrame = wheelTrueVelocitiesBodyFrame()
    true_velocity_bodyFrame.wheel = ["base_link", "back_left_wheel", "front_left_wheel", "back_right_wheel", "front_right_wheel"]

    baseLinkIndex, backLeftIndex, frontLeftIndex, backRightIndex, frontRightIndex = common._getLinksIndex( common.gazeboLinkStates )

    print("[wheelKinematics.py] Simulation cycle is running!")

    while not rospy.is_shutdown():
        try:
            ###
            linkStates = common.gazeboLinkStates

            rpy = _quaternionToEuler( linkStates.pose[baseLinkIndex].orientation )
            
            rotationMatrix = _quaternionToRotationMatrix( linkStates.pose[baseLinkIndex].orientation, rpy )

            true_velocity_baseLink_bodyFrame = _turn_velocity_into_bodyFrame( linkStates.twist[baseLinkIndex], rotationMatrix, rpy )

            v_bl = np.array( [ true_velocity_baseLink_bodyFrame.linear.x, true_velocity_baseLink_bodyFrame.linear.y, true_velocity_baseLink_bodyFrame.linear.z ] ) +\
                    np.cross( np.array( [ true_velocity_baseLink_bodyFrame.angular.x, true_velocity_baseLink_bodyFrame.angular.y, true_velocity_baseLink_bodyFrame.angular.z ] ),\
                                np.array( [ com2wheels["com2bl"][0], com2wheels["com2bl"][1], com2wheels["com2bl"][2] ] ) )
            
            v_fl = np.array( [ true_velocity_baseLink_bodyFrame.linear.x, true_velocity_baseLink_bodyFrame.linear.y, true_velocity_baseLink_bodyFrame.linear.z ] ) +\
                    np.cross( np.array( [ true_velocity_baseLink_bodyFrame.angular.x, true_velocity_baseLink_bodyFrame.angular.y, true_velocity_baseLink_bodyFrame.angular.z ] ),\
                                np.array( [ com2wheels["com2fl"][0], com2wheels["com2fl"][1], com2wheels["com2fl"][2] ] ) )
            
            v_br = np.array( [ true_velocity_baseLink_bodyFrame.linear.x, true_velocity_baseLink_bodyFrame.linear.y, true_velocity_baseLink_bodyFrame.linear.z ] ) +\
                    np.cross( np.array( [ true_velocity_baseLink_bodyFrame.angular.x, true_velocity_baseLink_bodyFrame.angular.y, true_velocity_baseLink_bodyFrame.angular.z ] ),\
                                np.array( [ com2wheels["com2br"][0], com2wheels["com2br"][1], com2wheels["com2br"][2] ] ) )
        
            v_fr = np.array( [ true_velocity_baseLink_bodyFrame.linear.x, true_velocity_baseLink_bodyFrame.linear.y, true_velocity_baseLink_bodyFrame.linear.z ] ) +\
                    np.cross( np.array( [ true_velocity_baseLink_bodyFrame.angular.x, true_velocity_baseLink_bodyFrame.angular.y, true_velocity_baseLink_bodyFrame.angular.z ] ),\
                                np.array( [ com2wheels["com2fr"][0], com2wheels["com2fr"][1], com2wheels["com2fr"][2] ] ) )

            true_velocity_backLeftWheel_bodyFrame = Twist( Vector3(v_bl[0], v_bl[1], v_bl[2]), Vector3() )
            true_velocity_frontLeftWheel_bodyFrame = Twist( Vector3(v_fl[0], v_fl[1], v_fl[2]), Vector3() )
            true_velocity_backRightWheel_bodyFrame = Twist( Vector3(v_br[0], v_br[1], v_br[2]), Vector3() )
            true_velocity_frontRightWheel_bodyFrame = Twist( Vector3(v_fr[0], v_fr[1], v_fr[2]), Vector3() )
            
            true_velocity_bodyFrame.header.frame_id = "base_link"
            true_velocity_bodyFrame.header.stamp = rospy.Time.now()
            true_velocity_bodyFrame.velocity = [ true_velocity_baseLink_bodyFrame, true_velocity_backLeftWheel_bodyFrame, true_velocity_frontLeftWheel_bodyFrame, true_velocity_backRightWheel_bodyFrame, true_velocity_frontRightWheel_bodyFrame ]
            
            pub_true_velocity_bodyFrame.publish( true_velocity_bodyFrame )

        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print( "[wheelKinematics.py] Something went wrong!" )
            continue
    
    rospy.spin()