#!/usr/bin/python3.8

from pathlib import Path

classes_path = Path(__file__)

from sys import path
path.append( str(classes_path) )

from classes.model_class_acados import *
from classes.common_class import *
from classes.planner_class import *

"""
    Notas: apenas introduzir PID fora do NMPC de atuação do torque das rodas depois de ter o modelo de cinemática, dinâmica, path tracking, slip e obstacle avoidance a funcionar
            isto é um pormenor que apenas acrescenta mais realismo
"""

if __name__ == '__main__':

    #   Start node
    rospy.init_node('nmpc', anonymous = True, disable_signals = True)

    common = Common()

    if( common.simulationType >= 2 ):
        rospy.signal_shutdown("[" + rospy.get_name() + "] Shut down node.")
    
    #   Publishers
    pub_backLeftWheelRate = rospy.Publisher('/back_left_wheel_plant/command', Float64, queue_size = 1)
    pub_frontLeftWheelRate = rospy.Publisher('/front_left_wheel_plant/command', Float64, queue_size = 1)
    pub_backRightWheelRate = rospy.Publisher('/back_right_wheel_plant/command', Float64, queue_size = 1)
    pub_frontRightWheelRate = rospy.Publisher('/front_right_wheel_plant/command', Float64, queue_size = 1)

    if( common.robot == "rover" ):
        pub_backLeftSteeringPosition = rospy.Publisher('/back_left_steering_plant/command', Float64, queue_size = 1)
        pub_frontLeftSteeringPosition = rospy.Publisher('/front_left_steering_plant/command', Float64, queue_size = 1)
        pub_backRightSteeringPosition = rospy.Publisher('/back_right_steering_plant/command', Float64, queue_size = 1)
        pub_frontRightSteeringPosition = rospy.Publisher('/front_right_steering_plant/command', Float64, queue_size = 1)

    pub_nodePeriod = rospy.Publisher( '/vehicle/node/nmpc_kinematics/period', Float64, queue_size = 1 )

    pub_horizonPath = rospy.Publisher( '/vehicle/nmpc_kinematics/horizonPath', Float32MultiArray, queue_size = 1 )
    pub_horizonVelocity = rospy.Publisher( '/vehicle/nmpc_kinematics/horizonVelocity', Float32MultiArray, queue_size = 1 )

    pub_command_vx = rospy.Publisher('/vehicle/nmpc_kinematics/vx', Float32, queue_size = 1)
    pub_command_wz = rospy.Publisher('/vehicle/nmpc_kinematics/wz', Float32, queue_size = 1)

    #   Subscriptions
    rospy.Subscriber( '/gazebo/link_states', LinkStates, common._callback, 0 )                                              #   '/gazebo/link_states' -> topic which collects ground truth
    rospy.Subscriber( '/joint_states', JointState, common._callback, 8 )                                                    #   '/vehicle/joint_states' -> topic which collects the joint position and velocity ( linear or angular )  
    rospy.Subscriber( '/vehicle/reference', referencePath, common._multiArrayCallback, 0 )                                  #   '/vehicle/reference' -> topic which collects the reference path   
    rospy.Subscriber( '/vehicle/true_velocity_bodyFrame', wheelTrueVelocitiesBodyFrame, common._callback, 6 )               #   '/vehicle/trueVelocity_bodyFrame' ->topic which collect robot links perfect velocity
    rospy.Subscriber( '/vehicle/true_pose3D', pose3DStamped, common._callback, 3 )                                          #   '/vehicle/truePose' -> topic which collects robot perfect pose
    #rospy.Subscriber( '/vehicle/inertia', Inertia, common._callback, 33 )                                                   #   '/vehicle/inertia' -> topic which collects robot inertia                                             

    rospy.wait_for_message( '/joint_states', JointState )
    rospy.wait_for_message( '/gazebo/link_states', LinkStates )
    rospy.wait_for_message( '/vehicle/reference', referencePath )
    rospy.wait_for_message( '/vehicle/true_velocity_bodyFrame', wheelTrueVelocitiesBodyFrame )
    rospy.wait_for_message( '/vehicle/true_pose3D', pose3DStamped )
    #rospy.wait_for_message( '/vehicle/inertia', Inertia )

    """
        Get center of mass to wheels vector
    """

    #   Path tracking
    if( common.robot == "rover" ):
        model = KinematicsBicycle()

    else:
        model = Kinematics()
    
    index = 0

    #   Define variables
    cmdVel = Twist()

    horizonPath = Float32MultiArray()
    horizonVelocity = Float32MultiArray()
    ###

    #   Register signal handler
    signal_handler = partial(common._signal_handler, node = "[" + rospy.get_name() + "]")
    signal.signal(signal.SIGINT, signal_handler)
    ###

    print("[" + rospy.get_name() + "] Simulation loop is running!")

    while not rospy.is_shutdown():
        try:
            start = time.time()
            
            currentPose = common.true_pose3D.pose
            #robotInertia = common.inertia

            #   Check if goal point is achieved in order to break simulation loop
            if( math.dist( [currentPose.x, currentPose.y], common.goalPoint ) <= common.goalCheck ):
                print("[" + rospy.get_name() + "] Goal point was reached. Simulation ends.")
                break
                
            else:
                if(index == 0):
                    print("[" + rospy.get_name() + "] Starting initial iterations to find a suitable initial guess.")

                    path2Follow = common.referencePath

                    initialPose = path2Follow.startingPose
                    reference = list(path2Follow.reference)

                    model._setReference(reference)
                    model._setInitialGuess(common.N + 1, [initialPose.x, initialPose.y, initialPose.z, initialPose.roll, initialPose.pitch, initialPose.yaw])

                    input("[" + rospy.get_name() + "] Wait for input to start simulation cycle.")

                if( common.nlp_solver_type == 'SQP' ):
                    path2Follow = common.referencePath

                    initialPose = path2Follow.startingPose
                    reference = path2Follow.reference
                    
                    currentVelocity = common.true_velocity_bodyFrame.velocity[0]

                    if( common.robot == "rover" ):
                        next_v, next_delta = model._solve_sqp([initialPose.x, initialPose.y, initialPose.z, initialPose.roll, initialPose.pitch, initialPose.yaw], reference)
                    
                    else:
                        next_v, next_wz = model._solve_sqp([initialPose.x, initialPose.y, initialPose.z, initialPose.roll, initialPose.pitch, initialPose.yaw], reference)

                elif( common.nlp_solver_type == 'SQP_RTI' ):
                    path2Follow = common.referencePath
                    
                    reference = path2Follow.reference

                    model._preparation_sqp_rti(reference)

                    currentPose = common.true_pose3D.pose

                    if( common.robot == "rover" ):
                        next_v, next_delta = model._feedback_sqp_rti([currentPose.x, currentPose.y, currentPose.z, currentPose.roll, currentPose.pitch, currentPose.yaw])

                    else:
                        next_vx, next_wz = model._feedback_sqp_rti([currentPose.x, currentPose.y, currentPose.z, currentPose.roll, currentPose.pitch, currentPose.yaw])

                solutionX, solutionU, cost, optTime = model._data()

                horizonPath.data = solutionX
                horizonVelocity.data = solutionU + solutionU[-2:]
                joint_states = common.jointStates.position

                if( common.simulationType < 2 ):
                    if( common.robot == "rover"):
                        L = common.wheelLonSeparation / 2
                        d = common.wheelLatSeparation / 2

                        icr = L / math.tan(next_delta)

                        #print("(v, delta, ICR): ", next_v, next_delta, icr)

                        delta_fl = math.atan( L / (icr - d) )
                        delta_fr = math.atan( L / (icr + d) )

                        w_fl = next_v * math.sqrt( math.pow(L, 2) + math.pow(icr - d, 2) ) / ( common.wheelRadius * abs(icr) )
                        w_fr = next_v * math.sqrt( math.pow(L, 2) + math.pow(icr + d, 2) ) / ( common.wheelRadius * abs(icr) )

                        pub_backLeftWheelRate.publish( w_fl )
                        pub_frontLeftWheelRate.publish( w_fl )
                        pub_backRightWheelRate.publish( w_fr )
                        pub_frontRightWheelRate.publish( w_fr )

                        pub_backLeftSteeringPosition.publish(-delta_fl)
                        pub_frontLeftSteeringPosition.publish(delta_fl)
                        pub_backRightSteeringPosition.publish(-delta_fr)
                        pub_frontRightSteeringPosition.publish(delta_fr)

                    else:
                        next_wr, next_wl = common._cmdVelocity2JointVelocity(next_vx, next_wz, rocker_l_angle = joint_states[4], rocker_r_angle = joint_states[5])
                        
                        pub_backLeftWheelRate.publish(next_wl)
                        pub_frontLeftWheelRate.publish(next_wl)
                        pub_backRightWheelRate.publish(next_wr)
                        pub_frontRightWheelRate.publish(next_wr)

                pub_command_vx.publish(solutionU[0])
                pub_command_wz.publish(solutionU[1])

                pub_horizonPath.publish(horizonPath)
                pub_horizonVelocity.publish(horizonVelocity)

            timeDiff = time.time() - start

            pub_nodePeriod.publish(timeDiff)

            index += 1

        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print( "[" + rospy.get_name() + "] Something went wrong!" )
    
    pub_backLeftWheelRate.publish(0.0)
    pub_frontLeftWheelRate.publish(0.0)
    pub_backRightWheelRate.publish(0.0)
    pub_frontRightWheelRate.publish(0.0)

    rospy.spin()