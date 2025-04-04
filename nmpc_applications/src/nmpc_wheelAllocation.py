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
    rospy.init_node('nmpc_wheelAllocation', anonymous = True, disable_signals = True)

    common = Common()

    if( common.simulationType == 0 ):
        rospy.signal_shutdown("[" + rospy.get_name() + "] Shut down node.")

    #   Publishers
    pub_backLeftWheelRate = rospy.Publisher('/back_left_wheel_plant/command', Float64, queue_size = 1)
    pub_frontLeftWheelRate = rospy.Publisher('/front_left_wheel_plant/command', Float64, queue_size = 1)
    pub_backRightWheelRate = rospy.Publisher('/back_right_wheel_plant/command', Float64, queue_size = 1)
    pub_frontRightWheelRate = rospy.Publisher('/front_right_wheel_plant/command', Float64, queue_size = 1)

    pub_nodePeriod = rospy.Publisher( '/vehicle/node/nmpc_kinematics/period', Float64, queue_size = 1 )

    pub_horizonWheelRates = rospy.Publisher( '/vehicle/nmpc_wheelAllocation/horizonWheelRates', Float32MultiArray, queue_size = 1 )
    pub_horizonForcesMoments = rospy.Publisher( '/vehicle/nmpc_wheelAllocation/horizonForcesMoments', Float32MultiArray, queue_size = 1 )

    pub_command_fz_bl = rospy.Publisher( '/vehicle/nmpc_wheelAllocation/fz_bl', Float32, queue_size = 1 )
    pub_command_fz_fl = rospy.Publisher( '/vehicle/nmpc_wheelAllocation/fz_fl', Float32, queue_size = 1 )
    pub_command_fz_br = rospy.Publisher( '/vehicle/nmpc_wheelAllocation/fz_br', Float32, queue_size = 1 )
    pub_command_fz_fr = rospy.Publisher( '/vehicle/nmpc_wheelAllocation/fz_fr', Float32, queue_size = 1 )

    pub_command_torque_l = rospy.Publisher( '/vehicle/nmpc_wheelAllocation/torque_l', Float32, queue_size = 1 )
    pub_command_torque_r = rospy.Publisher( '/vehicle/nmpc_wheelAllocation/torque_r', Float32, queue_size = 1 )

    pub_bl_wheel_torque = rospy.Publisher( '/back_left_wheel_plant/command', Float64, queue_size=1 )
    pub_fl_wheel_torque = rospy.Publisher( '/front_left_wheel_plant/command', Float64, queue_size=1 )
    pub_br_wheel_torque = rospy.Publisher( '/back_right_wheel_plant/command', Float64, queue_size=1 )
    pub_fr_wheel_torque = rospy.Publisher( '/front_right_wheel_plant/command', Float64, queue_size=1 )

    #   Subscribe to ground truth
    rospy.Subscriber( '/gazebo/link_states', LinkStates, common._callback, 0 )                                           #   '/gazebo/link_states' -> topic which collects ground truth
    rospy.Subscriber( '/vehicle/nmpc_kinematics/horizonPath', Float32MultiArray, common._multiArrayCallback, 1)          #   '/vehicle/nmpc_kinematics/horizonPath'
    rospy.Subscriber( '/vehicle/nmpc_dynamics/horizonVelocity', Float32MultiArray, common._multiArrayCallback, 2)        #   '/vehicle/nmpc_dynamics/horizonVelocity' -> topic which collects control horizon from kinematics problem
    rospy.Subscriber( '/vehicle/nmpc_dynamics/horizonForcesMoments', Float32MultiArray, common._multiArrayCallback, 3)   #   '/vehicle/nmpc_dynamics/horizonForcesMoments' -> topic which collects actuated forces and moments generated at dynamics problem
    rospy.Subscriber( '/vehicle/true_velocity_bodyFrame', wheelTrueVelocitiesBodyFrame, common._callback, 6 )            #   '/vehicle/trueVelocity_bodyFrame' -> topic which collect robot links perfect velocity
    rospy.Subscriber( '/vehicle/true_pose3D', pose3DStamped, common._callback, 3 )                                       #   '/vehicle/truePose' -> topic which collects robot perfect pose

    rospy.wait_for_message( '/gazebo/link_states', LinkStates )
    rospy.wait_for_message( '/vehicle/nmpc_kinematics/horizonPath', Float32MultiArray )
    rospy.wait_for_message( '/vehicle/nmpc_dynamics/horizonVelocity', Float32MultiArray )
    rospy.wait_for_message( '/vehicle/nmpc_dynamics/horizonForcesMoments', Float32MultiArray )
    rospy.wait_for_message( '/vehicle/true_velocity_bodyFrame', wheelTrueVelocitiesBodyFrame )
    rospy.wait_for_message( '/vehicle/true_pose3D', pose3DStamped )

    #   Subscriptions
    rospy.Subscriber( '/joint_states', JointState, common._callback, 8 )                                                    #   '/vehicle/joint_states' -> topic which collects the joint position and velocity ( linear or angular )
    rospy.wait_for_message( '/joint_states', JointState )

    #   Services
    rospy.wait_for_service( '/gazebo/get_world_properties' )
    rospy.wait_for_service( '/gazebo/get_model_properties' )
    rospy.wait_for_service( '/gazebo/get_link_properties' )
    rospy.wait_for_service( '/gazebo/get_physics_properties' )
    rospy.wait_for_service( '/controller_manager/list_controllers' )
    rospy.wait_for_service( '/controller_manager/unload_controller' )
    rospy.wait_for_service( '/controller_manager/switch_controller' )

    get_world_properties = rospy.ServiceProxy( '/gazebo/get_world_properties', GetWorldProperties )
    get_model_properties = rospy.ServiceProxy( '/gazebo/get_model_properties', GetModelProperties )
    get_link_properties = rospy.ServiceProxy( '/gazebo/get_link_properties', GetLinkProperties )
    get_physics_properties = rospy.ServiceProxy( '/gazebo/get_physics_properties', GetPhysicsProperties )
    list_controllers = rospy.ServiceProxy('/controller_manager/list_controllers', ListControllers)
    unload_controller = rospy.ServiceProxy('/controller_manager/unload_controller', UnloadController)
    switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)

    worldProperties = get_world_properties()

    vehicleProperties = common._getVehicleProperties( worldProperties, get_model_properties )

    vehicleMass = common._getVehicleMass( vehicleProperties, get_link_properties )

    physicsProperties = get_physics_properties()

    baseLinkIndex, backLeftIndex, frontLeftIndex, backRightIndex, frontRightIndex = common._getLinksVelocitiesIndex( common.true_velocity_bodyFrame )

    backLeftJointIndex, frontLeftJointIndex, backRightJointIndex, frontRightJointIndex = common._getJointStatesIndex( common.jointStates )

    gt_baseLinkIndex, _, _, _, _ = common._getLinksIndex( common.gazeboLinkStates )

    """
        Set constraints
    """

    tf_flag = True

    while(tf_flag):
        try:
            tfBuffer = tf2_ros.Buffer()
            listener = tf2_ros.TransformListener(tfBuffer)

            d_bl = tfBuffer.lookup_transform('base_link', 'back_left_hub', rospy.Time(0))
            d_fl = tfBuffer.lookup_transform('base_link', 'front_left_hub', rospy.Time(0))
            d_br = tfBuffer.lookup_transform('base_link', 'back_right_hub', rospy.Time(0))
            d_fr = tfBuffer.lookup_transform('base_link', 'front_right_hub', rospy.Time(0))
            
            robotInertia = common._computeCOM(vehicleProperties, get_link_properties, tfBuffer)
            
            com2wheels = { 'com2bl': [d_bl.transform.translation.x - robotInertia.com.x, d_bl.transform.translation.y - robotInertia.com.y, d_bl.transform.translation.z - robotInertia.com.z],\
                           'com2fl': [d_fl.transform.translation.x - robotInertia.com.x, d_fl.transform.translation.y - robotInertia.com.y, d_fl.transform.translation.z - robotInertia.com.z],\
                           'com2br': [d_br.transform.translation.x - robotInertia.com.x, d_br.transform.translation.y - robotInertia.com.y, d_br.transform.translation.z - robotInertia.com.z],\
                           'com2fr': [d_fr.transform.translation.x - robotInertia.com.x, d_fr.transform.translation.y - robotInertia.com.y, d_fr.transform.translation.z - robotInertia.com.z] }
            
            #   Save com2wheels to file
            with open( common.results_folder + "com2wheels_" + common.robot + '.pickle', 'wb') as handle:
                pickle.dump(com2wheels, handle, protocol = pickle.HIGHEST_PROTOCOL)

            tf_flag = False
            
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print( "[" + rospy.get_name() + "] COM and MoI were not computed." )

    #   Trajectory tracking
    model = WheelTorqueAllocation_qp(com2wheels)
    
    index = 0

    horizonWheelRates = Float32MultiArray()
    horizonForcesMoments = Float32MultiArray()
    ###

    #   Register signal handler
    signal_handler = partial(common._signal_handler, node = "[" + rospy.get_name() + "]")
    signal.signal(signal.SIGINT, signal_handler)
    ###

    input("[" + rospy.get_name() + "] Simulation loop is running.")

    while not rospy.is_shutdown():
        try:
            start = time.time()
            
            currentPose = common.true_pose3D.pose
            currentVelocity = common.true_velocity_bodyFrame.velocity[0]
            currentJointStates = common.jointStates.velocity

            #   Check if goal point is achieved in order to break simulation loop
            if( math.dist( [currentPose.x, currentPose.y], common.goalPoint ) <= common.goalCheck ):
                print("[" + rospy.get_name() + "] Goal point was reached. Simulation ends.")
                break
                
            else:
                path2Follow = list(common.horizonPath.data)
                velocity2Follow = list(common.horizonVelocity.data)
                forcesMoments2Follow = list(common.horizonForcesMoments.data)

                #print(path2Follow)
                #print(velocity2Follow)
                #print(forcesMoments2Follow)
                
                #print("[" + rospy.get_name() + "] velocity to follow: ", len(velocity2Follow))
                
                if(index == 0):
                    # do some initial iterations to start with a good initial guess

                    print("[" + rospy.get_name() + "] Starting initial iterations to find a suitable initial guess.")
                    
                    #model._setInitialGuess(5, [currentJointStates[0], currentJointStates[0], currentJointStates[2], currentJointStates[2]], path2Follow, velocity2Follow, forcesMoments2Follow)

                    x = model._callSolver( [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],\
                                           [path2Follow[3], path2Follow[4], velocity2Follow[0], velocity2Follow[1], velocity2Follow[2],\
                                            velocity2Follow[3], velocity2Follow[4], velocity2Follow[5], forcesMoments2Follow[0], forcesMoments2Follow[1], currentJointStates[0], currentJointStates[1], currentJointStates[2], currentJointStates[3] ] )

                    input("[" + rospy.get_name() + "] Wait for input to start simulation cycle.")

                x = model._callSolver( x, [path2Follow[3], path2Follow[4], velocity2Follow[0], velocity2Follow[1], velocity2Follow[2],\
                                            velocity2Follow[3], velocity2Follow[4], velocity2Follow[5], forcesMoments2Follow[0], forcesMoments2Follow[1], currentJointStates[0], currentJointStates[1], currentJointStates[2], currentJointStates[3] ] )
                #solutionX, solutionU, wheelRates, normalForces, wheelTorques = model._solve( [currentJointStates[0], currentJointStates[2]], path2Follow, velocity2Follow, forcesMoments2Follow)
                
                #horizonWheelRates.data = solutionX
                #horizonForcesMoments.data = solutionU + solutionU[-6:]

                #pub_backLeftWheelRate.publish(wheelRates[0])
                #pub_frontLeftWheelRate.publish(wheelRates[0])
                #pub_backRightWheelRate.publish(wheelRates[1])
                #pub_frontRightWheelRate.publish(wheelRates[1])

                """pub_command_fz_bl.publish(normalForces[0])
                pub_command_fz_fl.publish(normalForces[1])
                pub_command_fz_br.publish(normalForces[2])
                pub_command_fz_fr.publish(normalForces[3])

                pub_command_torque_l.publish(wheelTorques[0])
                pub_command_torque_r.publish(wheelTorques[1])"""

                #print("Torques: ", x[4], x[5])

                pub_bl_wheel_torque.publish(x[4])
                pub_fl_wheel_torque.publish(x[4])
                pub_br_wheel_torque.publish(x[5])
                pub_fr_wheel_torque.publish(x[5])

                pub_horizonWheelRates.publish(horizonWheelRates)
                pub_horizonForcesMoments.publish(horizonForcesMoments)

            end = time.time()

            timeDiff = end - start

            pub_nodePeriod.publish(timeDiff)

            index += 1

        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print( "[nmpc_dynamics.py] Something went wrong!" )
    
    pub_backLeftWheelRate.publish(0.0)
    pub_frontLeftWheelRate.publish(0.0)
    pub_backRightWheelRate.publish(0.0)
    pub_frontRightWheelRate.publish(0.0)

    rospy.spin()