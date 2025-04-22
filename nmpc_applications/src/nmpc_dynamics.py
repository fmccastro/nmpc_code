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
    rospy.init_node('nmpc_dynamics', anonymous = True, disable_signals = True)

    common = Common()

    if( common.simulationType < 1 ):
        rospy.signal_shutdown("[" + rospy.get_name() + "] Shut down node.")

    if( common.simulationType == 1 ):
        #   Publishers
        pub_backLeftWheelRate = rospy.Publisher('/back_left_wheel_plant/command', Float64, queue_size = 1)
        pub_frontLeftWheelRate = rospy.Publisher('/front_left_wheel_plant/command', Float64, queue_size = 1)
        pub_backRightWheelRate = rospy.Publisher('/back_right_wheel_plant/command', Float64, queue_size = 1)
        pub_frontRightWheelRate = rospy.Publisher('/front_right_wheel_plant/command', Float64, queue_size = 1)

    pub_nodePeriod = rospy.Publisher( '/vehicle/node/nmpc_dynamics/period', Float64, queue_size = 1 )

    pub_horizonState = rospy.Publisher( '/vehicle/nmpc_dynamics/horizonState', Float32MultiArray, queue_size = 1 )
    pub_horizonForcesMoments = rospy.Publisher( '/vehicle/nmpc_dynamics/horizonForcesMoments', Float32MultiArray, queue_size = 1 )

    pub_command_fx_l = rospy.Publisher('/vehicle/nmpc_dynamics/fx_l', Float32, queue_size = 1)
    pub_command_fx_r = rospy.Publisher('/vehicle/nmpc_dynamics/fx_r', Float32, queue_size = 1)

    #   Subscribe to ground truth
    rospy.Subscriber( '/gazebo/link_states', LinkStates, common._callback, 0 )                                           #   '/gazebo/link_states' -> topic which collects ground truth
    rospy.Subscriber( '/vehicle/true_velocity_bodyFrame', wheelTrueVelocitiesBodyFrame, common._callback, 6 )            #   '/vehicle/trueVelocity_bodyFrame' -> topic which collect robot links perfect velocity
    rospy.Subscriber( '/vehicle/true_pose3D', pose3DStamped, common._callback, 3 )                                       #   '/vehicle/truePose' -> topic which collects robot perfect pose
    rospy.Subscriber( '/vehicle/reference', referencePath, common._multiArrayCallback, 0 )                               #   '/vehicle/reference' -> topic which collects the reference path 
    rospy.Subscriber( '/joint_states', JointState, common._callback, 8 )                                                 #   '/vehicle/joint_states' -> topic which collects the joint position and velocity ( linear or angular )
    
    rospy.wait_for_message( '/gazebo/link_states', LinkStates )
    rospy.wait_for_message( '/vehicle/true_velocity_bodyFrame', wheelTrueVelocitiesBodyFrame )
    rospy.wait_for_message( '/vehicle/true_pose3D', pose3DStamped )
    rospy.wait_for_message( '/vehicle/reference', referencePath )
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
            print( "[nmpc.py] COM and MoI were not computed." )

    #   Trajectory tracking
    model = SimplifiedDynamics(com2wheels)
    integrator = wheelRateIntegrator(com2wheels)
    
    index = 0

    horizonState = Float32MultiArray()
    horizonForcesMoments = Float32MultiArray()
    ###

    #   Register signal handler
    signal_handler = partial(common._signal_handler, node = "[nmpc.py]")
    signal.signal(signal.SIGINT, signal_handler)
    ###

    print("[nmpc_dynamics.py] Simulation loop is running!")

    while not rospy.is_shutdown():
        try:
            start = time.time()
            
            currentPose = common.true_pose3D.pose
            jointStates = common.jointStates.velocity
            
            #   Check if goal point is achieved in order to break simulation loop
            if( math.dist( [currentPose.x, currentPose.y], common.goalPoint ) <= common.goalCheck ):
                print("[" + rospy.get_name() + "] Goal point was reached. Simulation ends.")
                break
                
            else:
                if(index == 0):
                    # do some initial iterations to start with a good initial guess
                    print("[" + rospy.get_name() + "] Starting initial iterations to find a suitable initial guess.")

                    currentPose = common.true_pose3D.pose
                    currentVelocity = common.true_velocity_bodyFrame.velocity[0]

                    path2Follow = common.referencePath

                    initialPose = path2Follow.startingPose
                    reference = list(path2Follow.reference)
                    initialVelocity = path2Follow.startingVelocity

                    initialPose = np.array( [initialPose.x, initialPose.y, initialPose.z, initialPose.roll, initialPose.pitch, initialPose.yaw] )
                    currentVelocity = np.array( [initialVelocity.linear.x, initialVelocity.linear.y, initialVelocity.angular.z] )
                    
                    model._setInitialGuess(10, initialPose, currentVelocity, reference)

                    input("[" + rospy.get_name() + "] Wait for input to start simulation cycle.")
                
                if( common.nlp_solver_type == 'SQP' ):
                    currentPose = common.true_pose3D.pose
                    currentVelocity = common.true_velocity_bodyFrame.velocity[0]

                    path2Follow = common.referencePath

                    initialPose = path2Follow.startingPose
                    reference = list(path2Follow.reference)
                    initialVelocity = path2Follow.startingVelocity

                    initialPose = np.array( [initialPose.x, initialPose.y, initialPose.z, initialPose.roll, initialPose.pitch, initialPose.yaw] )
                    currentVelocity = np.array( [initialVelocity.linear.x, initialVelocity.linear.y, initialVelocity.angular.z] )

                    controls_next, status = model._solve_sqp(initialPose, currentVelocity, reference)

                elif( common.nlp_solver_type == 'SQP_RTI' ):
                    model._preparation_sqp_rti()

                    currentPose = common.true_pose3D.pose
                    currentVelocity = common.true_velocity_bodyFrame.velocity[0]

                    path2Follow = common.referencePath

                    initialPose = path2Follow.startingPose
                    reference = list(path2Follow.reference)
                    initialVelocity = path2Follow.startingVelocity

                    initialPose = np.array( [initialPose.x, initialPose.y, initialPose.z, initialPose.roll, initialPose.pitch, initialPose.yaw] )
                    currentVelocity = np.array( [initialVelocity.linear.x, initialVelocity.linear.y, initialVelocity.angular.z] )

                    controls_next, status = model._feedback_sqp_rti(initialPose, currentVelocity, reference)

                if( index == 0 ):
                    timeDiff = 0.1
                
                else:
                    pass

                solutionX, solutionU, _, _ = model._data()

                horizonState.data = solutionX
                horizonForcesMoments.data = solutionU + solutionU[-6:]

                pub_horizonState.publish(horizonState)
                pub_horizonForcesMoments.publish(horizonForcesMoments)

                pub_command_fx_l.publish(controls_next[0])
                pub_command_fx_r.publish(controls_next[1])
                
                if( common.simulationType == 1 ):
                    pub_backLeftWheelRate.publish(common.wheelRadius * controls_next[0])
                    pub_frontLeftWheelRate.publish(common.wheelRadius * controls_next[0])
                    pub_backRightWheelRate.publish(common.wheelRadius * controls_next[1])
                    pub_frontRightWheelRate.publish(common.wheelRadius * controls_next[1])

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