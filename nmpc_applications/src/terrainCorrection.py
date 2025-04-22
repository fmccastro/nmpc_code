#!/usr/bin/python3.8

from pathlib import Path

classes_path = Path(__file__)

from sys import path
path.append( str(classes_path) )

from classes.model_class_acados import *
from classes.common_class import *

if __name__ == '__main__':

    #   Start node
    rospy.init_node('terrainCorrection', anonymous = True, disable_signals = True)

    common = Common()

    if( common.simulationType < 2 ):
        rospy.signal_shutdown("[" + rospy.get_name() + "] Shut down node.")

    pub_nodePeriod = rospy.Publisher( '/vehicle/node/terrainCorrection/period', Float64, queue_size = 1 )

    #   Publishers
    pub_backLeftWheelRate = rospy.Publisher('/back_left_wheel_plant/command', Float64, queue_size = 1)
    pub_frontLeftWheelRate = rospy.Publisher('/front_left_wheel_plant/command', Float64, queue_size = 1)
    pub_backRightWheelRate = rospy.Publisher('/back_right_wheel_plant/command', Float64, queue_size = 1)
    pub_frontRightWheelRate = rospy.Publisher('/front_right_wheel_plant/command', Float64, queue_size = 1)

    #   Subscribe to ground truth
    rospy.Subscriber( '/gazebo/link_states', LinkStates, common._callback, 0 )                                           #   '/gazebo/link_states' -> topic which collects ground truth
    rospy.Subscriber( '/vehicle/true_pose3D', pose3DStamped, common._callback, 3 )                                       #   '/vehicle/truePose' -> topic which collects robot perfect pose
    rospy.Subscriber( '/vehicle/true_velocity_bodyFrame', wheelTrueVelocitiesBodyFrame, common._callback, 6 )            #   '/vehicle/trueVelocity_bodyFrame' -> topic which collect robot links perfect velocity
    rospy.Subscriber( '/vehicle/nmpc_dynamics/fx_l', Float32, common._wheelTorqueInputCallback, 0 )
    rospy.Subscriber( '/vehicle/nmpc_dynamics/fx_r', Float32, common._wheelTorqueInputCallback, 2  )
    rospy.Subscriber( '/back_left_wheel_contact', ContactsState, common._contactCallback, 0 )
    rospy.Subscriber( '/front_left_wheel_contact', ContactsState, common._contactCallback, 1 )
    rospy.Subscriber( '/back_right_wheel_contact', ContactsState, common._contactCallback, 2 )
    rospy.Subscriber( '/front_right_wheel_contact', ContactsState, common._contactCallback, 3 )
    rospy.Subscriber( '/joint_states', JointState, common._callback, 8 )                                                 #   '/vehicle/joint_states' -> topic which collects the joint position and velocity ( linear or angular )

    rospy.wait_for_message( '/gazebo/link_states', LinkStates )
    rospy.wait_for_message( '/vehicle/true_pose3D', pose3DStamped )
    rospy.wait_for_message( '/vehicle/true_velocity_bodyFrame', wheelTrueVelocitiesBodyFrame )
    rospy.wait_for_message('/vehicle/nmpc_dynamics/fx_l', Float32)
    rospy.wait_for_message('/vehicle/nmpc_dynamics/fx_r', Float32)
    rospy.wait_for_message('/back_left_wheel_contact', ContactsState )
    rospy.wait_for_message('/front_left_wheel_contact', ContactsState )
    rospy.wait_for_message('/back_right_wheel_contact', ContactsState )
    rospy.wait_for_message('/front_right_wheel_contact', ContactsState )
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

    #   Get wheel force correction qp optimization model
    model = WheelTorqueAllocation_qp(com2wheels)
    
    index = 0

    ###

    #   Register signal handler
    signal_handler = partial(common._signal_handler, node = "[" + rospy.get_name() + "]")
    signal.signal(signal.SIGINT, signal_handler)
    ###

    print("[" + rospy.get_name() + "] Loop is running.")

    while not rospy.is_shutdown():
        try:
            start = time.time()
            
            #   Get normals
            contact_bl = common.backLeftWheelContact
            contact_fl = common.frontLeftWheelContact
            contact_br = common.backRightWheelContact
            contact_fr = common.frontRightWheelContact

            #   Get current pose
            currentPose = common.true_pose3D.pose

            #   Get longitudinal reference forces
            fx_ref_l = common.backLeftWheelTorque.data
            fx_ref_r = common.backRightWheelTorque.data

            robotWeight = common._3D_rotationMatrix(currentPose).T @ np.array([0, 0, 1]) * common.gz

            ###
            angle = []
            if( len(contact_bl.states) > 0 ):
                for contact in contact_bl.states:
                    if(contact.info == "ON"):
                        for normal in contact.contact_normals:
                            normal_body = common._3D_rotationMatrix(currentPose).T @ np.array( [normal.x, normal.y, normal.z] )

                            angle += [ math.acos( np.dot( normal_body, np.array( [0, 0, 1] ) ) / np.linalg.norm(normal_body) ) ]
                    
                        if( len(angle) > 0 ):
                            index_bl = angle.index( max(angle) )
                            angle_bl = angle[index_bl]

                            contact_status_bl = 1
                        
                        else:
                            angle_bl = 0.0
                            contact_status_bl = 0

                    else:
                        angle_bl = 0.0
                        contact_status_bl = 0
            
            else:
                angle_bl = 0.0
                contact_status_bl = 0

            ###
            angle = []
            if( len(contact_fl.states) > 0 ):
                for contact in contact_fl.states:
                    if(contact.info == "ON"):

                        for normal in contact.contact_normals:
                            normal_body = common._3D_rotationMatrix(currentPose).T @ np.array( [normal.x, normal.y, normal.z] )

                            angle += [ math.acos( np.dot( normal_body, np.array( [0, 0, 1] ) ) / np.linalg.norm(normal_body) ) ]

                        if( len(angle) > 0 ):
                            index_fl = angle.index( max(angle) )
                            angle_fl = angle[index_fl]

                            contact_status_fl = 1
                        
                        else:
                            angle_fl = 0.0
                            contact_status_fl = 0

                    else:
                        angle_fl = 0.0
                        contact_status_fl = 0
            else:
                angle_fl = 0.0
                contact_status_fl = 0

            ###
            angle = []
            if( len(contact_br.states) > 0 ):
                for contact in contact_br.states:
                    if(contact.info == "ON"):

                        for normal in contact.contact_normals:
                            normal_body = common._3D_rotationMatrix(currentPose).T @ np.array( [normal.x, normal.y, normal.z] )

                            angle += [ math.acos( np.dot( normal_body, np.array( [0, 0, 1] ) ) / np.linalg.norm(normal_body) )]

                        if( len(angle) > 0 ):
                            index_br = angle.index( max(angle) )
                            angle_br = angle[index_br]

                            contact_status_br = 1
                        
                        else:
                            angle_br = 0.0
                            contact_status_br = 0

                    else:
                        angle_br = 0.0
                        contact_status_br = 0
            
            else:
                angle_br = 0.0
                contact_status_br = 0

            ###
            angle = []
            if( len(contact_fr.states) > 0 ):
                for contact in contact_fr.states:
                    if(contact.info == "ON"):
                        
                        angle = []

                        for normal in contact.contact_normals:
                            normal_body = common._3D_rotationMatrix(currentPose).T @ np.array( [normal.x, normal.y, normal.z] )

                            angle += [ math.acos( np.dot( normal_body, np.array( [0, 0, 1] ) ) / np.linalg.norm(normal_body) ) ]

                        if( len(angle) > 0 ):
                            index_fr = angle.index( max(angle) )
                            angle_fr = angle[index_fr]

                            contact_status_fr = 1
                        
                        else:
                            angle_fr = 0.0
                            contact_status_fr = 0

                    else:
                        angle_fr = 0.0
                        contact_status_fr = 0
            else:
                angle_fr = 0.0
                contact_status_fr = 0
            
            if( contact_status_bl + contact_status_fl + contact_status_br + contact_status_fr > 0 ):

                """print( [fx_ref_l, fx_ref_r] + [-robotWeight[2] / 4] * 4, [angle_bl, angle_fl, angle_br, angle_fr,\
                                         contact_status_bl, contact_status_fl, contact_status_br, contact_status_fr,\
                                         -robotWeight[2] / 4, fx_ref_l, fx_ref_r] )"""

                res = model._callSolver( [fx_ref_l, fx_ref_r] + [-robotWeight[2] / 4] * 4, [angle_bl, angle_fl, angle_br, angle_fr,\
                                         contact_status_bl, contact_status_fl, contact_status_br, contact_status_fr,\
                                         -robotWeight[2] / 4, fx_ref_l, fx_ref_r])
            
                new_fx_l = res[0]
                new_fx_r = res[1]

                #print("(Fx_l, Fx_ref_l): ", new_fx_l, fx_ref_l)
                #print("(Fx_r, Fx_ref_r): ", new_fx_r, fx_ref_r)

                pub_backLeftWheelRate.publish( common.wheelRadius * new_fx_l )
                pub_frontLeftWheelRate.publish( common.wheelRadius * new_fx_l )
                pub_backRightWheelRate.publish( common.wheelRadius * new_fx_r )
                pub_frontRightWheelRate.publish( common.wheelRadius * new_fx_r )    
                
            else:
                pub_backLeftWheelRate.publish(0.0)
                pub_frontLeftWheelRate.publish(0.0)
                pub_backRightWheelRate.publish(0.0)
                pub_frontRightWheelRate.publish(0.0)  

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