#!/usr/bin/python3.8

import sys
sys.path.insert(0, "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/src")

from classes.all_imports import *

""" 
    Note (this is the typical order):   bl = back left wheel
                                        fl = front left wheel
                                        br = back right wheel
                                        fr = front right wheel
""" 

class Common:

    robot = "leo"

    if( robot == "leo" ):
        vehicle_specs = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/config/leo_rover.json"
    
    elif( robot == "pioneer3at" ):
        #   Pioneer 3AT parameters
        vehicle_specs = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/config/pioneer3at.json"
    
    elif(robot == "steering_rover"):
        vehicle_specs = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/config/steering_rover.json"
    
    elif(robot == "lamborghini"):
        vehicle_specs = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/config/lamborghini.json"

    """
        Convex inner approximation iteration parameters
    """
    refStep = 0.2
    minDistance2Obstacle = 1
    maxDistance2Obstacle = 4
    maxIters = 10

    """
        Algorithm parameters
    """ 
    gz = -9.81                                                          #   Gravity
    corFactor = 0.732166                                                #   Correction factor

    """     Elevation grids and cost maps folders   """
    
    #   Results folder
    results_folder = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/Resultados/"
    
    #   Raw digital elevation model data
    mapFile = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_gazebo/models/marsyard2021_terrain/dem/marsyard_terrain_hm.tif"
    #mapFile = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_gazebo/models/marsyard2020_terrain/marsyard2020_terrain.png"
    #mapFile = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_gazebo/models/marsyard2022_terrain/dem/marsyard2022_terrain_hm.tif"

    #   Directory where all elevation grids and traversability maps are saved with .npy format
    mapFolderDir = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/Resultados/costmaps/"

    #   Directory where data is exported for publication
    exportData2PubDir = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/mscthesis/mscthesis_pdf/Data/"

    #   Directory where plots are exported for publication
    exportPlot2PubDir = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/mscthesis/mscthesis_pdf/Figures/"
    
    #   Folder to save maps for each environment
    mapFolder = 1                                                       #   1 -> mars yard 2021
                                                                        #   2 -> mars yard 2020
                                                                        #   3 -> mars yard 2022
                                                                        #   ...
    
    if( mapFolder == 1 ):
        mapDimx = 47.0                                                      #   Square Map dimension x direction
        mapDimy = 47.0                                                      #   Square Map dimension y direction
        mapHeight = 6.57519871597                                           #   Map maximum height
        maxSpeed = 1.0                                                      #   Maximum speed (minimum cost)
    
    """ .json files with weights and constraint limits """

    #   Integrator for verification purposes
    dynamics_integrator = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/config/DynamicsIntegrator.json"

    #   Standard formulation kinematics
    std_kin_skid = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/config/mpc_kin_std_skid_steering.json"

    #   Standard formulation dynamics
    std_dyn_skid = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/config/mpc_dyn_std_skid_steering.json"
    
    #   Baseline standard formulation kinematics
    baseline_std_kin_skid = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/config/mpc_kin_baseline_std_skid_steering.json"

    #   Baseline standard formulation dynamics
    baseline_std_dyn_skid = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/config/mpc_dyn_baseline_std_skid_steering.json"

    #   Frenet-Serret frame formulation kinematics
    frenet_serret_kin_skid = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/config/mpcc_kin_skidsteering_frenet.json"

    #   Frenet-Serret frame formulation dynamics
    frenet_serret_dyn_skid = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/config/mpcc_dyn_skidsteering_frenet.json"

    #   CIAO parameters
    ciao_parameters = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/config/ciao_parameters.json"

    #   Obstacles
    list_obstacles = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/config/obstacles.json"

    #   Potential field
    potential_field = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/config/potential_field_parameters.json"

    """ Cost Function choice """
    costFunction = 0                                                    #   choose cost function

    optSolver = 'ipopt'

    if(optSolver == 'ipopt'):

        optOptions = { 'warn_initial_bounds': True,
                       'verbose': False,
                       'verbose_init': False,
                       'print_time': False,
                       'print_in': False,
                       'print_out': False,
                       'record_time': True,
                       'ipopt': {'print_level': 2},
                       'error_on_fail': False,
                       'jit': True,
                       'compiler': 'shell',
                       'jit_options': { 'flags': ["-O3", "-pipe"], 'verbose': False, 'compiler': 'gcc' },
                       'calc_multipliers': False,
                       'enable_jacobian': False,
                       'calc_f': False,
                       'calc_g': False }
    
        #dump_dir: "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/Resultados/HeightMinimization"
    
    elif(optSolver == 'qrsqp' or optSolver == 'sqpmethod'):
    
        optOptions = { 'max_iter': 3000,
                       'verbose': True,
                       'verbose_init': False,
                       'print_time': False,
                       'print_in': False,
                       'print_out': False,
                       'qpsol': 'qpoases',
                       'record_time': True,
                       'convexify_strategy': 'regularize',      #    Helps to avoid indefinite Hessians   https://github.com/casadi/casadi/issues/567
                       'print_iteration': False,
                       'print_header': False,
                       'qpsol_options': { 'jit': True,
                                          'jit_options': { 'compiler': 'gcc', 'flags': ['-O3', '-pipe'] },
                                          'compiler': 'shell',                   
                                          'print_time': True,
                                          'printLevel': '1' },
                       'expand': False }

    """ Path planning """
    pathGap = 0.05                                                      #   distance between gaps on path planning
    path = False                                                       #   path (True); trajectory (False)
    option1 = "mapRefinement"
    option3 = "Points"

    """ Goal position (x, y)    """
    goalPoint = [14.0, 12.0]                                     

    """ Goal check """
    goalCheck = 1.0                                  

    def __init__( self ):

        #   Pose
        """self.gazeboLinkStates = LinkStates()
        
        self.localOdometry = Odometry()
        self.globalOdometry = Odometry()

        self.true_pose3D = pose3DStamped()
        self.noisy_pose3D = pose3DStamped()"""

        ###
        
        #   Actuations
        #self.velocityActuation = Twist()

        #   Link velocity expressed in body frame
        """self.true_velocity_bodyFrame = wheelTrueVelocitiesBodyFrame()
        self.noisy_velocity_bodyFrame = wheelTrueVelocitiesBodyFrame()"""

        #   
        """self.backLeftWheelContactLoad = Float64()
        self.frontLeftWheelContactLoad = Float64()
        self.backRightWheelContactLoad = Float64()
        self.frontRightWheelContactLoad = Float64()"""

        #
        """self.backLeftWheelActuation = Float64()
        self.frontLeftWheelActuation = Float64()
        self.backRightWheelActuation = Float64()
        self.frontRightWheelActuation = Float64()
        """

        #
        #self.jointStates = JointState()

        #
        self.error = [0, 0, 0]
        
    #   Topics message subscribers
    def _callback( self, msg, option ):

        #   Type gazebo_msgs/LinkStates.msg, from Gazebo
        if( option == 0 ):
            self.gazeboLinkStates = msg

        #   Type nav_msgs/Odometry.msg, fused Pose
        elif( option == 1 ):
            self.localOdometry = msg

        elif( option == 2 ):
            self.globalOdometry = msg

        #   Type thesis_main/pose3D_stamped.msg, robot pose expressed on aircraft angles (roll, pitch, yaw)
        elif( option == 3 ):
            self.true_pose3D = msg
        
        elif( option == 4 ):
            self.noisy_pose3D = msg
        
        #   Velocity actuation with type geometry_msgs/Twist
        elif( option == 5 ):
            self.velocityActuation = msg
        
        elif( option == 6 ):
            self.true_velocity_bodyFrame = msg
        
        elif( option == 7 ):
            self.noisy_velocity_bodyFrame = msg
        
        elif( option == 8 ):
            self.jointStates = msg
        
        elif( option == 9 ):
            self.backLeftWheelContactLoad = msg
        
        elif( option == 10 ):
            self.frontLeftWheelContactLoad = msg
        
        elif( option == 11 ):
            self.backRightWheelContactLoad = msg
        
        elif( option == 12 ):
            self.frontRightWheelContactLoad = msg
        
        elif( option == 13 ):
            self.wheelLoad = msg
        
        elif( option == 14 ):
            self.wheelSlip = msg
        
        elif( option == 15 ):
            self.wheelSlipAngle = msg

        elif( option == 16 ):
            self.toInitialState = msg
        
        elif( option == 17 ):
            self.terramechanicsSwitch = msg
        
        elif( option == 22 ):
            self.obstaclesPcl = msg

        #   Subscribe to Float32MultiArray msg with obstacles list with each element of type Point32
        elif( option == 23 ):
            #   List of lists with each list of type [x, y, z]
            self.obstaclesList = msg
            
        #   Subscribe to NMPC states solution
        elif( option == 28 ):
            #   List concatenation of [x, y, z, roll, pitch, yaw]
            self.nmpcStatesSolution = msg
        
        #   Subscribe to NMPC control solution
        elif( option == 29 ):
            #   List concatenation of [ux, uy, uz, u_roll, u_pitch, u_yaw]
            self.nmpcControlSolution = msg
        
        elif( option == 30 ):
            self.gzPerformance = msg
        
        elif( option == 31 ):
            self.clock = msg
        
        #   Wheel contact forces
        elif( option == 32 ):
            self.wheelForces = msg
        
        #   Robot inertia
        elif( option == 33 ):
            self.inertia = msg
    
    def _wheelLoad(self, msg, option):

        #   back left wheel load
        if( option == 0 ):
            self.load_bl = msg

        #   front left wheel load
        elif( option == 1 ):
            self.load_fl = msg

        #   back right wheel load
        elif( option == 2 ):
            self.load_br = msg

        #   front right wheel load
        elif( option == 3 ):
            self.load_fr = msg
    
    def _wheelRate(self, msg, option):

        #   back left wheel rate
        if( option == 0 ):
            self.rate_bl = msg

        #   front left wheel rate
        elif( option == 1 ):
            self.rate_fl = msg

        #   back right wheel rate
        elif( option == 2 ):
            self.rate_br = msg

        #   front right wheel rate
        elif( option == 3 ):
            self.rate_fr = msg

    def _linkWrenchCallback(self, msg, option):

        #   Base link
        if( option == 0 ):
            self.base_link_wrench = msg
        
        #   Back left wheel
        elif( option == 1 ):
            self.back_left_wrench = msg

        #   Front left wheel
        elif( option == 2 ):
            self.front_left_wrench = msg

        #   Back right wheel
        elif( option == 3 ):
            self.back_right_wrench = msg

        #   Front right wheel
        elif( option == 4 ):
            self.front_right_wrench = msg

    def _multiArrayCallback(self, msg, option):

        #   List concatenation of [x, y, z, roll, pitch, yaw]
        if(option == 0 ):
            self.referencePath = msg
        
        #   List concatenation of [x, y, z, roll, pitch, yaw]
        elif(option == 1 ):
            self.horizonPath = msg
        
        #   List concatenation of [vx, vy, vz, wx, wy, wz]
        elif(option == 2):
            self.horizonVelocity = msg
        
        #   List concatenation of [fx, mz]
        elif(option == 3):
            self.horizonForcesMoments = msg
        
        #   List concatenation of contact status [bl, fl, br, fr]
        elif( option == 4 ):
            self.contactStatus = msg
        
        #   List concatenation of contact angles [bl, fl, br, fr]
        elif( option == 5 ):
            self.contactAngles = msg
        
    def _wheelTorqueInputCallback( self, msg, option ):

        if( option == 0 ):
            self.backLeftWheelTorque = msg
        
        elif( option == 1 ):
            self.frontLeftWheelTorque = msg
        
        elif( option == 2 ):
            self.backRightWheelTorque = msg
        
        elif( option == 3 ):
            self.frontRightWheelTorque = msg
    
    def _contactCallback(self, msg, option ):

        if( option == 0 ):
            self.backLeftWheelContact = msg
        
        elif( option == 1 ):
            self.frontLeftWheelContact = msg

        elif( option == 2 ):
            self.backRightWheelContact = msg

        elif( option == 3 ):
            self.frontRightWheelContact = msg

    def _syncSSECallback( self, topic1, topic2 ):

        deltaRoll = topic1.pose.roll - topic2.pose.roll
        deltaRoll = self._checkAngleDiff(deltaRoll)

        deltaPitch = topic1.pose.pitch - topic2.pose.pitch
        deltaPitch = self._checkAngleDiff(deltaPitch)

        deltaYaw = topic1.pose.yaw - topic2.pose.yaw
        deltaYaw = self._checkAngleDiff(deltaYaw)

        self.error = [ math.pow(topic1.pose.x - topic2.pose.x, 2), math.pow(topic1.pose.y - topic2.pose.y, 2), math.pow(topic1.pose.z - topic2.pose.z, 2), \
                       math.pow(deltaRoll, 2), math.pow(deltaPitch, 2), math.pow(deltaYaw, 2) ]
    
    def _getVehicleProperties( self, worldProperties, modelPropertiesService ):

        for model in worldProperties.model_names:

            if( model == "vehicle" ):
                vehicleProperties = modelPropertiesService( model )
        
        return vehicleProperties
    
    def _getVehicleMass( self, vehicleProperties, vehiclePropertiesService ):

        vehicleMass = 0

        for link in vehicleProperties.body_names:
            linkProperties = vehiclePropertiesService( link )
            vehicleMass += linkProperties.mass
        
        return vehicleMass

    def _computeCOM( self, vehicleProperties, vehiclePropertiesService, buffer ):

        """
            Update robot center of mass and inertia tensor
        """

        vehicleMass = 0
        
        sum_x = 0
        sum_y = 0
        sum_z = 0

        for link in vehicleProperties.body_names:
            linkProperties = vehiclePropertiesService( link )
            vehicleMass += linkProperties.mass

            trans = buffer.lookup_transform("base_link", link, rospy.Time(), rospy.Duration(5))

            rotation_matrix = R.from_quat( [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w] )

            com_position = rotation_matrix.as_matrix().T @ np.array( [linkProperties.com.position.x, linkProperties.com.position.y, linkProperties.com.position.z] )
        
            sum_x += linkProperties.mass * (com_position[0] + trans.transform.translation.x)
            sum_y += linkProperties.mass * (com_position[1] + trans.transform.translation.y)
            sum_z += linkProperties.mass * (com_position[2] + trans.transform.translation.z)
        
        #   Compute center of mass
        com_x = sum_x / vehicleMass
        com_y = sum_y / vehicleMass
        com_z = sum_z / vehicleMass

        robotInertia = Inertia()

        robotInertia.m = vehicleMass
        robotInertia.com.x = com_x
        robotInertia.com.y = com_y
        robotInertia.com.z = com_z

        for link in vehicleProperties.body_names:
            linkProperties = vehiclePropertiesService( link )

            trans = buffer.lookup_transform("base_link", link, rospy.Time(), rospy.Duration(5))

            rotation_matrix = R.from_quat( [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w] )
            
            com_position = rotation_matrix.as_matrix().T @ np.array( [linkProperties.com.position.x, linkProperties.com.position.y, linkProperties.com.position.z] )

            r_x = com_x - (com_position[0] + trans.transform.translation.x)
            r_y = com_y - (com_position[1] + trans.transform.translation.y)
            r_z = com_z - (com_position[2] + trans.transform.translation.z)

            #   Parallel axis theorem
            robotInertia.ixx += linkProperties.ixx + linkProperties.mass * ( math.pow(r_y, 2) + math.pow(r_z, 2) )
            robotInertia.iyy += linkProperties.iyy + linkProperties.mass * ( math.pow(r_z, 2) + math.pow(r_x, 2) )
            robotInertia.izz += linkProperties.izz + linkProperties.mass * ( math.pow(r_x, 2) + math.pow(r_y, 2) )
            robotInertia.ixy += linkProperties.ixy - linkProperties.mass * r_x * r_y
            robotInertia.iyz += linkProperties.iyz - linkProperties.mass * r_y * r_z
            robotInertia.ixz += linkProperties.ixz - linkProperties.mass * r_x * r_z

        return robotInertia
    
    def _getVehicleVerticalInertia( self, vehicleProperties, vehiclePropertiesService ):

        Izz = 0

        for link in vehicleProperties.body_names:
            linkProperties = vehiclePropertiesService( link )
            Izz += linkProperties.izz
        
        return Izz

    def _getWheelLateralInertia( self, vehicleProperties, vehiclePropertiesService ):

        for link in vehicleProperties.body_names:

            if( link[-5:] == "wheel" ):
                linkProperties = vehiclePropertiesService( link )
                Iyy = linkProperties.iyy

                return Iyy
    
    def _getLinksIndex( self, links ):

        linkIndex = 0

        for link in links.name:
            if( "base_link" in link ):
                baseLinkIndex = linkIndex
                
            elif( "back_left_wheel" in link ):
                backLeftIndex = linkIndex
            
            elif( "front_left_wheel" in link ):
                frontLeftIndex = linkIndex
            
            elif( "back_right_wheel" in link ):
                backRightIndex = linkIndex

            elif( "front_right_wheel" in link ):
                frontRightIndex = linkIndex

            linkIndex += 1

        return baseLinkIndex, backLeftIndex, frontLeftIndex, backRightIndex, frontRightIndex
    
    def _getJointStatesIndex( self, jointStates ):

        index = 0

        for joint in jointStates.name:

            if( joint == "back_left_wheel_joint" ):
                backLeftWheelJointIndex = index
            
            elif( joint == "front_left_wheel_joint" ):
                frontLeftWheelJointIndex = index
            
            elif( joint == "back_right_wheel_joint" ):
                backRightWheelJointIndex = index
            
            elif( joint == "front_right_wheel_joint" ):
                frontRightWheelJointIndex = index

            index += 1
    
        return backLeftWheelJointIndex, frontLeftWheelJointIndex, backRightWheelJointIndex, frontRightWheelJointIndex
    
    def _getLinksVelocitiesIndex( self, wheelVelocities ):

        index = 0

        for wheel in wheelVelocities.wheel:

            if( wheel == "base_link" ):
                baseLinkIndex = index
            
            elif( wheel == "back_left_wheel" ):
                backLeftWheelIndex = index
            
            elif( wheel == "front_left_wheel" ):
                frontLeftWheelIndex = index
            
            elif( wheel == "back_right_wheel" ):
                backRightWheelIndex = index
            
            elif( wheel == "front_right_wheel" ):
                frontRightWheelIndex = index

            index += 1

        return baseLinkIndex, backLeftWheelIndex, frontLeftWheelIndex, backRightWheelIndex, frontRightWheelIndex

    def _cmdVelocity2JointVelocity(self, vx_cmd, wz_cmd, rocker_l_angle = 0, rocker_r_angle = 0 ):
        
        if( self.robot == "pioneer3at" ):
            res = np.array( [ [ 1 / ( self.wheelRadius ), self.wheelLatSeparation / ( 2 * self.wheelRadius ) ],\
                              [ 1 / ( self.wheelRadius ), -self.wheelLatSeparation / ( 2 * self.wheelRadius ) ] ] ) @ np.array( [ vx_cmd, wz_cmd ] )
        
        elif( self.robot == "leo" ):
            res = np.array( [ [ 1 / ( math.cos(rocker_r_angle) * self.wheelRadius ), self.wheelLatSeparation / ( 2 * self.wheelRadius * math.cos(rocker_r_angle) ) ],\
                              [ 1 / ( math.cos(rocker_l_angle) * self.wheelRadius ), -self.wheelLatSeparation / ( 2 * self.wheelRadius * math.cos(rocker_l_angle) ) ] ] ) @ np.array( [ vx_cmd, wz_cmd ] )

        w_r_y = res[0]
        w_l_y = res[1]

        return w_r_y, w_l_y
    
    def _fixAngle(self, vx, vy, angle):

        if( vx > 0 and vy > 0 ):
            pass
        
        elif( vx > 0 and vy < 0 ):
            angle = -angle
        
        elif( vx < 0 and vy > 0 ):
            angle = -(math.pi - angle)
        
        elif( vx < 0 and vy < 0 ):
            angle = math.pi - angle

        return angle

    def _ackermanSteering(self, delta_cmd):

        R = self.wheelLonSeparation / (2 * math.tan(delta_cmd) )

        delta_fl = math.atan( self.wheelLonSeparation / (2 * (R - self.wheelLatSeparation / 2) ) )
        delta_fr = math.atan( self.wheelLonSeparation / (2 * (R + self.wheelLatSeparation / 2) ) )

        return delta_fl, delta_fr

    def xyz_rotationMatrix(self, roll, pitch, yaw):

        """
            3D rotation matrix from body frame to inertial frame
        """
    
        matrix = np.array( [ [ math.cos( yaw ) * math.cos( pitch ), math.cos( yaw ) * math.sin( pitch ) * math.sin( roll ) - math.sin( yaw ) * math.cos( roll ), math.cos( yaw ) * math.sin( pitch ) * math.cos( roll ) + math.sin( yaw ) * math.sin( roll ) ],\
                             [ math.sin( yaw ) * math.cos( pitch ), math.sin( yaw ) * math.sin( pitch ) * math.sin( roll ) + math.cos( yaw ) * math.cos( roll ), math.sin( yaw ) * math.sin( pitch ) * math.cos( roll ) - math.cos( yaw ) * math.sin( roll ) ],\
                             [ -math.sin( pitch ), math.cos( pitch ) * math.sin( roll ), math.cos( pitch ) * math.cos( roll ) ] ] )

        return matrix

    def _3D_rotationMatrix( self, pose ):

        """
            3D rotation matrix from body frame to inertial frame
        """

        matrix = np.array( [ [ math.cos( pose.yaw ) * math.cos( pose.pitch ), math.cos( pose.yaw ) * math.sin( pose.pitch ) * math.sin( pose.roll ) - math.sin( pose.yaw ) * math.cos( pose.roll ), math.cos( pose.yaw ) * math.sin( pose.pitch ) * math.cos( pose.roll ) + math.sin( pose.yaw ) * math.sin( pose.roll ) ],\
                             [ math.sin( pose.yaw ) * math.cos( pose.pitch ), math.sin( pose.yaw ) * math.sin( pose.pitch ) * math.sin( pose.roll ) + math.cos( pose.yaw ) * math.cos( pose.roll ), math.sin( pose.yaw ) * math.sin( pose.pitch ) * math.cos( pose.roll ) - math.cos( pose.yaw ) * math.sin( pose.roll ) ],\
                             [ -math.sin( pose.pitch ), math.cos( pose.pitch ) * math.sin( pose.roll ), math.cos( pose.pitch ) * math.cos( pose.roll ) ] ] )
    
        return matrix

    def _euler2Quat(self, sequence):

        """ 
            Roll pitch yaw to quaternion conversion (roll-pitch-yaw convention)

            :sequence [roll, pitch, yaw]

            :quaternion [qw, qx, qy, qz] of numpy type
        """

        roll = sequence[0]
        pitch = sequence[1]
        yaw = sequence[2]

        cy = math.cos(yaw / 2)
        sy = math.sin(yaw / 2)

        cp = math.cos(pitch / 2)
        sp = math.sin(pitch / 2)

        cr = math.cos(roll / 2)
        sr = math.sin(roll / 2)

        qw = cy * cp * cr + sy * sp * sr
        qx = cy * cp * sr - sy * sp * cr
        qy = cy * sp * cr + sy * cp * sr
        qz = sy * cp * cr - cy * sp * sr
    
        return np.array( [qw, qx, qy, qz] )

    def _checkAngleDiff(self, diff):

        while(1):
            if( diff > math.pi ):
                diff -= 2 * math.pi

            elif( diff < -math.pi ):
                diff += 2 * math.pi
            
            else:
                break

        return diff

    def _signal_handler(self, sig, frame, node):

        print(node + ': CTRL-C was pressed. Node is shutdown.')
        sys.exit(0)

    def skew(self, vector):
        """
        this function returns a numpy array with the skew symmetric cross product matrix for vector.
        the skew symmetric cross product matrix is defined such that
        np.cross(a, b) = np.dot(skew(a), b)

        :param vector: An array like vector to create the skew symmetric cross product matrix for
        :return: A numpy array of the skew symmetric cross product vector
        """

        return np.array( [ [0, -vector[2], vector[1] ], 
                           [vector[2], 0, -vector[0] ], 
                           [-vector[1], vector[0], 0 ] ] )