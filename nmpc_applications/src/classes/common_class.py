#!/usr/bin/python3.8

import sys
sys.path.insert(0, "/media/fmccastro/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/src")

from classes.all_imports import *

"""
    Note (this is the typical order):   bl = back left wheel
                                        fl = front left wheel
                                        br = back right wheel
                                        fr = front right wheel
"""

class Common:

    """                                                     
        Reference Pose: 0 -> true pose (from Gazebo)
                        1 -> fused pose (from noisy sensors) from robot_localization package
    """
    poseType = 0
    
    """
        Robot parameters
    """
    nbWheels = 4

    #   0: pioneer3at
    #   1: husky
    robot = "pioneer3at"

    if( robot == "husky" ):
        #   Husky parameters
        wheelLatSeparation = 0.5708
        wheelLonSeparation = 0.512
        wheelWidth = 0.05
        robotWidth = 0.67
        robotLength = 0.988
        robotHeight = 0.388
        wheelRadius = 0.165
        robotContactArea = robotWidth * wheelLonSeparation
        
        robotMass = 53.512

        ixx = 9.750858276455656
        ixy = -0.015058500000000002
        ixz = -0.012606628898103725

        iyy = 10.858470047780516
        iyz = -0.015058499999999999
        
        izz = 11.808587634684864

        i_wheel = 0.04411

    elif( robot == "pioneer3at" ):
        #   Pioneer 3AT parameters
        depth = 0.0007
        wheelRadius = 0.111
        loadRadius = wheelRadius - depth
        wheelWidth = 0.04
        wheelLonSeparation = 0.268
        wheelLatSeparation = 0.394
        robotWidth = 0.497
        robotLength = 0.508
        robotHeight = 0.277
        robotContactArea = robotWidth * wheelLonSeparation

        robotMass = 27.4

        ixx = 11.617014975544746
        ixy = 0.0
        #ixz = 0.00013954988745109795
        ixz = 0.0

        iyy = 11.669791786280525
        iyz = 0.0
        
        izz = 11.700973610735778

        #   Wheel joint inertia (experimental value)
        i_wheel = 0.015
        mass_wheel = 1.2

    """                 
        0 -> path-tracking (kinematics)
        1 -> path and trajectory tracking (kinematics + dynamics + wheel forces allocation)
        2 -> add obstacle avoidance
        3 -> add parameter estimation
    """
    simulationType = 1

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
    Ts = 0.1                                                            #   Sampling Time
    fixedTs = True                                                      #   Variable sampling time
    N = 20                                                              #   Control horizon
    maxCycles = 10                                                      #   Maximum number of cycles for look ahead point finder
    intAccuracy = 4                                                     #   Runge-kutta integrator accuracy
    
    NbPosition = 3                                                      #   Number of position variables
    NbOrientation = 3                                                   #   Number of orientation variables

    NbLinVelocity = 3                                                   #   Number of linear velocity variables
    NbAngVelocity = 3                                                   #   Number of angular velocity variables

    NbStates = 6                                                        #   Number of states
    NbControls = 2                                                      #   Number of Controls
    infinite = math.pow(10, 3)                                          #   High value

    gz = -9.81                                                          #   Gravity
    niu = 1.0                                                           #   General component of friction
    corFactor = 0.732166                                                #   Correction factor
    niu_c = niu * corFactor
    joint_friction = 0.1

    """     Elevation grids and cost maps folders   """
    
    #   Results folder
    results_folder = "/media/fmccastro/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/Resultados/"
    
    #   Raw digital elevation model data
    mapFile = "/media/fmccastro/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_gazebo/models/marsyard2021_terrain/dem/marsyard_terrain_hm.tif"
    
    #   Directory where all elevation grids and traversability maps are saved with .npy format
    mapFolderDir = "/media/fmccastro/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/Resultados/costmaps/"

    #   Directory where data is exported for publication
    exportData2PubDir = "/media/fmccastro/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/mscthesis/mscthesis_pdf/Data/"

    #   Directory where plots are exported for publication
    exportPlot2PubDir = "/media/fmccastro/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/mscthesis/mscthesis_pdf/Figures/"
    
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
        
    """ Robot's admissible space set """
    
    #   p = [x, y, z]
    z_min = -5.0  
    z_max = 10.0

    p_lb = [ -mapDimx/2.0, -mapDimy/2.0, z_min ]
    p_ub = [ mapDimx/2.0, mapDimy/2.0, z_max ]
    
    """ Limits on the Robot's Controls """
    
    #   v = (vx, vy, vz) :m/s
    v_lb = [ -100.0, -1000.0, -1000.0 ]
    v_ub = [ 100.0, 1000.0, 1000.0 ]

    #   w = (wx, wy, wz) :m/s
    w_lb = [ -1000.0, -1000.0, -380 * math.pi / 180.0 ] 
    w_ub = [ 1000.0, 1000.0, 380 * math.pi / 180.0 ] 

    #   wheel rate (state) = (w_bl, w_fl, w_br, w_fr) :rad/s
    wheel_lb = [-100.0, -100.0, -100.0, -100.0]
    wheel_ub = [100.0, 100.0, 100.0, 100.0]

    #   velocity derivative (control)
    v_dot_lb = [-10, -10, -10]
    w_dot_lb = [-10, -10, -10]

    v_dot_ub = [10, 10, 10]
    w_dot_ub = [10, 10, 10]

    #   wheel rate (control) : rad/s
    wheel_rate_lb = [-10.0, -10.0]
    wheel_rate_ub = [10.0, 10.0]

    #   forces limits: N
    f_lb = [0, -1000, 0]
    f_ub = [1000, 1000, 1000]

    #   moments limits: Nm
    m_lb = [-1000, -1000, -1000]
    m_ub = [1000, 1000, 1000]

    #   wheel lateral forces limits
    fy_lb = [-999999.0, -999999.0, -999999.0, -999999.0]
    fy_ub = [999999.0, 999999.0, 999999.0, 999999.0]

    #   wheel normal forces limits
    fn_lb = [0.0, 0.0, 0.0, 0.0]
    fn_ub = [999999.0, 999999.0, 999999.0, 999999.0]

    #   torque limits
    torque_lb = [-10.0, -10.0]
    torque_ub = [10.0, 10.0]

    """ Time constraints """
    dt_min = 0.001                                                      #   Minimum time
    dt_max = 0.5                                                        #   Maximum time

    """ Penalty Matrices """
    
    #   Pose cost
    Q_p = 2 * np.diag( [ 1e0, 1e0, 1e0 ] )                                  
    Q_o = 2 * np.diag( [ 1e0, 1e0, 1e0 ] )                                
    
    #   Velocities cost
    Q_v = 2 * np.diag( [ 1e0, 1e2, 1e2 ] )                                 
    Q_w = 2 * np.diag( [ 1e-1, 1e-1, 1e0 ] )
    
    Q_wheel_rate = 2 * np.diag( [5e-1, 5e-1] )        

    #   Forces cost
    Q_f = 2 * np.diag( [1e0, 1e0, 1e0] )                                #   forces cost                 [fx, fy, fz]

    #   Moments cost
    Q_m = 2 * np.diag( [1e0, 1e0, 1e0] )                                #   moments cost                [mx, my, mz]

    #   Wheel torque cost
    Q_torque = 2 * np.diag( [5e0, 5e0] )                                #   wheel torque control cost   [tau_l, tau_r]

    #   Lateral forces cost
    Q_fy = 2 * np.diag( [1e0, 1e0, 1e0, 1e0] )

    #   Normal forces cost
    Q_fn = 2 * np.diag( [1e0, 1e0, 1e0, 1e0] )                          #   Wheel normal forces cost    [fz_bl, fz_fl, fz_br, fz_fr]

    #   SQP or SQP_RTI
    nlp_solver_type = 'SQP'                                                                  


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
    
        #dump_dir: "/media/fmccastro/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/Resultados/HeightMinimization"
    
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
    goalPoint = [20.0, 20.0]                                       

    """ Goal check """
    goalCheck = robotLength / 2.0                                       

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
        
    def _wheelTorqueInputCallback( self, msg, option ):

        if( option == 0 ):
            self.backLeftWheelTorque = msg
        
        elif( option == 1 ):
            self.frontLeftWheelTorque = msg
        
        elif( option == 2 ):
            self.backRightWheelTorque = msg
        
        elif( option == 3 ):
            self.frontRightWheelTorque = msg

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

            if( model[-7:] == "vehicle" ):
                vehicleProperties = modelPropertiesService( model )
        
        return vehicleProperties
    
    def _getVehicleMass( self, vehicleProperties, vehiclePropertiesService ):

        vehicleMass = 0

        for link in vehicleProperties.body_names:
            linkProperties = vehiclePropertiesService( link )
            vehicleMass += linkProperties.mass
        
        return vehicleMass

    def _computeCOM( self, vehicleProperties, vehiclePropertiesService, buffer ):

        vehicleMass = 0
        
        sum_x = 0
        sum_y = 0
        sum_z = 0

        for link in vehicleProperties.body_names:
            linkProperties = vehiclePropertiesService( link )
            vehicleMass += linkProperties.mass

            trans = buffer.lookup_transform("base_link", link, rospy.Time())
        
            sum_x += linkProperties.mass * (linkProperties.com.position.x + trans.transform.translation.x)
            sum_y += linkProperties.mass * (linkProperties.com.position.y + trans.transform.translation.y)
            sum_z += linkProperties.mass * (linkProperties.com.position.z + trans.transform.translation.z)
        
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

            trans = buffer.lookup_transform("base_link", link, rospy.Time())

            r_x = com_x - (linkProperties.com.position.x + trans.transform.translation.x)
            r_y = com_y - (linkProperties.com.position.y + trans.transform.translation.y)
            r_z = com_z - (linkProperties.com.position.z + trans.transform.translation.z)

            #   Parallel axis theorem
            robotInertia.ixx += linkProperties.ixx + linkProperties.mass * ( math.pow(r_y, 2) + math.pow(r_z, 2) )
            robotInertia.iyy += linkProperties.iyy + linkProperties.mass * ( math.pow(r_z, 2) + math.pow(r_x, 2) )
            robotInertia.izz += linkProperties.izz + linkProperties.mass * ( math.pow(r_x, 2) + math.pow(r_y, 2) )
            robotInertia.ixy += linkProperties.ixy - linkProperties.mass * r_x * r_y
            robotInertia.iyz += linkProperties.ixy - linkProperties.mass * r_y * r_z
            robotInertia.ixz += linkProperties.ixy - linkProperties.mass * r_x * r_z

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
            
            if( link.split( "::", 1 )[1] == "base_link" ):
                baseLinkIndex = linkIndex
                
            elif( link.split( "::", 1 )[1] == "back_left_wheel" ):
                backLeftIndex = linkIndex
            
            elif( link.split( "::", 1 )[1] == "front_left_wheel" ):
                frontLeftIndex = linkIndex
            
            elif( link.split( "::", 1 )[1] == "back_right_wheel" ):
                backRightIndex = linkIndex

            elif( link.split( "::", 1 )[1] == "front_right_wheel" ):
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

    def _cmdVelocity2JointVelocity(self, vx_cmd, wz_cmd):
        
        v_r = vx_cmd + wz_cmd * self.wheelLatSeparation / 2
        v_l = vx_cmd - wz_cmd * self.wheelLatSeparation / 2

        w_r_y = v_r / self.wheelRadius
        w_l_y = v_l / self.wheelRadius

        return w_r_y, w_l_y
    
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