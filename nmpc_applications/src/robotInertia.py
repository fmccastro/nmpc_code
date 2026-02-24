#!/usr/bin/python3

from sys import path
path.append(r"/media/fmccastro/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/src")

from classes.common_class import *

"""
    Get center of mass and inertia tensor
    Publish it on a topic and on rviz with a marker
"""
    
if __name__ == '__main__':

    common = Common()

    rospy.init_node('robotInertia', anonymous = True)

    if( common.simulationType < 2 ):
        rospy.signal_shutdown("[" + rospy.get_name() + "] Shut down node.")

    #   Subscribe to ground truth
    rospy.Subscriber( '/vehicle/true_pose3D', pose3DStamped, common._callback, 3 )                 #   '/vehicle/truePose' -> topic which collects robot perfect pose
    rospy.wait_for_message( '/vehicle/true_pose3D', pose3DStamped )

    pub_inertia = rospy.Publisher( '/vehicle/inertia', Inertia, queue_size = 1 )                   #   '/vehicle/true_velocity_bodyFrame' -> topic for true velocity
    pub_inertia_marker = rospy.Publisher('/markers/inertia', Marker, queue_size = 1)

    pub_gx_marker = rospy.Publisher('/markers/gx', Marker, queue_size = 1)
    pub_gy_marker = rospy.Publisher('/markers/gy', Marker, queue_size = 1)
    pub_gz_marker = rospy.Publisher('/markers/gz', Marker, queue_size = 1)

    #   Services
    rospy.wait_for_service( '/gazebo/get_world_properties' )
    rospy.wait_for_service( '/gazebo/get_model_properties' )
    rospy.wait_for_service( '/gazebo/get_link_properties' )

    get_world_properties = rospy.ServiceProxy( '/gazebo/get_world_properties', GetWorldProperties )
    get_model_properties = rospy.ServiceProxy( '/gazebo/get_model_properties', GetModelProperties )
    get_link_properties = rospy.ServiceProxy( '/gazebo/get_link_properties', GetLinkProperties )

    #   Robot inertia marker (blue shere)
    markerInertia = Marker()
    
    markerInertia.header.stamp = rospy.Time.now()
    markerInertia.ns = "inertia"
    markerInertia.action = 0
    markerInertia.id = 0
    markerInertia.type = 2
    markerInertia.scale = Vector3(0.05, 0.05, 0.05)
    markerInertia.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
    markerInertia.header.frame_id = "base_link"
    ###

    #   Gravity vector marker w.r.t to body frame
    marker_gx = Marker()
    
    marker_gx.header.stamp = rospy.Time.now()
    marker_gx.ns = "gx"
    marker_gx.action = 0
    marker_gx.id = 1
    marker_gx.type = 0
    marker_gx.scale = Vector3(0.05, 0.1, 0.2)
    marker_gx.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
    marker_gx.header.frame_id = "base_link"

    marker_gy = Marker()
    
    marker_gy.header.stamp = rospy.Time.now()
    marker_gy.ns = "gy"
    marker_gy.action = 0
    marker_gy.id = 2
    marker_gy.type = 0
    marker_gy.scale = Vector3(0.05, 0.1, 0.2)
    marker_gy.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
    marker_gy.header.frame_id = "base_link"

    marker_gz = Marker()
    
    marker_gz.header.stamp = rospy.Time.now()
    marker_gz.ns = "gz"
    marker_gz.action = 0
    marker_gz.id = 7
    marker_gz.type = 0
    marker_gz.scale = Vector3(0.05, 0.1, 0.2)
    marker_gz.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
    marker_gz.header.frame_id = "base_link"

    ###
    worldProperties = get_world_properties()

    vehicleProperties = common._getVehicleProperties( worldProperties, get_model_properties )

    print( "[" + rospy.get_name() + "] Simulation cycle is running." )

    while not rospy.is_shutdown():
        try:
            currentPose = common.true_pose3D.pose

            #   Gravity
            gravity = common._3D_rotationMatrix(currentPose).T @ ca.vertcat(0, 0, common.gz)
            robotWeight = gravity * common.robotMass

            tfBuffer = tf2_ros.Buffer()
            listener = tf2_ros.TransformListener(tfBuffer)

            #   Compute robot inertia tensor
            robotInertia = common._computeCOM(vehicleProperties, get_link_properties, tfBuffer)

            markerInertia.header.stamp = rospy.Time.now()
            markerInertia.pose.position = Point(robotInertia.com.x, robotInertia.com.y, robotInertia.com.z)
            markerInertia.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

            marker_gx.header.stamp = rospy.Time.now()
            marker_gy.header.stamp = rospy.Time.now()
            marker_gz.header.stamp = rospy.Time.now()

            marker_gx.points = [ Point(0, 0, 0), Point(robotWeight[0] / ( common.robotMass * abs(common.gz) ), 0, 0) ]
            marker_gy.points = [ Point(0, 0, 0), Point(0, robotWeight[1] / ( common.robotMass * abs(common.gz) ), 0) ]
            marker_gz.points = [ Point(0, 0, 0), Point(0, 0, robotWeight[2] / ( common.robotMass * abs(common.gz) ) ) ]

            pub_inertia.publish(robotInertia)
            pub_inertia_marker.publish(markerInertia)

            pub_gx_marker.publish(marker_gx)
            pub_gy_marker.publish(marker_gy)
            pub_gz_marker.publish(marker_gz)

        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print( "[" + rospy.get_name() + "] Something went wrong." )
            continue
    
    rospy.spin()