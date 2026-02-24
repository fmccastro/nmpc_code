#!/usr/bin/python3.8

from pathlib import Path

classes_path = Path(__file__)

from sys import path
path.append( str(classes_path) )

from classes.model_class_acados import *
from classes.common_class import *

def _getNormalDirection(contacts, currentPose):

    if( len(contacts.states) > 0 ):
        for contact in contacts.states:
            if(contact.info == "ON"):

                sum_weighted_contact = 0
                sum_weighted_normal = 0
                sum_normal_modulus = 0

                #   Get number of contacts for respective wheel
                nb = len(contact.contact_normals)

                for index in range(nb):

                    normal_mod = math.sqrt( math.pow(contact.wrenches[index].force.x, 2) + math.pow(contact.wrenches[index].force.y, 2) + math.pow(contact.wrenches[index].force.z, 2) )
                    
                    sum_weighted_contact += np.array( [contact.contact_positions[index].x, contact.contact_positions[index].y, contact.contact_positions[index].z] ) * normal_mod
                    sum_weighted_normal += np.array( [contact.contact_normals[index].x, contact.contact_normals[index].y, contact.contact_normals[index].z] ) * normal_mod
                    sum_normal_modulus += normal_mod

                if( sum_normal_modulus > 1e-6 ):
                    contact_position = sum_weighted_contact / sum_normal_modulus
                    normal = sum_weighted_normal / sum_normal_modulus

                    normal_body = common._3D_rotationMatrix(currentPose).T @ normal

                    normal_body = np.array( [ normal_body[0], 0, normal_body[2] ] )

                    angle = math.acos( np.dot( normal_body, np.array( [0, 0, 1] ) ) / np.linalg.norm(normal_body) )
                    contact_status = 1

                    if( normal_body[0] > 0 ):
                        pass
                
                    else:
                        angle = -angle
                
                else:
                    angle = 0.0
                    contact_status = 0
                    contact_position = []

            else:
                angle = 0.0
                contact_status = 0
                contact_position = []
    
    else:
        angle = 0.0
        contact_status = 0
        contact_position = []
    
    return angle, contact_position, contact_status

def _getTransform(contact_position, currentPose, angle):

    #   base to contact vector
    res = contact_position - np.array( [currentPose.x, currentPose.y, currentPose.z] )

    #   Get contact to body frame rotation matrix
    aux = R.from_euler('y', angle)
    rotation_contact2body = aux.as_matrix()

    quat = R.from_matrix(rotation_contact2body).as_quat()

    translation = common._3D_rotationMatrix(currentPose).T @ res

    return translation, quat

if __name__ == '__main__':

    #   Start node
    rospy.init_node('broadcast_contactFrames', anonymous = True, disable_signals = True)

    common = Common()

    br = tf.TransformBroadcaster()

    pub_contactStatus = rospy.Publisher( '/contact_status', Int32MultiArray, queue_size = 1 )
    pub_contactAngle = rospy.Publisher( '/contact_angles', Float32MultiArray, queue_size = 1 )
    pub_nodePeriod = rospy.Publisher( '/vehicle/node/broadcast_contactFrames/period', Float64, queue_size = 1 )

    #   Subscribe to ground truth
    rospy.Subscriber( '/vehicle/true_pose3D', pose3DStamped, common._callback, 3 )                                       #   '/vehicle/truePose' -> topic which collects robot perfect pose
    rospy.wait_for_message( '/vehicle/true_pose3D', pose3DStamped )

    if( common.simulationType == 0 or common.simulationType == 2 ):
        pass

    else:
        rospy.Subscriber( '/back_left_wheel_contact', ContactsState, common._contactCallback, 0 )
        rospy.Subscriber( '/front_left_wheel_contact', ContactsState, common._contactCallback, 1 )
        rospy.Subscriber( '/back_right_wheel_contact', ContactsState, common._contactCallback, 2 )
        rospy.Subscriber( '/front_right_wheel_contact', ContactsState, common._contactCallback, 3 )

        rospy.wait_for_message('/back_left_wheel_contact', ContactsState )
        rospy.wait_for_message('/front_left_wheel_contact', ContactsState )
        rospy.wait_for_message('/back_right_wheel_contact', ContactsState )
        rospy.wait_for_message('/front_right_wheel_contact', ContactsState )

    """
        Get center of mass to wheels vector
    """

    index = 0

    ###

    #   Register signal handler
    signal_handler = partial(common._signal_handler, node = "[" + rospy.get_name() + "]")
    signal.signal(signal.SIGINT, signal_handler)
    ###

    print("[" + rospy.get_name() + "] Loop is running.")

    #   Reset contact forces message
    contactStatus = Int32MultiArray()
    contactAngle = Float32MultiArray()

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        try:
            start = time.time()

            #   Get current pose
            currentPose = common.true_pose3D.pose
            
            if( common.simulationType == 0 or common.simulationType == 2 ):
                    contactStatus.data = [1, 1, 1, 1]
                    contactAngle.data = [0, 0, 0, 0]
        
            else:
                #   Get normals
                contact_bl = common.backLeftWheelContact
                contact_fl = common.frontLeftWheelContact
                contact_br = common.backRightWheelContact
                contact_fr = common.frontRightWheelContact
                
                ###
                angle_bl, contact_position_bl, contact_status_bl = _getNormalDirection(contact_bl, currentPose)
                angle_fl, contact_position_fl, contact_status_fl = _getNormalDirection(contact_fl, currentPose)
                angle_br, contact_position_br, contact_status_br = _getNormalDirection(contact_br, currentPose)
                angle_fr, contact_position_fr, contact_status_fr = _getNormalDirection(contact_fr, currentPose)

                #   Broadcast contact frames with origin located at the contact point
                if( contact_status_bl == 1 ):
                    translation, quat = _getTransform(contact_position_bl, currentPose, angle_bl)

                    br.sendTransform( (translation[0], translation[1], translation[2]), (quat[0], quat[1], quat[2], quat[3]), rospy.Time.now(), "contact_back_left", "base_link" )

                if( contact_status_fl == 1 ):
                    translation, quat = _getTransform(contact_position_fl, currentPose, angle_fl)

                    br.sendTransform( (translation[0], translation[1], translation[2]), (quat[0], quat[1], quat[2], quat[3]), rospy.Time.now(), "contact_front_left", "base_link" )

                if( contact_status_br == 1 ):
                    translation, quat = _getTransform(contact_position_br, currentPose, angle_br)

                    br.sendTransform( (translation[0], translation[1], translation[2]), (quat[0], quat[1], quat[2], quat[3]), rospy.Time.now(), "contact_back_right", "base_link" )

                if( contact_status_fr == 1 ):
                    translation, quat = _getTransform(contact_position_fr, currentPose, angle_fr)

                    br.sendTransform( (translation[0], translation[1], translation[2]), (quat[0], quat[1], quat[2], quat[3]), rospy.Time.now(), "contact_front_right", "base_link" )

                contactStatus.data = [contact_status_bl, contact_status_fl, contact_status_br, contact_status_fr]
                contactAngle.data = [angle_bl, angle_fl, angle_br, angle_fr]

            rotation_body2world = common._3D_rotationMatrix(currentPose)
            quat = R.from_matrix(rotation_body2world).as_quat()

            br.sendTransform( (currentPose.x, currentPose.y, currentPose.z), (quat[0], quat[1], quat[2], quat[3]), rospy.Time.now(), 'base_link', 'map' )

            end = time.time()

            timeDiff = end - start

            pub_nodePeriod.publish(timeDiff)
            pub_contactStatus.publish(contactStatus)
            pub_contactAngle.publish(contactAngle)

            index += 1

            rate.sleep()

        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print( "[" + rospy.get_name() + "] Something went wrong!" )

    rospy.spin()