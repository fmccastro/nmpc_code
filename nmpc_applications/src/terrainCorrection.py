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

    if( common.simulationType < 3 ):
        rospy.signal_shutdown("[" + rospy.get_name() + "] Shut down node.")

    pub_nodePeriod = rospy.Publisher( '/vehicle/node/terrainCorrection/period', Float64, queue_size = 1 )
    
    #   Publishers
    pub_backLeftWheelRate = rospy.Publisher('/back_left_wheel_plant/command', Float64, queue_size = 1)
    pub_frontLeftWheelRate = rospy.Publisher('/front_left_wheel_plant/command', Float64, queue_size = 1)
    pub_backRightWheelRate = rospy.Publisher('/back_right_wheel_plant/command', Float64, queue_size = 1)
    pub_frontRightWheelRate = rospy.Publisher('/front_right_wheel_plant/command', Float64, queue_size = 1)

    pub_contactForces = rospy.Publisher('/contactForces', contactForces, queue_size = 1 )

    #   Subscribe to ground truth
    rospy.Subscriber( '/contact_status', Int32MultiArray, common._multiArrayCallback, 4 )
    rospy.Subscriber( '/contact_angles', Float32MultiArray, common._multiArrayCallback, 5 )
    rospy.Subscriber( '/vehicle/true_pose3D', pose3DStamped, common._callback, 3 )                                       #   '/vehicle/truePose' -> topic which collects robot perfect pose
    rospy.Subscriber( '/vehicle/nmpc_dynamics/fx_l', Float32, common._wheelTorqueInputCallback, 0 )
    rospy.Subscriber( '/vehicle/nmpc_dynamics/fx_r', Float32, common._wheelTorqueInputCallback, 2  )
    rospy.Subscriber( '/back_left_wheel_contact', ContactsState, common._contactCallback, 0 )
    rospy.Subscriber( '/front_left_wheel_contact', ContactsState, common._contactCallback, 1 )
    rospy.Subscriber( '/back_right_wheel_contact', ContactsState, common._contactCallback, 2 )
    rospy.Subscriber( '/front_right_wheel_contact', ContactsState, common._contactCallback, 3 )
    
    rospy.wait_for_message( '/contact_status', Int32MultiArray)
    rospy.wait_for_message( '/contact_angles', Float32MultiArray)
    rospy.wait_for_message( '/vehicle/true_pose3D', pose3DStamped )
    rospy.wait_for_message('/vehicle/nmpc_dynamics/fx_l', Float32)
    rospy.wait_for_message('/vehicle/nmpc_dynamics/fx_r', Float32)
    rospy.wait_for_message('/back_left_wheel_contact', ContactsState )
    rospy.wait_for_message('/front_left_wheel_contact', ContactsState )
    rospy.wait_for_message('/back_right_wheel_contact', ContactsState )
    rospy.wait_for_message('/front_right_wheel_contact', ContactsState )

    """
        Get center of mass to wheels vector
    """
            
    #   Get wheel force correction qp optimization model
    model = WheelTorqueAllocation_qp(common.com2wheels)

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

            robotWeight = common._3D_rotationMatrix(currentPose).T @ np.array([0, 0, 1]) * common.gz * common.robotMass

            #   Reset contact forces message
            forces = contactForces()

            contact_angles = common.contactAngles.data
            contact_status = common.contactStatus.data

            contact_status_bl = contact_status[0]
            contact_status_fl = contact_status[1]
            contact_status_br = contact_status[2]
            contact_status_fr = contact_status[3]

            angle_bl = contact_angles[0]
            angle_fl = contact_angles[1]
            angle_br = contact_angles[2]
            angle_fr = contact_angles[3]

            if( contact_status_bl + contact_status_fl + contact_status_br + contact_status_fr >= 3 or contact_status_bl + contact_status_fr == 2 or contact_status_br + contact_status_fl == 2 ):
                
                model._setSolver( [ angle_bl, angle_fl, angle_br, angle_fr,\
                                    contact_status_bl, contact_status_fl, contact_status_br, contact_status_fr,\
                                    -robotWeight[2], fx_ref_l, fx_ref_r ] )

                res = model._callSolver( [fx_ref_l, -robotWeight[2] / 4, fx_ref_l, -robotWeight[2] / 4,\
                                          fx_ref_r, -robotWeight[2] / 4, fx_ref_r, -robotWeight[2] / 4] )

                forces.contactStatus = contact_status
                forces.contactAngle = contact_angles
                forces.normalForce = res[1::2]
                forces.tractionForce = res[0::2]

                pub_backLeftWheelRate.publish( common.wheelRadius * res[0] )
                pub_frontLeftWheelRate.publish( common.wheelRadius * res[2] )
                pub_backRightWheelRate.publish( common.wheelRadius * res[4] )
                pub_frontRightWheelRate.publish( common.wheelRadius * res[6] )
                
                pub_contactForces.publish(forces)  
                
            else:
                forces.contactStatus = [-1] * 4
                forces.contactAngle = [0.0, 0.0, 0.0, 0.0]
                forces.normalForce = [-robotWeight[2] / 4] * 4
                forces.tractionForce = [0.0, 0.0, 0.0, 0.0]

                pub_backLeftWheelRate.publish( 0.0 )
                pub_frontLeftWheelRate.publish( 0.0 )
                pub_backRightWheelRate.publish( 0.0 )
                pub_frontRightWheelRate.publish( 0.0 )

                pub_contactForces.publish(forces)

            end = time.time()

            timeDiff = end - start

            pub_nodePeriod.publish(timeDiff)

            index += 1

        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print( "[" + rospy.get_name() + "] Something went wrong!" )
    
    pub_backLeftWheelRate.publish(0.0)
    pub_frontLeftWheelRate.publish(0.0)
    pub_backRightWheelRate.publish(0.0)
    pub_frontRightWheelRate.publish(0.0)

    rospy.spin()