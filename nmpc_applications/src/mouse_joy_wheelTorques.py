#!/usr/bin/python3
import sys
sys.path.append("/media/fmccastro/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/nmpc_ros2_ws/src/nmpc_application/src")

from classes.all_imports import *

WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLACK = (0, 0, 0)
BLUE = (0, 120, 215)
DARK_BLUE = (0, 100, 180)

pygame.init()

center_x = 250.0
center_y = 220.0
radius = 200

#   Maximum velocities
max_tau = 5.0
max_dtau = 5.0

#   Pixel to velocities conversion
"""
                 (20, max_vx)
                    ***    
                 *       *  
(50, max_wz)    *         *  (450, -max_wz)
                *         * 
                 *       *  
                    ***    
                (420, -max_vx)
"""

"""m_vx = -max_vx / radius
b_vx = max_vx - m_vx * (center_y - radius)

m_wz = -max_wz / radius
b_wz = max_wz - m_wz * (center_x - radius)

#   Set display size on pixel units
display = pygame.display.set_mode((500, 500))
pygame.draw.circle(display, WHITE, (center_x, center_y), radius, 1)
pygame.draw.circle(display, GREEN, (center_x, center_y), 3, 0)
pygame.draw.circle(display, RED, (center_x, center_y), 5, 0)

#   Present directions of vx and wz
pygame.draw.circle(display, RED, (450, 455), 2, 0)
pygame.draw.polygon(display, WHITE, ( (450, 450), (450, 420), (440, 420), (450, 410), (460, 420), (450, 420) ))
pygame.draw.polygon(display, WHITE, ( (445, 455), (415, 455), (415, 465), (405, 455), (415, 445), (415, 455) ))

pygame.display.set_caption("Joystick mouse handler")

# create a font object.
# 1st parameter is the font file
# which is present in pygame.
# 2nd parameter is size of the font
font = pygame.font.SysFont('monospace', 20)

# Button properties
button_rect = pygame.Rect(10, 30, 80, 30)
button_color = BLUE
hover_color = DARK_BLUE
text = font.render("RESET", True, WHITE)

clock = pygame.time.Clock()
FPS = 100    #   Set display update rate"""

def joy_display(pub_bl, pub_fl, pub_br, pub_fr, pub_fl_steer, pub_fr_steer, pub_bl_steer, pub_br_steer):

    pygame.init()
    win = pygame.display.set_mode((1000, 600))

    pygame.display.set_caption("Joystick mouse handler")

    slider_tau = Slider(win, 100, 100, 800, 40, min = 0, max = 99, step=0.1 )
    output_tau = TextBox(win, 475, 200, 60, 60, fontSize=30)

    slider_dtau = Slider(win, 100, 400, 800, 40, min = 0, max = 99, step=0.1 )
    output_dtau = TextBox(win, 475, 500, 60, 60, fontSize=30)

    output_tau.disable()  # Act as label instead of textbox
    output_dtau.disable()

    run = True
    while run:
        events = pygame.event.get()
        for event in events:
            if event.type == pygame.QUIT:
                pygame.quit()
                run = False
                quit()
        
        win.fill( (255, 255, 255) )

        tau_value = slider_tau.getValue() * (max_tau - (-max_tau)) / 99.0 + (-max_tau)
        dtau_value = slider_dtau.getValue() * (max_tau - (-max_tau)) / 99.0 + (-max_tau)

        output_tau.setText( round( tau_value, 2 ) )
        output_dtau.setText( round( dtau_value, 2 ) ) 

        pygame_widgets.update(events)
        pygame.display.update()

        pub_bl.publish(tau_value)
        pub_fl.publish(tau_value)
        pub_br.publish(tau_value)
        pub_fr.publish(tau_value)

        if( abs(dtau_value) < 1e-3 ):
            delta_lf = 0.0
            delta_rf = 0.0
        
        else:
            R = 0.211 / math.tan(dtau_value)
            delta_lf = math.atan( 0.211 / (R - 0.225) )
            delta_rf = math.atan( 0.211 / (R + 0.225) )

        pub_bl_steer.publish(0.0)
        pub_br_steer.publish(0.0)
        pub_fl_steer.publish(delta_lf)
        pub_fr_steer.publish(delta_rf)

if __name__ == '__main__':

    rospy.init_node('teleop_mouse_joy', anonymous = True)
    
    pub_backLeft_torque = rospy.Publisher('/back_left_wheel_plant/command', Float64, queue_size=10)
    pub_frontLeft_torque = rospy.Publisher('/front_left_wheel_plant/command', Float64, queue_size=10)
    pub_backRight_torque = rospy.Publisher('/back_right_wheel_plant/command', Float64, queue_size=10)
    pub_frontRight_torque = rospy.Publisher('/front_right_wheel_plant/command', Float64, queue_size=10)

    pub_backRight_steering = rospy.Publisher('/back_right_steering_plant/command', Float64, queue_size=10)
    pub_backLeft_steering = rospy.Publisher('/back_left_steering_plant/command', Float64, queue_size=10)
    pub_frontRight_steering = rospy.Publisher('/front_right_steering_plant/command', Float64, queue_size=10)
    pub_frontLeft_steering = rospy.Publisher('/front_left_steering_plant/command', Float64, queue_size=10)

    while not rospy.is_shutdown():
        try:
            joy_display(pub_backLeft_torque, pub_frontLeft_torque, pub_backRight_torque, pub_frontRight_torque, pub_frontLeft_steering, pub_frontRight_steering, pub_backLeft_steering, pub_backRight_steering)
        
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
            print( "[mouse_joy.py] Something went wrong!" )
    
    rospy.spin()