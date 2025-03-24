#!/usr/bin/python3
import sys
sys.path.append("/media/fmccastro/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/nmpc_ros2_ws/src/nmpc_application/src")

from classes.all_imports import *

WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLACK = (0, 0, 0)

pygame.init()

center_x = 250.0
center_y = 220.0
radius = 200

#   Maximum velocities
max_vx = 5.0
max_wz = 2.5

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

m_vx = -max_vx / radius
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

clock = pygame.time.Clock()
FPS = 100    #   Set display update rate

def joy_display(publisher):

    # create a text surface object,
    # on which text is drawn on it.
    vx = 0.0
    wz = 0.0

    v_x_text = font.render(f'vx [m/s]  : {vx}', True, WHITE)
    w_z_text = font.render(f'wz [rad/s]: {wz}', True, WHITE)

    v_x_legend = font.render('vx', True, WHITE)
    w_z_legend = font.render('wz', True, WHITE)

    last_pos = (center_x, center_y)

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return
        
        if( pygame.mouse.get_pressed(num_buttons=3)[0] ):
            mouse_pos = pygame.mouse.get_pos()

            #   Check if button is clicked or pressed inside white circle
            if( math.pow(center_x - mouse_pos[0], 2) + math.pow(center_y - mouse_pos[1], 2) <= math.pow(radius, 2) ):
                
                #   Erase text by placing black rectangle over it
                pygame.draw.rect(display, BLACK, (10, 440, 230, 30))
                pygame.draw.rect(display, BLACK, (10, 470, 230, 30))

                #   Delete last circle by drawing a black one above
                pygame.draw.circle(display, BLACK, last_pos, 6, 0)
                pygame.draw.circle(display, GREEN, (center_x, center_y), 3, 0)
                pygame.draw.circle(display, WHITE, (center_x, center_y), radius, 1)
                pygame.draw.circle(display, RED, mouse_pos, 5, 0)

                #   Assign velocities to print
                vx = round(mouse_pos[1] * m_vx + b_vx, 3)
                wz = round(mouse_pos[0] * m_wz + b_wz, 3)

                #   Set new text
                v_x_text = font.render(f'vx [m/s]  : {vx}', True, WHITE)
                w_z_text = font.render(f'wz [rad/s]: {wz}', True, WHITE)

                last_pos = mouse_pos

        msg = Twist()
        msg.linear.x = vx
        msg.angular.z = wz

        publisher.publish(msg)

        display.blit(v_x_text, (10, 440))
        display.blit(w_z_text, (10, 470))
        display.blit(v_x_legend, (465, 400))
        display.blit(w_z_legend, (400, 470))
        pygame.display.update()
        clock.tick(FPS)
    
if __name__ == '__main__':

    rospy.init_node('teleop_mouse_joy', anonymous = True)

    pub_command = rospy.Publisher('/model/pioneer3at/cmd_vel', Twist, queue_size=10)

    while not rospy.is_shutdown():
        try:
            joy_display(pub_command)

        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
            print( "[mouse_joy.py] Something went wrong!" )
    
    rospy.spin()