#!/usr/bin/python3
from pathlib import Path

classes_path = Path(__file__)

upper_directory = classes_path.parent.parent
lower_directory = upper_directory / "src"

from sys import path
path.append( str(lower_directory) )

from classes.integrator import *
from classes.common_class import *
from classes.planner_class import *

classes_path = Path(__file__)

upper_directory = classes_path.parent
upper_directory = upper_directory.parent
lower_directory = upper_directory / "data" / "modelIdentification"

filename = "modelDynamicsContactsEmptyWorld3D.bag"

bag_file = lower_directory / filename

dir2save = '/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/mscthesis/mscthesis_pdf/Figures'

def save_file(state):

    if( filename == "modelDynamicsContactsEmptyWorld3D.bag" ):
        plt.savefig(dir2save + '/states_comparison_3d_sim_' + state + '.pdf')

    elif( filename == "modelDynamicsContactsEmptyWorldPlanar.bag" ):
        plt.savefig(dir2save + '/states_comparison_planar_sim_' + state + '.pdf')

    elif( filename == "modelDynamics.bag" ):
        plt.savefig( dir2save + '/states_comparison_3d_sim_contacts_' + state + '.pdf' )

_time = []

x = []
y = []
z = []

roll = []
pitch = []
yaw = []

vx = []
vy = []
vz = []

wx = []
wy = []
wz = []

fx = []
fy = []
fz = []

mx = []
my = []
mz = []

fx_bl = []
fy_bl = []
fz_bl = []

fx_fl = []
fy_fl = []
fz_fl = []

fx_br = []
fy_br = []
fz_br = []

fx_fr = []
fy_fr = []
fz_fr = []

flag1 = False
flag2 = False

common = Common()

with open(common.vehicle_specs) as f:
    vehicle_param = json.load(f)

    com2wheels = vehicle_param["com2wheels"]

check = 0

#   Get simulated states
with rosbag.Bag(bag_file, 'r') as bag:

    for topic, msg, t in bag.read_messages():

        if( topic == "/record_data/true_pose3D" ):
            if(flag1 is False):
                flag1 = True
            
            x += [msg.pose.x]
            y += [msg.pose.y]
            z += [msg.pose.z]

            roll += [msg.pose.roll]
            pitch += [msg.pose.pitch]
            yaw += [msg.pose.yaw]

        if( topic == "/record_data/velocity_bodyFrame" ):
            vx += [msg.velocity[0].linear.x]
            vy += [msg.velocity[0].linear.y]
            vz += [msg.velocity[0].linear.z]

            wx += [msg.velocity[0].angular.x]
            wy += [msg.velocity[0].angular.y]
            wz += [msg.velocity[0].angular.z]

        if( topic == "/record_data/wrench_base_link" ):
            _time += [ msg.header.stamp.secs + msg.header.stamp.nsecs * math.pow(10, -9) ]

            base_link_wrench = msg.wrench

            check += 1

        elif( topic == "/record_data/wrench_back_left" ):
            back_left_wrench = msg.wrench
            check += 1

        elif( topic == "/record_data/wrench_front_left" ):
            front_left_wrench = msg.wrench
            check += 1

        elif( topic == "/record_data/wrench_back_right" ):
            back_right_wrench = msg.wrench
            check += 1
            
        elif( topic == "/record_data/wrench_front_right" ):
            front_right_wrench = msg.wrench
            check += 1

        elif( topic == '/record_data/bl_rel_pose' ):
            bl_rel_pose = msg
            check += 1

        elif( topic == '/record_data/fl_rel_pose' ):
            fl_rel_pose = msg
            check += 1

        elif( topic == '/record_data/br_rel_pose' ):
            br_rel_pose = msg
            check += 1

        elif( topic == '/record_data/fr_rel_pose' ):
            fr_rel_pose = msg
            check += 1

        """elif( topic == "/record_data/joint_states" ):
            joint_states = msg

            check += 1"""
        
        """if( topic == "/record_data/back_left_wheel_wrench" ):
            back_left_contact = msg

            back_left_wrench = Wrench()
            
            for wrench in back_left_contact.states[0].wrenches:
                back_left_wrench.force.x += wrench.force.x
                back_left_wrench.force.y += wrench.force.y
                back_left_wrench.force.z += wrench.force.z

            check += 1

        elif( topic == "/record_data/front_left_wheel_wrench" ):
            front_left_contact = msg

            front_left_wrench = Wrench()

            for wrench in front_left_contact.states[0].wrenches:
                front_left_wrench.force.x += wrench.force.x
                front_left_wrench.force.y += wrench.force.y
                front_left_wrench.force.z += wrench.force.z

            check += 1

        elif( topic == "/record_data/back_right_wheel_wrench" ):
            back_right_contact = msg

            back_right_wrench = Wrench()

            for wrench in back_right_contact.states[0].wrenches:
                back_right_wrench.force.x += wrench.force.x
                back_right_wrench.force.y += wrench.force.y
                back_right_wrench.force.z += wrench.force.z

            check += 1

        elif( topic == "/record_data/front_right_wheel_wrench" ):
            front_right_contact = msg

            front_right_wrench = Wrench()

            for wrench in front_right_contact.states[0].wrenches:
                front_right_wrench.force.x += wrench.force.x
                front_right_wrench.force.y += wrench.force.y
                front_right_wrench.force.z += wrench.force.z

            check += 1"""
        
        if( check == 9 ):
            rot_matrix = common.xyz_rotationMatrix( roll[-1], pitch[-1], yaw[-1] )

            rot_matrix_bl = common.xyz_rotationMatrix(bl_rel_pose.x, bl_rel_pose.y, bl_rel_pose.z)
            rot_matrix_fl = common.xyz_rotationMatrix(fl_rel_pose.x, fl_rel_pose.y, fl_rel_pose.z)
            rot_matrix_br = common.xyz_rotationMatrix(br_rel_pose.x, br_rel_pose.y, br_rel_pose.z)
            rot_matrix_fr = common.xyz_rotationMatrix(fr_rel_pose.x, fr_rel_pose.y, fr_rel_pose.z)
            
            force_b = np.array( [ base_link_wrench.force.x, base_link_wrench.force.y, base_link_wrench.force.z ] )
            force_bl = rot_matrix_bl @ np.array( [ back_left_wrench.force.x, back_left_wrench.force.y, back_left_wrench.force.z ] )
            force_fl = rot_matrix_fl @ np.array( [ front_left_wrench.force.x, front_left_wrench.force.y, front_left_wrench.force.z ] )
            force_br = rot_matrix_br @ np.array( [ back_right_wrench.force.x, back_right_wrench.force.y, back_right_wrench.force.z ] )
            force_fr = rot_matrix_fr @ np.array( [ front_right_wrench.force.x, front_right_wrench.force.y, front_right_wrench.force.z ] )

            forces = force_b + force_bl + force_fl + force_br + force_fr

            moment_b = np.array( [ base_link_wrench.torque.x, base_link_wrench.torque.y, base_link_wrench.torque.z ] )
            moment_bl = rot_matrix_bl @ np.array( [ back_left_wrench.torque.x, back_left_wrench.torque.y, back_left_wrench.torque.z ] )
            moment_fl = rot_matrix_fl @ np.array( [ front_left_wrench.torque.x, front_left_wrench.torque.y, front_left_wrench.torque.z ] )
            moment_br = rot_matrix_br @ np.array( [ back_right_wrench.torque.x, back_right_wrench.torque.y, back_right_wrench.torque.z ] )
            moment_fr = rot_matrix_fr @ np.array( [ front_right_wrench.torque.x, front_right_wrench.torque.y, front_right_wrench.torque.z ] )

            moments = moment_b +\
                        moment_bl + moment_fl + moment_br + moment_fr +\
                        force_bl @ np.array( com2wheels["bl"] ) +\
                        force_fl @ np.array( com2wheels["fl"] ) +\
                        force_br @ np.array( com2wheels["br"] ) +\
                        force_fr @ np.array( com2wheels["fr"] )

            fx += [ forces[0] ]
            fy += [ forces[1] ]
            fz += [ forces[2] ]

            mx += [ moments[0] ]
            my += [ moments[1] ]
            mz += [ moments[2] ]

            check = 0
        
        """if( topic == '/record_data/clock' ):
            _time += [msg.data]"""

#   Get propagated states with integrator
model = DynamicsIntegrator()

sim_x = [ x[0] ]
sim_y = [ y[0] ]
sim_z = [ z[0] ]

sim_roll = [ roll[0] ]
sim_pitch = [ pitch[0] ]
sim_yaw = [ yaw[0] ]

sim_vx = [ vx[0] ]
sim_vy = [ vy[0] ]
sim_vz = [ vz[0] ]

sim_wx = [ wx[0] ]
sim_wy = [ wy[0] ]
sim_wz = [ wz[0] ]

_dtime = []

for index in range( len(_time) - 1 ):
    
    dtime = _time[index + 1] - _time[index]
    _dtime += [dtime]
    #dtime = 0.001

    #print(dtime)

    if( index == 0 ):
        states = np.array( [ x[0], y[0], z[0], roll[0], pitch[0], yaw[0], vx[0], vy[0], vz[0], wx[0], wy[0], wz[0] ] )
    
    #else:
    #    states = np.array( [ states[0], states[1], 0.0, 0.0, 0.0, states[5], states[6], states[7], 0.0, 0.0, 0.0, states[11] ] )

    #states = model._simulate( states , np.array( [ fx[index], fy[index], fz[index], mx[index], my[index], mz[index] ] ), dtime )
    #states = model._simulate( states , np.array( [ 1.0, -0.05, 0.02, -0.1, 0.2, 0.5 ] ), dtime )
    #states = model._simulate( states , np.array( [ 2.0, 0.0, 0.0, 0.0, 0.0, 0.2 ] ), dtime )       #    Planar motion
    states = model._simulate( states , np.array( [ 1.0, 0.02, 0.04, -0.03, 0.02, 0.2 ] ), dtime )   #    3D motion

    sim_x += [states[0]]
    sim_y += [states[1]]
    sim_z += [states[2]]
    
    sim_roll += [states[3]]
    sim_pitch += [states[4]]
    sim_yaw += [states[5]]
    
    sim_vx += [states[6]]
    sim_vy += [states[7]]
    sim_vz += [states[8]]

    sim_wx += [states[9]]
    sim_wy += [states[10]]
    sim_wz += [states[11]]

_time = np.array(_time) - _time[0]

# using the variable axs for multiple Axes
#fig, ax = plt.subplots(4, 3, constrained_layout="constrained")

plt.rcParams.update( {'font.family': 'Arial',\
                      'font.size': 24,\
                      'lines.linewidth': 5} )

#   Limit the plot to the first ten seconds (up to index 3200) when getting results for model verification with contacts
plt.figure( layout='constrained', figsize=[6.0, 4.0] )

if(filename == "modelDynamicsContactsEmptyWorld3D.bag" ):
    l1, = plt.plot(_time, x, label=r'$x_{sim}$')
    l2, = plt.plot(_time, sim_x, 'r', label=r'$x_{pro}$')

elif(filename == "modelDynamics.bag"):
    l1, = plt.plot(_time[:3200], x[:3200], label=r'$x_{sim}$')
    l2, = plt.plot(_time[:3200], sim_x[:3200], 'r', label=r'$x_{pro}$')

plt.xlabel(r'$t\;[s]$')
plt.ylabel(r'$x\;[m]$')
#ax[0, 0].xaxis.set_major_formatter(FormatStrFormatter('%.2f'))
#ax[0, 0].yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
#ax[0, 0].set_title(r'$x$')
plt.legend( handles=[l1, l2] )

save_file('x')

plt.figure(layout="constrained", figsize=[6.0, 4.0])

if(filename == "modelDynamicsContactsEmptyWorld3D.bag" ):
    l3, = plt.plot(_time, y, label=r'$y_{sim}$')
    l4, = plt.plot(_time, sim_y, 'r', label=r'$y_{pro}$')

elif(filename == "modelDynamics.bag"):
    l3, = plt.plot(_time[:3200], y[:3200], label=r'$y_{sim}$')
    l4, = plt.plot(_time[:3200], sim_y[:3200], 'r', label=r'$y_{pro}$')

plt.xlabel(r'$t\;[s]$')
plt.ylabel(r'$y\;[m]$')
#ax[0, 1].xaxis.set_major_formatter(FormatStrFormatter('%.2f'))
#ax[0, 1].yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
#ax[0, 1].set_title(r'$y$')
plt.legend( handles=[l3, l4] )

save_file('y')

plt.figure(layout="constrained", figsize=[6.0, 4.0])

if(filename == "modelDynamicsContactsEmptyWorld3D.bag" ):
    l5, = plt.plot(_time, z, label=r'$z_{sim}$')
    l6, = plt.plot(_time, sim_z, 'r', label=r'$z_{pro}$')

elif(filename == "modelDynamics.bag"):
    l5, = plt.plot(_time[:3200], z[:3200], label=r'$z_{sim}$')
    l6, = plt.plot(_time[:3200], sim_z[:3200], 'r', label=r'$z_{pro}$')

plt.xlabel(r'$t\;[s]$')
plt.ylabel(r'$z\;[m]$')
#ax[0, 2].xaxis.set_major_formatter(FormatStrFormatter('%.2f'))
#ax[0, 2].yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
#ax[0, 2].set_title(r'$z$')
plt.legend( handles=[l5, l6] )

save_file('z')

#   Plot roll
plt.figure(layout="constrained", figsize=[6.0, 4.0])

if(filename == "modelDynamicsContactsEmptyWorld3D.bag" ):
    l7, = plt.plot(_time, roll, label=r'$\phi_{sim}$')
    l8, = plt.plot(_time, sim_roll, 'r', label=r'$\phi_{pro}$')

elif(filename == "modelDynamics.bag"):
    l7, = plt.plot(_time[:3200], roll[:3200], label=r'$\phi_{sim}$')
    l8, = plt.plot(_time[:3200], sim_roll[:3200], 'r', label=r'$\phi_{pro}$')

#ax[1, 0].xaxis.set_major_formatter(FormatStrFormatter('%.3f'))
#ax[1, 0].yaxis.set_major_formatter(FormatStrFormatter('%.3f'))
plt.xlabel(r'$t\;[s]$')
plt.ylabel(r'$\phi\;[rad]$')

plt.legend( handles=[l7, l8] )

save_file('roll')

#   Plot pitch
plt.figure(layout="constrained", figsize=[6.0, 4.0])

if(filename == "modelDynamicsContactsEmptyWorld3D.bag" ):
    l9, = plt.plot(_time, pitch, label=r'$\theta_{sim}$')
    l10, = plt.plot(_time, sim_pitch, 'r', label=r'$\theta_{pro}$')

elif(filename == "modelDynamics.bag"):
    l9, = plt.plot(_time[:3200], pitch[:3200], label=r'$\theta_{sim}$')
    l10, = plt.plot(_time[:3200], sim_pitch[:3200], 'r', label=r'$\theta_{pro}$')

#ax[1, 0].xaxis.set_major_formatter(FormatStrFormatter('%.3f'))
#ax[1, 0].yaxis.set_major_formatter(FormatStrFormatter('%.3f'))
plt.xlabel(r'$t\;[s]$')
plt.ylabel(r'$\theta\;[rad]$')

plt.legend( handles=[l9, l10] )

save_file('pitch')

#   Plot yaw
plt.figure(layout="constrained", figsize=[6.0, 4.0])

if(filename == "modelDynamicsContactsEmptyWorld3D.bag" ):
    l11, = plt.plot(_time, yaw, label=r'$\psi_{sim}$')
    l12, = plt.plot(_time, sim_yaw, 'r', label=r'$\psi_{pro}$')

elif(filename == "modelDynamics.bag"):
    l11, = plt.plot(_time[:3200], yaw[:3200], label=r'$\psi_{sim}$')
    l12, = plt.plot(_time[:3200], sim_yaw[:3200], 'r', label=r'$\psi_{pro}$')

#ax[1, 0].xaxis.set_major_formatter(FormatStrFormatter('%.3f'))
#ax[1, 0].yaxis.set_major_formatter(FormatStrFormatter('%.3f'))
plt.xlabel(r'$t\;[s]$')
plt.ylabel(r'$\psi\;[rad]$')

plt.legend( handles=[l11, l12] )

save_file('yaw')

#   Plot vx
plt.figure(layout="constrained", figsize=[6.0, 4.0])

if(filename == "modelDynamicsContactsEmptyWorld3D.bag" ):
    l1, = plt.plot(_time, vx, label=r'$u_{sim}$')
    l2, = plt.plot(_time, sim_vx, 'r', label=r'$u_{pro}$')

elif(filename == "modelDynamics.bag"):
    l1, = plt.plot(_time[:3200], vx[:3200], label=r'$u_{sim}$')
    l2, = plt.plot(_time[:3200], sim_vx[:3200], 'r', label=r'$u_{pro}$')

#ax[1, 0].xaxis.set_major_formatter(FormatStrFormatter('%.3f'))
#ax[1, 0].yaxis.set_major_formatter(FormatStrFormatter('%.3f'))
plt.xlabel(r'$t\;[s]$')
plt.ylabel(r'$u\;[m/s]$')

plt.legend( handles=[l1, l2] )

save_file('vx')

#   Plot vy
plt.figure(layout="constrained", figsize=[6.0, 4.0])

if(filename == "modelDynamicsContactsEmptyWorld3D.bag" ):
    l1, = plt.plot(_time, vy, label=r'$v_{sim}$')
    l2, = plt.plot(_time, sim_vy, 'r', label=r'$v_{pro}$')

elif(filename == "modelDynamics.bag"):
    l1, = plt.plot(_time[:3200], vy[:3200], label=r'$v_{sim}$')
    l2, = plt.plot(_time[:3200], sim_vy[:3200], 'r', label=r'$v_{pro}$')

#ax[1, 0].xaxis.set_major_formatter(FormatStrFormatter('%.3f'))
#ax[1, 0].yaxis.set_major_formatter(FormatStrFormatter('%.3f'))
plt.xlabel(r'$t\;[s]$')
plt.ylabel(r'$v\;[m/s]$')

plt.legend( handles=[l1, l2] )

save_file('vy')

#   Plot vz
plt.figure(layout="constrained", figsize=[6.0, 4.0])

if(filename == "modelDynamicsContactsEmptyWorld3D.bag" ):
    l1, = plt.plot(_time, vz, label=r'$w_{sim}$')
    l2, = plt.plot(_time, sim_vz, 'r', label=r'$w_{pro}$')

elif(filename == "modelDynamics.bag"):
    l1, = plt.plot(_time[:3200], vz[:3200], label=r'$w_{sim}$')
    l2, = plt.plot(_time[:3200], sim_vz[:3200], 'r', label=r'$w_{pro}$')

#ax[1, 0].xaxis.set_major_formatter(FormatStrFormatter('%.3f'))
#ax[1, 0].yaxis.set_major_formatter(FormatStrFormatter('%.3f'))
plt.xlabel(r'$t\;[s]$')
plt.ylabel(r'$w\;[m/s]$')

plt.legend( handles=[l1, l2] )

save_file('vz')

#   Plot wx
plt.figure(layout="constrained", figsize=[6.0, 4.0])

if(filename == "modelDynamicsContactsEmptyWorld3D.bag" ):
    l1, = plt.plot(_time, wx, label=r'$p_{sim}$')
    l2, = plt.plot(_time, sim_wx, 'r', label=r'$p_{pro}$')

elif(filename == "modelDynamics.bag"):
    l1, = plt.plot(_time[:3200], wx[:3200], label=r'$p_{sim}$')
    l2, = plt.plot(_time[:3200], sim_wx[:3200], 'r', label=r'$p_{pro}$')

#ax[1, 0].xaxis.set_major_formatter(FormatStrFormatter('%.3f'))
#ax[1, 0].yaxis.set_major_formatter(FormatStrFormatter('%.3f'))
plt.xlabel(r'$t\;[s]$')
plt.ylabel(r'$p\;[rad/s]$')

plt.legend( handles=[l1, l2] )

save_file('wx')

#   Plot wy
plt.figure(layout="constrained", figsize=[6.0, 4.0])

if(filename == "modelDynamicsContactsEmptyWorld3D.bag" ):
    l1, = plt.plot(_time, wy, label=r'$q_{sim}$')
    l2, = plt.plot(_time, sim_wy, 'r', label=r'$q_{pro}$')

elif(filename == "modelDynamics.bag"):
    l1, = plt.plot(_time[:3200], wy[:3200], label=r'$q_{sim}$')
    l2, = plt.plot(_time[:3200], sim_wy[:3200], 'r', label=r'$q_{pro}$')

#ax[1, 0].xaxis.set_major_formatter(FormatStrFormatter('%.3f'))
#ax[1, 0].yaxis.set_major_formatter(FormatStrFormatter('%.3f'))
plt.xlabel(r'$t\;[s]$')
plt.ylabel(r'$q\;[rad/s]$')

plt.legend( handles=[l1, l2] )

save_file('wy')

#   Plot wz
plt.figure(layout="constrained", figsize=[6.0, 4.0])

if(filename == "modelDynamicsContactsEmptyWorld3D.bag" ):
    l1, = plt.plot(_time, wz, label=r'$r_{sim}$')
    l2, = plt.plot(_time, sim_wz, 'r', label=r'$r_{pro}$')

elif(filename == "modelDynamics.bag"):
    l1, = plt.plot(_time[:3200], wz[:3200], label=r'$r_{sim}$')
    l2, = plt.plot(_time[:3200], sim_wz[:3200], 'r', label=r'$r_{pro}$')

#ax[1, 0].xaxis.set_major_formatter(FormatStrFormatter('%.3f'))
#ax[1, 0].yaxis.set_major_formatter(FormatStrFormatter('%.3f'))
plt.xlabel(r'$t\;[s]$')
plt.ylabel(r'$r\;[rad/s]$')

plt.legend( handles=[l1, l2] )

save_file('wz')

plt.show()

fig2, ax2 = plt.subplots(2, 3)

ax2[0, 0].plot(_time, fx)
ax2[0, 0].yaxis.set_major_formatter(FormatStrFormatter('%.3f'))
ax2[0, 0].set_ylabel(r'[$N$]')
ax2[0, 0].set_title(r'$f_x$')

ax2[0, 1].plot(_time, fy)
ax2[0, 1].yaxis.set_major_formatter(FormatStrFormatter('%.3f'))
ax2[0, 1].set_title(r'$f_y$')

ax2[0, 2].plot(_time, fz)
ax2[0, 2].yaxis.set_major_formatter(FormatStrFormatter('%.3f'))
ax2[0, 2].set_title(r'$f_z$')

ax2[1, 0].plot(_time, mx)
ax2[1, 0].yaxis.set_major_formatter(FormatStrFormatter('%.3f'))
ax2[1, 0].set_ylabel(r'$[N\,\cdot\,m]$')
ax2[1, 0].set_title(r'$m_x$')

ax2[1, 1].plot(_time, my)
ax2[1, 1].yaxis.set_major_formatter(FormatStrFormatter('%.3f'))
ax2[1, 1].set_title(r'$m_y$')

ax2[1, 2].plot(_time, mz)
ax2[1, 2].yaxis.set_major_formatter(FormatStrFormatter('%.3f'))
ax2[1, 2].set_title(r'$m_z$')

fig2.supxlabel(r'$\delta\,t\; [s]$')

plt.show()

if( filename == "modelDynamicsContactsEmptyWorld3D.bag" ):
    fig2.savefig(dir2save + '/forces_moments_comparison_3d_sim.pdf')

elif( filename == "modelDynamicsContactsEmptyWorldPlanar.bag" ):
    fig2.savefig(dir2save + '/forces_moments_comparison_planar_sim.pdf')

elif( filename == "modelDynamics.bag" ):
    fig2.savefig(dir2save + '/forces_moments_comparison_3d_sim_contacts.pdf')

plt.figure()
plt.plot(x, y)
plt.plot(sim_x, sim_y, 'r')
plt.xlabel(r'x [m]')
plt.ylabel(r'y [m]')
plt.title("Position")

plt.show()

if( filename == "modelDynamicsContactsEmptyWorld3D.bag" ):
    plt.savefig(dir2save + '/position_comparison_3d_sim.pdf')

elif( filename == "modelDynamicsContactsEmptyWorldPlanar.bag" ):
    plt.savefig(dir2save + '/position_comparison_planar_sim.pdf')

elif( filename == "modelDynamics.bag" ):
    plt.savefig(dir2save + '/position_comparison_3d_sim_contacts.pdf' )

plt.close()