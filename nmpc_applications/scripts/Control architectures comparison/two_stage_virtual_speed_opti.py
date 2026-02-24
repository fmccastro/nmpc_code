#!/usr/bin/python3
from pathlib import Path

classes_path = Path(__file__)

upper_directory = classes_path.parent.parent.parent
lower_directory = upper_directory / "src"

from sys import path
path.append( str(lower_directory) )

from classes.standard_opti import *
from classes.integrators_frenet import *
from classes.common_class import *

if __name__ == '__main__':
    
    common = Common()

    with open(common.std_dyn_skid) as f:
        solver_param = json.load(f)
        specs = solver_param["solver_specs"]
        con_pose = solver_param["con_pose"]
        con_vel = solver_param["con_vel"]
        weights = solver_param["weights"]
    
    with open(common.vehicle_specs) as g:
        vehicle_specs = json.load(g)

    range_aux = 1000
    
    ##  Generate reference

    #   Point
    s = ca.SX.sym('s')
    
    #x_ref = ca.Function( 'x_ref', [s], [s] )
    #y_ref = ca.Function( 'y_ref', [s], [0] )
    #yaw_ref = ca.Function( 'yaw_ref', [s], [ca.atan(0)] )
    #curvature = ca.Function( 'curvature', [s], [0] )

    #   Sinusoid
    #x_ref = ca.Function( 'x_ref', [s], [s] )
    #y_ref = ca.Function( 'y_ref', [s], [1.0 * ca.cos(s)] )
    #yaw_ref = ca.Function( 'yaw_ref', [s], [ ca.atan( -1.0 * ca.sin(s) ) ]  )
    #curvature = ca.Function( 'curvature', [s], [ -ca.cos(s) / (1 + ca.sin(s)**2)**1.5 ] )

    #   Lissajous curve
    x_ref = ca.Function( 'x_ref', [s], [ ca.sin(s + ca.pi/2.0) ] )
    y_ref = ca.Function( 'y_ref', [s], [ ca.sin(2 * s) ] )
    yaw_ref = ca.Function( 'yaw_ref', [s], [ ca.atan2( 2 * ca.cos(2 * s), ca.cos(s + ca.pi/2.0) ) ] )
    curvature = ca.Function( 'curvature', [s], [ ( -4 * ca.cos(s + ca.pi/2.0) * ca.sin(2 * s) + 2 * ca.cos(2 * s) * ca.sin(s + ca.pi/2) ) / ( ca.cos(s + ca.pi/2)**2 + 4 * ca.cos(2 * s)**2 )**1.5 ] )

    #   Set up nmpc countoring model
    kin_model = KinematicsRates(x_ref, y_ref)
    dyn_model = DynamicsPath()

    #   (x, y, yaw, s, vx, wz)
    kin_next_state = [ float( x_ref(0) ) + 0.5, float( y_ref(0) ) - 1.0, float(yaw_ref(0)) + ca.pi/4, 0.0, 0.0, 0.0]
    
    #   Set up animation figure
    plt.ion()
    fig_anim, ax_anim = plt.subplots()

    ax_anim.set(xlim=[-3.0, 3.0], ylim=[-3.0, 3.0], xlabel='X [m]', ylabel='Y [m]')
    ax_anim.set_aspect('equal')
    ax_anim.legend()

    #   (x, y, yaw, vx, wz)
    dyn_next_state = [kin_next_state[0], kin_next_state[1], kin_next_state[2], kin_next_state[4], kin_next_state[5]]

    pitch = 0.0
    roll = 0.0
    friction = 1.0

    cone = friction * vehicle_specs["m"] * common.gz * math.cos(pitch) * math.cos(roll) / 4

    if( cone >= 0 ):
        fl_max = fr_max = cone
        fl_min = fr_min  = -cone
    
    elif( cone < 0 ):
        fl_max = fr_max = -cone
        fl_min = fr_min  = cone

    param = {'pitch': pitch, 'roll': roll, 'friction': friction, 'fl_prev': 0.0, 'fr_prev': 0.0, 'fl_min': fl_min, 'fl_max': fl_max, 'fr_min': fr_min, 'fr_max': fr_max, 'x_ref': [], 'y_ref': []}
    
    kin_opt_time = []
    dyn_opt_time = []

    kin_cost = []
    dyn_cost = []

    for i in np.arange(range_aux):
        
        print("Iteration: ", i)

        if( i == 0 ):
            kin_states = np.stack(kin_next_state)
            kin_controls = np.stack( [0.0, 0.0, 0.0] )
            
            kin_res = kin_model._solve( kin_next_state, None, pitch )
            kin_prev_res = kin_res
        
        elif( i > 0 ):
            kin_states = np.vstack( ( kin_states, np.stack(kin_next_state) ) )
            kin_controls = np.vstack( ( kin_controls, np.stack( [ vx_rate_horizon[0], wz_rate_horizon[0], virtual_speed_horizon[0] ] ) ) )

            kin_res = kin_model._solve( kin_next_state, kin_prev_res, pitch )
            kin_prev_res = kin_res
        
        #kin_model._getStats()

        x_horizon = kin_res[0]
        y_horizon = kin_res[1]
        yaw_horizon = kin_res[2]
        progress_horizon = kin_res[3]
        vx_horizon = kin_res[4]
        wz_horizon = kin_res[5]
        vx_rate_horizon = kin_res[6]
        wz_rate_horizon = kin_res[7]
        virtual_speed_horizon = kin_res[8]

        #   Reference to be followed by dynamics solver
        x_new_ref = x_horizon
        y_new_ref = y_horizon

        param["x_ref"] = x_new_ref
        param["y_ref"] = y_new_ref

        if( i == 0 ):
            dyn_res = dyn_model._solve(dyn_next_state, None, param)
            dyn_prev_res = dyn_res

        elif( i > 0 ):
            param["fl_prev"] = fl_dyn_horizon[0]
            param["fr_prev"] = fr_dyn_horizon[0]

            dyn_res = dyn_model._solve(dyn_next_state, dyn_prev_res, param)
            dyn_prev_res = dyn_res
        
        x_dyn_horizon = dyn_res[0]
        y_dyn_horizon = dyn_res[1]
        yaw_dyn_horizon = dyn_res[2]
        vx_dyn_horizon = dyn_res[3]
        wz_dyn_horizon = dyn_res[4]
        fl_dyn_horizon = dyn_res[5]
        fr_dyn_horizon = dyn_res[6]

        horizon_path_plot, = ax_anim.plot( x_dyn_horizon, y_dyn_horizon, 'r' )
        kin_ref_path_plot, = ax_anim.plot( x_new_ref, y_new_ref, 'b--' )
        ref_path_plot, = ax_anim.plot( x_ref( progress_horizon ), y_ref( progress_horizon ), 'k--' )

        kin_next_state = [ x_dyn_horizon[1], y_dyn_horizon[1], yaw_dyn_horizon[1], progress_horizon[1], vx_dyn_horizon[1], wz_dyn_horizon[1] ]
        dyn_next_state = [ x_dyn_horizon[1], y_dyn_horizon[1], yaw_dyn_horizon[1], vx_dyn_horizon[1], wz_dyn_horizon[1] ]

        kin_stats = kin_model._getStats()
        dyn_stats = dyn_model._getStats()

        kin_opt_time += [ kin_stats["t_proc_total"] ]
        kin_cost += [ kin_stats["iterations"]["obj"][-1] ]

        dyn_opt_time += [ dyn_stats["t_proc_total"] ]
        dyn_cost += [ dyn_stats["iterations"]["obj"][-1] ]

        fig_anim.canvas.draw()
        fig_anim.canvas.flush_events()
        
        horizon_path_plot.remove()
        kin_ref_path_plot.remove()
        ref_path_plot.remove()
        
    plt.ioff()
    plt.show()

    fig, ax = plt.subplots(2, 5)
    
    ax[0, 0].plot( kin_states[:, 0] )
    ax[0, 0].set_title('x')

    ax[0, 1].plot( kin_states[:, 1] )
    ax[0, 1].set_title('y')

    ax[0, 2].plot( kin_states[:, 2] )
    ax[0, 2].set_title('Yaw')
    
    ax[0, 3].plot( kin_states[:, 3] )
    ax[0, 3].set_title('Progress')

    ax[0, 4].plot( kin_states[:, 4] )
    ax[0, 4].set_title('vx')

    ax[1, 0].plot( kin_states[:, 5] )
    ax[1, 0].set_title('wz')

    ax[1, 1].plot( kin_controls[:, 2] )
    ax[1, 1].set_title('Virtual speed')

    ax[1, 2].plot( kin_controls[:, 0] )
    ax[1, 2].set_title('vx_rate')

    ax[1, 3].plot( kin_controls[:, 1] )
    ax[1, 3].set_title('wz_rate')

    fig2, ax2 = plt.subplots(1, 2)
    
    ax2[0].plot(kin_cost, 'b')
    ax2[0].set_title('Cost')

    ax2[0].plot(dyn_cost, 'r')
    ax2[0].set_title('Cost')

    ax2[1].plot( np.array(kin_opt_time) * 1000, 'r' )
    ax2[1].plot( np.array(dyn_opt_time) * 1000, 'b' )
    ax2[1].set_title('Opt. time [ms]')

    plt.show()

    data2save = { 'dyn_cost': dyn_cost, 'kin_cost': kin_cost, 'dyn_opt_time': dyn_opt_time, 'kin_opt_time': kin_opt_time,\
                  'x': list( kin_states[:, 0] ), 'y': list( kin_states[:, 1] ), 'progress': list( kin_states[:, 3] ) }

    with open(common.results_folder + "Control_architectures/" + "two_stage_std_virtual_speed.json", "w") as f:
        json.dump(data2save, f, indent=4)