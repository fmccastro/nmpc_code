#!/usr/bin/python3
from pathlib import Path

classes_path = Path(__file__)

upper_directory = classes_path.parent.parent
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
    model = DynamicsPath()

    next_state = [float( x_ref(0) ) + 0.5, float( y_ref(0) ) - 1.0, float( yaw_ref(0) ) + ca.pi/4.0, 0.0, 0.0]
    
    #   Set up animation figure
    plt.ion()
    fig_anim, ax_anim = plt.subplots()

    ax_anim.set(xlim=[-3.0, 3.0], ylim=[-3.0, 3.0], xlabel='X [m]', ylabel='Y [m]')
    ax_anim.set_aspect('equal')
    ax_anim.legend()

    pitch = 0.0
    roll = 0.0
    friction = 1.0

    progress = 0.0
    virtual_speed = 0.5

    cone = friction * vehicle_specs["m"] * common.gz * math.cos(pitch) * math.cos(roll) / 4

    if( cone >= 0 ):
        fl_max = fr_max = cone
        fl_min = fr_min  = -cone
    
    elif( cone < 0 ):
        fl_max = fr_max = -cone
        fl_min = fr_min  = cone

    param = { 'pitch': pitch, 'roll': roll, 'friction': friction, 'fl_min': fl_min, 'fl_max': fl_max, 'fr_min': fr_min, 'fr_max': fr_max,\
             'fl_prev': 0.0, 'fr_prev': 0.0, 'x_ref': [], 'y_ref': [], 'yaw_ref': [] }

    opt_time = []
    cost = []

    for i in np.arange(range_aux):
        
        print("Iteration: ", i)

        p = progress

        x_r = []
        y_r = []
        yaw_r = []

        index = 0

        while index <= specs["N"]:
            x_r += [ float( x_ref(p) ) ]
            y_r += [ float( y_ref(p) ) ]
            yaw_r += [ float( yaw_ref(p) ) ]

            p += virtual_speed * specs["Ts"]

            index += 1

        param["x_ref"] = x_r
        param["y_ref"] = y_r

        if( i == 0 ):
            states = np.stack(next_state)
            controls = np.stack( [0.0, 0.0] )

            param["fl_prev"] = 0.0
            param["fr_prev"] = 0.0

        elif( i > 0 ):
            states = np.vstack( ( states, np.stack(next_state) ) )
            controls = np.vstack( ( controls, np.stack( [ fl_horizon[0], fr_horizon[0] ] ) ) )

            param["fl_prev"] = fl_horizon[0]
            param["fr_prev"] = fr_horizon[0]

        if( i == 0 ):
            res = model._solve( next_state, None, param )
            prev_res = res

        if( i > 0 ):
            res = model._solve( next_state, prev_res, param )
            prev_res = res
        
        stats = model._getStats()

        opt_time += [ stats["t_wall_total"] ]
        cost += [ stats["iterations"]["obj"][-1] ]

        x_horizon = res[0]
        y_horizon = res[1]
        yaw_horizon = res[2]
        vx_horizon = res[3]
        wz_horizon = res[4]
        fl_horizon = res[5]
        fr_horizon = res[6]

        horizon_path_plot, = ax_anim.plot( x_horizon, y_horizon, 'r' )
        ref_path_plot, = ax_anim.plot( x_r, y_r, 'k--' )

        next_state = np.stack( [ x_horizon[1], y_horizon[1], yaw_horizon[1], vx_horizon[1], wz_horizon[1] ] )

        fig_anim.canvas.draw()
        fig_anim.canvas.flush_events()

        horizon_path_plot.remove()
        ref_path_plot.remove()

        progress += virtual_speed * specs["Ts"]
    
    plt.ioff()
    plt.show()

    fig, ax = plt.subplots(2, 4)
    
    ax[0, 0].plot( states[:, 0] )
    ax[0, 0].set_title('x')

    ax[0, 1].plot( states[:, 1] )
    ax[0, 1].set_title('y')

    ax[0, 2].plot( states[:, 2] )
    ax[0, 2].set_title('Yaw')

    ax[0, 3].plot( states[:, 3] )
    ax[0, 3].set_title('vx')

    ax[1, 0].plot( states[:, 4] )
    ax[1, 0].set_title('wz')

    ax[1, 1].plot( controls[:, 0] )
    ax[1, 1].set_title('fl')

    ax[1, 2].plot( controls[:, 1] )
    ax[1, 2].set_title('fr')

    ax[1, 3].plot( states[:, 0], states[:, 1] )
    ax[1, 3].set_title('Position')

    fig2, ax2 = plt.subplots(1, 2)

    ax2[0].plot(cost)
    ax2[0].set_title('Cost')
    
    ax2[1].plot(opt_time)
    ax2[1].set_title('Opt time')

    plt.show()

    data2save = { 'cost': cost, 'opt_time': opt_time, 'x': list( states[:, 0] ), 'y': list( states[:, 1] ), 'progress': list( states[:, 3] ) }

    with open(common.results_folder + "Initial_Comparison_nmpc/" + "std_path.json", "w") as f:
        json.dump(data2save, f, indent=4)