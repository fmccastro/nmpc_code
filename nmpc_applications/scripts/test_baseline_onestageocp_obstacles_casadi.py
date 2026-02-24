#!/usr/bin/python3
from pathlib import Path
from scipy import ndimage
from nilearn.plotting.cm import _cmap_d as nilearn_cmaps

classes_path = Path(__file__)

upper_directory = classes_path.parent.parent
lower_directory = upper_directory / "src"

from sys import path
path.append( str(lower_directory) )

from classes.standard_baseline_casadi import *
from classes.common_class import *

"""
    Script to evaluate baseline one-stage ocp with and without closest-point optimization
    before simulation
"""

if __name__ == '__main__':
    
    common = Common()

    with open(common.baseline_std_dyn_skid) as f:
        solver_param = json.load(f)
        specs = solver_param["solver_specs"]
        con_pose = solver_param["con_pose"]
        con_vel = solver_param["con_vel"]
        weights = solver_param["weights"]
    
    with open(common.vehicle_specs) as g:
        vehicle_specs = json.load(g)

    range_aux = 1500
    
    ##  Generate reference (Lissajous curve)
    s = ca.SX.sym('s')
    
    #   Lissajous curve
    x_ref = ca.Function( 'x_ref', [s], [ 2*ca.sin(s + ca.pi/2.0) ] )
    y_ref = ca.Function( 'y_ref', [s], [ 2*ca.sin(2 * s) ] )
    yaw_ref = ca.Function( 'yaw_ref', [s], [ ca.atan2( 2 * ca.cos(2 * s), ca.cos(s + ca.pi/2.0) ) ] )
    curvature = ca.Function( 'curvature', [s], [ ( -4 * ca.cos(s + ca.pi/2.0) * ca.sin(2 * s) + 2 * ca.cos(2 * s) * ca.sin(s + ca.pi/2) ) / ( ca.cos(s + ca.pi/2)**2 + 4 * ca.cos(2 * s)**2 )**1.5 ] )

    #   Generate signed-distance field
    dy = dx = 0.025

    x = np.arange(-6.0, 6.0, dx)    # coordinates for columns
    y = np.arange(6.0, -6.0, -dy)    # coordinates for rows

    x_len = x.shape[0]
    y_len = y.shape[0]

    a = np.ones( ( x_len, y_len ) )
    
    X, Y = np.meshgrid(x, y, indexing="ij")

    mask1 = X**2 + Y**2 <= 0.2**2
    mask2 = (X>=-2.2) & (X<=-1.8) & (Y>=-0.1) & (Y<=0.1)
    mask3 = (X<=2.1) & (X>=1.9) & (Y>=-0.5) & (Y<=0.5)
    mask4 = (X - 1.4)**2 + (Y + 1.5)**2 <= 0.5**2
    mask5 = (X + 1.25)**2 + (Y - 0.78)**2 <= 0.15**2
    mask6 = ( (X + 1.81) / 0.35 )**2 + ( (Y - 1.4) / 0.1 )**2 <= 1
    mask7 = (X<=-0.89) & (X>=-1.7) & (Y>=-1.8) & (Y<=-1.7)

    a[mask1] = 0
    a[mask2] = 0
    a[mask3] = 0
    a[mask4] = 0
    a[mask5] = 0
    a[mask6] = 0
    a[mask7] = 0

    comp_a = a.copy()

    where_0 = np.where(comp_a == 0)
    where_1 = np.where(comp_a == 1)

    comp_a[where_0] = 1
    comp_a[where_1] = 0

    b = ndimage.distance_transform_edt(a, sampling = dx )
    b_comp = ndimage.distance_transform_edt(comp_a, sampling = dx )
    sdf = b - b_comp

    print("SDF set.")

    sdf_flat = sdf.ravel(order='F')
    sdf_sym = ca.interpolant('name', 'bspline', [-y, x], sdf_flat)
    print("Symbolic SDF set.\n")
    ###
    
    #   Set up nmpc countoring model
    model = Dynamics(x_ref, y_ref, yaw_ref, sdf = sdf_sym)
    closestPoint = ClosestPoint(x_ref, y_ref)

    initialPose = [-1.0, 0.5, ca.pi]

    print("Test at initial position: ", sdf_sym( [ -initialPose[1], initialPose[0] ] ) )

    #   Closest point optimization
    opt_progress = closestPoint._findInitialGuess(initialPose[0], initialPose[1], x_ref, y_ref)

    plt.figure()
    plt.scatter( initialPose[0], initialPose[1] )
    plt.plot( x_ref( np.arange(0, 2 * ca.pi, 0.01) ), y_ref( np.arange(0, 2 * ca.pi, 0.01) ), 'k--' )
    plt.scatter( float(x_ref( opt_progress ) ), float( y_ref( opt_progress ) ), c = 'b', marker='*', zorder = 2 )
    plt.show()

    progress_sol = closestPoint._solve( initialPose[0], initialPose[1], opt_progress )

    plt.figure()
    plt.scatter( initialPose[0], initialPose[1] )
    plt.plot( x_ref( np.arange(0, 2 * ca.pi, 0.01) ), y_ref( np.arange(0, 2 * ca.pi, 0.01) ), 'k--' )
    plt.scatter( float(x_ref( progress_sol ) ), float( y_ref( progress_sol ) ), c = 'r', marker='*', zorder = 2 )
    plt.scatter( float(x_ref( opt_progress ) ), float( y_ref( opt_progress ) ), c = 'b', marker='*', zorder = 2 )
    plt.show()
    
    next_state = [initialPose[0], initialPose[1], initialPose[2], float( progress_sol ), 0.0, 0.0]
    
    time_tot = []

    #   Set up animation figure
    plt.ion()
    fig_anim, ax_anim = plt.subplots()

    ax_anim.contourf(X, Y, a, cmap="binary_r")
    ax_anim.set( xlim=[-6.0, 6.0], ylim=[-6.0, 6.0], xlabel='X [m]', ylabel='Y [m]' )
    ax_anim.set_aspect('equal')
    ax_anim.legend()

    pitch = 0.0
    roll = 0.0
    friction = 1.0

    opt_time = []
    cost = []

    cone = friction * vehicle_specs["m"] * common.gz * math.cos(pitch) * math.cos(roll) / 4

    if( cone >= 0 ):
        fl_max = fr_max = cone
        fl_min = fr_min  = -cone
    
    elif( cone < 0 ):
        fl_max = fr_max = -cone
        fl_min = fr_min  = cone

    lbg = [-1e-3, -1e-3, -1e-3, -1e-3, -1e-3, -1e-3, 2e-1] * (specs["N"] + 1)
    ubg = [1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, ca.inf] * (specs["N"] + 1)

    for i in np.arange(range_aux):
        
        print("Iteration: ", i)

        if( i == 0 ):
            lbx = [next_state[0], next_state[1], next_state[2], progress_sol, con_vel["u_lb"], con_vel["r_lb"]]\
                + [con_vel["virtual_speed_lb"], fl_min, fr_min, -ca.inf, -ca.inf, -ca.inf, -ca.inf, con_vel["u_lb"], con_vel["r_lb"]] * (specs["N"] + 1)
        
            ubx = [next_state[0], next_state[1], next_state[2], progress_sol, con_vel["u_ub"], con_vel["r_ub"]]\
                + [con_vel["virtual_speed_ub"], fl_max, fr_max, ca.inf, ca.inf, ca.inf, ca.inf, con_vel["u_ub"], con_vel["r_ub"]] * (specs["N"] + 1)

        elif( i >  0 ):
            lbx = [next_state[0], next_state[1], next_state[2], next_state[3], con_vel["u_lb"], con_vel["r_lb"]]\
                + [con_vel["virtual_speed_lb"], fl_min, fr_min, -ca.inf, -ca.inf, -ca.inf, -ca.inf, con_vel["u_lb"], con_vel["r_lb"]] * (specs["N"] + 1)
        
            ubx = [next_state[0], next_state[1], next_state[2], next_state[3], con_vel["u_ub"], con_vel["r_ub"]]\
                + [con_vel["virtual_speed_ub"], fl_max, fr_max, ca.inf, ca.inf, ca.inf, ca.inf, con_vel["u_ub"], con_vel["r_ub"]] * (specs["N"] + 1)

        if( i == 0 ):
            states = np.array(next_state)
            controls = np.array( [0.0, 0.0, 0.0] )
        
        elif( i > 0 ):
            states = np.vstack( ( states, np.stack(next_state) ) )
            controls = np.vstack( ( controls, np.stack( [ fl_horizon[0], fr_horizon[0], virtual_speed_horizon[0] ] ) ) )

        if( i == 0 ):
            p = [pitch, 0, 0, 0, 2e-1]
            w0 = next_state + [0] * 9 * (specs["N"] + 1)
            res = model._solve(w0, lbx, ubx, lbg, ubg, p)

        if( i > 0 ):
            p = [pitch, fl_horizon[0], fr_horizon[0], virtual_speed_horizon[0], 2e-1]
            res = model._solve(w0, lbx, ubx, lbg, ubg, p)
        
        optcost, horizon = model._getStats(res)
        w0 = model._getInitialGuess(res)

        #opt_time += [ stats["t_proc_total"] ]
        cost += [ optcost ]

        x_horizon = horizon[0::9]
        y_horizon = horizon[1::9]
        yaw_horizon = horizon[2::9]
        progress_horizon = horizon[3::9]
        vx_horizon = horizon[4::9]
        wz_horizon = horizon[5::9]
        fl_horizon = horizon[6::9]
        fr_horizon = horizon[7::9]
        virtual_speed_horizon = horizon[8::9]

        horizon_path_plot, = ax_anim.plot( x_horizon, y_horizon, 'r' )
        ref_path_plot, = ax_anim.plot( x_ref( progress_horizon ), y_ref( progress_horizon ), 'k--' )

        #ax_anim.set_xlim( x_horizon[0] - 0.1, x_horizon[0] + 0.6)
        #ax_anim.set_ylim( y_horizon[0] - 0.3, y_horizon[0] + 0.3)
        
        #next_state = integrator._simulate( np.stack( next_state ), np.stack( [ fl_horizon[0], fr_horizon[0], virtual_speed_horizon[0] ] ), np.stack( [pitch] ) )

        next_state = [ x_horizon[1], y_horizon[1], yaw_horizon[1], progress_horizon[1], vx_horizon[1], wz_horizon[1] ]

        fig_anim.canvas.draw()
        fig_anim.canvas.flush_events()

        horizon_path_plot.remove()
        ref_path_plot.remove()
        
    plt.ioff()
    plt.show()

    fig, ax = plt.subplots(2, 6)
    
    ax[0, 0].plot( states[:, 0] )
    ax[0, 0].set_title('x')

    ax[0, 1].plot( states[:, 1] )
    ax[0, 1].set_title('y')

    ax[0, 2].plot( states[:, 2] )
    ax[0, 2].set_title('Yaw')
    
    ax[0, 3].plot( states[:, 3] )
    ax[0, 3].set_title('Progress')

    ax[0, 4].plot( states[:, 0], states[:, 1] )
    ax[0, 4].set_title('Position')

    ax[1, 0].plot( states[:, 4] )
    ax[1, 0].set_title('vx')

    ax[1, 1].plot( states[:, 5] )
    ax[1, 1].set_title('wz')

    ax[1, 2].plot( controls[:, 0] )
    ax[1, 2].set_title('fl')

    ax[1, 3].plot( controls[:, 1] )
    ax[1, 3].set_title('fr')

    ax[1, 4].plot( controls[:, 2] )
    ax[1, 4].set_title('Virtual speed')

    fig2, ax2 = plt.subplots(1, 2)

    ax2[0].plot(cost)
    ax2[0].set_title('Cost')
    
    ax2[1].plot(opt_time)
    ax2[1].set_title('Opt time')

    plt.show()

    with open(common.results_folder + "One stage OCP/" + "no_closest_point+" + "baseline_std.json", "w") as f:
        data2save = { 'cost': cost, 'opt_time': opt_time, 'x': list( states[:, 0] ), 'y': list( states[:, 1] ), 'progress': list( states[:, 3] ) }
        json.dump(data2save, f, indent=4)

    #with open(common.results_folder + "Initial_Comparison_nmpc/" + "" + "baseline_std.json", "w") as f:
    #    json.dump(data2save, f, indent=4)