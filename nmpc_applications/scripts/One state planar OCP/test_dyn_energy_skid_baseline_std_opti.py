#!/usr/bin/python3
from pathlib import Path

classes_path = Path(__file__)

upper_directory = classes_path.parent.parent.parent
lower_directory = upper_directory / "src"

from sys import path
path.append( str(lower_directory) )

from classes.all_imports import *
from classes.standard_baseline_sol_opti import *
from classes.references import *
from classes.common_class import *

"""
    Script to evaluate baseline one-stage ocp with closest point optimization with solver parameter variation
    This script takes the solver that account for the control effort minimization
"""

if __name__ == '__main__':

    change_N = False
    change_Ts = False
    change_weights = True

    if( change_weights ):
        #controllerType = "Pose_tracker"
        #controllerType = "Position_tracker"
        #controllerType = "Heading_tracker"
        #controllerType = "Focus_on_low_control_effort"
        controllerType = "Focus_on_smooth_control"

    common = Common()
    ref = Reference()

    with open(common.baseline_std_dyn_skid) as f:
        solver_param = json.load(f)
        specs = solver_param["solver_specs"]
        con_pose = solver_param["con_pose"]
        con_vel = solver_param["con_vel"]
        weights = solver_param["weights"]
    
    with open(common.vehicle_specs) as g:
        vehicle_specs = json.load(g)

    range_aux = 1500

    #   Set window size
    win_len = 6.0
    
    #   Generate reference
    x_ref = ref.__dict__["x_ref"]
    y_ref = ref.__dict__["y_ref"]
    yaw_ref = ref.__dict__["yaw_ref"]
    curvature = ref.__dict__["curvature"]

    #   Set up nmpc countoring model
    model = DynamicsEnergy(x_ref, y_ref, yaw_ref)
    closestPoint = ClosestPoint(x_ref, y_ref)

    initialPose = np.array( [-4.0, -2.0, -ca.pi] )
    
    opt_progress = closestPoint._findInitialGuess(initialPose[0], initialPose[1], x_ref, y_ref)
    progress_sol = closestPoint._solve( initialPose[0], initialPose[1], opt_progress )

    plt.figure()
    plt.scatter( initialPose[0], initialPose[1] )
    plt.plot( x_ref( np.arange(0, 2 * ca.pi, 0.01) ), y_ref( np.arange(0, 2 * ca.pi, 0.01) ), 'k--' )
    plt.scatter( float(x_ref( progress_sol ) ), float( y_ref( progress_sol ) ), c = 'r', marker='*', zorder = 2 )
    plt.scatter( float(x_ref( opt_progress ) ), float( y_ref( opt_progress ) ), c = 'b', marker='*', zorder = 2 )
    plt.show()
    
    next_state = [ initialPose[0], initialPose[1], initialPose[2], float( progress_sol ), 0.0, 0.0]
    
    time_tot = []

    #   Set up animation figure
    plt.ion()
    fig_anim, ax_anim = plt.subplots()

    ax_anim.set( xlim=[-win_len, win_len], ylim=[-win_len, win_len], xlabel='X [m]', ylabel='Y [m]' )
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

    param = { 'pitch': pitch, 'roll': roll, 'friction': friction, 'fl_min': fl_min, 'fl_max': fl_max, 'fr_min': fr_min, 'fr_max': fr_max, 'virtual_speed_prev': 0.0, 'fl_prev': 0.0, 'fr_prev': 0.0, 'friction': friction }
    
    for i in np.arange(range_aux):

        if( i == 0 ):
            states = np.stack(next_state)
            controls = np.stack( [0.0, 0.0, 0.0] )

            param["fl_prev"] = 0.0
            param["fr_prev"] = 0.0
            param["virtual_speed_prev"] = 0.0

            res = model._solve( next_state, None, param )
            prev_res = res
        
        elif( i > 0 ):
            states = np.vstack( ( states, np.stack(next_state) ) )
            controls = np.vstack( ( controls, np.stack( [ fl_horizon[0], fr_horizon[0], virtual_speed_horizon[0] ] ) ) )

            param["fl_prev"] = fl_horizon[0]
            param["fr_prev"] = fr_horizon[0]
            param["virtual_speed_prev"] = virtual_speed_horizon[0]

            res = model._solve( next_state, prev_res, param )
            prev_res = res
            
        stats = model._getStats()
        
        #print(stats)

        opt_time += [ stats["t_proc_total"] ]
        cost += [ stats["iterations"]["obj"][-1] ]

        x_horizon = res[0]
        y_horizon = res[1]
        yaw_horizon = res[2]
        progress_horizon = res[3]
        vx_horizon = res[4]
        wz_horizon = res[5]
        fl_horizon = res[6]
        fr_horizon = res[7]
        virtual_speed_horizon = res[8]

        horizon_path_plot, = ax_anim.plot( x_horizon, y_horizon, 'r' )
        ref_path_plot, = ax_anim.plot( x_ref( progress_horizon ), y_ref( progress_horizon ), 'k--' )

        #ax_anim.set_xlim( x_horizon[0] - 0.1, x_horizon[0] + 0.6)
        #ax_anim.set_ylim( y_horizon[0] - 0.3, y_horizon[0] + 0.3)
        
        #next_state = integrator._simulate( np.stack( next_state ), np.stack( [ fl_horizon[0], fr_horizon[0], virtual_speed_horizon[0] ] ), np.stack( [pitch] ) )

        next_state = [ x_horizon[1], y_horizon[1], yaw_horizon[1], progress_horizon[1], vx_horizon[1], wz_horizon[1] ]

        #print("Position: ", next_state[0], next_state[1])
        #print("Distance to nearest obstacle: ", sdf_sym( [ -next_state[1], next_state[0] ] ) )
        #print("\n")

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

    saveData = True

    if(saveData):
        if(change_N):
            with open(common.results_folder + "One stage OCP/Solver specs comparison/horizon/Virtual speed controlled inside OCP/" + str(specs["N"]) + ".json", "w") as f:
                data2save = { 'cost': cost, 'opt_time': opt_time, 'x': list( states[:, 0] ), 'y': list( states[:, 1] ), 'progress': list( states[:, 3] ), 'progress_0': progress_sol,\
                               'yaw': list( states[:, 2] ), 'fl': list( controls[:, 0] ), 'fr': list( controls[:, 1] ),\
                               'x_ref': x_ref( states[:, 3] ).elements(), 'y_ref': y_ref( states[:, 3] ).elements(), 'yaw_ref': yaw_ref( states[:, 3] ).elements() }
                json.dump(data2save, f, indent=4)
            
            with open(common.results_folder + "One stage OCP/Solver specs comparison/horizon/Virtual speed controlled inside OCP/" + str(specs["N"]) + "_specs.json", "w") as f:
                json.dump(solver_param, f, indent=4)
        
        elif(change_Ts):
            with open(common.results_folder + "One stage OCP/Solver specs comparison/sampling time/Virtual speed controlled inside OCP/" + str(specs["Ts"]) + ".json", "w") as f:
                data2save = { 'cost': cost, 'opt_time': opt_time, 'x': list( states[:, 0] ), 'y': list( states[:, 1] ), 'progress': list( states[:, 3] ), 'progress_0': progress_sol,\
                               'yaw': list( states[:, 2] ), 'fl': list( controls[:, 0] ), 'fr': list( controls[:, 1] ),\
                               'x_ref': x_ref( states[:, 3] ).elements(), 'y_ref': y_ref( states[:, 3] ).elements(), 'yaw_ref': yaw_ref( states[:, 3] ).elements() }
                json.dump(data2save, f, indent=4)
            
            with open(common.results_folder + "One stage OCP/Solver specs comparison/sampling time/Virtual speed controlled inside OCP/" + str(specs["Ts"]) + "_specs.json", "w") as f:
                json.dump(solver_param, f, indent=4)

        elif(change_weights):
            with open(common.results_folder + "One stage OCP/Solver specs comparison/weights/Virtual speed controlled inside OCP/" + controllerType + ".json", "w") as f:
                data2save = { 'cost': cost, 'opt_time': opt_time, 'x': list( states[:, 0] ), 'y': list( states[:, 1] ), 'progress': list( states[:, 3] ), 'progress_0': progress_sol,\
                               'yaw': list( states[:, 2] ), 'vx': list( states[:, 4] ), 'wz': list( states[:, 5] ), 'fl': list( controls[:, 0] ), 'fr': list( controls[:, 1] ),\
                               'virtual_speed': list( controls[:, 2] ), 'x_ref': x_ref( states[:, 3] ).elements(), 'y_ref': y_ref( states[:, 3] ).elements(), 'yaw_ref': yaw_ref( states[:, 3] ).elements() }
                json.dump(data2save, f, indent=4)
            
            with open(common.results_folder + "One stage OCP/Solver specs comparison/weights/Virtual speed controlled inside OCP/" + controllerType + "_specs.json", "w") as f:
                json.dump(solver_param, f, indent=4)