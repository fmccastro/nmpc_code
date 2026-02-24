#!/usr/bin/python3
from pathlib import Path

classes_path = Path(__file__)

upper_directory = classes_path.parent.parent.parent
lower_directory = upper_directory / "src"

from sys import path
path.append( str(lower_directory) )

from classes.standard_formulation import *
from classes.references import *
from classes.common_class import *

"""
    Script to test real-time baseline standard formulation algorithm
"""

if __name__ == '__main__':
    
    common = Common()

    with open(common.baseline_std_dyn_skid) as f:
        solver_param = json.load(f)
        specs = solver_param["solver_specs"]
        con_pose = solver_param["con_pose"]
        con_vel = solver_param["con_vel"]
        weights = solver_param["weights"]

        progress_lb = solver_param["progress_lb"]
        progress_ub = solver_param["progress_ub"]
    
    with open(common.vehicle_specs) as g:
        vehicle_specs = json.load(g)

    range_aux = 2000

    #   Set up nmpc countoring model
    model = DynamicsRatesPath()
    
    #   (x, y, yaw, vx, wz, tl, tr)
    next_state = np.stack( [0.0, 0.0, ca.pi/2, 0.0, 0.0, 0.0, 0.0] )

    cost_sqp = []

    cost_p = []
    cost_f = []

    time_p = []
    time_f = []

    #   Set up animation figure
    plt.ion()
    fig_anim, ax_anim = plt.subplots()

    ax_anim.set(xlim=[-6.0, 6.0], ylim=[-6.0, 6.0], xlabel='X [m]', ylabel='Y [m]')
    ax_anim.set_aspect('equal')
    ax_anim.legend()
    
    pitch = 0.0

    position_reg = True

    horizon_var = False
    sampling_time_var = False
    weights_var = True

    if( position_reg ):
        x_r = [2.0] * ( specs["N"] + 1 )
        y_r = [1.0] * ( specs["N"] + 1 )
        yaw_r = [0.0] * ( specs["N"] + 1 )
    
    else:
        x_r = [0.0] * ( specs["N"] + 1 )
        y_r = [0.0] * ( specs["N"] + 1 )
        yaw_r = [ca.pi/2] * ( specs["N"] + 1 )

    #   Set initial guess
    model._setInitialGuess(20, next_state, x_r, y_r, yaw_r)

    prev_times = []
    cycle_times = []

    cone = vehicle_specs["m"] * solver_param["solver_specs"]["gz"] / 4

    for i in np.arange(range_aux):

        start = time.time()
        
        print("Iteration: ", i)

        if( cone >= 0 ):
            f_lb = -cone
            f_ub = cone

        else:
            f_lb = cone 
            f_ub = -cone

        #   Transform regulation problem into tracking problem
        model._updateSolver(x_r, y_r, yaw_r, f_lb, f_ub)

        if( i == 0 ):
            states = np.stack(next_state)

        elif( i == 1 ):
            states = np.vstack( ( states, np.stack(next_state) ) )
            controls = np.stack( [ res[0], res[1] ] )
        
        elif( i > 1 ):
            states = np.vstack( ( states, np.stack(next_state) ) )
            controls = np.vstack( ( controls, np.stack( [ res[0], res[1] ] ) ) )

        if( i == 0 ):
            if( specs["solver"] == "SQP_RTI" ):
                model._preparation_sqp_rti()
                res, _ = model._feedback_sqp_rti( next_state )

            elif( specs["solver"] == "SQP" ):
                res, _ = model._solve_sqp(next_state)
                cost, opt_time = model._getData()

                cost_sqp += [cost]
                time_tot += [opt_time]

            prev_res = res

        else:
            if( specs["solver"] == "SQP_RTI" ):
                model._preparation_sqp_rti()

                cost, opt_time = model._getData()

                cost_p += [cost]
                time_p += [opt_time]

                res, _ = model._feedback_sqp_rti( next_state )

                cost, opt_time = model._getData()

                cost_f += [cost]
                time_f += [opt_time]
            
            elif( specs["solver"] == "SQP" ):
                res, _ = model._solve_sqp(next_state)
                cost, opt_time = model._getData()

                cost_sqp += [cost]
                time_tot += [opt_time]
                
            prev_res = res

        ###
        cut_start_1 = time.time()

        states_horizon = model._getStatesFlat()
        control_horizon = model._getControlsFlat()

        x_horizon = states_horizon[::7]
        y_horizon = states_horizon[1::7]
        yaw_horizon = states_horizon[2::7]
        vx_horizon = states_horizon[3::7]
        wz_horizon = states_horizon[4::7]
        fl_horizon = states_horizon[5::7]
        fr_horizon = states_horizon[6::7]

        fl_rate_horizon = control_horizon[0::2]
        fr_rate_horizon = control_horizon[1::2]

        horizon_path_plot, = ax_anim.plot( x_horizon, y_horizon, 'r' )
        ref_path_plot, = ax_anim.plot( x_r, y_r, 'k--' )

        cut_end_1 = time.time() - cut_start_1
        ###

        if( len(prev_times) == 0 ):
            sim_time = specs["Ts"]

        elif( len(prev_times) > 0 ):
            sim_time = sum(prev_times) / len(prev_times)

        next_state = model._simulate( np.stack( next_state ), np.stack( [ res[0], res[1] ] ), sim_time )

        ###
        cut_start_2 = time.time()
        
        fig_anim.canvas.draw()
        fig_anim.canvas.flush_events()

        horizon_path_plot.remove()
        ref_path_plot.remove()

        cut_end_2 = time.time() - cut_start_2
        ###
        
        end = time.time() - start - cut_end_2 - cut_end_1
    
        cycle_times += [end]

        print("Time: ", end)

        if( len(prev_times) < 5 ):
            prev_times += [end]
        
        else:
            prev_times.pop(0)
            prev_times += [end]

    plt.ioff()
    plt.show()

    fig, ax = plt.subplots(2, 5)

    ax[0, 0].plot( states[:, 0] )
    ax[0, 0].set_title('x')

    ax[0, 1].plot( states[:, 1] )
    ax[0, 1].set_title('y')

    ax[0, 2].plot( states[:, 2] )
    ax[0, 2].set_title('Yaw')

    ax[0, 3].plot( states[:, 3] )
    ax[0, 3].set_title('vx')

    ax[0, 4].plot( states[:, 4] )
    ax[0, 4].set_title('wz')

    ax[1, 0].plot( states[:, 5] )
    ax[1, 0].set_title('fl')

    ax[1, 1].plot( states[:, 6] )
    ax[1, 1].set_title('fr')

    ax[1, 2].plot( controls[:, 0] )
    ax[1, 2].set_title('fl_rate')

    ax[1, 3].plot( controls[:, 1] )
    ax[1, 3].set_title('fr_rate')

    ax[1, 4].plot( states[:, 0], states[:, 1] )
    ax[1, 4].set_title('Position')

    fig2, ax2 = plt.subplots(1, 3)

    if( specs["solver"] == "SQP_RTI" ):
        ax2[0].plot(cost_p, 'r')
        ax2[0].plot(cost_f, 'b')
        ax2[0].set_title('cost')
        
        ax2[1].plot(time_p)
        ax2[1].plot(time_f)
        ax2[1].plot( specs["Ts"] * np.ones( len(time_p) ), 'r--')
        ax2[1].set_title('Total opt. time')

        ax2[2].plot(cycle_times)
        ax2[2].plot( specs["Ts"] * np.ones( len(time_p) ), 'r--')
        ax2[2].set_title('Total opt. time')

    plt.show()

    saveData = False

    if(position_reg):
        reg_type = "Position regulation/"
    
    else:
        reg_type = "Heading regulation/"
    
    if(horizon_var):
        analysis_type = "horizon/"
        data_q = str(specs["N"])
    
    if(sampling_time_var):
        analysis_type = "sampling_time/"
        data_q = str(specs["Ts"])
    
    if(weights_var):
        analysis_type = "weights/"
        #data_q = "Standard"
        #data_q = "Focus on position tracking"
        #data_q = "Focus on heading tracking"
        data_q = "Focus on energy saving"
        #data_q = "Focus on smooth controls"

    if(saveData):
        with open(common.results_folder + "Real-time one stage planar OCP/" + reg_type + analysis_type + data_q + ".json", "w") as f:
            data2save = { 'cost_p': cost_p,\
                        'cost_f': cost_f,\
                        'time_p': time_p,\
                        'time_f': time_f,\
                        'cycle_times': cycle_times,\
                        'x': list( states[:, 0] ),\
                        'y': list( states[:, 1] ),\
                        'yaw': list( states[:, 2] ),\
                        'vx': list( states[:, 3] ),\
                        'wz': list( states[:, 4] ),\
                        'fl': list( states[:, 5] ),\
                        'fr': list( states[:, 6] ),\
                        'fl_rate': list( controls[:, 0] ),\
                        'fr_rate': list( controls[:, 1] ),\
                        'x_reg': x_r,\
                        'y_reg': y_r,\
                        'yaw_reg': yaw_r }
            json.dump(data2save, f, indent=4)
        
        with open(common.results_folder + "Real-time one stage planar OCP/" + reg_type + analysis_type + data_q + "_specs.json", "w") as f:
            json.dump(solver_param, f, indent=4)