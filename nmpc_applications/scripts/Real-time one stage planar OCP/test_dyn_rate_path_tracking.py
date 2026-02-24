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
    
custom_initial_pose = False
compare_forces_cum = False

#   Script parameters
robustness_test = False  #   Noise amplitude to test controller reaction (robusteness test)

cost_function_type = "Standard"
#cost_function_type = "Low_effort"

#weights_type = "constant_weights"
weights_type = "variable_weights"

reference_gen = "new_reference"
#reference_gen = "moving_reference"

range_sim = 1000   #   Number of simulation iterations

if(robustness_test):
    std_noise_position = 0e-1
    std_noise_heading = 0e-1

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

    #   Set up nmpc countoring model
    model = DynamicsRatesPath()
    ref = Reference(option = 0)

    x_ref = ref.__dict__["x_ref"]
    y_ref = ref.__dict__["y_ref"]
    yaw_ref = ref.__dict__["yaw_ref"]
    curvature = ref.__dict__["curvature"]

    #   Set up animation figure
    plt.ion()
    fig_anim, ax_anim = plt.subplots()

    ax_anim.set(xlim=[-6.0, 6.0], ylim=[-6.0, 6.0], xlabel='X [m]', ylabel='Y [m]')
    ax_anim.set_aspect('equal')
    ax_anim.legend()
    ###

    #   Find closest point
    #progress_guess = closest_pt._findInitialGuess( next_state[0], next_state[1], x_ref, y_ref )
    #progress_sol = closest_pt._solve( next_state[0], next_state[1], progress_guess )
    progress_sol = 1.0

    #   Initialize state: (x, y, yaw, vx, wz, tl, tr)
    if(custom_initial_pose):
        next_state = np.stack( [x_ref(progress_sol).elements()[0], y_ref(progress_sol).elements()[0], -ca.pi/2, 0.0, 0.0, 0.0, 0.0] )

    else:
        next_state = np.stack( [x_ref(progress_sol).elements()[0], y_ref(progress_sol).elements()[0], yaw_ref(progress_sol).elements()[0], 0.0, 0.0, 0.0, 0.0] )
    
    prev_times = []
    cycle_times = []
    
    #   Define Coulomb friction cone for each wheel (planar case)
    cone = vehicle_specs["m"] * solver_param["solver_specs"]["gz"] / 4

    #   Set wheel forces upper and lower limits based on the Coulomb friction model
    #  (planar case; constant throughout the simulation)
    if( cone >= 0 ):
        f_lb = -cone
        f_ub = cone

    else:
        f_lb = cone
        f_ub = -cone

    #   Set constraints (constraints are fixed in the planar case)
    model._setConstraints(f_lb, f_ub)

    progress = progress_sol
    progress_list = [progress]
    v_speed_list = []

    #   Set up increasing (or decreasing) weights along the control horizon
    if( weights_type == "variable_weights" ):
        model._setWeights(1.1, f_ub)

    if(reference_gen == "new_reference"):
        error = ( ( next_state[0] - x_ref(progress) )**2 + ( next_state[1] - y_ref(progress) )**2 )**0.5
        current_curvature = curvature(progress)

        #   Get virtual speed
        virtual_speed = model._getVirtualSpeed(error, current_curvature, solver_param)

        #   Virtual speed QP
        opt_v_speed = 0.3 * virtual_speed
        prev_v_speed = opt_v_speed

        #   Set initial guess (warm-up solver)
        x_r, y_r, yaw_r = ref._getReference( progress, opt_v_speed, specs["Ts"], specs["N"] )
        model._setInitialGuess(20, next_state, x_r, y_r, yaw_r)

    elif( reference_gen == "moving_reference" ):
        #   Virtual speed QP
        opt_v_speed = 0.0
        prev_v_speed = 0.0

        #   Set initial guess (warm-up solver)
        x_r, y_r, yaw_r = ref._getReference( progress, 0.0, specs["Ts"], specs["N"] )
        model._setInitialGuess(20, next_state, x_r, y_r, yaw_r)

    for i in np.arange(range_sim):

        start = time.time()

        print("Iteration: ", i)

        #   Set reference to track
        if( reference_gen == "new_reference" and i > 0 ):
            error = ( ( next_state[0] - x_ref(progress) )**2 + ( next_state[1] - y_ref(progress) )**2 )**0.5
            current_curvature = curvature(progress)
            
            #   Get virtual speed
            virtual_speed = model._getVirtualSpeed(error, current_curvature, solver_param)

            #   Virtual speed QP
            opt_v_speed = 0.3 * virtual_speed + 0.7 * prev_v_speed
            prev_v_speed = opt_v_speed

            #   Get reference
            x_r, y_r, yaw_r = ref._getReference( progress, opt_v_speed, specs["Ts"], specs["N"] )
            
            #   Transform regulation problem into tracking problem
            model._setReference(x_r, y_r, yaw_r)

        elif( reference_gen == "moving_reference" and i == 0 ):
            x_r, y_r, yaw_r = ref._getReference( progress, opt_v_speed, specs["Ts"], specs["N"] )

        elif( reference_gen == "moving_reference" and i > 0 ):
            error = np.sum( np.power( np.array(x_horizon) - np.array(x_r), 2) + np.power( np.array(y_horizon) - np.array(y_r), 2) )
            current_curvature = curvature(progress)
            
            #   Get virtual speed
            virtual_speed = model._getVirtualSpeed(error, current_curvature, solver_param, option = 1)

            #   Virtual speed QP
            opt_v_speed = 0.3 * virtual_speed + 0.7 * prev_v_speed
            prev_v_speed = opt_v_speed

            #   Get reference
            next_progress = progress + opt_v_speed * specs["Ts"]
            
            x_r, y_r, yaw_r = x_r[1:] + [ float( x_ref(next_progress) ) ],\
                              y_r[1:] + [ float( y_ref(next_progress) ) ],\
                              yaw_r[1:] + [ float( yaw_ref(next_progress) ) ]

            #   Transform regulation problem into tracking problem
            model._setReference(x_r, y_r, yaw_r)

        if( i == 0 ):
            states = np.stack(next_state)

        elif( i == 1 ):
            states = np.vstack( ( states, np.stack(next_state) ) )
            controls = np.stack( [ res[0], res[1] ] )
            
        elif( i > 1 ):
            states = np.vstack( ( states, np.stack(next_state) ) )
            controls = np.vstack( ( controls, np.stack( [ res[0], res[1] ] ) ) )

        if( specs["solver"] == "SQP_RTI" ):
            model._preparation_sqp_rti()
            new_cost_p, new_opt_time_p = model._getData()
            res, _ = model._feedback_sqp_rti(next_state)
            new_cost_f, new_opt_time_f = model._getData()

            if( i == 0 ):
                cost_p = [new_cost_p]
                time_p = [new_opt_time_p]
                cost_f = [new_cost_f]
                time_f = [new_opt_time_f]

            else:
                cost_p += [new_cost_p]
                time_p += [new_opt_time_f]
                cost_f += [new_cost_f]
                time_f += [new_opt_time_f]

        elif( specs["solver"] == "SQP" ):
            res, _ = model._solve_sqp(next_state)
            new_cost, new_opt_time = model._getData()

            if( i == 0 ):
                cost_sqp = [new_cost]
                time_tot = [new_opt_time]

            else:
                cost_sqp += [new_cost]
                time_tot += [new_opt_time]
        
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

        if( robustness_test ):
            next_state[0] = next_state[0] + np.random.normal(0.0, std_noise_position, 1)
            next_state[1] = next_state[1] + np.random.normal(0.0, std_noise_position, 1)
            next_state[2] = next_state[2] + np.random.normal(0.0, std_noise_heading, 1)

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

        #   Retrieve the previous 5 cycle times
        if( len(prev_times) < 5 ):
            prev_times += [end]
        
        else:
            prev_times.pop(0)
            prev_times += [end]
        ###
        
        if( i > 0 ):
            progress += opt_v_speed * sim_time
            
            progress_list += [ progress ]
            v_speed_list += [ opt_v_speed ]

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

    ax[1, 5].plot( v_speed_list )
    ax[1, 5].set_title('Virtual speed')

    fig2, ax2 = plt.subplots(1, 3)

    if( specs["solver"] == "SQP_RTI" ):
        ax2[0].plot(cost_p, 'r')
        ax2[0].plot(cost_f, 'b')
        ax2[0].set_title('cost')
        
        ax2[1].plot(time_p)
        ax2[1].plot(time_f)
        ax2[1].plot( specs["Ts"] * np.ones( len(time_p) ), 'r--')
        ax2[1].set_title('Opt. times')

        ax2[2].plot(cycle_times)
        ax2[2].plot( specs["Ts"] * np.ones( len(time_p) ), 'r--')
        ax2[2].set_title('Loop times')
    
    elif( specs["solver"] == "SQP" ):
        ax2[0].plot(cost_sqp, 'b')
        ax2[0].set_title('cost')
        
        ax2[1].plot(time_tot, 'b')
        ax2[1].plot( specs["Ts"] * np.ones( len(time_tot) ), 'r--')
        ax2[1].set_title('Opt. times')

        ax2[2].plot(cycle_times)
        ax2[2].plot( specs["Ts"] * np.ones( len(time_tot) ), 'r--')
        ax2[2].set_title('Loop times')

    plt.show()

    saveData = False

    if(saveData):
        if(compare_forces_cum):
            with open(common.results_folder + "Real-time one stage planar OCP/Path tracking/Compare forces cumulation/" + cost_function_type + "+" + weights_type + "+" + reference_gen + "2.json", "w") as f:
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
                                'fr_rate': list( controls[:, 1] ) }
                json.dump(data2save, f, indent=4)
            
            with open(common.results_folder + "Real-time one stage planar OCP/Path tracking/Compare forces cumulation/" + cost_function_type + "+" + weights_type + "+" + reference_gen + "2_specs.json", "w") as f:
                json.dump(solver_param, f, indent=4)
        
        else:
            if(not custom_initial_pose):
                with open(common.results_folder + "Real-time one stage planar OCP/Path tracking/Outside reference planner/" + cost_function_type + "+" + weights_type + "+" + reference_gen + ".json", "w") as f:
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
                                    'fr_rate': list( controls[:, 1] ) }
                    json.dump(data2save, f, indent=4)
                
                with open(common.results_folder + "Real-time one stage planar OCP/Path tracking/Outside reference planner/" + cost_function_type + "+" + weights_type + "+" + reference_gen + "_specs.json", "w") as f:
                    json.dump(solver_param, f, indent=4)

            else:
                with open(common.results_folder + "Real-time one stage planar OCP/Path tracking/Custom initial pose/" + cost_function_type + "+" + weights_type + "+" + reference_gen + ".json", "w") as f:
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
                                    'fr_rate': list( controls[:, 1] ) }
                    json.dump(data2save, f, indent=4)
                
                with open(common.results_folder + "Real-time one stage planar OCP/Path tracking/Custom initial pose/" + cost_function_type + "+" + weights_type + "+" + reference_gen + "_specs.json", "w") as f:
                    json.dump(solver_param, f, indent=4)