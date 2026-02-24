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

cost_function_type = "Standard"
#cost_function_type = "Low_effort"

weights_type = "variable_weights"
reference_gen = "new_reference"

range_sim = 10000   #   Number of simulation iterations

std_noise_position = 3e-2
std_noise_heading = 5e-3

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
    ref = Reference(option = 2)

    x_ref = ref.__dict__["x_ref"]
    y_ref = ref.__dict__["y_ref"]
    yaw_ref = ref.__dict__["yaw_ref"]
    curvature = ref.__dict__["curvature"]
    
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

    index_tests = 0

    data2save = {}

    while(index_tests < 10):

        model = DynamicsRatesPath()

        #   Set up animation figure
        plt.ion()
        fig_anim, ax_anim = plt.subplots()

        ax_anim.set(xlim=[-6.0, 6.0], ylim=[-6.0, 6.0], xlabel='X [m]', ylabel='Y [m]')
        ax_anim.set_aspect('equal')
        ax_anim.legend()
        ###
        
        #   Choose starting reference point
        progress_sol = 1.0
        
        #   Initialize state: (x, y, yaw, vx, wz, tl, tr)
        next_state = np.stack( [x_ref(progress_sol).elements()[0], y_ref(progress_sol).elements()[0], ca.pi/2, 0.0, 0.0, 0.0, 0.0] )

        #   Set constraints (constraints are fixed in the planar case)
        model._setConstraints(f_lb, f_ub)

        progress = progress_sol
        progress_list = [progress]

        #   Set up increasing (or decreasing) weights along the control horizon
        model._setWeights(1.1)

        error = ( ( next_state[0] - x_ref(progress) )**2 + ( next_state[1] - y_ref(progress) )**2 )**0.5
        current_curvature = curvature(progress)

        #   Get virtual speed
        virtual_speed = model._getVirtualSpeed(error, current_curvature)

        #   Virtual speed QP
        opt_v_speed = 0.3 * virtual_speed
        prev_v_speed = opt_v_speed

        #   Set initial guess (warm-up solver)
        x_r, y_r, yaw_r = ref._getReference( progress, opt_v_speed, specs["Ts"], specs["N"] )
        model._setInitialGuess(20, next_state, x_r, y_r, yaw_r)

        prev_times = []
        cycle_times = []
        v_speed_list = []

        for i in np.arange(range_sim):
            
            start = time.time()

            #print("Iteration: ", i)

            #   Set reference to track
            if( reference_gen == "new_reference" and i > 0 ):
                error = ( ( next_state[0] - x_ref(progress) )**2 + ( next_state[1] - y_ref(progress) )**2 )**0.5
                current_curvature = curvature(progress)

                #   Get virtual speed
                virtual_speed = model._getVirtualSpeed(error, current_curvature)

                #   Virtual speed QP
                opt_v_speed = 0.3 * virtual_speed + 0.7 * prev_v_speed
                prev_v_speed = opt_v_speed

                #   Get reference
                x_r, y_r, yaw_r = ref._getReference( progress, opt_v_speed, specs["Ts"], specs["N"] )

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
        plt.close(fig=fig_anim)
        
        data2save[ str(index_tests) ] = { 'cost_p': cost_p,\
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

        index_tests += 1

    saveData = True

    if(saveData):
        with open(common.results_folder + "Real-time one stage planar OCP/Robustness tests/"\
                                        + cost_function_type + "_std_noise_position_" + str(std_noise_position) + "_std_noise_heading_" + str(std_noise_heading) + ".json", "w") as f:
            json.dump(data2save, f, indent=4)

        with open(common.results_folder + "Real-time one stage planar OCP/Robustness tests/"\
                                        + cost_function_type + "_std_noise_position_" + str(std_noise_position) + "_std_noise_heading_" + str(std_noise_heading) + "_specs.json", "w") as f:
            json.dump(solver_param, f, indent=4)