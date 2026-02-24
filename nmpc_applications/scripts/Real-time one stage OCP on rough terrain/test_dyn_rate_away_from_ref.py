#!/usr/bin/python3
import os
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
    Script to test real-time baseline standard formulation algorithm on rough terrain
""" 

def _plotData(axes, fig, states_horizon, state):

    x_horizon = states_horizon[::9]
    y_horizon = states_horizon[1::9]

    """
    yaw_horizon = states_horizon[2::7]
    vx_horizon = states_horizon[3::7]
    wz_horizon = states_horizon[4::7]
    fl_horizon = states_horizon[5::7]
    fr_horizon = states_horizon[6::7]
    
    fl_rate_horizon = control_horizon[0::2]
    fr_rate_horizon = control_horizon[1::2]
    """

    horizon_path_plot = axes.scatter( x_horizon, y_horizon, c='r', s=4.0 )
    ref_path_plot = axes.scatter( x_r, y_r, c='k', s=4.0 )
    #state_plot = axes.scatter( state[0], state[1], c='g', s=10.0 )

    fig.canvas.draw()
    fig.canvas.flush_events()

    return horizon_path_plot, ref_path_plot

def _removePlotData(plots):

    for plot in plots:
        plot.remove()

custom_initial_pose = True

#   Script parameters
robustness_test = True  #   Noise amplitude to test controller reaction (robusteness test)

cost_function_type = "Standard"
#cost_function_type = "Low_effort"

weights_type = "variable_weights"
reference_gen = "new_reference"

range_sim = 100000   #   Number of simulation iterations

if(robustness_test):
    std_noise_position = 0.01
    std_noise_heading = 0.001

#   Coefficient of friction
niu = 0.7

std_noise_roll = 7e-2
std_noise_pitch = 7e-2

if __name__ == '__main__':
    
    common = Common()
    
    map_option = 1
    _qa = 1.0
    _qb = 1.0
    _qc = 0.4
    
    with open(common.baseline_std_dyn_skid) as f:
        solver_param = json.load(f)
        specs = solver_param["solver_specs"]
        con_pose = solver_param["con_pose"]
        con_vel = solver_param["con_vel"]
        weights = solver_param["weights"]
        virtual_speed_planner = solver_param["virtual_speed_planner"]

        progress_lb = solver_param["progress_lb"]
        progress_ub = solver_param["progress_ub"]

    with open(common.vehicle_specs) as g:
        vehicle_specs = json.load(g)

    #   Set up nmpc countoring model
    model = DynamicsRatesPathRoughTerrain()
    ref = Reference(option = map_option, qa=_qa, qb=_qb, qc=_qc)

    x_ref = ref.__dict__["x_ref"]
    y_ref = ref.__dict__["y_ref"]
    yaw_ref = ref.__dict__["yaw_ref"]
    curvature = ref.__dict__["curvature"]
    roll_ref = ref.__dict__["roll_ref"]
    pitch_ref = ref.__dict__["pitch_ref"]

    closest_point = ClosestPoint(x_ref, y_ref)
    
    #   Set up animation figure
    plt.ion()
    fig_anim, ax_anim = plt.subplots()

    ax_anim.set(xlim=[-6.0, 6.0], ylim=[-6.0, 6.0], xlabel='X [m]', ylabel='Y [m]')
    ax_anim.set_aspect('equal')
    ax_anim.legend()
    ###

    #   Initialize state: (x, y, roll, pitch, yaw, vx, wz, tl, tr)
    if( custom_initial_pose ):
        initial_state = next_state = np.stack( [ -2.0, 0.0, 0.0, 0.0, 3 * ca.pi / 2.0, 0.0, 0.0, 0.0, 0.0] )

        #   Find closest point 
        closest_point_guess = closest_point._findInitialGuess(next_state[0], next_state[1])
        closest_point_progress = closest_point._solve(next_state[0], next_state[1], closest_point_guess)

        plt.figure()
        plt.scatter( next_state[0], next_state[1] )
        plt.plot( x_ref( np.arange(0, 2 * ca.pi, 0.01) ), y_ref( np.arange(0, 2 * ca.pi, 0.01) ), 'k--' )
        plt.scatter( float(x_ref( closest_point_guess ) ), float( y_ref( closest_point_guess ) ), c = 'r', marker='*', zorder = 2 )
        plt.scatter( float(x_ref( closest_point_progress ) ), float( y_ref( closest_point_progress ) ), c = 'b', marker='*', zorder = 2 )
        plt.show()

        progress_sol = closest_point_progress

    else:
        progress_sol = 0.92
        initial_state = next_state = np.stack( [ x_ref(progress_sol).elements()[0], y_ref(progress_sol).elements()[0],\
                                                    0.0, 0.0, yaw_ref(progress_sol).elements()[0] + ca.pi, 0.0, 0.0, 0.0, 0.0] )

    prev_times = []
    cycle_times = []

    #   Update progress
    progress = progress_sol
    progress_list = [progress]
    v_speed_list = []

    roll_list = []
    pitch_list = []

    prep_status = []
    feedback_status = []
    
    for i in np.arange(range_sim):
        
        start = time.time()

        print("Sim iteration: ", i)

        #   Set reference to track
        error = ( ( next_state[0] - x_ref(progress) )**2 + ( next_state[1] - y_ref(progress) )**2 )**0.5
        current_curvature = curvature(progress)
        
        #   Get virtual speed
        virtual_speed = model._getVirtualSpeed(error, current_curvature, solver_param)
        
        if(i == 0):
            prev_v_speed = 0.0

        #   Virtual speed QP
        opt_v_speed = virtual_speed_planner["v_weight"] * virtual_speed + virtual_speed_planner["prev_v_weight"] * prev_v_speed
        prev_v_speed = opt_v_speed

        #   Get reference
        x_r, y_r, yaw_r = ref._getReference( progress, opt_v_speed, specs["Ts"], specs["N"] )

        if( i == 0 ):
            #   Get and save roll and pitch
            next_roll = roll_ref( next_state[0], next_state[1], next_state[4] ).elements()[0] + np.random.normal(0.0, std_noise_roll)
            next_pitch = pitch_ref( next_state[0], next_state[1], next_state[4] ).elements()[0] + np.random.normal(0.0, std_noise_pitch)

            next_state[2] = next_roll
            next_state[3] = next_pitch
            
            roll_list += [next_roll]
            pitch_list += [next_pitch]

        if(i == 0):
            #   Set up increasing (or decreasing) weights along the control horizon
            if( weights_type == "variable_weights" ):
                model._setWeights( weights["var_weight_coefficient"] )

            #   Find suitable initial guess
            model._setInitialGuess(50, next_state, x_r, y_r, yaw_r, niu=niu)
            states = np.stack(next_state)

            ###     Plot path horizon
            states_horizon = model._getStatesFlat()
            control_horizon = model._getControlsFlat()
            
            start1 = time.time()
            horizon_path_plot, ref_path_plot = _plotData(ax_anim, fig_anim, states_horizon, next_state)
            end1 = time.time() - start1
            ###

        elif(i > 0):
            #   Transform regulation problem into tracking problem
            model._setReference(x_r, y_r, yaw_r, niu=niu)

            if( specs["solver"] == "SQP_RTI" ):
                status_p = model._preparation_sqp_rti()
                new_cost_p, new_opt_time_p = model._getData()

                res, status_f = model._feedback_sqp_rti(next_state)
                new_cost_f, new_opt_time_f = model._getData()

                ###     Plot path horizon
                states_horizon = model._getStatesFlat()
                control_horizon = model._getControlsFlat()
                
                start1 = time.time()
                _removePlotData( [horizon_path_plot, ref_path_plot] )
                horizon_path_plot, ref_path_plot = _plotData(ax_anim, fig_anim, states_horizon, next_state)
                end1 = time.time() - start1

                #input()

                #   Get updated state from the mobile robot
                next_state = model._simulate( np.stack( next_state ), np.stack( [ res[0], res[1] ] ), new_opt_time_p + new_opt_time_f )                
                progress += opt_v_speed * (new_opt_time_p + new_opt_time_f)

                if( robustness_test ):
                    next_state = model._addNoise2State(next_state, [std_noise_position, std_noise_position, std_noise_heading] )
                ###

                #   Get and save roll and pitch
                next_roll = roll_ref( next_state[0], next_state[1], next_state[4] ).elements()[0] + np.random.normal(0.0, std_noise_roll)
                next_pitch = pitch_ref( next_state[0], next_state[1], next_state[4] ).elements()[0] + np.random.normal(0.0, std_noise_pitch)

                next_state[2] = next_roll
                next_state[3] = next_pitch

                roll_list += [next_roll]
                pitch_list += [next_pitch]

                prep_status += [status_p]
                feedback_status += [status_f]

                if( i == 1 ):
                    cost_p = [new_cost_p]
                    time_p = [new_opt_time_p]
                    cost_f = [new_cost_f]
                    time_f = [new_opt_time_f]

                    states = np.vstack( ( states, np.stack(next_state) ) )
                    controls = np.stack( [ res[0], res[1] ] )

                    progress_list += [ progress ]
                    v_speed_list += [ opt_v_speed ]

                elif( i > 1 ):
                    cost_p += [new_cost_p]
                    time_p += [new_opt_time_p]
                    cost_f += [new_cost_f]
                    time_f += [new_opt_time_f]

                    states = np.vstack( ( states, np.stack(next_state) ) )
                    controls = np.vstack( ( controls, np.stack( [ res[0], res[1] ] ) ) )
            
            elif( specs["solver"] == "SQP" ):
                res, _ = model._solve_sqp(next_state)
                new_cost, new_opt_time = model._getData()

                ###     Plot path horizon
                states_horizon = model._getStatesFlat()
                control_horizon = model._getControlsFlat()
                
                _removePlotData( [horizon_path_plot, ref_path_plot] )
                horizon_path_plot, ref_path_plot = _plotData(ax_anim, fig_anim, states_horizon, next_state)

                #input()

                #   Get updated state from the mobile robot
                next_state = model._simulate( np.stack( next_state ), np.stack( [ res[0], res[1] ] ), new_opt_time )                
                progress += opt_v_speed * (new_opt_time)

                if( robustness_test ):
                    next_state = model._addNoise2State(next_state, [std_noise_position, std_noise_position, std_noise_heading] )
                ###

                if( i == 1 ):
                    cost_sqp = [new_cost]
                    time_tot = [new_opt_time]

                    states = np.vstack( ( states, np.stack(next_state) ) )
                    controls = np.stack( [ res[0], res[1] ] )

                    progress_list += [ progress ]
                    v_speed_list += [ opt_v_speed ]

                elif( i > 1 ):
                    cost_sqp += [new_cost]
                    time_tot += [new_opt_time]

                    states = np.vstack( ( states, np.stack(next_state) ) )
                    controls = np.vstack( ( controls, np.stack( [ res[0], res[1] ] ) ) )

        cycle_times += [time.time() - start - end1 ]

    plt.ioff()
    plt.show()

    fig, ax = plt.subplots(2, 6)

    ax[0, 0].plot( states[:, 0] )
    ax[0, 0].set_title('x')

    ax[0, 1].plot( states[:, 1] )
    ax[0, 1].set_title('y')

    ax[0, 2].plot( states[:, 2] )
    ax[0, 2].set_title('Roll')

    ax[0, 3].plot( states[:, 3] )
    ax[0, 3].set_title('Pitch')

    ax[0, 4].plot( states[:, 4] )
    ax[0, 4].set_title('Yaw')

    ax[0, 5].plot( states[:, 5] )
    ax[0, 5].set_title('vx')

    ax[1, 0].plot( states[:, 6] )
    ax[1, 0].set_title('wz')

    ax[1, 1].plot( states[:, 7] )
    ax[1, 1].set_title('fl')

    ax[1, 2].plot( states[:, 8] )
    ax[1, 2].set_title('fr')

    ax[1, 3].plot( controls[:, 0] )
    ax[1, 3].set_title('fl_rate')

    ax[1, 4].plot( controls[:, 1] )
    ax[1, 4].set_title('fr_rate')

    ax[1, 5].plot( states[:, 0], states[:, 1] )
    ax[1, 5].set_title('Position')

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
    
    fig3, ax3 = plt.subplots()

    nz = np.absolute( np.cos( np.array(roll_list) ) * np.cos( np.array(pitch_list) ) * vehicle_specs["m"] * common.gz / 4 )

    t_l = np.sqrt( np.power( vehicle_specs["m"] * common.gz * np.sin( np.array(pitch_list) ) / 4 + states[:, 7], 2 )\
            + np.power( -vehicle_specs["m"] * common.gz * np.cos( np.array(pitch_list) ) * np.sin( np.array(roll_list) ) / 4, 2 ) )

    t_r = np.sqrt( np.power( vehicle_specs["m"] * common.gz * np.sin( np.array(pitch_list) ) / 4 + states[:, 8], 2 )\
            + np.power( -vehicle_specs["m"] * common.gz * np.cos( np.array(pitch_list) ) * np.sin( np.array(roll_list) ) / 4, 2 ) )

    ax3.scatter( t_l, nz, s=4.0, c='c', marker='X' )
    ax3.scatter( t_r, nz , s=4.0, c='m', marker='X' )

    ax3.plot( np.arange(0, 40, 1e-1), np.arange(0, 40, 1e-1) * (1/niu) , 'r')

    plt.show()

    saveData = True

    if(saveData):
        #           
        index = -1

        if(robustness_test):
            figures_file = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/"\
                                + "Tese_RoverNavigation/Resultados/Real-time one stage OCP on rough terrain/Robustness tests/"

        else:
            figures_file = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/"\
                                + "Tese_RoverNavigation/Resultados/Real-time one stage OCP on rough terrain/Start away from reference/"

        files = os.listdir(figures_file)

        for name in files:
            if("solver_specs_" in name or "sim_specs_" in name):
                pass

            else:
                index += 1
        
        print("Number of files: ", index)
        #

        if(robustness_test):
            file2open = common.results_folder + "Real-time one stage OCP on rough terrain/Robustness tests/"\
                                        + str(index + 1) + ".json"
        
        else:
            file2open = common.results_folder + "Real-time one stage OCP on rough terrain/Start away from reference/"\
                                        + str(index + 1) + ".json"

        with open(file2open, "w") as f:
            
            data2save = { 'cost_p': cost_p,\
                          'cost_f': cost_f,\
                          'time_p': time_p,\
                          'time_f': time_f,\
                          'cycle_times': cycle_times,\
                          'x': list( states[:, 0] ),\
                          'y': list( states[:, 1] ),\
                          'roll': list( states[:, 2] ),\
                          'pitch': list( states[:, 3] ),\
                          'yaw': list( states[:, 4] ),\
                          'vx': list( states[:, 5] ),\
                          'wz': list( states[:, 6] ),\
                          'fl': list( states[:, 7] ),\
                          'fr': list( states[:, 8] ),\
                          'fl_rate': list( controls[:, 0] ),\
                          'fr_rate': list( controls[:, 1] ),\
                          'prep_status': prep_status,\
                          'feedback_status': feedback_status }
            json.dump(data2save, f, indent=4)

        if(robustness_test):
            file2open = common.results_folder + "Real-time one stage OCP on rough terrain/Robustness tests/"\
                        + "sim_specs_" + str(index + 1) + ".json"
        
        else:
            file2open = common.results_folder + "Real-time one stage OCP on rough terrain/Start away from reference/"\
                        + "sim_specs_" + str(index + 1) + ".json"
        
        with open(file2open, "w") as f:
            
            data2save = { 'initial_state': list(initial_state),\
                          'cost function type': cost_function_type,\
                          'weights_type': weights_type,\
                          'reference_gen': reference_gen,\
                          'niu': niu,\
                          'vehicle_mass': vehicle_specs["m"],\
                          'gravity': common.gz,\
                          'map_option': map_option,\
                          'qa': _qa,\
                          'qb': _qb,\
                          'qc': _qc,\
                          'std_noise_roll': std_noise_roll,\
                          'std_noise_pitch': std_noise_pitch,\
                          'std_noise_position': std_noise_position,\
                          'std_noise_heading': std_noise_heading,\
                          'range_sim': range_sim }
            json.dump(data2save, f, indent=4)

        if(robustness_test):
            file2open = common.results_folder + "Real-time one stage OCP on rough terrain/Robustness tests/"\
                        + "solver_specs_" + str(index + 1) + ".json"
        
        else:
            file2open = common.results_folder + "Real-time one stage OCP on rough terrain/Start away from reference/"\
                        + "solver_specs_" + str(index + 1) + ".json"

        with open(file2open, "w") as f:
            json.dump(solver_param, f, indent=4)