#!/usr/bin/python3
import os
from pathlib import Path

classes_path = Path(__file__)

upper_directory = classes_path.parent.parent.parent
lower_directory = upper_directory / "src"

from sys import path
path.append( str(lower_directory) )

from classes.standard_formulation import *
from classes.obstacle_avoidance import *
from classes.references import *
from classes.common_class import *

""" 
    Script to test real-time baseline standard formulation algorithm on rough terrain
""" 

custom_initial_pose = False

#   Script parameters
robustness_test = True  #   Noise amplitude to test controller reaction (robusteness test)

#cost_function_type = "Standard"
cost_function_type = "Low_effort"

weights_type = "variable_weights"
reference_gen = "new_reference"

range_sim = 20000   #   Number of simulation iterations

if(robustness_test):
    std_noise_position = 0
    std_noise_heading = 0

#   Coefficient of friction
niu = 0.7

std_noise_roll = 7e-2
std_noise_pitch = 7e-2

plot_anim = False

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

    with open(common.vehicle_specs) as f:
        vehicle_specs = json.load(f)
    
    with open(common.ciao_parameters) as f:
        ciao_parameters = json.load(f)

    with open(common.potential_field) as f:
        potential_field_parameters = json.load(f)

    #   Set up nmpc countoring model
    model = DynamicsRatesPathRoughTerrainObstacleAvoidance()
    ref = Reference(option = map_option, qa=_qa, qb=_qb, qc=_qc)
    obs_exact = ObstacleAvoidanceExact()
    pot = PotentialField()
    
    target_progress = potential_field_parameters["target_progress"]

    x_ref = ref.__dict__["x_ref"]
    y_ref = ref.__dict__["y_ref"]
    yaw_ref = ref.__dict__["yaw_ref"]
    curvature = ref.__dict__["curvature"]
    roll_ref = ref.__dict__["roll_ref"]
    pitch_ref = ref.__dict__["pitch_ref"]

    disc_steps = np.arange(0.0, 2 * ca.pi, 0.03)

    _x_ref = x_ref( disc_steps )
    _y_ref = y_ref( disc_steps )
    
    #   Set up animation figure
    if(plot_anim):
        plt.ion()
        fig_anim, ax_anim = plt.subplots()

        fig_anim.canvas.draw()
        background = fig_anim.canvas.copy_from_bbox(ax_anim.bbox)

        #   Plot obstacles
        obs_exact._plotObstacles(ax_anim, plotSafetyMargin=True)
        ax_anim.set(xlim=[-4.0, 4.0], ylim=[-4.0, 4.0], xlabel=rf"$x\,[m]$", ylabel=rf"$y\,[m]$")
        ax_anim.set_aspect('equal')
        ax_anim.legend()
        ###

    #   Initialize state: (x, y, roll, pitch, yaw, vx, wz, tl, tr)
    if( custom_initial_pose ):
        closest_point = ClosestPoint(x_ref, y_ref)

        initial_state = next_state = np.stack( [ 2.25, 2.45, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] )

        #   Find closest point
        closest_point_guess = closest_point._findInitialGuess(next_state[0], next_state[1])
        closest_point_progress = closest_point._solve(next_state[0], next_state[1], closest_point_guess)

        figobs, axobs = plt.subplots()
        axobs.scatter( next_state[0], next_state[1] )
        axobs.plot( x_ref( np.arange(0, 2 * ca.pi, 0.01) ), y_ref( np.arange(0, 2 * ca.pi, 0.01) ), 'k--' )
        axobs.scatter( float(x_ref( closest_point_guess ) ), float( y_ref( closest_point_guess ) ), c = 'r', marker='*', zorder = 2 )
        axobs.scatter( float(x_ref( closest_point_progress ) ), float( y_ref( closest_point_progress ) ), c = 'b', marker='*', zorder = 2 )
        obs_exact._plotObstacles(axobs, plotSafetyMargin=True)
        plt.show()

        input()

        progress_sol = closest_point_progress

    else:
        progress_sol = 0.65
        initial_state = np.stack( [ x_ref(progress_sol).elements()[0], y_ref(progress_sol).elements()[0], 0.0, 0.0, yaw_ref(progress_sol).elements()[0], 0.0, 0.0, 0.0, 0.0 ] )
    ###
    
    next_state = initial_state

    if(plot_anim):
        ax_anim.plot(_x_ref, _y_ref, 'k--', linewidth=0.6, zorder=4 )

    #   Check if initial position is admissable (on free space)
    min_distance, _ = obs_exact._computeMinimumDistance( next_state[0], next_state[1] )

    if( min_distance <= ciao_parameters["safety_margin"] ):
        print("MOBILE ROBOT CAN'T START ON DANGEROUS LOCATION !")
        exit(0)
    
    else:
        print("MOBILE ROBOT IS STARTING ON SAFE LOCATION !")
    ###

    cycle_times = []

    #   Update progress
    progress = progress_sol
    progress_list = [progress]
    v_speed_list = []

    roll_list = []
    pitch_list = []

    prep_status = []
    feedback_status = []

    #   Initialize path and reference horizon
    if(plot_anim):
        target_plot = ax_anim.scatter([], [], c='g', s=20.0, zorder=5)
        target_free_plot = ax_anim.scatter([], [], c='g', marker='x', s=20.0, zorder=5)
        horizon_path_plot = ax_anim.scatter([], [], c='r', s=4.0, zorder=3)
        ref_path_plot = ax_anim.scatter([], [], c='k', s=4.0, zorder=3)

    #   Initialize CIAO circles
    max_circles = specs["N"] + 1

    if(plot_anim):
        circles = []

        for _ in range(max_circles):
            c = plt.Circle((0, 0), 0.0, color='b', fill=False, visible=True, linewidth=0.3, zorder=4)
            ax_anim.add_patch(c)
            circles.append(c)
    
    else:
        circles = None
    ###
    
    for i in np.arange(range_sim):
        
        time2remove = 0
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

        if( i == 0 ):
            #   Get and save roll and pitch
            next_roll = roll_ref( next_state[0], next_state[1], next_state[4] ).elements()[0] + np.random.normal(0.0, std_noise_roll)
            next_pitch = pitch_ref( next_state[0], next_state[1], next_state[4] ).elements()[0] + np.random.normal(0.0, std_noise_pitch)

            next_state[2] = next_roll
            next_state[3] = next_pitch

            roll_list += [next_roll]
            pitch_list += [next_pitch]

            #   Set up increasing (or decreasing) weights along the control horizon
            if( weights_type == "variable_weights" ):
                model._setWeights( weights["var_weight_coefficient"] )

            #   Set CIAO circles for first iteration
            new_cx_horizon = [0.0] * max_circles
            new_cy_horizon = [0.0] * max_circles
            new_d2o_horizon = [1e2] * max_circles

            #   Initialize horizon lists to save data
            cx_horizon = []
            cy_horizon = []
            d2o_horizon = []

            x_r_horizon = []
            y_r_horizon = []
            yaw_r_horizon = []

            states_horizon = []
            control_horizon = []
            slack_horizon = []

            target_list = []
            target_proj_list = []
            ###

            #   Get free collision target
            target_x = x_ref(progress + target_progress).elements()[0]
            target_y = y_ref(progress + target_progress).elements()[0]

            min_dist, _ = obs_exact._computeMinimumDistance(target_x, target_y)

            if(min_dist <= ciao_parameters["safety_margin"]):
                target_proj = obs_exact._projectPoint2FreeSpace(target_x, target_y, ciao_parameters["safety_margin"])
                
                if(plot_anim):
                    start_aux=time.time()
                    target_free_plot.set_visible(True)
                    target_free_plot.set_offsets( np.c_[target_proj[0], target_proj[1]] )
                    time2remove += time.time() - start_aux
            
            else:
                target_proj = [target_x, target_y]

                if(plot_anim):
                    start_aux=time.time()
                    target_free_plot.set_visible(False)
                    time2remove += time.time() - start_aux

            #   Get reference from potential field
            x_r, y_r, yaw_r = pot._computeFreeCollisionReference(next_state[0], next_state[1], next_state[4], target_proj[0], target_proj[1], opt_v_speed*0.2, specs["N"])

            #   Find suitable initial guess
            model._setInitialGuess(50, next_state, x_r, y_r, yaw_r, new_cx_horizon, new_cy_horizon, new_d2o_horizon, ciao_parameters["safety_margin"], niu=niu)
            states = np.stack(next_state)

            ###     Plot path horizon
            new_states_horizon = model._getStatesFlat()
            new_control_horizon = model._getControlsFlat()
            new_slack_horizon = model._getSlacksFlat()

            new_x_horizon = new_states_horizon[::9]
            new_y_horizon = new_states_horizon[1::9]

            if(plot_anim):
                start_aux = time.time()
                target_plot.set_offsets( np.c_[target_x, target_y] )
                horizon_path_plot.set_offsets( np.c_[new_x_horizon, new_y_horizon] )
                ref_path_plot.set_offsets( np.c_[x_r, y_r] )
                time2remove += time.time() - start_aux

        elif(i > 0):
            #   Find suitable initial guess for convex free region centers
            cx, cy, d2o = obs_exact._computeCiaoHorizon(new_x_horizon, new_y_horizon, circles, plot_anim=plot_anim)

            cx_horizon += cx
            cy_horizon += cy
            d2o_horizon += d2o
            ###

            #   Get free collision target
            target_x = x_ref(progress + target_progress).elements()[0]
            target_y = y_ref(progress + target_progress).elements()[0]

            min_dist, _ = obs_exact._computeMinimumDistance(target_x, target_y)

            if( min_dist <= ciao_parameters["safety_margin"] ):
                target_proj = obs_exact._projectPoint2FreeSpace(target_x, target_y, ciao_parameters["safety_margin"])
                
                if(plot_anim):
                    start_aux = time.time()
                    target_free_plot.set_visible(True)
                    target_free_plot.set_offsets( np.c_[target_proj[0], target_proj[1]] )
                    time2remove += time.time() - start_aux
            
            else:
                target_proj = [target_x, target_y]

                if(plot_anim):
                    start_aux = time.time()
                    target_free_plot.set_visible(False)
                    time2remove += time.time() - start_aux
            ###

            target_list += [target_x, target_y]
            target_proj_list += target_proj 

            #   Get reference from potential field
            x_r, y_r, yaw_r = pot._computeFreeCollisionReference(next_state[0], next_state[1], next_state[4], target_proj[0], target_proj[1], opt_v_speed * 0.2, specs["N"])

            #   Set reference and ciao parameters into solver
            if( d2o == [0] * ( specs["N"] + 1 ) ):
                print("In dangerous space.")
                model._setReference(x_r, y_r, yaw_r, cx, cy, d2o, 0.0, niu = niu)

            else:                                                                                           
                model._setReference(x_r, y_r, yaw_r, cx, cy, d2o, ciao_parameters["safety_margin"], niu = niu)

            if( specs["solver"] == "SQP_RTI" ):
                status_p = model._preparation_sqp_rti()
                new_cost_p, new_opt_time_p = model._getData()

                res, status_f = model._feedback_sqp_rti(next_state)
                new_cost_f, new_opt_time_f = model._getData()

                ###     Plot path horizon
                new_states_horizon = model._getStatesFlat()
                new_control_horizon = model._getControlsFlat()
                new_slack_horizon = model._getSlacksFlat()

                states_horizon += list(new_states_horizon)
                control_horizon += list(new_control_horizon)
                slack_horizon += list(new_slack_horizon)

                new_x_horizon = new_states_horizon[::9]
                new_y_horizon = new_states_horizon[1::9]

                x_r_horizon += x_r
                y_r_horizon += y_r
                yaw_r_horizon += yaw_r
                
                if(plot_anim):
                    start_aux = time.time()
                    target_plot.set_offsets( np.c_[x_ref(progress + target_progress).elements()[0], y_ref(progress + target_progress).elements()[0]] )
                    horizon_path_plot.set_offsets( np.c_[new_x_horizon, new_y_horizon] )
                    ref_path_plot.set_offsets( np.c_[x_r, y_r] )
                    time2remove += time.time() - start_aux

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

            if(plot_anim):
                start_aux = time.time()
                fig_anim.canvas.restore_region(background)

                ax_anim.draw_artist(horizon_path_plot)
                ax_anim.draw_artist(ref_path_plot)

                for c in circles:
                    if c.get_visible():
                        ax_anim.draw_artist(c)

                fig_anim.canvas.blit(ax_anim.bbox)
                fig_anim.canvas.flush_events()
                time2remove += time.time() - start_aux
                time2remove = 0

        cycle_times += [time.time() - start - time2remove ]

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
        
        ax2[1].plot(time_p, 'r')
        ax2[1].plot(time_f, 'b')
        ax2[1].plot( specs["Ts"] * np.ones( len(time_p) ), 'r--')
        ax2[1].set_title('Opt. times')

        ax2[2].plot(cycle_times)
        ax2[2].plot( specs["Ts"] * np.ones( len(time_p) ), 'r--')
        ax2[2].set_title('Loop times')
    
    #   Plot results for fast analysis
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

        figures_file = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/Resultados/Real-time one stage OCP on rough terrain/Obstacle avoidance/"

        files = os.listdir(figures_file)

        for name in files:
            if("solver_specs_" in name or "parameters_" in name or "horizon_" in name):
                pass

            else:
                index += 1
        
        print("Number of files: ", index)
        #

        with open(common.results_folder + "Real-time one stage OCP on rough terrain/Obstacle avoidance/" + str(index + 1) + ".json", "w") as f:
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
                          'feedback_status': feedback_status,\
                          'target_list': target_list,\
                          'target_proj_list': target_proj_list }
            json.dump(data2save, f, indent=4)
        
        with open(common.results_folder + "Real-time one stage OCP on rough terrain/Obstacle avoidance/parameters_" + str(index + 1) + ".json", "w") as f:
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
        
        with open(common.results_folder + "Real-time one stage OCP on rough terrain/Obstacle avoidance/horizon_" + str(index + 1) + ".json", "w") as f:
            data2save = { 'states_horizon': states_horizon,\
                          'control_horizon': control_horizon,\
                          'slack_horizon': slack_horizon,\
                          'x_r_horizon': x_r_horizon,\
                          'y_r_horizon': y_r_horizon,\
                          'yaw_r_horizon': yaw_r_horizon,\
                          'cx_horizon': cx_horizon,\
                          'cy_horizon': cy_horizon,\
                          'd2o_horizon': d2o_horizon }
            json.dump(data2save, f, indent=4)

        with open(common.results_folder + "Real-time one stage OCP on rough terrain/Obstacle avoidance/solver_specs_" + str(index + 1) + ".json", "w") as f:
            json.dump(solver_param, f, indent=4)