#!/usr/bin/python3
import os
from pathlib import Path

classes_path = Path(__file__)

upper_directory = classes_path.parent.parent.parent
lower_directory = upper_directory / "src"

from sys import path
path.append( str(lower_directory) )

from classes.standard_formulation import *
from classes.integrator import *

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
    
    with open(common.dynamics_integrator) as f:
        integrator_specs = json.load(f)

    #   Set up nmpc countoring model
    model_rk4 = DynamicsRatesPathRoughTerrainObstacleAvoidance()
    model_precise = SimpleDynamicsIntegratorCasadi()

    progress = 0.0
    virtual_speed = 0.1

    next_state_a = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]
    next_state_p = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]
    
    states_p = np.array( next_state_p )
    states_a = np.array( next_state_a )

    index = 0
    while(progress <= 4 * math.pi):

        tl_rate = 0.25 * math.sin(2 * progress)
        tr_rate = 0.25 * math.sin(2.01 * progress) 

        control = [tl_rate, tr_rate]

        if(index == 0):   
            controls = control

        elif(index > 0):
            controls = np.vstack( (controls, control) )

        res_p = model_precise._simulate(next_state_p, control)
        res_a = model_rk4._simulate( np.stack(next_state_a), np.stack(control), integrator_specs["Ts"] )

        #   Include terrain uneveness
        roll = np.random.normal(0, 7e-2)
        pitch = np.random.normal(0, 7e-2) 

        next_state_p = res_p['xf'].elements()
        next_state_a = res_a

        #   Include roll and pitch to simulate uneven terrain
        next_state_p[2] = roll
        next_state_a[2] = roll

        next_state_p[3] = pitch
        next_state_a[3] = pitch

        states_p = np.vstack( ( states_p, next_state_p ) )
        states_a = np.vstack( ( states_a, next_state_a ) )

        progress += integrator_specs["Ts"] * virtual_speed
        index += 1

    print("Index: ", index)

    plt.rcParams.update( {'font.family': 'DejaVu Sans',                                              
                      'font.size': 30,
                      'lines.linewidth': 4.0} )

    fig, ax = plt.subplots(layout='constrained')

    ax.plot( np.arange(index + 1) * integrator_specs["Ts"], states_p[:, 0], 'b', label='cvodes')
    ax.plot( np.arange(index + 1) * integrator_specs["Ts"], states_a[:, 0], 'r', label='rk4')
    ax.set_ylabel(rf"$x\,[m]$")
    ax.set_xlabel(rf"$t\,[s]$")

    ax.legend()

    fig, ax = plt.subplots(layout='constrained')

    ax.plot( np.arange(index + 1) * integrator_specs["Ts"], states_p[:, 1], 'b', label='cvodes')
    ax.plot( np.arange(index + 1) * integrator_specs["Ts"], states_a[:, 1], 'r', label='rk4')
    ax.set_ylabel(rf"$y\,[m]$")
    ax.set_xlabel(rf"$t\,[s]$")

    ax.legend()

    fig, ax = plt.subplots(layout='constrained')

    ax.plot( np.arange(index + 1) * integrator_specs["Ts"], states_p[:, 2], 'b', label='cvodes')
    ax.plot( np.arange(index + 1) * integrator_specs["Ts"], states_a[:, 2], 'r', label='rk4')
    ax.set_ylabel(rf"$\phi\,[rad]$")
    ax.set_xlabel(rf"$t\,[s]$")

    ax.legend()

    fig, ax = plt.subplots(layout='constrained')

    ax.plot( np.arange(index + 1) * integrator_specs["Ts"], states_p[:, 3], 'b', label='cvodes')
    ax.plot( np.arange(index + 1) * integrator_specs["Ts"], states_a[:, 3], 'r', label='rk4')
    ax.set_ylabel(rf"$\theta\,[rad]$")
    ax.set_xlabel(rf"$t\,[s]$")

    ax.legend()

    fig, ax = plt.subplots(layout='constrained')

    ax.plot( np.arange(index + 1) * integrator_specs["Ts"], states_p[:, 4], 'b', label='cvodes')
    ax.plot( np.arange(index + 1) * integrator_specs["Ts"], states_a[:, 4], 'r', label='rk4')
    ax.set_ylabel(rf"$\psi\,[rad]$")
    ax.set_xlabel(rf"$t\,[s]$")

    ax.legend()

    fig, ax = plt.subplots(layout='constrained')

    ax.plot( np.arange(index + 1) * integrator_specs["Ts"], states_p[:, 5], 'b', label='cvodes')
    ax.plot( np.arange(index + 1) * integrator_specs["Ts"], states_a[:, 5], 'r', label='rk4')
    ax.set_ylabel(rf"$v_x\,[m/s]$")
    ax.set_xlabel(rf"$t\,[s]$")

    ax.legend()

    fig, ax = plt.subplots(layout='constrained')

    ax.plot( np.arange(index + 1) * integrator_specs["Ts"], states_p[:, 6], 'b', label='cvodes')
    ax.plot( np.arange(index + 1) * integrator_specs["Ts"], states_a[:, 6], 'r', label='rk4')
    ax.set_ylabel(rf"$\omega_z\,[m/s]$")
    ax.set_xlabel(rf"$t\,[s]$")

    ax.legend()

    fig, ax = plt.subplots(layout='constrained')

    ax.plot( np.arange(index + 1) * integrator_specs["Ts"], states_p[:, 7], 'b', label='cvodes')
    ax.plot( np.arange(index + 1) * integrator_specs["Ts"], states_a[:, 7], 'r', label='rk4')
    ax.set_ylabel(rf"$t_l\,[N]$")
    ax.set_xlabel(rf"$t\,[s]$")

    ax.legend()

    fig, ax = plt.subplots(layout='constrained')

    ax.plot( np.arange(index + 1) * integrator_specs["Ts"], states_p[:, 8], 'b', label='cvodes')
    ax.plot( np.arange(index + 1) * integrator_specs["Ts"], states_a[:, 8], 'r', label='rk4')
    ax.set_ylabel(rf"$t_r\,[N]$")
    ax.set_xlabel(rf"$t\,[s]$")

    ax.legend()

    fig, ax = plt.subplots(layout='constrained')

    ax.plot( np.arange(index) * integrator_specs["Ts"], controls[:, 0], 'b')
    ax.plot( np.arange(index) * integrator_specs["Ts"], controls[:, 1], 'r')
    ax.set_ylabel(rf"$rates\,[N]$")
    ax.set_xlabel(rf"$t\,[s]$")

    plt.show()