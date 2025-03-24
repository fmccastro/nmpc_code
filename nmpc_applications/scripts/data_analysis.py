#!/usr/bin/python3
from __future__ import absolute_import
from __future__ import division

import matplotlib.pyplot as plt
import matplotlib.image as mpimg

import matplotlib, skfmm, math, os, pickle, json

import pandas as pd
import casadi as ca
import numpy as np

from matplotlib import cm
from scipy.optimize import minimize

from mpl_toolkits.mplot3d import Axes3D

from PIL import Image

import sys
sys.path.insert(0, "/media/fmccastro/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/src")

from classes.common_class import *
from classes.planner_class import *

"""
    get_stats() dict example

    {'iter_count': 16, 
     'iterations': { 'alpha_du': [0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.16954645020143344, 1.0, 0.07696962379010289, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0], 
                     'alpha_pr': [0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.8532486619262921, 0.01682002366394718, 0.4551993646742382, 0.17945167695010072, 0.7580326404663417, 0.511917097537291, 1.0, 0.9749815441905628, 0.9887569788370462, 1.0, 1.0], 
                     'd_norm': [0.0, 1.5628276102992147, 0.17855657021241084, 0.593886597301048, 1.8103536406686707, 0.6793875199743635, 2.044123078054813, 2.6284673082056518, 0.7987338499520924, 4.386734978602742, 0.4704455340816915, 0.4643972955469256, 0.1328547146928947, 0.014275815273835452, 0.007539683184914195, 0.00039843416416957147, 3.131958905213456e-05], 
                     'inf_du': [0.6041124002539116, 0.0017948195128445268, 0.17855557021241084, 0.19796217081607806, 0.2011503762344699, 0.20129997763405863, 0.20151926169479883, 0.19665140076059906, 0.35473150257194996, 0.37824378456624763, 0.03867264423772809, 0.08424535300895997, 0.0016832515604991204, 0.00036710955492536, 5.6628072480024993e-05, 1.6148745069538278e-08, 1.2175861797316416e-10],
                     'inf_pr': [0.0, 0.0002110186462589425, 1.4981688631898749e-05, 0.00017497715528147495, 0.0057972421954337605, 0.0006184372480064759, 0.00944421125841896, 0.00938366119444764, 0.011809500063022393, 0.045943186937088536, 0.020738808522505578, 0.013530602225932099, 0.0023937949141854906, 5.608092800456384e-05, 7.99462640948699e-06, 3.11909079163778e-08, 1.6290864776520055e-10], 
                     'mu': [0.1, 0.1, 0.1, 0.002828427124746191, 0.002828427124746191, 0.002828427124746191, 0.002828427124746191, 0.002828427124746191, 0.002828427124746191, 0.002828427124746191, 0.002828427124746191, 0.002828427124746191, 0.002828427124746191, 0.00015042412372345582, 1.8449144625279508e-06, 1.8449144625279508e-06, 2.5059035596800618e-09], 
                     'obj': [6.85219871597, 5.290057815768041, 5.112440297470794, 4.521107641146061, 2.73110501061905, 2.053473362655504, 0.33086273201785854, 0.31842167498957885, 0.26179810032526796, 0.23532268024118738, 0.24671421303588237, 0.21566766365184864, 0.20659981950744277, 0.1960442148619626, 0.1952487113393468, 0.19523785617940684, 0.19523048285007838], 
                     'regularization_size': [0.0, 0.0, 1.0, 0.3333333333333333, 0.1111111111111111, 0.2962962962962963, 0.09876543209876543, 0.03292181069958847, 0.08779149519890259, 0.029263831732967528, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] }, 
                     'n_call_callback_fun': 0, 
                     'n_call_nlp_f': 17, 
                     'n_call_nlp_g': 17, 
                     'n_call_nlp_grad': 0, 
                     'n_call_nlp_grad_f': 18, 
                     'n_call_nlp_hess_l': 16, 
                     'n_call_nlp_jac_g': 18, 
                     'n_call_total': 1, 
                     'return_status': 'Solve_Succeeded', 
                     'success': True, 
                     't_proc_callback_fun': 0.0, 
                     't_proc_nlp_f': 3.9000000000000006e-05, 
                     't_proc_nlp_g': 0.00011699999999999997, 
                     't_proc_nlp_grad': 0.0, 
                     't_proc_nlp_grad_f': 6.099999999999999e-05, 
                     't_proc_nlp_hess_l': 0.000259, 
                     't_proc_nlp_jac_g': 0.000166, 
                     't_proc_total': 0.01175, 
                     't_wall_callback_fun': 0.0, 
                     't_wall_nlp_f': 3.743299999999999e-05, 
                     't_wall_nlp_g': 0.00010029400000000001, 
                     't_wall_nlp_grad': 0.0, 
                     't_wall_nlp_grad_f': 5.391800000000001e-05, 
                     't_wall_nlp_hess_l': 0.000254227, 
                     't_wall_nlp_jac_g': 0.000166153, 
                     't_wall_total': 0.011907969, 
                     'unified_return_status': 'SOLVER_RET_UNKNOWN'}
    
    'stats': {'iter_count': 5,
              'n_call_BFGS': 0,
              'n_call_QP': 5,
              'n_call_callback_fun': 0,
              'n_call_convexify': 5,
              'n_call_linesearch': 5,
              'n_call_nlp_fg': 5,
              'n_call_nlp_grad': 0,
              'n_call_nlp_hess_l': 5,
              'n_call_nlp_jac_fg': 6,
              'n_call_total': 1,
              'return_status': 'Solve_Succeeded',
              'success': True,
              't_proc_BFGS': 0.0,
              't_proc_QP': 0.0013519999999999997,
              't_proc_callback_fun': 0.0,
              't_proc_convexify': 5e-05,
              't_proc_linesearch': 0.0005110000000000001,
              't_proc_nlp_fg': 0.00041999999999999996,
              't_proc_nlp_grad': 0.0,
              't_proc_nlp_hess_l': 0.00293,
              't_proc_nlp_jac_fg': 0.002097,
              't_proc_total': 0.008222,
              't_wall_BFGS': 0.0,
              't_wall_QP': 0.000337964,
              't_wall_callback_fun': 0.0,
              't_wall_convexify': 1.1941999999999998e-05,
              't_wall_linesearch': 0.0001283,
              't_wall_nlp_fg': 0.000105531,
              't_wall_nlp_grad': 0.0,
              't_wall_nlp_hess_l': 0.000744444,
              't_wall_nlp_jac_fg': 0.0006447810000000001,
              't_wall_total': 0.002221182,
              'unified_return_status': 'SOLVER_RET_SUCCESS'}
    
    {'stats': {'iter_count': 6,
               'iterations': {'alpha_du': [0.0, 1.0, 0.9164878632858737, 0.6624974747325839, 0.9725724553925621, 1.0, 1.0],
                              'alpha_pr': [0.0, 1.0, 0.8089017295568609, 0.6051644181335115, 1.0, 1.0, 1.0],
                              'd_norm': [0.0, 0.07235073987230663, 0.21013874227467744, 0.5891139818429199, 0.030740800212349855, 0.0026485799127983368, 3.643909487404899e-05],
                              'inf_du': [0.8430655601361953, 0.0019862630371922374, 0.009634472923747528, 0.024835926432967503, 0.0011497430003692966, 2.3608230536779575e-06, 8.091000475160433e-10],
                              'inf_pr': [0.0, 0.00022563176768392035, 0.0007950164559431261, 0.010105121675829625, 4.76393105811046e-05, 1.5362401793840921e-06, 1.8235984400446023e-10],
                              'mu': [0.1, 0.1, 0.002828427124746191, 0.00015042412372345582, 0.00015042412372345582, 1.8449144625279508e-06, 2.5059035596800618e-09],
                              'obj': [1.4216371078490448, 1.4911965260716973, 1.3276059786071768, 1.3222243908656293, 1.3117473684032073, 1.3112572753358662, 1.3112530650451086],
                              'regularization_size': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                            },
                'n_call_callback_fun': 0,
                'n_call_nlp_f': 7,
                'n_call_nlp_g': 7,
                'n_call_nlp_grad': 0,
                'n_call_nlp_grad_f': 8,
                'n_call_nlp_hess_l': 6,
                'n_call_nlp_jac_g': 8,
                'n_call_total': 1,
                'return_status': 'Solve_Succeeded',
                'success': True,
                't_proc_callback_fun': 0.0,
                't_proc_nlp_f': 8e-06,
                't_proc_nlp_g': 2.3999999999999997e-05,
                't_proc_nlp_grad': 0.0,
                't_proc_nlp_grad_f': 1.1999999999999999e-05,
                't_proc_nlp_hess_l': 4.5e-05,
                't_proc_nlp_jac_g': 3.2e-05,
                't_proc_total': 0.002188,
                't_wall_callback_fun': 0.0,
                't_wall_nlp_f': 6.1e-06,
                't_wall_nlp_g': 2.2526e-05,
                't_wall_nlp_grad': 0.0,
                't_wall_nlp_grad_f': 9.145999999999998e-06,
                't_wall_nlp_hess_l': 4.3695999999999996e-05,
                't_wall_nlp_jac_g': 3.1426e-05,
                't_wall_total': 0.002187782,
                'unified_return_status': 'SOLVER_RET_UNKNOWN'
            }, 
            'solution': {'f': DM(1.31125),
                         'g': DM([0.166616, 0.271391, -1.61701e-09, 0.258764, -6.53629e-09]),
                         'lam_g': DM([-1.55583e-08, -9.35275e-09, -0.315878, -9.69257e-09, -0.684122]),
                         'lam_p': DM([-0.385991, -0.0140375, -0.0260455]),
                         'lam_x': DM([-1.91581e-09, 4.20474e-10, 0.0583971]),
                         'x': DM([1.31125, 0.0806375, 0.785398])
                         },
            'inclination': 1.5707963267948966,
            'contacts': 2,
            'gradient': DM([[0.427878, 0.0812248]])
    }
"""

if __name__ == '__main__':

    common = Common()
    planner = Planner()

    """
        Option 1 simulation options
        
        option1:    HeightMin   ;   maxTerrainRoughness     ;   HeightMin_maxInclination    ;   WorstCaseMap    ;   RefinedMap
    """
    option1 = "RefinedMap"

    """
        Option 2 simulation options

        option2:    NO_InitialGuess ; With_InitialGuess_X0
    """
    option2 = "With_InitialGuess_X0"

    """
        solver: qrsqp   ;     sqpmethod   ; ipopt
    """
    solver = "ipopt"

    """
        option3:    Points  ;   Surface
    """
    option3 = "Points"

    """
        Parameters of map refinement

        alpha:  alpha
        radius:   radius
    """
    alpha = 0.9
    radius = 1.0

    if( option1 == "maxTerrainRoughness" ):
        with open( common.results_folder + "Traversability_maps/" + option1 + "+" + str(common.mapFolder) + '.json', 'rb') as handle:
            results = json.load(handle)

    elif( option1 == "WorstCaseMap" ):
        with open( common.results_folder + "Traversability_maps/" + option1 + "+" + option3 + "+" + str(common.mapFolder) + '.json', 'rb') as handle:
            results = json.load(handle)

    elif( option1 == "RefinedMap" ):
        with open( common.results_folder + "Traversability_maps/" + option1 + "+" + str(alpha) + "+" + str(radius) + "+" + option3 + "+" + str(common.mapFolder) + '.json', 'rb') as handle:
            results = json.load(handle)
    
    else:
        with open( common.results_folder + "Traversability_maps/" + common.optSolver + "+" + option1 + "+" + option2 + "+" + option3 + "+" + str(common.mapFolder) + '.json', 'rb') as handle:
            results = json.load(handle)

    n_success = 0
    n_contacts = 0

    #for key in results.keys():
        #print(results[key])
        #input()

    if( option1 == "HeightMin" ):
        total = 0

        time = []
        iters = []
        obj = []
        lambda_x_norm = []
        lambda_x_norm_contact = []
        inclination = []
        roll = []
        pitch = []
        distances2Ground = []
        mapGrad = []
        inclinationContact = []

        for index in np.arange( results['nb_iter'] ):

            z = results[str(index)]["solution"]['x'][0]
            r = results[str(index)]["solution"]['x'][1]
            p = results[str(index)]["solution"]['x'][2]

            if( not results[str(index)]["stats"]["success"] ):
                pass
                
            else:
                time += [ results[str(index)]["stats"]["t_wall_total"] ]
                iters += [ results[str(index)]["stats"]["iter_count"] ]
                obj += [ results[str(index)]["solution"]['f'][0] ]
                lambda_x_norm += [ np.linalg.norm( np.array( results[str(index)]["solution"]["lam_x"] ) ) ]
                inclination += [ results[str(index)]["inclination"] ]
                roll += [ results[str(index)]["solution"]['x'][1] ]
                pitch += [ results[str(index)]["solution"]['x'][2] ]
                distances2Ground += [ np.linalg.norm( results[str(index)]["solution"]["g"][1:] ) ]

                if( results[str(index)]["contacts"] >= 3 ):
                    grad = results[str(index)]["gradient"]

                    lambda_x_norm_contact += [ np.linalg.norm( np.array( results[str(index)]["solution"]["lam_x"] ) ) ]
                    mapGrad += [ np.linalg.norm( np.array( [ grad[0], grad[1] ] ) ) ]
                    inclinationContact += [ results[str(index)]["inclination"] ]
                    n_contacts += 1

                n_success += 1

            total += 1
        
        print("Rejected optimizations: ", n_success, total - n_success)
        print("Number of contacts: ", n_contacts)

    elif( option1 == "HeightMin_maxInclination" ):
        
        total = 0

        time = []
        iters = []
        obj = []
        lambda_x_norm = []
        lambda_x_norm_contact = []
        inclination = []
        roll = []
        pitch = []
        distances2Ground = []
        mapGrad = []
        inclinationContact = []

        for index in np.arange( results['nb_iter'] ):
            timeCell = 0
            iterCell = 0
            objCell = []
            lamxCell = []
            inclinationCell = []
            rollCell = []
            pitchCell = []
            distances2GroundCell = []
            sucessCell = []
            contactsCell = []

            for element in results[str(index)].keys():

                if(element != 'gradient' and results[str(index)][element]["stats"]["success"] == True ):
                    timeCell += results[str(index)][element]["stats"]["t_wall_total"]
                    iterCell += results[str(index)][element]["stats"]["iter_count"]
                    objCell += [ results[str(index)][element]["solution"]["f"][0] ]
                    lamxCell += [ np.linalg.norm( np.array( results[str(index)][element]["solution"]["lam_x"] ) ) ]
                    inclinationCell += [ results[str(index)][element]["inclination"] ]
                    rollCell += [ results[str(index)][element]["solution"]['x'][1] ]
                    pitchCell += [ results[str(index)][element]["solution"]['x'][2] ]

                    if(option3 == "Points"):
                        distances2GroundCell += [ np.linalg.norm( results[str(index)][element]["solution"]["g"][1:] ) ]
                    
                    elif(option3 == "Surface"):

                        distances = [ min( results[str(index)][element]["solution"]["g"][1: 5 + 1] ),\
                                      min( results[str(index)][element]["solution"]["g"][ 5 + 1 : 5 * 2 + 1 ] ),\
                                      min( results[str(index)][element]["solution"]["g"][ 5 * 2 + 1 : 5 * 3 + 1 ] ),\
                                      min( results[str(index)][element]["solution"]["g"][ 5 * 3 + 1 :] ) ]

                        distances2GroundCell += [ np.linalg.norm( distances ) ]
                    
                    contactsCell += [ results[str(index)][element]["contacts"] ]
                    sucessCell += [ results[str(index)][element]["stats"]["success"] ]

            if( any(item == True for item in sucessCell) ):
                n_success += 1

                time += [ timeCell ]
                iters += [ iterCell ]
                inclination += [ max(inclinationCell) ]

                maxIndex = inclinationCell.index( max(inclinationCell) )

                obj += [ objCell[maxIndex] ]
                lambda_x_norm += [ lamxCell[maxIndex] ]
                
                roll += [ rollCell[maxIndex] ]
                pitch += [ pitchCell[maxIndex] ]
                distances2Ground += [ distances2GroundCell[maxIndex] ]

                if( contactsCell[maxIndex] >= 3 ):
                    grad = results[str(index)]["gradient"]

                    lambda_x_norm_contact += [ lamxCell[maxIndex] ]
                    mapGrad += [ np.linalg.norm( np.array( [ grad[0], grad[1] ] ) ) ]
                    inclinationContact += [ max(inclinationCell) ]
                    n_contacts += 1

            total += 1
        
        print("Rejected optimizations: ", n_success, total - n_success)
        print("Number of contacts: ", n_contacts)
         
    elif( option1 == "maxTerrainRoughness" ):
        total = 0

        time = []
        mapGrad = []
        roughness = []

        for index in np.arange( results['nb_iter'] ):
            roughness += [ results[ str(index) ]["roughness"] ]
            time += [ results[ str(index) ]["time"] ]

            grad = results[str(index)]["gradient"]
            mapGrad += [ np.linalg.norm( np.array( [ grad[0], grad[1] ] ) ) ]

            total += 1
    
    elif( option1 == "WorstCaseMap" ):
        total = 0

        time = []
        mapGrad = []
        cost = []

        for index in np.arange( results['nb_iter'] ):
            cost += [ results[ str(index) ]["cost"] ]
            time += [ results[ str(index) ]["time"] ]
            
            grad = results[str(index)]["gradient"]
            mapGrad += [ np.linalg.norm( np.array( [ grad[0], grad[1] ] ) ) ]

            total += 1

    elif( option1 == "RefinedMap" ):
        total = 0

        mapGrad = []
        cost = []

        for index in np.arange( results['nb_iter'] ):
            cost += [ results[ str(index) ]["cost"] ]
            
            grad = results[str(index)]["gradient"]
            mapGrad += [ np.linalg.norm( np.array( [ grad[0], grad[1] ] ) ) ]

            total += 1
    
    if( option1 == "HeightMin" or option1 == "HeightMin_maxInclination" ):
        
        fig, ax1 = plt.subplots()

        color = 'tab:red'
        ax1.set_xlabel('index')
        ax1.set_ylabel('Nb iters', color=color)
        ax1.plot( np.arange( n_success ), iters, color=color )
        ax1.tick_params(axis='y', labelcolor=color)

        ax2 = ax1.twinx()  # instantiate a second Axes that shares the same x-axis

        color = 'tab:blue'
        ax2.set_ylabel('t_wall(ms)', color=color)  # we already handled the x-label with ax1
        ax2.plot( np.arange( n_success ), np.array(time) * 1e3, color=color )
        ax2.tick_params(axis='y', labelcolor=color)

        fig.tight_layout()  # otherwise the right y-label is slightly clipped

        ###
        fig2, ax = plt.subplots()

        color = 'tab:blue'
        ax.set_xlabel('index')
        ax.set_ylabel('Lambda x norm', color=color)
        ax.plot( np.arange( n_success ), lambda_x_norm, color=color)
        ax.tick_params(axis='y', labelcolor=color)

        ###
        figs, axs = plt.subplots(nrows=1, ncols=4, figsize=(9, 4))

        # plot violin plot
        axs[0].violinplot(np.array(time), showmeans=True, showmedians=True)
        axs[0].set_title('Time [s]')

        axs[1].violinplot(np.array(iters), showmeans=True, showmedians=True)
        axs[1].set_title('Iterations')

        axs[2].violinplot(np.array(obj), showmeans=True, showmedians=True)
        axs[2].set_title('Objective')

        axs[3].violinplot(np.array(lambda_x_norm), showmeans=True, showmedians=True)
        axs[3].set_title('Lambda x norm')

        ###
        fig3, ax3 = plt.subplots()

        color = 'tab:blue'
        ax3.set_xlabel('iters')
        ax3.set_ylabel('Time', color=color)
        ax3.scatter(np.array(iters), np.array(time), c='blue', s=0.5)
        ax3.tick_params(axis='y', labelcolor=color)

        planner._confidence_ellipse(np.array(iters), np.array(time), ax3, n_std = 1.0, label=r'$1\sigma$', edgecolor='firebrick')
        planner._confidence_ellipse(np.array(iters), np.array(time), ax3, n_std = 2.0, label=r'$2\sigma$', edgecolor='fuchsia', linestyle='--')
        planner._confidence_ellipse(np.array(iters), np.array(time), ax3, n_std = 3.0, label=r'$3\sigma$', edgecolor='blue', linestyle=':')
        
        ax3.scatter(np.mean( np.array(iters) ), np.mean( np.array(time) ), c = 'red', s = 3.0)
        ax3.set_title("Correlation R = " + str( np.corrcoef( np.array(iters), np.array(time) )[0, 1] ) )

        ###
        fig4, ax4 = plt.subplots()

        color = 'tab:blue'
        ax4.set_xlabel('Inclination')
        ax4.set_ylabel('Lambda x norm', color=color)
        ax4.scatter(np.array(inclinationContact), np.array(lambda_x_norm_contact), c='blue', s=0.5)
        ax4.tick_params(axis='y', labelcolor=color)

        planner._confidence_ellipse(np.array(inclinationContact), np.array(lambda_x_norm_contact), ax4, n_std = 1.0, label=r'$1\sigma$', edgecolor='firebrick')
        planner._confidence_ellipse(np.array(inclinationContact), np.array(lambda_x_norm_contact), ax4, n_std = 2.0, label=r'$2\sigma$', edgecolor='fuchsia', linestyle='--')
        planner._confidence_ellipse(np.array(inclinationContact), np.array(lambda_x_norm_contact), ax4, n_std = 3.0, label=r'$3\sigma$', edgecolor='blue', linestyle=':')
        
        ax4.scatter(np.mean( np.array(inclinationContact) ), np.mean( np.array(lambda_x_norm_contact) ), c = 'red', s = 3.0)
        ax4.set_title("Correlation R = " + str( np.corrcoef( np.array(inclinationContact), np.array(lambda_x_norm_contact) )[0, 1] ) )

        ###
        fig5, ax5 = plt.subplots()

        color = 'tab:blue'
        ax5.set_xlabel('Inclination')
        ax5.set_ylabel('Time', color=color)
        ax5.scatter(np.array(inclination), np.array(time), c='blue', s=0.5)
        ax5.tick_params(axis='y', labelcolor=color)

        planner._confidence_ellipse(np.array(inclination), np.array(time), ax5, n_std = 1.0, label=r'$1\sigma$', edgecolor='firebrick')
        planner._confidence_ellipse(np.array(inclination), np.array(time), ax5, n_std = 2.0, label=r'$2\sigma$', edgecolor='fuchsia', linestyle='--')
        planner._confidence_ellipse(np.array(inclination), np.array(time), ax5, n_std = 3.0, label=r'$3\sigma$', edgecolor='blue', linestyle=':')
        
        ax5.scatter(np.mean( np.array(inclination) ), np.mean( np.array(time) ), c = 'red', s = 3.0)
        ax5.set_title("Correlation R = " + str( np.corrcoef( np.array(inclination), np.array(time) )[0, 1] ) )
        
        ###
        """fig6, ax6 = plt.subplots()

        color = 'tab:blue'
        ax6.set_xlabel('Roll')
        ax6.set_ylabel('Pitch', color=color)
        ax6.scatter(np.array(roll), np.array(pitch), c='blue', s=0.5)
        ax6.tick_params(axis='y', labelcolor=color)

        planner._confidence_ellipse(np.array(roll), np.array(pitch), ax6, n_std = 1.0, label=r'$1\sigma$', edgecolor='firebrick')
        planner._confidence_ellipse(np.array(roll), np.array(pitch), ax6, n_std = 2.0, label=r'$2\sigma$', edgecolor='fuchsia', linestyle='--')
        planner._confidence_ellipse(np.array(roll), np.array(pitch), ax6, n_std = 3.0, label=r'$3\sigma$', edgecolor='blue', linestyle=':')
        
        ax6.scatter(np.mean( np.array(roll) ), np.mean( np.array(pitch) ), c = 'red', s = 3.0)
        ax6.set_title("Correlation R = " + str( np.corrcoef( np.array(roll), np.array(pitch) )[0, 1] ) )"""
        
        ###
        """fig7, ax7 = plt.subplots()

        color = 'tab:blue'
        ax7.set_xlabel('Distances to ground')
        ax7.set_ylabel('KKT norm', color=color)
        ax7.scatter(np.array(distances2Ground), np.array(lambda_x_norm), c='blue', s=0.5)
        ax7.tick_params(axis='y', labelcolor=color)

        planner._confidence_ellipse(np.array(distances2Ground), np.array(lambda_x_norm), ax7, n_std = 1.0, label=r'$1\sigma$', edgecolor='firebrick')
        planner._confidence_ellipse(np.array(distances2Ground), np.array(lambda_x_norm), ax7, n_std = 2.0, label=r'$2\sigma$', edgecolor='fuchsia', linestyle='--')
        planner._confidence_ellipse(np.array(distances2Ground), np.array(lambda_x_norm), ax7, n_std = 3.0, label=r'$3\sigma$', edgecolor='blue', linestyle=':')
        
        ax7.scatter(np.mean( np.array(distances2Ground) ), np.mean( np.array(lambda_x_norm) ), c = 'red', s = 3.0)
        ax7.set_title("Correlation R = " + str( np.corrcoef( np.array(distances2Ground), np.array(lambda_x_norm) )[0, 1] ) )"""
        
        ###
        """fig8, ax8 = plt.subplots()

        color = 'tab:blue'
        ax8.set_xlabel('Distances to ground')
        ax8.set_ylabel('Inclination', color=color)
        ax8.scatter(np.array(distances2Ground), np.array(inclination), c='blue', s=0.5)
        ax8.tick_params(axis='y', labelcolor=color)

        planner._confidence_ellipse(np.array(distances2Ground), np.array(inclination), ax8, n_std = 1.0, label=r'$1\sigma$', edgecolor='firebrick')
        planner._confidence_ellipse(np.array(distances2Ground), np.array(inclination), ax8, n_std = 2.0, label=r'$2\sigma$', edgecolor='fuchsia', linestyle='--')
        planner._confidence_ellipse(np.array(distances2Ground), np.array(inclination), ax8, n_std = 3.0, label=r'$3\sigma$', edgecolor='blue', linestyle=':')
        
        ax8.scatter(np.mean( np.array(distances2Ground) ), np.mean( np.array(inclination) ), c = 'red', s = 3.0)
        ax8.set_title("Correlation R = " + str( np.corrcoef( np.array(distances2Ground), np.array(inclination) )[0, 1] ) )"""

        ###
        fig9, ax9 = plt.subplots()

        color = 'tab:blue'
        ax9.set_xlabel('Inclination')
        ax9.set_ylabel('Elevation map gradient', color=color)
        ax9.scatter(np.array(inclinationContact), np.array(mapGrad), c='blue', s=0.5)
        ax9.tick_params(axis='y', labelcolor=color)

        planner._confidence_ellipse(np.array(inclinationContact), np.array(mapGrad), ax9, n_std = 1.0, label=r'$1\sigma$', edgecolor='firebrick')
        planner._confidence_ellipse(np.array(inclinationContact), np.array(mapGrad), ax9, n_std = 2.0, label=r'$2\sigma$', edgecolor='fuchsia', linestyle='--')
        planner._confidence_ellipse(np.array(inclinationContact), np.array(mapGrad), ax9, n_std = 3.0, label=r'$3\sigma$', edgecolor='blue', linestyle=':')
        
        ax9.scatter(np.mean( np.array(inclinationContact) ), np.mean( np.array(mapGrad) ), c = 'red', s = 3.0)
        ax9.set_title("Correlation R = " + str( np.corrcoef( np.array(inclinationContact), np.array(mapGrad) )[0, 1] ) )
        
        ###
        _folder = common.mapFolderDir + "/" + str( common.mapFolder )
        costMapFile = _folder + "/" + option1 + "+" + option3 + "+" + str( common.mapFolder ) + ".npy"

        costMap = np.load(costMapFile)

        plt.figure()
        plt.imshow(costMap)

        print()
        print("Time (std, mean): ", np.std(np.array(time)), np.mean(np.array(time)) )
        print("Iterations (std, mean): ", np.std(np.array(iters)), np.mean(np.array(iters)) )
        print("Objective (std, mean): ", np.std(np.array(obj)), np.mean(np.array(obj)) )
        print("lambda x (std, mean): ", np.std(np.array(lambda_x_norm)), np.mean(np.array(lambda_x_norm)) )

        plt.show()
    
    elif( option1 == "maxTerrainRoughness" ):
        _folder = common.mapFolderDir + "/" + str( common.mapFolder )
        costMapFile = _folder + "/" + option1 + "+" + str( common.mapFolder ) + ".npy"

        costMap = np.load(costMapFile)

        plt.figure()
        plt.imshow(costMap)

        fig9, ax9 = plt.subplots()

        color = 'tab:blue'
        ax9.set_xlabel('Inclination')
        ax9.set_ylabel('Elevation map gradient', color=color)
        ax9.scatter(np.array(roughness), np.array(mapGrad), c='blue', s=0.5)
        ax9.tick_params(axis='y', labelcolor=color)

        planner._confidence_ellipse(np.array(roughness), np.array(mapGrad), ax9, n_std = 1.0, label=r'$1\sigma$', edgecolor='firebrick')
        planner._confidence_ellipse(np.array(roughness), np.array(mapGrad), ax9, n_std = 2.0, label=r'$2\sigma$', edgecolor='fuchsia', linestyle='--')
        planner._confidence_ellipse(np.array(roughness), np.array(mapGrad), ax9, n_std = 3.0, label=r'$3\sigma$', edgecolor='blue', linestyle=':')
        
        ax9.scatter(np.mean( np.array(roughness) ), np.mean( np.array(mapGrad) ), c = 'red', s = 3.0)
        ax9.set_title("Correlation R = " + str( np.corrcoef( np.array(roughness), np.array(mapGrad) )[0, 1] ) )

        print("Time (std, mean): ", np.std(np.array(time)), np.mean(np.array(time)) )

        plt.show()

    elif( option1 == "WorstCaseMap" ):
        _folder = common.mapFolderDir + "/" + str( common.mapFolder )
        costMapFile = _folder + "/" + "worstCase" + "+" + option3 + "+" + str( common.mapFolder ) + ".npy"

        costMap = np.load(costMapFile)

        plt.figure()
        plt.imshow(costMap)

        fig9, ax9 = plt.subplots()

        color = 'tab:blue'
        ax9.set_xlabel('Inclination')
        ax9.set_ylabel('Elevation map gradient', color=color)
        ax9.scatter(np.array(cost), np.array(mapGrad), c='blue', s=0.5)
        ax9.tick_params(axis='y', labelcolor=color)

        planner._confidence_ellipse(np.array(cost), np.array(mapGrad), ax9, n_std = 1.0, label=r'$1\sigma$', edgecolor='firebrick')
        planner._confidence_ellipse(np.array(cost), np.array(mapGrad), ax9, n_std = 2.0, label=r'$2\sigma$', edgecolor='fuchsia', linestyle='--')
        planner._confidence_ellipse(np.array(cost), np.array(mapGrad), ax9, n_std = 3.0, label=r'$3\sigma$', edgecolor='blue', linestyle=':')
        
        ax9.scatter(np.mean( np.array(cost) ), np.mean( np.array(mapGrad) ), c = 'red', s = 3.0)
        ax9.set_title("Correlation R = " + str( np.corrcoef( np.array(cost), np.array(mapGrad) )[0, 1] ) )

        print("Time (std, mean): ", np.std(np.array(time)), np.mean(np.array(time)) )

        plt.show()

    elif( option1 == "RefinedMap" ):
        _folder = common.mapFolderDir + "/" + str( common.mapFolder )
        costMapFile = _folder + "/" + "mapRefinement" + "+" + option3 + "+" + str(alpha) + "+" + str(radius) + ".npy"
        costMap = np.load(costMapFile)

        plt.figure()
        plt.imshow(costMap)

        fig9, ax9 = plt.subplots()

        color = 'tab:blue'
        ax9.set_xlabel('Inclination')
        ax9.set_ylabel('Elevation map gradient', color=color)
        ax9.scatter(np.array(cost), np.array(mapGrad), c='blue', s=0.5)
        ax9.tick_params(axis='y', labelcolor=color)

        planner._confidence_ellipse(np.array(cost), np.array(mapGrad), ax9, n_std = 1.0, label=r'$1\sigma$', edgecolor='firebrick')
        planner._confidence_ellipse(np.array(cost), np.array(mapGrad), ax9, n_std = 2.0, label=r'$2\sigma$', edgecolor='fuchsia', linestyle='--')
        planner._confidence_ellipse(np.array(cost), np.array(mapGrad), ax9, n_std = 3.0, label=r'$3\sigma$', edgecolor='blue', linestyle=':')
        
        ax9.scatter(np.mean( np.array(cost) ), np.mean( np.array(mapGrad) ), c = 'red', s = 3.0)
        ax9.set_title("Correlation R = " + str( np.corrcoef( np.array(cost), np.array(mapGrad) )[0, 1] ) )

        plt.show()