#!/usr/bin/python3
from pathlib import Path

classes_path = Path(__file__)

upper_directory = classes_path.parent.parent
lower_directory = upper_directory / "src"

from sys import path
path.append( str(lower_directory) )

from classes.model_class_acados import *
from classes.common_class import *

if __name__ == '__main__':

    common = Common()

    #   Path tracking
    model = Dynamics()

    range = 10000
    range_aux = 8000

    #   Generate reference
    x_ref = np.ones( (range) ) * 0   #np.linspace(0, 100, range)
    y_ref = np.ones( (range) ) * 1   #np.linspace(0, 100, range)
    yaw_ref = np.ones( (range) ) * math.pi/2
    
    pose = np.stack([0, 0, 0, 0, 0, 0])
    path = pose

    cost_p = []
    cost_f = []

    time_tot = []

    vx = []
    wz = []

    for i in np.arange(range_aux):
        for _ in np.arange(common.N + 1):

            if(_ == 0):
                #yaw = math.atan2( y_ref[i + _ + 1] - y_ref[i + _], x_ref[i + _ + 1] - x_ref[i + _] )
                ref = [x_ref[i + _], y_ref[i + _], yaw_ref[i + _]]

            elif(_ > 0 and _ < common.N):
                #yaw = math.atan2( y_ref[i + _ + 1] - y_ref[i + _], x_ref[i + _ + 1] - x_ref[i + _] )
                ref = ref + [x_ref[i + _], y_ref[i + _], yaw_ref[i + _] ]
            
            else:
                ref = ref + [x_ref[i + _], y_ref[i + _], yaw_ref[i + _] ]

        if( i == 0 ):
            model._setInitialGuess(5, pose, np.stack(ref))

        model._preparation_sqp_rti()

        _, _, cost_prep, optTime_prep = model._data()

        next_vx, next_wz = model._feedback_sqp_rti(pose, np.stack(ref))

        _, _, cost_fed, optTime_fed = model._data()

        pose = model._simulate( pose, np.stack( [next_vx, next_wz] ) )
        path = np.append(path, pose)
        
        cost_p += [cost_prep]
        cost_f += [cost_fed]
        time_tot += [optTime_prep + optTime_fed]

        vx += [next_vx]
        wz += [next_wz]

        """
        if( i == 0 ):
            plt.figure()
            plt.plot(np.arange(common.N + 1) * common.Ts, np.stack(ref)[0::3], 'b')
            plt.plot(np.arange(common.N + 1) * common.Ts, solutionX[0::6], 'r')

            plt.figure()
            plt.plot(np.arange(common.N + 1) * common.Ts, np.stack(ref)[1::3], 'b')
            plt.plot(np.arange(common.N + 1) * common.Ts, solutionX[1::6], 'r')

            plt.figure()
            plt.plot(np.arange(common.N + 1) * common.Ts, np.stack(ref)[2::3], 'b')
            plt.plot(np.arange(common.N + 1) * common.Ts, solutionX[5::6], 'r')

            plt.figure()
            plt.plot( np.arange(common.N) * common.Ts, solutionU[0::2], 'r-' )
            plt.plot( np.arange(common.N) * common.Ts, solutionU[1::2], 'g-' )

            plt.show()
        """
    
    plt.figure()
    plt.plot( x_ref[:range_aux], 'r--')
    plt.plot( np.arange(range_aux + 1), path[0::6] )
    plt.title('x')

    plt.figure()
    plt.plot( y_ref[:range_aux], 'r--')
    plt.plot( np.arange(range_aux + 1), path[1::6] )
    plt.title('y')

    """plt.figure()
    plt.plot( np.arange(800 + 1), path[2::6] )
    plt.title('z')

    plt.figure()
    plt.plot( np.arange(800 + 1), path[3::6] )
    plt.title('roll')

    plt.figure()
    plt.plot( np.arange(800 + 1), path[4::6] )
    plt.title('pitch')"""

    plt.figure()
    plt.plot( yaw_ref[:range_aux], 'r--')
    plt.plot( np.arange(range_aux + 1), path[5::6] )
    plt.title('yaw')

    plt.figure()
    plt.plot(np.arange(range_aux), vx)
    plt.plot(np.arange(range_aux), np.ones(range_aux) * common.vx_lb, 'r--' )
    plt.plot(np.arange(range_aux), np.ones(range_aux) * common.vx_ub, 'r--' )
    plt.title('vx')

    plt.figure()
    plt.plot(np.arange(range_aux), wz)
    plt.plot(np.arange(range_aux), np.ones(range_aux) * common.wz_lb, 'r--' )
    plt.plot(np.arange(range_aux), np.ones(range_aux) * common.wz_ub, 'r--' )
    plt.title('wz')

    plt.figure()
    plt.plot(np.arange(range_aux), cost_p)
    plt.title('cost_prep')

    plt.figure()
    plt.plot(np.arange(range_aux), cost_f)
    plt.title('cost_fed')

    plt.figure()
    plt.plot(np.arange(range_aux), time_tot)
    plt.title('Optimization time')

    plt.show()