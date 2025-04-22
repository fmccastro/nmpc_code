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

    com2wheels = {'com2bl': [-0.13404387591240877, 0.197, -0.052573043795620514],\
                  'com2fl': [0.13495612408759125, 0.197, -0.052573043795620514],\
                  'com2br': [-0.13404387591240877, -0.197, -0.052573043795620514],\
                  'com2fr': [0.13495612408759125, -0.197, -0.052573043795620514] }
    
    com2contact = {'com2bl': [-0.13404387591240877, 0.197, -0.052573043795620514 - common.wheelRadius],\
                   'com2fl': [0.13495612408759125, 0.197, -0.052573043795620514 - common.wheelRadius],\
                   'com2br': [-0.13404387591240877, -0.197, -0.052573043795620514 - common.wheelRadius],\
                   'com2fr': [0.13495612408759125, -0.197, -0.052573043795620514 - common.wheelRadius] }

    #   Path tracking
    model = SimplifiedDynamics(com2wheels)

    range = 1000
    range_aux = 800

    #   Generate reference
    #   Generate reference
    x_ref = np.ones( (range) ) * 0   #np.linspace(0, 100, range)
    y_ref = np.ones( (range) ) * 1   #np.linspace(0, 100, range)
    yaw_ref = np.ones( (range) ) * math.pi / 2

    """x_ref = np.linspace(0, 100, range)
    y_ref = np.sin(x_ref)

    yaw_ref = []

    for _ in np.arange(range - 1):
        yaw_ref += [ math.atan2( y_ref[_ + 1] - y_ref[_], x_ref[_ + 1] - x_ref[_] ) ]
    
    yaw_ref += [yaw_ref[-1]]

    yaw_ref = np.array(yaw_ref)"""

    print(x_ref.shape, y_ref.shape, yaw_ref.shape)

    pose = np.stack( [0, 0, 0, 0, 0, 0] )
    velocity = np.stack( [0, 0, 0] )
    
    path = pose
    velList = velocity

    cost_p = []
    cost_f = []

    time_tot = []

    for i in np.arange(range_aux):

        print("Iteration: ", i)

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
            model._setInitialGuess(common.N + 1, pose, velocity, np.stack(ref))

        model._preparation_sqp_rti()

        _, _, cost_prep, optTime_prep = model._data()

        controls, status2 = model._feedback_sqp_rti(pose, velocity, np.stack(ref))

        if(status2 != 0):
            break

        solutionX, solutionU, cost_fed, optTime_fed = model._data()

        print("Optimization time: ", optTime_prep + optTime_fed)

        next_state = model._simulate( np.append(pose, velocity), controls )

        pose = next_state[0:6]
        velocity = next_state[6:9]

        path = np.append(path, pose)
        velList = np.append(velList, velocity)

        moment_bl = np.cross( np.array( com2contact['com2bl'] ), np.array( [ solutionU[0], solutionU[4], solutionU[8] ] ) )
        moment_fl = np.cross( np.array( com2contact['com2fl'] ), np.array( [ solutionU[1], solutionU[5], solutionU[9] ] ) )
        moment_br = np.cross( np.array( com2contact['com2br'] ), np.array( [ solutionU[2], solutionU[6], solutionU[10] ] ) )
        moment_fr = np.cross( np.array( com2contact['com2fr'] ), np.array( [ solutionU[3], solutionU[7], solutionU[11] ] ) )

        if( i == 0 ):
            forces = solutionU[0:6]
            #moments = moment_bl + moment_fl + moment_br + moment_fr
        
        elif( i > 0 ):
            forces = np.vstack( (forces, solutionU[0:6]) )
            #moments = np.vstack( ( moments, moment_bl + moment_fl + moment_br + moment_fr ) )
        
        cost_p += [cost_prep]
        cost_f += [cost_fed]
        time_tot += [optTime_prep + optTime_fed]

        """if( i == 0 ):
            plt.figure()
            plt.plot(np.arange(common.N + 1) * common.Ts, np.stack(ref)[0::3], 'b--')
            plt.plot(np.arange(common.N + 1) * common.Ts, solutionX[0::12], 'r')

            plt.figure()
            plt.plot(np.arange(common.N + 1) * common.Ts, np.stack(ref)[1::3], 'b--')
            plt.plot(np.arange(common.N + 1) * common.Ts, solutionX[1::12], 'r')

            plt.figure()
            plt.plot(np.arange(common.N + 1) * common.Ts, np.stack(ref)[2::3], 'b--')
            plt.plot(np.arange(common.N + 1) * common.Ts, solutionX[5::12], 'r')

            plt.figure()
            plt.plot( np.arange(common.N) * common.Ts, solutionU[0::12], 'r-' )
            plt.plot( np.arange(common.N) * common.Ts, solutionU[1::12], 'g-' )
            plt.plot( np.arange(common.N) * common.Ts, solutionU[2::12], 'r-' )
            plt.plot( np.arange(common.N) * common.Ts, solutionU[3::12], 'g-' )
            plt.title('fx')

            plt.figure()
            plt.plot( np.arange(common.N) * common.Ts, solutionU[4::12], 'r-' )
            plt.plot( np.arange(common.N) * common.Ts, solutionU[5::12], 'g-' )
            plt.plot( np.arange(common.N) * common.Ts, solutionU[6::12], 'r-' )
            plt.plot( np.arange(common.N) * common.Ts, solutionU[7::12], 'g-' )
            plt.title('fy')

            plt.figure()
            plt.plot( np.arange(common.N) * common.Ts, solutionU[8::12], 'r-' )
            plt.plot( np.arange(common.N) * common.Ts, solutionU[9::12], 'b-' )
            plt.plot( np.arange(common.N) * common.Ts, solutionU[10::12], 'k-' )
            plt.plot( np.arange(common.N) * common.Ts, solutionU[11::12], 'g-' )
            plt.title('fz')

            plt.show()"""

    #range_aux = i

    plt.figure()
    plt.plot( x_ref[:range_aux], y_ref[:range_aux], 'r--')
    plt.plot( path[0::6], path[1::6] )
    plt.title('position')

    plt.figure()
    plt.plot( x_ref[:range_aux], 'r--')
    plt.plot( np.arange(range_aux + 1), path[0::6] )
    plt.title('x')

    plt.figure()
    plt.plot( y_ref[:range_aux], 'r--')
    plt.plot( np.arange(range_aux + 1), path[1::6] )
    plt.title('y')

    plt.figure()
    plt.plot( np.arange(range_aux + 1), path[2::6] )
    plt.title('z')

    plt.figure()
    plt.plot( np.arange(range_aux + 1), path[3::6] )
    plt.title('roll')

    plt.figure()
    plt.plot( np.arange(range_aux + 1), path[4::6] )
    plt.title('pitch')

    plt.figure()
    plt.plot( yaw_ref[:range_aux], 'r--')
    plt.plot( np.arange(range_aux + 1), path[5::6] )
    plt.title('yaw')

    plt.figure()
    plt.plot( np.arange(range_aux + 1), velList[0::3] )
    plt.title('vx')

    plt.figure()
    plt.plot( np.arange(range_aux + 1), velList[1::3] )
    plt.title('vy')

    plt.figure()
    plt.plot( np.arange(range_aux + 1), velList[2::3] )
    plt.title('wz')

    fig, ax = plt.subplots()
    ax.plot( np.arange(range_aux), forces[:, 0], label = 'fx_l' )
    ax.plot( np.arange(range_aux), forces[:, 1], label = 'fx_r' )
    ax.legend()
    ax.set_title('fx')

    fig1, ax1 = plt.subplots()
    ax1.plot( np.arange(range_aux), forces[:, 2], label = 'fy_bl' )
    ax1.plot( np.arange(range_aux), forces[:, 3], label = 'fy_fl' )
    ax1.plot( np.arange(range_aux), forces[:, 4], label = 'fy_br' )
    ax1.plot( np.arange(range_aux), forces[:, 5], label = 'fy_fr' )
    ax1.legend()
    ax1.set_title('fy')

    """plt.figure()
    plt.plot(np.arange(range_aux), moments[:, 0])
    plt.title('mx')

    plt.figure()
    plt.plot(np.arange(range_aux), moments[:, 1])
    plt.title('my')

    plt.figure()
    plt.plot(np.arange(range_aux), moments[:, 2])
    plt.title('mz')"""

    ###
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