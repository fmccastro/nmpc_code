#!/usr/bin/python3
from pathlib import Path

classes_path = Path(__file__)

upper_directory = classes_path.parent.parent.parent
lower_directory = upper_directory / "src"

from sys import path
import matplotlib.colors as mcolors
path.append( str(lower_directory) )

from classes.obstacle_avoidance import *
from classes.references import *
from classes.common_class import *

"""
    Script to test elastic-bands algorithm
"""

if __name__ == '__main__':

    common = Common()

    with open(common.ciao_parameters) as f:
        specs = json.load(f)

    plt.rcParams.update( {'font.family': 'Arial',                                              
                      'font.size': 30,
                      'lines.linewidth': 4.0} )

    ref = Reference(option = 1, qa=1.0, qb=1.0, qc=0.4)
    obs = ObstacleAvoidance(6, 6, 2e-2, 2e-2)

    #   Generate reference
    x_ref = ref.__dict__["x_ref"]
    y_ref = ref.__dict__["y_ref"]
    yaw_ref = ref.__dict__["yaw_ref"]
    curvature = ref.__dict__["curvature"]

    x = obs.__dict__["x"]
    y = obs.__dict__["y"]

    sdf_sym = obs._get_sdf()

    #   Check sdf
    sdf_check = np.ones( ( len(x), len(y) ) )

    i = 0

    for _y in -y:
        j = 0

        for _x in x:
            sdf_check[i, j] = sdf_sym( [_y, _x] )

            j += 1
        i += 1
    
    norm = TwoSlopeNorm(vmin=sdf_check.min(), vcenter=0, vmax=sdf_check.max())
    X, Y = np.meshgrid(x, y, indexing='xy')

    #   Discretize reference
    disc_steps = np.arange(0.0, 2 * ca.pi, 0.03)

    _x_ref = x_ref( disc_steps )
    _y_ref = y_ref( disc_steps )

    X, Y = np.meshgrid(x, y, indexing='xy')

    fig, ax = plt.subplots( layout='constrained' )
    ax.plot(_x_ref, _y_ref, 'k--', linewidth=0.6, zorder=2 )
    ax.set_aspect('equal', adjustable='box')
    cf = ax.contourf(X, Y, sdf_check, levels=1000, cmap="brown_cyan", norm=norm, zorder=1)
    ax.set_xlabel(r'$x$ [m]')
    ax.set_ylabel(r'$y$ [m]')
    fig.colorbar(cf, ax=ax, label=rf"Distance to closest obstacle [m]")

    plt.show()
    ###
    
    #   Compute sdf gradient
    sdf_grad_x, sdf_grad_y = obs._get_sdf_jacobian(sdf_sym)
    
    #   Check sdf gradient
    """U = np.zeros( ( X.shape[0], X.shape[1] ) )
    V = np.zeros( ( Y.shape[0], Y.shape[1] ) )

    i = 0

    for _y in -y:
        j = 0

        for _x in x:
            check_sdf_jac_y, check_sdf_jac_x = sdf_grad_y( _y, _x ), sdf_grad_x( _y, _x )

            sdf_jac_norm = math.sqrt( check_sdf_jac_x**2 + check_sdf_jac_y**2 )

            U[i, j] = check_sdf_jac_x / sdf_jac_norm
            V[i, j] = check_sdf_jac_y / sdf_jac_norm

            j += 1
        i += 1

    step = 15

    fig, ax = plt.subplots(layout='constrained')
    ax.quiver(X[::step, ::step], Y[::step, ::step], U[::step, ::step] * 0.22, -V[::step, ::step] * 0.22, color='k', angles='xy', scale_units='xy', zorder=4)
    ax.set_aspect('equal', adjustable='box')
    cf = ax.contourf(X, Y, sdf_check, levels=1000, cmap="brown_cyan", norm=norm, zorder=1)
    ax.set_xlabel(rf"$x$ [m]")
    ax.set_ylabel(rf"$y$ [m]")
    fig.colorbar(cf, ax = ax, label=rf"Distance to closest obstacle [m]")
    plt.show()"""
    ###

    #   Second sdf gradient check
    step=15

    quiver_x = []
    quiver_y = []
    arrow_x = []
    arrow_y = []

    for _y in y[::step]:
        for _x in x[::step]:
            _norm = ( sdf_grad_x( _y, _x ).elements()[0]**2 + sdf_grad_y( _y, _x ).elements()[0]**2 )**0.5 + 1e-3

            arrow_x += [ 0.1 * sdf_grad_x( _y, _x ).elements()[0] / _norm ]
            arrow_y += [ 0.1 * sdf_grad_y( _y, _x ).elements()[0] / _norm ]

            quiver_x += [_x]
            quiver_y += [_y]

    fig2, ax2 = plt.subplots(layout='constrained')

    ax2.quiver( np.array( quiver_x ), -np.array( quiver_y ), np.array( arrow_x ), -np.array( arrow_y ), color='k', angles='xy', scale_units='xy', zorder=4)

    #   Single test
    #x_test = 0.2
    #y_test = 0.4
    #ax2.quiver( x_test, y_test, 0.1 * sdf_grad_x( -y_test, x_test ).elements()[0], -0.1 * sdf_grad_y( -y_test, x_test ).elements()[0], color='g', angles='xy', scale_units='xy', zorder=4)
    ###

    cf = ax2.contourf(X, Y, sdf_check, levels=1000, cmap="brown_cyan", norm=norm, zorder=1)
    ax2.set_aspect('equal', adjustable='box')

    plt.show()

    #   Test CIAO algorithm
    index = 0
    sum_time = []
    sum_iters = []
    
    for cx_0 in [-1.2, 0.4, 2.1, -1.36, 1.38]:
        for cy_0 in [-2.3, -0.4, 3.0, -1.69, -1.48]:
            start = time.time()

            cx = cx_0
            cy = cy_0

            if( sdf_sym( [-cy_0, cx_0] ) > specs["maximum_distance"] ):
                print("ALREADY SAFE REGION.")
                
            else:
                opt_step = min( specs["initial_opt_step"], specs["max_step_size"] )

                _sdf_grad_x = sdf_grad_x( -cy, cx ).elements()[0]
                _sdf_grad_y = sdf_grad_y( -cy, cx ).elements()[0]

                _norm_sdf_grad = ( _sdf_grad_x**2 + _sdf_grad_y**2 )**0.5

                norm_sdf_grad_x = _sdf_grad_x / _norm_sdf_grad
                norm_sdf_grad_y = _sdf_grad_y / _norm_sdf_grad

                sub_index = 0

                while(True):
                    if( sdf_sym( [-cy, cx] ) > specs["maximum_distance"] ):
                        break

                    prev_cx = cx
                    prev_cy = cy

                    cx, cy = obs._ciao_single_iteration(cx_0, cy_0, norm_sdf_grad_x, norm_sdf_grad_y, opt_step)

                    if( sdf_sym( [-cy, cx] ) <= sdf_sym( [-prev_cy, prev_cx] ) ):
                        break

                    opt_step = min( opt_step * specs["growth"], specs["max_step_size"] )

                    sub_index += 1
                
                sum_iters += [sub_index]
            
                if( sdf_sym( [-prev_cy, prev_cx] ) < specs["safety_margin"] ):
                    print("DANGEROUS AREA")
            
            end = time.time() - start
            sum_time += [end]

            print("CIAO time = ", end)

            index += 1
    
    print("(Sum time, avg time, avg iters): ", sum(sum_time), sum(sum_time) / len(sum_time), sum(sum_iters) / len(sum_iters) )