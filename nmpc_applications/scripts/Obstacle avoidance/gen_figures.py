#!/usr/bin/python3
from pathlib import Path

classes_path = Path(__file__)

upper_directory = classes_path.parent.parent.parent
lower_directory = upper_directory / "src"

from sys import path
import matplotlib.colors as mcolors
import matplotlib.animation as animation

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
    obs = ObstacleAvoidance(6, 6, 5e-3, 5e-3)

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
    step=10

    quiver_x = []
    quiver_y = []
    arrow_x = []
    arrow_y = []

    for _y in y[::step]:
        for _x in x[::step]:
            obs._computeSDFGradient(_x, _y)
            """_norm = ( sdf_grad_x( _y, _x ).elements()[0]**2 + sdf_grad_y( _y, _x ).elements()[0]**2 )**0.5 + 1e-3

            arrow_x += [ 0.1 * sdf_grad_x( _y, _x ).elements()[0] / _norm ]
            arrow_y += [ 0.1 * sdf_grad_y( _y, _x ).elements()[0] / _norm ]

            quiver_x += [_x]
            quiver_y += [_y]"""

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
    input("Test CIAO algorithm.")
    fig2, ax2 = plt.subplots(layout='constrained')

    colors_list = list( mcolors.CSS4_COLORS.keys() )

    index = 0

    for cx_0 in [-1.2, 0.4, 2.1, -0.06]:
        for cy_0 in [-2.3, -0.4, 3.0, 0.14]:
            start = time.time()

            cx = cx_0
            cy = cy_0

            sdf0 = sdf_sym( [ -cy_0, cx_0 ] ).elements()[0]

            if( sdf0 > specs["maximum_distance"] ):
                ax2.scatter(cx_0, cy_0, s=50, c=colors_list[index], marker='X', zorder=3)

                #   Create a circle to denote the safe area
                circle = plt.Circle( (cx_0, cy_0), sdf0 - specs["safety_margin"], color=str( colors_list[index] ), fill=False, zorder=4)

                # Add the patch to the axis
                ax2.add_patch(circle)

                print("ALREADY SAFE REGION.")
                
            else:
                opt_step = min( specs["initial_opt_step"], specs["max_step_size"] )

                _sdf_grad_x = sdf_grad_x( -cy, cx ).elements()[0]
                _sdf_grad_y = sdf_grad_y( -cy, cx ).elements()[0]

                _norm_sdf_grad = ( _sdf_grad_x**2 + _sdf_grad_y**2 )**0.5 + 1e-3

                norm_sdf_grad_x = _sdf_grad_x / _norm_sdf_grad
                norm_sdf_grad_y = _sdf_grad_y / _norm_sdf_grad

                ax2.scatter(cx_0, cy_0, s=50, c=colors_list[index], marker='X', zorder=3)
                ax2.quiver( cx_0, cy_0, opt_step * norm_sdf_grad_x, -opt_step * norm_sdf_grad_y,\
                            color=colors_list[index], angles='xy', scale=1.1, scale_units='xy', zorder=2)

                flag = False

                while( opt_step < specs["max_step_size"] ):
                    if(not flag):
                        flag = True
                    
                    else:
                        ax2.scatter(cx, cy, s=50, c=colors_list[index], marker='X', zorder=3)

                    prev_cx = cx
                    prev_cy = cy

                    cx, cy = obs._ciao_single_iteration(cx_0, cy_0, norm_sdf_grad_x, norm_sdf_grad_y, opt_step)

                    #print("UPDATE -> (cx, cy, cx0, cy0): ", cx, cy, cx_0, cy_0)

                    dist2obs = sdf_sym( [-cy, cx] ).elements()[0]

                    print("(Prev cx, prev cy, cx, cy, sdf0, opt_step + sdf0, dist2obs): ", prev_cx, prev_cy, cx, cy, sdf0, opt_step + sdf0, dist2obs)

                    if( dist2obs < opt_step + sdf0 - 1e-2 or dist2obs > opt_step + sdf0 + 1e-2 or dist2obs > specs["maximum_distance"] ):
                        break

                    opt_step = min( opt_step * specs["growth"], specs["max_step_size"] )
            
                prev_dist2obs = sdf_sym( [-prev_cy, prev_cx] ).elements()[0]

                if( prev_dist2obs <= specs["safety_margin"] ):
                    print("DANGEROUS AREA")
                
                else:
                    #   Create a circle to denote the safe area
                    circle = plt.Circle( (prev_cx, prev_cy), prev_dist2obs - specs["safety_margin"], color=str( colors_list[index] ), fill=False, zorder=4)

                    # Add the patch to the axis
                    ax2.add_patch(circle)
            
            end = time.time() - start

            print("CIAO time = ", end)

            index += 1

    cf = ax2.contourf(X, Y, sdf_check, levels=1000, cmap="brown_cyan", norm=norm, zorder=1)
    ax2.set_aspect('equal', adjustable='box')
    fig2.colorbar(cf, ax = ax2, label=rf"Distance to closest obstacle [m]")

    plt.show()

    #   Make picture with array from initial to end point
    fig2, ax2 = plt.subplots(layout='constrained')

    colors_list = list( mcolors.CSS4_COLORS.keys() )

    index = 0

    for cx_0 in [-1.2, 0.4, 2.1]:
        for cy_0 in [-2.3, -0.4, 3.0]:
            ax2.scatter(cx_0, cy_0, s=50, c=colors_list[index], marker='X', zorder=3)

            res = obs._ciao_full_iteration(cx_0, cy_0, sdf_sym, sdf_grad_x, sdf_grad_y)

            print(res)

            if( res != 1 and res != 2 ):
                ax2.scatter(res[0], res[1], s=50, c=colors_list[index], marker='o', zorder=3)
                ax2.quiver( cx_0, cy_0, res[0] - cx_0, res[1] - cy_0,\
                            color=colors_list[index], scale=1, angles='xy', scale_units='xy', zorder=2)

                #   Create a circle to denote the safe area
                circle = plt.Circle( (res[0], res[1]), sdf_sym( [-res[1], res[0]] ).elements()[0] - specs["safety_margin"], color=str( colors_list[index] ), fill=False, zorder=4)

                # Add the patch to the axis
                ax2.add_patch(circle)

            index += 1

    cf = ax2.contourf(X, Y, sdf_check, levels=1000, cmap="brown_cyan", norm=norm, zorder=1)
    ax2.set_aspect('equal', adjustable='box')
    fig2.colorbar(cf, ax = ax2, label=rf"Distance to closest obstacle [m]")

    plt.show()

    #   Get circles with respect to reference
    fig2, ax2 = plt.subplots(layout='constrained')

    colors_list = list( mcolors.CSS4_COLORS.keys() )

    index = 0

    for cx_0, cy_0 in zip( x_ref( np.arange(0, 2 * ca.pi, 0.2) ).elements(), y_ref( np.arange(0, 2 * ca.pi, 0.2) ).elements() ):
        ax2.scatter(cx_0, cy_0, s=50, c=colors_list[index], marker='X', zorder=3)

        start = time.time()

        res = obs._ciao_full_iteration(cx_0, cy_0, sdf_sym, sdf_grad_x, sdf_grad_y)
        
        end = time.time() - start
        print("CIAO time = ", end)

        if( res != 1 and res != 2 ):
            ax2.scatter(res[0], res[1], s=50, c=colors_list[index], marker='o', zorder=3)
            ax2.quiver( cx_0, cy_0, res[0] - cx_0, res[1] - cy_0,\
                        color=colors_list[index], scale=1, angles='xy', scale_units='xy', zorder=2)

            #   Create a circle to denote the safe area
            circle = plt.Circle( (res[0], res[1]), sdf_sym( [-res[1], res[0]] ).elements()[0] - specs["safety_margin"], color=str( colors_list[index] ), fill=False, zorder=4)

            # Add the patch to the axis
            ax2.add_patch(circle)

        index += 1

    ax2.plot(_x_ref, _y_ref, 'k--', linewidth=0.6, zorder=2 )
    cf = ax2.contourf(X, Y, sdf_check, levels=1000, cmap="brown_cyan", norm=norm, zorder=1)
    ax2.set_aspect('equal', adjustable='box')
    fig2.colorbar(cf, ax = ax2, label=rf"Distance to closest obstacle [m]")

    plt.show()