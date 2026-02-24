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
    obs = ObstacleAvoidanceExact()

    #   Generate reference
    x_ref = ref.__dict__["x_ref"]
    y_ref = ref.__dict__["y_ref"]
    yaw_ref = ref.__dict__["yaw_ref"]
    curvature = ref.__dict__["curvature"]

    x = np.arange(-6, 6, 1e-2)
    y = np.arange(-6, 6, 1e-2)

    #   Check sdf
    sdf_check = np.ones( ( len(x), len(y) ) )

    sum_times = 0

    i = 0

    for _y in y:
        j = 0

        for _x in x:
            start = time.time()
            sdf_check[i, j], _ = obs._computeMinimumDistance(_x, _y)
            sum_times += time.time() - start

            j += 1
        i += 1
    
    print("AVG times: ", sum_times / ( len(y) * len(x) ) )
    
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

    #   Second sdf gradient check
    step=15

    quiver_x = []
    quiver_y = []
    arrow_x = []
    arrow_y = []

    sum_times = 0

    for _y in y[::step]:
        for _x in x[::step]:
            start = time.time()
            norm_vx, norm_vy, _ = obs._computeSDFGradient(_x, _y)
            sum_times += time.time() - start

            quiver_x += [_x]
            quiver_y += [_y]

            arrow_x += [norm_vx]
            arrow_y += [norm_vy]
    
    print("AVG TIME: ", sum_times / ( len( y[::step] ) * len( x[::step] ) ) )

    fig2, ax2 = plt.subplots(layout='constrained')

    ax2.quiver( np.array( quiver_x ), np.array( quiver_y ), np.array( arrow_x ) * 0.1, np.array( arrow_y ) * 0.1, color='k', scale = 1.0, angles='xy', scale_units='xy', zorder=4)

    cf = ax2.contourf(X, Y, sdf_check, levels=1000, cmap="brown_cyan", norm=norm, zorder=1)
    ax2.set_xlabel(r'$x$ [m]')
    ax2.set_ylabel(r'$y$ [m]')
    ax2.set_aspect('equal', adjustable='box')
    fig2.colorbar(cf, ax=ax2, label=rf"Distance to closest obstacle [m]")

    plt.show()

    #   Get circles with respect to reference
    fig2, ax2 = plt.subplots(layout='constrained')

    colors_list = list( mcolors.CSS4_COLORS.keys() )

    index = 0

    sum_times = 0
    times = []

    for cx_0, cy_0 in zip( x_ref( np.arange(0, 2 * ca.pi, 0.01) ).elements(), y_ref( np.arange(0, 2 * ca.pi, 0.01) ).elements() ):
        #ax2.scatter(cx_0, cy_0, s=50, c='k', marker='X', zorder=3)

        start = time.time()

        res, prev_dist = obs._ciao_full_iteration(cx_0, cy_0)

        if( res != 1 and res != 2 ):
            pass
            #ax2.scatter(res[0], res[1], s=50, c='k', marker='o', zorder=3)
            #ax2.quiver( cx_0, cy_0, res[0] - cx_0, res[1] - cy_0, color='r', scale=1, angles='xy', scale_units='xy', zorder=2)

            dist, _ = obs._computeMinimumDistance(res[0], res[1])

            #   Create a circle to denote the safe area
            #circle = plt.Circle( (res[0], res[1]), dist - specs["safety_margin"], color='r', fill=False, zorder=4)

            # Add the patch to the axis
            #ax2.add_patch(circle)
        
        sum_times += time.time() - start
        times += [time.time() - start]

        index += 1

    ax2.set_xlim(left=-4.0, right=4.0)
    ax2.set_ylim(bottom=-4.0, top=4.0)
    ax2.set_xlabel(r'$x$ [m]')
    ax2.set_ylabel(r'$y$ [m]')
    cf = ax2.contourf(X, Y, sdf_check, levels=1000, cmap="brown_cyan", norm=norm, zorder=1)
    ax2.set_aspect('equal', adjustable='box')
    fig2.colorbar(cf, ax = ax2, label=rf"Distance to closest obstacle [m]")

    plt.show()

    print("AVG TIMES, max time, index: ", sum_times / index, max(times), index )