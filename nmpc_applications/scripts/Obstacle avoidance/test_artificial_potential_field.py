#!/usr/bin/python3
from pathlib import Path

classes_path = Path(__file__)

upper_directory = classes_path.parent.parent.parent
lower_directory = upper_directory / "src"

from sys import path
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
        ciao_parameters = json.load(f)

    plt.rcParams.update( {'font.family': 'Arial',                                              
                            'font.size': 30,
                            'lines.linewidth': 4.0} )
    
    obs = ObstacleAvoidanceExact()
    ref = Reference(option = 1, qa=1.0, qb=1.0, qc=0.4)
    pot = PotentialField()

    x_ref = ref.__dict__["x_ref"]
    y_ref = ref.__dict__["y_ref"]
    yaw_ref = ref.__dict__["yaw_ref"]
    curvature = ref.__dict__["curvature"]
    roll_ref = ref.__dict__["roll_ref"]
    pitch_ref = ref.__dict__["pitch_ref"]

    disc_steps = np.arange(0.0, 2 * ca.pi, 0.03)

    _x_ref = x_ref( disc_steps )
    _y_ref = y_ref( disc_steps )

    x = np.arange(-4, 4, 1e-2)
    y = np.arange(-4, 4, 1e-2)

    arrow_x = []
    arrow_y = []
    quiver_x = []
    quiver_y = []

    target_x = -1.97
    target_y = 0.7

    step=8

    i = 0

    for _y in y[::step]:
        j = 0

        for _x in x[::step]:
            arrow_x += [_x]
            arrow_y += [_y]

            dir_x_target_att, dir_y_target_att = pot._targetAttraction(target_x, target_y, _x, _y)

            dir_x_target_rep, dir_y_target_rep = pot._sumRepulsions(_x, _y)

            dir_x = dir_x_target_att + dir_x_target_rep
            dir_y = dir_y_target_att + dir_y_target_rep
            norm_dir = ( dir_x**2 + dir_y**2 )**0.5

            norm_dir_x = dir_x / norm_dir
            norm_dir_y = dir_y / norm_dir      

            quiver_x += [norm_dir_x]
            quiver_y += [norm_dir_y]

            j += 1
        i += 1
    
    fig, ax = plt.subplots(layout='constrained')
    obs._plotObstacles(ax, transparency=1.0)
    ax.scatter(target_x, target_y, s=80, c='r', zorder=2)
    ax.quiver(arrow_x, arrow_y, quiver_x, quiver_y, color='k', scale=12.0, angles='xy', scale_units='xy')
    ax.set_aspect('equal', adjustable='box')
    ax.set(xlim=[-2.8, 2.8], ylim=[-2.8, 2.8], xlabel=rf"$x\,[m]$", ylabel=rf"$y\,[m]$")

    #   Plot potential field local minimums
    """step=1

    x = np.arange(-4, 4, 1e-3)
    y = np.arange(-4, 4, 1e-3)

    i = 0

    for _y in y[::step]:
        print(_y)
        j = 0

        for _x in x[::step]:
            arrow_x += [_x]
            arrow_y += [_y]

            dir_x_target_att, dir_y_target_att = pot._targetAttraction(target_x, target_y, _x, _y)

            dir_x_target_rep, dir_y_target_rep = pot._sumRepulsions(_x, _y)

            dir_x = dir_x_target_att + dir_x_target_rep
            dir_y = dir_y_target_att + dir_y_target_rep

            #   Plot potential field local minimums
            if( abs(dir_x) < 2e-2 and abs(dir_y) < 2e-2 ):
                ax.scatter(_x, _y, s=80, c='g', zorder=2)

            j += 1
        i += 1"""

    plt.show()

    #   Generate collision-free reference
    """plt.ion()
    fig, ax = plt.subplots()

    fig.canvas.draw()
    background = fig.canvas.copy_from_bbox(ax.bbox)

    obs._plotObstacles(ax, transparency=1.0)
    ref_plot = ax.scatter([], [], c='k', s=20.0, zorder=5, visible=True)
    target_plot = ax.scatter([], [], c='g', s=20.0, zorder=5, visible=True)
    target_free_plot = ax.scatter([], [], c='g', marker='x', s=20.0, zorder=5, visible=True)
    ax.plot(_x_ref, _y_ref, 'k--', linewidth=0.6, zorder=4)
    ax.set_aspect('equal')
    ax.set(xlim=[-4.0, 4.0], ylim=[-4.0, 4.0], xlabel=rf"$x\,[m]$", ylabel=rf"$y\,[m]$")

    progress = 0.3
    progress_ahead = 0.3

    v_speed = 0.01
    n = 50
    i = 0
    range_sim = 10000
    
    while(i < range_sim):
        start = time.time()

        _x_ref = x_ref( progress ).elements()[0]
        _y_ref = y_ref( progress ).elements()[0]
        _yaw_ref = yaw_ref( progress ).elements()[0]

        target_x = x_ref( progress + progress_ahead ).elements()[0]
        target_y = y_ref( progress + progress_ahead ).elements()[0]

        min_dist, _ = obs._computeMinimumDistance(target_x, target_y)

        if(min_dist <= ciao_parameters["safety_margin"] * 1.2):
            target_proj = obs._projectPoint2FreeSpace(target_x, target_y, ciao_parameters["safety_margin"] * 1.2)
            
            target_free_plot.set_visible(True)
            target_free_plot.set_offsets( np.c_[target_proj[0], target_proj[1]] )
        
        else:
            target_proj = [target_x, target_y]
            target_free_plot.set_visible(False)

        target_plot.set_offsets( np.c_[ target_x, target_y ] )

        ref_x, ref_y, _ = pot._computeFreeCollisionReference(_x_ref, _y_ref, _yaw_ref, target_proj[0], target_proj[1], v_speed, n)
        
        ref_plot.set_offsets(np.c_[ref_x, ref_y])

        i += 1
        progress += v_speed

        fig.canvas.restore_region(background)

        ax.draw_artist(target_plot)

        fig.canvas.blit(ax.bbox)
        fig.canvas.flush_events()

        input()

    plt.ioff()
    plt.show()"""

    #   Plot some deformed references on the same picture
    fig, ax = plt.subplots(layout='constrained')

    obs._plotObstacles(ax, transparency=1.0, plotSafetyMargin=True)
    ax.plot(_x_ref, _y_ref, 'k--', linewidth=0.6, zorder=4, label='Baseline reference')
    ax.set(xlim=[-3.0, 3.0], ylim=[-2.5, 2.5], xlabel=rf"$x\,[m]$", ylabel=rf"$y\,[m]$")

    progress_ahead = 0.3
    v_speed = 0.02

    n = 50

    colors_list = ['r', 'b', 'g', 'k', 'c']

    index = 0
    
    for progress in [0.2, 1.25, 1.75, 2.5, 3.0]:
        _x_ref = x_ref( progress ).elements()[0]
        _y_ref = y_ref( progress ).elements()[0]
        _yaw_ref = yaw_ref( progress ).elements()[0]

        target_x = x_ref( progress + progress_ahead ).elements()[0]
        target_y = y_ref( progress + progress_ahead ).elements()[0]

        min_dist, _ = obs._computeMinimumDistance(target_x, target_y)

        if(min_dist <= ciao_parameters["safety_margin"]):
            target_proj = obs._projectPoint2FreeSpace(target_x, target_y, ciao_parameters["safety_margin"] * 1.2)
        
        else:
            target_proj = [target_x, target_y]

        ax.scatter(target_proj[0], target_proj[1], c=colors_list[index], s=80, zorder=5)
        ax.scatter(target_x, target_y, marker='X', c=colors_list[index], s=80, zorder=5)

        ref_x, ref_y, _ = pot._computeFreeCollisionReference(_x_ref, _y_ref, _yaw_ref, target_proj[0], target_proj[1], v_speed, n)

        ax.scatter(ref_x, ref_y, c=colors_list[index], s=10.0, zorder=5)

        index += 1

    ax.legend()
    ax.set_aspect('equal', adjustable='box')
    plt.show()

    #   Get computation time of algorithm 11
    fig, ax = plt.subplots(layout='constrained')

    progress_ahead = 0.3
    v_speed = 0.02

    n = 50

    index = 0
    times = []
    
    for progress in np.arange(0, 2 * math.pi, 1e-3):
        start = time.time()
        _x_ref = x_ref( progress ).elements()[0]
        _y_ref = y_ref( progress ).elements()[0]
        _yaw_ref = yaw_ref( progress ).elements()[0]

        ref_min_dist, _ = obs._computeMinimumDistance(_x_ref, _y_ref)

        if( ref_min_dist < ciao_parameters["safety_margin"] ):
            print("Position in the safe space complementary space.")
        
        else:
            target_x = x_ref( progress + progress_ahead ).elements()[0]
            target_y = y_ref( progress + progress_ahead ).elements()[0]

            min_dist, _ = obs._computeMinimumDistance(target_x, target_y)

            if(min_dist <= ciao_parameters["safety_margin"] * 1.2):
                target_proj = obs._projectPoint2FreeSpace(target_x, target_y, ciao_parameters["safety_margin"] * 1.2)
            
            else:
                target_proj = [target_x, target_y]

            ref_x, ref_y, _ = pot._computeFreeCollisionReference(_x_ref, _y_ref, _yaw_ref, target_proj[0], target_proj[1], v_speed, n)

            times.append(time.time() - start)

            index += 1
    
    print("(AVG times, max time, index): ", sum(times) / index, max(times), index)