#!/usr/bin/python3.8

import sys
sys.path.insert(0, "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/src")

from classes.all_imports import *
from classes.common_class import *

sys.path.insert(0, "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/src")

""" 
    Obstacle avoidance class

    Methods for related with collision avoidance
"""

class ObstacleAvoidance(Common):
    def __init__(self, x_len, y_len, dx, dy):

        with open(self.ciao_parameters) as f:
            self.specs = json.load(f)
        
        with open(self.list_obstacles) as f:
            self.obstacles = json.load(f)
        
        self.dx = dx
        self.dy = dy

        self.x = np.arange(-x_len, x_len + self.dx, self.dx)        # coordinates for columns
        self.y = np.arange(y_len, -y_len - self.dy, -self.dy)       # coordinates for rows

    def _defineObstacles(self):
        
        X, Y = np.meshgrid(self.x, self.y, indexing='xy')

        obs_list = []

        #   Get circles
        list_circles = self.obstacles["circles"]

        for circle_index in list_circles.keys():
            circle = list_circles[circle_index]
            mask = ( X - circle["x0"] )**2 + ( Y - circle["y0"] )**2 <= circle["r"]**2

            obs_list += [mask]
        ###
        
        #   Get rectangles
        list_rectangles = self.obstacles["rectangles"]

        for rectangle_index in list_rectangles.keys():
            rectangle = list_rectangles[rectangle_index]
            mask = (X >= rectangle["left"]) & (X <= rectangle["right"]) & (Y >= rectangle["bottom"]) & (Y <= rectangle["top"])

            obs_list += [mask]
        ###

        #   Get ellipse
        """list_ellipses = self.obstacles["ellipses"]

        for ellipse_index in list_ellipses.keys():
            ellipse = list_ellipses[ellipse_index]
            mask = ( (X - ellipse["x0"]) / ellipse["a"] )**2 + ( (Y - ellipse["y0"]) / ellipse["b"] )**2 <= 1

            obs_list += [mask]"""
        ###

        #   Get map walls
        map_walls = self.obstacles["map_wall"]

        obs_list += [ X >= map_walls["right"], X <= map_walls["left"], Y >= map_walls["top"], Y <= map_walls["bottom"] ] 
        ###

        return obs_list

    def _get_sdf(self, option = 1):

        #   Single circular obstacle
        if( option == 0 ):

            x_size = self.x.shape[0]
            y_size = self.y.shape[0]

            a = np.ones( ( x_size, y_size ) )

            X, Y = np.meshgrid(self.x, self.y, indexing='xy')

            mask1 = X**2 + Y**2 <= 0.2**2

            a[mask1] = 0

        #   Set of obstacles
        elif(option == 1):
            x_size = self.x.shape[0]
            y_size = self.y.shape[0]

            a = np.ones( ( x_size, y_size ) )

            X, Y = np.meshgrid(self.x, self.y, indexing='xy')

            #   Define obstacles here
            list_obs = self._defineObstacles()

            for obs_element in list_obs:
                a[obs_element] = 0
            ###

        b = ndimage.distance_transform_edt(a, sampling = self.dx)
        b_comp = ndimage.distance_transform_edt(1 - a, sampling = self.dx)

        B = b - b_comp

        B_flat = B.ravel(order='F')
        B_sym = ca.interpolant('name', 'linear', [-self.y, self.x], B_flat)

        return B_sym
    
    def _get_sdf_jacobian(self, sdf):

        x_sym = ca.SX.sym('x_sym')
        y_sym = ca.SX.sym('y_sym')

        z = sdf( ca.vertcat(y_sym, x_sym) )

        sdf_grad_x = ca.Function( 'sdf_grad_x', [y_sym, x_sym], [ ca.gradient(z, x_sym) ] )
        sdf_grad_y = ca.Function( 'sdf_grad_y', [y_sym, x_sym], [ ca.gradient(z, y_sym) ] )

        return sdf_grad_x, sdf_grad_y
    
    def _ciao_single_iteration(self, x0, y0, dir_x, dir_y, opt_step):

        cx = x0 + opt_step * dir_x
        cy = y0 - opt_step * dir_y

        return cx, cy

    def _ciao_full_iteration(self, x0, y0, sdf, sdf_grad_x, sdf_grad_y):

        """
        Docstring for _ciao_full_iteration
        
        :param x0: x coordinate of starting position of the iteration
        :param y0: y coordinate of starting position of the iteration
        :param sdf: distance function to closest obstacle
        :param sdf_grad_x: gradient of distance function with respect to x
        :param sdf_grad_y: gradient of distance function with respect to y
        """

        cx = x0
        cy = y0

        sdf0 = sdf( [-y0, x0] )

        if( sdf0 > self.specs["maximum_distance"] ):
            return 1
        
        else:
            opt_step = min( self.specs["initial_opt_step"], self.specs["max_step_size"] )

            _sdf_grad_x = sdf_grad_x( -y0, x0 ).elements()[0]
            _sdf_grad_y = sdf_grad_y( -y0, x0 ).elements()[0]

            _norm_sdf_grad = ( _sdf_grad_x**2 + _sdf_grad_y**2 )**0.5 + 1e-3

            norm_sdf_grad_x = _sdf_grad_x / _norm_sdf_grad
            norm_sdf_grad_y = _sdf_grad_y / _norm_sdf_grad

            while( opt_step < self.specs["max_step_size"] ):
                prev_cx = cx
                prev_cy = cy

                cx, cy = self._ciao_single_iteration(x0, y0, norm_sdf_grad_x, norm_sdf_grad_y, opt_step)

                dist2obs = sdf( [-cy, cx] ).elements()[0]

                print("(Prev cx, prev cy, cx, cy, sdf0, opt_step + sdf0, dist2obs): ", prev_cx, prev_cy, cx, cy, sdf0, opt_step + sdf0, dist2obs)

                if( dist2obs > opt_step + sdf0 + 1e-2 or dist2obs < opt_step + sdf0 - 1e-2 or dist2obs >= self.specs["maximum_distance"] ):
                    break

                opt_step = min( opt_step * self.specs["growth"], self.specs["max_step_size"] )

            prev_dist2obs = sdf( [-prev_cy, prev_cx] ).elements()[0]

            if( prev_dist2obs < self.specs["safety_margin"] ):
                return 2
            
            else:
                return [prev_cx, prev_cy]

class ObstacleAvoidanceExact(Common):
    def __init__(self):

        with open(self.ciao_parameters) as f:
            self.specs = json.load(f)
        
        with open(self.list_obstacles) as f:
            self.obstacles = json.load(f)

    def _computeMinimumDistance(self, px, py):

        distance2obstacles = []

        obstacle = []

        #   Get circles
        list_circles = self.obstacles["circles"]

        for circle_index in list_circles.keys():
            obstacle += [ [ "circles", circle_index ] ]
            circle = list_circles[circle_index]
            distance = ( ( px - circle["x0"] )**2 + ( py - circle["y0"] )**2 )**0.5 - circle["r"]

            distance2obstacles += [distance]
        ###
        
        #   Get rectangles
        """list_rectangles = self.obstacles["rectangles"]

        for rectangle_index in list_rectangles.keys():
            obstacle += [ [ "rectangle", rectangle_index ] ]
            rectangle = list_rectangles[rectangle_index]

            hx = ( rectangle["right"] - rectangle["left"] ) / 2.0
            hy = ( rectangle["top"] - rectangle["bottom"] ) / 2.0

            cx = rectangle["left"] + hx
            cy = rectangle["bottom"] + hy

            qx = abs(px - cx) - hx
            qy = abs(py - cy) - hy

            distance = ( max(qx, 0)**2 + max(qy, 0)**2 )**0.5 + min( max(qx, qy), 0 )

            distance2obstacles += [distance]"""
        ###

        #   Get map walls
        map_walls = self.obstacles["map_wall"]

        distance2obstacles += [ map_walls["right"] - px, px - map_walls["left"], map_walls["top"] - py, py - map_walls["bottom"] ]
        obstacle += [ [ "map_walls", "right" ], [ "map_walls", "left" ], [ "map_walls", "top" ], [ "map_walls", "bottom" ] ]
        ###

        min_distance = min(distance2obstacles)

        index = 0

        min_distance_objects = []

        for d in distance2obstacles:
            if d == min_distance:
                min_distance_objects += [ obstacle[index] ]
                
            index += 1

        return min_distance, min_distance_objects

    def _plotObstacles(self, ax, transparency=1.0, plotSafetyMargin=False):
        
        for key_obj in self.obstacles.keys():
            for key_index in self.obstacles[key_obj]:
                
                if(key_obj == "circles"):
                    c = plt.Circle( (self.obstacles[key_obj][key_index]["x0"], self.obstacles[key_obj][key_index]["y0"]), self.obstacles[key_obj][key_index]["r"], alpha = transparency, color='0.5', fill=True, visible=True, zorder=3)
                    ax.add_patch(c)

                    if(plotSafetyMargin):
                        c = plt.Circle( (self.obstacles[key_obj][key_index]["x0"], self.obstacles[key_obj][key_index]["y0"]), self.obstacles[key_obj][key_index]["r"] + self.specs["safety_margin"],\
                                            ls='--', alpha = transparency, color='0.5', fill=False, visible=True, zorder=3)
                        ax.add_patch(c)
                
                elif(key_obj == "rectangles"):
                    r = plt.Rectangle( (self.obstacles[key_obj][key_index]["left"], self.obstacles[key_obj][key_index]["bottom"]),\
                                        self.obstacles[key_obj][key_index]["right"] - self.obstacles[key_obj][key_index]["left"],\
                                        self.obstacles[key_obj][key_index]["top"] - self.obstacles[key_obj][key_index]["bottom"], alpha=transparency, color='0.5', fill=True, visible=True, zorder=3)
                    ax.add_patch(r)

    def _computeSDFGradient(self, px, py):
        min_distance, closest_obstacle = self._computeMinimumDistance(px, py)

        vx = 0
        vy = 0

        for closest_obstacle_element in closest_obstacle:
            object = closest_obstacle_element[0]
            object_index = closest_obstacle_element[1]

            if(object == 'circles'):
                _vx, _vy = self._gradClosestPoint2Circle(object_index, px, py)
            
            elif(object == 'rectangle'):
                _vx, _vy = self._gradClosestPoint2Rectangle(object_index, px, py)
            
            elif(object == 'map_walls'):
                _vx, _vy = self._gradClosestPoint2Wall(object_index)
            
            vx += _vx
            vy += _vy
        
        norm_v = ( vx**2 + vy**2 )**0.5 + 1e-6
        norm_vx = vx / norm_v
        norm_vy = vy / norm_v

        return norm_vx, norm_vy, min_distance
    
    def _gradClosestPoint2Circle(self, index, px, py):

        vx = px - self.obstacles["circles"][index]["x0"]
        vy = py - self.obstacles["circles"][index]["y0"]

        norm = (vx**2 + vy**2)**0.5 + 1e-6

        norm_vx = vx / norm
        norm_vy = vy / norm

        return norm_vx, norm_vy
    
    def _gradClosestPoint2Rectangle(self, index, px, py):

        left = self.obstacles["rectangles"][index]["left"]
        right = self.obstacles["rectangles"][index]["right"]
        top = self.obstacles["rectangles"][index]["top"]
        bottom = self.obstacles["rectangles"][index]["bottom"]

        if( px < left ):
            loc = "L"

        elif( px >= left and px <= right ):
            loc = "I"

        else:
            loc = "R"

        if( py < bottom ):
            loc += "B"

        elif( py >= bottom and py <= top ):
            loc += "I"
        
        else:
            loc += "U"

        #   Point is inside rectangle
        if( loc == "II" ):
            
            diff_r = right - px
            diff_l = px - left

            diff_top = top - py
            diff_bottom = py - bottom

            diff_list = [diff_l, diff_r, diff_bottom, diff_top]

            sdiff_list = sorted(diff_list)
            smallest_diff = sdiff_list[0]

            vx = 0
            vy = 0

            sindex = 0
            for s in diff_list:
                if s == smallest_diff:
                    
                    if( sindex == 0 ):
                        vx += -1.0
                    
                    elif( sindex == 1 ):
                        vx += 1.0
                    
                    elif( sindex == 2 ):
                        vy += -1.0
                    
                    elif( sindex == 3 ):
                        vy += 1.0

                sindex += 1

        #   Point is outside rectangle
        else:
            if( loc == "LU" ):
                vx = px - left
                vy = py - top
        
            elif( loc == "LI" ):
                vx = -1.0
                vy = 0.0
            
            elif( loc == "LB" ):
                vx = px - left
                vy = py - bottom
            
            elif( loc == "IB" ):
                vx = 0.0
                vy = -1.0
        
            elif( loc == "RB" ):
                vx = px - right
                vy = py - bottom
            
            elif( loc == "RI" ):
                vx = 1.0
                vy = 0.0
            
            elif( loc == "RU" ):
                vx = px - right
                vy = py - top
        
            elif( loc == "IU" ):
                vx = 0.0
                vy = 1.0
        
        norm = (vx**2 + vy**2)**0.5 + 1e-6

        norm_vx = vx / norm
        norm_vy = vy / norm

        return norm_vx, norm_vy
    
    def _gradClosestPoint2Wall(self, index):

        if( index == "left" ):
            vx = 1.0
            vy = 0.0
        
        elif( index == "right" ):
            vx = -1.0
            vy = 0.0
        
        elif( index == "top" ):
            vx = 0.0
            vy = -1.0
        
        elif( index == "bottom" ):
            vx = 0.0
            vy = 1.0
        
        return vx, vy

    def _ciao_single_iteration(self, x0, y0, dir_x, dir_y, opt_step):

        cx = x0 + opt_step * dir_x
        cy = y0 + opt_step * dir_y

        return cx, cy

    def _ciao_full_iteration(self, x0, y0):

        """
        Docstring for _ciao_full_iteration
        
        :param x0: x coordinate of starting position of the iteration
        :param y0: y coordinate of starting position of the iteration
        :param sdf: distance function to closest obstacle
        :param sdf_grad_x: gradient of distance function with respect to x
        :param sdf_grad_y: gradient of distance function with respect to y
        """

        cx = x0
        cy = y0

        norm_vx, norm_vy, sdf0 = self._computeSDFGradient(cx, cy)

        dist2obs = sdf0

        if( sdf0 > self.specs["maximum_distance"] ):
            return 1, sdf0
        
        else:
            opt_step = min( self.specs["initial_opt_step"], self.specs["max_step_size"] )

            while( opt_step < self.specs["max_step_size"] ):
                prev_cx = cx
                prev_cy = cy
                prev_dist2obs = dist2obs

                cx, cy = self._ciao_single_iteration(x0, y0, norm_vx, norm_vy, opt_step)

                dist2obs, _ = self._computeMinimumDistance(cx, cy)

                if( dist2obs > opt_step + sdf0 + 1e-6 or dist2obs < opt_step + sdf0 - 1e-6 or dist2obs >= self.specs["maximum_distance"] ):
                    break

                opt_step = min( opt_step * self.specs["growth"], self.specs["max_step_size"] )

            if( prev_dist2obs < self.specs["safety_margin"] ):
                #print("In dangerous space.")
                return [prev_cx, prev_cy], 0
            
            else:
                return [prev_cx, prev_cy], prev_dist2obs
    
    def _projectPoint2FreeSpace(self, x0, y0, step):
        
        norm_vx, norm_vy, sdf0 = self._computeSDFGradient(x0, y0)

        x_proj = x0 + norm_vx * (step - sdf0)
        y_proj = y0 + norm_vy * (step - sdf0) 

        return x_proj, y_proj

    def _computeCiaoHorizon(self, x_guess, y_guess, circles, plot_anim=True):

        cx = []
        cy = []
        d2o = []

        k = 0
        for x_element, y_element in zip(x_guess, y_guess):
            res, prev_dist = self._ciao_full_iteration( x_element, y_element)

            if(res == 1):
                cx += [x_element]
                cy += [y_element]

                d2o += [ prev_dist ]
            
            else:
                cx += [ res[0] ]
                cy += [ res[1] ]

                d2o += [ prev_dist ]

            if(plot_anim):
                circles[k].set_center( ( cx[-1], cy[-1] ) )
                circles[k].set_radius( d2o[-1] - self.specs["safety_margin"] )

            k += 1
        
        return cx, cy, d2o
    
class PotentialField(Common):
    def __init__(self):

        with open(self.ciao_parameters) as f:
            self.specs = json.load(f)
        
        with open(self.list_obstacles) as f:
            self.obstacles = json.load(f)

        with open(self.potential_field) as f:
            self.potential_field_parameters = json.load(f)

            self.d0 = self.potential_field_parameters["d0"]
    
    def _targetAttraction(self, xt, yt, x, y):

        norm = ( (xt - x)**2 + (yt - y)**2 )**0.5

        vx = self.potential_field_parameters["k_att"] * (xt - x) / norm
        vy = self.potential_field_parameters["k_att"] * (yt - y) / norm

        return vx, vy
    
    def _sumRepulsions(self, x, y):

        sum_x = 0
        sum_y = 0

        for key_obj in self.obstacles.keys():
            for key_index in self.obstacles[key_obj]:
                
                if(key_obj == "circles"):
                    vx, vy = self._circleRepulsion(x,\
                                                   y,\
                                                   self.obstacles[key_obj][key_index]["r"],\
                                                   self.obstacles[key_obj][key_index]["x0"],\
                                                   self.obstacles[key_obj][key_index]["y0"])
                    
                elif(key_obj == "rectangles"):
                    vx, vy = self._rectangleRepulsion(self.obstacles[key_obj][key_index]["left"],\
                                                      self.obstacles[key_obj][key_index]["right"],\
                                                      self.obstacles[key_obj][key_index]["top"],\
                                                      self.obstacles[key_obj][key_index]["bottom"],\
                                                      x,\
                                                      y )
            
                sum_x += vx
                sum_y += vy
        
        return sum_x, sum_y
    
    def _circleRepulsion(self, x, y, r, xo, yo):

        d2o = ( (x - xo)**2  + (y - yo)**2 )**0.5 - r 

        if(d2o < self.d0 and d2o > 1e-6):
            return self.potential_field_parameters["k_rep"] * (1 / d2o - 1 / self.d0) * (x - xo) / d2o,\
                   self.potential_field_parameters["k_rep"] * (1 / d2o - 1 / self.d0) * (y - yo) / d2o

        else:
            return 0, 0

    def _rectangleRepulsion(self, left, right, top, bottom, px, py):

        if( px < left + 1e-6 ):
            loc = "L"

        elif( px >= left + 1e-6 and px <= right + 1e-6 ):
            loc = "I"

        else:
            loc = "R"

        if( py < bottom + 1e-6):
            loc += "B"

        elif( py >= bottom + 1e-6 and py <= top + 1e-6 ):
            loc += "I"
        
        else:
            loc += "U"

        #   Point is inside rectangle
        if( loc == "II" ):
            return 0.0, 0.0

        #   Point is outside rectangle
        else:
            if( loc == "LU" ):
                vx = px - left
                vy = py - top
        
            elif( loc == "LI" ):
                vx = px - left
                vy = 0.0
            
            elif( loc == "LB" ):
                vx = px - left
                vy = py - bottom
            
            elif( loc == "IB" ):
                vx = 0.0
                vy = py - bottom
        
            elif( loc == "RB" ):
                vx = px - right
                vy = py - bottom
            
            elif( loc == "RI" ):
                vx = px - right
                vy = 0.0
            
            elif( loc == "RU" ):
                vx = px - right
                vy = py - top
        
            elif( loc == "IU" ):
                vx = 0.0
                vy = py - top
        
            d2o = (vx**2 + vy**2)**0.5

            if(d2o < self.d0 and d2o > 1e-6):
                norm_vx = vx / d2o * self.potential_field_parameters["k_rep"] * (1 / d2o - 1 / self.d0)
                norm_vy = vy / d2o * self.potential_field_parameters["k_rep"] * (1 / d2o - 1 / self.d0)

                return norm_vx, norm_vy
            
            else:
                return 0.0, 0.0
    
    def _computeFreeCollisionReference(self, px, py, yaw, tx, ty, v_speed, N):

        """
        Docstring for _computeFreeCollisionReference
        
        :param px: x coordinate of robot position
        :param py: y coordinate of robot position
        :param tx: x coordinate of target position
        :param ty: y coordinate of robot position
        :param v_speed: virtual speed
        :param N: horizon length
        """

        j = 0

        new_ref_x = px
        new_ref_y = py
        new_ref_yaw = yaw

        ref_x = [new_ref_x]
        ref_y = [new_ref_y]
        ref_yaw = [new_ref_yaw]

        while(j < N):
            dir_x_target_att, dir_y_target_att = self._targetAttraction(tx, ty, new_ref_x, new_ref_y)

            dir_x_target_rep, dir_y_target_rep = self._sumRepulsions(new_ref_x, new_ref_y)

            dir_x = dir_x_target_att + dir_x_target_rep
            dir_y = dir_y_target_att + dir_y_target_rep
            norm_dir = ( dir_x**2 + dir_y**2 )**0.5

            new_ref_x += dir_x / norm_dir * v_speed
            new_ref_y += dir_y / norm_dir * v_speed
            new_ref_yaw = math.atan2(new_ref_y - ref_y[-1], new_ref_x - ref_x[-1])

            yaw_diff2target = math.atan2( new_ref_y - ty, new_ref_x - tx )

            #   Predict next reference
            next_ref_x = new_ref_x + dir_x / norm_dir * v_speed
            next_ref_y = new_ref_y + dir_y / norm_dir * v_speed

            next_yaw_diff2target = math.atan2( next_ref_y - ty, next_ref_x - tx )

            yaw_diff = math.atan2( math.sin(next_yaw_diff2target - yaw_diff2target), math.cos(next_yaw_diff2target - yaw_diff2target) )

            if( ( (new_ref_x - tx)**2 + (new_ref_y - ty)**2 )**0.5 < 0.05 or ( (next_ref_x - tx)**2 + (next_ref_y - ty)**2 )**0.5 < 0.05 or yaw_diff <= -math.pi + 0.1 or yaw_diff >= math.pi - 0.1 ):
                break

            else:
                ref_x.append(new_ref_x)
                ref_y.append(new_ref_y)
                ref_yaw.append( new_ref_yaw )

            j += 1
        
        while(j < N):
            ref_x.append(new_ref_x)
            ref_y.append(new_ref_y)
            ref_yaw.append( ref_yaw[-1] )
            
            j += 1
        
        return ref_x, ref_y, ref_yaw