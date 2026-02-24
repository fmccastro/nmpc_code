#!/usr/bin/python3.8

import sys

import scipy.linalg
sys.path.insert(0, "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/src")

from classes.all_imports import *
from classes.common_class import *

"""
    Build and define NMPC optimization problems with the standard baseline formulation

    Heading regulation is included. Refinement of the solution is performed as well. Obstacle avoidance is included.

    Solver is implemented in casadi with direct multiple shooting discretization
"""

#   Optimization problem parameters
class ModelParameters(Common):

    #   Constructor
    def __init__(self):
        
        super().__init__()

        with open(self.baseline_std_dyn_skid) as f:
            solver_param = json.load(f)
            specs = solver_param["solver_specs"]
            self.con_pose = solver_param["con_pose"]
            self.con_vel = solver_param["con_vel"]
            self.weights = solver_param["weights"]
        
        #   Work with Leo Rover
        with open(self.vehicle_specs) as f:
            vehicle_param = json.load(f)
            lat_w = vehicle_param["wheelLatSeparation"]
        
        ###     Optimization symbolic variables   ############################

        """
            Mobile robot states
        """

        self.x = ca.MX.sym('x')
        self.y = ca.MX.sym('y')
        self.yaw = ca.MX.sym('yaw')
        self.progress = ca.MX.sym('progress')
        self.vx = ca.MX.sym('vx')
        self.wz = ca.MX.sym('wz')
        self.virtual_speed = ca.MX.sym('virtual_speed')
        self.fl = ca.MX.sym('fl')
        self.fr = ca.MX.sym('fr')

        self.pitch = ca.MX.sym('pitch')

        self.d_dynamics = ca.vertcat( ca.cos(self.pitch) * ca.cos(self.yaw) * self.vx,\
                                      ca.cos(self.pitch) * ca.sin(self.yaw) * self.vx,\
                                      self.wz,\
                                      self.virtual_speed,\
                                      2 * (self.fl + self.fr) / vehicle_param["m"] - ca.sin(self.pitch) * vehicle_param["m"] * self.gz,\
                                      2 * (self.fr - self.fl) * lat_w / ( 2 * vehicle_param["izz"] ) )

#   Find closest trajectory point to mobile robot
class ClosestPoint(ModelParameters, Common):
    def __init__(self, x_ref, y_ref):

        super().__init__()

        """
            Find closest point on the trajectory to robot
        """

        with open(self.baseline_std_dyn_skid) as f:
            solver_param = json.load(f)
            self.specs = solver_param["solver_specs"]
            self.con_pose = solver_param["con_pose"]
            self.con_vel = solver_param["con_vel"]
            self.weights = solver_param["weights"]
        
        #   Work with Leo Rover
        with open(self.vehicle_specs) as f:
            vehicle_param = json.load(f)
            lat_w = vehicle_param["wheelLatSeparation"]
        
        self.opti = ca.Opti()

        self.progress = self.opti.variable()
        self.x = self.opti.parameter()
        self.y = self.opti.parameter()

        J = ( self.x - x_ref(self.progress) )**2 + ( self.y - y_ref(self.progress) )**2

        self.opti.minimize(J)
        
        #self.opti.subject_to( self.opti.bounded( - * ca.pi, self.progress, 2 * ca.pi ) )

        #   Setup solver
        opts = {"ipopt.print_level": 0, "print_time": True, "ipopt.max_iter": 1000, "ipopt.tol": 1e-6, "ipopt.acceptable_tol": 1e-4}
        self.opti.solver("ipopt", opts)
    
    def _findInitialGuess(self, x0, y0, xref, yref):

        """
            Find closest point approximation to be used as initial guess
        """
        
        index = 0

        for progress in np.linspace(0, 2 * math.pi, num=100):
            if(index == 0):
                distance = math.sqrt( math.pow(x0 - float( xref( progress ) ), 2) + math.pow(y0 - float( yref( progress ) ), 2) )
                opt_progress = 0
            
            elif(index > 0):
                new_distance = math.sqrt( math.pow(x0 - float( xref( progress ) ), 2) + math.pow(y0 - float( yref( progress ) ), 2) )
                
                if(new_distance < distance):
                    distance = new_distance
                    opt_progress = np.linspace(0, 2 * math.pi, num=100)[index]

            index += 1
        
        return opt_progress

    def _solve(self, x0, y0, progress_guess):
        
        self.opti.set_initial( self.progress, progress_guess)

        self.opti.set_value( self.x, x0 )
        self.opti.set_value( self.y, y0 )

        #   Solve optimization problem
        sol = self.opti.solve()

        # Extract solution
        progress_sol  = sol.value(self.progress)

        return progress_sol

class Dynamics(ModelParameters, Common):
    def __init__(self, x_ref, y_ref, yaw_ref, sdf=None):
        
        super().__init__()

        with open(self.baseline_std_dyn_skid) as f:
            solver_param = json.load(f)
            self.specs = solver_param["solver_specs"]
            self.con_pose = solver_param["con_pose"]
            self.con_vel = solver_param["con_vel"]
            self.weights = solver_param["weights"]
        
        #   Work with Leo Rover
        with open(self.vehicle_specs) as f:
            vehicle_param = json.load(f)
            lat_w = vehicle_param["wheelLatSeparation"]
        
        fl_prev = ca.MX.sym('fl_prev')
        fr_prev = ca.MX.sym('fr_prev')
        
        self.virtual_speed_ref = ca.MX.sym('virtual_speed_ref')
        self.virtual_speed_prev = ca.MX.sym('virtual_speed_prev')
        
        states = ca.vertcat(self.x, self.y, self.yaw, self.progress, self.vx, self.wz)
        controls = ca.vertcat(self.virtual_speed, self.fl, self.fr)

        Q = ca.diag( ca.vertcat( self.weights["Q_x"],\
                                 self.weights["Q_y"],\
                                 self.weights["Q_yaw"],\
                                 self.weights["Q_virtual_speed"] ) )
        
        Q_t = ca.diag( ca.vertcat( self.weights["Q_x"],\
                                   self.weights["Q_y"],\
                                   self.weights["Q_yaw"] ) )

        err = ca.vertcat( self.x - x_ref(self.progress),\
                          self.y - y_ref(self.progress),\
                          ( ca.cos( self.yaw ) - ca.cos( yaw_ref( self.progress ) ) )**2 + ( ca.sin( self.yaw ) - ca.sin( yaw_ref( self.progress ) ) )**2,\
                          self.virtual_speed - self.virtual_speed_ref )
        
        err_t = ca.vertcat( self.x - x_ref(self.progress),\
                            self.y - y_ref(self.progress),\
                            ( ca.cos( self.yaw ) - ca.cos( yaw_ref( self.progress ) ) )**2 + ( ca.sin( self.yaw ) - ca.sin( yaw_ref( self.progress ) ) )**2 )
        
        obj = err.T @ Q @ err
        obj_t = err_t.T @ Q_t @ err_t

        #   Define RK integrator
        M = 4
        DT = self.specs["Ts"]
        f = ca.Function('f', [states, controls, self.pitch, fl_prev, fr_prev, self.virtual_speed_prev, self.virtual_speed_ref], [self.d_dynamics, obj] )
        f_t = ca.Function('f_t', [states, controls, self.pitch], [self.d_dynamics, obj_t] )

        x0 = ca.MX.sym('_x0', 6)
        u = ca.MX.sym('u', 3)
        _pitch = ca.MX.sym('pitch_param')
        _fl_prev = ca.MX.sym('fl_prev_param')
        _fr_prev = ca.MX.sym('fr_prev_param')
        _v_prev = ca.MX.sym('v_prev_param')
        _v_ref = ca.MX.sym('v_ref_param')

        x = x0
        L = 0

        for _ in range(M):
            k1, k1_q = f(x, u, _pitch, _fl_prev, _fr_prev, _v_prev, _v_ref)
            k2, k2_q = f(x + DT/2 * k1, u, _pitch, _fl_prev, _fr_prev, _v_prev, _v_ref)
            k3, k3_q = f(x + DT/2 * k2, u, _pitch, _fl_prev, _fr_prev, _v_prev, _v_ref)
            k4, k4_q = f(x + DT * k3, u, _pitch, _fl_prev, _fr_prev, _v_prev, _v_ref)
            x = x + DT / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
            L = L + DT / 6 * (k1_q + 2 * k2_q + 2 * k3_q + k4_q)

        F = ca.Function('F', [x0, u, _pitch, _fl_prev, _fr_prev, _v_prev, _v_ref], [x, L],\
                             ['x0', 'u', 'pitch', 'fl_prev', 'fr_prev', 'v_prev', 'v_ref'], ['xf', 'lf'] )
        
        x = x0
        L = 0

        for _ in range(M):
            k1, k1_q = f_t(x, u, _pitch)
            k2, k2_q = f_t(x + DT/2 * k1, u, _pitch)
            k3, k3_q = f_t(x + DT/2 * k2, u, _pitch)
            k4, k4_q = f_t(x + DT * k3, u, _pitch)
            x = x + DT / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
            L = L + DT / 6 * (k1_q + 2 * k2_q + 2 * k3_q + k4_q)

        F_t = ca.Function('F_t', [x0, u, _pitch], [x, L],\
                             ['x0', 'u', 'pitch'], ['xf', 'lf'] )

        #   Discretize problem (Direct multiple shooting)
        w = []
        w0 = []
        lbw = []
        ubw = []
        J = 0
        g = []
        lbg = []
        ubg = []

        Xk = ca.MX.sym('x0', 6)
        w += [Xk]
        lbw += [-ca.inf, -ca.inf, -ca.inf, -ca.inf, self.con_vel["u_lb"], self.con_vel["r_lb"]]
        ubw += [ca.inf, ca.inf, ca.inf, ca.inf, self.con_vel["u_ub"], self.con_vel["r_ub"]]
        w0 += [0, 0, 0, 0, 0, 0]

        p = [_pitch, _fl_prev, _fr_prev, _v_prev, _v_ref]

        #   Formulate the NLP
        for k in range(self.specs["N"] + 1):
            
            #   New NLP variable for the control
            if(k > 0):
                prev_uk = Uk

            Uk = ca.MX.sym('U_' + str(k), 3)
            w += [Uk]
            lbw += [ self.con_vel["virtual_speed_lb"], -1e3, -1e3 ]
            ubw += [ self.con_vel["virtual_speed_ub"], 1e3, 1e3 ]
            w0  += [0, 0, 0]
            
            #   Integrate till the end of the interval
            if(k == 0):
                Fk = F(x0 = Xk, u = Uk, pitch = _pitch, fl_prev = _fl_prev, fr_prev = _fr_prev, v_prev = _v_prev, v_ref = _v_ref )

            elif(k > 0 and k < self.specs["N"]):
                Fk = F(x0 = Xk, u = Uk, pitch = _pitch, fl_prev = prev_uk[1], fr_prev = prev_uk[2], v_prev = prev_uk[0], v_ref = _v_ref )
            
            elif(k == self.specs["N"]):
                Fk = F_t(x0 = Xk, u = Uk, pitch = _pitch)
            
            Xk_end = Fk['xf']
            J += Fk['lf']
            
            #   New NLP variable for state at end of interval
            Xk = ca.MX.sym('X_' + str(k + 1), 6)
            w += [Xk]
            lbw += [-ca.inf, -ca.inf, -ca.inf, -ca.inf, self.con_vel["u_lb"], self.con_vel["r_lb"]]
            ubw += [ca.inf, ca.inf, ca.inf, ca.inf, self.con_vel["u_ub"], self.con_vel["r_ub"]]
            w0  += [0, 0, 0, 0, 0, 0]

            # Add constraints
            if(sdf is not None):
                d2o = sdf( ca.vertcat( -Xk[1], Xk[0] ) )
                g += [ Xk_end - Xk, d2o ]
                lbg += [0, 0, 0, 0, 0, 0, 2e-1]
                ubg += [0, 0, 0, 0, 0, 0, ca.inf]

            else:
                g += [Xk_end - Xk]
                lbg += [0, 0, 0, 0, 0, 0]
                ubg += [0, 0, 0, 0, 0, 0]

        prob = {'f': J, 'x': ca.vertcat(*w), 'g': ca.vertcat(*g), 'p': ca.vertcat(*p)}
        self.solver = ca.nlpsol('solver', self.optSolver, prob, self.optOptions)

    def _solve(self, x0, lbx, ubx, lbg, ubg, p):

        sol = self.solver(x0 = x0, lbx = lbx, ubx = ubx, lbg = lbg, ubg = ubg, p = p)

        return sol
    
    def _getInitialGuess(self, solution):

        horizon = solution["x"].elements()

        guess = horizon[9:] + horizon[-9:]

        return guess
    
    def _getStats(self, solution):

        cost = solution["f"].elements()
        horizon = solution["x"].elements()

        return cost, horizon