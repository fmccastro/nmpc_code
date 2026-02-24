#!/usr/bin/python3.8
import sys

import scipy.linalg
sys.path.insert(0, "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/src")

from classes.all_imports import *
from classes.common_class import *

""" 
    Build and define NMPC optimization problems with the standard baseline formulation

    Heading regulation is included. Refinement of the solution is performed as well.
""" 

#   Optimization problem parameters
class ModelParameters(Common):

    #   Constructor
    def __init__(self):

        super().__init__()

        """with open(self.mpcc_dyn_skidsteering) as f:
            solver_param = json.load(f)
            specs = solver_param["solver_specs"]
            self.con_pose = solver_param["con_pose"]
            self.con_vel = solver_param["con_vel"]
            self.weights = solver_param["weights"]
        """
        
        #   Work with Leo Rover
        with open(self.vehicle_specs) as f:
            vehicle_param = json.load(f)
            lat_w = vehicle_param["wheelLatSeparation"]

        ###     Optimization symbolic variables   ############################

        """
            Mobile robot states
        """

        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        yaw = ca.SX.sym('yaw')
        pitch = ca.SX.sym('pitch')
        vx = ca.SX.sym('vx')
        wz = ca.SX.sym('wz')
        virtual_speed = ca.SX.sym('virtual_speed')
        fl = ca.SX.sym('fl')
        fr = ca.SX.sym('fr')

        d_kinematics = ca.vertcat( ca.cos(pitch) * ca.cos(yaw) * vx,\
                                   ca.cos(pitch) * ca.sin(yaw) * vx,\
                                   wz,\
                                   virtual_speed )
        
        d_dynamics = ca.vertcat( ca.cos(pitch) * ca.cos(yaw) * vx,\
                                 ca.cos(pitch) * ca.sin(yaw) * vx,\
                                 wz,\
                                 virtual_speed,\
                                 2 * (fl + fr) / vehicle_param["m"] - ca.sin(pitch) * vehicle_param["m"] * self.gz,\
                                 2 * (fr - fl) * lat_w / ( 2 * vehicle_param["izz"] ) )
        
        d_dynamics_path = ca.vertcat( ca.cos(pitch) * ca.cos(yaw) * vx,\
                                      ca.cos(pitch) * ca.sin(yaw) * vx,\
                                      wz,\
                                      2 * (fl + fr) / vehicle_param["m"] - ca.sin(pitch) * vehicle_param["m"] * self.gz,\
                                      2 * (fr - fl) * lat_w / ( 2 * vehicle_param["izz"] ) )

        """     Kinematics  """
        self.kinematics = ca.Function( 'kinematics', [x, y, yaw, vx, wz, virtual_speed, pitch], [d_kinematics] )

        """     Dynamics    """
        self.dynamics = ca.Function( 'dynamics', [x, y, yaw, vx, wz, virtual_speed, fl, fr, pitch], [ d_dynamics ] )

        """     Dynamics    """
        self.dynamics_path = ca.Function( 'dynamics', [x, y, yaw, vx, wz, fl, fr, pitch], [ d_dynamics_path ] )

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

#   Parameterized unicycle dynamics model with controlled virtual speed (simple solution) + countoring + lag error
class KinematicsMPCC(ModelParameters, Common):
    def __init__(self, x_ref, y_ref, yaw_ref, sdf=None):
        
        super().__init__()

        """
            Kinematics class to simulate one stage ocp with countoring and lag error contributions
        """

        with open(self.baseline_std_kin_skid) as f:
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

        #   States
        self.x = self.opti.variable( self.specs["N"] + 1 )
        self.y = self.opti.variable( self.specs["N"] + 1 )
        self.yaw = self.opti.variable( self.specs["N"] + 1 )
        self.progress = self.opti.variable( self.specs["N"] + 1 )

        #   Controls
        self.vx = self.opti.variable( self.specs["N"])
        self.wz = self.opti.variable( self.specs["N"] )
        self.virtual_speed = self.opti.variable( self.specs["N"] )

        #   Set initial state parameters
        self.x_0 = self.opti.parameter()
        self.y_0 = self.opti.parameter()
        self.yaw_0 = self.opti.parameter()
        self.progress_0 = self.opti.parameter()
        self.pitch = self.opti.parameter()

        self.virtual_speed_prev = self.opti.parameter()
        self.vx_prev = self.opti.parameter()
        self.wz_prev = self.opti.parameter()

        #   Initial constraints
        self.opti.subject_to(self.x[0] == self.x_0)
        self.opti.subject_to(self.y[0] == self.y_0)
        self.opti.subject_to(self.yaw[0] == self.yaw_0)
        self.opti.subject_to(self.progress[0] == self.progress_0 )
        
        #   RK4 integrator
        for k in range( self.specs["N"] ):
            x_next = ca.vertcat( self.x[k + 1], self.y[k + 1], self.yaw[k + 1], self.progress[k + 1] )
            x_now  = ca.vertcat( self.x[k], self.y[k], self.yaw[k], self.progress[k] )
            
            k1 = self.kinematics(self.x[k],\
                                 self.y[k],\
                                 self.yaw[k],\
                                 self.vx[k],\
                                 self.wz[k],\
                                 self.virtual_speed[k],\
                                 self.pitch )

            k2 = self.kinematics(self.x[k] + self.specs["Ts"] / 2 * k1[0],\
                                 self.y[k] + self.specs["Ts"] / 2 * k1[1],\
                                 self.yaw[k] + self.specs["Ts"] / 2 * k1[2],\
                                 self.vx[k],\
                                 self.wz[k],\
                                 self.virtual_speed[k],\
                                 self.pitch )
            
            k3 = self.kinematics(self.x[k] + self.specs["Ts"] / 2 * k2[0],\
                                    self.y[k] + self.specs["Ts"] / 2 * k2[1],\
                                    self.yaw[k] + self.specs["Ts"] / 2 * k2[2],\
                                    self.vx[k],\
                                    self.wz[k],\
                                    self.virtual_speed[k],\
                                    self.pitch )
            
            k4 = self.kinematics(self.x[k] + self.specs["Ts"] * k3[0],\
                                    self.y[k] + self.specs["Ts"] * k3[1],\
                                    self.yaw[k] + self.specs["Ts"] * k3[2],\
                                    self.vx[k],\
                                    self.wz[k],\
                                    self.virtual_speed[k],\
                                    self.pitch )

            x_next_pred = x_now + (self.specs["Ts"] / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
            
            self.opti.subject_to(x_next == x_next_pred)

        #   Cost function
        J = 0
        
        for k in range(self.specs["N"] + 1):

            el = -ca.cos( yaw_ref( self.progress[k] ) ) * ( self.x[k] - x_ref( self.progress[k] ) ) - ca.sin( yaw_ref( self.progress[k] ) ) * ( self.y[k] - y_ref( self.progress[k] ) )
            ec = ca.sin( yaw_ref( self.progress[k] ) ) * ( self.x[k] - x_ref( self.progress[k] ) ) - ca.cos( yaw_ref( self.progress[k] ) ) * ( self.y[k] - y_ref( self.progress[k] ) )

            delta_q = 1 - ca.exp( -3 * ( sdf( ca.vertcat(-self.y[k], self.x[k]) ) - 0.2 ) )
            delta_sdf = ca.exp( -3 * ( sdf( ca.vertcat(-self.y[k], self.x[k]) ) - 0.2 ) )

            if( k == 0 ):
                err = ca.vertcat( el,\
                                  ec,\
                                  ( ca.cos( self.yaw[k] ) - ca.cos( yaw_ref( self.progress[k] ) ) )**2 + ( ca.sin( self.yaw[k] ) - ca.sin( yaw_ref( self.progress[k] ) ) )**2,\
                                  delta_sdf,\
                                  self.vx[k] - self.vx_prev,\
                                  self.wz[k] - self.wz_prev,\
                                  self.virtual_speed[k] - 5e-1,\
                                  self.virtual_speed[k] - self.virtual_speed_prev )
                
                J += err.T @ ca.diag( ca.vertcat( self.weights["Q_x"],\
                                                  delta_q * self.weights["Q_y"],\
                                                  delta_q * self.weights["Q_yaw"],\
                                                  1.0,\
                                                  self.weights["Q_u_rate"],\
                                                  self.weights["Q_r_rate"],\
                                                  self.weights["Q_virtual_speed"],\
                                                  self.weights["Q_virtual_speed_rate"] ) ) @ err

            elif( k < self.specs["N"] ):
                err = ca.vertcat( el,\
                                  ec,\
                                  ( ca.cos( self.yaw[k] ) - ca.cos( yaw_ref( self.progress[k] ) ) )**2 + ( ca.sin( self.yaw[k] ) - ca.sin( yaw_ref( self.progress[k] ) ) )**2,\
                                  delta_sdf,\
                                  self.vx[k] - self.vx[k - 1],\
                                  self.wz[k] - self.wz[k - 1],\
                                  self.virtual_speed[k] - 5e-1,\
                                  self.virtual_speed[k] - self.virtual_speed[k - 1] )
                
                J += err.T @ ca.diag( ca.vertcat( self.weights["Q_x"],\
                                                  delta_q * self.weights["Q_y"],\
                                                  delta_q * self.weights["Q_yaw"],\
                                                  1.0,\
                                                  self.weights["Q_u_rate"],\
                                                  self.weights["Q_r_rate"],\
                                                  self.weights["Q_virtual_speed"],\
                                                  self.weights["Q_virtual_speed_rate"] ) ) @ err

            else:
                err = ca.vertcat( el,\
                                  ec,\
                                  ( ca.cos( self.yaw[k] ) - ca.cos( yaw_ref( self.progress[k] ) ) )**2 + ( ca.sin( self.yaw[k] ) - ca.sin( yaw_ref( self.progress[k] ) ) )**2,\
                                  delta_sdf )

                J += err.T @ ca.diag( ca.vertcat( self.weights["Q_x_t"],\
                                                  self.weights["Q_y_t"],\
                                                  self.weights["Q_yaw_t"],\
                                                  1.0 ) ) @ err
        
        self.opti.minimize(J)

        #   Constraints
        self.opti.subject_to( self.opti.bounded( self.con_vel["u_lb"], self.vx, self.con_vel["u_ub"] ) )
        self.opti.subject_to( self.opti.bounded( self.con_vel["r_lb"], self.wz, self.con_vel["r_ub"] ) )
        self.opti.subject_to( self.opti.bounded( self.con_vel["virtual_speed_lb"], self.virtual_speed, self.con_vel["virtual_speed_ub"] ) )

        for k in range(self.specs["N"] + 1):
            self.opti.subject_to( sdf( ca.vertcat(-self.y[k], self.x[k]) ) > 2e-1 )

        #   Setup solver
        opts = {"ipopt.print_level": 0, "print_time": True, "ipopt.max_iter": 1000, "ipopt.tol": 1e-6, "ipopt.acceptable_tol": 1e-4}
        self.opti.solver("ipopt", opts)

    def _setInitialState(self, initialState, parameters):
        
        #   Set initial state
        self.opti.set_value( self.x_0, initialState[0] )
        self.opti.set_value( self.y_0, initialState[1] )
        self.opti.set_value( self.yaw_0, initialState[2] )
        self.opti.set_value( self.progress_0, initialState[3] )
        self.opti.set_value( self.pitch, parameters["pitch"] )
        self.opti.set_value( self.vx_prev, parameters["vx_prev"] )
        self.opti.set_value( self.wz_prev, parameters["wz_prev"] )
        self.opti.set_value( self.virtual_speed_prev, parameters["virtual_speed_prev"] )

    def _setInitialGuess(self, lastSol):
        
        self.opti.set_initial( self.x, np.append( lastSol[0][1:], lastSol[0][-1] ) )
        self.opti.set_initial( self.y, np.append( lastSol[1][1:], lastSol[1][-1] ) )
        self.opti.set_initial( self.yaw, np.append( lastSol[2][1:], lastSol[2][-1] ) )
        self.opti.set_initial( self.progress, np.append( lastSol[3][1:], lastSol[3][-1] ) )
        self.opti.set_initial( self.vx, np.append( lastSol[4][1:], lastSol[4][-1] ) )
        self.opti.set_initial( self.wz, np.append( lastSol[5][1:], lastSol[5][-1] ) )
        self.opti.set_initial( self.virtual_speed, np.append( lastSol[6][1:], lastSol[6][-1] ) )

    def _getStats(self):
        return self.opti.stats()
    
    def _solve(self, initialState, prevSol, parameters):
        
        if( prevSol is None ):
            pass
        
        else:
            self._setInitialGuess(prevSol)

        self._setInitialState(initialState, parameters)

        #   Solve optimization problem
        sol = self.opti.solve()

        # Extract solution
        x_sol  = sol.value(self.x)
        y_sol  = sol.value(self.y)
        yaw_sol = sol.value(self.yaw)
        progress_sol = sol.value(self.progress)
        vx_sol = sol.value(self.vx)
        wz_sol = sol.value(self.wz)
        virtual_speed_sol = sol.value(self.virtual_speed)

        return [x_sol, y_sol, yaw_sol, progress_sol, vx_sol, wz_sol, virtual_speed_sol]

#   Parameterized unicycle dynamics model with controlled virtual speed (simple solution)
class Dynamics(ModelParameters, Common):
    def __init__(self, x_ref, y_ref, yaw_ref, sdf=None):
        
        super().__init__()

        """
            Dynamics class to simulate one stage ocp without heading regulation 
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

        #   States
        self.x = self.opti.variable( self.specs["N"] + 1 )
        self.y = self.opti.variable( self.specs["N"] + 1 )
        self.yaw = self.opti.variable( self.specs["N"] + 1 )
        self.progress = self.opti.variable( self.specs["N"] + 1 )
        self.vx = self.opti.variable( self.specs["N"] + 1 )
        self.wz = self.opti.variable( self.specs["N"] + 1 )

        #   Controls
        self.fl = self.opti.variable( self.specs["N"] )
        self.fr = self.opti.variable( self.specs["N"] )
        self.virtual_speed = self.opti.variable( self.specs["N"] )

        #   Set initial state parameters
        self.x_0 = self.opti.parameter()
        self.y_0 = self.opti.parameter()
        self.yaw_0 = self.opti.parameter()
        self.progress_0 = self.opti.parameter()
        self.vx_0 = self.opti.parameter()
        self.wz_0 = self.opti.parameter()
        self.pitch = self.opti.parameter()
        self.roll = self.opti.parameter()
        self.friction = self.opti.parameter()

        self.virtual_speed_prev = self.opti.parameter()
        self.fl_prev = self.opti.parameter()
        self.fr_prev = self.opti.parameter()

        self.fl_min = self.opti.parameter()
        self.fl_max = self.opti.parameter()
        self.fr_min = self.opti.parameter()
        self.fr_max = self.opti.parameter()

        #   Initial constraints
        self.opti.subject_to(self.x[0] == self.x_0)
        self.opti.subject_to(self.y[0] == self.y_0)
        self.opti.subject_to(self.yaw[0] == self.yaw_0)
        self.opti.subject_to(self.progress[0] == self.progress_0 )
        self.opti.subject_to(self.vx[0] == self.vx_0)
        self.opti.subject_to(self.wz[0] == self.wz_0)
        
        #   RK4 integrator
        for k in range( self.specs["N"] ):
            x_next = ca.vertcat( self.x[k + 1], self.y[k + 1], self.yaw[k + 1], self.progress[k + 1], self.vx[k + 1], self.wz[k + 1] )
            x_now  = ca.vertcat( self.x[k], self.y[k], self.yaw[k], self.progress[k], self.vx[k], self.wz[k] )
            
            k1 = self.dynamics(self.x[k],\
                               self.y[k],\
                               self.yaw[k],\
                               self.vx[k],\
                               self.wz[k],\
                               self.virtual_speed[k],\
                               self.fl[k],\
                               self.fr[k],\
                               self.pitch )

            k2 = self.dynamics(self.x[k] + self.specs["Ts"] / 2 * k1[0],\
                               self.y[k] + self.specs["Ts"] / 2 * k1[1],\
                               self.yaw[k] + self.specs["Ts"] / 2 * k1[2],\
                               self.vx[k] + self.specs["Ts"] / 2 * k1[4],\
                               self.wz[k] + self.specs["Ts"] / 2 * k1[5],\
                               self.virtual_speed[k],\
                               self.fl[k],\
                               self.fr[k],\
                               self.pitch )
            
            k3 = self.dynamics(self.x[k] + self.specs["Ts"] / 2 * k2[0],\
                               self.y[k] + self.specs["Ts"] / 2 * k2[1],\
                               self.yaw[k] + self.specs["Ts"] / 2 * k2[2],\
                               self.vx[k] + self.specs["Ts"] / 2 * k2[4],\
                               self.wz[k] + self.specs["Ts"] / 2 * k2[5],\
                               self.virtual_speed[k],\
                               self.fl[k],\
                               self.fr[k],\
                               self.pitch )
            
            k4 = self.dynamics(self.x[k] + self.specs["Ts"] * k3[0],\
                               self.y[k] + self.specs["Ts"] * k3[1],\
                               self.yaw[k] + self.specs["Ts"] * k3[2],\
                               self.vx[k] + self.specs["Ts"] * k3[4],\
                               self.wz[k] + self.specs["Ts"] * k3[5],\
                               self.virtual_speed[k],\
                               self.fl[k],\
                               self.fr[k],\
                               self.pitch )

            x_next_pred = x_now + (self.specs["Ts"] / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
            
            self.opti.subject_to(x_next == x_next_pred)

        #   Cost function
        J = 0
        
        for k in range(self.specs["N"] + 1):

            if( k == 0 ):
                err = ca.vertcat( self.x[k] - x_ref( self.progress[k] ),\
                                  self.y[k] - y_ref( self.progress[k] ),\
                                  ( ca.cos( self.yaw[k] ) - ca.cos( yaw_ref( self.progress[k] ) ) )**2 + ( ca.sin( self.yaw[k] ) - ca.sin( yaw_ref( self.progress[k] ) ) )**2,\
                                  self.fl[k] - self.fl_prev,\
                                  self.fr[k] - self.fr_prev,\
                                  self.virtual_speed[k] - 5e-1,\
                                  self.virtual_speed[k] - self.virtual_speed_prev )
                
                J += err.T @ ca.diag( ca.vertcat( self.weights["Q_x"],\
                                                  self.weights["Q_y"],\
                                                  self.weights["Q_yaw"],\
                                                  self.weights["Q_fl_rate"],\
                                                  self.weights["Q_fr_rate"],\
                                                  self.weights["Q_virtual_speed"],\
                                                  self.weights["Q_virtual_speed_rate"] ) ) @ err

            if( k < self.specs["N"] ):
                err = ca.vertcat( self.x[k] - x_ref( self.progress[k] ),\
                                  self.y[k] - y_ref( self.progress[k] ),\
                                  ( ca.cos( self.yaw[k] ) - ca.cos( yaw_ref( self.progress[k] ) ) )**2 + ( ca.sin( self.yaw[k] ) - ca.sin( yaw_ref( self.progress[k] ) ) )**2,\
                                  self.fl[k] - self.fl[k - 1],\
                                  self.fr[k] - self.fr[k - 1],\
                                  self.virtual_speed[k] - 5e-1,\
                                  self.virtual_speed[k] - self.virtual_speed[k - 1] )
                
                J += err.T @ ca.diag( ca.vertcat( self.weights["Q_x"],\
                                                  self.weights["Q_y"],\
                                                  self.weights["Q_yaw"],\
                                                  self.weights["Q_fl_rate"],\
                                                  self.weights["Q_fr_rate"],\
                                                  self.weights["Q_virtual_speed"],\
                                                  self.weights["Q_virtual_speed_rate"] ) ) @ err

            else:
                err = ca.vertcat( self.x[k] - x_ref( self.progress[k] ),\
                                  self.y[k] - y_ref( self.progress[k] ),\
                                  ( ca.cos( self.yaw[k] ) - ca.cos( yaw_ref( self.progress[k] ) ) )**2 + ( ca.sin( self.yaw[k] ) - ca.sin( yaw_ref( self.progress[k] ) ) )**2 )

                J += err.T @ ca.diag( ca.vertcat( self.weights["Q_x_t"], self.weights["Q_y_t"], self.weights["Q_yaw_t"] ) ) @ err
        
        self.opti.minimize(J)

        #   Constraints
        self.opti.subject_to( self.opti.bounded( self.con_vel["u_lb"], self.vx, self.con_vel["u_ub"] ) )
        self.opti.subject_to( self.opti.bounded( self.con_vel["r_lb"], self.wz, self.con_vel["r_ub"] ) )
        self.opti.subject_to( self.opti.bounded( self.con_vel["virtual_speed_lb"], self.virtual_speed, self.con_vel["virtual_speed_ub"] ) )
        self.opti.subject_to( self.opti.bounded( self.fl_min, self.fl, self.fl_max ) )
        self.opti.subject_to( self.opti.bounded( self.fr_min, self.fr, self.fr_max ) )
        
        if( sdf is not None ):
            for k in range(self.specs["N"] + 1):
                self.opti.subject_to( sdf( ca.vertcat(-self.y[k], self.x[k]) ) > 2e-1 )

        #   Setup solver
        opts = {"ipopt.print_level": 0, "print_time": True, "ipopt.max_iter": 1000, "ipopt.tol": 1e-6, "ipopt.acceptable_tol": 1e-4}
        self.opti.solver("ipopt", opts)

    def _setInitialState(self, initialState, parameters):
        
        #   Set initial state
        self.opti.set_value( self.x_0, initialState[0] )
        self.opti.set_value( self.y_0, initialState[1] )
        self.opti.set_value( self.yaw_0, initialState[2] )
        self.opti.set_value( self.progress_0, initialState[3] )
        self.opti.set_value( self.vx_0, initialState[4] )
        self.opti.set_value( self.wz_0, initialState[5] )
        self.opti.set_value( self.friction, parameters["friction"] )
        self.opti.set_value( self.pitch, parameters["pitch"] )
        self.opti.set_value( self.roll, parameters["roll"] )
        self.opti.set_value( self.fl_prev, parameters["fl_prev"] )
        self.opti.set_value( self.fr_prev, parameters["fr_prev"] )
        self.opti.set_value( self.virtual_speed_prev, parameters["virtual_speed_prev"] )
        
        self.opti.set_value( self.fl_min, parameters["fl_min"] )
        self.opti.set_value( self.fl_max, parameters["fl_max"] )
        self.opti.set_value( self.fr_min, parameters["fr_min"] )
        self.opti.set_value( self.fr_max, parameters["fr_max"] )

    def _setInitialGuess(self, lastSol):
        
        self.opti.set_initial( self.x, np.append( lastSol[0][1:], lastSol[0][-1] ) )
        self.opti.set_initial( self.y, np.append( lastSol[1][1:], lastSol[1][-1] ) )
        self.opti.set_initial( self.yaw, np.append( lastSol[2][1:], lastSol[2][-1] ) )
        self.opti.set_initial( self.progress, np.append( lastSol[3][1:], lastSol[3][-1] ) )
        self.opti.set_initial( self.vx, np.append( lastSol[4][1:], lastSol[4][-1] ) )
        self.opti.set_initial( self.wz, np.append( lastSol[5][1:], lastSol[5][-1] ) )
        self.opti.set_initial( self.fl, np.append( lastSol[6][1:], lastSol[6][-1] ) )
        self.opti.set_initial( self.fr, np.append( lastSol[7][1:], lastSol[7][-1] ) )
        self.opti.set_initial( self.virtual_speed, np.append( lastSol[8][1:], lastSol[8][-1] ) )

    def _getStats(self):
        return self.opti.stats()
    
    def _solve(self, initialState, prevSol, parameters):
        
        if( prevSol is None ):
            pass
        
        else:
            self._setInitialGuess(prevSol)

        self._setInitialState(initialState, parameters)

        #   Solve optimization problem
        sol = self.opti.solve()

        # Extract solution
        x_sol  = sol.value(self.x)
        y_sol  = sol.value(self.y)
        yaw_sol = sol.value(self.yaw)
        progress_sol = sol.value(self.progress)
        vx_sol = sol.value(self.vx)
        wz_sol = sol.value(self.wz)
        fl_sol = sol.value(self.fl)
        fr_sol = sol.value(self.fr)
        virtual_speed_sol = sol.value(self.virtual_speed)

        return [x_sol, y_sol, yaw_sol, progress_sol, vx_sol, wz_sol, fl_sol, fr_sol, virtual_speed_sol]

#   Parameterized unicycle dynamics model with controlled virtual speed outside the ocp (simple solution)
class DynamicsPath(ModelParameters, Common):
    def __init__(self, sdf=None):
        
        super().__init__()

        """
            Dynamics class to simulate one stage ocp without heading regulation 
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

        #   States
        self.x = self.opti.variable( self.specs["N"] + 1 )
        self.y = self.opti.variable( self.specs["N"] + 1 )
        self.yaw = self.opti.variable( self.specs["N"] + 1 )
        self.progress = self.opti.variable( self.specs["N"] + 1 )
        self.vx = self.opti.variable( self.specs["N"] + 1 )
        self.wz = self.opti.variable( self.specs["N"] + 1 )

        #   Controls
        self.fl = self.opti.variable( self.specs["N"] )
        self.fr = self.opti.variable( self.specs["N"] )

        #   Set initial state parameters
        self.x_0 = self.opti.parameter()
        self.y_0 = self.opti.parameter()
        self.yaw_0 = self.opti.parameter()
        self.vx_0 = self.opti.parameter()
        self.wz_0 = self.opti.parameter()
        self.pitch = self.opti.parameter()
        self.roll = self.opti.parameter()
        self.friction = self.opti.parameter()

        self.fl_prev = self.opti.parameter()
        self.fr_prev = self.opti.parameter()

        self.fl_min = self.opti.parameter()
        self.fl_max = self.opti.parameter()
        self.fr_min = self.opti.parameter()
        self.fr_max = self.opti.parameter()

        self.x_ref = self.opti.parameter(self.specs["N"] + 1)
        self.y_ref = self.opti.parameter(self.specs["N"] + 1)
        self.yaw_ref = self.opti.parameter(self.specs["N"] + 1)

        #   Initial constraints
        self.opti.subject_to(self.x[0] == self.x_0)
        self.opti.subject_to(self.y[0] == self.y_0)
        self.opti.subject_to(self.yaw[0] == self.yaw_0)
        self.opti.subject_to(self.vx[0] == self.vx_0)
        self.opti.subject_to(self.wz[0] == self.wz_0)
        
        #   RK4 integrator
        for k in range( self.specs["N"] ):
            x_next = ca.vertcat( self.x[k + 1], self.y[k + 1], self.yaw[k + 1], self.vx[k + 1], self.wz[k + 1] )
            x_now  = ca.vertcat( self.x[k], self.y[k], self.yaw[k], self.vx[k], self.wz[k] )
            
            k1 = self.dynamics_path(self.x[k],\
                                    self.y[k],\
                                    self.yaw[k],\
                                    self.vx[k],\
                                    self.wz[k],\
                                    self.fl[k],\
                                    self.fr[k],\
                                    self.pitch )

            k2 = self.dynamics_path(self.x[k] + self.specs["Ts"] / 2 * k1[0],\
                                    self.y[k] + self.specs["Ts"] / 2 * k1[1],\
                                    self.yaw[k] + self.specs["Ts"] / 2 * k1[2],\
                                    self.vx[k] + self.specs["Ts"] / 2 * k1[3],\
                                    self.wz[k] + self.specs["Ts"] / 2 * k1[4],\
                                    self.fl[k],\
                                    self.fr[k],\
                                    self.pitch )
            
            k3 = self.dynamics_path(self.x[k] + self.specs["Ts"] / 2 * k2[0],\
                                    self.y[k] + self.specs["Ts"] / 2 * k2[1],\
                                    self.yaw[k] + self.specs["Ts"] / 2 * k2[2],\
                                    self.vx[k] + self.specs["Ts"] / 2 * k2[3],\
                                    self.wz[k] + self.specs["Ts"] / 2 * k2[4],\
                                    self.fl[k],\
                                    self.fr[k],\
                                    self.pitch )
            
            k4 = self.dynamics_path(self.x[k] + self.specs["Ts"] * k3[0],\
                                    self.y[k] + self.specs["Ts"] * k3[1],\
                                    self.yaw[k] + self.specs["Ts"] * k3[2],\
                                    self.vx[k] + self.specs["Ts"] * k3[3],\
                                    self.wz[k] + self.specs["Ts"] * k3[4],\
                                    self.fl[k],\
                                    self.fr[k],\
                                    self.pitch )

            x_next_pred = x_now + (self.specs["Ts"] / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
            
            self.opti.subject_to(x_next == x_next_pred)

        #   Cost function
        J = 0
        
        for k in range(self.specs["N"] + 1):

            if( k == 0 ):
                err = ca.vertcat( self.x[k] - self.x_ref[k],\
                                  self.y[k] - self.y_ref[k],\
                                  ( ca.cos( self.yaw[k] ) - ca.cos( self.yaw_ref[k] ) )**2 + ( ca.sin( self.yaw[k] ) - ca.sin( self.yaw_ref[k] ) )**2,\
                                  self.fl[k] - self.fl_prev,\
                                  self.fr[k] - self.fr_prev )
                
                J += err.T @ ca.diag( ca.vertcat( self.weights["Q_x"],\
                                                  self.weights["Q_y"],\
                                                  self.weights["Q_yaw"],\
                                                  self.weights["Q_fl_rate"],\
                                                  self.weights["Q_fr_rate"] ) ) @ err

            if( k < self.specs["N"] ):
                err = ca.vertcat( self.x[k] - self.x_ref[k],\
                                  self.y[k] - self.y_ref[k],\
                                  ( ca.cos( self.yaw[k] ) - ca.cos( self.yaw_ref[k] ) )**2 + ( ca.sin( self.yaw[k] ) - ca.sin( self.yaw_ref[k] ) )**2,\
                                  self.fl[k] - self.fl[k - 1],\
                                  self.fr[k] - self.fr[k - 1] )
                
                J += err.T @ ca.diag( ca.vertcat( self.weights["Q_x"],\
                                                  self.weights["Q_y"],\
                                                  self.weights["Q_yaw"],\
                                                  self.weights["Q_fl_rate"],\
                                                  self.weights["Q_fr_rate"] ) ) @ err

            else:
                err = ca.vertcat( self.x[k] - self.x_ref[k],\
                                  self.y[k] - self.y_ref[k],\
                                  ( ca.cos( self.yaw[k] ) - ca.cos( self.yaw_ref[k] ) )**2 + ( ca.sin( self.yaw[k] ) - ca.sin( self.yaw_ref[k] ) )**2 )

                J += err.T @ ca.diag( ca.vertcat( self.weights["Q_x_t"], self.weights["Q_y_t"], self.weights["Q_yaw_t"] ) ) @ err

        self.opti.minimize(J)

        #   Constraints
        self.opti.subject_to( self.opti.bounded( self.con_vel["u_lb"], self.vx, self.con_vel["u_ub"] ) )
        self.opti.subject_to( self.opti.bounded( self.con_vel["r_lb"], self.wz, self.con_vel["r_ub"] ) )
        self.opti.subject_to( self.opti.bounded( self.fl_min, self.fl, self.fl_max ) )
        self.opti.subject_to( self.opti.bounded( self.fr_min, self.fr, self.fr_max ) )

        for k in range(self.specs["N"] + 1):
            self.opti.subject_to( sdf( ca.vertcat(-self.y[k], self.x[k]) ) > 2e-1 )

        #   Setup solver
        opts = {"ipopt.print_level": 0, "print_time": True, "ipopt.max_iter": 1000, "ipopt.tol": 1e-6, "ipopt.acceptable_tol": 1e-4}
        self.opti.solver("ipopt", opts)

    def _setInitialState(self, initialState, parameters):
        
        #   Set initial state
        self.opti.set_value( self.x_0, initialState[0] )
        self.opti.set_value( self.y_0, initialState[1] )
        self.opti.set_value( self.yaw_0, initialState[2] )
        self.opti.set_value( self.vx_0, initialState[3] )
        self.opti.set_value( self.wz_0, initialState[4] )
        self.opti.set_value( self.friction, parameters["friction"] )
        self.opti.set_value( self.pitch, parameters["pitch"] )
        self.opti.set_value( self.roll, parameters["roll"] )
        self.opti.set_value( self.fl_prev, parameters["fl_prev"] )
        self.opti.set_value( self.fr_prev, parameters["fr_prev"] )
        
        self.opti.set_value( self.fl_min, parameters["fl_min"] )
        self.opti.set_value( self.fl_max, parameters["fl_max"] )
        self.opti.set_value( self.fr_min, parameters["fr_min"] )
        self.opti.set_value( self.fr_max, parameters["fr_max"] )
        
    def _setInitialGuess(self, lastSol):
        
        self.opti.set_initial( self.x, np.append( lastSol[0][1:], lastSol[0][-1] ) )
        self.opti.set_initial( self.y, np.append( lastSol[1][1:], lastSol[1][-1] ) )
        self.opti.set_initial( self.yaw, np.append( lastSol[2][1:], lastSol[2][-1] ) )
        self.opti.set_initial( self.vx, np.append( lastSol[3][1:], lastSol[3][-1] ) )
        self.opti.set_initial( self.wz, np.append( lastSol[4][1:], lastSol[4][-1] ) )
        self.opti.set_initial( self.fl, np.append( lastSol[5][1:], lastSol[5][-1] ) )
        self.opti.set_initial( self.fr, np.append( lastSol[6][1:], lastSol[6][-1] ) )
    
    def _setReference(self, x_ref, y_ref, yaw_ref):

        for k in range(self.specs["N"] + 1):
            self.opti.set_value( self.x_ref[k], x_ref[k] )
            self.opti.set_value( self.y_ref[k], y_ref[k] )
            self.opti.set_value( self.yaw_ref[k], yaw_ref[k] )

    def _getStats(self):
        return self.opti.stats()
    
    def _solve(self, initialState, prevSol, parameters):
        
        if( prevSol is None ):
            pass
        
        else:
            self._setInitialGuess(prevSol)

        self._setInitialState(initialState, parameters)

        self._setReference(parameters["x_ref"], parameters["y_ref"], parameters["yaw_ref"])
        
        #   Solve optimization problem
        sol = self.opti.solve()

        # Extract solution
        x_sol  = sol.value(self.x)
        y_sol  = sol.value(self.y)
        yaw_sol = sol.value(self.yaw)
        vx_sol = sol.value(self.vx)
        wz_sol = sol.value(self.wz)
        fl_sol = sol.value(self.fl)
        fr_sol = sol.value(self.fr)

        return [x_sol, y_sol, yaw_sol, vx_sol, wz_sol, fl_sol, fr_sol]
    
    def _v2error(self, x):

        #   v - error
        L = 5e-1
        xm = 2.0
        k = 5

        return L / ( 1 + math.exp( k * (x - xm) ) )
    
    def _v2do(self, x):

        #   v - distance to obstacle
        L_min = 4.9e-1
        L_max = 5e-1
        xm = 1e0
        k = 10

        return (L_max - L_min) / ( 1 + math.exp( -k * (x - xm) ) ) + L_min
    
    def _v2k(self, x):

        #   v - curvature
        L_max = 5e-1
        xm = 1e1
        k = 0.1

        return L_max / ( 1 + math.exp( k * (x - xm) ) )
    
    def _getVirtualSpeed(self, error, d2o, curvature):

        v1 = self._v2error(error)
        v2 = self._v2do(d2o)
        v3 = self._v2k(curvature)

        return min(v1, v2, v3)

#   Parameterized unicycle dynamics model with controlled virtual speed (simple solution)
class DynamicsEnergy(ModelParameters, Common):
    def __init__(self, x_ref, y_ref, yaw_ref, sdf=None):
        
        super().__init__()

        """
            Dynamics class to simulate one stage ocp without heading regulation with power or force optimization
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

        #   States
        self.x = self.opti.variable( self.specs["N"] + 1 )
        self.y = self.opti.variable( self.specs["N"] + 1 )
        self.yaw = self.opti.variable( self.specs["N"] + 1 )
        self.progress = self.opti.variable( self.specs["N"] + 1 )
        self.vx = self.opti.variable( self.specs["N"] + 1 )
        self.wz = self.opti.variable( self.specs["N"] + 1 )

        #   Controls
        self.fl = self.opti.variable( self.specs["N"] )
        self.fr = self.opti.variable( self.specs["N"] )
        self.virtual_speed = self.opti.variable( self.specs["N"] )

        #   Set initial state parameters
        self.x_0 = self.opti.parameter()
        self.y_0 = self.opti.parameter()
        self.yaw_0 = self.opti.parameter()
        self.progress_0 = self.opti.parameter()
        self.vx_0 = self.opti.parameter()
        self.wz_0 = self.opti.parameter()
        self.pitch = self.opti.parameter()
        self.roll = self.opti.parameter()
        self.friction = self.opti.parameter()

        self.virtual_speed_prev = self.opti.parameter()
        self.fl_prev = self.opti.parameter()
        self.fr_prev = self.opti.parameter()

        self.fl_min = self.opti.parameter()
        self.fl_max = self.opti.parameter()
        self.fr_min = self.opti.parameter()
        self.fr_max = self.opti.parameter()

        #   Initial constraints
        self.opti.subject_to(self.x[0] == self.x_0)
        self.opti.subject_to(self.y[0] == self.y_0)
        self.opti.subject_to(self.yaw[0] == self.yaw_0)
        self.opti.subject_to(self.progress[0] == self.progress_0 )
        self.opti.subject_to(self.vx[0] == self.vx_0)
        self.opti.subject_to(self.wz[0] == self.wz_0)
        
        #   RK4 integrator
        for k in range( self.specs["N"] ):
            x_next = ca.vertcat( self.x[k + 1], self.y[k + 1], self.yaw[k + 1], self.progress[k + 1], self.vx[k + 1], self.wz[k + 1] )
            x_now  = ca.vertcat( self.x[k], self.y[k], self.yaw[k], self.progress[k], self.vx[k], self.wz[k] )
            
            k1 = self.dynamics(self.x[k],\
                               self.y[k],\
                               self.yaw[k],\
                               self.vx[k],\
                               self.wz[k],\
                               self.virtual_speed[k],\
                               self.fl[k],\
                               self.fr[k],\
                               self.pitch )

            k2 = self.dynamics(self.x[k] + self.specs["Ts"] / 2 * k1[0],\
                               self.y[k] + self.specs["Ts"] / 2 * k1[1],\
                               self.yaw[k] + self.specs["Ts"] / 2 * k1[2],\
                               self.vx[k] + self.specs["Ts"] / 2 * k1[4],\
                               self.wz[k] + self.specs["Ts"] / 2 * k1[5],\
                               self.virtual_speed[k],\
                               self.fl[k],\
                               self.fr[k],\
                               self.pitch )
            
            k3 = self.dynamics(self.x[k] + self.specs["Ts"] / 2 * k2[0],\
                               self.y[k] + self.specs["Ts"] / 2 * k2[1],\
                               self.yaw[k] + self.specs["Ts"] / 2 * k2[2],\
                               self.vx[k] + self.specs["Ts"] / 2 * k2[4],\
                               self.wz[k] + self.specs["Ts"] / 2 * k2[5],\
                               self.virtual_speed[k],\
                               self.fl[k],\
                               self.fr[k],\
                               self.pitch )
            
            k4 = self.dynamics(self.x[k] + self.specs["Ts"] * k3[0],\
                               self.y[k] + self.specs["Ts"] * k3[1],\
                               self.yaw[k] + self.specs["Ts"] * k3[2],\
                               self.vx[k] + self.specs["Ts"] * k3[4],\
                               self.wz[k] + self.specs["Ts"] * k3[5],\
                               self.virtual_speed[k],\
                               self.fl[k],\
                               self.fr[k],\
                               self.pitch )

            x_next_pred = x_now + (self.specs["Ts"] / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
            
            self.opti.subject_to(x_next == x_next_pred)

        #   Cost function
        J = 0
        
        for k in range(self.specs["N"] + 1):

            if( k == 0 ):
                err = ca.vertcat( self.x[k] - x_ref( self.progress[k] ),\
                                  self.y[k] - y_ref( self.progress[k] ),\
                                  ( ca.cos( self.yaw[k] ) - ca.cos( yaw_ref( self.progress[k] ) ) )**2 + ( ca.sin( self.yaw[k] ) - ca.sin( yaw_ref( self.progress[k] ) ) )**2,\
                                  self.fl[k],\
                                  self.fr[k],\
                                  self.fl[k] - self.fl_prev,\
                                  self.fr[k] - self.fr_prev,\
                                  self.virtual_speed[k] - 5e-1,\
                                  self.virtual_speed[k] - self.virtual_speed_prev )
                
                J += err.T @ ca.diag( ca.vertcat( self.weights["Q_x"],\
                                                  self.weights["Q_y"],\
                                                  self.weights["Q_yaw"],\
                                                  self.weights["Q_fl"],\
                                                  self.weights["Q_fr"],\
                                                  self.weights["Q_fl_rate"],\
                                                  self.weights["Q_fr_rate"],\
                                                  self.weights["Q_virtual_speed"],\
                                                  self.weights["Q_virtual_speed_rate"] ) ) @ err

            if( k < self.specs["N"] ):
                err = ca.vertcat( self.x[k] - x_ref( self.progress[k] ),\
                                  self.y[k] - y_ref( self.progress[k] ),\
                                  ( ca.cos( self.yaw[k] ) - ca.cos( yaw_ref( self.progress[k] ) ) )**2 + ( ca.sin( self.yaw[k] ) - ca.sin( yaw_ref( self.progress[k] ) ) )**2,\
                                  self.fl[k],\
                                  self.fr[k],\
                                  self.fl[k] - self.fl[k - 1],\
                                  self.fr[k] - self.fr[k - 1],\
                                  self.virtual_speed[k] - 5e-1,\
                                  self.virtual_speed[k] - self.virtual_speed[k - 1] )
                
                J += err.T @ ca.diag( ca.vertcat( self.weights["Q_x"],\
                                                  self.weights["Q_y"],\
                                                  self.weights["Q_yaw"],\
                                                  self.weights["Q_fl"],\
                                                  self.weights["Q_fr"],\
                                                  self.weights["Q_fl_rate"],\
                                                  self.weights["Q_fr_rate"],\
                                                  self.weights["Q_virtual_speed"],\
                                                  self.weights["Q_virtual_speed_rate"] ) ) @ err

            else:
                err = ca.vertcat( self.x[k] - x_ref( self.progress[k] ),\
                                  self.y[k] - y_ref( self.progress[k] ),\
                                  ( ca.cos( self.yaw[k] ) - ca.cos( yaw_ref( self.progress[k] ) ) )**2 + ( ca.sin( self.yaw[k] ) - ca.sin( yaw_ref( self.progress[k] ) ) )**2 )

                J += err.T @ ca.diag( ca.vertcat( self.weights["Q_x_t"], self.weights["Q_y_t"], self.weights["Q_yaw_t"] ) ) @ err
        
        self.opti.minimize(J)

        #   Constraints
        self.opti.subject_to( self.opti.bounded( self.con_vel["u_lb"], self.vx, self.con_vel["u_ub"] ) )
        self.opti.subject_to( self.opti.bounded( self.con_vel["r_lb"], self.wz, self.con_vel["r_ub"] ) )
        self.opti.subject_to( self.opti.bounded( self.con_vel["virtual_speed_lb"], self.virtual_speed, self.con_vel["virtual_speed_ub"] ) )
        self.opti.subject_to( self.opti.bounded( self.fl_min, self.fl, self.fl_max ) )
        self.opti.subject_to( self.opti.bounded( self.fr_min, self.fr, self.fr_max ) )
        
        if( sdf is not None ):
            for k in range(self.specs["N"] + 1):
                self.opti.subject_to( sdf( ca.vertcat(-self.y[k], self.x[k]) ) > 2e-1 )

        #   Setup solver
        opts = {"ipopt.print_level": 0, "print_time": True, "ipopt.max_iter": 1000, "ipopt.tol": 1e-6, "ipopt.acceptable_tol": 1e-4}
        self.opti.solver("ipopt", opts)

    def _setInitialState(self, initialState, parameters):
        
        #   Set initial state
        self.opti.set_value( self.x_0, initialState[0] )
        self.opti.set_value( self.y_0, initialState[1] )
        self.opti.set_value( self.yaw_0, initialState[2] )
        self.opti.set_value( self.progress_0, initialState[3] )
        self.opti.set_value( self.vx_0, initialState[4] )
        self.opti.set_value( self.wz_0, initialState[5] )
        self.opti.set_value( self.friction, parameters["friction"] )
        self.opti.set_value( self.pitch, parameters["pitch"] )
        self.opti.set_value( self.roll, parameters["roll"] )
        self.opti.set_value( self.fl_prev, parameters["fl_prev"] )
        self.opti.set_value( self.fr_prev, parameters["fr_prev"] )
        self.opti.set_value( self.virtual_speed_prev, parameters["virtual_speed_prev"] )
        
        self.opti.set_value( self.fl_min, parameters["fl_min"] )
        self.opti.set_value( self.fl_max, parameters["fl_max"] )
        self.opti.set_value( self.fr_min, parameters["fr_min"] )
        self.opti.set_value( self.fr_max, parameters["fr_max"] )

    def _setInitialGuess(self, lastSol):
        
        self.opti.set_initial( self.x, np.append( lastSol[0][1:], lastSol[0][-1] ) )
        self.opti.set_initial( self.y, np.append( lastSol[1][1:], lastSol[1][-1] ) )
        self.opti.set_initial( self.yaw, np.append( lastSol[2][1:], lastSol[2][-1] ) )
        self.opti.set_initial( self.progress, np.append( lastSol[3][1:], lastSol[3][-1] ) )
        self.opti.set_initial( self.vx, np.append( lastSol[4][1:], lastSol[4][-1] ) )
        self.opti.set_initial( self.wz, np.append( lastSol[5][1:], lastSol[5][-1] ) )
        self.opti.set_initial( self.fl, np.append( lastSol[6][1:], lastSol[6][-1] ) )
        self.opti.set_initial( self.fr, np.append( lastSol[7][1:], lastSol[7][-1] ) )
        self.opti.set_initial( self.virtual_speed, np.append( lastSol[8][1:], lastSol[8][-1] ) )

    def _getStats(self):
        return self.opti.stats()
    
    def _solve(self, initialState, prevSol, parameters):
        
        if( prevSol is None ):
            pass
        
        else:
            self._setInitialGuess(prevSol)

        self._setInitialState(initialState, parameters)

        #   Solve optimization problem
        sol = self.opti.solve()

        # Extract solution
        x_sol  = sol.value(self.x)
        y_sol  = sol.value(self.y)
        yaw_sol = sol.value(self.yaw)
        progress_sol = sol.value(self.progress)
        vx_sol = sol.value(self.vx)
        wz_sol = sol.value(self.wz)
        fl_sol = sol.value(self.fl)
        fr_sol = sol.value(self.fr)
        virtual_speed_sol = sol.value(self.virtual_speed)

        return [x_sol, y_sol, yaw_sol, progress_sol, vx_sol, wz_sol, fl_sol, fr_sol, virtual_speed_sol]

#   Parameterized unicycle dynamics model with controlled virtual speed (simple solution)
class DynamicsEnergyPath(ModelParameters, Common):
    def __init__(self, x_ref, y_ref, yaw_ref, sdf=None):
        
        super().__init__()

        """
            Dynamics class to simulate one stage ocp without heading regulation with power or force optimization
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

        #   Virtual speed functions parameters
        self.L_e = 5e-1
        self.L_k = 5e-1

        self.k_e = 5e0
        self.k_k = 5e-1

        self.x_m_e = 2e0
        self.x_m_k = 1e1

        #   States
        self.x = self.opti.variable( self.specs["N"] + 1 )
        self.y = self.opti.variable( self.specs["N"] + 1 )
        self.yaw = self.opti.variable( self.specs["N"] + 1 )
        self.vx = self.opti.variable( self.specs["N"] + 1 )
        self.wz = self.opti.variable( self.specs["N"] + 1 )

        #   Controls
        self.fl = self.opti.variable( self.specs["N"] )
        self.fr = self.opti.variable( self.specs["N"] )

        #   Set initial state parameters
        self.x_0 = self.opti.parameter()
        self.y_0 = self.opti.parameter()
        self.yaw_0 = self.opti.parameter()
        self.vx_0 = self.opti.parameter()
        self.wz_0 = self.opti.parameter()
        self.pitch = self.opti.parameter()
        self.roll = self.opti.parameter()
        self.friction = self.opti.parameter()

        self.fl_prev = self.opti.parameter()
        self.fr_prev = self.opti.parameter()

        self.fl_min = self.opti.parameter()
        self.fl_max = self.opti.parameter()
        self.fr_min = self.opti.parameter()
        self.fr_max = self.opti.parameter()

        self.x_ref = self.opti.parameter( self.specs["N"] + 1 )
        self.y_ref = self.opti.parameter( self.specs["N"] + 1 )
        self.yaw_ref = self.opti.parameter( self.specs["N"] + 1 )

        #   Initial constraints
        self.opti.subject_to(self.x[0] == self.x_0)
        self.opti.subject_to(self.y[0] == self.y_0)
        self.opti.subject_to(self.yaw[0] == self.yaw_0)
        self.opti.subject_to(self.vx[0] == self.vx_0)
        self.opti.subject_to(self.wz[0] == self.wz_0)
        
        #   RK4 integrator
        for k in range( self.specs["N"] ):
            x_next = ca.vertcat( self.x[k + 1], self.y[k + 1], self.yaw[k + 1], self.vx[k + 1], self.wz[k + 1] )
            x_now  = ca.vertcat( self.x[k], self.y[k], self.yaw[k], self.vx[k], self.wz[k] )
            
            k1 = self.dynamics_path(self.x[k],\
                                    self.y[k],\
                                    self.yaw[k],\
                                    self.vx[k],\
                                    self.wz[k],\
                                    self.fl[k],\
                                    self.fr[k],\
                                    self.pitch )

            k2 = self.dynamics_path(self.x[k] + self.specs["Ts"] / 2 * k1[0],\
                                    self.y[k] + self.specs["Ts"] / 2 * k1[1],\
                                    self.yaw[k] + self.specs["Ts"] / 2 * k1[2],\
                                    self.vx[k] + self.specs["Ts"] / 2 * k1[3],\
                                    self.wz[k] + self.specs["Ts"] / 2 * k1[4],\
                                    self.fl[k],\
                                    self.fr[k],\
                                    self.pitch )
            
            k3 = self.dynamics_path(self.x[k] + self.specs["Ts"] / 2 * k2[0],\
                                    self.y[k] + self.specs["Ts"] / 2 * k2[1],\
                                    self.yaw[k] + self.specs["Ts"] / 2 * k2[2],\
                                    self.vx[k] + self.specs["Ts"] / 2 * k2[3],\
                                    self.wz[k] + self.specs["Ts"] / 2 * k2[4],\
                                    self.fl[k],\
                                    self.fr[k],\
                                    self.pitch )
            
            k4 = self.dynamics_path(self.x[k] + self.specs["Ts"] * k3[0],\
                                    self.y[k] + self.specs["Ts"] * k3[1],\
                                    self.yaw[k] + self.specs["Ts"] * k3[2],\
                                    self.vx[k] + self.specs["Ts"] * k3[3],\
                                    self.wz[k] + self.specs["Ts"] * k3[4],\
                                    self.fl[k],\
                                    self.fr[k],\
                                    self.pitch )

            x_next_pred = x_now + (self.specs["Ts"] / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
            
            self.opti.subject_to(x_next == x_next_pred)

        #   Cost function
        J = 0
        
        for k in range(self.specs["N"] + 1):

            if( k == 0 ):
                err = ca.vertcat( self.x[k] - self.x_ref[k],\
                                  self.y[k] - self.y_ref[k],\
                                  ( ca.cos( self.yaw[k] ) - ca.cos( self.yaw_ref[k] ) )**2 + ( ca.sin( self.yaw[k] ) - ca.sin( self.yaw_ref[k] ) )**2,\
                                  self.fl[k],\
                                  self.fr[k],\
                                  self.fl[k] - self.fl_prev,\
                                  self.fr[k] - self.fr_prev )
                
                J += err.T @ ca.diag( ca.vertcat( self.weights["Q_x"],\
                                                  self.weights["Q_y"],\
                                                  self.weights["Q_yaw"],\
                                                  self.weights["Q_fl"],\
                                                  self.weights["Q_fr"],\
                                                  self.weights["Q_fl_rate"],\
                                                  self.weights["Q_fr_rate"] ) ) @ err

            if( k < self.specs["N"] ):
                err = ca.vertcat( self.x[k] - self.x_ref[k],\
                                  self.y[k] - self.y_ref[k],\
                                  ( ca.cos( self.yaw[k] ) - ca.cos( self.yaw_ref[k] ) )**2 + ( ca.sin( self.yaw[k] ) - ca.sin( self.yaw_ref[k] ) )**2,\
                                  self.fl[k],\
                                  self.fr[k],\
                                  self.fl[k] - self.fl[k - 1],\
                                  self.fr[k] - self.fr[k - 1] )
                
                J += err.T @ ca.diag( ca.vertcat( self.weights["Q_x"],\
                                                  self.weights["Q_y"],\
                                                  self.weights["Q_yaw"],\
                                                  self.weights["Q_fl"],\
                                                  self.weights["Q_fr"],\
                                                  self.weights["Q_fl_rate"],\
                                                  self.weights["Q_fr_rate"] ) ) @ err

            else:
                err = ca.vertcat( self.x[k] - self.x_ref[k],\
                                  self.y[k] - self.y_ref[k],\
                                  ( ca.cos( self.yaw[k] ) - ca.cos( self.yaw_ref[k] ) )**2 + ( ca.sin( self.yaw[k] ) - ca.sin( self.yaw_ref[k] ) )**2 )

                J += err.T @ ca.diag( ca.vertcat( self.weights["Q_x_t"], self.weights["Q_y_t"], self.weights["Q_yaw_t"] ) ) @ err
        
        self.opti.minimize(J)

        #   Constraints
        self.opti.subject_to( self.opti.bounded( self.con_vel["u_lb"], self.vx, self.con_vel["u_ub"] ) )
        self.opti.subject_to( self.opti.bounded( self.con_vel["r_lb"], self.wz, self.con_vel["r_ub"] ) )
        self.opti.subject_to( self.opti.bounded( self.fl_min, self.fl, self.fl_max ) )
        self.opti.subject_to( self.opti.bounded( self.fr_min, self.fr, self.fr_max ) )
        
        if( sdf is not None ):
            for k in range(self.specs["N"] + 1):
                self.opti.subject_to( sdf( ca.vertcat(-self.y[k], self.x[k]) ) > 2e-1 )

        #   Setup solver
        opts = {"ipopt.print_level": 0, "print_time": True, "ipopt.max_iter": 1000, "ipopt.tol": 1e-6, "ipopt.acceptable_tol": 1e-4}
        self.opti.solver("ipopt", opts)

    def _setInitialState(self, initialState, parameters):
        
        #   Set initial state
        self.opti.set_value( self.x_0, initialState[0] )
        self.opti.set_value( self.y_0, initialState[1] )
        self.opti.set_value( self.yaw_0, initialState[2] )
        self.opti.set_value( self.vx_0, initialState[3] )
        self.opti.set_value( self.wz_0, initialState[4] )
        self.opti.set_value( self.friction, parameters["friction"] )
        self.opti.set_value( self.pitch, parameters["pitch"] )
        self.opti.set_value( self.roll, parameters["roll"] )
        self.opti.set_value( self.fl_prev, parameters["fl_prev"] )
        self.opti.set_value( self.fr_prev, parameters["fr_prev"] )
        
        self.opti.set_value( self.fl_min, parameters["fl_min"] )
        self.opti.set_value( self.fl_max, parameters["fl_max"] )
        self.opti.set_value( self.fr_min, parameters["fr_min"] )
        self.opti.set_value( self.fr_max, parameters["fr_max"] )

        #   Set references
        self.opti.set_value( self.x_ref, parameters["x_ref"] )
        self.opti.set_value( self.y_ref, parameters["y_ref"] )
        self.opti.set_value( self.yaw_ref, parameters["yaw_ref"] )

    def _setInitialGuess(self, lastSol):
        
        self.opti.set_initial( self.x, np.append( lastSol[0][1:], lastSol[0][-1] ) )
        self.opti.set_initial( self.y, np.append( lastSol[1][1:], lastSol[1][-1] ) )
        self.opti.set_initial( self.yaw, np.append( lastSol[2][1:], lastSol[2][-1] ) )
        self.opti.set_initial( self.vx, np.append( lastSol[3][1:], lastSol[3][-1] ) )
        self.opti.set_initial( self.wz, np.append( lastSol[4][1:], lastSol[4][-1] ) )
        self.opti.set_initial( self.fl, np.append( lastSol[5][1:], lastSol[5][-1] ) )
        self.opti.set_initial( self.fr, np.append( lastSol[6][1:], lastSol[6][-1] ) )

    def _getStats(self):
        return self.opti.stats()
    
    def _solve(self, initialState, prevSol, parameters):
        
        if( prevSol is None ):
            pass
        
        else:
            self._setInitialGuess(prevSol)

        self._setInitialState(initialState, parameters)

        #   Solve optimization problem
        sol = self.opti.solve()

        # Extract solution
        x_sol  = sol.value(self.x)
        y_sol  = sol.value(self.y)
        yaw_sol = sol.value(self.yaw)
        vx_sol = sol.value(self.vx)
        wz_sol = sol.value(self.wz)
        fl_sol = sol.value(self.fl)
        fr_sol = sol.value(self.fr)

        return [x_sol, y_sol, yaw_sol, vx_sol, wz_sol, fl_sol, fr_sol]

    def _getVirtualSpeed(self, error, curvature):
        
        #   Error
        v_error = self.L_e / ( 1 + math.exp( self.k_e * (error - self.x_m_e) ) )

        #   Curvature
        v_curvature = self.L_k / ( 1 + math.exp( self.k_k * (curvature - self.x_m_k) ) )

        return min(v_error, v_curvature)

#   Parameterized unicycle dynamics model with controlled virtual speed (simple solution) + countoring + lag error
class DynamicsMPCC(ModelParameters, Common):
    def __init__(self, x_ref, y_ref, yaw_ref, sdf=None):
        
        super().__init__()

        """
            Dynamics class to simulate one stage ocp with countoring and lag error contributions
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

        #   States
        self.x = self.opti.variable( self.specs["N"] + 1 )
        self.y = self.opti.variable( self.specs["N"] + 1 )
        self.yaw = self.opti.variable( self.specs["N"] + 1 )
        self.progress = self.opti.variable( self.specs["N"] + 1 )
        self.vx = self.opti.variable( self.specs["N"] + 1 )
        self.wz = self.opti.variable( self.specs["N"] + 1 )

        #   Controls
        self.fl = self.opti.variable( self.specs["N"] )
        self.fr = self.opti.variable( self.specs["N"] )
        self.virtual_speed = self.opti.variable( self.specs["N"] )

        #   Set initial state parameters
        self.x_0 = self.opti.parameter()
        self.y_0 = self.opti.parameter()
        self.yaw_0 = self.opti.parameter()
        self.progress_0 = self.opti.parameter()
        self.vx_0 = self.opti.parameter()
        self.wz_0 = self.opti.parameter()
        self.pitch = self.opti.parameter()
        self.roll = self.opti.parameter()
        self.friction = self.opti.parameter()

        self.virtual_speed_prev = self.opti.parameter()
        self.fl_prev = self.opti.parameter()
        self.fr_prev = self.opti.parameter()

        self.fl_min = self.opti.parameter()
        self.fl_max = self.opti.parameter()
        self.fr_min = self.opti.parameter()
        self.fr_max = self.opti.parameter()

        #   Initial constraints
        self.opti.subject_to(self.x[0] == self.x_0)
        self.opti.subject_to(self.y[0] == self.y_0)
        self.opti.subject_to(self.yaw[0] == self.yaw_0)
        self.opti.subject_to(self.progress[0] == self.progress_0 )
        self.opti.subject_to(self.vx[0] == self.vx_0)
        self.opti.subject_to(self.wz[0] == self.wz_0)
        
        #   RK4 integrator
        for k in range( self.specs["N"] ):
            x_next = ca.vertcat( self.x[k + 1], self.y[k + 1], self.yaw[k + 1], self.progress[k + 1], self.vx[k + 1], self.wz[k + 1] )
            x_now  = ca.vertcat( self.x[k], self.y[k], self.yaw[k], self.progress[k], self.vx[k], self.wz[k] )
            
            k1 = self.dynamics(self.x[k],\
                               self.y[k],\
                               self.yaw[k],\
                               self.vx[k],\
                               self.wz[k],\
                               self.virtual_speed[k],\
                               self.fl[k],\
                               self.fr[k],\
                               self.pitch )

            k2 = self.dynamics(self.x[k] + self.specs["Ts"] / 2 * k1[0],\
                               self.y[k] + self.specs["Ts"] / 2 * k1[1],\
                               self.yaw[k] + self.specs["Ts"] / 2 * k1[2],\
                               self.vx[k] + self.specs["Ts"] / 2 * k1[4],\
                               self.wz[k] + self.specs["Ts"] / 2 * k1[5],\
                               self.virtual_speed[k],\
                               self.fl[k],\
                               self.fr[k],\
                               self.pitch )
            
            k3 = self.dynamics(self.x[k] + self.specs["Ts"] / 2 * k2[0],\
                               self.y[k] + self.specs["Ts"] / 2 * k2[1],\
                               self.yaw[k] + self.specs["Ts"] / 2 * k2[2],\
                               self.vx[k] + self.specs["Ts"] / 2 * k2[4],\
                               self.wz[k] + self.specs["Ts"] / 2 * k2[5],\
                               self.virtual_speed[k],\
                               self.fl[k],\
                               self.fr[k],\
                               self.pitch )
            
            k4 = self.dynamics(self.x[k] + self.specs["Ts"] * k3[0],\
                               self.y[k] + self.specs["Ts"] * k3[1],\
                               self.yaw[k] + self.specs["Ts"] * k3[2],\
                               self.vx[k] + self.specs["Ts"] * k3[4],\
                               self.wz[k] + self.specs["Ts"] * k3[5],\
                               self.virtual_speed[k],\
                               self.fl[k],\
                               self.fr[k],\
                               self.pitch )

            x_next_pred = x_now + (self.specs["Ts"] / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
            
            self.opti.subject_to(x_next == x_next_pred)

        #   Cost function
        J = 0
        
        for k in range(self.specs["N"] + 1):

            el = -ca.cos( yaw_ref( self.progress[k] ) ) * ( self.x[k] - x_ref( self.progress[k] ) ) - ca.sin( yaw_ref( self.progress[k] ) ) * ( self.y[k] - y_ref( self.progress[k] ) )
            ec = ca.sin( yaw_ref( self.progress[k] ) ) * ( self.x[k] - x_ref( self.progress[k] ) ) - ca.cos( yaw_ref( self.progress[k] ) ) * ( self.y[k] - y_ref( self.progress[k] ) )

            delta_q = 1 - ca.exp( -3 * ( sdf( ca.vertcat( -self.y[k], self.x[k] ) ) - 0.2 ) )
            delta_sdf = ca.exp( -2 * ( sdf( ca.vertcat( -self.y[k], self.x[k] ) ) - 0.2 ) )

            if( k == 0 ):
                err = ca.vertcat( el,\
                                  ec,\
                                  ( ca.cos( self.yaw[k] ) - ca.cos( yaw_ref( self.progress[k] ) ) )**2 + ( ca.sin( self.yaw[k] ) - ca.sin( yaw_ref( self.progress[k] ) ) )**2,\
                                  1 / ( sdf( ca.vertcat( -self.y[k], self.x[k] ) ) + 1e-2 ),\
                                  self.fl[k] - self.fl_prev,\
                                  self.fr[k] - self.fr_prev,\
                                  self.virtual_speed[k] - 5e-1,\
                                  self.virtual_speed[k] - self.virtual_speed_prev )
                
                J += err.T @ ca.diag( ca.vertcat( self.weights["Q_x"],\
                                                  delta_q * self.weights["Q_y"],\
                                                  delta_q * self.weights["Q_yaw"],\
                                                  3.0,\
                                                  self.weights["Q_fl_rate"],\
                                                  self.weights["Q_fr_rate"],\
                                                  self.weights["Q_virtual_speed"],\
                                                  self.weights["Q_virtual_speed_rate"] ) ) @ err

            if( k < self.specs["N"] ):
                err = ca.vertcat( el,\
                                  ec,\
                                  ( ca.cos( self.yaw[k] ) - ca.cos( yaw_ref( self.progress[k] ) ) )**2 + ( ca.sin( self.yaw[k] ) - ca.sin( yaw_ref( self.progress[k] ) ) )**2,\
                                  1 / ( sdf( ca.vertcat( -self.y[k], self.x[k] ) ) + 1e-2 ),\
                                  self.fl[k] - self.fl[k - 1],\
                                  self.fr[k] - self.fr[k - 1],\
                                  self.virtual_speed[k] - 5e-1,\
                                  self.virtual_speed[k] - self.virtual_speed[k - 1] )
                
                J += err.T @ ca.diag( ca.vertcat( self.weights["Q_x"],\
                                                  delta_q * self.weights["Q_y"],\
                                                  delta_q * self.weights["Q_yaw"],\
                                                  3.0,\
                                                  self.weights["Q_fl_rate"],\
                                                  self.weights["Q_fr_rate"],\
                                                  self.weights["Q_virtual_speed"],\
                                                  self.weights["Q_virtual_speed_rate"] ) ) @ err

            else:
                err = ca.vertcat( el,\
                                  ec,\
                                  ( ca.cos( self.yaw[k] ) - ca.cos( yaw_ref( self.progress[k] ) ) )**2 + ( ca.sin( self.yaw[k] ) - ca.sin( yaw_ref( self.progress[k] ) ) )**2,\
                                  1 / ( sdf( ca.vertcat( -self.y[k], self.x[k] ) ) + 1e-2 ) )

                J += err.T @ ca.diag( ca.vertcat( self.weights["Q_x_t"],\
                                                  delta_q * self.weights["Q_y_t"],\
                                                  delta_q * self.weights["Q_yaw_t"],\
                                                  3.0 ) ) @ err
        
        self.opti.minimize(J)

        #   Constraints
        self.opti.subject_to( self.opti.bounded( self.con_vel["u_lb"], self.vx, self.con_vel["u_ub"] ) )
        self.opti.subject_to( self.opti.bounded( self.con_vel["r_lb"], self.wz, self.con_vel["r_ub"] ) )
        self.opti.subject_to( self.opti.bounded( self.con_vel["virtual_speed_lb"], self.virtual_speed, self.con_vel["virtual_speed_ub"] ) )
        self.opti.subject_to( self.opti.bounded( self.fl_min, self.fl, self.fl_max ) )
        self.opti.subject_to( self.opti.bounded( self.fr_min, self.fr, self.fr_max ) )

        for k in range(self.specs["N"] + 1):
            self.opti.subject_to( sdf( ca.vertcat( -self.y[k], self.x[k] ) ) > 2e-1 )

        #   Setup solver
        opts = {"ipopt.print_level": 0, "print_time": True, "ipopt.max_iter": 1000, "ipopt.tol": 1e-6, "ipopt.acceptable_tol": 1e-4}
        self.opti.solver("ipopt", opts)

    def _setInitialState(self, initialState, parameters):
        
        #   Set initial state
        self.opti.set_value( self.x_0, initialState[0] )
        self.opti.set_value( self.y_0, initialState[1] )
        self.opti.set_value( self.yaw_0, initialState[2] )
        self.opti.set_value( self.progress_0, initialState[3] )
        self.opti.set_value( self.vx_0, initialState[4] )
        self.opti.set_value( self.wz_0, initialState[5] )
        self.opti.set_value( self.friction, parameters["friction"] )
        self.opti.set_value( self.pitch, parameters["pitch"] )
        self.opti.set_value( self.roll, parameters["roll"] )
        self.opti.set_value( self.fl_prev, parameters["fl_prev"] )
        self.opti.set_value( self.fr_prev, parameters["fr_prev"] )
        self.opti.set_value( self.virtual_speed_prev, parameters["virtual_speed_prev"] )
        
        self.opti.set_value( self.fl_min, parameters["fl_min"] )
        self.opti.set_value( self.fl_max, parameters["fl_max"] )
        self.opti.set_value( self.fr_min, parameters["fr_min"] )
        self.opti.set_value( self.fr_max, parameters["fr_max"] )

    def _setInitialGuess(self, lastSol):
        
        self.opti.set_initial( self.x, np.append( lastSol[0][1:], lastSol[0][-1] ) )
        self.opti.set_initial( self.y, np.append( lastSol[1][1:], lastSol[1][-1] ) )
        self.opti.set_initial( self.yaw, np.append( lastSol[2][1:], lastSol[2][-1] ) )
        self.opti.set_initial( self.progress, np.append( lastSol[3][1:], lastSol[3][-1] ) )
        self.opti.set_initial( self.vx, np.append( lastSol[4][1:], lastSol[4][-1] ) )
        self.opti.set_initial( self.wz, np.append( lastSol[5][1:], lastSol[5][-1] ) )
        self.opti.set_initial( self.fl, np.append( lastSol[6][1:], lastSol[6][-1] ) )
        self.opti.set_initial( self.fr, np.append( lastSol[7][1:], lastSol[7][-1] ) )
        self.opti.set_initial( self.virtual_speed, np.append( lastSol[8][1:], lastSol[8][-1] ) )

    def _getStats(self):
        return self.opti.stats()
    
    def _solve(self, initialState, prevSol, parameters):
        
        if( prevSol is None ):
            pass
        
        else:
            self._setInitialGuess(prevSol)

        self._setInitialState(initialState, parameters)

        #   Solve optimization problem
        sol = self.opti.solve()

        # Extract solution
        x_sol  = sol.value(self.x)
        y_sol  = sol.value(self.y)
        yaw_sol = sol.value(self.yaw)
        progress_sol = sol.value(self.progress)
        vx_sol = sol.value(self.vx)
        wz_sol = sol.value(self.wz)
        fl_sol = sol.value(self.fl)
        fr_sol = sol.value(self.fr)
        virtual_speed_sol = sol.value(self.virtual_speed)

        return [x_sol, y_sol, yaw_sol, progress_sol, vx_sol, wz_sol, fl_sol, fr_sol, virtual_speed_sol]
