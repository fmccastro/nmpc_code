#!/usr/bin/python3.8

import sys

import scipy.linalg
sys.path.insert(0, "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/src")

from classes.all_imports import *
from classes.common_class import *

"""
    Build and define NMPC optimization problems with the standard formulation

    Solvers are to be used only on the first simulations for test purposes. Heading regulation and further refinement are excluded.
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

        vx_rate = ca.SX.sym('vx_rate')
        wz_rate = ca.SX.sym('wz_rate')
        virtual_speed_rate = ca.SX.sym('virtual_speed_rate')

        fl_rate = ca.SX.sym('fl_rate')
        fr_rate = ca.SX.sym('fr_rate')
        virtual_speed_rate = ca.SX.sym('virtual_speed_rate')

        d_kinematics_rates = ca.vertcat( ca.cos(pitch) * ca.cos(yaw) * vx,\
                                            ca.cos(pitch) * ca.sin(yaw) * vx,\
                                            wz,\
                                            virtual_speed,\
                                            vx_rate,\
                                            wz_rate )

        d_kinematics_rates_path = ca.vertcat( ca.cos(pitch) * ca.cos(yaw) * vx,\
                                                ca.cos(pitch) * ca.sin(yaw) * vx,\
                                                wz,\
                                                vx_rate,\
                                                wz_rate )
        
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

        """ Standard kinematics with velocity rates """
        self.kinematics_rates = ca.Function( 'kinematics_rates', [x, y, yaw, vx, wz, virtual_speed, vx_rate, wz_rate, pitch], [ d_kinematics_rates ] )

        """ Standard kinematics with velocity rates """
        self.kinematics_rates_path = ca.Function( 'kinematics_rates_path', [x, y, yaw, vx, wz, vx_rate, wz_rate, pitch], [ d_kinematics_rates_path ] )

        """ Dynamics    """
        self.dynamics = ca.Function( 'dynamics', [x, y, yaw, vx, wz, virtual_speed, fl, fr, pitch], [ d_dynamics ] )

        """ Dynamics with path  """
        self.dynamics_path = ca.Function( 'dynamics_path', [x, y, yaw, vx, wz, fl, fr, pitch], [ d_dynamics_path ] )

#   Standard unicycle kinematics model with rates
class KinematicsRates(ModelParameters, Common):
    def __init__(self, x_ref, y_ref):
        
        super().__init__()

        with open(self.std_kin_skid) as f:
            solver_param = json.load(f)
            specs = solver_param["solver_specs"]
            self.con_pose = solver_param["con_pose"]
            self.con_vel = solver_param["con_vel"]
            self.weights = solver_param["weights"]
        
        #   Work with Leo Rover
        with open(self.vehicle_specs) as f:
            vehicle_param = json.load(f)
            lat_w = vehicle_param["wheelLatSeparation"]

        self.opti = ca.Opti()

        #   States
        self.x = self.opti.variable( specs["N"] + 1 )
        self.y = self.opti.variable( specs["N"] + 1 )
        self.yaw = self.opti.variable( specs["N"] + 1 )
        self.progress = self.opti.variable( specs["N"] + 1 )
        self.vx = self.opti.variable( specs["N"] + 1 )
        self.wz = self.opti.variable( specs["N"] + 1 )
        self.virtual_speed = self.opti.variable( specs["N"] + 1 )

        #   Controls
        self.vx_rate = self.opti.variable( specs["N"] )
        self.wz_rate = self.opti.variable( specs["N"] )
        self.virtual_speed = self.opti.variable( specs["N"] )

        #   Set initial state parameters
        self.x_0 = self.opti.parameter()
        self.y_0 = self.opti.parameter()
        self.yaw_0 = self.opti.parameter()
        self.progress_0 = self.opti.parameter()
        self.vx_0 = self.opti.parameter()
        self.wz_0 = self.opti.parameter()
        self.pitch = self.opti.parameter()

        #   Initial constraints
        self.opti.subject_to(self.x[0] == self.x_0)
        self.opti.subject_to(self.y[0] == self.y_0)
        self.opti.subject_to(self.yaw[0] == self.yaw_0)
        self.opti.subject_to(self.progress[0] == self.progress_0)
        self.opti.subject_to(self.vx[0] == self.vx_0)
        self.opti.subject_to(self.wz[0] == self.wz_0)
        
        #   RK4 integrator
        for k in range( specs["N"] ):
            x_next = ca.vertcat( self.x[k + 1], self.y[k + 1], self.yaw[k + 1], self.progress[k + 1], self.vx[k + 1], self.wz[k + 1] )
            x_now  = ca.vertcat( self.x[k], self.y[k], self.yaw[k], self.progress[k], self.vx[k], self.wz[k] )
            
            k1 = self.kinematics_rates(self.x[k],\
                                        self.y[k],\
                                        self.yaw[k],\
                                        self.vx[k],\
                                        self.wz[k],\
                                        self.virtual_speed[k],\
                                        self.vx_rate[k],\
                                        self.wz_rate[k],\
                                        self.pitch )

            k2 = self.kinematics_rates(self.x[k] + specs["Ts"] / 2 * k1[0],\
                                        self.y[k] + specs["Ts"] / 2 * k1[1],\
                                        self.yaw[k] + specs["Ts"] / 2 * k1[2],\
                                        self.vx[k] + specs["Ts"] / 2 * k1[4],\
                                        self.wz[k] + specs["Ts"] / 2 * k1[5],\
                                        self.virtual_speed[k],\
                                        self.vx_rate[k],\
                                        self.wz_rate[k],\
                                        self.pitch )
            
            k3 = self.kinematics_rates(self.x[k] + specs["Ts"] / 2 * k2[0],\
                                        self.y[k] + specs["Ts"] / 2 * k2[1],\
                                        self.yaw[k] + specs["Ts"] / 2 * k2[2],\
                                        self.vx[k] + specs["Ts"] / 2 * k2[4],\
                                        self.wz[k] + specs["Ts"] / 2 * k2[5],\
                                        self.virtual_speed[k],\
                                        self.vx_rate[k],\
                                        self.wz_rate[k],\
                                        self.pitch )
            
            k4 = self.kinematics_rates(self.x[k] + specs["Ts"] * k3[0],\
                                        self.y[k] + specs["Ts"] * k3[1],\
                                        self.yaw[k] + specs["Ts"] * k3[2],\
                                        self.vx[k] + specs["Ts"] * k3[4],\
                                        self.wz[k] + specs["Ts"] * k3[5],\
                                        self.virtual_speed[k],\
                                        self.vx_rate[k],\
                                        self.wz_rate[k],\
                                        self.pitch )

            x_next_pred = x_now + (specs["Ts"] / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
            
            self.opti.subject_to(x_next == x_next_pred)

        #   Cost function
        J = 0
        
        for k in range(specs["N"] + 1):
            if( k < specs["N"] ):
                err = ca.vertcat( self.x[k] - x_ref( self.progress[k] ),\
                                  self.y[k] - y_ref( self.progress[k] ),\
                                  self.virtual_speed[k] - 5e-1,\
                                  self.vx_rate[k],\
                                  self.wz_rate[k] )
                
                J += err.T @ ca.diag( ca.vertcat( self.weights["Q_x"], self.weights["Q_y"], self.weights["Q_virtual_speed"], self.weights["Q_vx_rate"], self.weights["Q_wz_rate"] ) ) @ err

            else:
                err = ca.vertcat( self.x[k] - x_ref(self.progress[k] ),\
                                  self.y[k] - y_ref(self.progress[k] ) )
                
                J += err.T @ ca.diag( ca.vertcat( self.weights["Q_x_t"], self.weights["Q_y_t"] ) ) @ err

        self.opti.minimize(J)

        #   Constraints
        self.opti.subject_to( self.opti.bounded( self.con_vel["u_lb"], self.vx, self.con_vel["u_ub"] ) )
        self.opti.subject_to( self.opti.bounded( self.con_vel["r_lb"], self.wz, self.con_vel["r_ub"] ) )

        #   Setup solver
        opts = {"ipopt.print_level": 0, "print_time": True, "ipopt.max_iter": 1000, "ipopt.tol": 1e-6, "ipopt.acceptable_tol": 1e-4}
        self.opti.solver("ipopt", opts)

    def _setInitialState(self, initialState, pitch):
        
        #   Set initial state
        self.opti.set_value( self.x_0, initialState[0] )
        self.opti.set_value( self.y_0, initialState[1] )
        self.opti.set_value( self.yaw_0, initialState[2] )
        self.opti.set_value( self.progress_0, initialState[3] )
        self.opti.set_value( self.vx_0, initialState[4] )
        self.opti.set_value( self.wz_0, initialState[5] )
        self.opti.set_value( self.pitch, pitch )

    def _setInitialGuess(self, lastSol):
    
        self.opti.set_initial( self.x, np.append( lastSol[0][1:], lastSol[0][-1] ) )
        self.opti.set_initial( self.y, np.append( lastSol[1][1:], lastSol[1][-1] ) )
        self.opti.set_initial( self.yaw, np.append( lastSol[2][1:], lastSol[2][-1] ) )
        self.opti.set_initial( self.progress, np.append( lastSol[3][1:], lastSol[3][-1] ) )
        self.opti.set_initial( self.vx, np.append( lastSol[4][1:], lastSol[4][-1] ) )
        self.opti.set_initial( self.wz, np.append( lastSol[5][1:], lastSol[5][-1] ) )
    
    def _getStats(self):
        return self.opti.stats()
    
    def _solve(self, initialState, prevSol, pitch):
        
        if( prevSol is None ):
            pass
        
        else:
            self._setInitialGuess(prevSol)

        self._setInitialState(initialState, pitch)

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
        vx_rate_sol = sol.value(self.vx_rate)
        wz_rate_sol = sol.value(self.wz_rate)

        return [x_sol, y_sol, yaw_sol, progress_sol, vx_sol, wz_sol, vx_rate_sol, wz_rate_sol, virtual_speed_sol]

#   Standard unicycle kinematics model with rates
class KinematicsRatesPath(ModelParameters, Common):
    def __init__(self):
        
        super().__init__()

        with open(self.std_kin_skid) as f:
            solver_param = json.load(f)
            specs = solver_param["solver_specs"]
            self.con_pose = solver_param["con_pose"]
            self.con_vel = solver_param["con_vel"]
            self.weights = solver_param["weights"]
        
        #   Work with Leo Rover
        with open(self.vehicle_specs) as f:
            vehicle_param = json.load(f)
            lat_w = vehicle_param["wheelLatSeparation"]

        self.opti = ca.Opti()

        #   States
        self.x = self.opti.variable( specs["N"] + 1 )
        self.y = self.opti.variable( specs["N"] + 1 )
        self.yaw = self.opti.variable( specs["N"] + 1 )
        self.vx = self.opti.variable( specs["N"] + 1 )
        self.wz = self.opti.variable( specs["N"] + 1 )

        #   Controls
        self.vx_rate = self.opti.variable( specs["N"] )
        self.wz_rate = self.opti.variable( specs["N"] )

        #   Set initial state parameters
        self.x_0 = self.opti.parameter()
        self.y_0 = self.opti.parameter()
        self.yaw_0 = self.opti.parameter()
        self.vx_0 = self.opti.parameter()
        self.wz_0 = self.opti.parameter()
        self.pitch = self.opti.parameter()

        self.x_ref = self.opti.parameter( specs["N"] + 1 )
        self.y_ref = self.opti.parameter( specs["N"] + 1 )

        #   Initial constraints
        self.opti.subject_to(self.x[0] == self.x_0)
        self.opti.subject_to(self.y[0] == self.y_0)
        self.opti.subject_to(self.yaw[0] == self.yaw_0)
        self.opti.subject_to(self.vx[0] == self.vx_0)
        self.opti.subject_to(self.wz[0] == self.wz_0)
        
        #   RK4 integrator
        for k in range( specs["N"] ):
            x_next = ca.vertcat( self.x[k + 1], self.y[k + 1], self.yaw[k + 1], self.vx[k + 1], self.wz[k + 1] )
            x_now  = ca.vertcat( self.x[k], self.y[k], self.yaw[k], self.vx[k], self.wz[k] )
            
            k1 = self.kinematics_rates_path(self.x[k],\
                                            self.y[k],\
                                            self.yaw[k],\
                                            self.vx[k],\
                                            self.wz[k],\
                                            self.vx_rate[k],\
                                            self.wz_rate[k],\
                                            self.pitch )

            k2 = self.kinematics_rates_path(self.x[k] + specs["Ts"] / 2 * k1[0],\
                                            self.y[k] + specs["Ts"] / 2 * k1[1],\
                                            self.yaw[k] + specs["Ts"] / 2 * k1[2],\
                                            self.vx[k] + specs["Ts"] / 2 * k1[3],\
                                            self.wz[k] + specs["Ts"] / 2 * k1[4],\
                                            self.vx_rate[k],\
                                            self.wz_rate[k],\
                                            self.pitch )
            
            k3 = self.kinematics_rates_path(self.x[k] + specs["Ts"] / 2 * k2[0],\
                                            self.y[k] + specs["Ts"] / 2 * k2[1],\
                                            self.yaw[k] + specs["Ts"] / 2 * k2[2],\
                                            self.vx[k] + specs["Ts"] / 2 * k2[3],\
                                            self.wz[k] + specs["Ts"] / 2 * k2[4],\
                                            self.vx_rate[k],\
                                            self.wz_rate[k],\
                                            self.pitch )
            
            k4 = self.kinematics_rates_path(self.x[k] + specs["Ts"] * k3[0],\
                                            self.y[k] + specs["Ts"] * k3[1],\
                                            self.yaw[k] + specs["Ts"] * k3[2],\
                                            self.vx[k] + specs["Ts"] * k3[3],\
                                            self.wz[k] + specs["Ts"] * k3[4],\
                                            self.vx_rate[k],\
                                            self.wz_rate[k],\
                                            self.pitch )

            x_next_pred = x_now + (specs["Ts"] / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
            
            self.opti.subject_to(x_next == x_next_pred)

        #   Cost function
        J = 0
        
        for k in range(specs["N"] + 1):
            if( k < specs["N"] ):
                err = ca.vertcat( self.x[k] - self.x_ref[k],\
                                  self.y[k] - self.y_ref[k],\
                                  self.vx_rate[k],\
                                  self.wz_rate[k] )
                
                J += err.T @ ca.diag( ca.vertcat( self.weights["Q_x"], self.weights["Q_y"], self.weights["Q_vx_rate"], self.weights["Q_wz_rate"] ) ) @ err

            else:
                err = ca.vertcat( self.x[k] - self.x_ref[k],\
                                  self.y[k] - self.y_ref[k] )
                
                J += err.T @ ca.diag( ca.vertcat( self.weights["Q_x_t"], self.weights["Q_y_t"] ) ) @ err

        self.opti.minimize(J)

        #   Constraints
        self.opti.subject_to( self.opti.bounded( self.con_vel["u_lb"], self.vx, self.con_vel["u_ub"] ) )
        self.opti.subject_to( self.opti.bounded( self.con_vel["r_lb"], self.wz, self.con_vel["r_ub"] ) )

        #   Setup solver
        opts = {"ipopt.print_level": 0, "print_time": True, "ipopt.max_iter": 1000, "ipopt.tol": 1e-6, "ipopt.acceptable_tol": 1e-4}
        self.opti.solver("ipopt", opts)

    def _setInitialState(self, initialState, pitch, xref, yref):
        
        #   Set initial state
        self.opti.set_value( self.x_0, initialState[0] )
        self.opti.set_value( self.y_0, initialState[1] )
        self.opti.set_value( self.yaw_0, initialState[2] )
        self.opti.set_value( self.vx_0, initialState[3] )
        self.opti.set_value( self.wz_0, initialState[4] )
        self.opti.set_value( self.pitch, pitch )

        k = 0
        for xr, yr in zip( xref, yref ):
            self.opti.set_value( self.x_ref[k], xr )
            self.opti.set_value( self.y_ref[k], yr )

            k += 1

    def _setInitialGuess(self, lastSol):

        self.opti.set_initial( self.x, np.append( lastSol[0][1:], lastSol[0][-1] ) )
        self.opti.set_initial( self.y, np.append( lastSol[1][1:], lastSol[1][-1] ) )
        self.opti.set_initial( self.yaw, np.append( lastSol[2][1:], lastSol[2][-1] ) )
        self.opti.set_initial( self.vx, np.append( lastSol[3][1:], lastSol[3][-1] ) )
        self.opti.set_initial( self.wz, np.append( lastSol[4][1:], lastSol[4][-1] ) )
    
    def _getStats(self):
        return self.opti.stats()
    
    def _solve(self, initialState, prevSol, pitch, xref, yref):
        
        if( prevSol is None ):
            pass
        
        else:
            self._setInitialGuess(prevSol)

        self._setInitialState(initialState, pitch, xref, yref)

        #   Solve optimization problem
        sol = self.opti.solve()

        # Extract solution
        x_sol  = sol.value(self.x)
        y_sol  = sol.value(self.y)
        yaw_sol = sol.value(self.yaw)
        vx_sol = sol.value(self.vx)
        wz_sol = sol.value(self.wz)
        vx_rate_sol = sol.value(self.vx_rate)
        wz_rate_sol = sol.value(self.wz_rate)

        return [x_sol, y_sol, yaw_sol, vx_sol, wz_sol, vx_rate_sol, wz_rate_sol]

#   Parameterized unicycle dynamics model
class DynamicsPath(ModelParameters, Common):
    def __init__(self):
        
        super().__init__()

        with open(self.std_dyn_skid) as f:
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
        self.vx_0 = self.opti.parameter()
        self.wz_0 = self.opti.parameter()
        self.pitch = self.opti.parameter()
        self.roll = self.opti.parameter()
        self.friction = self.opti.parameter()

        self.xref = self.opti.parameter( self.specs["N"] + 1 )
        self.yref = self.opti.parameter( self.specs["N"] + 1 )

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
                err = ca.vertcat( self.x[k] - self.xref[k],\
                                  self.y[k] - self.yref[k],\
                                  self.fl[k] - self.fl_prev,\
                                  self.fr[k] - self.fr_prev )
                
                J += err.T @ ca.diag( ca.vertcat( self.weights["Q_x"],\
                                                  self.weights["Q_y"],\
                                                  self.weights["Q_fl_rate"],\
                                                  self.weights["Q_fr_rate"] ) ) @ err

            if( k < self.specs["N"] ):
                err = ca.vertcat( self.x[k] - self.xref[k],\
                                  self.y[k] - self.yref[k],\
                                  self.fl[k] - self.fl[k - 1],\
                                  self.fr[k] - self.fr[k - 1] )
                
                J += err.T @ ca.diag( ca.vertcat( self.weights["Q_x"],\
                                                  self.weights["Q_y"],\
                                                  self.weights["Q_fl_rate"],\
                                                  self.weights["Q_fr_rate"] ) ) @ err

            else:
                err = ca.vertcat( self.x[k] - self.xref[k],\
                                  self.y[k] - self.yref[k] )
                
                J += err.T @ ca.diag( ca.vertcat( self.weights["Q_x_t"], self.weights["Q_y_t"] ) ) @ err

        self.opti.minimize(J)

        #   Constraints
        self.opti.subject_to( self.opti.bounded( self.con_vel["u_lb"], self.vx, self.con_vel["u_ub"] ) )
        self.opti.subject_to( self.opti.bounded( self.con_vel["r_lb"], self.wz, self.con_vel["r_ub"] ) )
        self.opti.subject_to( self.opti.bounded( self.fl_min, self.fl, self.fl_max ) )
        self.opti.subject_to( self.opti.bounded( self.fr_min, self.fr, self.fr_max ) )

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

        k = 0
        for xr, yr in zip( parameters["x_ref"], parameters["y_ref"] ):
            self.opti.set_value( self.xref[k], xr )
            self.opti.set_value( self.yref[k], yr )

            k += 1

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

#   Parameterized unicycle dynamics model with controlled virtual speed (simple solution)
class Dynamics(ModelParameters, Common):
    def __init__(self, x_ref, y_ref):
        
        super().__init__()

        """
            Dynamics class to simulate one stage ocp without heading regulation 
        """

        with open(self.std_dyn_skid) as f:
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
                                  self.fl[k] - self.fl_prev,\
                                  self.fr[k] - self.fr_prev,\
                                  self.virtual_speed[k] - 5e-1 )
                
                J += err.T @ ca.diag( ca.vertcat( self.weights["Q_x"],\
                                                  self.weights["Q_y"],\
                                                  self.weights["Q_fl_rate"],\
                                                  self.weights["Q_fr_rate"],\
                                                  self.weights["Q_virtual_speed"] ) ) @ err

            if( k < self.specs["N"] ):
                err = ca.vertcat( self.x[k] - x_ref( self.progress[k] ),\
                                  self.y[k] - y_ref( self.progress[k] ),\
                                  self.fl[k] - self.fl[k - 1],\
                                  self.fr[k] - self.fr[k - 1],\
                                  self.virtual_speed[k] - 5e-1 )
                
                J += err.T @ ca.diag( ca.vertcat( self.weights["Q_x"],\
                                                  self.weights["Q_y"],\
                                                  self.weights["Q_fl_rate"],\
                                                  self.weights["Q_fr_rate"],\
                                                  self.weights["Q_virtual_speed"] ) ) @ err

            else:
                err = ca.vertcat( self.x[k] - x_ref( self.progress[k] ),\
                                  self.y[k] - y_ref( self.progress[k] ) )
                
                J += err.T @ ca.diag( ca.vertcat( self.weights["Q_x_t"], self.weights["Q_y_t"] ) ) @ err

        self.opti.minimize(J)

        #   Constraints
        self.opti.subject_to( self.opti.bounded( self.con_vel["u_lb"], self.vx, self.con_vel["u_ub"] ) )
        self.opti.subject_to( self.opti.bounded( self.con_vel["r_lb"], self.wz, self.con_vel["r_ub"] ) )
        self.opti.subject_to( self.opti.bounded( self.fl_min, self.fl, self.fl_max ) )
        self.opti.subject_to( self.opti.bounded( self.fr_min, self.fr, self.fr_max ) )

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