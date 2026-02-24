#!/usr/bin/python3.8

import sys

import scipy.linalg
sys.path.insert(0, "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/src")

from classes.all_imports import *
from classes.common_class import *

"""
    Build and define NMPC optimization problem with the Frenet-Serret formulation
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

        s1 = ca.SX.sym('s1')
        y1 = ca.SX.sym('y1')
        diff_yaw = ca.SX.sym('diff_yaw')
        pitch = ca.SX.sym('pitch')
        roll = ca.SX.sym('roll')
        friction = ca.SX.sym('friction')
        vx = ca.SX.sym('vx')
        wz = ca.SX.sym('wz')
        virtual_speed = ca.SX.sym('virtual_speed')
        curvature = ca.SX.sym('curvature')
        fl = ca.SX.sym('fl')
        fr = ca.SX.sym('fr')

        gravity = ca.SX.sym('gravity')

        vx_rate = ca.SX.sym('vx_rate')
        wz_rate = ca.SX.sym('wz_rate')
        virtual_speed_rate = ca.SX.sym('virtual_speed_rate')

        fl_rate = ca.SX.sym('fl_rate')
        fr_rate = ca.SX.sym('fr_rate')
        virtual_speed_rate = ca.SX.sym('virtual_speed_rate')

        """ 2D Rotation matrices """
        #self.TransRotationMatrix_2d = ca.Function( 'transRotMatrix_2d', [yaw], [ ca.vertcat( ca.horzcat( ca.cos(yaw), -ca.sin(yaw) ),
        #                                                                                     ca.horzcat( ca.sin(yaw),  ca.cos(yaw) ) ) ] )

        d_kinematics = ca.vertcat( ca.cos(pitch) * ca.cos(diff_yaw) * vx - virtual_speed * (1 - curvature * y1),\
                                   ca.cos(pitch) * ca.sin(diff_yaw) * vx - virtual_speed * curvature * s1,\
                                   wz - curvature * virtual_speed,\
                                   virtual_speed )

        d_kinematics_rates = ca.vertcat( ca.cos(pitch) * ca.cos(diff_yaw) * vx - virtual_speed * (1 - curvature * y1),\
                                         ca.cos(pitch) * ca.sin(diff_yaw) * vx - virtual_speed * curvature * s1,\
                                         wz - curvature * virtual_speed,\
                                         virtual_speed,\
                                         vx_rate,\
                                         wz_rate,\
                                         virtual_speed_rate )
    
        d_kinematics_rates_std = ca.vertcat( ca.cos(pitch) * ca.cos(yaw) * vx,\
                                             ca.cos(pitch) * ca.sin(yaw) * vx,\
                                             wz,\
                                             virtual_speed,\
                                             vx_rate,\
                                             wz_rate,\
                                             virtual_speed_rate )
        
        d_dynamics = ca.vertcat( ca.cos(pitch) * ca.cos(diff_yaw) * vx - virtual_speed * (1 - curvature * y1),\
                                 ca.cos(pitch) * ca.sin(diff_yaw) * vx - virtual_speed * curvature * s1,\
                                 wz - curvature * virtual_speed,\
                                 virtual_speed,\
                                 2 * (fl + fr) / vehicle_param["m"] - ca.sin(pitch) * vehicle_param["m"] * self.gz,\
                                 2 * (fr - fl) * lat_w / ( 2 * vehicle_param["izz"] ) )
        
        d_dynamics_path = ca.vertcat( ca.cos(pitch) * ca.cos(diff_yaw) * vx - virtual_speed * (1 - curvature * y1),\
                                        ca.cos(pitch) * ca.sin(diff_yaw) * vx - virtual_speed * curvature * s1,\
                                        wz - curvature * virtual_speed,\
                                        (fl + fr) / vehicle_param["m"] - ca.sin(pitch) * vehicle_param["m"] * self.gz,\
                                        (fr - fl) * lat_w / ( 2 * vehicle_param["izz"] ) )
        
        d_dynamics_path_std = ca.vertcat( ca.cos(pitch) * ca.cos(yaw) * vx,\
                                          ca.cos(pitch) * ca.sin(yaw) * vx,\
                                          wz,\
                                          2 * (fl + fr) / vehicle_param["m"] - ca.sin(pitch) * vehicle_param["m"] * self.gz,\
                                          2 * (fr - fl) * lat_w / ( 2 * vehicle_param["izz"] ) )

        """ Kinematics """
        self.kinematics = ca.Function( 'kinematics', [s1, y1, diff_yaw, curvature, vx, wz, virtual_speed, pitch], [d_kinematics] )

        """ Kinematics rates """
        self.kinematics_rates = ca.Function( 'kinematics_rates', [s1, y1, diff_yaw, curvature, vx, wz, virtual_speed, vx_rate, wz_rate, virtual_speed_rate, pitch], [ d_kinematics_rates ] )

        """ Standard kinematics with velocity rates """
        self.kinematics_rates_std = ca.Function( 'kinematics_rates_std', [x, y, yaw, vx, wz, virtual_speed, vx_rate, wz_rate, virtual_speed_rate, pitch], [ d_kinematics_rates_std ] )

        """ Dynamics """
        self.dynamics = ca.Function( 'dynamics', [s1, y1, diff_yaw, curvature, vx, wz, fl, fr, virtual_speed, gravity, pitch], [ d_dynamics ] )

        """ Dynamics with force rates path """
        self.dynamics_path = ca.Function( 'dynamics_path', [s1, y1, diff_yaw, curvature, vx, wz, fl, fr, virtual_speed, gravity, pitch], [ d_dynamics_path ] )

        """ Dynamics with force rates path  """
        self.dynamics_path_std = ca.Function( 'dynamics_path_std', [x, y, yaw, vx, wz, fl, fr, pitch, roll, friction], [ d_dynamics_path_std ] )

#   Unicycle kinematics model
class KinematicsRatesFrenet(ModelParameters, Common):
    def __init__(self, curvature):
        
        super().__init__()
        
        with open(self.frenet_serret_kin_skid) as f:
            solver_param = json.load(f)
            specs = solver_param["solver_specs"]
            self.con_pose = solver_param["con_pose"]
            self.con_vel = solver_param["con_vel"]
            self.con_rates = solver_param["con_rates"]
            self.weights = solver_param["weights"]

            virtual_speed_lb = solver_param["virtual_speed_lb"]
            virtual_speed_ub = solver_param["virtual_speed_ub"]
        
        #   Work with Leo Rover
        with open(self.vehicle_specs) as f:
            vehicle_param = json.load(f)
            lat_w = vehicle_param["wheelLatSeparation"]
        
        self.opti = ca.Opti()

        #   States
        self.s1  = self.opti.variable( specs["N"] + 1 )
        self.y1  = self.opti.variable( specs["N"] + 1 )
        self.diff_yaw = self.opti.variable( specs["N"] + 1 )
        self.progress  = self.opti.variable( specs["N"] + 1 )
        self.vx = self.opti.variable( specs["N"] + 1 )
        self.wz = self.opti.variable( specs["N"] + 1 )
        self.virtual_speed = self.opti.variable( specs["N"] + 1 )
        
        #   Controls
        self.vx_rate = self.opti.variable( specs["N"] )
        self.wz_rate = self.opti.variable( specs["N"] )
        self.virtual_speed_rate = self.opti.variable( specs["N"] )

        #   Set initial state parameters
        self.s1_0  = self.opti.parameter()
        self.y1_0  = self.opti.parameter()
        self.diff_yaw_0 = self.opti.parameter()
        self.progress_0 = self.opti.parameter()
        self.vx_0 = self.opti.parameter()
        self.wz_0 = self.opti.parameter()
        self.virtual_speed_0 = self.opti.parameter()
        self.pitch = self.opti.parameter()

        #   Initial constraints
        self.opti.subject_to(self.s1[0] == self.s1_0)
        self.opti.subject_to(self.y1[0] == self.y1_0)
        self.opti.subject_to(self.diff_yaw[0] == self.diff_yaw_0)
        self.opti.subject_to(self.progress[0] == self.progress_0)
        self.opti.subject_to(self.vx[0] == self.vx_0)
        self.opti.subject_to(self.wz[0] == self.wz_0)
        self.opti.subject_to(self.virtual_speed[0] == self.virtual_speed_0)

        #   RK4 integrator
        for k in range( specs["N"] ):
            x_next = ca.vertcat( self.s1[k + 1], self.y1[k + 1], self.diff_yaw[k + 1], self.progress[k + 1], self.vx[k + 1], self.wz[k + 1], self.virtual_speed[k + 1] )
            x_now  = ca.vertcat( self.s1[k], self.y1[k], self.diff_yaw[k], self.progress[k], self.vx[k], self.wz[k], self.virtual_speed[k] )

            k1 = self.kinematics_rates( self.s1[k],\
                                        self.y1[k],\
                                        self.diff_yaw[k],\
                                        curvature( self.progress[k] ),\
                                        self.vx[k],\
                                        self.wz[k],\
                                        self.virtual_speed[k],\
                                        self.vx_rate[k],\
                                        self.wz_rate[k],\
                                        self.virtual_speed_rate[k],\
                                        self.pitch )

            k2 = self.kinematics_rates( self.s1[k] + specs["Ts"] / 2 * k1[0],\
                                     self.y1[k] + specs["Ts"] / 2 * k1[1],\
                                     self.diff_yaw[k] + specs["Ts"] / 2 * k1[2],\
                                     curvature( self.progress[k] + specs["Ts"] / 2 * k1[3] ),\
                                     self.vx[k] + specs["Ts"] / 2 * k1[4],\
                                     self.wz[k] + specs["Ts"] / 2 * k1[5],\
                                     self.virtual_speed[k] + specs["Ts"] / 2 * k1[6],\
                                     self.vx_rate[k],\
                                     self.wz_rate[k],\
                                     self.virtual_speed_rate[k],\
                                     self.pitch )
            
            k3 = self.kinematics_rates( self.s1[k] + specs["Ts"] / 2 * k2[0],\
                                     self.y1[k] + specs["Ts"] / 2 * k2[1],\
                                     self.diff_yaw[k] + specs["Ts"] / 2 * k2[2],\
                                     curvature( self.progress[k] + specs["Ts"] / 2 * k2[3] ),\
                                     self.vx[k] + specs["Ts"] / 2 * k2[4],\
                                     self.wz[k] + specs["Ts"] / 2 * k2[5],\
                                     self.virtual_speed[k] + specs["Ts"] / 2 * k2[6],\
                                     self.vx_rate[k],\
                                     self.wz_rate[k],\
                                     self.virtual_speed_rate[k],\
                                     self.pitch )
            
            k4 = self.kinematics_rates( self.s1[k] + specs["Ts"] * k3[0],\
                                     self.y1[k] + specs["Ts"] * k3[1],\
                                     self.diff_yaw[k] + specs["Ts"] * k3[2],\
                                     curvature( self.progress[k] + specs["Ts"] * k3[3] ),\
                                     self.vx[k] + specs["Ts"] * k3[4],\
                                     self.wz[k] + specs["Ts"] * k3[5],\
                                     self.virtual_speed[k] + specs["Ts"] * k3[6],\
                                     self.vx_rate[k],\
                                     self.wz_rate[k],\
                                     self.virtual_speed_rate[k],\
                                     self.pitch )

            x_next_pred = x_now + (specs["Ts"] / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
            
            self.opti.subject_to(x_next == x_next_pred)

        #   Set cost function
        J = 0
        
        for k in range(specs["N"] + 1):
            if( k < specs["N"] ):
                err = ca.vertcat( self.s1[k], self.y1[k], self.diff_yaw[k], self.vx_rate[k], self.wz_rate[k], self.virtual_speed_rate[k], self.virtual_speed[k] - 5e-1 )
                J += err.T @ ca.diag( ca.vertcat( self.weights["Q_s1"], self.weights["Q_y1"], self.weights["Q_diff_yaw"],\
                                                  self.weights["Q_u_rate"], self.weights["Q_r_rate"], self.weights["Q_virtual_speed_rate"], self.weights["Q_virtual_speed"] ) ) @ err

            else:
                err = ca.vertcat( self.s1[k], self.y1[k], self.diff_yaw[k], self.virtual_speed[k] - 5e-1 )
                J += err.T @ ca.diag( ca.vertcat( self.weights["Q_s1_t"], self.weights["Q_y1_t"], self.weights["Q_diff_yaw_t"], self.weights["Q_virtual_speed"] ) ) @ err

        self.opti.minimize(J)

        #   Constraints
        self.opti.subject_to( self.opti.bounded( self.con_vel["u_lb"], self.vx, self.con_vel["u_ub"] ) )
        self.opti.subject_to( self.opti.bounded( self.con_vel["r_lb"], self.wz, self.con_vel["r_ub"] ) )
        self.opti.subject_to( self.opti.bounded( self.con_rates["u_rate_lb"], self.vx, self.con_rates["u_rate_ub"] ) )
        self.opti.subject_to( self.opti.bounded( self.con_rates["r_rate_lb"], self.wz, self.con_rates["r_rate_ub"] ) )
        self.opti.subject_to( self.opti.bounded( virtual_speed_lb, self.virtual_speed, virtual_speed_ub ) )

        #   Setup solver
        opts = {"ipopt.print_level": 1, "print_time": True}
        #opts = {'print_header': True, 'qpsol': 'qpoases'}
        self.opti.solver("ipopt", opts)

    def _setInitialState(self, initialState, pitch):

        self.opti.set_value(self.s1_0, initialState[0] )
        self.opti.set_value(self.y1_0, initialState[1] )
        self.opti.set_value(self.diff_yaw_0, initialState[2] )
        self.opti.set_value(self.progress_0, initialState[3] )
        self.opti.set_value(self.vx_0, initialState[4] )
        self.opti.set_value(self.wz_0, initialState[5] )
        self.opti.set_value(self.virtual_speed_0, initialState[6] )
        self.opti.set_value(self.pitch, pitch)

    def _setInitialGuess(self, lastSol):

        self.opti.set_initial( self.s1, np.append( lastSol[0][1:], lastSol[0][-1] ) )
        self.opti.set_initial( self.y1, np.append( lastSol[1][1:], lastSol[1][-1] ) )
        self.opti.set_initial( self.diff_yaw, np.append( lastSol[2][1:], lastSol[2][-1] ) )
        self.opti.set_initial( self.progress, np.append( lastSol[3][1:], lastSol[3][-1] ) )
        self.opti.set_initial( self.vx, np.append( lastSol[4][1:], lastSol[4][-1] ) )
        self.opti.set_initial( self.wz, np.append( lastSol[5][1:], lastSol[5][-1] ) )
        self.opti.set_initial( self.virtual_speed, np.append( lastSol[6][1:], lastSol[6][-1] ) )
        self.opti.set_initial( self.vx_rate, np.append( lastSol[7][1:], lastSol[7][-1] ) )
        self.opti.set_initial( self.wz_rate, np.append( lastSol[8][1:], lastSol[8][-1] ) )
        self.opti.set_initial( self.virtual_speed_rate, np.append( lastSol[9][1:], lastSol[9][-1] ) )

    def _solve(self, initialState, prevSol, pitch):

        if( prevSol is None ):
            pass
        
        else:
            self._setInitialGuess(prevSol)
        
        self._setInitialState(initialState, pitch)

        #   Solve optimization problem
        sol = self.opti.solve()

        # Extract solution
        s1_sol  = sol.value(self.s1)
        y1_sol  = sol.value(self.y1)
        diff_yaw_sol = sol.value(self.diff_yaw)
        progress_sol = sol.value(self.progress)
        vx_sol = sol.value(self.vx)
        wz_sol = sol.value(self.wz)
        virtual_speed_sol = sol.value(self.virtual_speed)
        vx_rate_sol = sol.value(self.vx_rate)
        wz_rate_sol = sol.value(self.wz_rate)
        virtual_speed_rate_sol = sol.value(self.virtual_speed_rate)

        return [s1_sol, y1_sol, diff_yaw_sol, progress_sol, vx_sol, wz_sol, virtual_speed_sol, vx_rate_sol, wz_rate_sol, virtual_speed_rate_sol]

    def _getStats(self):

        return self.opti.stats()

#   Unicycle dynamics model with virtual speed control
class DynamicsFrenet(ModelParameters, Common):
    def __init__(self, curvature):
        
        super().__init__()
        
        with open(self.frenet_serret_dyn_skid) as f:
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
        self.s1  = self.opti.variable( specs["N"] + 1 )
        self.y1  = self.opti.variable( specs["N"] + 1 )
        self.diff_yaw = self.opti.variable( specs["N"] + 1 )
        self.progress  = self.opti.variable( specs["N"] + 1 )
        self.vx = self.opti.variable( specs["N"] + 1 )
        self.wz = self.opti.variable( specs["N"] + 1 )
        
        #   Controls
        self.fl = self.opti.variable( specs["N"] )
        self.fr = self.opti.variable( specs["N"] )
        self.virtual_speed = self.opti.variable( specs["N"] )

        #   Set initial state parameters
        self.s1_0  = self.opti.parameter()
        self.y1_0  = self.opti.parameter()
        self.diff_yaw_0 = self.opti.parameter()
        self.progress_0  = self.opti.parameter()
        self.vx_0 = self.opti.parameter()
        self.wz_0 = self.opti.parameter()
        self.fl_prev = self.opti.parameter()
        self.fr_prev = self.opti.parameter()
        self.pitch = self.opti.parameter()
        self.roll = self.opti.parameter()
        self.niu = self.opti.parameter()

        self.fl_min = self.opti.parameter()
        self.fl_max = self.opti.parameter()

        self.fr_min = self.opti.parameter()
        self.fr_max = self.opti.parameter()

        #   Initial constraints
        self.opti.subject_to(self.s1[0] == self.s1_0)
        self.opti.subject_to(self.y1[0] == self.y1_0)
        self.opti.subject_to(self.diff_yaw[0] == self.diff_yaw_0)
        self.opti.subject_to(self.progress[0] == self.progress_0)
        self.opti.subject_to(self.vx[0] == self.vx_0)
        self.opti.subject_to(self.wz[0] == self.wz_0)
        
        #   RK4 integrator
        for k in range( specs["N"] ):
            x_next = ca.vertcat( self.s1[k + 1], self.y1[k + 1], self.diff_yaw[k + 1], self.progress[k + 1], self.vx[k + 1], self.wz[k + 1] )
            x_now  = ca.vertcat( self.s1[k], self.y1[k], self.diff_yaw[k], self.progress[k], self.vx[k], self.wz[k] )

            k1 = self.dynamics( self.s1[k],\
                                self.y1[k],\
                                self.diff_yaw[k],\
                                curvature( self.progress[k] ),\
                                self.vx[k],\
                                self.wz[k],\
                                self.fl[k],\
                                self.fr[k],\
                                self.virtual_speed[k],\
                                self.gz,\
                                self.pitch )

            k2 = self.dynamics( self.s1[k] + specs["Ts"] / 2 * k1[0],\
                                self.y1[k] + specs["Ts"] / 2 * k1[1],\
                                self.diff_yaw[k] + specs["Ts"] / 2 * k1[2],\
                                curvature( self.progress[k] + specs["Ts"] / 2 * k1[3] ),\
                                self.vx[k] + specs["Ts"] / 2 * k1[4],\
                                self.wz[k] + specs["Ts"] / 2 * k1[5],\
                                self.fl[k],\
                                self.fr[k],\
                                self.virtual_speed[k],\
                                self.gz,\
                                self.pitch )
            
            k3 = self.dynamics( self.s1[k] + specs["Ts"] / 2 * k2[0],\
                                self.y1[k] + specs["Ts"] / 2 * k2[1],\
                                self.diff_yaw[k] + specs["Ts"] / 2 * k2[2],\
                                curvature( self.progress[k] + specs["Ts"] / 2 * k2[3] ),\
                                self.vx[k] + specs["Ts"] / 2 * k2[4],\
                                self.wz[k] + specs["Ts"] / 2 * k2[5],\
                                self.fl[k],\
                                self.fr[k],\
                                self.virtual_speed[k],\
                                self.gz,\
                                self.pitch )
            
            k4 = self.dynamics( self.s1[k] + specs["Ts"] * k3[0],\
                                self.y1[k] + specs["Ts"] * k3[1],\
                                self.diff_yaw[k] + specs["Ts"] * k3[2],\
                                curvature( self.progress[k] + specs["Ts"] * k3[3] ),\
                                self.vx[k] + specs["Ts"] * k3[4],\
                                self.wz[k] + specs["Ts"] * k3[5],\
                                self.fl[k],\
                                self.fr[k],\
                                self.virtual_speed[k],\
                                self.gz,\
                                self.pitch )

            x_next_pred = x_now + (specs["Ts"] / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
            
            self.opti.subject_to(x_next == x_next_pred)

        #   Cost function
        J = 0
        
        for k in range( specs["N"] + 1 ):

            if( k == 0 ):
                err = ca.vertcat( self.s1[k],\
                                  self.y1[k],\
                                  self.fl[k] - self.fl_prev,\
                                  self.fr[k] - self.fr_prev,\
                                  self.virtual_speed[k] - 5e-1 )
                
                J += err.T @ ca.diag( ca.vertcat( self.weights["Q_s1"],\
                                                  self.weights["Q_y1"],\
                                                  self.weights["Q_fl_rate"],\
                                                  self.weights["Q_fr_rate"],\
                                                  self.weights["Q_virtual_speed"] ) ) @ err

            elif( k < specs["N"] and k > 0 ):
                err = ca.vertcat( self.s1[k],\
                                  self.y1[k],\
                                  self.fl[k] - self.fl[k - 1],\
                                  self.fr[k] - self.fr[k - 1],\
                                  self.virtual_speed[k] - 5e-1 )
                
                J += err.T @ ca.diag( ca.vertcat( self.weights["Q_s1"],\
                                                  self.weights["Q_y1"],\
                                                  self.weights["Q_fl_rate"],\
                                                  self.weights["Q_fr_rate"],\
                                                  self.weights["Q_virtual_speed"] ) ) @ err

            else:
                err = ca.vertcat( self.s1[k], self.y1[k] )
                J += err.T @ ca.diag( ca.vertcat( self.weights["Q_s1_t"], self.weights["Q_y1_t"] ) ) @ err

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
        self.opti.set_value( self.s1_0, initialState[0] )
        self.opti.set_value( self.y1_0, initialState[1] )
        self.opti.set_value( self.diff_yaw_0, initialState[2] )
        self.opti.set_value( self.progress_0, initialState[3] )
        self.opti.set_value( self.vx_0, initialState[4] )
        self.opti.set_value( self.wz_0, initialState[5] )
        self.opti.set_value( self.pitch, parameters["pitch"] )
        self.opti.set_value( self.roll, parameters["roll"] )
        self.opti.set_value( self.niu, parameters["friction"] )
        self.opti.set_value( self.fl_prev, parameters["fl_prev"] )
        self.opti.set_value( self.fr_prev, parameters["fr_prev"] )

        self.opti.set_value( self.fl_min, parameters["fl_min"] )
        self.opti.set_value( self.fl_max, parameters["fl_max"] )

        self.opti.set_value( self.fr_min, parameters["fr_min"] )
        self.opti.set_value( self.fr_max, parameters["fr_max"] )

    def _setInitialGuess(self, lastSol):

        self.opti.set_initial( self.s1, np.append( lastSol[0][1:], lastSol[0][-1] ) )
        self.opti.set_initial( self.y1, np.append( lastSol[1][1:], lastSol[1][-1] ) )
        self.opti.set_initial( self.diff_yaw, np.append( lastSol[2][1:], lastSol[2][-1] ) )
        self.opti.set_initial( self.progress, np.append( lastSol[3][1:], lastSol[3][-1] ) )
        self.opti.set_initial( self.vx, np.append( lastSol[4][1:], lastSol[4][-1] ) )
        self.opti.set_initial( self.wz, np.append( lastSol[5][1:], lastSol[5][-1] ) )
        self.opti.set_initial( self.fl, np.append( lastSol[6][1:], lastSol[6][-1] ) )
        self.opti.set_initial( self.fr, np.append( lastSol[7][1:], lastSol[7][-1] ) )
        self.opti.set_initial( self.virtual_speed, np.append( lastSol[8][1:], lastSol[8][-1] ) )
    
    def _getStats(self):
        return self.opti.stats()
    
    def _solve(self, initialState, prevSol, parameters):
        
        """ 
            Solve optimization problem

            :initialState   initial state of the state variables
            :prevSol    solution of the previous optimization problem
            :pitch  current pitch of the mobile robot
            :roll   current roll angle of the mobile robot
            :friction    friction coefficient considered for the mobile robot
        """

        if( prevSol is None ):
            pass
        
        else:
            self._setInitialGuess(prevSol)

        self._setInitialState(initialState, parameters)

        #   Solve optimization problem
        sol = self.opti.solve()

        # Extract solution
        s1_sol  = sol.value(self.s1)
        y1_sol  = sol.value(self.y1)
        diff_yaw_sol = sol.value(self.diff_yaw)
        progress_sol = sol.value(self.progress)
        vx_sol = sol.value(self.vx)
        wz_sol = sol.value(self.wz)
        fl_sol = sol.value(self.fl)
        fr_sol = sol.value(self.fr)
        virtual_speed_sol = sol.value(self.virtual_speed)

        return [s1_sol, y1_sol, diff_yaw_sol, progress_sol, vx_sol, wz_sol, fl_sol, fr_sol, virtual_speed_sol]

#   Parameterized unicycle dynamics model with rates
class DynamicsFrenetPath(ModelParameters, Common):
    def __init__(self):
        
        super().__init__()

        with open(self.frenet_serret_dyn_skid) as f:
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
        self.s1 = self.opti.variable( self.specs["N"] + 1 )
        self.y1 = self.opti.variable( self.specs["N"] + 1 )
        self.diff_yaw = self.opti.variable( self.specs["N"] + 1 )
        self.vx = self.opti.variable( self.specs["N"] + 1 )
        self.wz = self.opti.variable( self.specs["N"] + 1 )

        #   Controls
        self.fl = self.opti.variable( self.specs["N"] )
        self.fr = self.opti.variable( self.specs["N"] )

        #   Set initial state parameters
        self.s1_0 = self.opti.parameter()
        self.y1_0 = self.opti.parameter()
        self.diff_yaw_0 = self.opti.parameter()
        self.vx_0 = self.opti.parameter()
        self.wz_0 = self.opti.parameter()
        self.pitch = self.opti.parameter()
        self.roll = self.opti.parameter()
        self.niu = self.opti.parameter()

        self.fl_prev = self.opti.parameter()
        self.fr_prev = self.opti.parameter()

        self.curvature = self.opti.parameter( self.specs["N"] )
        self.virtual_speed = self.opti.parameter( self.specs["N"] )

        #   Initial constraints
        self.opti.subject_to(self.s1[0] == self.s1_0)
        self.opti.subject_to(self.y1[0] == self.y1_0)
        self.opti.subject_to(self.diff_yaw[0] == self.diff_yaw_0)
        self.opti.subject_to(self.vx[0] == self.vx_0)
        self.opti.subject_to(self.wz[0] == self.wz_0)
        
        #   RK4 integrator
        for k in range( self.specs["N"] ):
            x_next = ca.vertcat( self.s1[k + 1], self.y1[k + 1], self.diff_yaw[k + 1], self.vx[k + 1], self.wz[k + 1] )
            x_now  = ca.vertcat( self.s1[k], self.y1[k], self.diff_yaw[k], self.vx[k], self.wz[k] )
            
            k1 = self.dynamics_path(self.s1[k],\
                                    self.y1[k],\
                                    self.diff_yaw[k],\
                                    self.curvature[k],\
                                    self.vx[k],\
                                    self.wz[k],\
                                    self.fl[k],\
                                    self.fr[k],\
                                    self.virtual_speed[k],\
                                    self.gz,\
                                    self.pitch )

            k2 = self.dynamics_path(self.s1[k] + self.specs["Ts"] / 2 * k1[0],\
                                    self.y1[k] + self.specs["Ts"] / 2 * k1[1],\
                                    self.diff_yaw[k] + self.specs["Ts"] / 2 * k1[2],\
                                    self.curvature[k],\
                                    self.vx[k] + self.specs["Ts"] / 2 * k1[3],\
                                    self.wz[k] + self.specs["Ts"] / 2 * k1[4],\
                                    self.fl[k],\
                                    self.fr[k],\
                                    self.virtual_speed[k],\
                                    self.gz,\
                                    self.pitch )
            
            k3 = self.dynamics_path(self.s1[k] + self.specs["Ts"] / 2 * k2[0],\
                                    self.y1[k] + self.specs["Ts"] / 2 * k2[1],\
                                    self.diff_yaw[k] + self.specs["Ts"] / 2 * k2[2],\
                                    self.curvature[k],\
                                    self.vx[k] + self.specs["Ts"] / 2 * k2[3],\
                                    self.wz[k] + self.specs["Ts"] / 2 * k2[4],\
                                    self.fl[k],\
                                    self.fr[k],\
                                    self.virtual_speed[k],\
                                    self.gz,\
                                    self.pitch )
            
            k4 = self.dynamics_path(self.s1[k] + self.specs["Ts"] * k3[0],\
                                    self.y1[k] + self.specs["Ts"] * k3[1],\
                                    self.diff_yaw[k] + self.specs["Ts"] * k3[2],\
                                    self.curvature[k],\
                                    self.vx[k] + self.specs["Ts"] * k3[3],\
                                    self.wz[k] + self.specs["Ts"] * k3[4],\
                                    self.fl[k],\
                                    self.fr[k],\
                                    self.virtual_speed[k],\
                                    self.gz,\
                                    self.pitch )

            x_next_pred = x_now + (self.specs["Ts"] / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
            
            self.opti.subject_to(x_next == x_next_pred)

        #   Cost function
        J = 0
        
        for k in range(self.specs["N"] + 1):

            if( k == 0 ):
                err = ca.vertcat( self.s1[k],\
                                  self.y1[k],\
                                  self.fl[k] - self.fl_prev,\
                                  self.fr[k] - self.fr_prev )
                
                J += err.T @ ca.diag( ca.vertcat( self.weights["Q_s1"], self.weights["Q_y1"], self.weights["Q_fl_rate"], self.weights["Q_fr_rate"] ) ) @ err

            if( k < self.specs["N"] ):
                err = ca.vertcat( self.s1[k],\
                                  self.y1[k],\
                                  self.fl[k] - self.fl[k - 1],\
                                  self.fr[k] - self.fr[k - 1] )
                
                J += err.T @ ca.diag( ca.vertcat( self.weights["Q_s1"], self.weights["Q_y1"], self.weights["Q_fl_rate"], self.weights["Q_fr_rate"] ) ) @ err

            else:
                err = ca.vertcat( self.s1[k],\
                                  self.y1[k] )
                
                J += err.T @ ca.diag( ca.vertcat( self.weights["Q_s1_t"], self.weights["Q_y1_t"] ) ) @ err

        self.opti.minimize(J)

        #   Constraints
        self.opti.subject_to( self.opti.bounded( self.con_vel["u_lb"], self.vx, self.con_vel["u_ub"] ) )
        self.opti.subject_to( self.opti.bounded( self.con_vel["r_lb"], self.wz, self.con_vel["r_ub"] ) )
        #self.opti.subject_to( self.opti.bounded( 0, self.fl**2, ( self.niu * vehicle_param["m"] * self.gz * ca.cos(self.pitch) * ca.cos(self.roll) )**2 ) )
        #self.opti.subject_to( self.opti.bounded( 0, self.fr**2, ( self.niu * vehicle_param["m"] * self.gz * ca.cos(self.pitch) * ca.cos(self.roll) )**2 ) )

        #   Setup solver
        opts = {"ipopt.print_level": 0, "print_time": True, "ipopt.max_iter": 1000, "ipopt.tol": 1e-6, "ipopt.acceptable_tol": 1e-4}
        self.opti.solver("ipopt", opts)

    def _setInitialState(self, initialState, pitch, roll, niu, curvature, virtual_speed, fl_prev, fr_prev):
        
        #   Set initial state
        self.opti.set_value( self.s1_0, initialState[0] )
        self.opti.set_value( self.y1_0, initialState[1] )
        self.opti.set_value( self.diff_yaw_0, initialState[2] )
        self.opti.set_value( self.vx_0, initialState[3] )
        self.opti.set_value( self.wz_0, initialState[4] )
        self.opti.set_value( self.niu, niu )
        self.opti.set_value( self.pitch, pitch )
        self.opti.set_value( self.roll, roll )
        self.opti.set_value( self.fl_prev, fl_prev )
        self.opti.set_value( self.fr_prev, fr_prev )

        index = 0

        while index < self.specs["N"]:
            self.opti.set_value( self.curvature[index], curvature[index] )
            self.opti.set_value( self.virtual_speed[index], virtual_speed[index] )

            index += 1

    def _setInitialGuess(self, lastSol):

        self.opti.set_initial( self.s1, np.append( lastSol[0][1:], lastSol[0][-1] ) )
        self.opti.set_initial( self.y1, np.append( lastSol[1][1:], lastSol[1][-1] ) )
        self.opti.set_initial( self.diff_yaw, np.append( lastSol[2][1:], lastSol[2][-1] ) )
        self.opti.set_initial( self.vx, np.append( lastSol[3][1:], lastSol[3][-1] ) )
        self.opti.set_initial( self.wz, np.append( lastSol[4][1:], lastSol[4][-1] ) )
        self.opti.set_initial( self.fl, np.append( lastSol[5][1:], lastSol[5][-1] ) )
        self.opti.set_initial( self.fr, np.append( lastSol[6][1:], lastSol[6][-1] ) )

    def _getStats(self):
        print(self.opti.stats())
    
    def _solve(self, initialState, prevSol, pitch, roll, niu, curvature, virtual_speed, fl_prev, fr_prev):
        
        if( prevSol is None ):
            pass
        
        else:
            self._setInitialGuess(prevSol)

        self._setInitialState(initialState, pitch, roll, niu, curvature, virtual_speed, fl_prev, fr_prev)

        #   Solve optimization problem
        sol = self.opti.solve()

        # Extract solution
        s1_sol  = sol.value(self.s1)
        y1_sol  = sol.value(self.y1)
        diff_yaw_sol = sol.value(self.diff_yaw)
        vx_sol = sol.value(self.vx)
        wz_sol = sol.value(self.wz)
        fl_sol = sol.value(self.fl)
        fr_sol = sol.value(self.fr)

        return [s1_sol, y1_sol, diff_yaw_sol, vx_sol, wz_sol, fl_sol, fr_sol]
    