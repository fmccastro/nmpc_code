#!/usr/bin/python3.8

import sys

import scipy.linalg
sys.path.insert(0, "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/src")

from classes.all_imports import *
from classes.common_class import *

"""
    Build and define NMPC optimization problem with the countoring formulation

    Drawbacks: 
                > it is more cumbersome to introduce references where yaw change along its full domain
                  like 8-shape trajectories.
    
    TO DO:
            > there is improvement to be done on sharp turns (solver may fail)
            > yet to include coulomb friction cone
            > yet to test with test-bed vehicle (changes to the model to be done)
"""

#   Optimization problem parameters
class ModelParameters():

    #   Constructor
    def __init__(self):

        super().__init__()

        """with open(self.mpcc_dyn_skidsteering) as f:
            solver_param = json.load(f)
            specs = solver_param["solver_specs"]
            self.con_pose = solver_param["con_pose"]
            self.con_vel = solver_param["con_vel"]
            self.weights = solver_param["weights"]"""
        
        #   Work with Leo Rover
        with open(self.vehicle_specs) as f:
            vehicle_param = json.load(f)
            lat_w = vehicle_param["wheelLatSeparation"]
        
        ###   Optimization symbolic variables   ############################

        """
            Mobile robot states
        """

        vx = ca.SX.sym('vx')
        vy = ca.SX.sym('vy')
        wz = ca.SX.sym('wz')
        yaw = ca.SX.sym('yaw')
        virtual_speed = ca.SX.sym('virtual_speed')

        fl = ca.SX.sym('fl')
        fr = ca.SX.sym('fr')

        fl_rate = ca.SX.sym('fl_rate')
        fr_rate = ca.SX.sym('fr_rate')

        """ 2D Rotation matrices """
        self.TransRotationMatrix_2d = ca.Function( 'transRotMatrix_2d', [yaw], [ ca.vertcat( ca.horzcat( ca.cos(yaw), -ca.sin(yaw) ),
                                                                                       ca.horzcat( ca.sin(yaw),  ca.cos(yaw) ) ) ] )

        """ 2D Kinematics """
        self.kinematics_2d = ca.Function( 'kinematics_2d', [yaw, vx, wz, virtual_speed], [ ca.vertcat( self.TransRotationMatrix_2d(yaw) @ ca.vertcat(vx, 0.0), wz, virtual_speed ) ] )

        """ 2D Dynamics """
        self.dynamics_2d = ca.Function( 'dynamics_2d', [yaw, vx, vy, wz, fl, fr, fl_rate, fr_rate, virtual_speed],\
                                                      [ ca.vertcat( self.TransRotationMatrix_2d(yaw) @ ca.vertcat(vx, vy), wz, virtual_speed,\
                                                                    wz * vy + (fl + fr) / vehicle_param["m"], -wz * vx, (fr - fl) * lat_w / ( 2 * vehicle_param["izz"] ),\
                                                                    fl_rate, fr_rate ) ] )

#   Unicycle kinematics integrator
class IntegratorKinematicsSkidSteeringPlanar(ModelParameters, Common):
    def __init__(self):
        
        super().__init__()
        
        with open(self.mpcc_kin_skidsteering) as f:
            solver_param = json.load(f)
            specs = solver_param["solver_specs"]
            #self.con_pose = solver_param["con_pose"]
            #self.con_vel = solver_param["con_vel"]
            #self.weights = solver_param["weights"]
        
        #   Work with Leo Rover
        #with open(self.vehicle_specs) as f:
        #    vehicle_param = json.load(f)
        #    lat_w = vehicle_param["wheelLatSeparation"]
        
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        yaw = ca.SX.sym('yaw')
        progress = ca.SX.sym('progress')

        x_dot = ca.SX.sym('x_dot')
        y_dot = ca.SX.sym('y_dot')
        yaw_dot = ca.SX.sym('yaw_dot')
        progress_dot = ca.SX.sym('progress_dot')

        vx = ca.SX.sym('vx')
        wz = ca.SX.sym('wz')
        virtual_speed = ca.SX.sym('virtual_speed')

        #   State
        state = ca.vertcat(x, y, yaw, progress)
        
        #   State derivative
        state_dot = ca.vertcat(x_dot, y_dot, yaw_dot, progress_dot)
        
        #   Controls
        controls = ca.vertcat(vx, wz, virtual_speed)

        #   Explicit model
        f_expl = ca.vertcat( self.TransRotationMatrix_2d(yaw) @ ca.vertcat(vx, 0.0),\
                             wz,\
                             virtual_speed )
        
        #   Implicit model
        f_impl = ca.vertcat( ca.vertcat(x_dot, y_dot) - self.TransRotationMatrix_2d(yaw) @ ca.vertcat(vx, 0.0),\
                             yaw_dot - wz,\
                             progress_dot - virtual_speed )
        
        #   Call model instance
        model = AcadosModel()
        
        model.f_impl_expr = f_impl
        model.f_expl_expr = f_expl
        model.x = state
        model.xdot = state_dot
        model.u = controls
        model.name = "integrator_kinematics_skid"

        model.x_labels = [r'$x$ [m]', r'$y$ [m]', r'$\psi$ [rad]', r'progress'] 
        model.u_labels = [r'v_x [m/s]', r'w_z [rad/s]', r'virtual_speed [m/s]']
        model.t_label = '$t$ [s]'

        #   Call dims instance
        dims = AcadosSimDims()

        dims.nx = model.x.rows()
        dims.nu = model.u.rows()
        ###

        #
        solver_options = AcadosSimOptions()
        
        solver_options.T = specs["Ts"]

        #   Call ocp instance
        sim = AcadosSim()

        #   Set ocp
        sim.model = model
        sim.dims = dims
        sim.solver_options = solver_options

        #   Set folder path where generated c code is located
        sim.code_export_directory = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/scripts/acados_c_generated_code/Countoring/SkidSteeringKinematics/acadosSim"
        sim.acados_lib_path = "/home/francisco/acados/lib"
        sim.acados_include_path = "/home/francisco/acados/include"

        json_file_path = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/scripts/acados_c_generated_code/Countoring/SkidSteeringKinematics/acadosSim/acados_sim.json"
        ###

        self.integrator = AcadosSimSolver(sim, json_file = json_file_path)

    def _simulate(self, state, controls):
        return self.integrator.simulate(x = state, u = controls)

#   Unicycle dynamics integrator
class IntegratorDynamicsSkidSteeringPlanar(ModelParameters, Common):
    def __init__(self):
        
        super().__init__()
        
        with open(self.mpcc_kin_skidsteering) as f:
            solver_param = json.load(f)
            specs = solver_param["solver_specs"]
            #self.con_pose = solver_param["con_pose"]
            #self.con_vel = solver_param["con_vel"]
            #self.weights = solver_param["weights"]
        
        #   Work with Leo Rover
        with open(self.vehicle_specs) as f:
            vehicle_param = json.load(f)
            lat_w = vehicle_param["wheelLatSeparation"]
        
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        yaw = ca.SX.sym('yaw')
        progress = ca.SX.sym('progress')
        vx = ca.SX.sym('vx')
        vy = ca.SX.sym('vy')
        wz = ca.SX.sym('wz')

        x_dot = ca.SX.sym('x_dot')
        y_dot = ca.SX.sym('y_dot')
        yaw_dot = ca.SX.sym('yaw_dot')
        progress_dot = ca.SX.sym('progress_dot')
        vx_dot = ca.SX.sym('vx_dot')
        vy_dot = ca.SX.sym('vy_dot')
        wz_dot = ca.SX.sym('wz_dot')

        fl_dot = ca.SX.sym('fl_dot')
        fr_dot = ca.SX.sym('fr_dot')

        fl_rate = ca.SX.sym('fl_rate')
        fr_rate = ca.SX.sym('fr_rate')

        fl = ca.SX.sym('fl')
        fr = ca.SX.sym('fr')
        virtual_speed = ca.SX.sym('virtual_speed')

        #   State
        state = ca.vertcat(x, y, yaw, progress, vx, vy, wz, fl, fr)
        
        #   State derivative
        state_dot = ca.vertcat(x_dot, y_dot, yaw_dot, progress_dot, vx_dot, vy_dot, wz_dot, fl_dot, fr_dot)
        
        #   Controls
        controls = ca.vertcat(fl_rate, fr_rate, virtual_speed)

        d_vx = wz * vy + (fl + fr) / vehicle_param["m"]
        d_vy = -wz * vx
        d_wz = (fr - fl) * lat_w / ( 2 * vehicle_param["izz"] )

        #   Explicit model
        f_expl = ca.vertcat( self.TransRotationMatrix_2d(yaw) @ ca.vertcat(vx, vy),\
                             wz,\
                             virtual_speed,\
                             d_vx,\
                             d_vy,\
                             d_wz,\
                             fl_rate,\
                             fr_rate )
        
        #   Implicit model
        f_impl = ca.vertcat( ca.vertcat(x_dot, y_dot) - self.TransRotationMatrix_2d(yaw) @ ca.vertcat(vx, vy),\
                             yaw_dot - wz,\
                             progress_dot - virtual_speed,\
                             vx_dot - d_vx,\
                             vy_dot - d_vy,\
                             wz_dot - d_wz,\
                             fl_dot - fl_rate,\
                             fr_dot - fr_rate )
        
        #   Call model instance
        model = AcadosModel()
        
        model.f_impl_expr = f_impl
        model.f_expl_expr = f_expl
        model.x = state
        model.xdot = state_dot
        model.u = controls
        model.name = "integrator_dynamics_skid"

        model.x_labels = [r'$x$ [m]', r'$y$ [m]', r'$z$ [m]', r'$\phi$ [rad]', r'$\theta$ [rad]', r'$\psi$ [rad]', r'progress'] 
        model.u_labels = [r'v_x [m/s^2]', r'w_z [rad/s^2]', r'virtual_speed [m/s]']
        model.t_label = '$t$ [s]'

        #   Call dims instance
        dims = AcadosSimDims()

        dims.nx = model.x.rows()
        dims.nu = model.u.rows()
        ###

        #
        solver_options = AcadosSimOptions()
        
        solver_options.T = specs["Ts"]

        #   Call ocp instance
        sim = AcadosSim()

        #   Set ocp
        sim.model = model
        sim.dims = dims
        sim.solver_options = solver_options

        #   Set folder path where generated c code is located
        sim.code_export_directory = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/scripts/acados_c_generated_code/Countoring/SkidSteeringDynamics/acadosSim"
        sim.acados_lib_path = "/home/francisco/acados/lib"
        sim.acados_include_path = "/home/francisco/acados/include"

        json_file_path = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/scripts/acados_c_generated_code/Countoring/SkidSteeringDynamics/acadosSim/acados_sim.json"
        ###

        self.integrator = AcadosSimSolver(sim, json_file = json_file_path)

    def _simulate(self, state, controls):
        return self.integrator.simulate(x = state, u = controls)

#   Unicycle kinematics model
class KinematicsSkidSteeringPlanar(ModelParameters, Common):
    def __init__(self, x_ref, y_ref, yaw_ref):
        
        super().__init__()
        
        with open(self.mpcc_kin_skidsteering) as f:
            solver_param = json.load(f)
            specs = solver_param["solver_specs"]
            self.con_pose = solver_param["con_pose"]
            self.con_vel = solver_param["con_vel"]
            self.weights = solver_param["weights"]

            virtual_speed_lb = solver_param["virtual_speed_lb"]
            virtual_speed_ub = solver_param["virtual_speed_ub"]
        
        #   Work with Leo Rover
        with open(self.vehicle_specs) as f:
            vehicle_param = json.load(f)
            lat_w = vehicle_param["wheelLatSeparation"]
        
        self.opti = ca.Opti()

        #   States
        self.x  = self.opti.variable( specs["N"] + 1 )
        self.y  = self.opti.variable( specs["N"] + 1 )
        self.yaw = self.opti.variable( specs["N"] + 1 )
        self.progress  = self.opti.variable( specs["N"] + 1 )
        
        #   Controls
        self.vx = self.opti.variable( specs["N"] )
        self.wz = self.opti.variable( specs["N"] )
        self.virtual_speed = self.opti.variable( specs["N"] )

        #   Set initial state parameters
        self.x_0  = self.opti.parameter()
        self.y_0  = self.opti.parameter()
        self.yaw_0 = self.opti.parameter()
        self.progress_0  = self.opti.parameter()

        #   Initial constraints
        self.opti.subject_to(self.x[0] == self.x_0)
        self.opti.subject_to(self.y[0] == self.y_0)
        self.opti.subject_to(self.yaw[0] == self.yaw_0)
        self.opti.subject_to(self.progress[0] == self.progress_0)

        #   RK4 integrator
        for k in range( specs["N"] ):
            x_next = ca.vertcat( self.x[k + 1], self.y[k + 1], self.yaw[k + 1], self.progress[k + 1] )
            x_now  = ca.vertcat( self.x[k], self.y[k], self.yaw[k], self.progress[k] )

            k1 = self.kinematics_2d( self.yaw[k], self.vx[k], self.wz[k], self.virtual_speed[k] )
            k2 = self.kinematics_2d( self.yaw[k] + specs["Ts"] / 2 * k1[2], self.vx[k], self.wz[k], self.virtual_speed[k] )
            k3 = self.kinematics_2d( self.yaw[k] + specs["Ts"] / 2 * k2[2], self.vx[k], self.wz[k], self.virtual_speed[k] )
            k4 = self.kinematics_2d( self.yaw[k] + specs["Ts"] * k3[2], self.vx[k], self.wz[k], self.virtual_speed[k] )

            x_next_pred = x_now + (specs["Ts"] / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
            
            self.opti.subject_to(x_next == x_next_pred)

        #   Cost function
        J = 0
        
        for k in range(specs["N"] + 1):
            x_r = x_ref( self.progress[k] )
            y_r = y_ref( self.progress[k] )
            yaw_r = yaw_ref( self.progress[k] )

            if( k < specs["N"] ):
                err = ca.vertcat( self.x[k] - x_r, self.y[k] - y_r, self.yaw[k] - yaw_r, self.virtual_speed[k] - 0.5, self.vx[k], self.wz[k] )
                J += err.T @ ca.diag( ca.vertcat( self.weights["Q_e_x"], self.weights["Q_e_y"], self.weights["Q_e_yaw"], self.weights["Q_virtual_speed"], self.weights["Q_u"], self.weights["Q_r"] ) ) @ err

            else:
                err = ca.vertcat( self.x[k] - x_r, self.y[k] - y_r, self.yaw[k] - yaw_r )
                J += err.T @ ca.diag( ca.vertcat( self.weights["Q_e_x_t"], self.weights["Q_e_y_t"], self.weights["Q_e_yaw_t"] ) ) @ err

        self.opti.minimize(J)

        #   Constraints
        self.opti.subject_to( self.opti.bounded( self.con_vel["u_lb"], self.vx, self.con_vel["u_ub"] ) )
        self.opti.subject_to( self.opti.bounded( self.con_vel["r_lb"], self.wz, self.con_vel["r_ub"] ) )
        self.opti.subject_to( self.opti.bounded( virtual_speed_lb, self.virtual_speed, virtual_speed_ub ) )

        #   Setup solver
        opts = {"ipopt.print_level": 1, "print_time": False}
        self.opti.solver("ipopt", opts)

    def _solve(self, initialState):
        
        self.opti.set_value(self.x_0, initialState[0])
        self.opti.set_value(self.y_0, initialState[1])
        self.opti.set_value(self.yaw_0, initialState[2])
        self.opti.set_value(self.progress_0, initialState[3])

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
        
#   Unicycle kinematics model
class DynamicsSkidSteeringPlanar(ModelParameters, Common):
    def __init__(self, x_ref, y_ref, yaw_ref):
        
        super().__init__()
        
        with open(self.mpcc_dyn_skidsteering) as f:
            solver_param = json.load(f)
            specs = solver_param["solver_specs"]
            self.con_pose = solver_param["con_pose"]
            self.con_vel = solver_param["con_vel"]
            self.con_controls = solver_param["con_controls"]
            self.weights = solver_param["weights"]

            virtual_speed_lb = solver_param["virtual_speed_lb"]
            virtual_speed_ub = solver_param["virtual_speed_ub"]
        
        #   Work with Leo Rover
        with open(self.vehicle_specs) as f:
            vehicle_param = json.load(f)
            lat_w = vehicle_param["wheelLatSeparation"]
        
        self.opti = ca.Opti()

        #   States
        self.x  = self.opti.variable( specs["N"] + 1 )
        self.y  = self.opti.variable( specs["N"] + 1 )
        self.yaw = self.opti.variable( specs["N"] + 1 )
        self.progress  = self.opti.variable( specs["N"] + 1 )
        self.vx = self.opti.variable( specs["N"] + 1 )
        self.vy = self.opti.variable( specs["N"] + 1 )
        self.wz = self.opti.variable( specs["N"] + 1 )
        self.fl = self.opti.variable( specs["N"] + 1 )
        self.fr = self.opti.variable( specs["N"] + 1 )
        
        #   Controls
        self.fr_rate = self.opti.variable( specs["N"] )
        self.fl_rate = self.opti.variable( specs["N"] )
        self.virtual_speed = self.opti.variable( specs["N"] )

        #   Set initial state parameters
        self.x_0  = self.opti.parameter()
        self.y_0  = self.opti.parameter()
        self.yaw_0 = self.opti.parameter()
        self.progress_0  = self.opti.parameter()
        self.vx_0  = self.opti.parameter()
        self.vy_0  = self.opti.parameter()
        self.wz_0  = self.opti.parameter()
        self.fl_0 = self.opti.parameter()
        self.fr_0 = self.opti.parameter()

        #   Initial constraints
        self.opti.subject_to(self.x[0] == self.x_0)
        self.opti.subject_to(self.y[0] == self.y_0)
        self.opti.subject_to(self.yaw[0] == self.yaw_0)
        self.opti.subject_to(self.progress[0] == self.progress_0)
        self.opti.subject_to(self.vx[0] == self.vx_0 )
        self.opti.subject_to(self.vy[0] == self.vy_0 )
        self.opti.subject_to(self.wz[0] == self.wz_0 )
        self.opti.subject_to(self.fl[0] == self.fl_0 )
        self.opti.subject_to(self.fr[0] == self.fr_0 )

        for k in range( specs["N"] ):
            x_next = ca.vertcat( self.x[k + 1], self.y[k + 1], self.yaw[k + 1], self.progress[k + 1], self.vx[k + 1], self.vy[k + 1], self.wz[k + 1], self.fl[k + 1], self.fr[k + 1] )
            x_now  = ca.vertcat( self.x[k], self.y[k], self.yaw[k], self.progress[k], self.vx[k], self.vy[k], self.wz[k], self.fl[k], self.fr[k] )

            k1 = self.dynamics_2d( self.yaw[k], self.vx[k], self.vy[k], self.wz[k], self.fl[k], self.fr[k], self.fl_rate[k], self.fr_rate[k], 0.3 )

            k2 = self.dynamics_2d( self.yaw[k] + specs["Ts"] / 2 * k1[2],\
                                   self.vx[k] + specs["Ts"] / 2 * k1[4],\
                                   self.vy[k] + specs["Ts"] / 2 * k1[5],\
                                   self.wz[k] + specs["Ts"] / 2 * k1[6],\
                                   self.fl[k] + specs["Ts"] / 2 * k1[7],\
                                   self.fr[k] + specs["Ts"] / 2 * k1[8],\
                                   self.fl_rate[k], self.fr_rate[k], 0.3 )
            
            k3 = self.dynamics_2d( self.yaw[k] + specs["Ts"] / 2 * k2[2],\
                                   self.vx[k] + specs["Ts"] / 2 * k2[4],\
                                   self.vy[k] + specs["Ts"] / 2 * k2[5],\
                                   self.wz[k] + specs["Ts"] / 2 * k2[6],\
                                   self.fl[k] + specs["Ts"] / 2 * k2[7],\
                                   self.fr[k] + specs["Ts"] / 2 * k2[8],\
                                   self.fl_rate[k], self.fr_rate[k], 0.3 )
            
            k4 = self.dynamics_2d( self.yaw[k] + specs["Ts"] / 2 * k3[2],\
                                   self.vx[k] + specs["Ts"] / 2 * k3[4],\
                                   self.vy[k] + specs["Ts"] / 2 * k3[5],\
                                   self.wz[k] + specs["Ts"] / 2 * k3[6],\
                                   self.fl[k] + specs["Ts"] / 2 * k3[7],\
                                   self.fr[k] + specs["Ts"] / 2 * k3[8],\
                                   self.fl_rate[k], self.fr_rate[k], 0.3 )

            x_next_pred = x_now + (specs["Ts"] / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
            
            self.opti.subject_to(x_next == x_next_pred)

        #   Cost function
        J = 0
        
        for k in range(specs["N"] + 1):
            x_r = x_ref( self.progress[k] )
            y_r = y_ref( self.progress[k] )
            yaw_r = yaw_ref( self.progress[k] )

            if( k < specs["N"] ):
                err = ca.vertcat( self.x[k] - x_r, self.y[k] - y_r, self.yaw[k] - yaw_r, self.vy[k] )
                J += err.T @ ca.diag( ca.vertcat( self.weights["Q_e_x"], self.weights["Q_e_y"], self.weights["Q_e_yaw"], self.weights["Q_v"] ) ) @ err

            else:
                err = ca.vertcat( self.x[k] - x_r, self.y[k] - y_r, self.yaw[k] - yaw_r, self.vy[k] )
                J += err.T @ ca.diag( ca.vertcat( self.weights["Q_e_x"], self.weights["Q_e_y"], self.weights["Q_e_yaw"], self.weights["Q_v"] ) ) @ err

        self.opti.minimize(J)

        #   Constraints
        self.opti.subject_to( self.opti.bounded( self.con_vel["u_lb"], self.vx, self.con_vel["u_ub"] ) )
        self.opti.subject_to( self.opti.bounded( self.con_vel["v_lb"], self.vy, self.con_vel["v_ub"] ) )
        self.opti.subject_to( self.opti.bounded( self.con_vel["r_lb"], self.wz, self.con_vel["r_ub"] ) )
        self.opti.subject_to( self.opti.bounded( self.con_controls["fl_lb"], self.fl, self.con_controls["fl_ub"] ) )
        self.opti.subject_to( self.opti.bounded( self.con_controls["fr_lb"], self.fr, self.con_controls["fr_ub"] ) )
        self.opti.subject_to( self.opti.bounded( self.con_controls["fl_rate_lb"], self.fl_rate, self.con_controls["fl_rate_ub"] ) )
        self.opti.subject_to( self.opti.bounded( self.con_controls["fr_rate_lb"], self.fr_rate, self.con_controls["fr_rate_ub"] ) )
        #self.opti.subject_to( self.opti.bounded( virtual_speed_lb, self.virtual_speed, virtual_speed_ub ) )
        
        #   Setup solver
        opts = {"ipopt.print_level": 0, "print_time": False}
        self.opti.solver("ipopt", opts)

    def _solve(self, initialState):

        self.opti.set_value(self.x_0, initialState[0])
        self.opti.set_value(self.y_0, initialState[1])
        self.opti.set_value(self.yaw_0, initialState[2])
        self.opti.set_value(self.progress_0, initialState[3])
        self.opti.set_value(self.vx_0, initialState[4])
        self.opti.set_value(self.vy_0, initialState[5])
        self.opti.set_value(self.wz_0, initialState[6])
        self.opti.set_value(self.fl_0, initialState[7])
        self.opti.set_value(self.fr_0, initialState[8])

        #   Solve optimization problem
        sol = self.opti.solve()

        # Extract solution
        x_sol  = sol.value(self.x)
        y_sol  = sol.value(self.y)
        yaw_sol = sol.value(self.yaw)
        progress_sol = sol.value(self.progress)
        vx_sol = sol.value(self.vx)
        vy_sol = sol.value(self.vy)
        wz_sol = sol.value(self.wz)
        #virtual_speed_sol = sol.value(self.virtual_speed)
        fl_sol = sol.value(self.fl)
        fr_sol = sol.value(self.fr)
        fl_rate_sol = sol.value(self.fl_rate)
        fr_rate_sol = sol.value(self.fr_rate)
        
        return [x_sol, y_sol, yaw_sol, progress_sol, vx_sol, vy_sol, wz_sol, fl_sol, fr_sol, fl_rate_sol, fr_rate_sol]