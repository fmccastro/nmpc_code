#!/usr/bin/python3.8

import sys

import scipy.linalg
sys.path.insert(0, "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/src")

from classes.all_imports import *
from classes.common_class import *

#   Unicycle kinematics integrator with rates
class IntegratorKinematicsSkidSteering(Common):
    def __init__(self, curvature):
        
        super().__init__()
        
        with open(self.frenet_serret_kin_skid) as f:
            solver_param = json.load(f)
            specs = solver_param["solver_specs"]
            #self.con_pose = solver_param["con_pose"]
            #self.con_vel = solver_param["con_vel"]
            #self.weights = solver_param["weights"]
        
        #   Work with Leo Rover
        #with open(self.vehicle_specs) as f:
        #    vehicle_param = json.load(f)
        #    lat_w = vehicle_param["wheelLatSeparation"]
        
        s1 = ca.SX.sym('s1')
        y1 = ca.SX.sym('y1')
        diff_yaw = ca.SX.sym('yaw')
        progress = ca.SX.sym('progress')
        vx = ca.SX.sym('vx')
        wz = ca.SX.sym('wz')
        virtual_speed = ca.SX.sym('virtual_speed')

        s1_dot = ca.SX.sym('s1_dot')
        y1_dot = ca.SX.sym('y1_dot')
        diff_yaw_dot = ca.SX.sym('diff_yaw_dot')
        progress_dot = ca.SX.sym('progress_dot')
        vx_dot = ca.SX.sym('vx_dot')
        wz_dot = ca.SX.sym('wz_dot')
        virtual_speed_dot = ca.SX.sym('virtual_speed_dot')
        
        vx_rate = ca.SX.sym('vx_rate')
        wz_rate = ca.SX.sym('wz_rate')
        virtual_speed_rate = ca.SX.sym('virtual_speed_rate')

        pitch = ca.SX.sym('pitch')

        #   State
        state = ca.vertcat(s1, y1, diff_yaw, progress)
        
        #   State derivative
        state_dot = ca.vertcat(s1_dot, y1_dot, diff_yaw_dot, progress_dot)
        
        #   Controls
        controls = ca.vertcat(vx, wz, virtual_speed)

        parameters = ca.vertcat(pitch)

        #   Explicit model
        f_expl = ca.vertcat( ca.cos(pitch) * ca.cos(diff_yaw) * vx - virtual_speed * (1 - curvature(progress) * y1),\
                             ca.cos(pitch) * ca.sin(diff_yaw) * vx - virtual_speed * curvature(progress) * s1,\
                             wz - curvature(progress) * virtual_speed,\
                             virtual_speed )
        
        #   Implicit model
        f_impl = ca.vertcat( s1_dot - ( ca.cos(pitch) * ca.cos(diff_yaw) * vx - virtual_speed * (1 - curvature(progress) * y1) ),\
                             y1_dot - ( ca.cos(pitch) * ca.sin(diff_yaw) * vx - virtual_speed * curvature(progress) * s1 ),\
                             diff_yaw_dot - ( wz - curvature(progress) * virtual_speed ),\
                             progress_dot - virtual_speed )
        
        #   Call model instance
        model = AcadosModel()
        
        model.f_impl_expr = f_impl
        model.f_expl_expr = f_expl
        model.x = state
        model.xdot = state_dot
        model.u = controls
        model.p = parameters
        model.name = "integrator_kinematics_skid"

        model.x_labels = [r'$x$ [m]', r'$y$ [m]', r'$\psi$ [rad]', r'progress'] 
        model.u_labels = [r'v_x [m/s]', r'w_z [rad/s]', r'virtual_speed [m/s]']
        model.t_label = '$t$ [s]'

        #   Call dims instance
        dims = AcadosSimDims()

        dims.nx = model.x.rows()
        dims.nu = model.u.rows()
        dims.np = model.p.rows()
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
        sim.code_export_directory = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/scripts/acados_c_generated_code/Countoring/FrenetSerretKinematics/acadosSim"
        sim.acados_lib_path = "/home/francisco/acados/lib"
        sim.acados_include_path = "/home/francisco/acados/include"

        json_file_path = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/scripts/acados_c_generated_code/Countoring/FrenetSerretKinematics/acadosSim/acados_sim.json"
        ###

        self.integrator = AcadosSimSolver(sim, json_file = json_file_path)

    def _simulate(self, state, controls, parameters):
        return self.integrator.simulate(x = state, u = controls, p = parameters)

#   Unicycle kinematics integrator with rates
class IntegratorKinematicsRatesSkidSteering(Common):
    def __init__(self, curvature):
        
        super().__init__()
        
        with open(self.frenet_serret_kin_skid) as f:
            solver_param = json.load(f)
            specs = solver_param["solver_specs"]
            #self.con_pose = solver_param["con_pose"]
            #self.con_vel = solver_param["con_vel"]
            #self.weights = solver_param["weights"]
        
        #   Work with Leo Rover
        #with open(self.vehicle_specs) as f:
        #    vehicle_param = json.load(f)
        #    lat_w = vehicle_param["wheelLatSeparation"]
        
        s1 = ca.SX.sym('s1')
        y1 = ca.SX.sym('y1')
        diff_yaw = ca.SX.sym('yaw')
        progress = ca.SX.sym('progress')
        vx = ca.SX.sym('vx')
        wz = ca.SX.sym('wz')
        virtual_speed = ca.SX.sym('virtual_speed')

        s1_dot = ca.SX.sym('s1_dot')
        y1_dot = ca.SX.sym('y1_dot')
        diff_yaw_dot = ca.SX.sym('diff_yaw_dot')
        progress_dot = ca.SX.sym('progress_dot')
        vx_dot = ca.SX.sym('vx_dot')
        wz_dot = ca.SX.sym('wz_dot')
        virtual_speed_dot = ca.SX.sym('virtual_speed_dot')
        
        vx_rate = ca.SX.sym('vx_rate')
        wz_rate = ca.SX.sym('wz_rate')
        virtual_speed_rate = ca.SX.sym('virtual_speed_rate')

        pitch = ca.SX.sym('pitch')

        #   State
        state = ca.vertcat(s1, y1, diff_yaw, progress, vx, wz, virtual_speed)
        
        #   State derivative
        state_dot = ca.vertcat(s1_dot, y1_dot, diff_yaw_dot, progress_dot, vx_dot, wz_dot, virtual_speed_dot)
        
        #   Controls
        controls = ca.vertcat(vx_rate, wz_rate, virtual_speed_rate)

        parameters = ca.vertcat(pitch)

        #   Explicit model
        f_expl = ca.vertcat( ca.cos(pitch) * ca.cos(diff_yaw) * vx - virtual_speed * (1 - curvature(progress) * y1),\
                             ca.cos(pitch) * ca.sin(diff_yaw) * vx - virtual_speed * curvature(progress) * s1,\
                             wz - curvature(progress) * virtual_speed,\
                             virtual_speed,\
                             vx_rate,\
                             wz_rate,\
                             virtual_speed_rate )
        
        #   Implicit model
        f_impl = ca.vertcat( s1_dot - ( ca.cos(pitch) * ca.cos(diff_yaw) * vx - virtual_speed * (1 - curvature(progress) * y1) ),\
                             y1_dot - ( ca.cos(pitch) * ca.sin(diff_yaw) * vx - virtual_speed * curvature(progress) * s1 ),\
                             diff_yaw_dot - ( wz - curvature(progress) * virtual_speed ),\
                             progress_dot - virtual_speed,\
                             vx_dot - vx_rate,\
                             wz_dot - wz_rate,\
                             virtual_speed_dot - virtual_speed_rate )
        
        #   Call model instance
        model = AcadosModel()
        
        model.f_impl_expr = f_impl
        model.f_expl_expr = f_expl
        model.x = state
        model.xdot = state_dot
        model.u = controls
        model.p = parameters
        model.name = "integrator_kinematics_rates_skid"

        model.x_labels = [r'$x$ [m]', r'$y$ [m]', r'$\psi$ [rad]', r'progress'] 
        model.u_labels = [r'v_x [m/s]', r'w_z [rad/s]', r'virtual_speed [m/s]']
        model.t_label = '$t$ [s]'

        #   Call dims instance
        dims = AcadosSimDims()

        dims.nx = model.x.rows()
        dims.nu = model.u.rows()
        dims.np = model.p.rows()
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
        sim.parameter_values = np.stack( [0.0] )

        #   Set folder path where generated c code is located
        sim.code_export_directory = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/scripts/acados_c_generated_code/Countoring/FrenetSerretKinematicsRates/acadosSim"
        sim.acados_lib_path = "/home/francisco/acados/lib"
        sim.acados_include_path = "/home/francisco/acados/include"

        json_file_path = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/scripts/acados_c_generated_code/Countoring/FrenetSerretKinematicsRates/acadosSim/acados_sim.json"
        ###

        self.integrator = AcadosSimSolver(sim, json_file = json_file_path)

    def _simulate(self, state, controls, parameters):
        return self.integrator.simulate(x = state, u = controls, p = parameters)

#   Unicycle kinematics integrator with rates
class IntegratorKinematicsRatesSkidSteeringStd(Common):
    def __init__(self):
        
        super().__init__()
        
        with open(self.frenet_serret_kin_skid) as f:
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
        vx = ca.SX.sym('vx')
        wz = ca.SX.sym('wz')
        virtual_speed = ca.SX.sym('virtual_speed')

        x_dot = ca.SX.sym('x_dot')
        y_dot = ca.SX.sym('y_dot')
        yaw_dot = ca.SX.sym('yaw_dot')
        progress_dot = ca.SX.sym('progress_dot')
        vx_dot = ca.SX.sym('vx_dot')
        wz_dot = ca.SX.sym('wz_dot')
        virtual_speed_dot = ca.SX.sym('virtual_speed_dot')
        
        vx_rate = ca.SX.sym('vx_rate')
        wz_rate = ca.SX.sym('wz_rate')
        virtual_speed_rate = ca.SX.sym('virtual_speed_rate')

        pitch = ca.SX.sym('pitch')

        #   State
        state = ca.vertcat(x, y, yaw, progress, vx, wz, virtual_speed)
        
        #   State derivative
        state_dot = ca.vertcat(x_dot, y_dot, yaw_dot, progress_dot, vx_dot, wz_dot, virtual_speed_dot)
        
        #   Controls
        controls = ca.vertcat(vx_rate, wz_rate, virtual_speed_rate)

        parameters = ca.vertcat(pitch)

        #   Explicit model
        f_expl = ca.vertcat( ca.cos(pitch) * ca.cos(yaw) * vx,\
                             ca.cos(pitch) * ca.sin(yaw) * vx,\
                             wz,\
                             virtual_speed,\
                             vx_rate,\
                             wz_rate,\
                             virtual_speed_rate )
        
        #   Implicit model
        f_impl = ca.vertcat( x_dot - ( ca.cos(pitch) * ca.cos(yaw) * vx ),\
                             y_dot - ( ca.cos(pitch) * ca.sin(yaw) * vx ),\
                             yaw_dot - wz,\
                             progress_dot - virtual_speed,\
                             vx_dot - vx_rate,\
                             wz_dot - wz_rate,\
                             virtual_speed_dot - virtual_speed_rate )
        
        #   Call model instance
        model = AcadosModel()
        
        model.f_impl_expr = f_impl
        model.f_expl_expr = f_expl
        model.x = state
        model.xdot = state_dot
        model.u = controls
        model.p = parameters
        model.name = "integrator_kinematics_rates_skid"

        model.x_labels = [r'$x$ [m]', r'$y$ [m]', r'$\psi$ [rad]', r'progress'] 
        model.u_labels = [r'v_x [m/s]', r'w_z [rad/s]', r'virtual_speed [m/s]']
        model.t_label = '$t$ [s]'

        #   Call dims instance
        dims = AcadosSimDims()

        dims.nx = model.x.rows()
        dims.nu = model.u.rows()
        dims.np = model.p.rows()
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
        sim.parameter_values = np.stack( [0.0] )

        #   Set folder path where generated c code is located
        sim.code_export_directory = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/scripts/acados_c_generated_code/Countoring/FrenetSerretKinematicsRates/acadosSim"
        sim.acados_lib_path = "/home/francisco/acados/lib"
        sim.acados_include_path = "/home/francisco/acados/include"

        json_file_path = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/scripts/acados_c_generated_code/Countoring/FrenetSerretKinematicsRates/acadosSim/acados_sim.json"
        ###

        self.integrator = AcadosSimSolver(sim, json_file = json_file_path)

    def _simulate(self, state, controls, parameters):
        return self.integrator.simulate(x = state, u = controls, p = parameters)

#   Unicycle dynamics integrator
class IntegratorDynamicsSkidSteering(Common):
    def __init__(self, curvature):
        
        super().__init__()
        
        with open(self.frenet_serret_dyn_skid) as f:
            solver_param = json.load(f)
            specs = solver_param["solver_specs"]
            #self.con_pose = solver_param["con_pose"]
            #self.con_vel = solver_param["con_vel"]
            #self.weights = solver_param["weights"]
        
        #   Work with Leo Rover
        with open(self.vehicle_specs) as f:
            vehicle_param = json.load(f)
            lat_w = vehicle_param["wheelLatSeparation"]
        
        s1 = ca.SX.sym('s1')
        y1 = ca.SX.sym('y1')
        diff_yaw = ca.SX.sym('yaw')
        progress = ca.SX.sym('progress')
        vx = ca.SX.sym('vx')
        wz = ca.SX.sym('wz')

        s1_dot = ca.SX.sym('x_dot')
        y1_dot = ca.SX.sym('y_dot')
        diff_yaw_dot = ca.SX.sym('yaw_dot')
        progress_dot = ca.SX.sym('progress_dot')
        vx_dot = ca.SX.sym('vx_dot')
        wz_dot = ca.SX.sym('wz_dot')

        fl = ca.SX.sym('fl')
        fr = ca.SX.sym('fr')
        
        virtual_speed = ca.SX.sym('virtual_speed')

        pitch = ca.SX.sym('pitch')

        #   State
        state = ca.vertcat(s1, y1, diff_yaw, progress, vx, wz)
        
        #   State derivative
        state_dot = ca.vertcat(s1_dot, y1_dot, diff_yaw_dot, progress_dot, vx_dot, wz_dot)
        
        #   Controls
        controls = ca.vertcat(fl, fr, virtual_speed)

        #   Parameters
        parameters = ca.vertcat(pitch)

        #   Explicit model
        f_expl = ca.vertcat( ca.cos(diff_yaw) * vx - virtual_speed * (1 - curvature(progress) * y1),\
                             ca.sin(diff_yaw) * vx - virtual_speed * curvature(progress) * s1,\
                             wz - curvature(progress) * virtual_speed,\
                             virtual_speed,\
                             (fr + fl) / vehicle_param["m"] - ca.sin(pitch) * vehicle_param["m"] * self.gz,\
                             (fr - fl) * lat_w / ( 2 * vehicle_param["izz"] ) )
        
        #   Implicit model
        f_impl = ca.vertcat( s1_dot - ( ca.cos(diff_yaw) * vx - virtual_speed * (1 - curvature(progress) * y1) ),\
                             y1_dot - ( ca.sin(diff_yaw) * vx - virtual_speed * curvature(progress) * s1 ),\
                             diff_yaw_dot - ( wz - curvature(progress) * virtual_speed ),\
                             progress_dot - virtual_speed,\
                             vx_dot - ( (fr + fl) / vehicle_param["m"] - ca.sin(pitch) * vehicle_param["m"] * self.gz ),\
                             wz_dot - ( (fr - fl) * lat_w / ( 2 * vehicle_param["izz"] ) ) )
        
        #   Call model instance
        model = AcadosModel()
        
        model.f_impl_expr = f_impl
        model.f_expl_expr = f_expl
        model.x = state
        model.xdot = state_dot
        model.u = controls
        model.p = parameters
        model.name = "integrator_dynamics_skid"

        model.x_labels = [r'$x$ [m]', r'$y$ [m]', r'$\psi$ [rad]', r'progress', r'v_x [m/s]', r'w_z [rad/s]', r'f_l [N]', r'f_r [N]'] 
        model.u_labels = [r'v_x [m/s]', r'w_z [rad/s]', r'virtual_speed [m/s]']
        model.t_label = '$t$ [s]'

        #   Call dims instance
        dims = AcadosSimDims()

        dims.nx = model.x.rows()
        dims.nu = model.u.rows()
        dims.np = model.p.rows()
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
        sim.parameter_values = np.stack( [1.0] )

        #   Set folder path where generated c code is located
        sim.code_export_directory = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/scripts/acados_c_generated_code/Countoring/FrenetSerretDynamics/acadosSim"
        sim.acados_lib_path = "/home/francisco/acados/lib"
        sim.acados_include_path = "/home/francisco/acados/include"

        json_file_path = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/scripts/acados_c_generated_code/Countoring/FrenetSerretDynamics/acadosSim/acados_sim.json"
        ###

        self.integrator = AcadosSimSolver(sim, json_file = json_file_path)

    def _simulate(self, state, controls, parameters):
        return self.integrator.simulate(x = state, u = controls, p = parameters)

#   Unicycle dynamics integrator
class IntegratorDynamicsSkidSteeringRates(Common):
    def __init__(self, curvature):
        
        super().__init__()
        
        with open(self.frenet_serret_dyn_skid) as f:
            solver_param = json.load(f)
            specs = solver_param["solver_specs"]
            #self.con_pose = solver_param["con_pose"]
            #self.con_vel = solver_param["con_vel"]
            #self.weights = solver_param["weights"]
        
        #   Work with Leo Rover
        with open(self.vehicle_specs) as f:
            vehicle_param = json.load(f)
            lat_w = vehicle_param["wheelLatSeparation"]
        
        s1 = ca.SX.sym('s1')
        y1 = ca.SX.sym('y1')
        diff_yaw = ca.SX.sym('yaw')
        progress = ca.SX.sym('progress')
        vx = ca.SX.sym('vx')
        wz = ca.SX.sym('wz')
        fl = ca.SX.sym('fl')
        fr = ca.SX.sym('fr')

        s1_dot = ca.SX.sym('x_dot')
        y1_dot = ca.SX.sym('y_dot')
        diff_yaw_dot = ca.SX.sym('yaw_dot')
        progress_dot = ca.SX.sym('progress_dot')
        vx_dot = ca.SX.sym('vx_dot')
        wz_dot = ca.SX.sym('wz_dot')
        fl_dot = ca.SX.sym('fl_dot')
        fr_dot = ca.SX.sym('fr_dot')
        
        fl_rate = ca.SX.sym('fl_rate')
        fr_rate = ca.SX.sym('fr_rate')
        virtual_speed = ca.SX.sym('virtual_speed')

        pitch = ca.SX.sym('pitch')

        #   State
        state = ca.vertcat(s1, y1, diff_yaw, progress, vx, wz, fl, fr)
        
        #   State derivative
        state_dot = ca.vertcat(s1_dot, y1_dot, diff_yaw_dot, progress_dot, vx_dot, wz_dot, fl_dot, fr_dot)
        
        #   Controls
        controls = ca.vertcat(fl_rate, fr_rate, virtual_speed)

        #   Parameters
        parameters = ca.vertcat(pitch)

        #   Explicit model
        f_expl = ca.vertcat( ca.cos(diff_yaw) * vx - virtual_speed * (1 - curvature(progress) * y1),\
                             ca.sin(diff_yaw) * vx - virtual_speed * curvature(progress) * s1,\
                             wz - curvature(progress) * virtual_speed,\
                             virtual_speed,\
                             (fr + fl) / vehicle_param["m"] - ca.sin(pitch) * vehicle_param["m"] * self.gz,\
                             (fr - fl) * lat_w / ( 2 * vehicle_param["izz"] ),\
                             fl_rate,\
                             fr_rate )
        
        #   Implicit model
        f_impl = ca.vertcat( s1_dot - ( ca.cos(diff_yaw) * vx - virtual_speed * (1 - curvature(progress) * y1) ),\
                             y1_dot - ( ca.sin(diff_yaw) * vx - virtual_speed * curvature(progress) * s1 ),\
                             diff_yaw_dot - ( wz - curvature(progress) * virtual_speed ),\
                             progress_dot - virtual_speed,\
                             vx_dot - ( (fr + fl) / vehicle_param["m"] - ca.sin(pitch) * vehicle_param["m"] * self.gz ),\
                             wz_dot - ( (fr - fl) * lat_w / ( 2 * vehicle_param["izz"] ) ),\
                             fl_dot - fl_rate,\
                             fr_dot - fr_rate )
        
        #   Call model instance
        model = AcadosModel()
        
        model.f_impl_expr = f_impl
        model.f_expl_expr = f_expl
        model.x = state
        model.xdot = state_dot
        model.u = controls
        model.p = parameters
        model.name = "integrator_dynamics_skid"

        model.x_labels = [r'$x$ [m]', r'$y$ [m]', r'$\psi$ [rad]', r'progress', r'v_x [m/s]', r'w_z [rad/s]', r'f_l [N]', r'f_r [N]'] 
        model.u_labels = [r'v_x [m/s]', r'w_z [rad/s]', r'virtual_speed [m/s]']
        model.t_label = '$t$ [s]'

        #   Call dims instance
        dims = AcadosSimDims()

        dims.nx = model.x.rows()
        dims.nu = model.u.rows()
        dims.np = model.p.rows()
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
        sim.parameter_values = np.stack( [1.0] )

        #   Set folder path where generated c code is located
        sim.code_export_directory = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/scripts/acados_c_generated_code/Countoring/FrenetSerretDynamics/acadosSim"
        sim.acados_lib_path = "/home/francisco/acados/lib"
        sim.acados_include_path = "/home/francisco/acados/include"

        json_file_path = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/scripts/acados_c_generated_code/Countoring/FrenetSerretDynamics/acadosSim/acados_sim.json"
        ###

        self.integrator = AcadosSimSolver(sim, json_file = json_file_path)

    def _simulate(self, state, controls, parameters):
        return self.integrator.simulate(x = state, u = controls, p = parameters)

#   Unicycle standard dynamics integrator
class IntegratorDynamicsSkidSteeringRatesStd(Common):
    def __init__(self):
        
        super().__init__()
        
        with open(self.std_dyn_skid) as f:
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
        vx = ca.SX.sym('vx')
        wz = ca.SX.sym('wz')
        fl = ca.SX.sym('fl')
        fr = ca.SX.sym('fr')
        
        x_dot = ca.SX.sym('x_dot')
        y_dot = ca.SX.sym('y_dot')
        yaw_dot = ca.SX.sym('yaw_dot')
        vx_dot = ca.SX.sym('vx_dot')
        wz_dot = ca.SX.sym('wz_dot')
        fl_dot = ca.SX.sym('fl_dot')
        fr_dot = ca.SX.sym('fr_dot')

        fl_rate = ca.SX.sym('fl_rate')
        fr_rate = ca.SX.sym('fr_rate')
        
        pitch = ca.SX.sym('pitch')

        #   State
        state = ca.vertcat(x, y, yaw, vx, wz, fl, fr)
        
        #   State derivative
        state_dot = ca.vertcat(x_dot, y_dot, yaw_dot, vx_dot, wz_dot, fl_dot, fr_dot)
        
        #   Controls
        controls = ca.vertcat(fl_rate, fr_rate)

        #   Parameters
        parameters = ca.vertcat(pitch)

        #   Explicit model
        f_expl = ca.vertcat( ca.cos(pitch) * ca.cos(yaw) * vx,\
                             ca.cos(pitch) * ca.sin(yaw) * vx,\
                             wz,\
                             (fr + fl) / vehicle_param["m"] - ca.sin(pitch) * vehicle_param["m"] * self.gz,\
                             (fr - fl) * lat_w / ( 2 * vehicle_param["izz"] ),\
                             fl_rate,\
                             fr_rate )
        
        #   Implicit model
        f_impl = ca.vertcat( x_dot - ca.cos(pitch) * ca.cos(yaw) * vx,\
                             y_dot - ca.cos(pitch) * ca.sin(yaw) * vx,\
                             yaw_dot - wz,\
                             vx_dot - ( (fr + fl) / vehicle_param["m"] - ca.sin(pitch) * vehicle_param["m"] * self.gz ),\
                             wz_dot - ( (fr - fl) * lat_w / ( 2 * vehicle_param["izz"] ) ),\
                             fl_dot - fl_rate,\
                             fr_dot - fr_rate )
        
        #   Call model instance
        model = AcadosModel()
        
        model.f_impl_expr = f_impl
        model.f_expl_expr = f_expl
        model.x = state
        model.xdot = state_dot
        model.u = controls
        model.p = parameters
        model.name = "integrator_dynamics_skid"

        model.x_labels = [r'$x$ [m]', r'$y$ [m]', r'$\psi$ [rad]', r'progress', r'v_x [m/s]', r'w_z [rad/s]', r'f_l [N]', r'f_r [N]'] 
        model.u_labels = [r'v_x [m/s]', r'w_z [rad/s]', r'virtual_speed [m/s]']
        model.t_label = '$t$ [s]'

        #   Call dims instance
        dims = AcadosSimDims()

        dims.nx = model.x.rows()
        dims.nu = model.u.rows()
        dims.np = model.p.rows()
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
        sim.parameter_values = np.stack( [1.0] )

        #   Set folder path where generated c code is located
        sim.code_export_directory = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/scripts/acados_c_generated_code/Countoring/FrenetSerretDynamics/acadosSim"
        sim.acados_lib_path = "/home/francisco/acados/lib"
        sim.acados_include_path = "/home/francisco/acados/include"

        json_file_path = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/scripts/acados_c_generated_code/Countoring/FrenetSerretDynamics/acadosSim/acados_sim.json"
        ###

        self.integrator = AcadosSimSolver(sim, json_file = json_file_path)

    def _simulate(self, state, controls, parameters):
        return self.integrator.simulate(x = state, u = controls, p = parameters)