#!/usr/bin/python3.8

import sys

import scipy.linalg
sys.path.insert(0, "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/src")

from classes.all_imports import *
from classes.common_class import *

"""
    Build and define integrators for identification and validation purposes
"""

class ModelParameters():

    #   Constructor
    def __init__(self):

        super().__init__()
        
        ###   Optimization symbolic variables   ############################

        """
            Mobile robot states
        """

        #   Position
        self.x = ca.SX.sym('x')
        self.y = ca.SX.sym('y')
        self.z = ca.SX.sym('z')

        self.position = ca.vertcat( self.x, self.y, self.z )

        #   Position derivatives
        self.x_dot = ca.SX.sym('x_dot')
        self.y_dot = ca.SX.sym('y_dot')
        self.z_dot = ca.SX.sym('z_dot')

        self.position_dot = ca.vertcat(self.x_dot, self.y_dot, self.z_dot )

        #   Fixed angles (rpy)
        self.roll = ca.SX.sym('roll')
        self.pitch = ca.SX.sym('pitch')
        self.yaw = ca.SX.sym('yaw')

        self.orientation = ca.vertcat(self.roll, self.pitch, self.yaw)

        #   Fixed angles derivative (rpy)
        self.roll_dot = ca.SX.sym('roll_dot')
        self.pitch_dot = ca.SX.sym('pitch_dot')
        self.yaw_dot = ca.SX.sym('yaw_dot')

        self.orientation_dot = ca.vertcat(self.roll_dot, self.pitch_dot, self.yaw_dot)

        #   Linear velocity (w.r. to body frame)
        self.vx = ca.SX.sym('vx')
        self.vy = ca.SX.sym('vy')
        self.vz = ca.SX.sym('vz')

        self.lin_vel = ca.vertcat( self.vx, self.vy, self.vz )

        #   Angular velocity (w.r. to body frame)
        self.wx = ca.SX.sym('wx')
        self.wy = ca.SX.sym('wy')
        self.wz = ca.SX.sym('wz')

        self.ang_vel = ca.vertcat( self.wx, self.wy, self.wz )

        #   Linear velocity derivative (w.r. to body frame)
        self.vx_dot = ca.SX.sym('vx_dot') 
        self.vy_dot = ca.SX.sym('vy_dot')
        self.vz_dot = ca.SX.sym('vz_dot')

        self.lin_vel_dot = ca.vertcat( self.vx_dot, self.vy_dot, self.vz_dot )

        #   Angular velocity derivative (w.r. to body frame)
        self.wx_dot = ca.SX.sym('wx_dot')
        self.wy_dot = ca.SX.sym('wy_dot')
        self.wz_dot = ca.SX.sym('wz_dot')

        self.ang_vel_dot = ca.vertcat( self.wx_dot, self.wy_dot, self.wz_dot )

        """
            Mobile robot controls
        """

        #   Forces
        self.fx = ca.SX.sym('fx')
        self.fy = ca.SX.sym('fy')
        self.fz = ca.SX.sym('fz')

        self.mx = ca.SX.sym('mx')
        self.my = ca.SX.sym('my')
        self.mz = ca.SX.sym('mz')

        #   Wheel forces
        self.tl = ca.SX.sym('tl')
        self.tr = ca.SX.sym('tr')

        #   Wheel force rates
        self.tl_rate = ca.SX.sym('tl_rate')
        self.tr_rate = ca.SX.sym('tr_rate')

        """ Rotation matrices """
        self.TransRotationMatrix = ca.horzcat( ca.vertcat( ca.cos(self.yaw) * ca.cos(self.pitch), ca.sin(self.yaw) * ca.cos(self.pitch), -ca.sin(self.pitch) ),\
                                               ca.vertcat( ca.cos(self.yaw) * ca.sin(self.pitch) * ca.sin(self.roll) - ca.sin(self.yaw) * ca.cos(self.roll), ca.sin(self.yaw) * ca.sin(self.pitch) * ca.sin(self.roll) + ca.cos(self.yaw) * ca.cos(self.roll), ca.cos(self.pitch) * ca.sin(self.roll) ),\
                                               ca.vertcat( ca.cos(self.yaw) * ca.sin(self.pitch) * ca.cos(self.roll) + ca.sin(self.yaw) * ca.sin(self.roll), ca.sin(self.yaw) * ca.sin(self.pitch) * ca.cos(self.roll) - ca.cos(self.yaw) * ca.sin(self.roll), ca.cos(self.pitch) * ca.cos(self.roll) ) )

        self.RotRotationMatrix = ca.horzcat( ca.vertcat( 1, 0, 0 ),\
                                             ca.vertcat( ca.tan(self.pitch) * ca.sin(self.roll), ca.cos(self.roll), -ca.sin(self.roll) / ca.cos(self.pitch) ),\
                                             ca.vertcat( -ca.tan(self.pitch) * ca.cos(self.roll), ca.sin(self.roll), ca.cos(self.roll) / ca.cos(self.pitch) ) )

#   Dynamics integrator
class DynamicsIntegrator(ModelParameters, Common):

    """
        Integrator of the full dynamics model (SRBD)
    """

    #   Constructor
    def __init__(self):
        
        """
            :com2wheel dictionary
        """
        
        super().__init__()
        
        with open(self.vehicle_specs) as f:
            vehicle_param = json.load(f)
        
        with open(self.dynamics_integrator) as f:
            integrator_specs = json.load(f)

        #   State
        state = ca.vertcat(self.position, self.orientation, self.lin_vel, self.ang_vel)
        
        #   Derivative state
        state_dot = ca.vertcat(self.position_dot, self.orientation_dot, self.lin_vel_dot, self.ang_vel_dot)

        #   Inertia tensor of vehicle
        inertia = ca.horzcat( ca.vertcat(vehicle_param["ixx"], 0.0, 0.0),\
                              ca.vertcat(0.0, vehicle_param["iyy"], 0.0),\
                              ca.vertcat(0.0, 0.0, vehicle_param["izz"]) )

        #   Controls
        fx = ca.SX.sym('fx')
        fy = ca.SX.sym('fy')
        fz = ca.SX.sym('fz')

        mx = ca.SX.sym('mx')
        my = ca.SX.sym('my')
        mz = ca.SX.sym('mz')

        controls = ca.vertcat(fx, fy, fz, mx, my, mz)

        sumForces = ca.vertcat(fx, fy, fz)
        sumMoments = ca.vertcat(mx, my, mz)
        
        v_dot = -ca.cross( self.ang_vel, self.lin_vel ) + sumForces / vehicle_param["m"]
        w_dot = ca.inv_minor( inertia ) @ ( -ca.cross( self.ang_vel, inertia @ self.ang_vel ) + sumMoments )

        #   Explicit model
        f_expl = ca.vertcat( self.TransRotationMatrix @ self.lin_vel,\
                             self.RotRotationMatrix @ self.ang_vel,\
                             v_dot,\
                             w_dot )

        #   Implicit model
        f_impl = ca.vertcat( self.position_dot - self.TransRotationMatrix @ self.lin_vel,\
                             self.orientation_dot - self.RotRotationMatrix @ self.ang_vel,\
                             self.lin_vel_dot - v_dot,\
                             self.ang_vel_dot - w_dot )                                  

        #   Call model instance
        model = AcadosModel()

        model.f_impl_expr = f_impl
        model.f_expl_expr = f_expl
        
        model.x = state
        model.xdot = state_dot
        model.u = controls
        
        model.name = "dynamics_integrator"

        model.x_labels = [r'x [m]', r'y [m]', r'z [m]', r'$\phi$ [rad]', r'$\theta$ [rad]', r'$\psi$ [rad]', r'$u$ [m/s]', r'$v$ [m/s]', r'$w$ [m/s]', r'$p$ [rad/s]', r'$q$ [rad/s]', r'$r$ [rad/s]']
        model.u_labels = [r'f_x [N]', r'f_y [N]', r'f_z [N]', r'm_x [Nm]', r'm_y [Nm]', r'm_z [Nm]']
        model.t_label = r'$t$ [s]'

        #   Call dims instance
        dims = AcadosSimDims()

        dims.nu = model.u.rows()
        dims.nx = model.x.rows()
        ###

        #   Call solver options instance
        solver_options = AcadosSimOptions()

        solver_options.T = integrator_specs["Ts"]
        solver_options.integrator_type = 'ERK'
        solver_options.newton_iter = 3
        solver_options.num_stages = 4
        solver_options.num_steps = 1
        ###

        #   Call ocp instance
        sim = AcadosSim()

        #   Set ocp
        sim.model = model
        sim.dims = dims
        sim.solver_options = solver_options
        
        #   Set folder path where generated c code is located
        sim.code_export_directory = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/scripts/acados_c_generated_code/DynamicsIntegrator/acadosSim"
        sim.acados_lib_path = "/home/francisco/acados/lib"
        sim.acados_include_path = "/home/francisco/acados/include"

        json_file_path = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/scripts/acados_c_generated_code/DynamicsIntegrator/acadosSim/acados_sim.json"
        ###

        self.integrator = AcadosSimSolver(sim, json_file = json_file_path)
    
    def _simulate(self, state, controls, dtime):
        self.integrator.set('T', dtime)
        return self.integrator.simulate(x = state, u = controls)

class SimpleDynamicsIntegratorCasadi(ModelParameters, Common):

    """
        Integrator of the full dynamics model (SRBD)
    """

    #   Constructor
    def __init__(self):
        
        """
            :com2wheel dictionary
        """
        
        super().__init__()
        
        with open(self.vehicle_specs) as f:
            vehicle_param = json.load(f)
        
        with open(self.dynamics_integrator) as f:
            integrator_specs = json.load(f)

        #   State
        state = ca.vertcat(self.x, self.y, self.roll, self.pitch, self.yaw, self.vx, self.wz, self.tl, self.tr)
        controls = ca.vertcat(self.tl_rate, self.tr_rate)

        fx = self.tl + self.tr
        mz = (self.tr - self.tl) * vehicle_param["wheelLatSeparation"] / 2.0

        #   Explicit model
        f_expl = ca.vertcat( ca.cos(self.pitch) * ca.cos(self.yaw) * self.vx,\
                             ca.cos(self.pitch) * ca.sin(self.yaw) * self.vx,\
                             -ca.tan(self.pitch) * ca.cos(self.roll) * self.wz,\
                             ca.sin(self.roll) * self.wz,\
                             ca.cos(self.roll) / ca.cos(self.pitch) * self.wz,\
                             fx / vehicle_param["m"] - ca.sin(self.pitch) * self.gz,\
                             mz / vehicle_param["izz"],\
                             self.tl_rate,\
                             self.tr_rate )
        
        opts = {'abstol': 1e-8, 'reltol': 1e-8}

        dae = { 'x': state, 'u': controls, 'ode': f_expl }
        self.F = ca.integrator('F', 'cvodes', dae, 0.0, integrator_specs["Ts"], opts )
    
    def _simulate(self, state, control):
        return self.F(x0=state, u=control)