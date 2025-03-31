#!/usr/bin/python3.8

import sys
sys.path.insert(0, "/media/fmccastro/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/src")

from classes.all_imports import *
from classes.common_class import *

"""
    Build and define NMPC optimization problem
"""

#   Optimization problem parameters
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

        #   Position reference
        self.x_ref = ca.SX.sym('x_ref')
        self.y_ref = ca.SX.sym('y_ref')
        self.z_ref = ca.SX.sym('z_ref')

        self.position_ref = ca.vertcat(self.x_ref, self.y_ref, self.z_ref)

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

        #   Fixed angles reference (rpy)
        self.roll_ref = ca.SX.sym('roll_ref')
        self.pitch_ref = ca.SX.sym('pitch_ref')
        self.yaw_ref = ca.SX.sym('yaw_ref')

        self.orientation_ref = ca.vertcat(self.roll_ref, self.pitch_ref, self.yaw_ref)

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

        #   Linear velocity reference (w.r. to body frame)
        self.vx_ref = ca.SX.sym('vx_ref')
        self.vy_ref = ca.SX.sym('vy_ref')
        self.vz_ref = ca.SX.sym('vz_ref')

        self.lin_vel_ref = ca.vertcat( self.vx_ref, self.vy_ref, self.vz_ref)

        #   Angular velocity (w.r. to body frame)
        self.wx = ca.SX.sym('wx')
        self.wy = ca.SX.sym('wy')
        self.wz = ca.SX.sym('wz')

        self.ang_vel = ca.vertcat( self.wx, self.wy, self.wz )

        #   Angular velocity reference (w.r. to body frame)
        self.wx_ref = ca.SX.sym('wx_ref')
        self.wy_ref = ca.SX.sym('wy_ref')
        self.wz_ref = ca.SX.sym('wz_ref')

        self.ang_vel_ref = ca.vertcat( self.wx_ref, self.wy_ref, self.wz_ref)

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

        #   Wheel rates
        self.w_l = ca.SX.sym('w_l')
        self.w_r = ca.SX.sym('w_r')

        self.w = ca.vertcat(self.w_l, self.w_r)

        self.d_w_l = ca.SX.sym('d_w_l')
        self.d_w_r = ca.SX.sym('d_w_r')

        self.w_dot = ca.vertcat( self.d_w_l, self.d_w_r )

        self.w_l_ref = ca.SX.sym( 'w_l_ref' )
        self.w_r_ref = ca.SX.sym( 'w_r_ref' )

        self.w_ref = ca.vertcat( self.w_l_ref, self.w_r_ref)

        #   Forces at each contact point
        self.fx_l = ca.SX.sym('fx_l')
        self.fx_r = ca.SX.sym('fx_r')

        self.fx_wheels = ca.vertcat( self.fx_l, self.fx_r)

        self.fy_bl = ca.SX.sym('fy_bl')
        self.fy_fl = ca.SX.sym('fy_fl')
        self.fy_br = ca.SX.sym('fy_br')
        self.fy_fr = ca.SX.sym('fy_fr')

        self.fy_wheels = ca.vertcat( self.fy_bl, self.fy_fl, self.fy_br, self.fy_fr)

        self.fz_bl = ca.SX.sym('fz_bl')
        self.fz_fl = ca.SX.sym('fz_fl')
        self.fz_br = ca.SX.sym('fz_br')
        self.fz_fr = ca.SX.sym('fz_fr')

        self.fz_wheels = ca.vertcat( self.fz_bl, self.fz_fl, self.fz_br, self.fz_fr)

        #   General forces
        self.fx = ca.SX.sym('fx')
        self.fy = ca.SX.sym('fy')
        self.fz = ca.SX.sym('fz')

        self.f = ca.vertcat( self.fx, self.fy, self.fz )

        #   General moments
        self.mx = ca.SX.sym('mx')
        self.my = ca.SX.sym('my')
        self.mz = ca.SX.sym('mz')

        self.m = ca.vertcat( self.mx, self.my, self.mz )

        #   Wheel torques
        self.torque_l = ca.SX.sym('torque_l')
        self.torque_r = ca.SX.sym('torque_r')

        """ Rotation matrices """
        self.TransRotationMatrix = ca.horzcat( ca.vertcat( ca.cos(self.yaw) * ca.cos(self.pitch), ca.sin(self.yaw) * ca.cos(self.pitch), -ca.sin(self.pitch) ),\
                                               ca.vertcat( ca.cos(self.yaw) * ca.sin(self.pitch) * ca.sin(self.roll) - ca.sin(self.yaw) * ca.cos(self.roll), ca.sin(self.yaw) * ca.sin(self.pitch) * ca.sin(self.roll) + ca.cos(self.yaw) * ca.cos(self.roll), ca.cos(self.pitch) * ca.sin(self.roll) ),\
                                               ca.vertcat( ca.cos(self.yaw) * ca.sin(self.pitch) * ca.cos(self.roll) + ca.sin(self.yaw) * ca.sin(self.roll), ca.sin(self.yaw) * ca.sin(self.pitch) * ca.cos(self.roll) - ca.cos(self.yaw) * ca.sin(self.roll), ca.cos(self.pitch) * ca.cos(self.roll) ) )

        self.RotRotationMatrix = ca.horzcat( ca.vertcat(1, 0, 0),\
                                             ca.vertcat( ca.tan(self.pitch) * ca.sin(self.roll), ca.cos(self.roll), -ca.sin(self.roll) / ca.cos(self.pitch) ),\
                                             ca.vertcat( -ca.tan(self.pitch) * ca.cos(self.roll), ca.sin(self.roll), ca.cos(self.roll) / ca.cos(self.pitch) ) )

#   Kinematics model discretization
class Kinematics(ModelParameters, Common):

    #   Constructor
    def __init__(self):
        
        super().__init__()

        #   State
        state = ca.vertcat(self.position, self.orientation)

        #   State derivative
        state_dot = ca.vertcat(self.position_dot, self.orientation_dot)

        #   Controls
        controls = ca.vertcat(self.vx, self.wz)

        #   Parameters
        parameters = ca.vertcat(self.x_ref, self.y_ref, self.yaw_ref)

        #   Explicit model
        f_expl = ca.vertcat( self.TransRotationMatrix @ ca.vertcat(self.vx, 0.0, 0.0),\
                             self.RotRotationMatrix @ ca.vertcat(0.0, 0.0, self.wz) )
        f_impl = ca.vertcat( self.position_dot - self.TransRotationMatrix @ ca.vertcat(self.vx, 0.0, 0.0),\
                             self.orientation_dot - self.RotRotationMatrix @ ca.vertcat(0.0, 0.0, self.wz) )

        #   Call model instance
        model = AcadosModel()

        model.f_impl_expr = f_impl
        model.f_expl_expr = f_expl
        model.x = state
        model.xdot = state_dot
        model.p = parameters
        model.u = controls
        model.name = "kinematics"

        model.x_labels = ['$x$ [m]', '$y$ [m]', '$z$ [m]', r'$\phi$ [rad]', r'$\theta$ [rad]', r'$\psi$ [rad]'] 
        model.u_labels = ['$u_cmd$ [m/s]', '$p_cmd$ [rad/s]', '$q_cmd$ [rad/s]', '$r_cmd$ [rad/s]']
        model.t_label = '$t$ [s]'
        
        #error_roll = ca.power( ca.cos(self.roll) - ca.cos(self.roll_ref), 2 ) + ca.power( ca.sin(self.roll) - ca.sin(self.roll_ref), 2 )
        #error_pitch = ca.power( ca.cos(self.pitch) - ca.cos(self.pitch_ref), 2 ) + ca.power( ca.sin(self.pitch) - ca.sin(self.pitch_ref), 2 )
        error_yaw = ca.power( ca.cos(self.yaw) - ca.cos(self.yaw_ref), 2 ) + ca.power( ca.sin(self.yaw) - ca.sin(self.yaw_ref), 2 )

        y_0 = ca.vertcat(self.x - self.x_ref,\
                         self.y - self.y_ref,\
                         error_yaw,\
                         self.vx,\
                         self.wz )
    
        y = ca.vertcat(self.x - self.x_ref,\
                       self.y - self.y_ref,\
                       error_yaw,\
                       self.vx,\
                       self.wz )
    
        y_e = ca.vertcat(self.x - self.x_ref,\
                         self.y - self.y_ref,\
                         error_yaw )

        model.cost_expr_ext_cost_0 = y_0.T @ scipy.linalg.block_diag(self.Q_p[0, 0], self.Q_p[1, 1], self.Q_o[2, 2], self.Q_v[0, 0], self.Q_w[2, 2]) @ y_0
        
        model.cost_expr_ext_cost = y.T @ scipy.linalg.block_diag(self.Q_p[0, 0], self.Q_p[1, 1], self.Q_o[2, 2], self.Q_v[0, 0], self.Q_w[2, 2]) @ y
        
        model.cost_expr_ext_cost_e = y_e.T @ scipy.linalg.block_diag(self.Q_p[0, 0], self.Q_p[1, 1], self.Q_o[2, 2]) @ y_e
        ###

        #   Call dims instance
        dims = AcadosOcpDims()

        dims.N = self.N
        dims.nx = model.x.rows()
        dims.nu = model.u.rows()
        dims.np = model.p.rows()
        ###

        self.nx = dims.nx
        self.nu = dims.nu
        self.np = dims.np

        #   Call cost instance
        cost = AcadosOcpCost()

        cost.cost_type_0 = 'EXTERNAL'
        cost.cost_type = 'EXTERNAL'
        cost.cost_type_e = 'EXTERNAL'

        cost.cost_ext_fun_type_0 = 'casadi'
        cost.cost_ext_fun_type = 'casadi'
        cost.cost_ext_fun_type_e = 'casadi'
        ###
        
        #  Call constraints instance
        constraints = AcadosOcpConstraints()

        #constraints.lbx_0 = np.stack( self.p_lb + self.q_lb )
        #constraints.lbx = np.stack( self.p_lb + self.q_lb )
        #constraints.lbx_e = np.stack( self.p_lb + self.q_lb )

        #constraints.ubx_0 = np.stack( self.p_ub + self.q_ub )
        #constraints.ubx = np.stack( self.p_ub + self.q_ub )
        #constraints.ubx_e = np.stack( self.p_ub + self.q_ub )

        constraints.lbu = np.stack( [ self.v_lb[0], self.w_lb[2] ] )
        constraints.ubu = np.stack( [ self.v_ub[0], self.w_ub[2] ] )

        #constraints.idxbx_0 = np.array( [0, 1, 2, 3, 4, 5, 6] )
        #constraints.idxbx = np.array( [0, 1, 2, 3, 4, 5, 6] )
        #constraints.idxbx_e = np.array( [0, 1, 2, 3, 4, 5, 6] )
        #constraints.idxbxe_0 = np.array( [0, 1, 2, 3, 4, 5, 6] )

        constraints.idxbu = np.array( [0, 1] )

        constraints.x0 = np.stack( [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] ) 
        ###

        #   Call solver options instance
        solver_options = AcadosOcpOptions()

        solver_options.N_horizon = self.N
        solver_options.tf = self.Ts * self.N
        solver_options.Tsim = self.Ts
        solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        solver_options.hessian_approx = 'EXACT'
        solver_options.integrator_type = 'ERK'

        if( self.nlp_solver_type == 'SQP_RTI' ):
            solver_options.nlp_solver_type = self.nlp_solver_type

        elif( self.nlp_solver_type == 'SQP' ):
            solver_options.nlp_solver_type = self.nlp_solver_type
            solver_options.globalization = 'MERIT_BACKTRACKING'
            solver_options.nlp_solver_max_iter = 150

        solver_options.qp_solver_warm_start = 1
        solver_options.qp_solver_cond_N = int(self.N / 4)
        solver_options.print_level = 0
        solver_options.regularize_method = 'CONVEXIFY'
        solver_options.sim_method_num_stages = 4
        solver_options.sim_method_num_steps = 3
        solver_options.sim_method_newton_iter = 20
        solver_options.qp_solver_iter_max = 100
        solver_options.num_threads_in_batch_solve = 4
        solver_options.line_search_use_sufficient_descent = 0
        #ocp.solver_options.ext_cost_num_hess = 1
        ###

        #   Call ocp instance
        ocp = AcadosOcp()

        #   Set ocp
        ocp.model = model
        ocp.cost = cost
        ocp.dims = dims
        ocp.solver_options = solver_options
        ocp.constraints = constraints
        ocp.parameter_values = np.ones( dims.np )

        #   Set folder path where generated c code is located
        ocp.code_export_directory = "/media/fmccastro/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/scripts/acados_c_generated_code/Kinematics/acadosOcp"
        ocp.acados_lib_path = "/media/fmccastro/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/acados/lib"
        ocp.acados_include_path = "/media/fmccastro/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/acados/include"

        json_file_path = "/media/fmccastro/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/scripts/acados_c_generated_code/Kinematics/acadosOcp/acados_ocp.json"
        ###

        self.solver = AcadosOcpSolver(ocp, json_file = json_file_path)
        """AcadosOcpSolver.generate(ocp, json_file = json_file_path)
        AcadosOcpSolver.build(ocp.code_export_directory, with_cython=True)
        self.solver = AcadosOcpSolver.create_cython_solver(json_file_path)"""

        self.integrator = AcadosSimSolver(ocp, json_file = json_file_path)

    def _constraints(self, pose, reference):

        """
            Return constraints to input at NMPC model on each iteration
            
            :pose geometry_msgs/Pose.msg type, custom message
            :iteration (int)
            :reference (list type), list with reference path to follow
        """

        #   Set initial state
        self.solver.set(0, 'lbx', np.stack( [ pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw ] ) )
        self.solver.set(0, 'ubx', np.stack( [ pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw ] ) )

        for i in range(self.N + 1):
            
            if( i == self.N ):
                self.solver.set(i, 'p', np.stack( reference[-self.np:] ) )

            else:
                self.solver.set(i, 'p', np.stack( reference[i * self.np : (i + 1) * self.np] ) )

    def _setInitialGuess(self, numIter, pose, reference):
        
        """ 
            :numIter int type, number of iterations
            :pose geometry_msgs/Pose.msg type, mobile robot pose
        """ 

        x0 = np.stack( [ pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw ] )

        # do some initial iterations to start with a good initial guess
        for _ in range(numIter):
            print("Initial iterations: ", _)

            self._constraints(pose, reference)
            
            u0 = self.solver.solve_for_x0(x0_bar = x0)

            print("Total time: ", self.solver.get_stats("time_tot"))

    def _solve(self, pose, reference):

        x0 = pose

        if( self.nlp_solver_type == 'SQP' ):
            #   Retrieve optimization problem constraints, initial guess and parameters
            self._constraints(x0, reference)

            status = self.solver.solve()

            #print("Total time: ", self.solver.get_stats("time_tot"))

            if status != 0:
                print("acados returned status {} in closed loop iteration.".format(status))

        elif( self.nlp_solver_type == 'SQP_RTI' ):
            self.solver.options_set('rti_phase', 1)
            status1 = self.solver.solve()

            t_preparation = self.solver.get_stats("time_tot")

            #   Retrieve optimization problem constraints, initial guess and parameters
            self._constraints(x0, reference)

            self.solver.options_set('rti_phase', 2)
            status2 = self.solver.solve()

            t_feedback = self.solver.get_stats("time_tot")

            print("Total time: ", t_preparation + t_feedback)

            if( status1 != 0 or status2 != 0 ):
                print("acados returned status {} in preparation phase.".format(status1))
                print("acados returned status {} in feedback phase.".format(status2))

        #self.solver.print_statistics()

        solutionX, solutionU, next_vx, next_wz = self._data()

        return solutionX, solutionU, next_vx, next_wz
        
    def _data(self):

        """ 
            Optimization solution data treatment

            solution: dictionary returned by casadi solver
        """ 

        solutionX = []
        solutionU = []

        for i in range(self.N + 1):

            #   solutionX -> get optimized states solution
            opt_x = self.solver.get(i, 'x')

            solutionX += list(opt_x)
            
            if(i < self.N):
                #   solutionU -> get optimized controls solution
                opt_u = self.solver.get(i, 'u')

                if( i == 0 ):
                    next_vx = opt_u[0]
                    next_wz = opt_u[1]

                solutionU += list(opt_u)

        return solutionX, solutionU, next_vx, next_wz
    
#   Full dynamics model
class Dynamics(ModelParameters, Common):

    """
        Contact angles between wheel and terrain are neglected.
        Thus, traction and friction is assumed to hold a direction longitudinal to the body frame longitudinal direction.
    """

    #   Constructor
    def __init__(self):
        
        """
            :com2wheel dictionary
        """
        
        super().__init__()

        #   state
        state = ca.vertcat(self.lin_vel, self.ang_vel, self.w )
        
        #   Derivative state
        state_dot = ca.vertcat(self.lin_vel_dot, self.ang_vel_dot, self.w_dot )

        #   Parameters
        parameters = ca.vertcat(self.roll, self.pitch, self.vx_ref, self.wz_ref)

        #   Inertia tensor of vehicle
        inertia = ca.horzcat( ca.vertcat(self.ixx, self.ixy, self.ixz),\
                              ca.vertcat(self.ixy, self.iyy, self.iyz),\
                              ca.vertcat(self.ixz, self.iyz, self.izz) )

        #   Controls
        controls = ca.vertcat(self.fx, self.mz)

        #   Gravity
        gravity = self.TransRotationMatrix.T @ ca.vertcat(0, 0, self.gz)
        robotWeight = gravity * self.robotMass

        v_dot = -ca.cross( self.ang_vel, self.lin_vel ) + ca.vertcat(self.fx - robotWeight[0], 0.0, 0.0) / self.robotMass
        w_dot = ca.inv_minor( inertia ) @ ( -ca.cross( self.ang_vel, inertia @ self.ang_vel ) + ca.vertcat(0.0, 0.0, self.mz) )
        
        #   Explicit model
        f_expl = ca.vertcat( -ca.cross( self.ang_vel, self.lin_vel ) + ca.vertcat(self.fx - robotWeight[0], 0.0, 0.0) / self.robotMass,\
                             ca.inv_minor( inertia ) @ ( -ca.cross( self.ang_vel, inertia @ self.ang_vel ) + ca.vertcat(0.0, 0.0, self.mz) ),\
                             v_dot[0] / self.wheelRadius - w_dot[2] * self.robotWidth / (2 * self.wheelRadius),\
                             v_dot[0] / self.wheelRadius + w_dot[2] * self.robotWidth / (2 * self.wheelRadius) )
                                  
        #   Implicit model
        f_impl = ca.vertcat( self.lin_vel_dot + ca.cross( self.ang_vel, self.lin_vel ) - ca.vertcat(self.fx - robotWeight[0], 0.0, 0.0) / self.robotMass,\
                             self.ang_vel_dot - ca.inv_minor( inertia ) @ ( -ca.cross( self.ang_vel, inertia @ self.ang_vel ) + ca.vertcat(0.0, 0.0, self.mz) ),\
                             self.d_w_l - v_dot[0] / self.wheelRadius + w_dot[2] * self.robotWidth / (2 * self.wheelRadius),\
                             self.d_w_r - v_dot[0] / self.wheelRadius - w_dot[2] * self.robotWidth / (2 * self.wheelRadius) )

        #   Call model instance
        model = AcadosModel()

        model.f_impl_expr = f_impl
        model.f_expl_expr = f_expl
        
        model.x = state
        model.xdot = state_dot
        model.u = controls
        model.p = parameters
        
        model.name = "dynamics"

        model.x_labels = ['$u$ [m/s]', '$v$ [m/s]', '$w$ [m/s]', '$p$ [rad/s]', '$q$ [rad/s]', '$r$ [rad/s]']
        model.u_labels = [r'f_x [N]', r'f_y [N]', r'f_z [N]', r'm_z [Nm]']
        model.t_label = '$t$ [s]'

        y_0 = ca.vertcat(self.vx - self.vx_ref,\
                         self.vy,\
                         self.vz,\
                         self.wz - self.wz_ref,\
                         self.fx,\
                         self.mz,\
                         self.w )
    
        y = ca.vertcat(self.vx - self.vx_ref,\
                       self.vy,\
                       self.vz,\
                       self.wz - self.wz_ref,\
                       self.fx,\
                       self.mz,\
                       self.w )
    
        y_e = ca.vertcat(self.vx - self.vx_ref,\
                         self.vy,\
                         self.vz,\
                         self.wz - self.wz_ref,\
                         self.w )

        model.cost_expr_ext_cost_0 = y_0.T @ scipy.linalg.block_diag(self.Q_v, self.Q_w[2, 2], self.Q_f[0, 0], self.Q_m[2, 2], 1e0, 1e0 ) @ y_0
        model.cost_expr_ext_cost = y.T @ scipy.linalg.block_diag(self.Q_v, self.Q_w[2, 2], self.Q_f[0, 0], self.Q_m[2, 2], 1e0, 1e0 ) @ y
        model.cost_expr_ext_cost_e = y_e.T @ scipy.linalg.block_diag(self.Q_v, self.Q_w[2, 2], 1e0, 1e0) @ y_e
        ###

        #   Call dims instance
        dims = AcadosOcpDims()

        dims.N = self.N
        dims.nu = model.u.rows()
        dims.nx = model.x.rows()
        dims.np = model.p.rows()
        ###
        
        self.nx = dims.nx
        self.nu = dims.nu
        self.np = dims.np

        #   Call cost instance
        cost = AcadosOcpCost()

        cost.cost_type_0 = 'EXTERNAL'
        cost.cost_type = 'EXTERNAL'
        cost.cost_type_e = 'EXTERNAL'

        cost.cost_ext_fun_type_0 = 'casadi'
        cost.cost_ext_fun_type = 'casadi'
        cost.cost_ext_fun_type_e = 'casadi'
        ###

        #  Call constraints instance
        constraints = AcadosOcpConstraints()

        #constraints.lbx_0 = np.stack( [ self.p_lb[0], self.p_lb[1], self.qsi_lb[2], self.v_lb[0], self.v_lb[1], self.w_lb[2] ] )
        #constraints.lbx = np.stack( [ self.p_lb[0], self.p_lb[1], self.qsi_lb[0], self.qsi_lb[1], self.qsi_lb[2], self.v_lb[0], self.v_lb[1], self.w_lb[2] ] )
        #constraints.lbx_e = np.stack( [ self.p_lb[0], self.p_lb[1], self.qsi_lb[0], self.qsi_lb[1], self.qsi_lb[2], self.v_lb[0], self.v_lb[1], self.w_lb[2] ] )

        #constraints.ubx_0 = np.stack( [ self.p_ub[0], self.p_ub[1], self.qsi_ub[2], self.v_ub[0], self.v_ub[1], self.w_ub[2] ] )
        #constraints.ubx = np.stack( [ self.p_ub[0], self.p_ub[1], self.qsi_ub[0], self.qsi_ub[1], self.qsi_ub[2], self.v_ub[0], self.v_ub[1], self.w_ub[2] ] )
        #constraints.ubx_e = np.stack( [ self.p_ub[0], self.p_ub[1], self.qsi_ub[0], self.qsi_ub[1], self.qsi_ub[2], self.v_ub[0], self.v_ub[1], self.w_ub[2] ] )

        constraints.lbu = np.stack( [ self.f_lb[0], self.m_lb[2] ] )
        constraints.ubu = np.stack( [ self.f_ub[0], self.m_ub[2] ] )
        
        #constraints.idxbx_0 = np.array( [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15] )
        #constraints.idxbx = np.array( [0, 1, 3, 4, 5, 6, 7, 11] )
        #constraints.idxbx_e = np.array( [0, 1, 3, 4, 5, 6, 7, 11] )
        #constraints.idxbxe_0 = np.array( [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15] )
        
        constraints.idxbu = np.array( [0, 1] )

        constraints.x0 = np.stack( [0, 0, 0, 0, 0, 0, 0, 0] ) 
        ###
        
        #   Call solver options instance
        solver_options = AcadosOcpOptions()

        solver_options.N_horizon = self.N
        solver_options.tf = self.Ts * self.N
        solver_options.Tsim = self.Ts
        solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        solver_options.hessian_approx = 'EXACT'
        solver_options.integrator_type = 'ERK'
        
        if( self.nlp_solver_type == 'SQP_RTI' ):
            solver_options.nlp_solver_type = self.nlp_solver_type

        elif( self.nlp_solver_type == 'SQP' ):
            solver_options.nlp_solver_type = self.nlp_solver_type
            solver_options.globalization = 'FIXED_STEP'
            solver_options.nlp_solver_max_iter = 50

        solver_options.qp_solver_warm_start = 1
        solver_options.qp_solver_cond_N = self.N
        solver_options.print_level = 0
        solver_options.regularize_method = 'CONVEXIFY'
        solver_options.sim_method_num_stages = 4
        solver_options.sim_method_num_steps = 3
        solver_options.sim_method_newton_iter = 20
        solver_options.qp_solver_iter_max = 25
        solver_options.num_threads_in_batch_solve = 4
        solver_options.line_search_use_sufficient_descent = 0
        solver_options.levenberg_marquardt = 0.0
        #solver_options.output_z = False
        solver_options.qp_tol = 1e-1
        solver_options.tol = 1e-1
        #ocp.solver_options.ext_cost_num_hess = 1
        ###

        #   Call ocp instance
        ocp = AcadosOcp()

        #   Set ocp
        ocp.model = model
        ocp.cost = cost
        ocp.dims = dims
        ocp.solver_options = solver_options
        ocp.constraints = constraints
        ocp.parameter_values = np.ones( dims.np )

        #   Set folder path where generated c code is located
        ocp.code_export_directory = "/media/fmccastro/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/scripts/acados_c_generated_code/Dynamics/acadosOcp"
        ocp.acados_lib_path = "/media/fmccastro/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/acados/lib"
        ocp.acados_include_path = "/media/fmccastro/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/acados/include"

        json_file_path = "/media/fmccastro/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/scripts/acados_c_generated_code/Dynamics/acadosOcp/acados_ocp.json"
        ###

        self.solver = AcadosOcpSolver(ocp, json_file = json_file_path)
    
    def _constraints(self, wheelRates, initialState, velocityReference, pathReference ):

        """
            Return constraints to input at NMPC model on each iteration

            :initialState       [vx, vy, vz, wx, wy, wz]
            :velocityReference  [vx, wz]_ref
            :pathReference      [x, y, z, roll, pitch, yaw]_ref
        """

        vx0 = initialState.linear.x
        vy0 = initialState.linear.y
        vz0 = initialState.linear.z
        wx0 = initialState.angular.x
        wy0 = initialState.angular.y
        wz0 = initialState.angular.z

        w_l = wheelRates[0]
        w_r = wheelRates[1]

        #   Set initial state
        self.solver.set(0, 'lbx', np.stack( [vx0, vy0, vz0, wx0, wy0, wz0, w_l, w_r] ) )
        self.solver.set(0, 'ubx', np.stack( [vx0, vy0, vz0, wx0, wy0, wz0, w_l, w_r] ) )

        for i in range(self.N + 1):
            reference = np.stack( [ pathReference[i * (self.NbPosition + self.NbOrientation) + 3],  pathReference[i * (self.NbPosition + self.NbOrientation) + 4] ]\
                                    + velocityReference[ i * 2 : (i + 1) * 2 ] )
            self.solver.set(i, 'p', reference )
        
    def _setInitialGuess(self, numIter, wheelRates, velocity, velocityReference, pathReference):
        
        """
            :numIter int type, number of iterations
            :velocity                       [vx, vy, vz, wx, wy, wz]
            :velocityReference              [vx, wz]_ref
            :pathReference                  [x, y, z, roll, pitch, yaw]_ref
        """

        vx0 = velocity.linear.x
        vy0 = velocity.linear.y
        vz0 = velocity.linear.z
        wx0 = velocity.angular.x
        wy0 = velocity.angular.y
        wz0 = velocity.angular.z

        w_l = wheelRates[0]
        w_r = wheelRates[1]

        # do some initial iterations to start with a good initial guess
        for _ in range(numIter):
            
            self._constraints(wheelRates, velocity, velocityReference, pathReference)

            u0 = self.solver.solve_for_x0(x0_bar = np.stack( [vx0, vy0, vz0, wx0, wy0, wz0, w_l, w_r] ))

            #self.solver.dump_last_qp_to_json("/home/fmccastro/Desktop/last_qp.json")

            print("Total time: ", self.solver.get_stats("time_tot"))

    def _solve(self, wheelRates, velocity, velocityReference, pathReference):

        """
            :velocity                       [vx, vy, vz, wx, wy, wz]
            :velocityReference              [vx, wz]_ref
            :pathReference                  [x, y, z, roll, pitch, yaw]_ref
        """

        state_0 = velocity

        if( self.nlp_solver_type == 'SQP' ):
            #   Retrieve optimization problem constraints, initial guess and parameters
            self._constraints(wheelRates, state_0, velocityReference, pathReference)

            status = self.solver.solve()

            #print("Total time: ", self.solver.get_stats("time_tot"))

            if status != 0:
                print("acados returned status {} in closed loop iteration.".format(status))

        elif( self.nlp_solver_type == 'SQP_RTI' ):
            self.solver.options_set('rti_phase', 1)
            status1 = self.solver.solve()

            t_preparation = self.solver.get_stats("time_tot")

            #   Retrieve optimization problem constraints, initial guess and parameters
            self._constraints(wheelRates, state_0, velocityReference, pathReference)

            self.solver.options_set('rti_phase', 2)
            status2 = self.solver.solve()

            t_feedback = self.solver.get_stats("time_tot")

            print("Total time: ", t_preparation + t_feedback)

            if( status1 != 0 or status2 != 0 ):
                print("acados returned status {} in preparation phase.".format(status1))
                print("acados returned status {} in feedback phase.".format(status2))
        
        #self.solver.print_statistics()

        solutionX, solutionU, next_fx, next_mz, w_l, w_r = self._data()

        return solutionX, solutionU, next_fx, next_mz, w_l, w_r

    def _data(self):
        
        """ 
            Optimization solution data treatment

            solution: dictionary returned by casadi solver
        """ 
            
        solutionX = []
        solutionU = []

        for i in range(self.N + 1):
            #   solutionX -> get optimized states solution
            opt_x = self.solver.get(i, 'x')

            if(i == 1):
                w_l = opt_x[6]
                w_r = opt_x[7]

            solutionX += list(opt_x) 

            if(i < self.N):
                #   solutionU -> get optimized controls solution
                opt_u = self.solver.get(i, 'u')

                if( i == 0 ):
                    next_fx = opt_u[0]
                    next_mz = opt_u[1]
                
                solutionU += list(opt_u)

        return solutionX, solutionU, next_fx, next_mz, w_l, w_r

class WheelTorqueAllocation(ModelParameters, Common):

    #   Constructor
    def __init__(self, com2wheel):
        
        """
            :com2wheel dictionary
        """
        
        super().__init__()

        #   State
        state = ca.vertcat(self.w)
        
        #   Derivative state
        state_dot = ca.vertcat(self.w_dot)

        #   Parameters
        parameters = ca.vertcat(self.roll, self.pitch, self.lin_vel, self.ang_vel, self.fx, self.mz)

        #   Controls (normals forces and wheel torques)
        controls = ca.vertcat(self.fz_wheels, self.torque_l, self.torque_r)
        
        #   Gravity
        gravity = self.TransRotationMatrix.T @ ca.vertcat(0, 0, self.gz)
        
        #   Vector from robot center of mass to each wheel center (w.r.t to body frame)
        com2bl = ca.vertcat( com2wheel["com2bl"][0], com2wheel["com2bl"][1], com2wheel["com2bl"][2] )
        com2fl = ca.vertcat( com2wheel["com2fl"][0], com2wheel["com2fl"][1], com2wheel["com2fl"][2] )
        com2br = ca.vertcat( com2wheel["com2br"][0], com2wheel["com2br"][1], com2wheel["com2br"][2] )
        com2fr = ca.vertcat( com2wheel["com2fr"][0], com2wheel["com2fr"][1], com2wheel["com2fr"][2] )

        com2bl_contact = ca.vertcat( com2wheel["com2bl"][0], com2wheel["com2bl"][1], com2wheel["com2bl"][2] - self.wheelRadius )
        com2fl_contact = ca.vertcat( com2wheel["com2fl"][0], com2wheel["com2fl"][1], com2wheel["com2fl"][2] - self.wheelRadius )
        com2br_contact = ca.vertcat( com2wheel["com2br"][0], com2wheel["com2br"][1], com2wheel["com2br"][2] - self.wheelRadius )
        com2fr_contact = ca.vertcat( com2wheel["com2fr"][0], com2wheel["com2fr"][1], com2wheel["com2fr"][2] - self.wheelRadius )

        S_bl = ca.skew( com2bl_contact )
        S_fl = ca.skew( com2fl_contact ) 
        S_br = ca.skew( com2br_contact )
        S_fr = ca.skew( com2fr_contact )

        #   Wheel velocity w.r.t body frame
        v_bl = self.lin_vel + ca.cross( self.ang_vel, com2bl )
        v_fl = self.lin_vel + ca.cross( self.ang_vel, com2fl )
        v_br = self.lin_vel + ca.cross( self.ang_vel, com2br )
        v_fr = self.lin_vel + ca.cross( self.ang_vel, com2fr )
        
        slip_bl = self.wheelRadius * self.w_bl - v_bl[0, 0]
        slip_fl = self.wheelRadius * self.w_fl - v_fl[0, 0]
        slip_br = self.wheelRadius * self.w_br - v_br[0, 0]
        slip_fr = self.wheelRadius * self.w_fr - v_fr[0, 0]

        #   Traction
        #traction_1_bl = self.torque_l / self.loadRadius
        #traction_1_fl = self.torque_l / self.loadRadius
        #traction_1_br = self.torque_r / self.loadRadius
        #traction_1_fr = self.torque_r / self.loadRadius

        #traction_2_bl = self.niu_c * self.fz_bl * ca.tanh( 1e2 * self.torque_l )
        #traction_2_fl = self.niu_c * self.fz_fl * ca.tanh( 1e2 * self.torque_l )
        #traction_2_br = self.niu_c * self.fz_br * ca.tanh( 1e2 * self.torque_r )
        #traction_2_fr = self.niu_c * self.fz_fr * ca.tanh( 1e2 * self.torque_r )

        #a_t_bl = ca.sqrt( ca.power( self.torque_l / self.loadRadius, 2 ) + 1e-6 ) - self.niu_c * self.fz_bl
        #a_t_fl = ca.sqrt( ca.power( self.torque_l / self.loadRadius, 2 ) + 1e-6 ) - self.niu_c * self.fz_fl
        #a_t_br = ca.sqrt( ca.power( self.torque_r / self.loadRadius, 2 ) + 1e-6 ) - self.niu_c * self.fz_br
        #a_t_fr = ca.sqrt( ca.power( self.torque_r / self.loadRadius, 2 ) + 1e-6 ) - self.niu_c * self.fz_fr

        #sigmoid_a_t_bl = 1 / ( 1 + ca.exp( -a_t_bl ) )
        #sigmoid_a_t_fl = 1 / ( 1 + ca.exp( -a_t_fl ) )
        #sigmoid_a_t_br = 1 / ( 1 + ca.exp( -a_t_br ) )
        #sigmoid_a_t_fr = 1 / ( 1 + ca.exp( -a_t_fr ) )

        #   Lateral friction
        #lat_f_bl = -self.niu_c * self.fz_bl * ca.tanh( 1e2 * v_bl[1] )
        #lat_f_fl = -self.niu_c * self.fz_fl * ca.tanh( 1e2 * v_fl[1] )
        #lat_f_br = -self.niu_c * self.fz_br * ca.tanh( 1e2 * v_br[1] )
        #lat_f_fr = -self.niu_c * self.fz_fr * ca.tanh( 1e2 * v_fr[1] )

        #t_bl = ( (1 - sigmoid_a_t_bl) * traction_1_bl + sigmoid_a_t_bl * traction_2_bl ) * ca.tanh( 1e2 * ca.sqrt( ca.power( slip_bl, 2 ) + 1e-6 ) )
        #t_fl = ( (1 - sigmoid_a_t_fl) * traction_1_fl + sigmoid_a_t_fl * traction_2_fl ) * ca.tanh( 1e2 * ca.sqrt( ca.power( slip_fl, 2 ) + 1e-6 ) )
        #t_br = ( (1 - sigmoid_a_t_br) * traction_1_br + sigmoid_a_t_br * traction_2_br ) * ca.tanh( 1e2 * ca.sqrt( ca.power( slip_br, 2 ) + 1e-6 ) )
        #t_fr = ( (1 - sigmoid_a_t_fr) * traction_1_fr + sigmoid_a_t_fr * traction_2_fr ) * ca.tanh( 1e2 * ca.sqrt( ca.power( slip_fr, 2 ) + 1e-6 ) )

        t_bl = self.torque_l / self.wheelRadius
        t_fl = self.torque_l / self.wheelRadius
        t_br = self.torque_r / self.wheelRadius
        t_fr = self.torque_r / self.wheelRadius

        #t_bl = self.niu_c * self.fz_bl * ca.tanh(1e2 * slip_bl)
        #t_fl = self.niu_c * self.fz_fl * ca.tanh(1e2 * slip_fl)
        #t_br = self.niu_c * self.fz_br * ca.tanh(1e2 * slip_br)
        #t_fr = self.niu_c * self.fz_fr * ca.tanh(1e2 * slip_fr)

        #   Force transferred to the ground by wheel
        f_bl = ca.vertcat( t_bl, 0.0, self.fz_bl)
        f_fl = ca.vertcat( t_fl, 0.0, self.fz_fl)
        f_br = ca.vertcat( t_br, 0.0, self.fz_br)
        f_fr = ca.vertcat( t_fr, 0.0, self.fz_fr)

        gravity = self.TransRotationMatrix.T @ ca.vertcat(0, 0, self.gz)

        sumForces = gravity * self.robotMass + f_bl + f_fl + f_br + f_fr - ca.vertcat(self.fx, 0.0, 0.0)

        m_bl = S_bl @ f_bl
        m_fl = S_fl @ f_fl
        m_br = S_br @ f_br
        m_fr = S_fr @ f_fr

        sumMoments = m_bl + m_fl + m_br + m_fr - ca.vertcat(0.0, 0.0, self.mz)

        """f_expl = ca.vertcat( 1 / self.i_wheel * ( self.i_wheel * gravity[0] / self.wheelRadius + self.torque_l - self.wheelRadius * t_bl ),\
                             1 / self.i_wheel * ( self.i_wheel * gravity[0] / self.wheelRadius + self.torque_l - self.wheelRadius * t_fl ),\
                             1 / self.i_wheel * ( self.i_wheel * gravity[0] / self.wheelRadius + self.torque_r - self.wheelRadius * t_br ),\
                             1 / self.i_wheel * ( self.i_wheel * gravity[0] / self.wheelRadius + self.torque_r - self.wheelRadius * t_fr ) )"""

        """f_impl = ca.vertcat( self.d_w_bl - 1 / self.i_wheel * ( self.i_wheel * gravity[0] / self.wheelRadius + self.torque_l - self.wheelRadius * t_bl ),\
                             self.d_w_fl - 1 / self.i_wheel * ( self.i_wheel * gravity[0] / self.wheelRadius + self.torque_l - self.wheelRadius * t_fl ),\
                             self.d_w_br - 1 / self.i_wheel * ( self.i_wheel * gravity[0] / self.wheelRadius + self.torque_r - self.wheelRadius * t_br ),\
                             self.d_w_fr - 1 / self.i_wheel * ( self.i_wheel * gravity[0] / self.wheelRadius + self.torque_r - self.wheelRadius * t_fr ) )"""

        #   Call model instance
        model = AcadosModel()

        #model.f_impl_expr = f_impl
        #model.f_expl_expr = f_expl
        
        #model.x = state
        #model.xdot = state_dot
        model.u = controls
        model.p = parameters
        model.con_h_expr_0 = ca.vertcat( ca.power(t_bl, 2) - ca.power(self.niu_c * self.fz_bl * self.niu_c * ca.tanh( 100 * slip_bl ), 2),\
                                         ca.power(t_fl, 2) - ca.power(self.niu_c * self.fz_fl * self.niu_c * ca.tanh( 100 * slip_fl ), 2),\
                                         ca.power(t_br, 2) - ca.power(self.niu_c * self.fz_br * self.niu_c * ca.tanh( 100 * slip_br ), 2),\
                                         ca.power(t_fr, 2) - ca.power(self.niu_c * self.fz_fr * self.niu_c * ca.tanh( 100 * slip_fr ), 2) )
        
        model.con_h_expr = ca.vertcat( ca.power(t_bl, 2) - ca.power(self.niu_c * self.fz_bl * self.niu_c * ca.tanh( 100 * slip_bl ), 2),\
                                       ca.power(t_fl, 2) - ca.power(self.niu_c * self.fz_fl * self.niu_c * ca.tanh( 100 * slip_fl ), 2),\
                                       ca.power(t_br, 2) - ca.power(self.niu_c * self.fz_br * self.niu_c * ca.tanh( 100 * slip_br ), 2),\
                                       ca.power(t_fr, 2) - ca.power(self.niu_c * self.fz_fr * self.niu_c * ca.tanh( 100 * slip_fr ), 2) )
        
        model.name = "wheel_force_allocation"

        #model.x_labels = ['$w_l$ [rad/s]', '$w_r$ [rad/s]']
        model.u_labels = [r'fz_bl [N]', r'fz_fl [N]', r'fz_br [N]', r'fz_fr [N]', r'tau_l [Nm]', r'tau_r [Nm]']
        model.t_label = '$t$ [s]'

        y_0 = ca.vertcat( sumForces, sumMoments[2, 0], self.torque_l, self.torque_r )
        y = ca.vertcat( sumForces, sumMoments[2, 0], self.torque_l, self.torque_r )
        #y_e = self.w

        model.cost_expr_ext_cost_0 = y_0.T @ scipy.linalg.block_diag( self.Q_fn, self.Q_torque ) @ y_0
        #model.cost_expr_ext_cost = y.T @ scipy.linalg.block_diag( self.Q_fn, self.Q_torque ) @ y
        #model.cost_expr_ext_cost_e = 1
        #model.cost_expr_ext_cost_e = y_e.T @ scipy.linalg.block_diag( self.Q_wheel_rate ) @ y_e
        ###

        #   Call dims instance
        dims = AcadosOcpDims()

        dims.N = 1
        dims.nu = model.u.rows()
        #dims.nx = model.x.rows()
        dims.np = model.p.rows()
        #dims.ng = 2
        #dims.ng_e = 2
        ###
        
        self.nx = dims.nx
        self.nu = dims.nu
        self.np = dims.np

        #   Call cost instance
        cost = AcadosOcpCost()

        cost.cost_type_0 = 'EXTERNAL'
        #cost.cost_type = 'EXTERNAL'
        #cost.cost_type_e = 'EXTERNAL'

        cost.cost_ext_fun_type_0 = 'casadi'
        #cost.cost_ext_fun_type = 'casadi'
        #cost.cost_ext_fun_type_e = 'casadi'
        ###

        #  Call constraints instance
        constraints = AcadosOcpConstraints()

        """constraints.C = np.array( [ [0.0, 0.0, 0.0, 0.0],\
                                    [0.0, 0.0, 0.0, 0.0] ] )
        
        constraints.C_e = np.array( [ [0.0, 0.0, 0.0, 0.0],\
                                      [0.0, 0.0, 0.0, 0.0] ] )
        
        constraints.D = np.array( [ [0.0, 0.0, 0.0, 0.0, 1 / self.wheelRadius, 1 / self.wheelRadius],\
                                    [0.0, 0.0, 0.0, 0.0, 1 / self.wheelRadius, 1 / self.wheelRadius] ] )"""

        """constraints.lg = np.stack( [self., 0] )
        constraints.lg_e = np.stack( [0, 0] )

        constraints.ug = np.stack( [0, 0] )
        constraints.ug_e = np.stack( [0, 0] )"""

        constraints.lbu = np.stack( self.fn_lb + self.torque_lb )
        constraints.ubu = np.stack( self.fn_ub + self.torque_ub )

        #constraints.lh_0 = np.stack( [-999999.0, -999999.0, -999999.0, -999999.0] )
        #constraints.lh = np.stack( [-999999.0, -999999.0, -999999.0, -999999.0] )

        #constraints.uh_0 = np.stack( [0, 0, 0, 0] )
        #constraints.uh = np.stack( [0, 0, 0, 0] )

        constraints.idxbu = np.array( [0, 1, 2, 3, 4, 5] )

        #constraints.x0 = np.stack( [0, 0, 0, 0] )
        ###
        
        #   Call solver options instance
        solver_options = AcadosOcpOptions()

        solver_options.N_horizon = self.N
        solver_options.tf = self.Ts * self.N
        solver_options.Tsim = self.Ts
        solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        solver_options.hessian_approx = 'EXACT'
        solver_options.integrator_type = 'ERK'
        
        if( self.nlp_solver_type == 'SQP_RTI' ):
            solver_options.nlp_solver_type = self.nlp_solver_type

        elif( self.nlp_solver_type == 'SQP' ):
            solver_options.nlp_solver_type = self.nlp_solver_type
            solver_options.globalization = 'FIXED_STEP'
            solver_options.nlp_solver_max_iter = 50

        solver_options.qp_solver_warm_start = 1
        solver_options.qp_solver_cond_N = self.N
        solver_options.print_level = 0
        solver_options.regularize_method = 'CONVEXIFY'
        solver_options.sim_method_num_stages = 4
        solver_options.sim_method_num_steps = 3
        solver_options.sim_method_newton_iter = 20
        solver_options.qp_solver_iter_max = 25
        solver_options.num_threads_in_batch_solve = 4
        solver_options.line_search_use_sufficient_descent = 0
        solver_options.levenberg_marquardt = 0.0
        #solver_options.output_z = False
        solver_options.qp_tol = 1e-1
        solver_options.tol = 1e-1
        #ocp.solver_options.ext_cost_num_hess = 1
        ###

        #   Call ocp instance
        ocp = AcadosOcp()

        #   Set ocp
        ocp.model = model
        ocp.cost = cost
        ocp.dims = dims
        ocp.solver_options = solver_options
        ocp.constraints = constraints
        ocp.parameter_values = np.ones( dims.np )

        #   Set folder path where generated c code is located
        ocp.code_export_directory = "/media/fmccastro/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/scripts/acados_c_generated_code/WheelTorqueAllocation/acadosOcp"
        ocp.acados_lib_path = "/media/fmccastro/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/acados/lib"
        ocp.acados_include_path = "/media/fmccastro/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/acados/include"

        json_file_path = "/media/fmccastro/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/scripts/acados_c_generated_code/WheelTorqueAllocation/acadosOcp/acados_ocp.json"
        ###

        self.solver = AcadosOcpSolver(ocp, json_file = json_file_path)

    def _constraints(self, initialState, pathReference, velocityReference, forcesReference):

        """
            Return constraints to input at NMPC model on each iteration

            :initialState       [w_l, w_r]
            :velocityReference  [vx, wz]_ref
            :pathReference      [x, y, z, roll, pitch, yaw]_ref
        """

        #   Set initial state
        #self.solver.set(0, 'lbx', np.stack( initialState ) )
        #self.solver.set(0, 'ubx', np.stack( initialState ) )

        #print(len(initialState), len(pathReference), len(velocityReference), len(forcesReference))
        #print("\n")

        for i in range(self.N + 1):
            reference = np.stack( [ pathReference[i * (self.NbPosition + self.NbOrientation) + 3],  pathReference[i * (self.NbPosition + self.NbOrientation) + 4] ]\
                                    + velocityReference[ i * 6 : (i + 1) * 6 ] + forcesReference[i * 2 : (i + 1) * 2] )
            
            #print(f"Reference {i}: ", reference)
            
            self.solver.set(i, 'p', reference )
        
    def _setInitialGuess(self, numIter, initialState, pathReference, velocityReference, forcesMomentsReference):
        
        """
            :numIter int type, number of iterations
            :velocity                       [vx, vy, vz, wx, wy, wz]
            :velocityReference              [vx, wz]_ref
            :pathReference                  [x, y, z, roll, pitch, yaw]_ref
        """

        print("(wl, wr): ", initialState)
        print("(roll, pitch): ", pathReference[3], pathReference[4])
        print("(fx, mz): ", forcesMomentsReference[0], forcesMomentsReference[1])

        # do some initial iterations to start with a good initial guess
        for _ in range(numIter):

            self._constraints(initialState, pathReference, velocityReference, forcesMomentsReference)

            u0 = self.solver.solve_for_x0(x0_bar = np.stack( initialState ))

            #self.solver.dump_last_qp_to_json("/home/fmccastro/Desktop/last_qp.json")

            print("Total time: ", self.solver.get_stats("time_tot"))
    
    def _solve(self, initialState, pathReference, velocityReference, forcesReference):

        """
            :initialState                   [w_l, w_r]_0
            :pathReference                  [x, y, z, roll, pitch, yaw]_ref
            :velocityReference              [vx, wz]_ref
            :forcesReference                [fx, mz]_ref
        """

        if( self.nlp_solver_type == 'SQP' ):
            #   Retrieve optimization problem constraints, initial guess and parameters
            self._constraints(initialState, pathReference, velocityReference, forcesReference)

            status = self.solver.solve()

            #print("Total time: ", self.solver.get_stats("time_tot"))

            if status != 0:
                print("acados returned status {} in closed loop iteration.".format(status))

        elif( self.nlp_solver_type == 'SQP_RTI' ):
            self.solver.options_set('rti_phase', 1)
            status1 = self.solver.solve()

            t_preparation = self.solver.get_stats("time_tot")

            #   Retrieve optimization problem constraints, initial guess and parameters
            self._constraints(initialState, pathReference, velocityReference, forcesReference)

            self.solver.options_set('rti_phase', 2)
            status2 = self.solver.solve()

            t_feedback = self.solver.get_stats("time_tot")

            print("Total time: ", t_preparation + t_feedback)

            if( status1 != 0 or status2 != 0 ):
                print("acados returned status {} in preparation phase.".format(status1))
                print("acados returned status {} in feedback phase.".format(status2))
        
        #self.solver.print_statistics()

        solutionX, normalForces, wheelTorques = self._data()

        return solutionX, normalForces, wheelTorques

    def _data(self):
        
        """ 
            Optimization solution data treatment

            solution: dictionary returned by casadi solver
        """ 
        
        solutionX = []
        solutionU = []

        for i in range(self.N + 1):
            #   solutionX -> get optimized states solution
            opt_x = self.solver.get(i, 'x')

            if( i == 1 ):
                pass
                #next_w_l = opt_x[0]
                #next_w_r = opt_x[2]

            #solutionX += list(opt_x) 

            if(i < self.N):
                #   solutionU -> get optimized controls solution
                opt_u = self.solver.get(i, 'u')

                if( i == 0 ):
                    #next_fy_bl = opt_u[0]
                    #next_fy_fl = opt_u[1]
                    #next_fy_br = opt_u[2]
                    #next_fy_fr = opt_u[3]

                    next_fz_bl = opt_u[0]
                    next_fz_fl = opt_u[1]
                    next_fz_br = opt_u[2]
                    next_fz_fr = opt_u[3]

                    next_torque_l = opt_u[4]
                    next_torque_r = opt_u[5]
                
                solutionU += list(opt_u)
        
        #wheelRates = [next_w_l, next_w_r]
        #lateralForces = [next_fy_bl, next_fy_fl, next_fy_br, next_fy_fr]
        normalForces = [next_fz_bl, next_fz_fl, next_fz_br, next_fz_fr]
        wheelTorques = [next_torque_l, next_torque_r]

        return solutionU, normalForces, wheelTorques

class WheelTorqueAllocation_qp(ModelParameters, Common):

    #   Constructor
    def __init__(self, com2wheel):
        
        """
            :com2wheel dictionary
        """
        
        super().__init__()
        
        #   Vector from robot center of mass to each wheel center (w.r.t to body frame)
        com2bl = ca.vertcat( com2wheel["com2bl"][0], com2wheel["com2bl"][1], com2wheel["com2bl"][2] )
        com2fl = ca.vertcat( com2wheel["com2fl"][0], com2wheel["com2fl"][1], com2wheel["com2fl"][2] )
        com2br = ca.vertcat( com2wheel["com2br"][0], com2wheel["com2br"][1], com2wheel["com2br"][2] )
        com2fr = ca.vertcat( com2wheel["com2fr"][0], com2wheel["com2fr"][1], com2wheel["com2fr"][2] )

        com2bl_contact = ca.vertcat( com2wheel["com2bl"][0], com2wheel["com2bl"][1], com2wheel["com2bl"][2] - self.wheelRadius )
        com2fl_contact = ca.vertcat( com2wheel["com2fl"][0], com2wheel["com2fl"][1], com2wheel["com2fl"][2] - self.wheelRadius )
        com2br_contact = ca.vertcat( com2wheel["com2br"][0], com2wheel["com2br"][1], com2wheel["com2br"][2] - self.wheelRadius )
        com2fr_contact = ca.vertcat( com2wheel["com2fr"][0], com2wheel["com2fr"][1], com2wheel["com2fr"][2] - self.wheelRadius )

        S_bl = ca.skew( com2bl_contact )
        S_fl = ca.skew( com2fl_contact ) 
        S_br = ca.skew( com2br_contact )
        S_fr = ca.skew( com2fr_contact )

        #   Wheel velocity w.r.t body frame
        v_bl = self.lin_vel + ca.cross( self.ang_vel, com2bl )
        v_fl = self.lin_vel + ca.cross( self.ang_vel, com2fl )
        v_br = self.lin_vel + ca.cross( self.ang_vel, com2br )
        v_fr = self.lin_vel + ca.cross( self.ang_vel, com2fr )
        
        slip_bl = self.wheelRadius * self.w_bl - v_bl[0, 0]
        slip_fl = self.wheelRadius * self.w_fl - v_fl[0, 0]
        slip_br = self.wheelRadius * self.w_br - v_br[0, 0]
        slip_fr = self.wheelRadius * self.w_fr - v_fr[0, 0]

        t_bl = self.torque_l / self.wheelRadius
        t_fl = self.torque_l / self.wheelRadius
        t_br = self.torque_r / self.wheelRadius
        t_fr = self.torque_r / self.wheelRadius

        #   Force transferred to the ground by wheel
        f_bl = ca.vertcat( t_bl, 0.0, self.fz_bl)
        f_fl = ca.vertcat( t_fl, 0.0, self.fz_fl)
        f_br = ca.vertcat( t_br, 0.0, self.fz_br)
        f_fr = ca.vertcat( t_fr, 0.0, self.fz_fr)

        gravity = self.TransRotationMatrix.T @ ca.vertcat(0, 0, self.gz)

        sumForces = gravity * self.robotMass + f_bl + f_fl + f_br + f_fr - ca.vertcat(self.fx, 0.0, 0.0)

        m_bl = S_bl @ f_bl
        m_fl = S_fl @ f_fl
        m_br = S_br @ f_br
        m_fr = S_fr @ f_fr

        sumMoments = m_bl + m_fl + m_br + m_fr - ca.vertcat(0.0, 0.0, self.mz)

        g = ca.vertcat( ca.power(t_bl, 2) - ca.power(self.niu_c * self.fz_bl * self.niu_c * ca.tanh( 100 * slip_bl ), 2),\
                        ca.power(t_fl, 2) - ca.power(self.niu_c * self.fz_fl * self.niu_c * ca.tanh( 100 * slip_fl ), 2),\
                        ca.power(t_br, 2) - ca.power(self.niu_c * self.fz_br * self.niu_c * ca.tanh( 100 * slip_br ), 2),\
                        ca.power(t_fr, 2) - ca.power(self.niu_c * self.fz_fr * self.niu_c * ca.tanh( 100 * slip_fr ), 2) )

        y = ca.vertcat( sumForces, sumMoments[2, 0], self.torque_l, self.torque_r ) 
        Q = scipy.linalg.block_diag(self.Q_fn, self.Q_torque)

        cost = y.T @ Q @ y

        nlp = {'x': ca.vertcat(self.fz_wheels, self.torque_l, self.torque_r), 'f': cost, 'g': g, 'p': ca.vertcat(self.roll, self.pitch, self.lin_vel, self.ang_vel, self.fx, self.mz, self.w) }

        self.solver = ca.nlpsol('solver', self.optSolver, nlp, self.optOptions)

    def _callSolver(self, state, parameters):

        a = time.time()

        res = self.solver(x0 = state, lbx = [0.0, 0.0, 0.0, 0.0, -ca.inf, -ca.inf], ubx = [ca.inf, ca.inf, ca.inf, ca.inf, ca.inf, ca.inf],\
                                      lbg = [-ca.inf, -ca.inf, -ca.inf, -ca.inf], ubg = [0.0, 0.0, 0.0, 0.0], p = parameters)

        print(time.time() - a)

        x = res['x']

        return x