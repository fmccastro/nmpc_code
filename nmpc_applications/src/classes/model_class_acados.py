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

        #   Linear acceleration
        self.a_ref = ca.SX.sym('a_ref', 3)

        #   Angular acceleration
        self.b_ref = ca.SX.sym('b_ref', 3)

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
        self.fx_bl = ca.SX.sym('fx_bl')
        self.fx_fl = ca.SX.sym('fx_fl')
        self.fx_br = ca.SX.sym('fx_br')
        self.fx_fr = ca.SX.sym('fx_fr')

        self.fx_wheels = ca.vertcat( self.fx_bl, self.fx_fl, self.fx_br, self.fx_fr)

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

        #   Wheel forces (simplified dynamics)
        self.fx_l = ca.SX.sym('fx_l')
        self.fx_r = ca.SX.sym('fx_r')

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
        f_expl = ca.vertcat( self.TransRotationMatrix @ ca.vertcat(self.vx, 0, 0),\
                             self.RotRotationMatrix @ ca.vertcat(0, 0, self.wz) )
        
        #   Implicit model
        f_impl = ca.vertcat( self.position_dot - self.TransRotationMatrix @ ca.vertcat(self.vx, 0, 0),\
                             self.orientation_dot - self.RotRotationMatrix @ ca.vertcat(0, 0, self.wz) )

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
        
        error_roll = ca.power( ca.cos(self.roll) - ca.cos(self.roll_ref), 2 ) + ca.power( ca.sin(self.roll) - ca.sin(self.roll_ref), 2 )
        error_pitch = ca.power( ca.cos(self.pitch) - ca.cos(self.pitch_ref), 2 ) + ca.power( ca.sin(self.pitch) - ca.sin(self.pitch_ref), 2 )
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

        model.cost_expr_ext_cost_0 = y_0.T @ scipy.linalg.block_diag(self.Q_p_kin[0, 0], self.Q_p_kin[1, 1], self.Q_o_kin[2, 2], self.Q_vx_kin, self.Q_wz_kin) @ y_0
        model.cost_expr_ext_cost = y.T @ scipy.linalg.block_diag(self.Q_p_kin[0, 0], self.Q_p_kin[1, 1], self.Q_o_kin[2, 2], self.Q_vx_kin, self.Q_wz_kin) @ y
        model.cost_expr_ext_cost_e = y_e.T @ scipy.linalg.block_diag(self.Q_p_kin_t[0, 0], self.Q_p_kin_t[1, 1], self.Q_o_kin_t[2, 2]) @ y_e
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

        constraints.lbu = np.stack( [self.vx_lb, self.wz_lb] )
        constraints.ubu = np.stack( [self.vx_ub, self.wz_ub] )

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
            solver_options.globalization = 'FIXED_STEP'
            solver_options.nlp_solver_max_iter = 150

        solver_options.qp_solver_warm_start = 1
        solver_options.qp_solver_cond_N = int(self.N / 4)
        solver_options.print_level = 0
        solver_options.regularize_method = 'CONVEXIFY'
        solver_options.sim_method_num_stages = 4
        solver_options.sim_method_num_steps = 3
        solver_options.sim_method_newton_iter = 3
        solver_options.qp_solver_iter_max = 100
        solver_options.num_threads_in_batch_solve = 4
        solver_options.globalization_line_search_use_sufficient_descent = 0
        solver_options.tol = 1e-2
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
        self.solver.set(0, 'lbx', np.stack( pose ) )
        self.solver.set(0, 'ubx', np.stack( pose ) )

        for i in range(self.N + 1):
            if( i == 0 ):
                self.solver.set(i, 'p', np.stack( [ pose[0], pose[1], pose[5] ] ) )

            else:
                self.solver.set(i, 'p', np.stack( reference[i * self.np : (i + 1) * self.np] ) )

    def _setInitialGuess(self, numIter, pose, reference):
        
        """ 
            :numIter int type, number of iterations
            :pose geometry_msgs/Pose.msg type, mobile robot pose
        """

        # do some initial iterations to start with a good initial guess
        for _ in range(numIter):
            print("Initial iterations: ", _)

            self._constraints(pose, reference)
            
            self.solver.solve_for_x0(x0_bar = np.stack(pose), fail_on_nonzero_status=False)

            print("Cost: ", self.solver.get_cost())
            print("Total time: ", self.solver.get_stats("time_tot"))

    def _solve_sqp(self, pose, reference):

        #   Retrieve optimization problem constraints, initial guess and parameters
        self._constraints(pose, reference)

        status = self.solver.solve()

        #print("Total time: ", self.solver.get_stats("time_tot"))

        if status != 0:
            print("acados returned status {} in closed loop iteration.".format(status))

        opt_u = self.solver.get(0, 'u')
        next_vx = opt_u[0]
        next_wz = opt_u[1]

        return next_vx, next_wz
    
    def _preparation_sqp_rti(self):
        self.solver.options_set('rti_phase', 1)
        status1 = self.solver.solve()

        if( status1 != 0 ):
            print("acados returned status {} in preparation phase.".format(status1))
    
    def _feedback_sqp_rti(self, pose, reference):
        self._constraints(pose, reference)
        self.solver.options_set('rti_phase', 2)
        status2 = self.solver.solve()

        if( status2 != 0 ):
            print("acados returned status {} in feedback phase.".format(status2))
        
        opt_u = self.solver.get(0, 'u')
        next_vx = opt_u[0]
        next_wz = opt_u[1]
        
        return next_vx, next_wz
        
    def _data(self):

        """ 
            Optimization solution data treatment

            solution: dictionary returned by casadi solver
        """ 

        solutionX = []
        solutionU = []

        #print("Cost kinematics: ", self.solver.get_cost())

        for i in range(self.N + 1):

            #   solutionX -> get optimized states solution
            opt_x = self.solver.get(i, 'x')

            solutionX += list(opt_x)
            
            if(i < self.N):
                #   solutionU -> get optimized controls solution
                opt_u = self.solver.get(i, 'u')

                solutionU += list(opt_u)
        
        #   Retrieve cost and optimization time
        cost = self.solver.get_cost()
        optTime = self.solver.get_stats('time_tot')

        return solutionX, solutionU, cost, optTime

    def _simulate(self, state, controls):
        return self.integrator.simulate(x = state, u = controls)

#   Simplified dynamics
class SimplifiedDynamics(ModelParameters, Common):

    def __init__(self, com2wheel):

        """
            :com2wheel dictionary
        """

        super().__init__()

        #   State
        state = ca.vertcat(self.position, self.orientation, self.vx, self.vy, self.wz)

        #   State derivative
        state_dot = ca.vertcat(self.position_dot, self.orientation_dot, self.vx_dot, self.vy_dot, self.wz_dot)

        #   Parameters
        parameters = ca.vertcat(self.x_ref, self.y_ref, self.yaw_ref)

        #   Controls
        controls = ca.vertcat(self.fx_l, self.fx_r, self.fy_wheels)

        com2bl_contact = ca.vertcat( com2wheel["com2bl"][0], com2wheel["com2bl"][1], com2wheel["com2bl"][2] - self.wheelRadius )
        com2fl_contact = ca.vertcat( com2wheel["com2fl"][0], com2wheel["com2fl"][1], com2wheel["com2fl"][2] - self.wheelRadius )
        com2br_contact = ca.vertcat( com2wheel["com2br"][0], com2wheel["com2br"][1], com2wheel["com2br"][2] - self.wheelRadius )
        com2fr_contact = ca.vertcat( com2wheel["com2fr"][0], com2wheel["com2fr"][1], com2wheel["com2fr"][2] - self.wheelRadius )

        #   Gravity
        gravity = self.TransRotationMatrix.T @ ca.vertcat(0, 0, self.gz)
        robotWeight = gravity * self.robotMass

        fz_bl = -robotWeight[2] / 4
        fz_fl = -robotWeight[2] / 4
        fz_br = -robotWeight[2] / 4
        fz_fr = -robotWeight[2] / 4

        position_dot = self.TransRotationMatrix @ ca.vertcat(self.vx, self.vy, 0)
        euler_dot = self.RotRotationMatrix @ ca.vertcat(0, 0, self.wz)

        fx = 2 * self.fx_l + 2 * self.fx_r + robotWeight[0]
        fy = self.fy_bl + self.fy_fl + self.fy_br + self.fy_fr + robotWeight[1]
        mz = -self.fx_l * com2bl_contact[1] - self.fx_l * com2fl_contact[1] - self.fx_r * com2br_contact[1] - self.fx_r * com2fr_contact[1]\
             + self.fy_bl * com2bl_contact[0] + self.fy_fl * com2fl_contact[0] + self.fy_br * com2br_contact[0] + self.fy_fr * com2fr_contact[0]

        vx_dot = self.vy * self.wz + fx / self.robotMass
        vy_dot = -self.wz * self.vx + fy / self.robotMass
        wz_dot = mz / self.izz

        #   Explicit model
        f_expl = ca.vertcat( position_dot,\
                             euler_dot,\
                             vx_dot,\
                             vy_dot,\
                             wz_dot )
                                  
        #   Implicit model
        f_impl = ca.vertcat( self.position_dot - position_dot,\
                             self.orientation_dot - euler_dot,\
                             self.vx_dot - vx_dot,\
                             self.vy_dot - vy_dot,\
                             self.wz_dot - wz_dot )
        
        error_yaw = ca.power( ca.cos(self.yaw) - ca.cos(self.yaw_ref), 2 ) + ca.power( ca.sin(self.yaw) - ca.sin(self.yaw_ref), 2 )

        #   Call model instance
        model = AcadosModel()

        model.f_impl_expr = f_impl
        model.f_expl_expr = f_expl
        
        model.x = state
        model.xdot = state_dot
        model.u = controls
        model.p = parameters

        model.con_h_expr_0 = ca.vertcat( ca.power(self.fx_l, 2) - ca.power(self.niu * fz_bl, 2),\
                                         ca.power(self.fx_l, 2) - ca.power(self.niu * fz_fl, 2),\
                                         ca.power(self.fx_r, 2) - ca.power(self.niu * fz_br, 2),\
                                         ca.power(self.fx_r, 2) - ca.power(self.niu * fz_fr, 2),\
                                         ca.power(self.fy_bl, 2) - ca.power(self.niu * fz_bl, 2),\
                                         ca.power(self.fy_fl, 2) - ca.power(self.niu * fz_fl, 2),\
                                         ca.power(self.fy_br, 2) - ca.power(self.niu * fz_br, 2),\
                                         ca.power(self.fy_fr, 2) - ca.power(self.niu * fz_fr, 2) )

        model.con_h_expr = ca.vertcat( ca.power(self.fx_l, 2) - ca.power(self.niu * fz_bl, 2),\
                                       ca.power(self.fx_l, 2) - ca.power(self.niu * fz_fl, 2),\
                                       ca.power(self.fx_r, 2) - ca.power(self.niu * fz_br, 2),\
                                       ca.power(self.fx_r, 2) - ca.power(self.niu * fz_fr, 2),\
                                       ca.power(self.fy_bl, 2) - ca.power(self.niu * fz_bl, 2),\
                                       ca.power(self.fy_fl, 2) - ca.power(self.niu * fz_fl, 2),\
                                       ca.power(self.fy_br, 2) - ca.power(self.niu * fz_br, 2),\
                                       ca.power(self.fy_fr, 2) - ca.power(self.niu * fz_fr, 2) )
        
        model.name = "simple_dynamics"

        model.x_labels = ['x [m]', 'y [m]', 'z [m]', 'roll [rad]', 'pitch [rad]', 'yaw [rad]', '$u$ [m/s]', '$v$ [m/s]', '$w$ [m/s]', '$p$ [rad/s]', '$q$ [rad/s]', '$r$ [rad/s]']
        model.u_labels = [r'f_x [N]', r'f_y [N]', r'f_z [N]', r'm_z [Nm]']
        model.t_label = '$t$ [s]'

        y_0 = ca.vertcat(self.x - self.x_ref,\
                         self.y - self.y_ref,\
                         error_yaw,\
                         self.vx,\
                         self.vy,\
                         self.wz,\
                         controls )
        
        y = ca.vertcat(self.x - self.x_ref,\
                       self.y - self.y_ref,\
                       error_yaw,\
                       self.vx,\
                       self.vy,\
                       self.wz,\
                       controls )

        y_e = ca.vertcat(self.x - self.x_ref,\
                         self.y - self.y_ref,\
                         error_yaw,\
                         self.vx,\
                         self.vy,\
                         self.wz )

        model.cost_expr_ext_cost_0 = y_0.T @ scipy.linalg.block_diag(self.Q_p_simple_dyn[0, 0], self.Q_p_simple_dyn[1, 1], self.Q_o_simple_dyn[2, 2], self.Q_vx_simple_dyn, self.Q_vy_simple_dyn, self.Q_wz_simple_dyn, self.Q_f_simple_dyn ) @ y_0
        model.cost_expr_ext_cost = y.T @ scipy.linalg.block_diag(self.Q_p_simple_dyn[0, 0], self.Q_p_simple_dyn[1, 1], self.Q_o_simple_dyn[2, 2], self.Q_vx_simple_dyn, self.Q_vy_simple_dyn, self.Q_wz_simple_dyn, self.Q_f_simple_dyn ) @ y
        model.cost_expr_ext_cost_e = y_e.T @ scipy.linalg.block_diag(self.Q_p_simple_dyn_t[0, 0], self.Q_p_simple_dyn_t[1, 1], self.Q_o_simple_dyn_t[2, 2], self.Q_vx_simple_dyn_t, self.Q_vy_simple_dyn_t, self.Q_wz_simple_dyn_t ) @ y_e
        ###

        #   Call dims instance
        dims = AcadosOcpDims()

        dims.N = self.N
        dims.nu = model.u.rows()
        dims.nx = model.x.rows()
        dims.np = model.p.rows()
        dims.nh_0 = 8
        dims.nh = 8
        #dims.nh_e = 3
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
        constraints.lbx = np.stack( [-100.0, -100.0, -100.0, -math.pi/4, -math.pi/4, -math.pi, -100.0, -100.0, -100.0] )
        constraints.lbx_e = np.stack( [-100.0, -100.0, -100.0, -math.pi/4, -math.pi/4, -math.pi, -100.0, -100.0, -100.0] )

        #constraints.ubx_0 = np.stack( [ self.p_ub[0], self.p_ub[1], self.qsi_ub[2], self.v_ub[0], self.v_ub[1], self.w_ub[2] ] )
        constraints.ubx = np.stack( [100.0, 100.0, 100.0, math.pi/4, math.pi/4, math.pi, 100.0, 100.0, 100.0] )
        constraints.ubx_e = np.stack( [100.0, 100.0, 100.0, math.pi/4, math.pi/4, math.pi, 100.0, 100.0, 100.0] )
        
        constraints.lbu = np.array( [-100.0, -100.0, -100.0, -100.0, -100.0, -100.0] )
        constraints.ubu = np.array( [1e2, 1e2, 1e2, 1e2, 1e2, 1e2] )
        
        constraints.lh_0 = np.array( [-1e4] * 8 )
        constraints.lh = np.array( [-1e4] * 8 )

        constraints.uh_0 = np.array( [0.0] * 8 )
        constraints.uh = np.array( [0.0] * 8 )

        #constraints.idxbx_0 = np.array( [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15] )
        constraints.idxbx = np.array( [0, 1, 2, 3, 4, 5, 6, 7, 8] )
        constraints.idxbx_e = np.array( [0, 1, 2, 3, 4, 5, 6, 7, 8] )
        #constraints.idxbxe_0 = np.array( [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15] )
        
        constraints.idxbu = np.array( [0, 1, 2, 3, 4, 5] )

        constraints.x0 = np.stack( [0, 0, 0, 0, 0, 0, 0, 0, 0] ) 
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
        solver_options.sim_method_newton_iter = 3
        solver_options.qp_solver_iter_max = 25
        solver_options.num_threads_in_batch_solve = 4
        solver_options.globalization_line_search_use_sufficient_descent = 1
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
        ocp.code_export_directory = "/media/fmccastro/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/scripts/acados_c_generated_code/SimplifiedDynamics/acadosOcp"
        ocp.acados_lib_path = "/media/fmccastro/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/acados/lib"
        ocp.acados_include_path = "/media/fmccastro/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/acados/include"

        json_file_path = "/media/fmccastro/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/scripts/acados_c_generated_code/SimplifiedDynamics/acadosOcp/acados_ocp.json"
        ###

        self.solver = AcadosOcpSolver(ocp, json_file = json_file_path)
        self.integrator = AcadosSimSolver(ocp, json_file = json_file_path)
    
    def _constraints(self, pose, velocity, pathReference, flag = False, index = 0 ):

        """
            Return constraints to input at NMPC model on each iteration

            :initialState       [vx, vy, vz, wx, wy, wz]
            :velocityReference  [vx, wz]_ref
            :pathReference      [x, y, z, roll, pitch, yaw]_ref
        """

        x0 = pose[0]
        y0 = pose[1]
        yaw0 = pose[5]

        #   Set initial state
        self.solver.set(0, 'lbx', np.append(pose, velocity) )
        self.solver.set(0, 'ubx', np.append(pose, velocity) )

        for i in range(self.N + 1):
            if( i == 0 ):
                self.solver.set(i, 'p', np.stack( [x0, y0, yaw0] ))

            else:
                self.solver.set(i, 'p', np.stack( pathReference[i * 3 : (i + 1) * 3] ))
    
    def _setInitialGuess(self, numIter, pose, velocity, pathReference):
        
        """
            :numIter int type, number of iterations
            :velocity                       [vx, vy, vz, wx, wy, wz]
            :velocityReference              [vx, wz]_ref
            :pathReference                  [x, y, z, roll, pitch, yaw]_ref
        """

        # do some initial iterations to start with a good initial guess
        for _ in range(numIter):
            self._constraints(pose, velocity, pathReference, flag = True, index = _)

            u0 = self.solver.solve_for_x0(x0_bar = np.append(pose, velocity) )

            #self.solver.dump_last_qp_to_json("/home/fmccastro/Desktop/last_qp.json")

            print("Cost: ", self.solver.get_cost())
            print("Total time: ", self.solver.get_stats("time_tot"))
    
    def _solve_sqp(self, pose, velocity, pathReference):

        """
            :pose                           []
            :velocity                       [vx, vy, vz, wx, wy, wz]
            :velocityReference              [vx, wz]_ref
            :pathReference                  [x, y, z, roll, pitch, yaw]_ref
        """

        #   Retrieve optimization problem constraints, initial guess and parameters
        self._constraints(pose, velocity, pathReference)

        status = self.solver.solve()

        #print("Total time: ", self.solver.get_stats("time_tot"))

        if status != 0:
            print("acados returned status {} in closed loop iteration.".format(status))
        
        #self.solver.print_statistics()

        opt_u = self.solver.get(0, 'u')

        return opt_u[0:6], status

    def _preparation_sqp_rti(self):
        self.solver.options_set('rti_phase', 1)
        status1 = self.solver.solve()

        t_preparation = self.solver.get_stats("time_tot")

        if( status1 != 0 ):
            print("acados returned status {} in preparation phase.".format(status1))
    
    def _feedback_sqp_rti(self, pose, velocity, pathReference):

        #   Retrieve optimization problem constraints, initial guess and parameters
        self._constraints(pose, velocity, pathReference)

        self.solver.options_set('rti_phase', 2)
        status2 = self.solver.solve()

        if( status2 != 0 ):
            print("acados returned status {} in feedback phase.".format(status2))
        
        opt_u = self.solver.get(0, 'u')

        return opt_u[0:6], status2

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
                
                solutionU += list(opt_u)
        
        #   Retrieve cost and optimization time
        cost = self.solver.get_cost()
        optTime = self.solver.get_stats('time_tot')

        return solutionX, solutionU, cost, optTime
    
    def _simulate(self, state, controls):
        return self.integrator.simulate(x = state, u = controls)

#   Full dynamics 
class Dynamics(ModelParameters, Common):

    """
        Contact angles between wheel and terrain are neglected.
        Thus, traction and friction is assumed to hold a direction longitudinal to the body frame longitudinal direction.
    """

    #   Constructor
    def __init__(self, com2wheel):
        
        """
            :com2wheel dictionary
        """
        
        super().__init__()

        #   State
        state = ca.vertcat(self.position, self.orientation, self.lin_vel, self.ang_vel)
        
        #   Derivative state
        state_dot = ca.vertcat(self.position_dot, self.orientation_dot, self.lin_vel_dot, self.ang_vel_dot)

        #   Parameters
        parameters = ca.vertcat(self.x_ref, self.y_ref, self.yaw_ref)

        #   Inertia tensor of vehicle
        inertia = ca.horzcat( ca.vertcat(self.ixx, self.ixy, self.ixz),\
                              ca.vertcat(self.ixy, self.iyy, self.iyz),\
                              ca.vertcat(self.ixz, self.iyz, self.izz) )

        #   Controls
        controls = ca.vertcat(self.fx_l, self.fx_r)

        #   Gravity
        gravity = self.TransRotationMatrix.T @ ca.vertcat(0, 0, self.gz)
        robotWeight = gravity * self.robotMass

        com2bl_contact = ca.vertcat( com2wheel["com2bl"][0], com2wheel["com2bl"][1], com2wheel["com2bl"][2] - self.wheelRadius )
        com2fl_contact = ca.vertcat( com2wheel["com2fl"][0], com2wheel["com2fl"][1], com2wheel["com2fl"][2] - self.wheelRadius )
        com2br_contact = ca.vertcat( com2wheel["com2br"][0], com2wheel["com2br"][1], com2wheel["com2br"][2] - self.wheelRadius )
        com2fr_contact = ca.vertcat( com2wheel["com2fr"][0], com2wheel["com2fr"][1], com2wheel["com2fr"][2] - self.wheelRadius )

        S_bl = ca.skew( com2bl_contact )
        S_fl = ca.skew( com2fl_contact ) 
        S_br = ca.skew( com2br_contact )
        S_fr = ca.skew( com2fr_contact )

        #   wheel velocities
        v_bl = self.lin_vel + ca.skew(self.ang_vel) @ com2bl_contact
        v_fl = self.lin_vel + ca.skew(self.ang_vel) @ com2fl_contact
        v_br = self.lin_vel + ca.skew(self.ang_vel) @ com2br_contact
        v_fr = self.lin_vel + ca.skew(self.ang_vel) @ com2fr_contact

        #   Lateral forces
        """slip_angle_bl = ca.atan2(v_bl[1], v_bl[0])
        slip_angle_fl = ca.atan2(v_fl[1], v_fl[0])
        slip_angle_br = ca.atan2(v_br[1], v_br[0])
        slip_angle_fr = ca.atan2(v_fr[1], v_fr[0])

        cbf = 92027

        fy_bl = 0.0 #-robotWeight[1] / 4 - cbf * slip_angle_bl
        fy_fl = 0.0 #-robotWeight[1] / 4 - cbf * slip_angle_fl
        fy_br = 0.0 #-robotWeight[1] / 4 - cbf * slip_angle_br
        fy_fr = 0.0 #-robotWeight[1] / 4 - cbf * slip_angle_fr"""

        fy_bl = -robotWeight[1] / 4
        fy_fl = -robotWeight[1] / 4
        fy_br = -robotWeight[1] / 4
        fy_fr = -robotWeight[1] / 4

        fz_bl = -robotWeight[2] / 4
        fz_fl = -robotWeight[2] / 4
        fz_br = -robotWeight[2] / 4
        fz_fr = -robotWeight[2] / 4

        sum_fx = 2 * self.fx_l + 2 * self.fx_r
        sum_fy = fy_bl + fy_fl + fy_br + fy_fr
        sum_fz = fz_bl + fz_fl + fz_br + fz_fr

        #   Vertical forces
        """l = 2 * abs(com2wheel["com2bl"][0])
        lw = 2 * abs(com2wheel["com2bl"][1])

        fz_fl = -robotWeight[2] / 4 - sum_fx * (abs(com2wheel["com2fl"][2]) + self.wheelRadius) / (2 * l)\
                                    - sum_fy * (abs(com2wheel["com2fl"][2]) + self.wheelRadius) / (2 * lw)
        
        fz_fr = -robotWeight[2] / 4 - sum_fx * (abs(com2wheel["com2fr"][2]) + self.wheelRadius) / (2 * l)\
                                    + sum_fy * (abs(com2wheel["com2fr"][2]) + self.wheelRadius) / (2 * lw)
        
        fz_bl = -robotWeight[2] / 4 + sum_fx * (abs(com2wheel["com2bl"][2]) + self.wheelRadius) / (2 * l)\
                                    - sum_fy * (abs(com2wheel["com2bl"][2]) + self.wheelRadius) / (2 * lw)
        
        fz_br = -robotWeight[2] / 4 + sum_fx * (abs(com2wheel["com2br"][2]) + self.wheelRadius) / (2 * l)\
                                    + sum_fy * (abs(com2wheel["com2br"][2]) + self.wheelRadius) / (2 * lw)"""

        sumForces = ca.vertcat(sum_fx, sum_fy, sum_fz) + robotWeight

        m_bl = S_bl @ ca.vertcat(self.fx_l, self.fy_bl, fz_bl)
        m_fl = S_fl @ ca.vertcat(self.fx_l, self.fy_fl, fz_fl)
        m_br = S_br @ ca.vertcat(self.fx_r, self.fy_br, fz_br)
        m_fr = S_fr @ ca.vertcat(self.fx_r, self.fy_fr, fz_fr)

        sumMoments = m_bl + m_fl + m_br + m_fr

        v_dot = -ca.cross( self.ang_vel, self.lin_vel ) + sumForces / self.robotMass
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

        error_roll = ca.power( ca.cos(self.roll) - ca.cos(self.roll_ref), 2 ) + ca.power( ca.sin(self.roll) - ca.sin(self.roll_ref), 2 )
        error_pitch = ca.power( ca.cos(self.pitch) - ca.cos(self.pitch_ref), 2 ) + ca.power( ca.sin(self.pitch) - ca.sin(self.pitch_ref), 2 )
        error_yaw = ca.power( ca.cos(self.yaw) - ca.cos(self.yaw_ref), 2 ) + ca.power( ca.sin(self.yaw) - ca.sin(self.yaw_ref), 2 )

        #   Call model instance
        model = AcadosModel()

        model.f_impl_expr = f_impl
        model.f_expl_expr = f_expl
        
        model.x = state
        model.xdot = state_dot
        model.u = controls
        model.p = parameters

        model.con_h_expr_0 = ca.vertcat( ca.power(self.fx_l, 2) - ca.power(self.niu * fz_bl, 2),\
                                         ca.power(self.fx_l, 2) - ca.power(self.niu * fz_fl, 2),\
                                         ca.power(self.fx_r, 2) - ca.power(self.niu * fz_br, 2),\
                                         ca.power(self.fx_r, 2) - ca.power(self.niu * fz_fr, 2) )
    
        model.con_h_expr = ca.vertcat( ca.power(self.fx_l, 2) - ca.power(self.niu * fz_bl, 2),\
                                       ca.power(self.fx_l, 2) - ca.power(self.niu * fz_fl, 2),\
                                       ca.power(self.fx_r, 2) - ca.power(self.niu * fz_br, 2),\
                                       ca.power(self.fx_r, 2) - ca.power(self.niu * fz_fr, 2) )
        
        model.name = "trajectory_generation"

        model.x_labels = ['x [m]', 'y [m]', 'z [m]', 'roll [rad]', 'pitch [rad]', 'yaw [rad]', '$u$ [m/s]', '$v$ [m/s]', '$w$ [m/s]', '$p$ [rad/s]', '$q$ [rad/s]', '$r$ [rad/s]']
        model.u_labels = [r'f_x [N]', r'f_y [N]', r'f_z [N]', r'm_z [Nm]']
        model.t_label = '$t$ [s]'

        y_0 = ca.vertcat(self.x - self.x_ref,\
                         self.y - self.y_ref,\
                         error_yaw,\
                         self.vx,\
                         self.vy,\
                         self.vz,\
                         self.wx,\
                         self.wy,\
                         self.wz,\
                         self.fx_l,\
                         self.fx_r,\
                         sumMoments[0],\
                         sumMoments[1] )
    
        y = ca.vertcat(self.x - self.x_ref,\
                       self.y - self.y_ref,\
                       error_yaw,\
                       self.vx,\
                       self.vy,\
                       self.vz,\
                       self.wx,\
                       self.wy,\
                       self.wz,\
                       self.fx_l,\
                       self.fx_r,\
                       sumMoments[0],\
                       sumMoments[1] )

        y_e = ca.vertcat(self.x - self.x_ref,\
                         self.y - self.y_ref,\
                         error_yaw,\
                         self.vx,\
                         self.vy,\
                         self.vz,\
                         self.wx,\
                         self.wy,\
                         self.wz )

        model.cost_expr_ext_cost_0 = y_0.T @ scipy.linalg.block_diag(self.Q_p_dyn[0, 0], self.Q_p_dyn[1, 1], self.Q_o_dyn[2, 2], self.Q_v_dyn, self.Q_w_dyn, self.Q_f_dyn, self.Q_m_dyn ) @ y_0
        model.cost_expr_ext_cost = y.T @ scipy.linalg.block_diag(self.Q_p_dyn[0, 0], self.Q_p_dyn[1, 1], self.Q_o_dyn[2, 2], self.Q_v_dyn, self.Q_w_dyn, self.Q_f_dyn, self.Q_m_dyn ) @ y
        model.cost_expr_ext_cost_e = y_e.T @ scipy.linalg.block_diag(self.Q_p_dyn_t[0, 0], self.Q_p_dyn_t[1, 1], self.Q_o_dyn_t[2, 2], self.Q_v_dyn_t, self.Q_w_dyn_t ) @ y_e
        ###

        #   Call dims instance
        dims = AcadosOcpDims()

        dims.N = self.N
        dims.nu = model.u.rows()
        dims.nx = model.x.rows()
        dims.np = model.p.rows()
        dims.nh_0 = 8
        dims.nh = 8
        #dims.nh_e = 3
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
        constraints.lbx = np.stack( [-100.0, -100.0, -100.0, -math.pi/4, -math.pi/4, -math.pi, -100.0, -100.0, -100.0, -100.0, -100.0, -100.0] )
        constraints.lbx_e = np.stack( [-100.0, -100.0, -100.0, -math.pi/4, -math.pi/4, -math.pi, -100.0, -100.0, -100.0, -100.0, -100.0, -100.0] )

        #constraints.ubx_0 = np.stack( [ self.p_ub[0], self.p_ub[1], self.qsi_ub[2], self.v_ub[0], self.v_ub[1], self.w_ub[2] ] )
        constraints.ubx = np.stack( [100.0, 100.0, 100.0, math.pi/4, math.pi/4, math.pi, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0] )
        constraints.ubx_e = np.stack( [100.0, 100.0, 100.0, math.pi/4, math.pi/4, math.pi, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0] )
        
        constraints.lbu = np.array( [-1000.0, -1000.0] )
        constraints.ubu = np.array( [1000.0, 1000.0] )
        
        constraints.lh_0 = np.array( [-1e3, -1e3, -1e3, -1e3] )
        constraints.lh = np.array( [-1e3, -1e3, -1e3, -1e3] )

        constraints.uh_0 = np.array( [0.0, 0.0, 0.0, 0.0] )
        constraints.uh = np.array( [0.0, 0.0, 0.0, 0.0] )

        #constraints.idxbx_0 = np.array( [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15] )
        constraints.idxbx = np.array( [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11] )
        constraints.idxbx_e = np.array( [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11] )
        #constraints.idxbxe_0 = np.array( [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15] )
        
        constraints.idxbu = np.array( [0, 1] )

        constraints.x0 = np.stack( [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] ) 
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
        solver_options.sim_method_newton_iter = 3
        solver_options.qp_solver_iter_max = 25
        solver_options.num_threads_in_batch_solve = 4
        solver_options.globalization_line_search_use_sufficient_descent = 1
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
        self.integrator = AcadosSimSolver(ocp, json_file = json_file_path)
    
    def _constraints(self, pose, velocity, pathReference, flag = False, index = 0 ):

        """
            Return constraints to input at NMPC model on each iteration

            :initialState       [vx, vy, vz, wx, wy, wz]
            :velocityReference  [vx, wz]_ref
            :pathReference      [x, y, z, roll, pitch, yaw]_ref
        """

        x0 = pose[0]
        y0 = pose[1]
        yaw0 = pose[5]

        #   Set initial state
        self.solver.set(0, 'lbx', np.append(pose, velocity) )
        self.solver.set(0, 'ubx', np.append(pose, velocity) )

        for i in range(self.N + 1):
            if( i == 0 ):
                self.solver.set(i, 'p', np.stack( [x0, y0, yaw0] ))

            else:
                self.solver.set(i, 'p', np.stack( pathReference[i * 3 : (i + 1) * 3] ))

                #   Fix pitch and roll
                #self.solver.set(i, 'lbx', np.stack( [-100.0, -100.0, -100.0, pose[3], pose[4], -math.pi, -100.0, -100.0, -100.0, -100.0, -100.0, -100.0] ) )
                #self.solver.set(i, 'ubx', np.stack( [100.0, 100.0, 100.0, pose[3], pose[4], math.pi, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0] ) )

    def _setInitialGuess(self, numIter, pose, velocity, pathReference):
        
        """
            :numIter int type, number of iterations
            :velocity                       [vx, vy, vz, wx, wy, wz]
            :velocityReference              [vx, wz]_ref
            :pathReference                  [x, y, z, roll, pitch, yaw]_ref
        """

        # do some initial iterations to start with a good initial guess
        for _ in range(numIter):
            self._constraints(pose, velocity, pathReference, flag = True, index = _)

            u0 = self.solver.solve_for_x0(x0_bar = np.append(pose, velocity) )

            #self.solver.dump_last_qp_to_json("/home/fmccastro/Desktop/last_qp.json")

            print("Cost: ", self.solver.get_cost())
            print("Total time: ", self.solver.get_stats("time_tot"))

    def _solve_sqp(self, pose, velocity, pathReference):

        """
            :pose                           []
            :velocity                       [vx, vy, vz, wx, wy, wz]
            :velocityReference              [vx, wz]_ref
            :pathReference                  [x, y, z, roll, pitch, yaw]_ref
        """

        #   Retrieve optimization problem constraints, initial guess and parameters
        self._constraints(pose, velocity, pathReference)

        status = self.solver.solve()

        #print("Total time: ", self.solver.get_stats("time_tot"))

        if status != 0:
            print("acados returned status {} in closed loop iteration.".format(status))
        
        #self.solver.print_statistics()

        opt_u = self.solver.get(0, 'u')

        return opt_u[0:2]

    def _preparation_sqp_rti(self):
        self.solver.options_set('rti_phase', 1)
        status1 = self.solver.solve()

        t_preparation = self.solver.get_stats("time_tot")

        if( status1 != 0 ):
            print("acados returned status {} in preparation phase.".format(status1))
    
    def _feedback_sqp_rti(self, pose, velocity, pathReference):

        #   Retrieve optimization problem constraints, initial guess and parameters
        self._constraints(pose, velocity, pathReference)

        self.solver.options_set('rti_phase', 2)
        status2 = self.solver.solve()

        if( status2 != 0 ):
            print("acados returned status {} in feedback phase.".format(status2))
        
        opt_u = self.solver.get(0, 'u')

        return opt_u[0:2], status2

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
                
                solutionU += list(opt_u)
        
        #   Retrieve cost and optimization time
        cost = self.solver.get_cost()
        optTime = self.solver.get_stats('time_tot')

        return solutionX, solutionU, cost, optTime
    
    def _simulate(self, state, controls):
        return self.integrator.simulate(x = state, u = controls)

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

        alpha_bl = ca.SX.sym('alpha_bl')
        alpha_fl = ca.SX.sym('alpha_fl')
        alpha_br = ca.SX.sym('alpha_br')
        alpha_fr = ca.SX.sym('alpha_fr')
        contact_bl = ca.SX.sym('contact_bl')
        contact_fl = ca.SX.sym('contact_fl')
        contact_br = ca.SX.sym('contact_br')
        contact_fr = ca.SX.sym('contact_fr')
        fz_ref = ca.SX.sym('fz_b')
        fx_ref_l = ca.SX.sym('fx_ref_l')
        fx_ref_r = ca.SX.sym('fx_ref_r')

        rotation_bl = ca.horzcat( ca.vertcat( ca.cos(alpha_bl), ca.sin(alpha_bl) ),\
                                  ca.vertcat( -ca.sin(alpha_bl), ca.cos(alpha_bl) ) )
    
        rotation_fl = ca.horzcat( ca.vertcat( ca.cos(alpha_fl), ca.sin(alpha_fl) ),\
                                  ca.vertcat( -ca.sin(alpha_fl), ca.cos(alpha_fl) ) )
        
        rotation_br = ca.horzcat( ca.vertcat( ca.cos(alpha_br), ca.sin(alpha_br) ),\
                                  ca.vertcat( -ca.sin(alpha_br), ca.cos(alpha_br) ) )
        
        rotation_fr = ca.horzcat( ca.vertcat( ca.cos(alpha_fr), ca.sin(alpha_fr) ),\
                                  ca.vertcat( -ca.sin(alpha_fr), ca.cos(alpha_fr) ) )
        
        #   Vector from robot center of mass to each wheel center (w.r.t to body frame)
        com2bl_contact = ca.vertcat( com2wheel["com2bl"][0], com2wheel["com2bl"][1], com2wheel["com2bl"][2] - self.wheelRadius )
        com2fl_contact = ca.vertcat( com2wheel["com2fl"][0], com2wheel["com2fl"][1], com2wheel["com2fl"][2] - self.wheelRadius )
        com2br_contact = ca.vertcat( com2wheel["com2br"][0], com2wheel["com2br"][1], com2wheel["com2br"][2] - self.wheelRadius )
        com2fr_contact = ca.vertcat( com2wheel["com2fr"][0], com2wheel["com2fr"][1], com2wheel["com2fr"][2] - self.wheelRadius )

        parameters = ca.vertcat( alpha_bl,\
                                 alpha_fl,\
                                 alpha_br,\
                                 alpha_fr,\
                                 contact_bl,\
                                 contact_fl,\
                                 contact_br,\
                                 contact_fr,\
                                 fz_ref,\
                                 fx_ref_l,\
                                 fx_ref_r )

        g = ca.vertcat( ca.power(self.fx_l, 2) - ca.power(self.niu * self.fz_bl, 2),\
                        ca.power(self.fx_l, 2) - ca.power(self.niu * self.fz_fl, 2),\
                        ca.power(self.fx_r, 2) - ca.power(self.niu * self.fz_br, 2),\
                        ca.power(self.fx_r, 2) - ca.power(self.niu * self.fz_fr, 2) )

        y_bl = ( rotation_bl @ ca.vertcat(self.fx_l, self.fz_bl) - ca.vertcat(fx_ref_l, fz_ref) ) * contact_bl
        y_fl = ( rotation_fl @ ca.vertcat(self.fx_l, self.fz_fl) - ca.vertcat(fx_ref_l, fz_ref) ) * contact_fl
        y_br = ( rotation_br @ ca.vertcat(self.fx_r, self.fz_br) - ca.vertcat(fx_ref_r, fz_ref) ) * contact_br
        y_fr = ( rotation_fr @ ca.vertcat(self.fx_r, self.fz_fr) - ca.vertcat(fx_ref_r, fz_ref) ) * contact_fr

        cost = y_bl.T @ y_bl + y_fl.T @ y_fl + y_br.T @ y_br + y_fr.T @ y_fr

        nlp = {'x': ca.vertcat(self.fx_l, self.fx_r, self.fz_wheels), 'f': cost, 'g': g, 'p': parameters}

        self.solver = ca.nlpsol('solver', self.optSolver, nlp, self.optOptions)

    def _callSolver(self, state, parameters):

        a = time.time()

        res = self.solver(x0 = state, lbx = [-ca.inf, -ca.inf, 0.0, 0.0, 0.0, 0.0], ubx = [ca.inf, ca.inf, ca.inf, ca.inf, ca.inf, ca.inf],\
                                      lbg = [-ca.inf, -ca.inf, -ca.inf, -ca.inf], ubg = [0.0, 0.0, 0.0, 0.0], p = parameters)

        print(time.time() - a)

        x = res['x']

        return x

class wheelRateIntegrator(ModelParameters, Common):

    #   Constructor
    def __init__(self, com2wheel):
        
        """
            :com2wheel dictionary
        """
        
        super().__init__()

        state = ca.vertcat(self.w_l, self.w_r)
        controls = ca.vertcat(self.fx_l, self.fx_r)

        com2bl_contact = ca.vertcat( com2wheel["com2bl"][0], com2wheel["com2bl"][1], com2wheel["com2bl"][2] - self.wheelRadius )
        com2fl_contact = ca.vertcat( com2wheel["com2fl"][0], com2wheel["com2fl"][1], com2wheel["com2fl"][2] - self.wheelRadius )
        com2br_contact = ca.vertcat( com2wheel["com2br"][0], com2wheel["com2br"][1], com2wheel["com2br"][2] - self.wheelRadius )
        com2fr_contact = ca.vertcat( com2wheel["com2fr"][0], com2wheel["com2fr"][1], com2wheel["com2fr"][2] - self.wheelRadius )

        #   Gravity
        gravity = self.TransRotationMatrix.T @ ca.vertcat(0, 0, self.gz)
        robotWeight = gravity * self.robotMass

        i11 = math.pow(self.wheelRadius, 2) * ( self.robotMass / 4 + self.izz / math.pow( 2 * com2bl_contact[1], 2) )
        i22 = i11
        i12 = math.pow(self.wheelRadius, 2) * (self.robotMass/ 4 - self.izz / math.pow( 2 * com2bl_contact[1], 2) )
        i21 = i12

        inertiaMatrix = ca.horzcat( ca.vertcat(i11, i21), ca.vertcat(i12, i22) )

        print("Inertia matrix: ", inertiaMatrix)

        sum_fx = 2 * self.fx_l + 2 * self.fx_r + robotWeight[0]
        sum_mz = -self.fx_l * com2bl_contact[1] - self.fx_l * com2fl_contact[1] - self.fx_r * com2br_contact[1] - self.fx_r * com2fr_contact[1]

        lw = 2 * com2bl_contact[1]

        model = AcadosModel()

        model.x = state
        model.u = controls
        model.f_expl_expr = ca.inv_minor(inertiaMatrix) @ (self.wheelRadius * ca.vertcat(self.fx_l, self.fx_r) - 0.7 * self.wheelRadius * ca.vertcat(self.fx_l, self.fx_r))
        model.name = "integrator_wheelrate"

        #   Call dims instance
        dims = AcadosSimDims()

        dims.nu = model.u.rows()
        dims.nx = model.x.rows()

        options = AcadosSimOptions()
        options.T = 0.1

        #   Call ocp instance
        ocp = AcadosSim()

        #   Set ocp
        ocp.model = model
        ocp.dims = dims
        ocp.solver_options = options

        #   Set folder path where generated c code is located
        ocp.code_export_directory = "/media/fmccastro/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/scripts/acados_c_generated_code/wheelRate_integrator/acadosSim"
        ocp.acados_lib_path = "/media/fmccastro/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/acados/lib"
        ocp.acados_include_path = "/media/fmccastro/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/acados/include"

        json_file_path = "/media/fmccastro/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/scripts/acados_c_generated_code/wheelRate_integrator/acadosSim/acados_sim.json"
        ###

        self.integrator = AcadosSimSolver(ocp, json_file = json_file_path)

    def _callIntegrator(self, wl, wr, fx_l, fx_r, time):

        self.integrator.set('T', time)

        return self.integrator.simulate(x = np.stack([wl, wr]), u = np.stack([fx_l, fx_r]) )