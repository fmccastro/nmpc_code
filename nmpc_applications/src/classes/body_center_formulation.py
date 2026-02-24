#!/usr/bin/python3.8

import sys

import scipy.linalg
sys.path.insert(0, "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/src")

from classes.all_imports import *
from classes.common_class import *

"""
    Build and define NMPC optimization problem with the frenet-serret frame
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

        #   Reference position with respect to body frame
        #   Body frame is aligned with vehicle longitudinal axis
        self.xb = ca.SX.sym('xb')
        self.yb = ca.SX.sym('yb')

        #   Angle difference
        self.yaw = ca.SX.sym('yaw')

        #   Position derivatives
        self.xb_dot = ca.SX.sym('t_dot')
        self.yb_dot = ca.SX.sym('n_dot')

        #   Fixed angles derivative (rpy)
        self.yaw_dot = ca.SX.sym('yaw_dot')

        #   Linear velocity (w.r. to body frame)
        self.vx = ca.SX.sym('vx')
        self.vy = ca.SX.sym('vy')

        #   Linear velocity derivative (w.r. to body frame)
        self.vx_dot = ca.SX.sym('vx_dot') 
        self.vy_dot = ca.SX.sym('vy_dot')

        #   Angular velocity (w.r. to body frame)
        self.wz = ca.SX.sym('wz')

        #   Angular velocity derivative (w.r. to body frame)
        self.wz_dot = ca.SX.sym('wz_dot')

        #   Progress
        self.progress = ca.SX.sym('progress')
        self.progress_dot = ca.SX.sym('progress_dot')
        
        #   Virtual speed
        self.virtual_speed = ca.SX.sym('virtual_speed')
        self.virtual_speed_dot = ca.SX.sym('virtual_speed_dot')

        #   Forces
        self.delta = ca.SX.sym('delta')
        self.fr = ca.SX.sym('fr')

        self.fr_dot = ca.SX.sym('fr_dot')
        self.delta_dot = ca.SX.sym('delta_dot')

        self.virtual_speed_rate = ca.SX.sym('virtual_speed_rate')
        self.fr_rate = ca.SX.sym('fr_rate')
        self.delta_rate = ca.SX.sym('delta_rate')

#   Dynamics bicycle model (planar + frenet serret formulation)
class DynamicsBicyclePlanar(ModelParameters, Common):
    def __init__(self, x_ref, y_ref):

        super().__init__()
        
        with open(self.body_center_dyn_bic_planar) as f:
            solver_param = json.load(f)
            specs = solver_param["solver_specs"]
            con_state = solver_param["con_state"]
            con_controls = solver_param["con_controls"]
            weights = solver_param["weights"]
        
        with open(self.vehicle_specs) as f:
            vehicle_param = json.load(f)
        
        #   State
        state = ca.vertcat(self.xb, self.yb, self.yaw, self.vx, self.vy, self.wz, self.progress, self.virtual_speed, self.fr, self.delta)
        
        #   State derivative
        state_dot = ca.vertcat(self.xb_dot, self.yb_dot, self.yaw_dot, self.vx_dot, self.vy_dot, self.wz_dot, self.progress_dot, self.virtual_speed_dot, self.fr_dot, self.delta_dot)
        
        #   Controls
        controls = ca.vertcat(self.virtual_speed_rate, self.fr_rate, self.delta_rate)

        #   Wheel cornering forces
        alpha_r = ca.atan( ( self.wz * vehicle_param["lr"] - self.vy ) * self.vx / ( self.vx**2 + 1e-3 ) )
        alpha_f = -ca.atan( ( self.wz * vehicle_param["lf"] + self.vy ) * self.vx / ( self.vx**2 + 1e-3 ) ) + self.delta

        fs_r = vehicle_param["dr"] * ca.sin( vehicle_param["cr"] * ca.atan(vehicle_param["br"] * alpha_r) )
        fs_f = vehicle_param["df"] * ca.sin( vehicle_param["cf"] * ca.atan(vehicle_param["bf"] * alpha_f) )

        #   Wheel forces on body frame
        fx_f = -ca.sin(self.delta) * fs_f
        fy_f = ca.cos(self.delta) * fs_f

        fx_r = self.fr
        fy_r = fs_r

        #   Sum of forces and moments
        fx = fx_f + fx_r
        fy = fy_f + fy_r
        mz = -fy_r * vehicle_param["lr"] + fy_f * vehicle_param["lf"]

        sum_fx = fx
        sum_fy = fy
        sum_mz = mz

        vx_dot = self.wz * self.vy + sum_fx / vehicle_param["m"]
        vy_dot = -self.wz * self.vx + sum_fy / vehicle_param["m"]
        wz_dot = sum_mz / vehicle_param["izz"]

        #   Lag error
        x_p = x_ref(self.progress)
        y_p = y_ref(self.progress)

        d_x_p = ca.jacobian(x_p, self.progress)
        d_y_p = ca.jacobian(y_p, self.progress)

        dd_x_p, g_x_p = ca.hessian(x_p, self.progress)
        dd_y_p, g_y_p = ca.hessian(y_p, self.progress)

        curvature = (d_x_p * dd_y_p - d_y_p * dd_x_p) / (d_x_p**2 + d_y_p**2)**(1.5)

        #   Explicit model
        f_expl = ca.vertcat( ca.cos(self.yaw) * self.virtual_speed - self.vx + self.yb * self.wz,\
                             ca.sin(self.yaw) * self.virtual_speed - self.vy - self.xb * self.wz,\
                             curvature * self.virtual_speed - self.wz,\
                             vx_dot,\
                             vy_dot,\
                             wz_dot,\
                             self.virtual_speed,\
                             self.virtual_speed_rate,\
                             self.fr_rate,\
                             self.delta_rate )
        
        #   Implicit model
        f_impl = ca.vertcat( self.xb_dot - ( ca.cos(self.yaw) * self.virtual_speed - self.vx + self.yb * self.wz ),\
                             self.yb_dot - ( ca.sin(self.yaw) * self.virtual_speed - self.vy - self.xb * self.wz ),\
                             self.yaw_dot - ( curvature * self.virtual_speed - self.wz ),\
                             self.vx_dot - vx_dot,\
                             self.vy_dot - vy_dot,\
                             self.wz_dot - wz_dot,\
                             self.progress_dot - self.virtual_speed,\
                             self.virtual_speed_dot - self.virtual_speed_rate,\
                             self.fr_dot - self.fr_rate,\
                             self.delta_dot - self.delta_rate )
        
        #   Call model instance
        model = AcadosModel()

        model.f_impl_expr = f_impl
        model.f_expl_expr = f_expl
        model.x = state
        model.xdot = state_dot
        model.u = controls
        model.name = "dynamics_bicycle_planar"

        model.x_labels = [r'$x$ [m]', r'$y$ [m]', r'$\psi$ [rad]', r'$u$ [m/s]', r'$v$ [m/s]', r'$r$ [rad/s]', r'progress'] 
        model.u_labels = [r'fr_rate [N]', r'delta_rate [N]', r'v [rad]']
        model.t_label = '$t$ [s]'
        
        ###

        #   Call dims instance
        dims = AcadosOcpDims()

        dims.N = specs["N"]
        dims.nx = model.x.rows()
        dims.nu = model.u.rows()
        ###

        #   Call cost instance
        cost = AcadosOcpCost()
        
        if( specs["cost_type"] == "EXTERNAL" ):
            #   Scaled errors
            scale_e_l = self.xb
            scale_e_c = self.yb
            
            #   Normalized variables
            norm_v_rate = self.virtual_speed_rate / con_controls["virtual_speed_rate_ub"]
            norm_fr_rate = self.fr_rate / con_controls["fr_rate_ub"]
            norm_delta_rate = self.delta_rate / con_controls["delta_rate_ub"]
            norm_v = self.virtual_speed / con_state["virtual_speed_ub"]

            #   Error yaw

            y_0 = ca.vertcat( scale_e_l, scale_e_c, norm_v_rate, norm_fr_rate, norm_delta_rate )
            y = ca.vertcat( scale_e_l, scale_e_c, norm_v_rate, norm_fr_rate, norm_delta_rate )
            y_e = ca.vertcat( scale_e_l, scale_e_c )

            cost_expr_0 = y_0.T @ ca.diagcat( weights["Q_xb"], weights["Q_yb"],\
                                         weights["Q_virtual_speed_rate"], weights["Q_fr_rate"], weights["Q_delta_rate"] ) @ y_0\
                                         - weights["Q_virtualSpeed"] * norm_v
            
            cost_expr = y.T @ ca.diagcat( weights["Q_xb"], weights["Q_yb"],\
                                     weights["Q_virtual_speed_rate"], weights["Q_fr_rate"], weights["Q_delta_rate"] ) @ y\
                                     - weights["Q_virtualSpeed"] * norm_v

            cost_expr_e = y_e.T @ ca.diagcat( weights["Q_xb_t"], weights["Q_yb_t"]) @ y_e - weights["Q_virtualSpeed"] * norm_v

            model.cost_expr_ext_cost_0 = cost_expr_0 * 0.1
            model.cost_expr_ext_cost = cost_expr * 0.1
            model.cost_expr_ext_cost_e = cost_expr_e * 0.1

            H_0 = ca.diagcat( ca.hessian(model.cost_expr_ext_cost_0, model.x)[0],\
                              2 * weights["Q_virtual_speed_rate"] / con_controls["virtual_speed_rate_ub"],\
                              2 * weights["Q_fr_rate"] / con_controls["fr_rate_ub"],\
                              2 * weights["Q_delta_rate"] / con_controls["delta_rate_ub"] )
            
            H = ca.diagcat( ca.hessian(model.cost_expr_ext_cost, model.x)[0],\
                              2 * weights["Q_virtual_speed_rate"] / con_controls["virtual_speed_rate_ub"],\
                              2 * weights["Q_fr_rate"] / con_controls["fr_rate_ub"],\
                              2 * weights["Q_delta_rate"] / con_controls["delta_rate_ub"] )
            
            H_e = ca.hessian(model.cost_expr_ext_cost_e, model.x)[0]

            model.cost_expr_ext_cost_custom_hess_0 = H_0 * 0.1
            model.cost_expr_ext_cost_custom_hess = H * 0.1
            model.cost_expr_ext_cost_custom_hess_e = H_e * 0.1

            cost.cost_type_0 = 'EXTERNAL'
            cost.cost_type = 'EXTERNAL'
            cost.cost_type_e = 'EXTERNAL'

        elif( specs["cost_type"] == "CONVEX_OVER_NONLINEAR" ):
            r = ca.SX.sym('r')
            r_e = ca.SX.sym('r_e', 2)

            y_0 = ca.vertcat( e_l, e_c )
            y = ca.vertcat( e_l, e_c )
            y_e = ca.vertcat( e_l, e_c )

            model.cost_y_expr_0 = y_0
            model.cost_y_expr = y
            model.cost_y_expr_e = y_e

            model.cost_r_in_psi_expr = 0.5 * (r.T @ ca.diagcat( weights["Q_e_l"], weights["Q_e_c"] ) @ r )
            model.cost_r_in_psi_expr_e = r_e
            
            cost.cost_type_0 = 'CONVEX_OVER_NONLINEAR'
            cost.cost_type = 'CONVEX_OVER_NONLINEAR'
            cost.cost_type_e = 'CONVEX_OVER_NONLINEAR'

        cost.cost_ext_fun_type_0 = 'casadi'
        cost.cost_ext_fun_type = 'casadi'
        cost.cost_ext_fun_type_e = 'casadi'
        ###

        #  Call constraints instance
        constraints = AcadosOcpConstraints()

        constraints.lbx_0 = np.stack( [con_state["xb_lb"], con_state["yb_lb"], con_state["yaw_lb"], con_state["u_lb"], con_state["v_lb"], con_state["r_lb"], con_state["progress_lb"], con_state["virtual_speed_lb"], con_state["fr_lb"], con_state["delta_lb"]] )
        constraints.lbx = np.stack( [con_state["xb_lb"], con_state["yb_lb"], con_state["yaw_lb"], con_state["u_lb"], con_state["v_lb"], con_state["r_lb"], con_state["progress_lb"], con_state["virtual_speed_lb"], con_state["fr_lb"], con_state["delta_lb"]] )
        constraints.lbx_e = np.stack( [con_state["xb_lb"], con_state["yb_lb"], con_state["yaw_lb"], con_state["u_lb"], con_state["v_lb"], con_state["r_lb"], con_state["progress_lb"], con_state["virtual_speed_lb"], con_state["fr_lb"], con_state["delta_lb"]] )
        
        constraints.ubx_0 = np.stack( [con_state["xb_ub"], con_state["yb_ub"], con_state["yaw_ub"], con_state["u_ub"], con_state["v_ub"], con_state["r_ub"], con_state["progress_ub"], con_state["virtual_speed_ub"], con_state["fr_ub"], con_state["delta_ub"]] )
        constraints.ubx = np.stack( [con_state["xb_ub"], con_state["yb_ub"], con_state["yaw_ub"], con_state["u_ub"], con_state["v_ub"], con_state["r_ub"], con_state["progress_ub"], con_state["virtual_speed_ub"], con_state["fr_ub"], con_state["delta_ub"]] )
        constraints.ubx_e = np.stack( [con_state["xb_ub"], con_state["yb_ub"], con_state["yaw_ub"], con_state["u_ub"], con_state["v_ub"], con_state["r_ub"], con_state["progress_ub"], con_state["virtual_speed_ub"], con_state["fr_ub"], con_state["delta_ub"]] )

        #constraints.C = np.array( ( [ [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0] ] ) )
        #constraints.C_e = np.array( [ [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0] ] )
        #constraints.D = np.array( [ [-1, 1, 0], [1, 1, 0] ] )

        #constraints.lh_0 = np.array( [-1e2] )
        #constraints.lh = np.array( [-1e-1] )

        #constraints.uh_0 = np.array( [1e2] )
        #constraints.uh = np.array( [1e-1] )

        constraints.lbu = np.stack( [con_controls["virtual_speed_rate_lb"], con_controls["fr_rate_lb"], con_controls["delta_rate_lb"]] )
        constraints.ubu = np.stack( [con_controls["virtual_speed_rate_ub"], con_controls["fr_rate_ub"], con_controls["delta_rate_ub"]] )

        #constraints.lg = np.stack( [-1e5, 0] )
        #constraints.ug = np.stack( [0, 1e5] )

        #constraints.lg_e = np.stack( [0, 0] )
        #constraints.ug_e = np.stack( [0, 0] )

        #constraints.idxbx_0 = np.array( [0, 1, 2, 3, 4, 5, 6, 7, 8] )
        constraints.idxbx = np.array( [0, 1, 2, 3, 4, 5, 6, 7, 8, 9] )
        constraints.idxbx_e = np.array( [0, 1, 2, 3, 4, 5, 6, 7, 8, 9] )
        #constraints.idxbxe_0 = np.array( [0, 1, 2, 3, 4, 5, 6, 7, 8] )
        
        constraints.idxbu = np.array( [0, 1, 2] )

        constraints.x0 = np.stack( [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] ) 
        ###

        #   Call solver options instance
        solver_options = AcadosOcpOptions()

        solver_options.N_horizon = specs["N"]
        solver_options.tf = specs["Ts"] * specs["N"]
        solver_options.Tsim = specs["Ts"]
        solver_options.qp_solver = specs["qp_solver"]
        solver_options.hessian_approx = 'EXACT'
        solver_options.integrator_type = 'ERK'
        
        solver_options.nlp_solver_type = specs["solver"]
        solver_options.nlp_solver_warm_start_first_qp = False
        solver_options.nlp_solver_max_iter = 150
        
        solver_options.qp_solver_warm_start = 1
        solver_options.qp_solver_cond_N = specs["N"]
        solver_options.print_level = 0
        solver_options.regularize_method = 'CONVEXIFY'
        solver_options.sim_method_num_stages = 4
        solver_options.sim_method_num_steps = 3
        solver_options.sim_method_newton_iter = 3
        solver_options.qp_solver_iter_max = 100
        solver_options.with_batch_functionality = True
        solver_options.globalization = 'MERIT_BACKTRACKING'
        solver_options.globalization_line_search_use_sufficient_descent = 0
        solver_options.tol = 1e-2
        
        #solver_options.exact_hess_constr = 0
        #solver_options.exact_hess_dyn = 0
        #solver_options.ext_cost_num_hess = 0
        ###

        #   Call ocp instance
        ocp = AcadosOcp()

        #   Set ocp
        ocp.model = model
        ocp.cost = cost
        ocp.dims = dims
        ocp.solver_options = solver_options
        ocp.constraints = constraints

        #   Set folder path where generated c code is located
        ocp.code_export_directory = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/scripts/acados_c_generated_code/Countoring/DynamicsBicyclePlanar/acadosOcp"
        ocp.acados_lib_path = "/home/francisco/acados/lib"
        ocp.acados_include_path = "/home/francisco/acados/include"

        json_file_path = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/scripts/acados_c_generated_code/Countoring/DynamicsBicyclePlanar/acadosOcp/acados_ocp.json"
        ###

        self.solver = AcadosOcpSolver(ocp, json_file = json_file_path)
        """AcadosOcpSolver.generate(ocp, json_file = json_file_path)
        AcadosOcpSolver.build(ocp.code_export_directory, with_cython=True)
        self.solver = AcadosOcpSolver.create_cython_solver(json_file_path)"""

        self.integrator = AcadosSimSolver(ocp, json_file = json_file_path)

    def _getStatesFlat(self):
        return self.solver.get_flat('x')

    def _getControlsFlat(self):
        return self.solver.get_flat('u')

    def _getParametersFlat(self):
        return self.solver.get_flat('p')

    def _setInitialState(self, state):

        """
            state: numpy array
        """

        #   Set initial state
        self.solver.set(0, 'lbx', state )
        self.solver.set(0, 'ubx', state )

    def _setInitialGuess(self, numIter, state):
        
        """
            :numIter int type, number of iterations
            :pose geometry_msgs/Pose.msg type, mobile robot pose
        """

        # do some initial iterations to start with a good initial guess
        for _ in range(numIter):
            print("Iteration: ", _)
            
            self.solver.solve_for_x0(state)

            print("Cost: ", self.solver.get_cost())
            print("Total time: ", self.solver.get_stats("time_tot"))

    def _solve_sqp(self, state):
        
        #   Set initial state
        self._setInitialState(state)

        status = self.solver.solve()

        if status != 0:
            print("acados returned status {} in closed loop iteration.".format(status))

        opt_u = self.solver.get(0, 'u')

        return opt_u, status

    def _preparation_sqp_rti(self):
        self.solver.options_set('rti_phase', 1)
        
        status1 = self.solver.solve()

        if( status1 != 0 ):
            print("acados returned status {} in preparation phase.".format(status1))
    
    def _feedback_sqp_rti(self, state):
        self.solver.options_set('rti_phase', 2)

        #   Set initial state
        self._setInitialState(state)

        status2 = self.solver.solve()

        if( status2 != 0 ):
            print("acados returned status {} in feedback phase.".format(status2))

        opt_u = self.solver.get(0, 'u')

        return opt_u, status2
        
    def _getData(self):

        """ 
            Optimization solution data treatment

            solution: dictionary returned by casadi solver
        """ 
        
        #   Retrieve cost and optimization time
        cost = self.solver.get_cost()
        optTime = self.solver.get_stats('time_tot')

        return cost, optTime
    
    def _plotSolution(self):
        
        """
            Plot solution
        """

        states = self.solver.get_flat('x')
        controls = self.solver.get_flat('u')
        parameters = self.solver.get_flat('p')

        fig, ax = plt.subplots(2, 3)
        ax[0, 0].plot( states[0::6] )
        ax[0, 0].plot( parameters[0::3], 'r--' )
        ax[0, 0].set_title('x')
        
        ax[0, 1].plot( states[1::6] )
        ax[0, 1].plot( parameters[1::3], 'r--' )
        ax[0, 1].set_title('y')

        ax[0, 2].plot( states[2::6] )
        ax[0, 2].plot( parameters[2::3], 'r--' )
        ax[0, 2].set_title('yaw')

        ax[1, 0].plot( controls[0::3] )
        ax[1, 0].set_title('fl_r')

        ax[1, 1].plot( controls[1::3] )
        ax[1, 1].set_title('fl_f')

        ax[1, 2].plot( controls[2::3] )
        ax[1, 2].set_title('delta')

        plt.show()

    def _simulate(self, state, controls):
        return self.integrator.simulate(x = state, u = controls)