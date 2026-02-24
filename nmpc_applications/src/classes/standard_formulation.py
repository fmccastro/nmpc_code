#!/usr/bin/python3.8

import sys

sys.path.insert(0, "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/src")

from classes.all_imports import *
from classes.common_class import *

"""
    Build and define NMPC optimization problem with the standard formulation
"""

#   Optimization problem parameters
class ModelParameters(Common):

    #   Constructor
    def __init__(self):

        super().__init__()
        
        ###   Optimization symbolic variables   ############################

        """
            Mobile robot states
        """

        with open(self.baseline_std_dyn_skid) as f:
            solver_param = json.load(f)
            self.specs = solver_param["solver_specs"]
            self.con_pose = solver_param["con_pose"]
            self.con_vel = solver_param["con_vel"]
            self.weights = solver_param["weights"]
            self.con_rates = solver_param["con_rates"]
            self.virtual_speed_planner = solver_param["virtual_speed_planner"]
        
        #   Work with Leo Rover
        with open(self.vehicle_specs) as f:
            vehicle_param = json.load(f)
            lat_w = vehicle_param["wheelLatSeparation"]

        #   Position
        self.x = ca.SX.sym('x')
        self.y = ca.SX.sym('y')

        #   Fixed angles (rpy)
        self.roll = ca.SX.sym('roll')
        self.pitch = ca.SX.sym('pitch')
        self.yaw = ca.SX.sym('yaw')

        #   Position derivatives
        self.x_dot = ca.SX.sym('x_dot')
        self.y_dot = ca.SX.sym('y_dot')

        #   Fixed angles derivative (rpy)
        self.roll_dot = ca.SX.sym('roll_dot')
        self.pitch_dot = ca.SX.sym('pitch_dot')
        self.yaw_dot = ca.SX.sym('yaw_dot')

        #   Linear velocity (w.r. to body frame)
        self.vx = ca.SX.sym('vx')

        #   Linear velocity derivative (w.r. to body frame)
        self.vx_dot = ca.SX.sym('vx_dot')

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
        self.v_ref = ca.SX.sym('v_ref')

        #   Forces
        self.fl = ca.SX.sym('fl')
        self.fl_dot = ca.SX.sym('fl_dot')
        
        self.fr = ca.SX.sym('fr')
        self.fr_dot = ca.SX.sym('fr_dot')
        
        #   Front and rear wheel steering
        self.fl_rate = ca.SX.sym('fl_rate')
        self.fr_rate = ca.SX.sym('fr_rate')
        self.virtual_speed_rate = ca.SX.sym('virtual_speed_rate')

        #   Parameters
        self.friction = ca.SX.sym('friction')

        #   Reference
        self.x_ref = ca.SX.sym('x_ref')
        self.y_ref = ca.SX.sym('y_ref')
        self.yaw_ref = ca.SX.sym('yaw_ref')

        #   CIAO parameters

        #   Center of convex free region
        self.cx = ca.SX.sym('cx')                                          
        self.cy = ca.SX.sym('cy')

        #   Distance to closest object
        self.distance2obstacle = ca.SX.sym('distance2obstacle')

        #   Safety margin to prevent collisions
        self.safety_margin = ca.SX.sym('safety_margin')

#   Find closest trajectory point to mobile robot
class ClosestPoint(ModelParameters):
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
        self.x_ref = x_ref
        self.y_ref = y_ref

        J = ( self.x - self.x_ref(self.progress) )**2 + ( self.y - self.y_ref(self.progress) )**2

        self.opti.minimize(J)
        
        #self.opti.subject_to( self.opti.bounded( - * ca.pi, self.progress, 2 * ca.pi ) )

        #   Setup solver
        opts = {"ipopt.print_level": 0, "print_time": True, "ipopt.max_iter": 1000, "ipopt.tol": 1e-6, "ipopt.acceptable_tol": 1e-4}
        self.opti.solver("ipopt", opts)
    
    def _findInitialGuess(self, x0, y0):

        """
            Find closest point approximation to be used as initial guess
        """
        
        index = 0

        for progress in np.linspace(0, 2 * math.pi, num=100):
            if(index == 0):
                distance = math.sqrt( math.pow(x0 - float( self.x_ref( progress ) ), 2) + math.pow(y0 - float( self.y_ref( progress ) ), 2) )
                opt_progress = 0
            
            elif(index > 0):
                new_distance = math.sqrt( math.pow(x0 - float( self.x_ref( progress ) ), 2) + math.pow(y0 - float( self.y_ref( progress ) ), 2) )
                
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

#   Standard formulation with rates as controls
class Dynamics(ModelParameters):
    def __init__(self, x_ref, y_ref, yaw_ref):

        super().__init__()
        
        with open(self.baseline_std_dyn_skid) as f:
            solver_param = json.load(f)
            specs = solver_param["solver_specs"]
            con_pose = solver_param["con_pose"]
            con_vel = solver_param["con_vel"]
            con_rates = solver_param["con_rates"]
            weights = solver_param["weights"]

            progress_lb = solver_param["progress_lb"]
            progress_ub = solver_param["progress_ub"]

            fl_lb = solver_param["fl_lb"]
            fl_ub = solver_param["fl_ub"]

            fr_lb = solver_param["fr_lb"]
            fr_ub = solver_param["fr_ub"]
        
        with open(self.vehicle_specs) as f:
            vehicle_param = json.load(f)
            lat_w = vehicle_param["wheelLatSeparation"]
        
        #   State   
        state = ca.vertcat(self.x, self.y, self.yaw, self.progress, self.vx, self.wz)
        
        #   State derivative    
        state_dot = ca.vertcat(self.x_dot, self.y_dot, self.yaw_dot, self.progress_dot, self.vx_dot, self.wz_dot)
        
        #   Controls    
        controls = ca.vertcat(self.virtual_speed, self.fl, self.fr)

        #   Parameters      
        parameters = ca.vertcat(self.roll, self.pitch, self.friction)

        #   Sum of forces and moments
        fx = self.fl + self.fr
        mz = ( self.fr - self.fl ) * lat_w / 2

        sum_fx = fx
        sum_mz = mz

        vx_dot = sum_fx / vehicle_param["m"] - ca.sin( self.pitch ) * vehicle_param["m"] * self.gz
        wz_dot = sum_mz / vehicle_param["izz"]

        #   Explicit model
        f_expl = ca.vertcat( ca.cos(self.pitch) * ca.cos(self.yaw) * self.vx,\
                             ca.cos(self.pitch) * ca.sin(self.yaw) * self.vx,\
                             self.wz,\
                             self.virtual_speed,\
                             vx_dot,\
                             wz_dot )

        #   Implicit model
        f_impl = ca.vertcat( self.x_dot - ca.cos(self.pitch) * ca.cos(self.yaw) * self.vx,\
                             self.y_dot - ca.cos(self.pitch) * ca.sin(self.yaw) * self.vx,\
                             self.yaw_dot - self.wz,\
                             self.progress_dot - self.virtual_speed,\
                             self.vx_dot - vx_dot,\
                             self.wz_dot - wz_dot )

        #   Call model instance
        model = AcadosModel()

        model.f_impl_expr = f_impl
        model.f_expl_expr = f_expl
        model.x = state
        model.xdot = state_dot
        model.u = controls
        model.p = parameters
        model.name = "dynamics"

        model.x_labels = [r'$x$ [m]', r'$y$ [m]', r'$\psi$ [rad]', r'Progress', r'$u$ [m/s]', r'$r$ [rad/s]'] 
        model.u_labels = [r'virtual speed [m/s]', r'fl_rate [N]', r'fl_rate [N]']
        model.t_label = '$t$ [s]'

        #   Error formulations                          
        x_p = x_ref(self.progress)
        y_p = y_ref(self.progress)
        yaw_p = yaw_ref(self.progress)

        e_x = self.x - x_p
        e_y = self.y - y_p
        #error_yaw = self.yaw - yaw_p
        error_yaw = ( ca.cos(self.yaw) - ca.cos(yaw_p) )**2 + ( ca.sin(self.yaw) - ca.sin(yaw_p) )**2
        ###

        #   Call dims instance
        dims = AcadosOcpDims()

        dims.N = specs["N"]
        dims.nx = model.x.rows()
        dims.nu = model.u.rows()
        dims.np = model.p.rows()
        ###

        #   Call cost instance
        cost = AcadosOcpCost()
        
        y_0 = ca.vertcat( e_x, e_y, error_yaw, self.virtual_speed - 0.5, self.fl, self.fr )
        y = ca.vertcat( e_x, e_y, error_yaw, self.virtual_speed - 0.5, self.fl, self.fr )
        y_e = ca.vertcat( e_x, e_y, error_yaw )

        #model.cost_y_expr_0 = y_0              
        #model.cost_y_expr = y
        #model.cost_y_expr_e = y_e

        model.cost_expr_ext_cost_0 = y_0.T @ ca.diag( np.array( [ weights["Q_x"], weights["Q_y"], weights["Q_yaw"],\
                                                                  weights["Q_virtual_speed"], weights["Q_fl"], weights["Q_fr"] ] ) ) @ y_0
        model.cost_expr_ext_cost = y.T @ ca.diag( np.array( [ weights["Q_x"], weights["Q_y"], weights["Q_yaw"],\
                                                              weights["Q_virtual_speed"], weights["Q_fl"], weights["Q_fr"] ] ) ) @ y
        model.cost_expr_ext_cost_e = y_e.T @ ca.diag( np.array( [ weights["Q_x"], weights["Q_y"], weights["Q_yaw"] ] ) ) @ y_e

        cost.cost_type_0 = solver_param["solver_specs"]["cost_type"]
        cost.cost_type = solver_param["solver_specs"]["cost_type"]
        cost.cost_type_e = solver_param["solver_specs"]["cost_type"]

        cost.cost_ext_fun_type_0 = 'casadi'
        cost.cost_ext_fun_type = 'casadi'
        cost.cost_ext_fun_type_e = 'casadi'

        #cost.W_0 = np.diag( np.array( [ weights["Q_x"], weights["Q_y"], weights["Q_yaw"], weights["Q_virtual_speed"] ] ) )
        #cost.W = np.diag( np.array( [ weights["Q_x"], weights["Q_y"], weights["Q_yaw"], weights["Q_virtual_speed"] ] ) )
        #cost.W_e = np.diag( np.array( [ weights["Q_x_t"], weights["Q_y_t"], weights["Q_yaw_t"] ] ) )

        #cost.yref_0 = np.array( [0.0, 0.0, 0.0, 0.0] )
        #cost.yref = np.array( [0.0, 0.0, 0.0, 0.0] )
        #cost.yref_e = np.array( [0.0, 0.0, 0.0] )
        ###

        #  Call constraints instance
        constraints = AcadosOcpConstraints()

        constraints.lbx_0 = np.stack( [ con_pose["x_lb"], con_pose["y_lb"], con_pose["yaw_lb"], progress_lb, con_vel["u_lb"], con_vel["r_lb"] ] )
        constraints.lbx = np.stack( [ con_pose["x_lb"], con_pose["y_lb"], con_pose["yaw_lb"], progress_lb, con_vel["u_lb"], con_vel["r_lb"] ] )
        constraints.lbx_e = np.stack( [ con_pose["x_lb"], con_pose["y_lb"], con_pose["yaw_lb"], progress_lb, con_vel["u_lb"], con_vel["r_lb"] ] )
        
        constraints.ubx_0 = np.stack( [ con_pose["x_ub"], con_pose["y_ub"], con_pose["yaw_ub"], progress_ub, con_vel["u_ub"], con_vel["r_ub"] ] )
        constraints.ubx = np.stack( [ con_pose["x_ub"], con_pose["y_ub"], con_pose["yaw_ub"], progress_ub, con_vel["u_ub"], con_vel["r_ub"] ] )
        constraints.ubx_e = np.stack( [ con_pose["x_ub"], con_pose["y_ub"], con_pose["yaw_ub"], progress_ub, con_vel["u_ub"], con_vel["r_ub"] ] )

        constraints.lbu = np.stack( [ con_vel["virtual_speed_lb"], fl_lb, fr_lb ] )
        constraints.ubu = np.stack( [ con_vel["virtual_speed_ub"], fl_ub, fr_ub ] )

        #constraints.idxbx_0 = np.array( [0, 1, 2, 3, 4, 5, 6, 7, 8] )
        constraints.idxbx = np.array( [0, 1, 2, 3, 4, 5] )
        constraints.idxbx_e = np.array( [0, 1, 2, 3, 4, 5] )
        #constraints.idxbxe_0 = np.array( [0, 1, 2, 3, 4, 5, 6, 7, 8] )
        
        constraints.idxbu = np.array( [0, 1, 2] )

        constraints.x0 = np.stack( [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] ) 
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
        ocp.parameter_values = np.stack([0.0, 0.0, 1.0])

        #   Set folder path where generated c code is located
        ocp.code_export_directory = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/scripts/acados_c_generated_code/Countoring/DynamicsStandard/acadosOcp"
        ocp.acados_lib_path = "/home/francisco/acados/lib"
        ocp.acados_include_path = "/home/francisco/acados/include"

        json_file_path = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/scripts/acados_c_generated_code/Countoring/DynamicsStandard/acadosOcp/acados_ocp.json"
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

#   Standard formulation with rates as controls
class DynamicsRates(ModelParameters):
    def __init__(self, x_ref, y_ref, yaw_ref):

        super().__init__()
        
        with open(self.baseline_std_dyn_skid) as f:
            solver_param = json.load(f)
            specs = solver_param["solver_specs"]
            con_pose = solver_param["con_pose"]
            con_vel = solver_param["con_vel"]
            con_rates = solver_param["con_rates"]
            weights = solver_param["weights"]

            progress_lb = solver_param["progress_lb"]
            progress_ub = solver_param["progress_ub"]

            fl_lb = solver_param["fl_lb"]
            fl_ub = solver_param["fl_ub"]

            fr_lb = solver_param["fr_lb"]
            fr_ub = solver_param["fr_ub"]
        
        with open(self.vehicle_specs) as f:
            vehicle_param = json.load(f)
            lat_w = vehicle_param["wheelLatSeparation"]
        
        #   State           
        state = ca.vertcat(self.x, self.y, self.yaw, self.progress, self.vx, self.wz, self.virtual_speed, self.fl, self.fr)
        
        #   State derivative    
        state_dot = ca.vertcat(self.x_dot, self.y_dot, self.yaw_dot, self.progress_dot, self.vx_dot, self.wz_dot, self.virtual_speed_dot, self.fl_dot, self.fr_dot)
        
        #   Controls            
        controls = ca.vertcat(self.virtual_speed_rate, self.fl_rate, self.fr_rate)

        #   Parameters                                                      
        parameters = ca.vertcat(self.roll, self.pitch, self.friction)

        #   Sum of forces and moments
        fx = self.fl + self.fr
        mz = ( self.fr - self.fl ) * lat_w / 2

        sum_fx = fx
        sum_mz = mz

        vx_dot = sum_fx / vehicle_param["m"] - ca.sin( self.pitch ) * vehicle_param["m"] * self.gz
        wz_dot = sum_mz / vehicle_param["izz"]

        #   Explicit model
        f_expl = ca.vertcat( ca.cos(self.pitch) * ca.cos(self.yaw) * self.vx,\
                             ca.cos(self.pitch) * ca.sin(self.yaw) * self.vx,\
                             self.wz,\
                             self.virtual_speed,\
                             vx_dot,\
                             wz_dot,\
                             self.virtual_speed_rate,\
                             self.fl_rate,\
                             self.fr_rate )

        #   Implicit model
        f_impl = ca.vertcat( self.x_dot - ca.cos(self.pitch) * ca.cos(self.yaw) * self.vx,\
                             self.y_dot - ca.cos(self.pitch) * ca.sin(self.yaw) * self.vx,\
                             self.yaw_dot - self.wz,\
                             self.progress_dot - self.virtual_speed,\
                             self.vx_dot - vx_dot,\
                             self.wz_dot - wz_dot,\
                             self.virtual_speed_dot - self.virtual_speed_rate,\
                             self.fl_dot - self.fl_rate,\
                             self.fr_dot - self.fr_rate )

        #   Call model instance
        model = AcadosModel()

        model.f_impl_expr = f_impl
        model.f_expl_expr = f_expl
        model.x = state
        model.xdot = state_dot
        model.u = controls
        model.p = parameters
        model.name = "dynamics"

        model.x_labels = [r'$x$ [m]', r'$y$ [m]', r'$\psi$ [rad]', r'Progress', r'$u$ [m/s]', r'$r$ [rad/s]'] 
        model.u_labels = [r'virtual speed [m/s]', r'fl_rate [N]', r'fl_rate [N]']
        model.t_label = '$t$ [s]'

        #   Error formulations                          
        x_p = x_ref(self.progress)
        y_p = y_ref(self.progress)
        yaw_p = yaw_ref(self.progress)

        e_x = self.x - x_p
        e_y = self.y - y_p
        #error_yaw = self.yaw - yaw_p
        error_yaw = ( ca.cos(self.yaw) - ca.cos(yaw_p) )**2 + ( ca.sin(self.yaw) - ca.sin(yaw_p) )**2
        ###

        #   Call dims instance
        dims = AcadosOcpDims()

        dims.N = specs["N"]
        dims.nx = model.x.rows()
        dims.nu = model.u.rows()
        dims.np = model.p.rows()
        ###

        #   Call cost instance
        cost = AcadosOcpCost()
        
        y_0 = ca.vertcat( e_x, e_y, error_yaw, self.virtual_speed - 0.5, self.virtual_speed_rate, self.fl_rate, self.fr_rate )
        y = ca.vertcat( e_x, e_y, error_yaw, self.virtual_speed - 0.5, self.virtual_speed_rate, self.fl_rate, self.fr_rate )
        y_e = ca.vertcat( e_x, e_y, error_yaw, self.virtual_speed - 0.5 )

        #model.cost_y_expr_0 = y_0              
        #model.cost_y_expr = y
        #model.cost_y_expr_e = y_e

        model.cost_expr_ext_cost_0 = y_0.T @ ca.diag( np.array( [ weights["Q_x"], weights["Q_y"], weights["Q_yaw"],\
                                                                  weights["Q_virtual_speed"], weights["Q_virtual_speed_rate"], weights["Q_fl_rate"], weights["Q_fr_rate"] ] ) ) @ y_0
        model.cost_expr_ext_cost = y.T @ ca.diag( np.array( [ weights["Q_x"], weights["Q_y"], weights["Q_yaw"],\
                                                              weights["Q_virtual_speed"], weights["Q_virtual_speed_rate"], weights["Q_fl_rate"], weights["Q_fr_rate"] ] ) ) @ y
        model.cost_expr_ext_cost_e = y_e.T @ ca.diag( np.array( [ weights["Q_x"], weights["Q_y"], weights["Q_yaw"], weights["Q_virtual_speed"] ] ) ) @ y_e

        cost.cost_type_0 = solver_param["solver_specs"]["cost_type"]
        cost.cost_type = solver_param["solver_specs"]["cost_type"]
        cost.cost_type_e = solver_param["solver_specs"]["cost_type"]

        cost.cost_ext_fun_type_0 = 'casadi'
        cost.cost_ext_fun_type = 'casadi'
        cost.cost_ext_fun_type_e = 'casadi'

        #cost.W_0 = np.diag( np.array( [ weights["Q_x"], weights["Q_y"], weights["Q_yaw"], weights["Q_virtual_speed"] ] ) )
        #cost.W = np.diag( np.array( [ weights["Q_x"], weights["Q_y"], weights["Q_yaw"], weights["Q_virtual_speed"] ] ) )
        #cost.W_e = np.diag( np.array( [ weights["Q_x_t"], weights["Q_y_t"], weights["Q_yaw_t"] ] ) )

        #cost.yref_0 = np.array( [0.0, 0.0, 0.0, 0.0] )
        #cost.yref = np.array( [0.0, 0.0, 0.0, 0.0] )
        #cost.yref_e = np.array( [0.0, 0.0, 0.0] )
        ###

        #  Call constraints instance
        constraints = AcadosOcpConstraints()

        constraints.lbx_0 = np.stack( [ con_pose["x_lb"], con_pose["y_lb"], con_pose["yaw_lb"], progress_lb,\
                                        con_vel["u_lb"], con_vel["r_lb"], con_vel["virtual_speed_lb"], fl_lb, fr_lb ] )
        constraints.lbx = np.stack( [ con_pose["x_lb"], con_pose["y_lb"], con_pose["yaw_lb"], progress_lb,\
                                        con_vel["u_lb"], con_vel["r_lb"], con_vel["virtual_speed_lb"], fl_lb, fr_lb ] )
        constraints.lbx_e = np.stack( [ con_pose["x_lb"], con_pose["y_lb"], con_pose["yaw_lb"], progress_lb,\
                                        con_vel["u_lb"], con_vel["r_lb"], con_vel["virtual_speed_lb"], fl_lb, fr_lb ] )
        
        constraints.ubx_0 = np.stack( [ con_pose["x_ub"], con_pose["y_ub"], con_pose["yaw_ub"], progress_ub,\
                                        con_vel["u_ub"], con_vel["r_ub"], con_vel["virtual_speed_ub"], fl_ub, fr_ub] )
        constraints.ubx = np.stack( [ con_pose["x_ub"], con_pose["y_ub"], con_pose["yaw_ub"], progress_ub,\
                                        con_vel["u_ub"], con_vel["r_ub"], con_vel["virtual_speed_ub"], fl_ub, fr_ub] )
        constraints.ubx_e = np.stack( [ con_pose["x_ub"], con_pose["y_ub"], con_pose["yaw_ub"], progress_ub,\
                                        con_vel["u_ub"], con_vel["r_ub"], con_vel["virtual_speed_ub"], fl_ub, fr_ub] )

        constraints.lbu = np.stack( [ con_rates["virtual_speed_rate_lb"], con_rates["fl_rate_lb"], con_rates["fr_rate_lb"] ] )
        constraints.ubu = np.stack( [ con_rates["virtual_speed_rate_ub"], con_rates["fl_rate_ub"], con_rates["fr_rate_ub"] ] )

        #constraints.idxbx_0 = np.array( [0, 1, 2, 3, 4, 5, 6, 7, 8] )
        constraints.idxbx = np.array( [0, 1, 2, 3, 4, 5, 6, 7, 8] )
        constraints.idxbx_e = np.array( [0, 1, 2, 3, 4, 5, 6, 7, 8] )
        #constraints.idxbxe_0 = np.array( [0, 1, 2, 3, 4, 5, 6, 7, 8] )
        
        constraints.idxbu = np.array( [0, 1, 2] )

        constraints.x0 = np.stack( [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] ) 
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
        solver_options.tol = 1e-4
        
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
        ocp.parameter_values = np.stack( [0.0, 0.0, 1.0] )

        #   Set folder path where generated c code is located
        ocp.code_export_directory = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/scripts/acados_c_generated_code/Countoring/DynamicsStandard/acadosOcp"
        ocp.acados_lib_path = "/home/francisco/acados/lib"
        ocp.acados_include_path = "/home/francisco/acados/include"

        json_file_path = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/scripts/acados_c_generated_code/Countoring/DynamicsStandard/acadosOcp/acados_ocp.json"
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

#   Standard formulation with rates as controls
class DynamicsPath(ModelParameters):
    def __init__(self):

        super().__init__()
        
        with open(self.baseline_std_dyn_skid) as f:
            solver_param = json.load(f)
            self.specs = solver_param["solver_specs"]
            self.con_pose = solver_param["con_pose"]
            self.con_vel = solver_param["con_vel"]
            self.con_rates = solver_param["con_rates"]
            self.weights = solver_param["weights"]

            progress_lb = solver_param["progress_lb"]
            progress_ub = solver_param["progress_ub"]

            fl_lb = solver_param["fl_lb"]
            fl_ub = solver_param["fl_ub"]

            fr_lb = solver_param["fr_lb"]
            fr_ub = solver_param["fr_ub"]
        
        with open(self.vehicle_specs) as f:
            self.vehicle_param = json.load(f)
            lat_w = self.vehicle_param["wheelLatSeparation"]

        #   State
        state = ca.vertcat(self.x, self.y, self.yaw, self.vx, self.wz)

        #   State derivative
        state_dot = ca.vertcat(self.x_dot, self.y_dot, self.yaw_dot, self.vx_dot, self.wz_dot)
        
        #   Controls
        controls = ca.vertcat(self.fl, self.fr)

        #   Parameters
        parameters = ca.vertcat(self.roll, self.pitch, self.friction, self.yaw_ref)

        tl = self.vehicle_param["m"] * self.gz * ca.sin(self.pitch) / 4 + self.fl
        tr = self.vehicle_param["m"] * self.gz * ca.sin(self.pitch) / 4 + self.fr

        #   Sum of forces and moments
        fx = 2 * ( tl + tr )
        mz = ( tr - tl ) * lat_w 

        sum_fx = fx
        sum_mz = mz

        vx_dot = sum_fx / self.vehicle_param["m"] - ca.sin( self.pitch ) * self.gz
        wz_dot = sum_mz / self.vehicle_param["izz"]

        #   Explicit model  
        f_expl = ca.vertcat( ca.cos(self.pitch) * ca.cos(self.yaw) * self.vx,\
                             ca.cos(self.pitch) * ca.sin(self.yaw) * self.vx,\
                             self.wz,\
                             vx_dot,\
                             wz_dot )
        
        #   Implicit model  
        f_impl = ca.vertcat( self.x_dot - ca.cos(self.pitch) * ca.cos(self.yaw) * self.vx,\
                             self.y_dot - ca.cos(self.pitch) * ca.sin(self.yaw) * self.vx,\
                             self.yaw_dot - self.wz,\
                             self.vx_dot - vx_dot,\
                             self.wz_dot - wz_dot )
        
        #   Call model instance
        model = AcadosModel()

        model.f_impl_expr = f_impl
        model.f_expl_expr = f_expl
        model.x = state
        model.xdot = state_dot
        model.u = controls
        model.p = parameters
        model.name = "dynamics_path"

        model.x_labels = [r'$x$ [m]', r'$y$ [m]', r'$\psi$ [rad]', r'$u$ [m/s]', r'$r$ [rad/s]', r'$t_l$ [N]', r'$t_r$ [N]'] 
        model.u_labels = [r'$t_{l,\,rate}$ [N s]', r'$t_{r,\,rate} [N s]']
        model.t_label = '$t$ [s]'

        #   Error formulations
        e_x = self.x - self.x_ref
        e_y = self.y - self.y_ref
        #error_yaw = self.yaw - self.yaw_ref
        error_yaw = ( ca.cos(self.yaw) - ca.cos(self.yaw_ref) )**2 + ( ca.sin(self.yaw) - ca.sin(self.yaw_ref) )**2
        ###

        #   Call dims instance
        dims = AcadosOcpDims()

        dims.N = self.specs["N"]
        dims.nx = model.x.rows()
        dims.nu = model.u.rows()
        dims.np = model.p.rows()
        ###

        #   Call cost instance
        cost = AcadosOcpCost()
        
        y_0 = ca.vertcat( self.x, self.y, error_yaw, self.vx, self.wz, self.fl, self.fr )
        y = ca.vertcat( self.x, self.y, error_yaw, self.vx, self.wz, self.fl, self.fr )
        y_e = ca.vertcat( self.x, self.y, error_yaw, self.vx, self.wz )

        #model.cost_expr_ext_cost_0 = y_0.T @ ca.diag( np.array( [ weights["Q_x"], weights["Q_y"], weights["Q_yaw"], weights["Q_fl"], weights["Q_fr"] ] ) ) @ y_0
        #model.cost_expr_ext_cost = y.T @ ca.diag( np.array( [ weights["Q_x"], weights["Q_y"], weights["Q_yaw"], weights["Q_fl"], weights["Q_fr"] ] ) ) @ y
        #model.cost_expr_ext_cost_e = y_e.T @ ca.diag( np.array( [ weights["Q_x"], weights["Q_y"], weights["Q_yaw"] ] ) ) @ y_e

        model.cost_y_expr_0 = y_0
        model.cost_y_expr = y
        model.cost_y_expr_e = y_e

        cost.cost_type_0 = solver_param["solver_specs"]["cost_type"]
        cost.cost_type = solver_param["solver_specs"]["cost_type"]
        cost.cost_type_e = solver_param["solver_specs"]["cost_type"]

        cost.cost_ext_fun_type_0 = 'casadi'
        cost.cost_ext_fun_type = 'casadi'
        cost.cost_ext_fun_type_e = 'casadi'

        cost.W_0 = np.diag( np.array( [ self.weights["Q_x"],\
                                        self.weights["Q_y"],\
                                        self.weights["Q_yaw"],\
                                        self.weights["Q_u"],\
                                        self.weights["Q_r"],\
                                        self.weights["Q_fl"],\
                                        self.weights["Q_fr"] ] ) )
        
        cost.W = np.diag( np.array( [ self.weights["Q_x"],\
                                        self.weights["Q_y"],\
                                        self.weights["Q_yaw"],\
                                        self.weights["Q_u"],\
                                        self.weights["Q_r"],\
                                        self.weights["Q_fl"],\
                                        self.weights["Q_fr"] ] ) )
        
        cost.W_e = np.diag( np.array( [ self.weights["Q_x"],\
                                        self.weights["Q_y"],\
                                        self.weights["Q_yaw"],\
                                        self.weights["Q_u"],\
                                        self.weights["Q_r"] ] ) )
        
        cost.yref_0 = np.array( [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] )
        cost.yref = np.array( [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] )
        cost.yref_e = np.array( [0.0, 0.0, 0.0, 0.0, 0.0] )
        ###

        #  Call constraints instances
        constraints = AcadosOcpConstraints()

        constraints.lbx_0 = np.stack( [ self.con_pose["x_lb"], self.con_pose["y_lb"], self.con_pose["yaw_lb"], self.con_vel["u_lb"], self.con_vel["r_lb"]] )
        constraints.lbx = np.stack( [ self.con_pose["x_lb"], self.con_pose["y_lb"], self.con_pose["yaw_lb"], self.con_vel["u_lb"], self.con_vel["r_lb"] ] )
        constraints.lbx_e = np.stack( [ self.con_pose["x_lb"], self.con_pose["y_lb"], self.con_pose["yaw_lb"], self.con_vel["u_lb"], self.con_vel["r_lb"] ] )
        
        constraints.ubx_0 = np.stack( [ self.con_pose["x_ub"], self.con_pose["y_ub"], self.con_pose["yaw_ub"], self.con_vel["u_ub"], self.con_vel["r_ub"] ] )
        constraints.ubx = np.stack( [ self.con_pose["x_ub"], self.con_pose["y_ub"], self.con_pose["yaw_ub"], self.con_vel["u_ub"], self.con_vel["r_ub"] ] )
        constraints.ubx_e = np.stack( [ self.con_pose["x_ub"], self.con_pose["y_ub"], self.con_pose["yaw_ub"], self.con_vel["u_ub"], self.con_vel["r_ub"] ] )

        constraints.lbu = np.stack( [ fl_lb, fr_lb ] )
        constraints.ubu = np.stack( [ fl_ub, fr_ub ] )

        #constraints.idxbx_0 = np.array( [0, 1, 2, 3, 4, 5, 6, 7, 8] )
        constraints.idxbx = np.array( [0, 1, 2, 3, 4] )
        constraints.idxbx_e = np.array( [0, 1, 2, 3, 4] )
        #constraints.idxbxe_0 = np.array( [0, 1, 2, 3, 4, 5, 6, 7, 8] )
        
        constraints.idxbu = np.array( [0, 1] )

        constraints.x0 = np.stack( [0.0, 0.0, 0.0, 0.0, 0.0] ) 
        ###

        #   Call solver options instance
        solver_options = AcadosOcpOptions()

        solver_options.N_horizon = self.specs["N"]
        solver_options.tf = self.specs["Ts"] * self.specs["N"]
        solver_options.Tsim = self.specs["Ts"]
        solver_options.qp_solver = self.specs["qp_solver"]
        solver_options.hessian_approx = 'EXACT'
        solver_options.integrator_type = 'ERK'
        
        solver_options.nlp_solver_type = self.specs["solver"]
        solver_options.nlp_solver_warm_start_first_qp = False
        solver_options.nlp_solver_max_iter = 150
        
        solver_options.qp_solver_warm_start = 1
        solver_options.qp_solver_cond_N = self.specs["N"]
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
        ocp.parameter_values = np.stack( [0.0, 0.0, 1.0, 0.0] )

        #   Set folder path where generated c code is located
        ocp.code_export_directory = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/scripts/acados_c_generated_code/DynamicsStandard/acadosOcp"
        ocp.acados_lib_path = "/home/francisco/acados/lib"
        ocp.acados_include_path = "/home/francisco/acados/include"

        json_file_path = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/scripts/acados_c_generated_code/DynamicsStandard/acadosOcp/acados_ocp.json"
        ###

        self.solver = AcadosOcpSolver(ocp, json_file = json_file_path)
        """AcadosOcpSolver.generate(ocp, json_file = json_file_path)
        AcadosOcpSolver.build(ocp.code_export_directory, with_cython=True)
        self.solver = AcadosOcpSolver.create_cython_solver(json_file_path)"""

        self.integrator = AcadosSimSolver(ocp, json_file = json_file_path)

    def _resetSolver(self):
        self.solver.reset()

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
    
    def _setWeights(self, alpha):

        for i in range(self.solver.acados_ocp.dims.N + 1):
            if( i < self.solver.acados_ocp.dims.N ):
                self.solver.cost_set(i, 'W', np.diag( np.array( [ math.pow(alpha, i) * self.weights["Q_x"],\
                                                                  math.pow(alpha, i) * self.weights["Q_y"],\
                                                                  math.pow(alpha, i) * self.weights["Q_yaw"],\
                                                                  self.weights["Q_u"],\
                                                                  self.weights["Q_r"],\
                                                                  self.weights["Q_fl"],\
                                                                  self.weights["Q_fr"] ] ) ) )
            
            elif( i == self.solver.acados_ocp.dims.N ):
                self.solver.cost_set(i, 'W', np.diag( np.array( [ math.pow(alpha, i) * self.weights["Q_x"],\
                                                                  math.pow(alpha, i) * self.weights["Q_y"],\
                                                                  math.pow(alpha, i) * self.weights["Q_yaw"],\
                                                                  self.weights["Q_u"],\
                                                                  self.weights["Q_r"] ] ) ) )

    def _setReference(self, x_r, y_r, yaw_r, roll=0.0, pitch=0.0, niu=1.0):
        
        for i in range(self.solver.acados_ocp.dims.N + 1):
            self.solver.set( i, 'p', np.stack( [ roll, pitch, niu, yaw_r[i] ] ) )

            if( i < self.solver.acados_ocp.dims.N ):
                self.solver.cost_set(i, 'yref', np.stack( [x_r[i], y_r[i], 0.0, 0.0, 0.0, 0.0, 0.0] ) )
            
            elif( i == self.solver.acados_ocp.dims.N ):
                self.solver.cost_set(i, 'yref', np.stack( [x_r[i], y_r[i], 0.0, 0.0, 0.0] ) )
    
    def _setConstraints(self, f_lb, f_ub):
        for i in range(self.solver.acados_ocp.dims.N + 1):
            self.solver.constraints_set(i, 'lbx', np.array( [ self.con_pose["x_lb"], self.con_pose["y_lb"], self.con_pose["yaw_lb"], self.con_vel["u_lb"], self.con_vel["r_lb"] ] ) )
            self.solver.constraints_set(i, 'ubx', np.array( [ self.con_pose["x_ub"], self.con_pose["y_ub"], self.con_pose["yaw_ub"], self.con_vel["u_ub"], self.con_vel["r_ub"] ] ) )

            if( i < self.solver.acados_ocp.dims.N ):
                self.solver.constraints_set(i, 'lbu', np.array( [ f_lb, f_lb ] ) )
                self.solver.constraints_set(i, 'ubu', np.array( [ f_ub, f_ub ] ) )

    def _setInitialGuess(self, numIter, state, x_r, y_r, yaw_r, roll=0.0, pitch=0.0, niu=1.0):
        
        """                                                         
            :numIter int type, number of iterations
            :pose geometry_msgs/Pose.msg type, mobile robot pose
        """                                                                 

        self._setReference(x_r, y_r, yaw_r, roll=roll, pitch=pitch, niu=niu)

        # do some initial iterations to start with a good initial guess
        for _ in range(numIter):
            print("Initial guess iteration: ", _)

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
    
    def _getVirtualSpeed(self, error, curvature, specs, option = 0):
        
        """
        Docstring for _getVirtualSpeed
        
        :param error: Distance from mobile robot to first reference node
        :param curvature: Path curvature at first reference node
        :param option: whether to include error and curvature or just curvature
        """

        #   Parameters used for planar path tracking
        #L_e = 1e-1
        #L_k = 1e-1

        #k_e = 5e0
        #k_k = 5e-1
        
        #x_m_e = 2e0
        #x_m_k = 1e1

        #   Virtual speed functions parameters
        L_e = specs["virtual_speed_planner"]["L_e"]
        L_k = specs["virtual_speed_planner"]["L_k"]

        k_e = specs["virtual_speed_planner"]["k_e"]
        k_k = specs["virtual_speed_planner"]["k_k"]
        
        x_m_e = specs["virtual_speed_planner"]["x_m_e"]
        x_m_k = specs["virtual_speed_planner"]["x_m_k"]
        
        #   Error
        v_error = L_e / ( 1 + math.exp( k_e * (error - x_m_e) ) )

        #   Curvature
        v_curvature = L_k / ( 1 + math.exp( k_k * (curvature - x_m_k) ) )

        if(option == 0):
            return min(v_error, v_curvature)

        elif(option == 1):
            return v_curvature
    
    def _addNoise2State(state, noise_variance):

        noise = []

        for n in noise_variance:
            noise += [n]

        state[0] = state[0] + np.random.normal(0.0, noise[0], 1)
        state[1] = state[1] + np.random.normal(0.0, noise[1], 1)
        state[2] = state[2] + np.random.normal(0.0, noise[2], 1)

        return state

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
    
    def _getForcesLimits(self, roll, pitch, niu):
        
        #   Define Coulomb friction cone for each wheel (planar case)
        cone = niu * self.vehicle_param["m"] * self.specs["gz"] * math.cos(roll) * math.cos(pitch) / 4 

        #   Set wheel forces upper and lower limits based on the Coulomb friction model
        #  (planar case; constant throughout the simulation)
        if( cone >= 0 ):
            f_lb = -cone
            f_ub = cone

        else:
            f_lb = cone
            f_ub = -cone
        
        return f_lb, f_ub

    def _simulate(self, state, controls, sim_time):
        self.integrator.set('T', sim_time)
        return self.integrator.simulate(x = state, u = controls)

#   Standard formulation with rates as controls
class DynamicsRatesPath(ModelParameters):
    def __init__(self):

        super().__init__()
        
        with open(self.baseline_std_dyn_skid) as f:
            solver_param = json.load(f)
            self.specs = solver_param["solver_specs"]
            self.con_pose = solver_param["con_pose"]
            self.con_vel = solver_param["con_vel"]
            self.con_rates = solver_param["con_rates"]
            self.weights = solver_param["weights"]

            progress_lb = solver_param["progress_lb"]
            progress_ub = solver_param["progress_ub"]

            fl_lb = solver_param["fl_lb"]
            fl_ub = solver_param["fl_ub"]

            fr_lb = solver_param["fr_lb"]
            fr_ub = solver_param["fr_ub"]
        
        with open(self.vehicle_specs) as f:
            self.vehicle_param = json.load(f)
            lat_w = self.vehicle_param["wheelLatSeparation"]

        #   State   
        state = ca.vertcat(self.x, self.y, self.yaw, self.vx, self.wz, self.fl, self.fr)

        #   State derivative    
        state_dot = ca.vertcat(self.x_dot, self.y_dot, self.yaw_dot, self.vx_dot, self.wz_dot, self.fl_dot, self.fr_dot)
        
        #   Controls    
        controls = ca.vertcat(self.fl_rate, self.fr_rate)

        #   Parameters      
        parameters = ca.vertcat(self.roll, self.pitch, self.friction, self.yaw_ref)

        tl = self.vehicle_param["m"] * self.gz * ca.sin(self.pitch) / 4 + self.fl
        tr = self.vehicle_param["m"] * self.gz * ca.sin(self.pitch) / 4 + self.fr

        #   Sum of forces and moments
        fx = 2 * ( tl + tr )
        mz = ( tr - tl ) * lat_w

        sum_fx = fx
        sum_mz = mz
        
        vx_dot = sum_fx / self.vehicle_param["m"] - ca.sin( self.pitch ) * self.gz
        wz_dot = sum_mz / self.vehicle_param["izz"]

        #   Explicit model  
        f_expl = ca.vertcat( ca.cos(self.pitch) * ca.cos(self.yaw) * self.vx,\
                             ca.cos(self.pitch) * ca.sin(self.yaw) * self.vx,\
                             self.wz,\
                             vx_dot,\
                             wz_dot,\
                             self.fl_rate,\
                             self.fr_rate)

        #   Implicit model                                                              
        f_impl = ca.vertcat( self.x_dot - ca.cos(self.pitch) * ca.cos(self.yaw) * self.vx,\
                             self.y_dot - ca.cos(self.pitch) * ca.sin(self.yaw) * self.vx,\
                             self.yaw_dot - self.wz,\
                             self.vx_dot - vx_dot,\
                             self.wz_dot - wz_dot,\
                             self.fl_dot - self.fl_rate,\
                             self.fr_dot - self.fr_rate )

        #   Call model instance
        model = AcadosModel()

        model.f_impl_expr = f_impl
        model.f_expl_expr = f_expl
        model.x = state
        model.xdot = state_dot
        model.u = controls
        model.p = parameters
        model.name = "dynamics_rates_path"

        model.x_labels = [r'$x$ [m]', r'$y$ [m]', r'$\psi$ [rad]', r'$u$ [m/s]', r'$r$ [rad/s]', r'$t_l$ [N]', r'$t_r$ [N]'] 
        model.u_labels = [r'$t_{l,\,rate}$ [N s]', r'$t_{r,\,rate} [N s]']
        model.t_label = '$t$ [s]'
        
        #   Error formulations
        e_x = self.x - self.x_ref
        e_y = self.y - self.y_ref
        #error_yaw = self.yaw - self.yaw_ref
        error_yaw = ( ca.cos(self.yaw) - ca.cos(self.yaw_ref) )**2 + ( ca.sin(self.yaw) - ca.sin(self.yaw_ref) )**2
        ###
        
        #   Call dims instance
        dims = AcadosOcpDims()

        dims.N = self.specs["N"]
        dims.nx = model.x.rows()
        dims.nu = model.u.rows()
        dims.np = model.p.rows()
        ###
        
        #   Call cost instance
        cost = AcadosOcpCost()
        
        y_0 = ca.vertcat( self.x, self.y, error_yaw, self.vx, self.wz, self.fl, self.fr, self.fl_rate, self.fr_rate )
        y = ca.vertcat( self.x, self.y, error_yaw, self.vx, self.wz, self.fl, self.fr, self.fl_rate, self.fr_rate )
        y_e = ca.vertcat( self.x, self.y, error_yaw, self.vx, self.wz, self.fl, self.fr )

        #model.cost_expr_ext_cost_0 = y_0.T @ ca.diag( np.array( [ weights["Q_x"], weights["Q_y"], weights["Q_yaw"],\
        #                                                          weights["Q_fl"], weights["Q_fr"], weights["Q_fl_rate"], weights["Q_fr_rate"] ] ) ) @ y_0
        
        #model.cost_expr_ext_cost = y.T @ ca.diag( np.array( [ weights["Q_x"], weights["Q_y"], weights["Q_yaw"],\
        #                                                      weights["Q_fl"], weights["Q_fr"], weights["Q_fl_rate"], weights["Q_fr_rate"] ] ) ) @ y

        #model.cost_expr_ext_cost_e = y_e.T @ ca.diag( np.array( [ weights["Q_x"], weights["Q_y"], weights["Q_yaw"],\
        #                                                          weights["Q_fl"], weights["Q_fr"] ] ) ) @ y_e

        model.cost_y_expr_0 = y_0
        model.cost_y_expr = y
        model.cost_y_expr_e = y_e

        cost.cost_type_0 = solver_param["solver_specs"]["cost_type"]
        cost.cost_type = solver_param["solver_specs"]["cost_type"]
        cost.cost_type_e = solver_param["solver_specs"]["cost_type"]

        cost.cost_ext_fun_type_0 = 'casadi'
        cost.cost_ext_fun_type = 'casadi'
        cost.cost_ext_fun_type_e = 'casadi'

        cost.W_0 = np.diag( np.array( [ self.weights["Q_x"], self.weights["Q_y"], self.weights["Q_yaw"], self.weights["Q_u"], self.weights["Q_r"],\
                                        self.weights["Q_fl"], self.weights["Q_fr"], self.weights["Q_fl_rate"], self.weights["Q_fr_rate"] ] ) )
        
        cost.W = np.diag( np.array( [ self.weights["Q_x"], self.weights["Q_y"], self.weights["Q_yaw"], self.weights["Q_u"], self.weights["Q_r"],\
                                      self.weights["Q_fl"], self.weights["Q_fr"], self.weights["Q_fl_rate"], self.weights["Q_fr_rate"] ] ) )
        
        cost.W_e = np.diag( np.array( [ self.weights["Q_x_t"], self.weights["Q_y_t"], self.weights["Q_yaw_t"], self.weights["Q_u"], self.weights["Q_r"],\
                                        self.weights["Q_fl"], self.weights["Q_fr"] ] ) )

        cost.yref_0 = np.array( [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] )
        cost.yref = np.array( [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] )
        cost.yref_e = np.array( [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] )
        ###

        #  Call constraints instances
        constraints = AcadosOcpConstraints()

        constraints.lbx_0 = np.stack( [ self.con_pose["x_lb"], self.con_pose["y_lb"], self.con_pose["yaw_lb"], self.con_vel["u_lb"], self.con_vel["r_lb"], fl_lb, fr_lb ] )
        constraints.lbx = np.stack( [ self.con_pose["x_lb"], self.con_pose["y_lb"], self.con_pose["yaw_lb"], self.con_vel["u_lb"], self.con_vel["r_lb"], fl_lb, fr_lb ] )
        constraints.lbx_e = np.stack( [ self.con_pose["x_lb"], self.con_pose["y_lb"], self.con_pose["yaw_lb"], self.con_vel["u_lb"], self.con_vel["r_lb"], fl_lb, fr_lb ] )
        
        constraints.ubx_0 = np.stack( [ self.con_pose["x_ub"], self.con_pose["y_ub"], self.con_pose["yaw_ub"], self.con_vel["u_ub"], self.con_vel["r_ub"], fl_ub, fr_ub ] )
        constraints.ubx = np.stack( [ self.con_pose["x_ub"], self.con_pose["y_ub"], self.con_pose["yaw_ub"], self.con_vel["u_ub"], self.con_vel["r_ub"], fl_ub, fr_ub ] )
        constraints.ubx_e = np.stack( [ self.con_pose["x_ub"], self.con_pose["y_ub"], self.con_pose["yaw_ub"], self.con_vel["u_ub"], self.con_vel["r_ub"], fl_ub, fr_ub ] )

        constraints.lbu = np.stack( [ self.con_rates["fl_rate_lb"], self.con_rates["fr_rate_lb"] ] )
        constraints.ubu = np.stack( [ self.con_rates["fl_rate_ub"], self.con_rates["fr_rate_ub"] ] )

        #constraints.idxbx_0 = np.array( [0, 1, 2, 3, 4, 5, 6, 7, 8] )
        constraints.idxbx = np.array( [0, 1, 2, 3, 4, 5, 6] )
        constraints.idxbx_e = np.array( [0, 1, 2, 3, 4, 5, 6] )
        #constraints.idxbxe_0 = np.array( [0, 1, 2, 3, 4, 5, 6, 7, 8] )
        
        constraints.idxbu = np.array( [0, 1] )

        constraints.x0 = np.stack( [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] )
        ###
        
        #   Call solver options instance
        solver_options = AcadosOcpOptions()

        solver_options.N_horizon = self.specs["N"]
        solver_options.tf = self.specs["Ts"] * self.specs["N"]
        solver_options.Tsim = self.specs["Ts"]
        solver_options.qp_solver = self.specs["qp_solver"]
        solver_options.hessian_approx = 'EXACT'
        solver_options.integrator_type = 'ERK'
        
        solver_options.nlp_solver_type = self.specs["solver"]
        solver_options.nlp_solver_warm_start_first_qp = True
        solver_options.nlp_solver_max_iter = 150
        
        solver_options.qp_solver_warm_start = 1
        solver_options.qp_solver_cond_N = self.specs["N"]
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
        ocp.parameter_values = np.stack( [0.0, 0.0, 1.0, 0.0] )

        #   Set folder path where generated c code is located
        ocp.code_export_directory = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/scripts/acados_c_generated_code/Countoring/DynamicsStandard/acadosOcp"
        ocp.acados_lib_path = "/home/francisco/acados/lib"
        ocp.acados_include_path = "/home/francisco/acados/include"

        json_file_path = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/scripts/acados_c_generated_code/Countoring/DynamicsStandard/acadosOcp/acados_ocp.json"
        ###

        self.solver = AcadosOcpSolver(ocp, json_file = json_file_path)
        #AcadosOcpSolver.generate(ocp, json_file = json_file_path)
        #AcadosOcpSolver.build(ocp.code_export_directory, with_cython=True)
        #self.solver = AcadosOcpSolver.create_cython_solver(json_file_path)

        self.integrator = AcadosSimSolver(ocp, json_file = json_file_path)

    def _resetSolver(self):
        self.solver.reset()

    def _getStatesFlat(self):
        return self.solver.get_flat('x')

    def _getControlsFlat(self):
        return self.solver.get_flat('u')

    def _getParametersFlat(self):
        return self.solver.get_flat('p')

    def _setWeights(self, alpha, f_ub):

        for i in range(self.solver.acados_ocp.dims.N + 1):
            if( i < self.solver.acados_ocp.dims.N ):
                self.solver.cost_set(i, 'W', np.diag( np.array( [ math.pow(alpha, i) * self.weights["Q_x"],\
                                                                  math.pow(alpha, i) * self.weights["Q_y"],\
                                                                  math.pow(alpha, i) * self.weights["Q_yaw"],\
                                                                  self.weights["Q_u"],\
                                                                  self.weights["Q_r"],\
                                                                  self.weights["Q_fl"],\
                                                                  self.weights["Q_fr"],\
                                                                  self.weights["Q_fl_rate"],\
                                                                  self.weights["Q_fr_rate"] ] ) ) )
            
            elif( i == self.solver.acados_ocp.dims.N ):
                self.solver.cost_set(i, 'W', np.diag( np.array( [ math.pow(alpha, i) * self.weights["Q_x"],\
                                                                  math.pow(alpha, i) * self.weights["Q_y"],\
                                                                  math.pow(alpha, i) * self.weights["Q_yaw"],\
                                                                  self.weights["Q_u"],\
                                                                  self.weights["Q_r"],\
                                                                  self.weights["Q_fl"],\
                                                                  self.weights["Q_fr"] ] ) ) )
    
    def _setConstraints(self, f_lb, f_ub):
        for i in range(self.solver.acados_ocp.dims.N + 1):
            self.solver.constraints_set(i, 'lbx', np.array( [ self.con_pose["x_lb"], self.con_pose["y_lb"], self.con_pose["yaw_lb"], self.con_vel["u_lb"], self.con_vel["r_lb"], f_lb, f_lb ] ) )
            self.solver.constraints_set(i, 'ubx', np.array( [ self.con_pose["x_ub"], self.con_pose["y_ub"], self.con_pose["yaw_ub"], self.con_vel["u_ub"], self.con_vel["r_ub"], f_ub, f_ub ] ) )

    def _setGuess(self, solution):

        x = solution[0]
        y = solution[1]
        yaw = solution[2]
        u = solution[3]
        r = solution[4]
        tl = solution[5]
        tr = solution[6]
        tl_rate = solution[7]
        tr_rate = solution[8]

        for i in range(self.solver.acados_ocp.dims.N + 1):

            state_guess = np.stack( [ x[i],\
                                        y[i],\
                                        yaw[i],\
                                        u[i],\
                                        r[i],\
                                        tl[i],\
                                        tr[i] ] )
            
            self.solver.set( i, 'x', state_guess )

            if( i < self.solver.acados_ocp.dims.N ):
                controls_guess = np.stack( [ tl_rate[i],\
                                            tr_rate[i] ] )
                
                self.solver.set(i, 'u', controls_guess )        

    def _setReference(self, x_r, y_r, yaw_r, roll = 0.0, pitch = 0.0, niu = 1.0):
        
        for i in range(self.solver.acados_ocp.dims.N + 1):
            self.solver.set( i, 'p', np.stack( [ roll, pitch, niu, yaw_r[i] ] ) )

            if( i < self.solver.acados_ocp.dims.N ):
                self.solver.cost_set(i, 'yref', np.stack( [x_r[i], y_r[i], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] ) )
            
            elif( i == self.solver.acados_ocp.dims.N ):
                self.solver.cost_set(i, 'yref', np.stack( [x_r[i], y_r[i], 0.0, 0.0, 0.0, 0.0, 0.0] ) )
    
    def _updateSolver(self, x_r, y_r, yaw_r, roll = 0.0, pitch = 0.0, niu = 1.0):
        
        for i in range(self.solver.acados_ocp.dims.N + 1):
            self.solver.set( i, 'p', np.stack( [ roll, pitch, niu, yaw_r[i] ] ) )

            if( i < self.solver.acados_ocp.dims.N ):
                self.solver.cost_set(i, 'yref', np.stack( [x_r[i], y_r[i], 0.0, 0.0, 0.0, 0.0, 0.0] ) )
            
            elif( i == self.solver.acados_ocp.dims.N ):
                self.solver.cost_set(i, 'yref', np.stack( [x_r[i], y_r[i], 0.0, 0.0, 0.0] ) )

            #self.solver.constraints_set(i, 'lbx', np.array( [ self.con_pose["x_lb"], self.con_pose["y_lb"], self.con_pose["yaw_lb"], self.con_vel["u_lb"], self.con_vel["r_lb"], f_lb, f_lb ] ) )
            #self.solver.constraints_set(i, 'ubx', np.array( [ self.con_pose["x_ub"], self.con_pose["y_ub"], self.con_pose["yaw_ub"], self.con_vel["u_ub"], self.con_vel["r_ub"], f_ub, f_ub ] ) )

    def _setInitialState(self, state):
        
        """ 
            state: numpy array
        """ 

        #   Set initial state
        self.solver.set(0, 'lbx', state )
        self.solver.set(0, 'ubx', state )

    def _setInitialGuess(self, numIter, state, x_r, y_r, yaw_r, roll = 0.0, pitch = 0.0, niu = 1.0):
        
        """ 
            :numIter int type, number of iterations
            :pose geometry_msgs/Pose.msg type, mobile robot pose
        """ 
        
        self._setReference(x_r, y_r, yaw_r, roll=roll, pitch=pitch, niu=niu)

        # do some initial iterations to start with a good initial guess
        for _ in range(numIter):
            print("Initial guess iteration: ", _)

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

    def _getVirtualSpeed(self, error, curvature, specs, option = 0):
        
        """
        Docstring for _getVirtualSpeed
        
        :param error: Distance from mobile robot to first reference node
        :param curvature: Path curvature at first reference node
        :param option: whether to include error and curvature or just curvature
        """

        #   Parameters used for planar path tracking
        #L_e = 1e-1
        #L_k = 1e-1

        #k_e = 5e0
        #k_k = 5e-1
        
        #x_m_e = 2e0
        #x_m_k = 1e1

        #   Virtual speed functions parameters
        L_e = specs["virtual_speed_planner"]["L_e"]
        L_k = specs["virtual_speed_planner"]["L_k"]

        k_e = specs["virtual_speed_planner"]["k_e"]
        k_k = specs["virtual_speed_planner"]["k_k"]
        
        x_m_e = specs["virtual_speed_planner"]["x_m_e"]
        x_m_k = specs["virtual_speed_planner"]["x_m_k"]
        
        #   Error
        v_error = L_e / ( 1 + math.exp( k_e * (error - x_m_e) ) )

        #   Curvature
        v_curvature = L_k / ( 1 + math.exp( k_k * (curvature - x_m_k) ) )

        if(option == 0):
            return min(v_error, v_curvature)

        elif(option == 1):
            return v_curvature
    
    def _getForcesLimits(self, roll, pitch, niu):
        
        #   Define Coulomb friction cone for each wheel (planar case)
        cone = niu * self.vehicle_param["m"] * self.specs["gz"] * math.cos(roll) * math.cos(pitch) / 4 

        steady_state_force = abs( self.vehicle_param["m"] * self.specs["gz"] * math.sin(pitch) / 4 )

        #   Set wheel forces upper and lower limits based on the Coulomb friction model
        #  (planar case; constant throughout the simulation)
        if( cone >= 0 ):
            f_lb = -cone + steady_state_force
            f_ub = cone - steady_state_force

        else:
            f_lb = cone + steady_state_force
            f_ub = -cone - steady_state_force
        
        return f_lb, f_ub

    def _addNoise2State(state, noise_variance):

        noise = []

        for n in noise_variance:
            noise += [n]

        state[0] = state[0] + np.random.normal(0.0, noise[0], 1)
        state[1] = state[1] + np.random.normal(0.0, noise[1], 1)
        state[2] = state[2] + np.random.normal(0.0, noise[2], 1)

        return state
        
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

    def _simulate(self, state, controls, sim_time):
        self.integrator.set('T', sim_time)
        return self.integrator.simulate(x = state, u = controls)
    
#   Standard formulation with rates as controls on a 3D scenario
class DynamicsRatesPathRoughTerrain(ModelParameters):
    def __init__(self):

        super().__init__()
        
        with open(self.baseline_std_dyn_skid) as f:
            solver_param = json.load(f)
            self.specs = solver_param["solver_specs"]
            self.con_pose = solver_param["con_pose"]
            self.con_vel = solver_param["con_vel"]
            self.con_rates = solver_param["con_rates"]
            self.weights = solver_param["weights"]

            fl_lb = solver_param["fl_lb"]
            fl_ub = solver_param["fl_ub"]

            fr_lb = solver_param["fr_lb"]
            fr_ub = solver_param["fr_ub"]

        with open(self.vehicle_specs) as f:
            self.vehicle_param = json.load(f)
            lat_w = self.vehicle_param["wheelLatSeparation"]

        #   State   
        state = ca.vertcat(self.x, self.y, self.roll, self.pitch, self.yaw, self.vx, self.wz, self.fl, self.fr)

        #   State derivative    
        state_dot = ca.vertcat(self.x_dot, self.y_dot, self.roll_dot, self.pitch_dot, self.yaw_dot, self.vx_dot, self.wz_dot, self.fl_dot, self.fr_dot)
        
        #   Controls
        controls = ca.vertcat(self.fl_rate, self.fr_rate)

        #   Parameters
        parameters = ca.vertcat(self.friction, self.yaw_ref)

        tl = self.vehicle_param["m"] * self.gz * ca.sin(self.pitch) / 4 + self.fl
        tr = self.vehicle_param["m"] * self.gz * ca.sin(self.pitch) / 4 + self.fr

        #   Sum of forces and moments
        fx = 2 * ( tl + tr )
        mz = ( tr - tl ) * lat_w

        sum_fx = fx
        sum_mz = mz
        
        vx_dot = sum_fx / self.vehicle_param["m"] - ca.sin( self.pitch ) * self.gz
        wz_dot = sum_mz / self.vehicle_param["izz"]

        #   Explicit model
        f_expl = ca.vertcat( ca.cos(self.pitch) * ca.cos(self.yaw) * self.vx,\
                             ca.cos(self.pitch) * ca.sin(self.yaw) * self.vx,\
                             -ca.tan(self.pitch) * ca.cos(self.roll) * self.wz,\
                             ca.sin(self.roll) * self.wz,\
                             ca.cos(self.roll) / ca.cos(self.pitch) * self.wz,\
                             vx_dot,\
                             wz_dot,\
                             self.fl_rate,\
                             self.fr_rate)

        #   Implicit model                                                              
        f_impl = ca.vertcat( self.x_dot - ca.cos(self.pitch) * ca.cos(self.yaw) * self.vx,\
                             self.y_dot - ca.cos(self.pitch) * ca.sin(self.yaw) * self.vx,\
                             self.roll_dot - (-ca.tan(self.pitch) * ca.cos(self.roll) * self.wz),\
                             self.pitch_dot - ca.sin(self.roll) * self.wz,\
                             self.yaw_dot - ca.cos(self.roll) / ca.cos(self.pitch) * self.wz,\
                             self.vx_dot - vx_dot,\
                             self.wz_dot - wz_dot,\
                             self.fl_dot - self.fl_rate,\
                             self.fr_dot - self.fr_rate )

        #   Call model instance
        model = AcadosModel()

        model.f_impl_expr = f_impl
        model.f_expl_expr = f_expl
        model.x = state
        model.xdot = state_dot
        model.u = controls
        model.p = parameters
        model.name = "dynamics_rates_path"

        model.x_labels = [r'$x$ [m]', r'$y$ [m]', r'$\psi$ [rad]', r'$u$ [m/s]', r'$r$ [rad/s]', r'$t_l$ [N]', r'$t_r$ [N]'] 
        model.u_labels = [r'$t_{l,\,rate}$ [N s]', r'$t_{r,\,rate} [N s]']
        model.t_label = '$t$ [s]'
        
        #   Error formulations
        #error_yaw = self.yaw - self.yaw_ref
        error_yaw = ( ca.cos(self.yaw) - ca.cos(self.yaw_ref) )**2 + ( ca.sin(self.yaw) - ca.sin(self.yaw_ref) )**2
        ###
        
        #   Call dims instance
        dims = AcadosOcpDims()

        dims.N = self.specs["N"]
        dims.nx = model.x.rows()
        dims.nu = model.u.rows()
        dims.np = model.p.rows()
        ###
        
        #   Call cost instance
        cost = AcadosOcpCost()
        
        y_0 = ca.vertcat( self.x, self.y, error_yaw, self.vx, self.wz, self.fl, self.fr, self.fl_rate, self.fr_rate )
        y = ca.vertcat( self.x, self.y, error_yaw, self.vx, self.wz, self.fl, self.fr, self.fl_rate, self.fr_rate )
        y_e = ca.vertcat( self.x, self.y, error_yaw, self.vx, self.wz, self.fl, self.fr )

        #model.cost_expr_ext_cost_0 = y_0.T @ ca.diag( np.array( [ weights["Q_x"], weights["Q_y"], weights["Q_yaw"],\
        #                                                          weights["Q_fl"], weights["Q_fr"], weights["Q_fl_rate"], weights["Q_fr_rate"] ] ) ) @ y_0
        
        #model.cost_expr_ext_cost = y.T @ ca.diag( np.array( [ weights["Q_x"], weights["Q_y"], weights["Q_yaw"],\
        #                                                      weights["Q_fl"], weights["Q_fr"], weights["Q_fl_rate"], weights["Q_fr_rate"] ] ) ) @ y

        #model.cost_expr_ext_cost_e = y_e.T @ ca.diag( np.array( [ weights["Q_x"], weights["Q_y"], weights["Q_yaw"],\
        #                                                          weights["Q_fl"], weights["Q_fr"] ] ) ) @ y_e

        model.cost_y_expr_0 = y_0
        model.cost_y_expr = y
        model.cost_y_expr_e = y_e

        model.con_h_expr_0 = ca.vertcat( ( self.vehicle_param["m"] * self.gz * ca.sin(self.pitch) / 4 + self.fr )**2\
                                            + ( -self.vehicle_param["m"] * self.gz * ca.cos(self.pitch) * ca.sin(self.roll) / 4 )**2\
                                            - ( self.friction * self.vehicle_param["m"] * self.gz * ca.cos(self.pitch) * ca.cos(self.roll) / 4 )**2,\
                                            ( self.vehicle_param["m"] * self.gz * ca.sin(self.pitch) / 4 + self.fl )**2\
                                            + ( -self.vehicle_param["m"] * self.gz * ca.cos(self.pitch) * ca.sin(self.roll) / 4 )**2\
                                            - ( self.friction * self.vehicle_param["m"] * self.gz * ca.cos(self.pitch) * ca.cos(self.roll) / 4 )**2 )
        
        model.con_h_expr = ca.vertcat( ( self.vehicle_param["m"] * self.gz * ca.sin(self.pitch) / 4 + self.fr )**2\
                                            + ( -self.vehicle_param["m"] * self.gz * ca.cos(self.pitch) * ca.sin(self.roll) / 4 )**2\
                                            - ( self.friction * self.vehicle_param["m"] * self.gz * ca.cos(self.pitch) * ca.cos(self.roll) / 4 )**2,\
                                            ( self.vehicle_param["m"] * self.gz * ca.sin(self.pitch) / 4 + self.fl )**2\
                                            + ( -self.vehicle_param["m"] * self.gz * ca.cos(self.pitch) * ca.sin(self.roll) / 4 )**2\
                                            - ( self.friction * self.vehicle_param["m"] * self.gz * ca.cos(self.pitch) * ca.cos(self.roll) / 4 )**2 )
        
        model.con_h_expr_e = ca.vertcat( ( self.vehicle_param["m"] * self.gz * ca.sin(self.pitch) / 4 + self.fr )**2\
                                            + ( -self.vehicle_param["m"] * self.gz * ca.cos(self.pitch) * ca.sin(self.roll) / 4 )**2\
                                            - ( self.friction * self.vehicle_param["m"] * self.gz * ca.cos(self.pitch) * ca.cos(self.roll) / 4 )**2,\
                                            ( self.vehicle_param["m"] * self.gz * ca.sin(self.pitch) / 4 + self.fl )**2\
                                            + ( -self.vehicle_param["m"] * self.gz * ca.cos(self.pitch) * ca.sin(self.roll) / 4 )**2\
                                            - ( self.friction * self.vehicle_param["m"] * self.gz * ca.cos(self.pitch) * ca.cos(self.roll) / 4 )**2 )

        cost.cost_type_0 = solver_param["solver_specs"]["cost_type"]
        cost.cost_type = solver_param["solver_specs"]["cost_type"]
        cost.cost_type_e = solver_param["solver_specs"]["cost_type"]

        cost.cost_ext_fun_type_0 = 'casadi'
        cost.cost_ext_fun_type = 'casadi'
        cost.cost_ext_fun_type_e = 'casadi'

        cost.W_0 = np.diag( np.array( [ self.weights["Q_x"], self.weights["Q_y"], self.weights["Q_yaw"], self.weights["Q_u"], self.weights["Q_r"],\
                                        self.weights["Q_fl"], self.weights["Q_fr"], self.weights["Q_fl_rate"], self.weights["Q_fr_rate"] ] ) )
        
        cost.W = np.diag( np.array( [ self.weights["Q_x"], self.weights["Q_y"], self.weights["Q_yaw"], self.weights["Q_u"], self.weights["Q_r"],\
                                      self.weights["Q_fl"], self.weights["Q_fr"], self.weights["Q_fl_rate"], self.weights["Q_fr_rate"] ] ) )
        
        cost.W_e = np.diag( np.array( [ self.weights["Q_x_t"], self.weights["Q_y_t"], self.weights["Q_yaw_t"], self.weights["Q_u"], self.weights["Q_r"],\
                                        self.weights["Q_fl"], self.weights["Q_fr"] ] ) )

        cost.yref_0 = np.array( [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] )
        cost.yref = np.array( [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] )
        cost.yref_e = np.array( [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] )
        ###

        #  Call constraints instances
        constraints = AcadosOcpConstraints()

        constraints.lbx_0 = np.stack( [ self.con_pose["x_lb"], self.con_pose["y_lb"], -1e3, -1e3, self.con_pose["yaw_lb"], self.con_vel["u_lb"], self.con_vel["r_lb"], fl_lb, fr_lb ] )
        constraints.lbx = np.stack( [ self.con_pose["x_lb"], self.con_pose["y_lb"], -1e3, -1e3, self.con_pose["yaw_lb"], self.con_vel["u_lb"], self.con_vel["r_lb"], fl_lb, fr_lb ] )
        constraints.lbx_e = np.stack( [ self.con_pose["x_lb"], self.con_pose["y_lb"], -1e3, -1e3, self.con_pose["yaw_lb"], self.con_vel["u_lb"], self.con_vel["r_lb"], fl_lb, fr_lb ] )
        
        constraints.ubx_0 = np.stack( [ self.con_pose["x_ub"], self.con_pose["y_ub"], 1e3, 1e3, self.con_pose["yaw_ub"], self.con_vel["u_ub"], self.con_vel["r_ub"], fl_ub, fr_ub ] )
        constraints.ubx = np.stack( [ self.con_pose["x_ub"], self.con_pose["y_ub"], 1e3, 1e3, self.con_pose["yaw_ub"], self.con_vel["u_ub"], self.con_vel["r_ub"], fl_ub, fr_ub ] )
        constraints.ubx_e = np.stack( [ self.con_pose["x_ub"], self.con_pose["y_ub"], 1e3, 1e3, self.con_pose["yaw_ub"], self.con_vel["u_ub"], self.con_vel["r_ub"], fl_ub, fr_ub ] )

        constraints.lbu = np.stack( [ self.con_rates["fl_rate_lb"], self.con_rates["fr_rate_lb"] ] )
        constraints.ubu = np.stack( [ self.con_rates["fl_rate_ub"], self.con_rates["fr_rate_ub"] ] )

        constraints.lh_0 = np.array( [-1e3, -1e3] )
        constraints.lh = np.array( [-1e3, -1e3] )
        constraints.lh_e = np.array( [-1e3, -1e3] )

        constraints.uh_0 = np.array( [0.0, 0.0] )
        constraints.uh = np.array( [0.0, 0.0] )
        constraints.uh_e = np.array( [0.0, 0.0] )

        #constraints.idxbx_0 = np.array( [0, 1, 2, 3, 4, 5, 6, 7, 8] )
        constraints.idxbx = np.array( [0, 1, 2, 3, 4, 5, 6, 7, 8] )
        constraints.idxbx_e = np.array( [0, 1, 2, 3, 4, 5, 6, 7, 8] )
        #constraints.idxbxe_0 = np.array( [0, 1, 2, 3, 4, 5, 6, 7, 8] )

        constraints.idxbu = np.array( [0, 1] )

        constraints.x0 = np.stack( [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] )
        ###
        
        #   Call solver options instance
        solver_options = AcadosOcpOptions()

        solver_options.N_horizon = self.specs["N"]
        solver_options.tf = self.specs["Ts"] * self.specs["N"]
        solver_options.Tsim = self.specs["Ts"]
        solver_options.qp_solver = self.specs["qp_solver"]
        solver_options.hessian_approx = 'GAUSS_NEWTON'
        solver_options.integrator_type = 'ERK'
        
        solver_options.nlp_solver_type = self.specs["solver"]
        solver_options.nlp_solver_warm_start_first_qp = True
        solver_options.nlp_solver_max_iter = 150
        
        solver_options.qp_solver_warm_start = 1
        solver_options.qp_solver_cond_N = self.specs["N"]
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
        ocp.parameter_values = np.stack( [1.0, 0.0] )

        #   Set folder path where generated c code is located
        ocp.code_export_directory = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/scripts/acados_c_generated_code/Countoring/DynamicsStandard/acadosOcp"
        ocp.acados_lib_path = "/home/francisco/acados/lib"
        ocp.acados_include_path = "/home/francisco/acados/include"

        json_file_path = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/scripts/acados_c_generated_code/Countoring/DynamicsStandard/acadosOcp/acados_ocp.json"
        ###

        self.solver = AcadosOcpSolver(ocp, json_file = json_file_path)
        #AcadosOcpSolver.generate(ocp, json_file = json_file_path)
        #AcadosOcpSolver.build(ocp.code_export_directory, with_cython=True)
        #self.solver = AcadosOcpSolver.create_cython_solver(json_file_path)

        self.integrator = AcadosSimSolver(ocp, json_file = json_file_path)

    def _resetSolver(self):
        self.solver.reset()

    def _getStatesFlat(self):
        return self.solver.get_flat('x')

    def _getControlsFlat(self):
        return self.solver.get_flat('u')

    def _getParametersFlat(self):
        return self.solver.get_flat('p')

    def _setWeights(self, alpha):

        for i in range(self.solver.acados_ocp.dims.N + 1):
            if( i < self.solver.acados_ocp.dims.N ):
                self.solver.cost_set(i, 'W', np.diag( np.array( [ math.pow(alpha, i) * self.weights["Q_x"],\
                                                                  math.pow(alpha, i) * self.weights["Q_y"],\
                                                                  math.pow(alpha, i) * self.weights["Q_yaw"],\
                                                                  self.weights["Q_u"],\
                                                                  self.weights["Q_r"],\
                                                                  self.weights["Q_fl"],\
                                                                  self.weights["Q_fr"],\
                                                                  self.weights["Q_fl_rate"],\
                                                                  self.weights["Q_fr_rate"] ] ) ) )
            
            elif( i == self.solver.acados_ocp.dims.N ):
                self.solver.cost_set(i, 'W', np.diag( np.array( [ math.pow(alpha, i) * self.weights["Q_x"],\
                                                                  math.pow(alpha, i) * self.weights["Q_y"],\
                                                                  math.pow(alpha, i) * self.weights["Q_yaw"],\
                                                                  self.weights["Q_u"],\
                                                                  self.weights["Q_r"],\
                                                                  self.weights["Q_fl"],\
                                                                  self.weights["Q_fr"] ] ) ) )
    
    def _setConstraints(self, f_lb, f_ub):
        for i in range(self.solver.acados_ocp.dims.N + 1):
            self.solver.constraints_set(i, 'lbx', np.array( [ self.con_pose["x_lb"], self.con_pose["y_lb"], -1e3, -1e3, self.con_pose["yaw_lb"], self.con_vel["u_lb"], self.con_vel["r_lb"], f_lb, f_lb ] ) )
            self.solver.constraints_set(i, 'ubx', np.array( [ self.con_pose["x_ub"], self.con_pose["y_ub"], 1e3, 1e3, self.con_pose["yaw_ub"], self.con_vel["u_ub"], self.con_vel["r_ub"], f_ub, f_ub ] ) )      

    def _setReference(self, x_r, y_r, yaw_r, niu = 1.0):
        
        for i in range(self.solver.acados_ocp.dims.N + 1):
            self.solver.set( i, 'p', np.stack( [ niu, yaw_r[i] ] ) )

            if( i < self.solver.acados_ocp.dims.N ):
                self.solver.cost_set(i, 'yref', np.stack( [x_r[i], y_r[i], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] ) )
            
            elif( i == self.solver.acados_ocp.dims.N ):
                self.solver.cost_set(i, 'yref', np.stack( [x_r[i], y_r[i], 0.0, 0.0, 0.0, 0.0, 0.0] ) )
    
    def _updateSolver(self, x_r, y_r, yaw_r, niu = 1.0):
        
        for i in range(self.solver.acados_ocp.dims.N + 1):
            self.solver.set( i, 'p', np.stack( [ niu, yaw_r[i] ] ) )

            if( i < self.solver.acados_ocp.dims.N ):
                self.solver.cost_set(i, 'yref', np.stack( [x_r[i], y_r[i], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] ) )
            
            elif( i == self.solver.acados_ocp.dims.N ):
                self.solver.cost_set(i, 'yref', np.stack( [x_r[i], y_r[i], 0.0, 0.0, 0.0, 0.0, 0.0] ) )

            #self.solver.constraints_set(i, 'lbx', np.array( [ self.con_pose["x_lb"], self.con_pose["y_lb"], self.con_pose["yaw_lb"], self.con_vel["u_lb"], self.con_vel["r_lb"], f_lb, f_lb ] ) )
            #self.solver.constraints_set(i, 'ubx', np.array( [ self.con_pose["x_ub"], self.con_pose["y_ub"], self.con_pose["yaw_ub"], self.con_vel["u_ub"], self.con_vel["r_ub"], f_ub, f_ub ] ) )

    def _setInitialState(self, state):
        
        """ 
            state: numpy array
        """ 

        #   Set initial state
        self.solver.set(0, 'lbx', state )
        self.solver.set(0, 'ubx', state )

    def _setInitialGuess(self, numIter, state, x_r, y_r, yaw_r, niu = 1.0, solver = "SQP-RTI"):
        
        """ 
            :numIter int type, number of iterations
            :pose geometry_msgs/Pose.msg type, mobile robot pose
        """ 
        
        self._resetSolver()

        self._setReference(x_r, y_r, yaw_r, niu=niu)

        # do some initial iterations to start with a good initial guess
        for _ in range(numIter):
            print("Initial guess iteration: ", _)

            if( solver == "SQP-RTI" ):
                status_p = self._preparation_sqp_rti()
                #new_cost_p, new_opt_time_p = self._getData()

                res, status_f = self._feedback_sqp_rti(state)
                #new_cost_f, new_opt_time_f = self._getData()

                #self.solver.solve_for_x0(state)

            elif( solver == "SQP" ):
                status = self.solver.solve()

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
        
        return status1
    
    def _getResiduals(self):

        print( self.solver.get_residuals() )

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

    def _getVirtualSpeed(self, error, curvature, specs, option = 0):
        
        """
        Docstring for _getVirtualSpeed
        
        :param error: Distance from mobile robot to first reference node
        :param curvature: Path curvature at first reference node
        :param option: whether to include error and curvature or just curvature
        """

        #   Parameters used for planar path tracking
        #L_e = 1e-1
        #L_k = 1e-1

        #k_e = 5e0
        #k_k = 5e-1
        
        #x_m_e = 2e0
        #x_m_k = 1e1

        #   Virtual speed functions parameters
        L_e = specs["virtual_speed_planner"]["L_e"]
        L_k = specs["virtual_speed_planner"]["L_k"]

        k_e = specs["virtual_speed_planner"]["k_e"]
        k_k = specs["virtual_speed_planner"]["k_k"]
        
        x_m_e = specs["virtual_speed_planner"]["x_m_e"]
        x_m_k = specs["virtual_speed_planner"]["x_m_k"]
        
        #   Error
        v_error = L_e / ( 1 + math.exp( k_e * (error - x_m_e) ) )

        #   Curvature
        v_curvature = L_k / ( 1 + math.exp( k_k * (curvature - x_m_k) ) )

        if(option == 0):
            return min(v_error, v_curvature)

        elif(option == 1):
            return v_curvature
    
    def _getForcesLimits(self, roll, pitch, niu):
        
        #   Define Coulomb friction cone for each wheel (planar case)
        cone = niu * self.vehicle_param["m"] * self.specs["gz"] * math.cos(roll) * math.cos(pitch) / 4 

        steady_state_force = self.vehicle_param["m"] * self.specs["gz"] * math.sin(pitch) / 4

        #   Set wheel forces upper and lower limits based on the Coulomb friction model
        #  (planar case; constant throughout the simulation)
        if( cone >= 0 ):
            f_lb = -cone - steady_state_force
            f_ub = cone - steady_state_force

        else:
            f_lb = cone - steady_state_force
            f_ub = -cone - steady_state_force
        
        return f_lb, f_ub

    def _addNoise2State(self, state, noise_variance):

        noise = []

        for n in noise_variance:
            noise += [n]

        state[0] = state[0] + np.random.normal(0.0, noise[0])
        state[1] = state[1] + np.random.normal(0.0, noise[1])
        state[4] = state[4] + np.random.normal(0.0, noise[2])

        return state
        
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

    def _simulate(self, state, controls, sim_time):
        self.integrator.set('T', sim_time)
        return self.integrator.simulate(x = state, u = controls)

#   Standard formulation with rates as controls on a 3D scenario
class DynamicsRatesPathRoughTerrainObstacleAvoidance(ModelParameters):
    def __init__(self):

        super().__init__()
        
        with open(self.baseline_std_dyn_skid) as f:
            solver_param = json.load(f)
            self.specs = solver_param["solver_specs"]
            self.con_pose = solver_param["con_pose"]
            self.con_vel = solver_param["con_vel"]
            self.con_rates = solver_param["con_rates"]
            self.weights = solver_param["weights"]
            self.slacks = solver_param["slacks"]

            fl_lb = solver_param["fl_lb"]
            fl_ub = solver_param["fl_ub"]

            fr_lb = solver_param["fr_lb"]
            fr_ub = solver_param["fr_ub"]

        with open(self.vehicle_specs) as f:
            self.vehicle_param = json.load(f)
            lat_w = self.vehicle_param["wheelLatSeparation"]
        
        #   Load CIAO parameters
        with open(self.ciao_parameters) as f:
            self.ciao_parameters = json.load(f)

        #   State   
        state = ca.vertcat(self.x, self.y, self.roll, self.pitch, self.yaw, self.vx, self.wz, self.fl, self.fr)

        #   State derivative    
        state_dot = ca.vertcat(self.x_dot, self.y_dot, self.roll_dot, self.pitch_dot, self.yaw_dot, self.vx_dot, self.wz_dot, self.fl_dot, self.fr_dot)
        
        #   Controls
        controls = ca.vertcat(self.fl_rate, self.fr_rate)

        #   Parameters
        parameters = ca.vertcat(self.friction, self.yaw_ref, self.cx, self.cy, self.distance2obstacle, self.safety_margin)

        tl = self.vehicle_param["m"] * self.gz * ca.sin(self.pitch) / 4 + self.fl
        tr = self.vehicle_param["m"] * self.gz * ca.sin(self.pitch) / 4 + self.fr

        #   Sum of forces and moments
        fx = 2 * ( tl + tr )
        mz = ( tr - tl ) * lat_w

        sum_fx = fx
        sum_mz = mz
        
        vx_dot = sum_fx / self.vehicle_param["m"] - ca.sin( self.pitch ) * self.gz
        wz_dot = sum_mz / self.vehicle_param["izz"]

        #   Explicit model
        f_expl = ca.vertcat( ca.cos(self.pitch) * ca.cos(self.yaw) * self.vx,\
                             ca.cos(self.pitch) * ca.sin(self.yaw) * self.vx,\
                             -ca.tan(self.pitch) * ca.cos(self.roll) * self.wz,\
                             ca.sin(self.roll) * self.wz,\
                             ca.cos(self.roll) / ca.cos(self.pitch) * self.wz,\
                             vx_dot,\
                             wz_dot,\
                             self.fl_rate,\
                             self.fr_rate)

        #   Implicit model                                                              
        f_impl = ca.vertcat( self.x_dot - ca.cos(self.pitch) * ca.cos(self.yaw) * self.vx,\
                             self.y_dot - ca.cos(self.pitch) * ca.sin(self.yaw) * self.vx,\
                             self.roll_dot - (-ca.tan(self.pitch) * ca.cos(self.roll) * self.wz),\
                             self.pitch_dot - ca.sin(self.roll) * self.wz,\
                             self.yaw_dot - ca.cos(self.roll) / ca.cos(self.pitch) * self.wz,\
                             self.vx_dot - vx_dot,\
                             self.wz_dot - wz_dot,\
                             self.fl_dot - self.fl_rate,\
                             self.fr_dot - self.fr_rate )

        #   Call model instance
        model = AcadosModel()

        model.f_impl_expr = f_impl
        model.f_expl_expr = f_expl
        model.x = state
        model.xdot = state_dot
        model.u = controls
        model.p = parameters
        model.name = "dynamics_rates_path"

        model.x_labels = [r'$x$ [m]', r'$y$ [m]', r'$\psi$ [rad]', r'$u$ [m/s]', r'$r$ [rad/s]', r'$t_l$ [N]', r'$t_r$ [N]'] 
        model.u_labels = [r'$t_{l,\,rate}$ [N s]', r'$t_{r,\,rate} [N s]']
        model.t_label = '$t$ [s]'
        
        #   Error formulations
        error_yaw = ( ca.cos(self.yaw) - ca.cos(self.yaw_ref) )**2 + ( ca.sin(self.yaw) - ca.sin(self.yaw_ref) )**2
        ###
        
        #   Call dims instance
        dims = AcadosOcpDims()

        dims.N = self.specs["N"]
        dims.nx = model.x.rows()
        dims.nu = model.u.rows()
        dims.np = model.p.rows()

        dims.nsh_0 = 1
        dims.nsh = 1
        dims.nsh_e = 1
        ###
        
        #   Call cost instance
        cost = AcadosOcpCost()
        
        y_0 = ca.vertcat( self.x, self.y, error_yaw, self.vx, self.wz, self.fl, self.fr, self.fl_rate, self.fr_rate )
        y = ca.vertcat( self.x, self.y, error_yaw, self.vx, self.wz, self.fl, self.fr, self.fl_rate, self.fr_rate )
        y_e = ca.vertcat( self.x, self.y, error_yaw, self.vx, self.wz, self.fl, self.fr )

        model.cost_y_expr_0 = y_0
        model.cost_y_expr = y
        model.cost_y_expr_e = y_e

        model.con_h_expr_0 = ca.vertcat( ( self.vehicle_param["m"] * self.gz * ca.sin(self.pitch) / 4 + self.fl )**2\
                                            + ( -self.vehicle_param["m"] * self.gz * ca.cos(self.pitch) * ca.sin(self.roll) / 4 )**2\
                                            - ( self.friction * self.vehicle_param["m"] * self.gz * ca.cos(self.pitch) * ca.cos(self.roll) / 4 )**2,\
                                            ( self.vehicle_param["m"] * self.gz * ca.sin(self.pitch) / 4 + self.fr )**2\
                                            + ( -self.vehicle_param["m"] * self.gz * ca.cos(self.pitch) * ca.sin(self.roll) / 4 )**2\
                                            - ( self.friction * self.vehicle_param["m"] * self.gz * ca.cos(self.pitch) * ca.cos(self.roll) / 4 )**2,\
                                            (self.x - self.cx)**2 + (self.y - self.cy)**2 - (self.distance2obstacle - self.safety_margin)**2 )
        
        model.con_h_expr = ca.vertcat( ( self.vehicle_param["m"] * self.gz * ca.sin(self.pitch) / 4 + self.fl )**2\
                                            + ( -self.vehicle_param["m"] * self.gz * ca.cos(self.pitch) * ca.sin(self.roll) / 4 )**2\
                                            - ( self.friction * self.vehicle_param["m"] * self.gz * ca.cos(self.pitch) * ca.cos(self.roll) / 4 )**2,\
                                            ( self.vehicle_param["m"] * self.gz * ca.sin(self.pitch) / 4 + self.fr )**2\
                                            + ( -self.vehicle_param["m"] * self.gz * ca.cos(self.pitch) * ca.sin(self.roll) / 4 )**2\
                                            - ( self.friction * self.vehicle_param["m"] * self.gz * ca.cos(self.pitch) * ca.cos(self.roll) / 4 )**2,\
                                            (self.x - self.cx)**2 + (self.y - self.cy)**2 - (self.distance2obstacle - self.safety_margin)**2 )
        
        model.con_h_expr_e = ca.vertcat( ( self.vehicle_param["m"] * self.gz * ca.sin(self.pitch) / 4 + self.fl )**2\
                                            + ( -self.vehicle_param["m"] * self.gz * ca.cos(self.pitch) * ca.sin(self.roll) / 4 )**2\
                                            - ( self.friction * self.vehicle_param["m"] * self.gz * ca.cos(self.pitch) * ca.cos(self.roll) / 4 )**2,\
                                            ( self.vehicle_param["m"] * self.gz * ca.sin(self.pitch) / 4 + self.fr )**2\
                                            + ( -self.vehicle_param["m"] * self.gz * ca.cos(self.pitch) * ca.sin(self.roll) / 4 )**2\
                                            - ( self.friction * self.vehicle_param["m"] * self.gz * ca.cos(self.pitch) * ca.cos(self.roll) / 4 )**2,\
                                            (self.x - self.cx)**2 + (self.y - self.cy)**2 - (self.distance2obstacle - self.safety_margin)**2 )

        cost.cost_type_0 = solver_param["solver_specs"]["cost_type"]
        cost.cost_type = solver_param["solver_specs"]["cost_type"]
        cost.cost_type_e = solver_param["solver_specs"]["cost_type"]

        cost.cost_ext_fun_type_0 = 'casadi'
        cost.cost_ext_fun_type = 'casadi'
        cost.cost_ext_fun_type_e = 'casadi'

        cost.W_0 = np.diag( np.array( [ self.weights["Q_x"], self.weights["Q_y"], self.weights["Q_yaw"], self.weights["Q_u"], self.weights["Q_r"],\
                                        self.weights["Q_fl"], self.weights["Q_fr"], self.weights["Q_fl_rate"], self.weights["Q_fr_rate"] ] ) )
        
        cost.W = np.diag( np.array( [ self.weights["Q_x"], self.weights["Q_y"], self.weights["Q_yaw"], self.weights["Q_u"], self.weights["Q_r"],\
                                      self.weights["Q_fl"], self.weights["Q_fr"], self.weights["Q_fl_rate"], self.weights["Q_fr_rate"] ] ) )
        
        cost.W_e = np.diag( np.array( [ self.weights["Q_x_t"], self.weights["Q_y_t"], self.weights["Q_yaw_t"], self.weights["Q_u"], self.weights["Q_r"],\
                                        self.weights["Q_fl"], self.weights["Q_fr"] ] ) )

        cost.yref_0 = np.array( [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] )
        cost.yref = np.array( [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] )
        cost.yref_e = np.array( [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] )

        #   Cost linked to slack variable
        cost.Zl_0 = np.array( [0.0] )
        cost.Zl = np.array( [0.0] )
        cost.Zl_e = np.array( [0.0] )
        cost.zl_0 = np.array( [0.0] )
        cost.zl = np.array( [0.0] )
        cost.zl_e = np.array( [0.0] )
        
        cost.Zu_0 = np.array( [ self.slacks["Zu"] ] )
        cost.Zu = np.array( [ self.slacks["Zu"] ] )
        cost.Zu_e = np.array( [ self.slacks["Zu"] ] )

        cost.zu_0 = np.array( [ self.slacks["zu"] ] )
        cost.zu = np.array( [ self.slacks["zu"] ] )
        cost.zu_e = np.array( [ self.slacks["zu"] ] )
        ###

        #  Call constraints instances
        constraints = AcadosOcpConstraints()

        constraints.lbx_0 = np.stack( [ self.con_pose["x_lb"], self.con_pose["y_lb"], -1e3, -1e3, self.con_pose["yaw_lb"], self.con_vel["u_lb"], self.con_vel["r_lb"], fl_lb, fr_lb ] )
        constraints.lbx = np.stack( [ self.con_pose["x_lb"], self.con_pose["y_lb"], -1e3, -1e3, self.con_pose["yaw_lb"], self.con_vel["u_lb"], self.con_vel["r_lb"], fl_lb, fr_lb ] )
        constraints.lbx_e = np.stack( [ self.con_pose["x_lb"], self.con_pose["y_lb"], -1e3, -1e3, self.con_pose["yaw_lb"], self.con_vel["u_lb"], self.con_vel["r_lb"], fl_lb, fr_lb ] )
        
        constraints.ubx_0 = np.stack( [ self.con_pose["x_ub"], self.con_pose["y_ub"], 1e3, 1e3, self.con_pose["yaw_ub"], self.con_vel["u_ub"], self.con_vel["r_ub"], fl_ub, fr_ub ] )
        constraints.ubx = np.stack( [ self.con_pose["x_ub"], self.con_pose["y_ub"], 1e3, 1e3, self.con_pose["yaw_ub"], self.con_vel["u_ub"], self.con_vel["r_ub"], fl_ub, fr_ub ] )
        constraints.ubx_e = np.stack( [ self.con_pose["x_ub"], self.con_pose["y_ub"], 1e3, 1e3, self.con_pose["yaw_ub"], self.con_vel["u_ub"], self.con_vel["r_ub"], fl_ub, fr_ub ] )

        constraints.lbu = np.stack( [ self.con_rates["fl_rate_lb"], self.con_rates["fr_rate_lb"] ] )
        constraints.ubu = np.stack( [ self.con_rates["fl_rate_ub"], self.con_rates["fr_rate_ub"] ] )

        constraints.lh_0 = np.array( [-1e5] * 3 )
        constraints.lh = np.array( [-1e5] * 3 )
        constraints.lh_e = np.array( [-1e5] * 3 )

        constraints.uh_0 = np.array( [0.0] * 3 )
        constraints.uh = np.array( [0.0] * 3 )
        constraints.uh_e = np.array( [0.0] * 3 )

        #constraints.idxbx_0 = np.array( [0, 1, 2, 3, 4, 5, 6, 7, 8] )
        constraints.idxbx = np.array( [0, 1, 2, 3, 4, 5, 6, 7, 8] )
        constraints.idxbx_e = np.array( [0, 1, 2, 3, 4, 5, 6, 7, 8] )
        #constraints.idxbxe_0 = np.array( [0, 1, 2, 3, 4, 5, 6, 7, 8] )

        #   Set slack variable on second nonlinear inequality
        constraints.idxsh_0 = np.array( [2] )
        constraints.idxsh = np.array( [2] )
        constraints.idxsh_e = np.array( [2] )

        constraints.lsh = np.array( [0.0] )
        constraints.ush = np.array( [ self.slacks["ush"] ] )

        constraints.idxbu = np.array( [0, 1] )

        constraints.x0 = np.stack( [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] )
        ###
        
        #   Call solver options instance
        solver_options = AcadosOcpOptions()

        solver_options.N_horizon = self.specs["N"]
        solver_options.tf = self.specs["Ts"] * self.specs["N"]
        solver_options.Tsim = self.specs["Ts"]
        solver_options.qp_solver = self.specs["qp_solver"]
        solver_options.hessian_approx = self.specs["hessian_approx"]
        solver_options.integrator_type = 'ERK'
        
        solver_options.nlp_solver_type = self.specs["solver"]
        solver_options.nlp_solver_warm_start_first_qp = True
        solver_options.nlp_solver_max_iter = 150
        
        solver_options.qp_solver_warm_start = 1
        solver_options.qp_solver_cond_N = self.specs["N"]
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
        ocp.parameter_values = np.stack( [1.0, 0.0, 0.0, 0.0, self.ciao_parameters["maximum_distance"], self.ciao_parameters["safety_margin"]] )

        #   Set folder path where generated c code is located
        ocp.code_export_directory = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/scripts/acados_c_generated_code/Countoring/DynamicsStandard/acadosOcp"
        ocp.acados_lib_path = "/home/francisco/acados/lib"
        ocp.acados_include_path = "/home/francisco/acados/include"

        json_file_path = "/media/francisco/My_Passport/Universidade/IST_repositorio_pessoal/5_ano_2_sem/Tese_RoverNavigation_root/Tese_RoverNavigation/ROS_workspaces/thesis_ws/src/nmpc_applications/scripts/acados_c_generated_code/Countoring/DynamicsStandard/acadosOcp/acados_ocp.json"
        ###

        self.solver = AcadosOcpSolver(ocp, json_file = json_file_path)
        #AcadosOcpSolver.generate(ocp, json_file = json_file_path)
        #AcadosOcpSolver.build(ocp.code_export_directory, with_cython=True)
        #self.solver = AcadosOcpSolver.create_cython_solver(json_file_path)

        self.integrator = AcadosSimSolver(ocp, json_file = json_file_path)

    def _resetSolver(self):
        self.solver.reset()

    def _getStatesFlat(self):
        return self.solver.get_flat('x')

    def _getControlsFlat(self):
        return self.solver.get_flat('u')
    
    def _getSlacksFlat(self):
        return self.solver.get_flat('su')

    def _getParametersFlat(self):
        return self.solver.get_flat('p')

    def _setWeights(self, alpha):

        for i in range(self.solver.acados_ocp.dims.N + 1):
            if( i < self.solver.acados_ocp.dims.N ):
                self.solver.cost_set(i, 'W', np.diag( np.array( [ math.pow(alpha, i) * self.weights["Q_x"],\
                                                                  math.pow(alpha, i) * self.weights["Q_y"],\
                                                                  math.pow(alpha, i) * self.weights["Q_yaw"],\
                                                                  self.weights["Q_u"],\
                                                                  self.weights["Q_r"],\
                                                                  self.weights["Q_fl"],\
                                                                  self.weights["Q_fr"],\
                                                                  self.weights["Q_fl_rate"],\
                                                                  self.weights["Q_fr_rate"] ] ) ) )
            
            elif( i == self.solver.acados_ocp.dims.N ):
                self.solver.cost_set(i, 'W', np.diag( np.array( [ math.pow(alpha, i) * self.weights["Q_x"],\
                                                                  math.pow(alpha, i) * self.weights["Q_y"],\
                                                                  math.pow(alpha, i) * self.weights["Q_yaw"],\
                                                                  self.weights["Q_u"],\
                                                                  self.weights["Q_r"],\
                                                                  self.weights["Q_fl"],\
                                                                  self.weights["Q_fr"] ] ) ) )
    
    def _setConstraints(self, f_lb, f_ub):
        for i in range(self.solver.acados_ocp.dims.N + 1):
            self.solver.constraints_set(i, 'lbx', np.array( [ self.con_pose["x_lb"], self.con_pose["y_lb"], -1e3, -1e3, self.con_pose["yaw_lb"], self.con_vel["u_lb"], self.con_vel["r_lb"], f_lb, f_lb ] ) )
            self.solver.constraints_set(i, 'ubx', np.array( [ self.con_pose["x_ub"], self.con_pose["y_ub"], 1e3, 1e3, self.con_pose["yaw_ub"], self.con_vel["u_ub"], self.con_vel["r_ub"], f_ub, f_ub ] ) )      
    
    def _setReference(self, x_r, y_r, yaw_r, cx, cy, distance2closestobstacle, safety_margin, niu = 1.0):

        """
        Docstring for _setReference
        
        :param x_r: list of x references
        :param y_r: list of y references
        :param yaw_r: list of heading references
        :param cx: list of convex free regions x coordinate center
        :param cy: list of convex free regions y coordinate center
        :param distance2closestobstacle: list of distance to closest obstacle from each convex free region center
        :param safety_margin: float of safety margin (constant along horizon)
        :param niu: float of coefficient of friction (constant along horizon)
        """
        
        for i in range(self.solver.acados_ocp.dims.N + 1):
            self.solver.set( i, 'p', np.stack( [ niu, yaw_r[i], cx[i], cy[i], distance2closestobstacle[i], safety_margin ] ) )

            if( i < self.solver.acados_ocp.dims.N ):
                self.solver.cost_set(i, 'yref', np.stack( [x_r[i], y_r[i], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] ) )
            
            elif( i == self.solver.acados_ocp.dims.N ):
                self.solver.cost_set(i, 'yref', np.stack( [x_r[i], y_r[i], 0.0, 0.0, 0.0, 0.0, 0.0] ) )
    
    def _setGuess(self, state, x_guess, y_guess, yaw_guess):

        for i in range(self.solver.acados_ocp.dims.N + 1):

            if(i == 0):
                self._setInitialState(state)
                self.solver.set( i, 'x', state )

            else:
                self.solver.set( i, 'x', np.stack( [x_guess[i], y_guess[i], 0.0, 0.0, yaw_guess[i], 0.0, 0.0, 0.0, 0.0] ) )
    
    def _updateSolver(self, x_r, y_r, yaw_r, cx, cy, distance2closestobstacle, safety_margin, niu = 1.0):

        """
        Docstring for _updateSolver
        
        :param x_r: list of x references
        :param y_r: list of y references
        :param yaw_r: list of heading references
        :param cx: list of convex free regions x coordinate center
        :param cy: list of convex free regions y coordinate center
        :param distance2closestobstacle: list of distance to closest obstacle from each convex free region center
        :param safety_margin: float of safety margin (constant along horizon)
        :param niu: float of coefficient of friction (constant along horizon)
        """
        
        for i in range(self.solver.acados_ocp.dims.N + 1):
            self.solver.set( i, 'p', np.stack( [ niu, yaw_r[i], cx[i], cy[i], distance2closestobstacle[i], safety_margin ] ) )

            if( i < self.solver.acados_ocp.dims.N ):
                self.solver.cost_set(i, 'yref', np.stack( [x_r[i], y_r[i], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] ) )
            
            elif( i == self.solver.acados_ocp.dims.N ):
                self.solver.cost_set(i, 'yref', np.stack( [x_r[i], y_r[i], 0.0, 0.0, 0.0, 0.0, 0.0] ) )

            #self.solver.constraints_set(i, 'lbx', np.array( [ self.con_pose["x_lb"], self.con_pose["y_lb"], self.con_pose["yaw_lb"], self.con_vel["u_lb"], self.con_vel["r_lb"], f_lb, f_lb ] ) )
            #self.solver.constraints_set(i, 'ubx', np.array( [ self.con_pose["x_ub"], self.con_pose["y_ub"], self.con_pose["yaw_ub"], self.con_vel["u_ub"], self.con_vel["r_ub"], f_ub, f_ub ] ) )

    def _setInitialState(self, state):
        
        """ 
            state: numpy array
        """ 

        #   Set initial state
        self.solver.set(0, 'lbx', state )
        self.solver.set(0, 'ubx', state )

    def _setInitialGuess(self, numIter, state, x_r, y_r, yaw_r, cx, cy, distance2closestobstacle, safety_margin, niu = 1.0, solver="SQP-RTI"):
        
        """
        Docstring for _setInitialGuess
        
        :param numIter: Number of iterations to find proper initial guess
        :param state: vehicle current state
        :param x_r: list of x references
        :param y_r: list of y references
        :param yaw_r: list of heading references
        :param cx: list of convex free regions x coordinate center
        :param cy: list of convex free regions y coordinate center
        :param distance2closestobstacle: list of distance to closest obstacle from each convex free region center
        :param safety_margin: float of safety margin (constant along horizon)
        :param niu: float of coefficient of friction (constant along horizon)
        """
        
        self._resetSolver()

        self._setReference(x_r, y_r, yaw_r, cx, cy, distance2closestobstacle, safety_margin, niu=niu)

        self._setGuess(state, x_r, y_r, yaw_r)

        # do some initial iterations to start with a good initial guess
        for _ in range(numIter):
            print("Initial guess iteration: ", _)

            if( solver == "SQP-RTI" ):
                status_p = self._preparation_sqp_rti()
                #new_cost_p, new_opt_time_p = self._getData()

                res, status_f = self._feedback_sqp_rti(state)
                #new_cost_f, new_opt_time_f = self._getData()

                #self.solver.solve_for_x0(state)

            elif( solver == "SQP" ):
                status = self.solver.solve()

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
        
        return status1
    
    def _getResiduals(self):

        print( self.solver.get_residuals() )

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

    def _getVirtualSpeed(self, error, curvature, specs, option = 0):
        
        """
        Docstring for _getVirtualSpeed
        
        :param error: Distance from mobile robot to first reference node
        :param curvature: Path curvature at first reference node
        :param option: whether to include error and curvature or just curvature
        """

        #   Parameters used for planar path tracking
        #L_e = 1e-1
        #L_k = 1e-1

        #k_e = 5e0
        #k_k = 5e-1
        
        #x_m_e = 2e0
        #x_m_k = 1e1

        #   Virtual speed functions parameters
        L_e = specs["virtual_speed_planner"]["L_e"]
        L_k = specs["virtual_speed_planner"]["L_k"]

        k_e = specs["virtual_speed_planner"]["k_e"]
        k_k = specs["virtual_speed_planner"]["k_k"]
        
        x_m_e = specs["virtual_speed_planner"]["x_m_e"]
        x_m_k = specs["virtual_speed_planner"]["x_m_k"]
        
        #   Error
        v_error = L_e / ( 1 + math.exp( k_e * (error - x_m_e) ) )

        #   Curvature
        v_curvature = L_k / ( 1 + math.exp( k_k * (curvature - x_m_k) ) )

        if(option == 0):
            return min(v_error, v_curvature)

        elif(option == 1):
            return v_curvature
    
    def _getForcesLimits(self, roll, pitch, niu):
        
        #   Define Coulomb friction cone for each wheel (planar case)
        cone = niu * self.vehicle_param["m"] * self.specs["gz"] * math.cos(roll) * math.cos(pitch) / 4 

        steady_state_force = self.vehicle_param["m"] * self.specs["gz"] * math.sin(pitch) / 4

        #   Set wheel forces upper and lower limits based on the Coulomb friction model
        #  (planar case; constant throughout the simulation)
        if( cone >= 0 ):
            f_lb = -cone - steady_state_force
            f_ub = cone - steady_state_force

        else:
            f_lb = cone - steady_state_force
            f_ub = -cone - steady_state_force
        
        return f_lb, f_ub

    def _addNoise2State(self, state, noise_variance):

        noise = []

        for n in noise_variance:
            noise += [n]

        state[0] = state[0] + np.random.normal(0.0, noise[0])
        state[1] = state[1] + np.random.normal(0.0, noise[1])
        state[4] = state[4] + np.random.normal(0.0, noise[2])

        return state
        
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

    def _simulate(self, state, controls, sim_time):
        self.integrator.set('T', sim_time)
        return self.integrator.simulate(x = state, u = controls)