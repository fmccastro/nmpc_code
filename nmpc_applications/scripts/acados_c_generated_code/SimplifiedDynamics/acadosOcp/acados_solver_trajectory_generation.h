/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * The 2-Clause BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.;
 */

#ifndef ACADOS_SOLVER_trajectory_generation_H_
#define ACADOS_SOLVER_trajectory_generation_H_

#include "acados/utils/types.h"

#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"

#define TRAJECTORY_GENERATION_NX     9
#define TRAJECTORY_GENERATION_NZ     0
#define TRAJECTORY_GENERATION_NU     6
#define TRAJECTORY_GENERATION_NP     3
#define TRAJECTORY_GENERATION_NP_GLOBAL     0
#define TRAJECTORY_GENERATION_NBX    9
#define TRAJECTORY_GENERATION_NBX0   9
#define TRAJECTORY_GENERATION_NBU    6
#define TRAJECTORY_GENERATION_NSBX   0
#define TRAJECTORY_GENERATION_NSBU   0
#define TRAJECTORY_GENERATION_NSH    0
#define TRAJECTORY_GENERATION_NSH0   0
#define TRAJECTORY_GENERATION_NSG    0
#define TRAJECTORY_GENERATION_NSPHI  0
#define TRAJECTORY_GENERATION_NSHN   0
#define TRAJECTORY_GENERATION_NSGN   0
#define TRAJECTORY_GENERATION_NSPHIN 0
#define TRAJECTORY_GENERATION_NSPHI0 0
#define TRAJECTORY_GENERATION_NSBXN  0
#define TRAJECTORY_GENERATION_NS     0
#define TRAJECTORY_GENERATION_NS0    0
#define TRAJECTORY_GENERATION_NSN    0
#define TRAJECTORY_GENERATION_NG     0
#define TRAJECTORY_GENERATION_NBXN   9
#define TRAJECTORY_GENERATION_NGN    0
#define TRAJECTORY_GENERATION_NY0    0
#define TRAJECTORY_GENERATION_NY     0
#define TRAJECTORY_GENERATION_NYN    0
#define TRAJECTORY_GENERATION_N      50
#define TRAJECTORY_GENERATION_NH     8
#define TRAJECTORY_GENERATION_NHN    0
#define TRAJECTORY_GENERATION_NH0    8
#define TRAJECTORY_GENERATION_NPHI0  0
#define TRAJECTORY_GENERATION_NPHI   0
#define TRAJECTORY_GENERATION_NPHIN  0
#define TRAJECTORY_GENERATION_NR     0

#ifdef __cplusplus
extern "C" {
#endif


// ** capsule for solver data **
typedef struct trajectory_generation_solver_capsule
{
    // acados objects
    ocp_nlp_in *nlp_in;
    ocp_nlp_out *nlp_out;
    ocp_nlp_out *sens_out;
    ocp_nlp_solver *nlp_solver;
    void *nlp_opts;
    ocp_nlp_plan_t *nlp_solver_plan;
    ocp_nlp_config *nlp_config;
    ocp_nlp_dims *nlp_dims;

    // number of expected runtime parameters
    unsigned int nlp_np;

    /* external functions */

    // dynamics

    external_function_external_param_casadi *expl_vde_forw;
    external_function_external_param_casadi *expl_ode_fun;
    external_function_external_param_casadi *expl_vde_adj;

    external_function_external_param_casadi *expl_ode_hess;



    // cost

    external_function_external_param_casadi *ext_cost_fun;
    external_function_external_param_casadi *ext_cost_fun_jac;
    external_function_external_param_casadi *ext_cost_fun_jac_hess;





    external_function_external_param_casadi ext_cost_0_fun;
    external_function_external_param_casadi ext_cost_0_fun_jac;
    external_function_external_param_casadi ext_cost_0_fun_jac_hess;




    external_function_external_param_casadi ext_cost_e_fun;
    external_function_external_param_casadi ext_cost_e_fun_jac;
    external_function_external_param_casadi ext_cost_e_fun_jac_hess;



    // constraints
    external_function_external_param_casadi *nl_constr_h_fun_jac;
    external_function_external_param_casadi *nl_constr_h_fun;
    external_function_external_param_casadi *nl_constr_h_fun_jac_hess;





    external_function_external_param_casadi nl_constr_h_0_fun_jac;
    external_function_external_param_casadi nl_constr_h_0_fun;
    external_function_external_param_casadi nl_constr_h_0_fun_jac_hess;






} trajectory_generation_solver_capsule;

ACADOS_SYMBOL_EXPORT trajectory_generation_solver_capsule * trajectory_generation_acados_create_capsule(void);
ACADOS_SYMBOL_EXPORT int trajectory_generation_acados_free_capsule(trajectory_generation_solver_capsule *capsule);

ACADOS_SYMBOL_EXPORT int trajectory_generation_acados_create(trajectory_generation_solver_capsule * capsule);

ACADOS_SYMBOL_EXPORT int trajectory_generation_acados_reset(trajectory_generation_solver_capsule* capsule, int reset_qp_solver_mem);

/**
 * Generic version of trajectory_generation_acados_create which allows to use a different number of shooting intervals than
 * the number used for code generation. If new_time_steps=NULL and n_time_steps matches the number used for code
 * generation, the time-steps from code generation is used.
 */
ACADOS_SYMBOL_EXPORT int trajectory_generation_acados_create_with_discretization(trajectory_generation_solver_capsule * capsule, int n_time_steps, double* new_time_steps);
/**
 * Update the time step vector. Number N must be identical to the currently set number of shooting nodes in the
 * nlp_solver_plan. Returns 0 if no error occurred and a otherwise a value other than 0.
 */
ACADOS_SYMBOL_EXPORT int trajectory_generation_acados_update_time_steps(trajectory_generation_solver_capsule * capsule, int N, double* new_time_steps);
/**
 * This function is used for updating an already initialized solver with a different number of qp_cond_N.
 */
ACADOS_SYMBOL_EXPORT int trajectory_generation_acados_update_qp_solver_cond_N(trajectory_generation_solver_capsule * capsule, int qp_solver_cond_N);
ACADOS_SYMBOL_EXPORT int trajectory_generation_acados_update_params(trajectory_generation_solver_capsule * capsule, int stage, double *value, int np);
ACADOS_SYMBOL_EXPORT int trajectory_generation_acados_update_params_sparse(trajectory_generation_solver_capsule * capsule, int stage, int *idx, double *p, int n_update);
ACADOS_SYMBOL_EXPORT int trajectory_generation_acados_set_p_global_and_precompute_dependencies(trajectory_generation_solver_capsule* capsule, double* data, int data_len);

ACADOS_SYMBOL_EXPORT int trajectory_generation_acados_solve(trajectory_generation_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT int trajectory_generation_acados_setup_qp_matrices_and_factorize(trajectory_generation_solver_capsule* capsule);

ACADOS_SYMBOL_EXPORT void trajectory_generation_acados_batch_solve(trajectory_generation_solver_capsule ** capsules, int * status_out, int N_batch);

ACADOS_SYMBOL_EXPORT void trajectory_generation_acados_batch_set_flat(trajectory_generation_solver_capsule ** capsules, const char *field, double *data, int N_data, int N_batch);
ACADOS_SYMBOL_EXPORT void trajectory_generation_acados_batch_get_flat(trajectory_generation_solver_capsule ** capsules, const char *field, double *data, int N_data, int N_batch);

ACADOS_SYMBOL_EXPORT void trajectory_generation_acados_batch_eval_solution_sens_adj_p(trajectory_generation_solver_capsule ** capsules, const char *field, int stage, double *out, int offset, int N_batch);
ACADOS_SYMBOL_EXPORT void trajectory_generation_acados_batch_eval_params_jac(trajectory_generation_solver_capsule ** capsules, int N_batch);


ACADOS_SYMBOL_EXPORT int trajectory_generation_acados_free(trajectory_generation_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT void trajectory_generation_acados_print_stats(trajectory_generation_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT int trajectory_generation_acados_custom_update(trajectory_generation_solver_capsule* capsule, double* data, int data_len);


ACADOS_SYMBOL_EXPORT ocp_nlp_in *trajectory_generation_acados_get_nlp_in(trajectory_generation_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_out *trajectory_generation_acados_get_nlp_out(trajectory_generation_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_out *trajectory_generation_acados_get_sens_out(trajectory_generation_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_solver *trajectory_generation_acados_get_nlp_solver(trajectory_generation_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_config *trajectory_generation_acados_get_nlp_config(trajectory_generation_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT void *trajectory_generation_acados_get_nlp_opts(trajectory_generation_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_dims *trajectory_generation_acados_get_nlp_dims(trajectory_generation_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_plan_t *trajectory_generation_acados_get_nlp_plan(trajectory_generation_solver_capsule * capsule);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif  // ACADOS_SOLVER_trajectory_generation_H_
