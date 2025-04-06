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

#ifndef ACADOS_SOLVER_kinematics_H_
#define ACADOS_SOLVER_kinematics_H_

#include "acados/utils/types.h"

#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"

#define KINEMATICS_NX     6
#define KINEMATICS_NZ     0
#define KINEMATICS_NU     2
#define KINEMATICS_NP     3
#define KINEMATICS_NP_GLOBAL     0
#define KINEMATICS_NBX    0
#define KINEMATICS_NBX0   6
#define KINEMATICS_NBU    2
#define KINEMATICS_NSBX   0
#define KINEMATICS_NSBU   0
#define KINEMATICS_NSH    0
#define KINEMATICS_NSH0   0
#define KINEMATICS_NSG    0
#define KINEMATICS_NSPHI  0
#define KINEMATICS_NSHN   0
#define KINEMATICS_NSGN   0
#define KINEMATICS_NSPHIN 0
#define KINEMATICS_NSPHI0 0
#define KINEMATICS_NSBXN  0
#define KINEMATICS_NS     0
#define KINEMATICS_NS0    0
#define KINEMATICS_NSN    0
#define KINEMATICS_NG     0
#define KINEMATICS_NBXN   0
#define KINEMATICS_NGN    0
#define KINEMATICS_NY0    0
#define KINEMATICS_NY     0
#define KINEMATICS_NYN    0
#define KINEMATICS_N      50
#define KINEMATICS_NH     0
#define KINEMATICS_NHN    0
#define KINEMATICS_NH0    0
#define KINEMATICS_NPHI0  0
#define KINEMATICS_NPHI   0
#define KINEMATICS_NPHIN  0
#define KINEMATICS_NR     0

#ifdef __cplusplus
extern "C" {
#endif


// ** capsule for solver data **
typedef struct kinematics_solver_capsule
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







} kinematics_solver_capsule;

ACADOS_SYMBOL_EXPORT kinematics_solver_capsule * kinematics_acados_create_capsule(void);
ACADOS_SYMBOL_EXPORT int kinematics_acados_free_capsule(kinematics_solver_capsule *capsule);

ACADOS_SYMBOL_EXPORT int kinematics_acados_create(kinematics_solver_capsule * capsule);

ACADOS_SYMBOL_EXPORT int kinematics_acados_reset(kinematics_solver_capsule* capsule, int reset_qp_solver_mem);

/**
 * Generic version of kinematics_acados_create which allows to use a different number of shooting intervals than
 * the number used for code generation. If new_time_steps=NULL and n_time_steps matches the number used for code
 * generation, the time-steps from code generation is used.
 */
ACADOS_SYMBOL_EXPORT int kinematics_acados_create_with_discretization(kinematics_solver_capsule * capsule, int n_time_steps, double* new_time_steps);
/**
 * Update the time step vector. Number N must be identical to the currently set number of shooting nodes in the
 * nlp_solver_plan. Returns 0 if no error occurred and a otherwise a value other than 0.
 */
ACADOS_SYMBOL_EXPORT int kinematics_acados_update_time_steps(kinematics_solver_capsule * capsule, int N, double* new_time_steps);
/**
 * This function is used for updating an already initialized solver with a different number of qp_cond_N.
 */
ACADOS_SYMBOL_EXPORT int kinematics_acados_update_qp_solver_cond_N(kinematics_solver_capsule * capsule, int qp_solver_cond_N);
ACADOS_SYMBOL_EXPORT int kinematics_acados_update_params(kinematics_solver_capsule * capsule, int stage, double *value, int np);
ACADOS_SYMBOL_EXPORT int kinematics_acados_update_params_sparse(kinematics_solver_capsule * capsule, int stage, int *idx, double *p, int n_update);
ACADOS_SYMBOL_EXPORT int kinematics_acados_set_p_global_and_precompute_dependencies(kinematics_solver_capsule* capsule, double* data, int data_len);

ACADOS_SYMBOL_EXPORT int kinematics_acados_solve(kinematics_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT int kinematics_acados_setup_qp_matrices_and_factorize(kinematics_solver_capsule* capsule);

ACADOS_SYMBOL_EXPORT void kinematics_acados_batch_solve(kinematics_solver_capsule ** capsules, int * status_out, int N_batch);

ACADOS_SYMBOL_EXPORT void kinematics_acados_batch_set_flat(kinematics_solver_capsule ** capsules, const char *field, double *data, int N_data, int N_batch);
ACADOS_SYMBOL_EXPORT void kinematics_acados_batch_get_flat(kinematics_solver_capsule ** capsules, const char *field, double *data, int N_data, int N_batch);

ACADOS_SYMBOL_EXPORT void kinematics_acados_batch_eval_solution_sens_adj_p(kinematics_solver_capsule ** capsules, const char *field, int stage, double *out, int offset, int N_batch);
ACADOS_SYMBOL_EXPORT void kinematics_acados_batch_eval_params_jac(kinematics_solver_capsule ** capsules, int N_batch);


ACADOS_SYMBOL_EXPORT int kinematics_acados_free(kinematics_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT void kinematics_acados_print_stats(kinematics_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT int kinematics_acados_custom_update(kinematics_solver_capsule* capsule, double* data, int data_len);


ACADOS_SYMBOL_EXPORT ocp_nlp_in *kinematics_acados_get_nlp_in(kinematics_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_out *kinematics_acados_get_nlp_out(kinematics_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_out *kinematics_acados_get_sens_out(kinematics_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_solver *kinematics_acados_get_nlp_solver(kinematics_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_config *kinematics_acados_get_nlp_config(kinematics_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT void *kinematics_acados_get_nlp_opts(kinematics_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_dims *kinematics_acados_get_nlp_dims(kinematics_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_plan_t *kinematics_acados_get_nlp_plan(kinematics_solver_capsule * capsule);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif  // ACADOS_SOLVER_kinematics_H_
