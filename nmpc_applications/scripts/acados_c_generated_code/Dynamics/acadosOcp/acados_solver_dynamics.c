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

// standard
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
// acados
// #include "acados/utils/print.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
// openmp
#include <omp.h>

// example specific
#include "dynamics_model/dynamics_model.h"


#include "dynamics_cost/dynamics_cost.h"



#include "acados_solver_dynamics.h"

#define NX     DYNAMICS_NX
#define NZ     DYNAMICS_NZ
#define NU     DYNAMICS_NU
#define NP     DYNAMICS_NP
#define NP_GLOBAL     DYNAMICS_NP_GLOBAL
#define NY0    DYNAMICS_NY0
#define NY     DYNAMICS_NY
#define NYN    DYNAMICS_NYN

#define NBX    DYNAMICS_NBX
#define NBX0   DYNAMICS_NBX0
#define NBU    DYNAMICS_NBU
#define NG     DYNAMICS_NG
#define NBXN   DYNAMICS_NBXN
#define NGN    DYNAMICS_NGN

#define NH     DYNAMICS_NH
#define NHN    DYNAMICS_NHN
#define NH0    DYNAMICS_NH0
#define NPHI   DYNAMICS_NPHI
#define NPHIN  DYNAMICS_NPHIN
#define NPHI0  DYNAMICS_NPHI0
#define NR     DYNAMICS_NR

#define NS     DYNAMICS_NS
#define NS0    DYNAMICS_NS0
#define NSN    DYNAMICS_NSN

#define NSBX   DYNAMICS_NSBX
#define NSBU   DYNAMICS_NSBU
#define NSH0   DYNAMICS_NSH0
#define NSH    DYNAMICS_NSH
#define NSHN   DYNAMICS_NSHN
#define NSG    DYNAMICS_NSG
#define NSPHI0 DYNAMICS_NSPHI0
#define NSPHI  DYNAMICS_NSPHI
#define NSPHIN DYNAMICS_NSPHIN
#define NSGN   DYNAMICS_NSGN
#define NSBXN  DYNAMICS_NSBXN



// ** solver data **

dynamics_solver_capsule * dynamics_acados_create_capsule(void)
{
    void* capsule_mem = malloc(sizeof(dynamics_solver_capsule));
    dynamics_solver_capsule *capsule = (dynamics_solver_capsule *) capsule_mem;

    return capsule;
}


int dynamics_acados_free_capsule(dynamics_solver_capsule *capsule)
{
    free(capsule);
    return 0;
}


int dynamics_acados_create(dynamics_solver_capsule* capsule)
{
    int N_shooting_intervals = DYNAMICS_N;
    double* new_time_steps = NULL; // NULL -> don't alter the code generated time-steps
    return dynamics_acados_create_with_discretization(capsule, N_shooting_intervals, new_time_steps);
}


int dynamics_acados_update_time_steps(dynamics_solver_capsule* capsule, int N, double* new_time_steps)
{
    if (N != capsule->nlp_solver_plan->N) {
        fprintf(stderr, "dynamics_acados_update_time_steps: given number of time steps (= %d) " \
            "differs from the currently allocated number of " \
            "time steps (= %d)!\n" \
            "Please recreate with new discretization and provide a new vector of time_stamps!\n",
            N, capsule->nlp_solver_plan->N);
        return 1;
    }

    ocp_nlp_config * nlp_config = capsule->nlp_config;
    ocp_nlp_dims * nlp_dims = capsule->nlp_dims;
    ocp_nlp_in * nlp_in = capsule->nlp_in;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "Ts", &new_time_steps[i]);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "scaling", &new_time_steps[i]);
    }
    return 0;
}

/**
 * Internal function for dynamics_acados_create: step 1
 */
void dynamics_acados_create_set_plan(ocp_nlp_plan_t* nlp_solver_plan, const int N)
{
    assert(N == nlp_solver_plan->N);

    /************************************************
    *  plan
    ************************************************/

    nlp_solver_plan->nlp_solver = SQP_RTI;

    nlp_solver_plan->ocp_qp_solver_plan.qp_solver = PARTIAL_CONDENSING_HPIPM;
    nlp_solver_plan->relaxed_ocp_qp_solver_plan.qp_solver = PARTIAL_CONDENSING_HPIPM;

    nlp_solver_plan->nlp_cost[0] = EXTERNAL;
    for (int i = 1; i < N; i++)
        nlp_solver_plan->nlp_cost[i] = EXTERNAL;

    nlp_solver_plan->nlp_cost[N] = EXTERNAL;

    for (int i = 0; i < N; i++)
    {
        nlp_solver_plan->nlp_dynamics[i] = CONTINUOUS_MODEL;
        nlp_solver_plan->sim_solver_plan[i].sim_solver = ERK;
    }

    nlp_solver_plan->nlp_constraints[0] = BGH;

    for (int i = 1; i < N; i++)
    {
        nlp_solver_plan->nlp_constraints[i] = BGH;
    }
    nlp_solver_plan->nlp_constraints[N] = BGH;

    nlp_solver_plan->regularization = CONVEXIFY;

    nlp_solver_plan->globalization = FIXED_STEP;
}


static ocp_nlp_dims* dynamics_acados_create_setup_dimensions(dynamics_solver_capsule* capsule)
{
    ocp_nlp_plan_t* nlp_solver_plan = capsule->nlp_solver_plan;
    const int N = nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;

    /************************************************
    *  dimensions
    ************************************************/
    #define NINTNP1MEMS 18
    int* intNp1mem = (int*)malloc( (N+1)*sizeof(int)*NINTNP1MEMS );

    int* nx    = intNp1mem + (N+1)*0;
    int* nu    = intNp1mem + (N+1)*1;
    int* nbx   = intNp1mem + (N+1)*2;
    int* nbu   = intNp1mem + (N+1)*3;
    int* nsbx  = intNp1mem + (N+1)*4;
    int* nsbu  = intNp1mem + (N+1)*5;
    int* nsg   = intNp1mem + (N+1)*6;
    int* nsh   = intNp1mem + (N+1)*7;
    int* nsphi = intNp1mem + (N+1)*8;
    int* ns    = intNp1mem + (N+1)*9;
    int* ng    = intNp1mem + (N+1)*10;
    int* nh    = intNp1mem + (N+1)*11;
    int* nphi  = intNp1mem + (N+1)*12;
    int* nz    = intNp1mem + (N+1)*13;
    int* ny    = intNp1mem + (N+1)*14;
    int* nr    = intNp1mem + (N+1)*15;
    int* nbxe  = intNp1mem + (N+1)*16;
    int* np  = intNp1mem + (N+1)*17;

    for (int i = 0; i < N+1; i++)
    {
        // common
        nx[i]     = NX;
        nu[i]     = NU;
        nz[i]     = NZ;
        ns[i]     = NS;
        // cost
        ny[i]     = NY;
        // constraints
        nbx[i]    = NBX;
        nbu[i]    = NBU;
        nsbx[i]   = NSBX;
        nsbu[i]   = NSBU;
        nsg[i]    = NSG;
        nsh[i]    = NSH;
        nsphi[i]  = NSPHI;
        ng[i]     = NG;
        nh[i]     = NH;
        nphi[i]   = NPHI;
        nr[i]     = NR;
        nbxe[i]   = 0;
        np[i]     = NP;
    }

    // for initial state
    nbx[0] = NBX0;
    nsbx[0] = 0;
    ns[0] = NS0;
    nbxe[0] = 12;
    ny[0] = NY0;
    nh[0] = NH0;
    nsh[0] = NSH0;
    nsphi[0] = NSPHI0;
    nphi[0] = NPHI0;


    // terminal - common
    nu[N]   = 0;
    nz[N]   = 0;
    ns[N]   = NSN;
    // cost
    ny[N]   = NYN;
    // constraint
    nbx[N]   = NBXN;
    nbu[N]   = 0;
    ng[N]    = NGN;
    nh[N]    = NHN;
    nphi[N]  = NPHIN;
    nr[N]    = 0;

    nsbx[N]  = NSBXN;
    nsbu[N]  = 0;
    nsg[N]   = NSGN;
    nsh[N]   = NSHN;
    nsphi[N] = NSPHIN;

    /* create and set ocp_nlp_dims */
    ocp_nlp_dims * nlp_dims = ocp_nlp_dims_create(nlp_config);

    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nx", nx);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nu", nu);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nz", nz);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "ns", ns);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "np", np);

    ocp_nlp_dims_set_global(nlp_config, nlp_dims, "np_global", 0);
    ocp_nlp_dims_set_global(nlp_config, nlp_dims, "n_global_data", 0);

    for (int i = 0; i <= N; i++)
    {
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbx", &nbx[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbu", &nbu[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsbx", &nsbx[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsbu", &nsbu[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "ng", &ng[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsg", &nsg[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbxe", &nbxe[i]);
    }
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, 0, "nh", &nh[0]);
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, 0, "nsh", &nsh[0]);

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nh", &nh[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsh", &nsh[i]);
    }
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, N, "nh", &nh[N]);
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, N, "nsh", &nsh[N]);

    free(intNp1mem);

    return nlp_dims;
}


/**
 * Internal function for dynamics_acados_create: step 3
 */
void dynamics_acados_create_setup_functions(dynamics_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;

    /************************************************
    *  external functions
    ************************************************/

#define MAP_CASADI_FNC(__CAPSULE_FNC__, __MODEL_BASE_FNC__) do{ \
        capsule->__CAPSULE_FNC__.casadi_fun = & __MODEL_BASE_FNC__ ;\
        capsule->__CAPSULE_FNC__.casadi_n_in = & __MODEL_BASE_FNC__ ## _n_in; \
        capsule->__CAPSULE_FNC__.casadi_n_out = & __MODEL_BASE_FNC__ ## _n_out; \
        capsule->__CAPSULE_FNC__.casadi_sparsity_in = & __MODEL_BASE_FNC__ ## _sparsity_in; \
        capsule->__CAPSULE_FNC__.casadi_sparsity_out = & __MODEL_BASE_FNC__ ## _sparsity_out; \
        capsule->__CAPSULE_FNC__.casadi_work = & __MODEL_BASE_FNC__ ## _work; \
        external_function_external_param_casadi_create(&capsule->__CAPSULE_FNC__, &ext_fun_opts); \
    } while(false)

    external_function_opts ext_fun_opts;
    external_function_opts_set_to_default(&ext_fun_opts);


    ext_fun_opts.external_workspace = true;
    // external cost
    MAP_CASADI_FNC(ext_cost_0_fun, dynamics_cost_ext_cost_0_fun);
    MAP_CASADI_FNC(ext_cost_0_fun_jac, dynamics_cost_ext_cost_0_fun_jac);
    MAP_CASADI_FNC(ext_cost_0_fun_jac_hess, dynamics_cost_ext_cost_0_fun_jac_hess);




    // explicit ode
    capsule->expl_vde_forw = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*N);
    for (int i = 0; i < N; i++) {
        MAP_CASADI_FNC(expl_vde_forw[i], dynamics_expl_vde_forw);
    }

    capsule->expl_ode_fun = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*N);
    for (int i = 0; i < N; i++) {
        MAP_CASADI_FNC(expl_ode_fun[i], dynamics_expl_ode_fun);
    }

    capsule->expl_vde_adj = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*N);
    for (int i = 0; i < N; i++) {
        MAP_CASADI_FNC(expl_vde_adj[i], dynamics_expl_vde_adj);
    }
    capsule->expl_ode_hess = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*N);
    for (int i = 0; i < N; i++) {
        MAP_CASADI_FNC(expl_ode_hess[i], dynamics_expl_ode_hess);
    }


    // external cost
    capsule->ext_cost_fun = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*(N-1));
    for (int i = 0; i < N-1; i++)
    {
        MAP_CASADI_FNC(ext_cost_fun[i], dynamics_cost_ext_cost_fun);
    }

    capsule->ext_cost_fun_jac = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*(N-1));
    for (int i = 0; i < N-1; i++)
    {
        MAP_CASADI_FNC(ext_cost_fun_jac[i], dynamics_cost_ext_cost_fun_jac);
    }

    capsule->ext_cost_fun_jac_hess = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*(N-1));
    for (int i = 0; i < N-1; i++)
    {
        MAP_CASADI_FNC(ext_cost_fun_jac_hess[i], dynamics_cost_ext_cost_fun_jac_hess);
    }

    

    
    // external cost - function
    MAP_CASADI_FNC(ext_cost_e_fun, dynamics_cost_ext_cost_e_fun);

    // external cost - jacobian
    MAP_CASADI_FNC(ext_cost_e_fun_jac, dynamics_cost_ext_cost_e_fun_jac);

    // external cost - hessian
    MAP_CASADI_FNC(ext_cost_e_fun_jac_hess, dynamics_cost_ext_cost_e_fun_jac_hess);

    // external cost - jacobian wrt params
    

    

#undef MAP_CASADI_FNC
}


/**
 * Internal function for dynamics_acados_create: step 4
 */
void dynamics_acados_create_set_default_parameters(dynamics_solver_capsule* capsule)
{

    const int N = capsule->nlp_solver_plan->N;
    // initialize parameters to nominal value
    double* p = calloc(NP, sizeof(double));
    p[0] = 1;
    p[1] = 1;
    p[2] = 1;

    for (int i = 0; i <= N; i++) {
        dynamics_acados_update_params(capsule, i, p, NP);
    }
    free(p);


    // no global parameters defined
}


/**
 * Internal function for dynamics_acados_create: step 5
 */
void dynamics_acados_setup_nlp_in(dynamics_solver_capsule* capsule, const int N, double* new_time_steps)
{
    assert(N == capsule->nlp_solver_plan->N);
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;

    int tmp_int = 0;

    /************************************************
    *  nlp_in
    ************************************************/
//    ocp_nlp_in * nlp_in = ocp_nlp_in_create(nlp_config, nlp_dims);
//    capsule->nlp_in = nlp_in;
    ocp_nlp_in * nlp_in = capsule->nlp_in;

    // set up time_steps and cost_scaling

    if (new_time_steps)
    {
        // NOTE: this sets scaling and time_steps
        dynamics_acados_update_time_steps(capsule, N, new_time_steps);
    }
    else
    {
        // set time_steps
    double time_step = 0.01;
        for (int i = 0; i < N; i++)
        {
            ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "Ts", &time_step);
        }
        // set cost scaling
        double* cost_scaling = malloc((N+1)*sizeof(double));
        cost_scaling[0] = 0.01;
        cost_scaling[1] = 0.01;
        cost_scaling[2] = 0.01;
        cost_scaling[3] = 0.01;
        cost_scaling[4] = 0.01;
        cost_scaling[5] = 0.01;
        cost_scaling[6] = 0.01;
        cost_scaling[7] = 0.01;
        cost_scaling[8] = 0.01;
        cost_scaling[9] = 0.01;
        cost_scaling[10] = 0.01;
        cost_scaling[11] = 0.01;
        cost_scaling[12] = 0.01;
        cost_scaling[13] = 0.01;
        cost_scaling[14] = 0.01;
        cost_scaling[15] = 0.01;
        cost_scaling[16] = 0.01;
        cost_scaling[17] = 0.01;
        cost_scaling[18] = 0.01;
        cost_scaling[19] = 0.01;
        cost_scaling[20] = 0.01;
        cost_scaling[21] = 0.01;
        cost_scaling[22] = 0.01;
        cost_scaling[23] = 0.01;
        cost_scaling[24] = 0.01;
        cost_scaling[25] = 0.01;
        cost_scaling[26] = 0.01;
        cost_scaling[27] = 0.01;
        cost_scaling[28] = 0.01;
        cost_scaling[29] = 0.01;
        cost_scaling[30] = 0.01;
        cost_scaling[31] = 0.01;
        cost_scaling[32] = 0.01;
        cost_scaling[33] = 0.01;
        cost_scaling[34] = 0.01;
        cost_scaling[35] = 0.01;
        cost_scaling[36] = 0.01;
        cost_scaling[37] = 0.01;
        cost_scaling[38] = 0.01;
        cost_scaling[39] = 0.01;
        cost_scaling[40] = 0.01;
        cost_scaling[41] = 0.01;
        cost_scaling[42] = 0.01;
        cost_scaling[43] = 0.01;
        cost_scaling[44] = 0.01;
        cost_scaling[45] = 0.01;
        cost_scaling[46] = 0.01;
        cost_scaling[47] = 0.01;
        cost_scaling[48] = 0.01;
        cost_scaling[49] = 0.01;
        cost_scaling[50] = 0.01;
        cost_scaling[51] = 0.01;
        cost_scaling[52] = 0.01;
        cost_scaling[53] = 0.01;
        cost_scaling[54] = 0.01;
        cost_scaling[55] = 0.01;
        cost_scaling[56] = 0.01;
        cost_scaling[57] = 0.01;
        cost_scaling[58] = 0.01;
        cost_scaling[59] = 0.01;
        cost_scaling[60] = 0.01;
        cost_scaling[61] = 0.01;
        cost_scaling[62] = 0.01;
        cost_scaling[63] = 0.01;
        cost_scaling[64] = 0.01;
        cost_scaling[65] = 0.01;
        cost_scaling[66] = 0.01;
        cost_scaling[67] = 0.01;
        cost_scaling[68] = 0.01;
        cost_scaling[69] = 0.01;
        cost_scaling[70] = 0.01;
        cost_scaling[71] = 0.01;
        cost_scaling[72] = 0.01;
        cost_scaling[73] = 0.01;
        cost_scaling[74] = 0.01;
        cost_scaling[75] = 0.01;
        cost_scaling[76] = 0.01;
        cost_scaling[77] = 0.01;
        cost_scaling[78] = 0.01;
        cost_scaling[79] = 0.01;
        cost_scaling[80] = 0.01;
        cost_scaling[81] = 0.01;
        cost_scaling[82] = 0.01;
        cost_scaling[83] = 0.01;
        cost_scaling[84] = 0.01;
        cost_scaling[85] = 0.01;
        cost_scaling[86] = 0.01;
        cost_scaling[87] = 0.01;
        cost_scaling[88] = 0.01;
        cost_scaling[89] = 0.01;
        cost_scaling[90] = 0.01;
        cost_scaling[91] = 0.01;
        cost_scaling[92] = 0.01;
        cost_scaling[93] = 0.01;
        cost_scaling[94] = 0.01;
        cost_scaling[95] = 0.01;
        cost_scaling[96] = 0.01;
        cost_scaling[97] = 0.01;
        cost_scaling[98] = 0.01;
        cost_scaling[99] = 0.01;
        cost_scaling[100] = 0.01;
        cost_scaling[101] = 0.01;
        cost_scaling[102] = 0.01;
        cost_scaling[103] = 0.01;
        cost_scaling[104] = 0.01;
        cost_scaling[105] = 0.01;
        cost_scaling[106] = 0.01;
        cost_scaling[107] = 0.01;
        cost_scaling[108] = 0.01;
        cost_scaling[109] = 0.01;
        cost_scaling[110] = 0.01;
        cost_scaling[111] = 0.01;
        cost_scaling[112] = 0.01;
        cost_scaling[113] = 0.01;
        cost_scaling[114] = 0.01;
        cost_scaling[115] = 0.01;
        cost_scaling[116] = 0.01;
        cost_scaling[117] = 0.01;
        cost_scaling[118] = 0.01;
        cost_scaling[119] = 0.01;
        cost_scaling[120] = 0.01;
        cost_scaling[121] = 0.01;
        cost_scaling[122] = 0.01;
        cost_scaling[123] = 0.01;
        cost_scaling[124] = 0.01;
        cost_scaling[125] = 0.01;
        cost_scaling[126] = 0.01;
        cost_scaling[127] = 0.01;
        cost_scaling[128] = 0.01;
        cost_scaling[129] = 0.01;
        cost_scaling[130] = 0.01;
        cost_scaling[131] = 0.01;
        cost_scaling[132] = 0.01;
        cost_scaling[133] = 0.01;
        cost_scaling[134] = 0.01;
        cost_scaling[135] = 0.01;
        cost_scaling[136] = 0.01;
        cost_scaling[137] = 0.01;
        cost_scaling[138] = 0.01;
        cost_scaling[139] = 0.01;
        cost_scaling[140] = 0.01;
        cost_scaling[141] = 0.01;
        cost_scaling[142] = 0.01;
        cost_scaling[143] = 0.01;
        cost_scaling[144] = 0.01;
        cost_scaling[145] = 0.01;
        cost_scaling[146] = 0.01;
        cost_scaling[147] = 0.01;
        cost_scaling[148] = 0.01;
        cost_scaling[149] = 0.01;
        cost_scaling[150] = 0.01;
        cost_scaling[151] = 0.01;
        cost_scaling[152] = 0.01;
        cost_scaling[153] = 0.01;
        cost_scaling[154] = 0.01;
        cost_scaling[155] = 0.01;
        cost_scaling[156] = 0.01;
        cost_scaling[157] = 0.01;
        cost_scaling[158] = 0.01;
        cost_scaling[159] = 0.01;
        cost_scaling[160] = 0.01;
        cost_scaling[161] = 0.01;
        cost_scaling[162] = 0.01;
        cost_scaling[163] = 0.01;
        cost_scaling[164] = 0.01;
        cost_scaling[165] = 0.01;
        cost_scaling[166] = 0.01;
        cost_scaling[167] = 0.01;
        cost_scaling[168] = 0.01;
        cost_scaling[169] = 0.01;
        cost_scaling[170] = 0.01;
        cost_scaling[171] = 0.01;
        cost_scaling[172] = 0.01;
        cost_scaling[173] = 0.01;
        cost_scaling[174] = 0.01;
        cost_scaling[175] = 0.01;
        cost_scaling[176] = 0.01;
        cost_scaling[177] = 0.01;
        cost_scaling[178] = 0.01;
        cost_scaling[179] = 0.01;
        cost_scaling[180] = 0.01;
        cost_scaling[181] = 0.01;
        cost_scaling[182] = 0.01;
        cost_scaling[183] = 0.01;
        cost_scaling[184] = 0.01;
        cost_scaling[185] = 0.01;
        cost_scaling[186] = 0.01;
        cost_scaling[187] = 0.01;
        cost_scaling[188] = 0.01;
        cost_scaling[189] = 0.01;
        cost_scaling[190] = 0.01;
        cost_scaling[191] = 0.01;
        cost_scaling[192] = 0.01;
        cost_scaling[193] = 0.01;
        cost_scaling[194] = 0.01;
        cost_scaling[195] = 0.01;
        cost_scaling[196] = 0.01;
        cost_scaling[197] = 0.01;
        cost_scaling[198] = 0.01;
        cost_scaling[199] = 0.01;
        cost_scaling[200] = 0.01;
        cost_scaling[201] = 0.01;
        cost_scaling[202] = 0.01;
        cost_scaling[203] = 0.01;
        cost_scaling[204] = 0.01;
        cost_scaling[205] = 0.01;
        cost_scaling[206] = 0.01;
        cost_scaling[207] = 0.01;
        cost_scaling[208] = 0.01;
        cost_scaling[209] = 0.01;
        cost_scaling[210] = 0.01;
        cost_scaling[211] = 0.01;
        cost_scaling[212] = 0.01;
        cost_scaling[213] = 0.01;
        cost_scaling[214] = 0.01;
        cost_scaling[215] = 0.01;
        cost_scaling[216] = 0.01;
        cost_scaling[217] = 0.01;
        cost_scaling[218] = 0.01;
        cost_scaling[219] = 0.01;
        cost_scaling[220] = 0.01;
        cost_scaling[221] = 0.01;
        cost_scaling[222] = 0.01;
        cost_scaling[223] = 0.01;
        cost_scaling[224] = 0.01;
        cost_scaling[225] = 0.01;
        cost_scaling[226] = 0.01;
        cost_scaling[227] = 0.01;
        cost_scaling[228] = 0.01;
        cost_scaling[229] = 0.01;
        cost_scaling[230] = 0.01;
        cost_scaling[231] = 0.01;
        cost_scaling[232] = 0.01;
        cost_scaling[233] = 0.01;
        cost_scaling[234] = 0.01;
        cost_scaling[235] = 0.01;
        cost_scaling[236] = 0.01;
        cost_scaling[237] = 0.01;
        cost_scaling[238] = 0.01;
        cost_scaling[239] = 0.01;
        cost_scaling[240] = 0.01;
        cost_scaling[241] = 0.01;
        cost_scaling[242] = 0.01;
        cost_scaling[243] = 0.01;
        cost_scaling[244] = 0.01;
        cost_scaling[245] = 0.01;
        cost_scaling[246] = 0.01;
        cost_scaling[247] = 0.01;
        cost_scaling[248] = 0.01;
        cost_scaling[249] = 0.01;
        cost_scaling[250] = 0.01;
        cost_scaling[251] = 0.01;
        cost_scaling[252] = 0.01;
        cost_scaling[253] = 0.01;
        cost_scaling[254] = 0.01;
        cost_scaling[255] = 0.01;
        cost_scaling[256] = 0.01;
        cost_scaling[257] = 0.01;
        cost_scaling[258] = 0.01;
        cost_scaling[259] = 0.01;
        cost_scaling[260] = 0.01;
        cost_scaling[261] = 0.01;
        cost_scaling[262] = 0.01;
        cost_scaling[263] = 0.01;
        cost_scaling[264] = 0.01;
        cost_scaling[265] = 0.01;
        cost_scaling[266] = 0.01;
        cost_scaling[267] = 0.01;
        cost_scaling[268] = 0.01;
        cost_scaling[269] = 0.01;
        cost_scaling[270] = 0.01;
        cost_scaling[271] = 0.01;
        cost_scaling[272] = 0.01;
        cost_scaling[273] = 0.01;
        cost_scaling[274] = 0.01;
        cost_scaling[275] = 0.01;
        cost_scaling[276] = 0.01;
        cost_scaling[277] = 0.01;
        cost_scaling[278] = 0.01;
        cost_scaling[279] = 0.01;
        cost_scaling[280] = 0.01;
        cost_scaling[281] = 0.01;
        cost_scaling[282] = 0.01;
        cost_scaling[283] = 0.01;
        cost_scaling[284] = 0.01;
        cost_scaling[285] = 0.01;
        cost_scaling[286] = 0.01;
        cost_scaling[287] = 0.01;
        cost_scaling[288] = 0.01;
        cost_scaling[289] = 0.01;
        cost_scaling[290] = 0.01;
        cost_scaling[291] = 0.01;
        cost_scaling[292] = 0.01;
        cost_scaling[293] = 0.01;
        cost_scaling[294] = 0.01;
        cost_scaling[295] = 0.01;
        cost_scaling[296] = 0.01;
        cost_scaling[297] = 0.01;
        cost_scaling[298] = 0.01;
        cost_scaling[299] = 0.01;
        cost_scaling[300] = 1;
        for (int i = 0; i <= N; i++)
        {
            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "scaling", &cost_scaling[i]);
        }
        free(cost_scaling);
    }


    /**** Dynamics ****/
    for (int i = 0; i < N; i++)
    {
        ocp_nlp_dynamics_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "expl_vde_forw", &capsule->expl_vde_forw[i]);
        ocp_nlp_dynamics_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "expl_ode_fun", &capsule->expl_ode_fun[i]);
        ocp_nlp_dynamics_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "expl_vde_adj", &capsule->expl_vde_adj[i]);
        ocp_nlp_dynamics_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "expl_ode_hess", &capsule->expl_ode_hess[i]);
    }

    /**** Cost ****/
    ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, 0, "ext_cost_fun", &capsule->ext_cost_0_fun);
    ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, 0, "ext_cost_fun_jac", &capsule->ext_cost_0_fun_jac);
    ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, 0, "ext_cost_fun_jac_hess", &capsule->ext_cost_0_fun_jac_hess);
    
    
    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "ext_cost_fun", &capsule->ext_cost_fun[i-1]);
        ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "ext_cost_fun_jac", &capsule->ext_cost_fun_jac[i-1]);
        ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "ext_cost_fun_jac_hess", &capsule->ext_cost_fun_jac_hess[i-1]);
        
        
    }
    ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, N, "ext_cost_fun", &capsule->ext_cost_e_fun);
    ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, N, "ext_cost_fun_jac", &capsule->ext_cost_e_fun_jac);
    ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, N, "ext_cost_fun_jac_hess", &capsule->ext_cost_e_fun_jac_hess);
    
    






    /**** Constraints ****/

    // bounds for initial stage
    // x0
    int* idxbx0 = malloc(NBX0 * sizeof(int));
    idxbx0[0] = 0;
    idxbx0[1] = 1;
    idxbx0[2] = 2;
    idxbx0[3] = 3;
    idxbx0[4] = 4;
    idxbx0[5] = 5;
    idxbx0[6] = 6;
    idxbx0[7] = 7;
    idxbx0[8] = 8;
    idxbx0[9] = 9;
    idxbx0[10] = 10;
    idxbx0[11] = 11;

    double* lubx0 = calloc(2*NBX0, sizeof(double));
    double* lbx0 = lubx0;
    double* ubx0 = lubx0 + NBX0;
    // change only the non-zero elements:

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);
    free(idxbx0);
    free(lubx0);
    // idxbxe_0
    int* idxbxe_0 = malloc(12 * sizeof(int));
    idxbxe_0[0] = 0;
    idxbxe_0[1] = 1;
    idxbxe_0[2] = 2;
    idxbxe_0[3] = 3;
    idxbxe_0[4] = 4;
    idxbxe_0[5] = 5;
    idxbxe_0[6] = 6;
    idxbxe_0[7] = 7;
    idxbxe_0[8] = 8;
    idxbxe_0[9] = 9;
    idxbxe_0[10] = 10;
    idxbxe_0[11] = 11;
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbxe", idxbxe_0);
    free(idxbxe_0);








    /* constraints that are the same for initial and intermediate */
    // u
    int* idxbu = malloc(NBU * sizeof(int));
    idxbu[0] = 0;
    idxbu[1] = 1;
    idxbu[2] = 2;
    idxbu[3] = 3;
    idxbu[4] = 4;
    idxbu[5] = 5;
    idxbu[6] = 6;
    idxbu[7] = 7;
    idxbu[8] = 8;
    idxbu[9] = 9;
    double* lubu = calloc(2*NBU, sizeof(double));
    double* lbu = lubu;
    double* ubu = lubu + NBU;
    lbu[0] = -1000;
    ubu[0] = 1000;
    lbu[1] = -1000;
    ubu[1] = 1000;
    lbu[2] = -1000;
    ubu[2] = 1000;
    lbu[3] = -1000;
    ubu[3] = 1000;
    lbu[4] = -1000;
    ubu[4] = 1000;
    lbu[5] = -1000;
    ubu[5] = 1000;
    lbu[6] = -1000;
    ubu[6] = 1000;
    lbu[7] = -1000;
    ubu[7] = 1000;
    lbu[8] = -1000;
    ubu[8] = 1000;
    lbu[9] = -1000;
    ubu[9] = 1000;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "idxbu", idxbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lbu", lbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ubu", ubu);
    }
    free(idxbu);
    free(lubu);















    /* terminal constraints */













}


static void dynamics_acados_create_set_opts(dynamics_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    void *nlp_opts = capsule->nlp_opts;

    /************************************************
    *  opts
    ************************************************/


    int nlp_solver_exact_hessian = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "exact_hess", &nlp_solver_exact_hessian);

    int exact_hess_dyn = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "exact_hess_dyn", &exact_hess_dyn);

    int exact_hess_cost = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "exact_hess_cost", &exact_hess_cost);

    int exact_hess_constr = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "exact_hess_constr", &exact_hess_constr);

    int fixed_hess = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "fixed_hess", &fixed_hess);

    double globalization_fixed_step_length = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "globalization_fixed_step_length", &globalization_fixed_step_length);




    int with_solution_sens_wrt_params = false;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "with_solution_sens_wrt_params", &with_solution_sens_wrt_params);

    int with_value_sens_wrt_params = false;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "with_value_sens_wrt_params", &with_value_sens_wrt_params);

    double solution_sens_qp_t_lam_min = 0.000000001;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "solution_sens_qp_t_lam_min", &solution_sens_qp_t_lam_min);

    int globalization_full_step_dual = 0;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "globalization_full_step_dual", &globalization_full_step_dual);

    // set collocation type (relevant for implicit integrators)
    sim_collocation_type collocation_type = GAUSS_LEGENDRE;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_collocation_type", &collocation_type);

    // set up sim_method_num_steps
    // all sim_method_num_steps are identical
    int sim_method_num_steps = 3;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_num_steps", &sim_method_num_steps);

    // set up sim_method_num_stages
    // all sim_method_num_stages are identical
    int sim_method_num_stages = 4;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_num_stages", &sim_method_num_stages);

    int newton_iter_val = 3;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_newton_iter", &newton_iter_val);

    double newton_tol_val = 0;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_newton_tol", &newton_tol_val);

    // set up sim_method_jac_reuse
    bool tmp_bool = (bool) 0;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_jac_reuse", &tmp_bool);

    double levenberg_marquardt = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "levenberg_marquardt", &levenberg_marquardt);

    /* options QP solver */
    int qp_solver_cond_N;const int qp_solver_cond_N_ori = 300;
    qp_solver_cond_N = N < qp_solver_cond_N_ori ? N : qp_solver_cond_N_ori; // use the minimum value here
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_cond_N", &qp_solver_cond_N);
    double reg_epsilon = 0.0001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "reg_epsilon", &reg_epsilon);

    int nlp_solver_ext_qp_res = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "ext_qp_res", &nlp_solver_ext_qp_res);

    bool store_iterates = false;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "store_iterates", &store_iterates);
    // set HPIPM mode: should be done before setting other QP solver options
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_hpipm_mode", "BALANCE");



    int qp_solver_t0_init = 2;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_t0_init", &qp_solver_t0_init);




    int as_rti_iter = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "as_rti_iter", &as_rti_iter);

    int as_rti_level = 4;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "as_rti_level", &as_rti_level);

    int rti_log_residuals = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "rti_log_residuals", &rti_log_residuals);

    int rti_log_only_available_residuals = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "rti_log_only_available_residuals", &rti_log_only_available_residuals);

    int qp_solver_iter_max = 25;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_iter_max", &qp_solver_iter_max);


    double qp_solver_tol_stat = 0.1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_tol_stat", &qp_solver_tol_stat);
    double qp_solver_tol_eq = 0.1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_tol_eq", &qp_solver_tol_eq);
    double qp_solver_tol_ineq = 0.1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_tol_ineq", &qp_solver_tol_ineq);
    double qp_solver_tol_comp = 0.1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_tol_comp", &qp_solver_tol_comp);
    int qp_solver_warm_start = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_warm_start", &qp_solver_warm_start);

    int print_level = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "print_level", &print_level);
    int qp_solver_cond_ric_alg = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_cond_ric_alg", &qp_solver_cond_ric_alg);

    int qp_solver_ric_alg = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_ric_alg", &qp_solver_ric_alg);


    int ext_cost_num_hess = 0;
    for (int i = 0; i < N; i++)
    {
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "cost_numerical_hessian", &ext_cost_num_hess);
    }
    ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, N, "cost_numerical_hessian", &ext_cost_num_hess);
}


/**
 * Internal function for dynamics_acados_create: step 7
 */
void dynamics_acados_set_nlp_out(dynamics_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;
    ocp_nlp_out* nlp_out = capsule->nlp_out;

    // initialize primal solution
    double* xu0 = calloc(NX+NU, sizeof(double));
    double* x0 = xu0;

    // initialize with x0


    double* u0 = xu0 + NX;

    for (int i = 0; i < N; i++)
    {
        // x0
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x0);
        // u0
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u0);
    }
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N, "x", x0);
    free(xu0);
}


/**
 * Internal function for dynamics_acados_create: step 9
 */
int dynamics_acados_create_precompute(dynamics_solver_capsule* capsule) {
    int status = ocp_nlp_precompute(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    if (status != ACADOS_SUCCESS) {
        printf("\nocp_nlp_precompute failed!\n\n");
        exit(1);
    }

    return status;
}


int dynamics_acados_create_with_discretization(dynamics_solver_capsule* capsule, int N, double* new_time_steps)
{
    // If N does not match the number of shooting intervals used for code generation, new_time_steps must be given.
    if (N != DYNAMICS_N && !new_time_steps) {
        fprintf(stderr, "dynamics_acados_create_with_discretization: new_time_steps is NULL " \
            "but the number of shooting intervals (= %d) differs from the number of " \
            "shooting intervals (= %d) during code generation! Please provide a new vector of time_stamps!\n", \
             N, DYNAMICS_N);
        return 1;
    }

    // number of expected runtime parameters
    capsule->nlp_np = NP;

    // 1) create and set nlp_solver_plan; create nlp_config
    capsule->nlp_solver_plan = ocp_nlp_plan_create(N);
    dynamics_acados_create_set_plan(capsule->nlp_solver_plan, N);
    capsule->nlp_config = ocp_nlp_config_create(*capsule->nlp_solver_plan);

    // 2) create and set dimensions
    capsule->nlp_dims = dynamics_acados_create_setup_dimensions(capsule);

    // 3) create and set nlp_opts
    capsule->nlp_opts = ocp_nlp_solver_opts_create(capsule->nlp_config, capsule->nlp_dims);
    dynamics_acados_create_set_opts(capsule);

    // 4) create nlp_in
    capsule->nlp_in = ocp_nlp_in_create(capsule->nlp_config, capsule->nlp_dims);

    // 5) setup functions, nlp_in and default parameters
    dynamics_acados_create_setup_functions(capsule);
    dynamics_acados_setup_nlp_in(capsule, N, new_time_steps);
    dynamics_acados_create_set_default_parameters(capsule);

    // 6) create solver
    capsule->nlp_solver = ocp_nlp_solver_create(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_opts, capsule->nlp_in);

    // 7) create and set nlp_out
    // 7.1) nlp_out
    capsule->nlp_out = ocp_nlp_out_create(capsule->nlp_config, capsule->nlp_dims);
    // 7.2) sens_out
    capsule->sens_out = ocp_nlp_out_create(capsule->nlp_config, capsule->nlp_dims);
    dynamics_acados_set_nlp_out(capsule);

    // 8) do precomputations
    int status = dynamics_acados_create_precompute(capsule);

    return status;
}

/**
 * This function is for updating an already initialized solver with a different number of qp_cond_N. It is useful for code reuse after code export.
 */
int dynamics_acados_update_qp_solver_cond_N(dynamics_solver_capsule* capsule, int qp_solver_cond_N)
{
    // 1) destroy solver
    ocp_nlp_solver_destroy(capsule->nlp_solver);

    // 2) set new value for "qp_cond_N"
    const int N = capsule->nlp_solver_plan->N;
    if(qp_solver_cond_N > N)
        printf("Warning: qp_solver_cond_N = %d > N = %d\n", qp_solver_cond_N, N);
    ocp_nlp_solver_opts_set(capsule->nlp_config, capsule->nlp_opts, "qp_cond_N", &qp_solver_cond_N);

    // 3) continue with the remaining steps from dynamics_acados_create_with_discretization(...):
    // -> 8) create solver
    capsule->nlp_solver = ocp_nlp_solver_create(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_opts, capsule->nlp_in);

    // -> 9) do precomputations
    int status = dynamics_acados_create_precompute(capsule);
    return status;
}


int dynamics_acados_reset(dynamics_solver_capsule* capsule, int reset_qp_solver_mem)
{

    // set initialization to all zeros

    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;
    ocp_nlp_out* nlp_out = capsule->nlp_out;
    ocp_nlp_in* nlp_in = capsule->nlp_in;
    ocp_nlp_solver* nlp_solver = capsule->nlp_solver;

    double* buffer = calloc(NX+NU+NZ+2*NS+2*NSN+2*NS0+NBX+NBU+NG+NH+NPHI+NBX0+NBXN+NHN+NH0+NPHIN+NGN, sizeof(double));

    for(int i=0; i<N+1; i++)
    {
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "sl", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "su", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "lam", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "z", buffer);
        if (i<N)
        {
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "pi", buffer);
        }
    }
    // get qp_status: if NaN -> reset memory
    int qp_status;
    ocp_nlp_get(capsule->nlp_solver, "qp_status", &qp_status);
    if (reset_qp_solver_mem || (qp_status == 3))
    {
        // printf("\nin reset qp_status %d -> resetting QP memory\n", qp_status);
        ocp_nlp_solver_reset_qp_memory(nlp_solver, nlp_in, nlp_out);
    }

    free(buffer);
    return 0;
}




int dynamics_acados_update_params(dynamics_solver_capsule* capsule, int stage, double *p, int np)
{
    int solver_status = 0;

    int casadi_np = 3;
    if (casadi_np != np) {
        printf("acados_update_params: trying to set %i parameters for external functions."
            " External function has %i parameters. Exiting.\n", np, casadi_np);
        exit(1);
    }
    ocp_nlp_in_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, stage, "parameter_values", p);

    return solver_status;
}


int dynamics_acados_update_params_sparse(dynamics_solver_capsule * capsule, int stage, int *idx, double *p, int n_update)
{
    ocp_nlp_in_set_params_sparse(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, stage, idx, p, n_update);

    return 0;
}


int dynamics_acados_set_p_global_and_precompute_dependencies(dynamics_solver_capsule* capsule, double* data, int data_len)
{

    // printf("No global_data, dynamics_acados_set_p_global_and_precompute_dependencies does nothing.\n");
    return 0;
}




int dynamics_acados_solve(dynamics_solver_capsule* capsule)
{
    // solve NLP
    int solver_status = ocp_nlp_solve(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    return solver_status;
}



int dynamics_acados_setup_qp_matrices_and_factorize(dynamics_solver_capsule* capsule)
{
    int solver_status = ocp_nlp_setup_qp_matrices_and_factorize(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    return solver_status;
}



void dynamics_acados_batch_solve(dynamics_solver_capsule ** capsules, int * status_out, int N_batch)
{

    int num_threads_bkp = omp_get_num_threads();
    omp_set_num_threads(4);

    #pragma omp parallel for
    for (int i = 0; i < N_batch; i++)
    {
        status_out[i] = ocp_nlp_solve(capsules[i]->nlp_solver, capsules[i]->nlp_in, capsules[i]->nlp_out);
    }


    omp_set_num_threads( num_threads_bkp );
    return;
}


void dynamics_acados_batch_setup_qp_matrices_and_factorize(dynamics_solver_capsule ** capsules, int * status_out, int N_batch)
{

    int num_threads_bkp = omp_get_num_threads();
    omp_set_num_threads(4);

    #pragma omp parallel for
    for (int i = 0; i < N_batch; i++)
    {
        status_out[i] = ocp_nlp_setup_qp_matrices_and_factorize(capsules[i]->nlp_solver, capsules[i]->nlp_in, capsules[i]->nlp_out);
    }


    omp_set_num_threads( num_threads_bkp );
    return;
}


void dynamics_acados_batch_eval_params_jac(dynamics_solver_capsule ** capsules, int N_batch)
{

    int num_threads_bkp = omp_get_num_threads();
    omp_set_num_threads(4);

    #pragma omp parallel for
    for (int i = 0; i < N_batch; i++)
    {
        ocp_nlp_eval_params_jac(capsules[i]->nlp_solver, capsules[i]->nlp_in, capsules[i]->nlp_out);
    }


    omp_set_num_threads( num_threads_bkp );
    return;
}



void dynamics_acados_batch_eval_solution_sens_adj_p(dynamics_solver_capsule ** capsules, const char *field, int stage, double *out, int offset, int N_batch)
{


    int num_threads_bkp = omp_get_num_threads();
    omp_set_num_threads(4);

    #pragma omp parallel for
    for (int i = 0; i < N_batch; i++)
    {
        ocp_nlp_eval_solution_sens_adj_p(capsules[i]->nlp_solver, capsules[i]->nlp_in, capsules[i]->sens_out, field, stage, out + i*offset);
    }


    omp_set_num_threads( num_threads_bkp );
    return;
}


void dynamics_acados_batch_set_flat(dynamics_solver_capsule ** capsules, const char *field, double *data, int N_data, int N_batch)
{
    int offset = ocp_nlp_dims_get_total_from_attr(capsules[0]->nlp_solver->config, capsules[0]->nlp_solver->dims, capsules[0]->nlp_out, field);

    if (N_batch*offset != N_data)
    {
        printf("batch_set_flat: wrong input dimension, expected %d, got %d\n", N_batch*offset, N_data);
        exit(1);
    }


    int num_threads_bkp = omp_get_num_threads();
    omp_set_num_threads(4);

    #pragma omp parallel for
    for (int i = 0; i < N_batch; i++)
    {
        ocp_nlp_set_all(capsules[i]->nlp_solver, capsules[i]->nlp_in, capsules[i]->nlp_out, field, data + i * offset);
    }


    omp_set_num_threads( num_threads_bkp );
    return;
}



void dynamics_acados_batch_get_flat(dynamics_solver_capsule ** capsules, const char *field, double *data, int N_data, int N_batch)
{
    int offset = ocp_nlp_dims_get_total_from_attr(capsules[0]->nlp_solver->config, capsules[0]->nlp_solver->dims, capsules[0]->nlp_out, field);

    if (N_batch*offset != N_data)
    {
        printf("batch_get_flat: wrong input dimension, expected %d, got %d\n", N_batch*offset, N_data);
        exit(1);
    }


    int num_threads_bkp = omp_get_num_threads();
    omp_set_num_threads(4);

    #pragma omp parallel for
    for (int i = 0; i < N_batch; i++)
    {
        ocp_nlp_get_all(capsules[i]->nlp_solver, capsules[i]->nlp_in, capsules[i]->nlp_out, field, data + i * offset);
    }


    omp_set_num_threads( num_threads_bkp );
    return;
}


int dynamics_acados_free(dynamics_solver_capsule* capsule)
{
    // before destroying, keep some info
    const int N = capsule->nlp_solver_plan->N;
    // free memory
    ocp_nlp_solver_opts_destroy(capsule->nlp_opts);
    ocp_nlp_in_destroy(capsule->nlp_in);
    ocp_nlp_out_destroy(capsule->nlp_out);
    ocp_nlp_out_destroy(capsule->sens_out);
    ocp_nlp_solver_destroy(capsule->nlp_solver);
    ocp_nlp_dims_destroy(capsule->nlp_dims);
    ocp_nlp_config_destroy(capsule->nlp_config);
    ocp_nlp_plan_destroy(capsule->nlp_solver_plan);

    /* free external function */
    // dynamics
    for (int i = 0; i < N; i++)
    {
        external_function_external_param_casadi_free(&capsule->expl_vde_forw[i]);
        external_function_external_param_casadi_free(&capsule->expl_ode_fun[i]);
        external_function_external_param_casadi_free(&capsule->expl_vde_adj[i]);
        external_function_external_param_casadi_free(&capsule->expl_ode_hess[i]);
    }
    free(capsule->expl_vde_adj);
    free(capsule->expl_vde_forw);
    free(capsule->expl_ode_fun);
    free(capsule->expl_ode_hess);

    // cost
    external_function_external_param_casadi_free(&capsule->ext_cost_0_fun);
    external_function_external_param_casadi_free(&capsule->ext_cost_0_fun_jac);
    external_function_external_param_casadi_free(&capsule->ext_cost_0_fun_jac_hess);
    
    
    for (int i = 0; i < N - 1; i++)
    {
        external_function_external_param_casadi_free(&capsule->ext_cost_fun[i]);
        external_function_external_param_casadi_free(&capsule->ext_cost_fun_jac[i]);
        external_function_external_param_casadi_free(&capsule->ext_cost_fun_jac_hess[i]);
        
        
    }
    free(capsule->ext_cost_fun);
    free(capsule->ext_cost_fun_jac);
    free(capsule->ext_cost_fun_jac_hess);
    external_function_external_param_casadi_free(&capsule->ext_cost_e_fun);
    external_function_external_param_casadi_free(&capsule->ext_cost_e_fun_jac);
    external_function_external_param_casadi_free(&capsule->ext_cost_e_fun_jac_hess);
    
    

    // constraints



    return 0;
}


void dynamics_acados_print_stats(dynamics_solver_capsule* capsule)
{
    int nlp_iter, stat_m, stat_n, tmp_int;
    ocp_nlp_get(capsule->nlp_solver, "nlp_iter", &nlp_iter);
    ocp_nlp_get(capsule->nlp_solver, "stat_n", &stat_n);
    ocp_nlp_get(capsule->nlp_solver, "stat_m", &stat_m);


    double stat[1200];
    ocp_nlp_get(capsule->nlp_solver, "statistics", stat);

    int nrow = nlp_iter+1 < stat_m ? nlp_iter+1 : stat_m;


    printf("iter\tqp_stat\tqp_iter\n");
    for (int i = 0; i < nrow; i++)
    {
        for (int j = 0; j < stat_n + 1; j++)
        {
            tmp_int = (int) stat[i + j * nrow];
            printf("%d\t", tmp_int);
        }
        printf("\n");
    }
}

int dynamics_acados_custom_update(dynamics_solver_capsule* capsule, double* data, int data_len)
{
    (void)capsule;
    (void)data;
    (void)data_len;
    printf("\ndummy function that can be called in between solver calls to update parameters or numerical data efficiently in C.\n");
    printf("nothing set yet..\n");
    return 1;

}



ocp_nlp_in *dynamics_acados_get_nlp_in(dynamics_solver_capsule* capsule) { return capsule->nlp_in; }
ocp_nlp_out *dynamics_acados_get_nlp_out(dynamics_solver_capsule* capsule) { return capsule->nlp_out; }
ocp_nlp_out *dynamics_acados_get_sens_out(dynamics_solver_capsule* capsule) { return capsule->sens_out; }
ocp_nlp_solver *dynamics_acados_get_nlp_solver(dynamics_solver_capsule* capsule) { return capsule->nlp_solver; }
ocp_nlp_config *dynamics_acados_get_nlp_config(dynamics_solver_capsule* capsule) { return capsule->nlp_config; }
void *dynamics_acados_get_nlp_opts(dynamics_solver_capsule* capsule) { return capsule->nlp_opts; }
ocp_nlp_dims *dynamics_acados_get_nlp_dims(dynamics_solver_capsule* capsule) { return capsule->nlp_dims; }
ocp_nlp_plan_t *dynamics_acados_get_nlp_plan(dynamics_solver_capsule* capsule) { return capsule->nlp_solver_plan; }
