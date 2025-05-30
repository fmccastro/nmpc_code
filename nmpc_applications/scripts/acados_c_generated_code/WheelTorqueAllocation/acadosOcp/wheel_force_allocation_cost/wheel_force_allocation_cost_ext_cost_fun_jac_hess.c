/* This file was automatically generated by CasADi 3.6.7.
 *  It consists of: 
 *   1) content generated by CasADi runtime: not copyrighted
 *   2) template code copied from CasADi source: permissively licensed (MIT-0)
 *   3) user code: owned by the user
 *
 */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) wheel_force_allocation_cost_ext_cost_fun_jac_hess_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)
#define casadi_s5 CASADI_PREFIX(s5)
#define casadi_s6 CASADI_PREFIX(s6)
#define casadi_s7 CASADI_PREFIX(s7)
#define casadi_sq CASADI_PREFIX(sq)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

casadi_real casadi_sq(casadi_real x) { return x*x;}

static const casadi_int casadi_s0[8] = {4, 1, 0, 4, 0, 1, 2, 3};
static const casadi_int casadi_s1[10] = {6, 1, 0, 6, 0, 1, 2, 3, 4, 5};
static const casadi_int casadi_s2[4] = {0, 1, 0, 0};
static const casadi_int casadi_s3[14] = {10, 1, 0, 10, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
static const casadi_int casadi_s4[5] = {1, 1, 0, 1, 0};
static const casadi_int casadi_s5[79] = {10, 10, 0, 8, 16, 24, 32, 33, 34, 42, 50, 58, 66, 0, 1, 2, 3, 6, 7, 8, 9, 0, 1, 2, 3, 6, 7, 8, 9, 0, 1, 2, 3, 6, 7, 8, 9, 0, 1, 2, 3, 6, 7, 8, 9, 4, 5, 0, 1, 2, 3, 6, 7, 8, 9, 0, 1, 2, 3, 6, 7, 8, 9, 0, 1, 2, 3, 6, 7, 8, 9, 0, 1, 2, 3, 6, 7, 8, 9};
static const casadi_int casadi_s6[3] = {0, 0, 0};
static const casadi_int casadi_s7[13] = {0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/* wheel_force_allocation_cost_ext_cost_fun_jac_hess:(i0[4],i1[6],i2[0],i3[10])->(o0,o1[10],o2[10x10,66nz],o3[],o4[0x10]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a5, a50, a51, a52, a53, a6, a7, a8, a9;
  a0=2.;
  a1=7.3216599999999998e-01;
  a2=arg[1]? arg[1][0] : 0;
  a3=(a1*a2);
  a4=100.;
  a5=1.1100000000000000e-01;
  a6=arg[0]? arg[0][0] : 0;
  a6=(a5*a6);
  a7=arg[3]? arg[3][2] : 0;
  a8=-5.2573043795620514e-02;
  a9=arg[3]? arg[3][6] : 0;
  a8=(a8*a9);
  a9=1.9700000000000001e-01;
  a10=arg[3]? arg[3][7] : 0;
  a11=(a9*a10);
  a11=(a8-a11);
  a11=(a7+a11);
  a6=(a6-a11);
  a6=(a4*a6);
  a6=tanh(a6);
  a12=(a3*a6);
  a13=2.7399999999999999e+01;
  a14=-9.8100000000000005e+00;
  a15=arg[3]? arg[3][1] : 0;
  a16=sin(a15);
  a16=(a14*a16);
  a16=(a13*a16);
  a16=(a12-a16);
  a17=arg[1]? arg[1][1] : 0;
  a18=(a1*a17);
  a19=arg[0]? arg[0][1] : 0;
  a19=(a5*a19);
  a19=(a19-a11);
  a19=(a4*a19);
  a19=tanh(a19);
  a11=(a18*a19);
  a16=(a16+a11);
  a20=arg[1]? arg[1][2] : 0;
  a21=(a1*a20);
  a22=arg[0]? arg[0][2] : 0;
  a22=(a5*a22);
  a23=-1.9700000000000001e-01;
  a10=(a23*a10);
  a8=(a8-a10);
  a7=(a7+a8);
  a22=(a22-a7);
  a22=(a4*a22);
  a22=tanh(a22);
  a8=(a21*a22);
  a16=(a16+a8);
  a10=arg[1]? arg[1][3] : 0;
  a24=(a1*a10);
  a25=arg[0]? arg[0][3] : 0;
  a25=(a5*a25);
  a25=(a25-a7);
  a25=(a4*a25);
  a25=tanh(a25);
  a7=(a24*a25);
  a16=(a16+a7);
  a26=arg[3]? arg[3][8] : 0;
  a16=(a16-a26);
  a26=(a0*a16);
  a16=(a26*a16);
  a15=cos(a15);
  a27=arg[3]? arg[3][0] : 0;
  a28=sin(a27);
  a28=(a15*a28);
  a28=(a14*a28);
  a28=(a13*a28);
  a29=(a0*a28);
  a29=(a29*a28);
  a16=(a16+a29);
  a27=cos(a27);
  a15=(a15*a27);
  a14=(a14*a15);
  a13=(a13*a14);
  a13=(a13+a2);
  a13=(a13+a17);
  a13=(a13+a20);
  a13=(a13+a10);
  a10=(a0*a13);
  a13=(a10*a13);
  a16=(a16+a13);
  a12=(a23*a12);
  a11=(a23*a11);
  a12=(a12+a11);
  a8=(a9*a8);
  a12=(a12+a8);
  a7=(a9*a7);
  a12=(a12+a7);
  a7=arg[3]? arg[3][9] : 0;
  a12=(a12-a7);
  a7=(a0*a12);
  a12=(a7*a12);
  a16=(a16+a12);
  a12=10.;
  a8=arg[1]? arg[1][4] : 0;
  a11=(a12*a8);
  a8=(a11*a8);
  a16=(a16+a8);
  a8=arg[1]? arg[1][5] : 0;
  a12=(a12*a8);
  a8=(a12*a8);
  a16=(a16+a8);
  if (res[0]!=0) res[0][0]=a16;
  a10=(a10+a10);
  a7=(a7+a7);
  a16=(a23*a7);
  a26=(a26+a26);
  a16=(a16+a26);
  a8=(a6*a16);
  a8=(a1*a8);
  a8=(a10+a8);
  if (res[1]!=0) res[1][0]=a8;
  a8=(a19*a16);
  a8=(a1*a8);
  a8=(a10+a8);
  if (res[1]!=0) res[1][1]=a8;
  a7=(a9*a7);
  a7=(a7+a26);
  a26=(a22*a7);
  a26=(a1*a26);
  a26=(a10+a26);
  if (res[1]!=0) res[1][2]=a26;
  a26=(a25*a7);
  a26=(a1*a26);
  a10=(a10+a26);
  if (res[1]!=0) res[1][3]=a10;
  a11=(a11+a11);
  if (res[1]!=0) res[1][4]=a11;
  a12=(a12+a12);
  if (res[1]!=0) res[1][5]=a12;
  a12=1.;
  a11=casadi_sq(a6);
  a11=(a12-a11);
  a10=(a3*a16);
  a26=(a11*a10);
  a26=(a4*a26);
  a26=(a5*a26);
  if (res[1]!=0) res[1][6]=a26;
  a26=casadi_sq(a19);
  a26=(a12-a26);
  a8=(a18*a16);
  a13=(a26*a8);
  a13=(a4*a13);
  a13=(a5*a13);
  if (res[1]!=0) res[1][7]=a13;
  a13=casadi_sq(a22);
  a13=(a12-a13);
  a20=(a21*a7);
  a17=(a13*a20);
  a17=(a4*a17);
  a17=(a5*a17);
  if (res[1]!=0) res[1][8]=a17;
  a17=casadi_sq(a25);
  a12=(a12-a17);
  a17=(a24*a7);
  a2=(a12*a17);
  a2=(a4*a2);
  a2=(a5*a2);
  if (res[1]!=0) res[1][9]=a2;
  a2=4.;
  a14=(a1*a6);
  a15=(a23*a14);
  a15=(a0*a15);
  a15=(a15+a15);
  a15=(a23*a15);
  a14=(a0*a14);
  a14=(a14+a14);
  a15=(a15+a14);
  a15=(a6*a15);
  a15=(a1*a15);
  a15=(a2+a15);
  if (res[2]!=0) res[2][0]=a15;
  a15=(a1*a19);
  a14=(a23*a15);
  a14=(a0*a14);
  a14=(a14+a14);
  a14=(a23*a14);
  a15=(a0*a15);
  a15=(a15+a15);
  a14=(a14+a15);
  a15=(a6*a14);
  a15=(a1*a15);
  a15=(a2+a15);
  if (res[2]!=0) res[2][1]=a15;
  a27=(a1*a22);
  a29=(a9*a27);
  a29=(a0*a29);
  a29=(a29+a29);
  a28=(a23*a29);
  a27=(a0*a27);
  a27=(a27+a27);
  a28=(a28+a27);
  a30=(a6*a28);
  a30=(a1*a30);
  a30=(a2+a30);
  if (res[2]!=0) res[2][2]=a30;
  a31=(a1*a25);
  a32=(a9*a31);
  a32=(a0*a32);
  a32=(a32+a32);
  a33=(a23*a32);
  a31=(a0*a31);
  a31=(a31+a31);
  a33=(a33+a31);
  a34=(a6*a33);
  a34=(a1*a34);
  a34=(a2+a34);
  if (res[2]!=0) res[2][3]=a34;
  a35=1.1100000000000000e+01;
  a36=(a35*a11);
  a37=(a16*a36);
  a38=(a3*a36);
  a39=(a23*a38);
  a39=(a0*a39);
  a39=(a39+a39);
  a40=(a23*a39);
  a38=(a0*a38);
  a38=(a38+a38);
  a40=(a40+a38);
  a41=(a6*a40);
  a37=(a37+a41);
  a37=(a1*a37);
  if (res[2]!=0) res[2][4]=a37;
  a41=(a35*a26);
  a42=(a18*a41);
  a43=(a23*a42);
  a43=(a0*a43);
  a43=(a43+a43);
  a44=(a23*a43);
  a42=(a0*a42);
  a42=(a42+a42);
  a44=(a44+a42);
  a45=(a6*a44);
  a45=(a1*a45);
  if (res[2]!=0) res[2][5]=a45;
  a46=(a35*a13);
  a47=(a21*a46);
  a48=(a9*a47);
  a48=(a0*a48);
  a48=(a48+a48);
  a49=(a23*a48);
  a47=(a0*a47);
  a47=(a47+a47);
  a49=(a49+a47);
  a50=(a6*a49);
  a50=(a1*a50);
  if (res[2]!=0) res[2][6]=a50;
  a35=(a35*a12);
  a51=(a24*a35);
  a52=(a9*a51);
  a52=(a0*a52);
  a52=(a52+a52);
  a23=(a23*a52);
  a0=(a0*a51);
  a0=(a0+a0);
  a23=(a23+a0);
  a51=(a6*a23);
  a51=(a1*a51);
  if (res[2]!=0) res[2][7]=a51;
  if (res[2]!=0) res[2][8]=a15;
  a14=(a19*a14);
  a14=(a1*a14);
  a14=(a2+a14);
  if (res[2]!=0) res[2][9]=a14;
  a28=(a19*a28);
  a28=(a1*a28);
  a28=(a2+a28);
  if (res[2]!=0) res[2][10]=a28;
  a33=(a19*a33);
  a33=(a1*a33);
  a33=(a2+a33);
  if (res[2]!=0) res[2][11]=a33;
  a14=(a19*a40);
  a14=(a1*a14);
  if (res[2]!=0) res[2][12]=a14;
  a16=(a16*a41);
  a15=(a19*a44);
  a16=(a16+a15);
  a16=(a1*a16);
  if (res[2]!=0) res[2][13]=a16;
  a15=(a19*a49);
  a15=(a1*a15);
  if (res[2]!=0) res[2][14]=a15;
  a53=(a19*a23);
  a53=(a1*a53);
  if (res[2]!=0) res[2][15]=a53;
  if (res[2]!=0) res[2][16]=a30;
  if (res[2]!=0) res[2][17]=a28;
  a29=(a9*a29);
  a29=(a29+a27);
  a29=(a22*a29);
  a29=(a1*a29);
  a29=(a2+a29);
  if (res[2]!=0) res[2][18]=a29;
  a32=(a9*a32);
  a32=(a32+a31);
  a31=(a22*a32);
  a31=(a1*a31);
  a31=(a2+a31);
  if (res[2]!=0) res[2][19]=a31;
  a39=(a9*a39);
  a39=(a39+a38);
  a38=(a22*a39);
  a38=(a1*a38);
  if (res[2]!=0) res[2][20]=a38;
  a43=(a9*a43);
  a43=(a43+a42);
  a42=(a22*a43);
  a42=(a1*a42);
  if (res[2]!=0) res[2][21]=a42;
  a29=(a7*a46);
  a48=(a9*a48);
  a48=(a48+a47);
  a47=(a22*a48);
  a29=(a29+a47);
  a29=(a1*a29);
  if (res[2]!=0) res[2][22]=a29;
  a9=(a9*a52);
  a9=(a9+a0);
  a0=(a22*a9);
  a0=(a1*a0);
  if (res[2]!=0) res[2][23]=a0;
  if (res[2]!=0) res[2][24]=a34;
  if (res[2]!=0) res[2][25]=a33;
  if (res[2]!=0) res[2][26]=a31;
  a32=(a25*a32);
  a32=(a1*a32);
  a2=(a2+a32);
  if (res[2]!=0) res[2][27]=a2;
  a39=(a25*a39);
  a39=(a1*a39);
  if (res[2]!=0) res[2][28]=a39;
  a43=(a25*a43);
  a43=(a1*a43);
  if (res[2]!=0) res[2][29]=a43;
  a2=(a25*a48);
  a2=(a1*a2);
  if (res[2]!=0) res[2][30]=a2;
  a7=(a7*a35);
  a32=(a25*a9);
  a7=(a7+a32);
  a1=(a1*a7);
  if (res[2]!=0) res[2][31]=a1;
  a7=20.;
  if (res[2]!=0) res[2][32]=a7;
  if (res[2]!=0) res[2][33]=a7;
  if (res[2]!=0) res[2][34]=a37;
  if (res[2]!=0) res[2][35]=a14;
  if (res[2]!=0) res[2][36]=a38;
  if (res[2]!=0) res[2][37]=a39;
  a40=(a3*a40);
  a40=(a11*a40);
  a6=(a6+a6);
  a6=(a6*a36);
  a10=(a10*a6);
  a40=(a40-a10);
  a40=(a4*a40);
  a40=(a5*a40);
  if (res[2]!=0) res[2][38]=a40;
  a40=(a3*a44);
  a40=(a11*a40);
  a40=(a4*a40);
  a40=(a5*a40);
  if (res[2]!=0) res[2][39]=a40;
  a10=(a3*a49);
  a10=(a11*a10);
  a10=(a4*a10);
  a10=(a5*a10);
  if (res[2]!=0) res[2][40]=a10;
  a3=(a3*a23);
  a11=(a11*a3);
  a11=(a4*a11);
  a11=(a5*a11);
  if (res[2]!=0) res[2][41]=a11;
  if (res[2]!=0) res[2][42]=a45;
  if (res[2]!=0) res[2][43]=a16;
  if (res[2]!=0) res[2][44]=a42;
  if (res[2]!=0) res[2][45]=a43;
  if (res[2]!=0) res[2][46]=a40;
  a44=(a18*a44);
  a44=(a26*a44);
  a19=(a19+a19);
  a19=(a19*a41);
  a8=(a8*a19);
  a44=(a44-a8);
  a44=(a4*a44);
  a44=(a5*a44);
  if (res[2]!=0) res[2][47]=a44;
  a49=(a18*a49);
  a49=(a26*a49);
  a49=(a4*a49);
  a49=(a5*a49);
  if (res[2]!=0) res[2][48]=a49;
  a18=(a18*a23);
  a26=(a26*a18);
  a26=(a4*a26);
  a26=(a5*a26);
  if (res[2]!=0) res[2][49]=a26;
  if (res[2]!=0) res[2][50]=a50;
  if (res[2]!=0) res[2][51]=a15;
  if (res[2]!=0) res[2][52]=a29;
  if (res[2]!=0) res[2][53]=a2;
  if (res[2]!=0) res[2][54]=a10;
  if (res[2]!=0) res[2][55]=a49;
  a48=(a21*a48);
  a48=(a13*a48);
  a22=(a22+a22);
  a22=(a22*a46);
  a20=(a20*a22);
  a48=(a48-a20);
  a48=(a4*a48);
  a48=(a5*a48);
  if (res[2]!=0) res[2][56]=a48;
  a21=(a21*a9);
  a13=(a13*a21);
  a13=(a4*a13);
  a13=(a5*a13);
  if (res[2]!=0) res[2][57]=a13;
  if (res[2]!=0) res[2][58]=a51;
  if (res[2]!=0) res[2][59]=a53;
  if (res[2]!=0) res[2][60]=a0;
  if (res[2]!=0) res[2][61]=a1;
  if (res[2]!=0) res[2][62]=a11;
  if (res[2]!=0) res[2][63]=a26;
  if (res[2]!=0) res[2][64]=a13;
  a24=(a24*a9);
  a12=(a12*a24);
  a25=(a25+a25);
  a25=(a25*a35);
  a17=(a17*a25);
  a12=(a12-a17);
  a4=(a4*a12);
  a5=(a5*a4);
  if (res[2]!=0) res[2][65]=a5;
  return 0;
}

CASADI_SYMBOL_EXPORT int wheel_force_allocation_cost_ext_cost_fun_jac_hess(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int wheel_force_allocation_cost_ext_cost_fun_jac_hess_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int wheel_force_allocation_cost_ext_cost_fun_jac_hess_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void wheel_force_allocation_cost_ext_cost_fun_jac_hess_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int wheel_force_allocation_cost_ext_cost_fun_jac_hess_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void wheel_force_allocation_cost_ext_cost_fun_jac_hess_release(int mem) {
}

CASADI_SYMBOL_EXPORT void wheel_force_allocation_cost_ext_cost_fun_jac_hess_incref(void) {
}

CASADI_SYMBOL_EXPORT void wheel_force_allocation_cost_ext_cost_fun_jac_hess_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int wheel_force_allocation_cost_ext_cost_fun_jac_hess_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int wheel_force_allocation_cost_ext_cost_fun_jac_hess_n_out(void) { return 5;}

CASADI_SYMBOL_EXPORT casadi_real wheel_force_allocation_cost_ext_cost_fun_jac_hess_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* wheel_force_allocation_cost_ext_cost_fun_jac_hess_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* wheel_force_allocation_cost_ext_cost_fun_jac_hess_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    case 3: return "o3";
    case 4: return "o4";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* wheel_force_allocation_cost_ext_cost_fun_jac_hess_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* wheel_force_allocation_cost_ext_cost_fun_jac_hess_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s4;
    case 1: return casadi_s3;
    case 2: return casadi_s5;
    case 3: return casadi_s6;
    case 4: return casadi_s7;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int wheel_force_allocation_cost_ext_cost_fun_jac_hess_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 4;
  if (sz_res) *sz_res = 5;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

CASADI_SYMBOL_EXPORT int wheel_force_allocation_cost_ext_cost_fun_jac_hess_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 4*sizeof(const casadi_real*);
  if (sz_res) *sz_res = 5*sizeof(casadi_real*);
  if (sz_iw) *sz_iw = 0*sizeof(casadi_int);
  if (sz_w) *sz_w = 0*sizeof(casadi_real);
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
