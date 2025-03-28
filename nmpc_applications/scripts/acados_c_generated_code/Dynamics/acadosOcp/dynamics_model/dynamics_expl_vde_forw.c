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
  #define CASADI_PREFIX(ID) dynamics_expl_vde_forw_ ## ID
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

static const casadi_int casadi_s0[10] = {6, 1, 0, 6, 0, 1, 2, 3, 4, 5};
static const casadi_int casadi_s1[45] = {6, 6, 0, 6, 12, 18, 24, 30, 36, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5};
static const casadi_int casadi_s2[17] = {6, 2, 0, 6, 12, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5};
static const casadi_int casadi_s3[6] = {2, 1, 0, 2, 0, 1};
static const casadi_int casadi_s4[8] = {4, 1, 0, 4, 0, 1, 2, 3};

/* dynamics_expl_vde_forw:(i0[6],i1[6x6],i2[6x2],i3[2],i4[4])->(o0[6],o1[6x6],o2[6x2]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a3, a4, a5, a6, a7, a8, a9;
  a0=arg[3]? arg[3][0] : 0;
  a1=2.7399999999999999e+01;
  a2=-9.8100000000000005e+00;
  a3=arg[4]? arg[4][1] : 0;
  a3=sin(a3);
  a2=(a2*a3);
  a2=(a1*a2);
  a0=(a0-a2);
  a0=(a0/a1);
  a1=arg[0]? arg[0][4] : 0;
  a2=arg[0]? arg[0][2] : 0;
  a3=(a1*a2);
  a4=arg[0]? arg[0][5] : 0;
  a5=arg[0]? arg[0][1] : 0;
  a6=(a4*a5);
  a3=(a3-a6);
  a0=(a0-a3);
  if (res[0]!=0) res[0][0]=a0;
  a0=arg[0]? arg[0][0] : 0;
  a3=(a4*a0);
  a6=arg[0]? arg[0][3] : 0;
  a7=(a6*a2);
  a3=(a3-a7);
  a3=(-a3);
  if (res[0]!=0) res[0][1]=a3;
  a3=(a6*a5);
  a7=(a1*a0);
  a3=(a3-a7);
  a3=(-a3);
  if (res[0]!=0) res[0][2]=a3;
  a3=8.6080632770563145e-02;
  a7=1.1700973610735778e+01;
  a8=(a7*a4);
  a9=(a1*a8);
  a10=1.1669791786280525e+01;
  a11=(a10*a1);
  a12=(a4*a11);
  a9=(a9-a12);
  a9=(a3*a9);
  a9=(-a9);
  if (res[0]!=0) res[0][3]=a9;
  a9=8.5691331800421669e-02;
  a12=1.1617014975544746e+01;
  a13=(a12*a6);
  a14=(a4*a13);
  a15=(a6*a8);
  a14=(a14-a15);
  a14=(a9*a14);
  a14=(-a14);
  if (res[0]!=0) res[0][4]=a14;
  a14=8.5462973703529135e-02;
  a15=arg[3]? arg[3][1] : 0;
  a16=(a6*a11);
  a17=(a1*a13);
  a16=(a16-a17);
  a15=(a15-a16);
  a15=(a14*a15);
  if (res[0]!=0) res[0][5]=a15;
  a15=arg[1]? arg[1][4] : 0;
  a16=(a2*a15);
  a17=arg[1]? arg[1][2] : 0;
  a18=(a1*a17);
  a16=(a16+a18);
  a18=arg[1]? arg[1][5] : 0;
  a19=(a5*a18);
  a20=arg[1]? arg[1][1] : 0;
  a21=(a4*a20);
  a19=(a19+a21);
  a16=(a16-a19);
  a16=(-a16);
  if (res[1]!=0) res[1][0]=a16;
  a16=(a0*a18);
  a19=arg[1]? arg[1][0] : 0;
  a21=(a4*a19);
  a16=(a16+a21);
  a21=arg[1]? arg[1][3] : 0;
  a22=(a2*a21);
  a17=(a6*a17);
  a22=(a22+a17);
  a16=(a16-a22);
  a16=(-a16);
  if (res[1]!=0) res[1][1]=a16;
  a16=(a5*a21);
  a20=(a6*a20);
  a16=(a16+a20);
  a20=(a0*a15);
  a19=(a1*a19);
  a20=(a20+a19);
  a16=(a16-a20);
  a16=(-a16);
  if (res[1]!=0) res[1][2]=a16;
  a16=(a8*a15);
  a20=(a7*a18);
  a19=(a1*a20);
  a16=(a16+a19);
  a19=(a11*a18);
  a22=(a10*a15);
  a17=(a4*a22);
  a19=(a19+a17);
  a16=(a16-a19);
  a16=(a3*a16);
  a16=(-a16);
  if (res[1]!=0) res[1][3]=a16;
  a18=(a13*a18);
  a16=(a12*a21);
  a19=(a4*a16);
  a18=(a18+a19);
  a19=(a8*a21);
  a20=(a6*a20);
  a19=(a19+a20);
  a18=(a18-a19);
  a18=(a9*a18);
  a18=(-a18);
  if (res[1]!=0) res[1][4]=a18;
  a21=(a11*a21);
  a22=(a6*a22);
  a21=(a21+a22);
  a15=(a13*a15);
  a16=(a1*a16);
  a15=(a15+a16);
  a21=(a21-a15);
  a21=(a14*a21);
  a21=(-a21);
  if (res[1]!=0) res[1][5]=a21;
  a21=arg[1]? arg[1][10] : 0;
  a15=(a2*a21);
  a16=arg[1]? arg[1][8] : 0;
  a22=(a1*a16);
  a15=(a15+a22);
  a22=arg[1]? arg[1][11] : 0;
  a18=(a5*a22);
  a19=arg[1]? arg[1][7] : 0;
  a20=(a4*a19);
  a18=(a18+a20);
  a15=(a15-a18);
  a15=(-a15);
  if (res[1]!=0) res[1][6]=a15;
  a15=(a0*a22);
  a18=arg[1]? arg[1][6] : 0;
  a20=(a4*a18);
  a15=(a15+a20);
  a20=arg[1]? arg[1][9] : 0;
  a17=(a2*a20);
  a16=(a6*a16);
  a17=(a17+a16);
  a15=(a15-a17);
  a15=(-a15);
  if (res[1]!=0) res[1][7]=a15;
  a15=(a5*a20);
  a19=(a6*a19);
  a15=(a15+a19);
  a19=(a0*a21);
  a18=(a1*a18);
  a19=(a19+a18);
  a15=(a15-a19);
  a15=(-a15);
  if (res[1]!=0) res[1][8]=a15;
  a15=(a8*a21);
  a19=(a7*a22);
  a18=(a1*a19);
  a15=(a15+a18);
  a18=(a11*a22);
  a17=(a10*a21);
  a16=(a4*a17);
  a18=(a18+a16);
  a15=(a15-a18);
  a15=(a3*a15);
  a15=(-a15);
  if (res[1]!=0) res[1][9]=a15;
  a22=(a13*a22);
  a15=(a12*a20);
  a18=(a4*a15);
  a22=(a22+a18);
  a18=(a8*a20);
  a19=(a6*a19);
  a18=(a18+a19);
  a22=(a22-a18);
  a22=(a9*a22);
  a22=(-a22);
  if (res[1]!=0) res[1][10]=a22;
  a20=(a11*a20);
  a17=(a6*a17);
  a20=(a20+a17);
  a21=(a13*a21);
  a15=(a1*a15);
  a21=(a21+a15);
  a20=(a20-a21);
  a20=(a14*a20);
  a20=(-a20);
  if (res[1]!=0) res[1][11]=a20;
  a20=arg[1]? arg[1][16] : 0;
  a21=(a2*a20);
  a15=arg[1]? arg[1][14] : 0;
  a17=(a1*a15);
  a21=(a21+a17);
  a17=arg[1]? arg[1][17] : 0;
  a22=(a5*a17);
  a18=arg[1]? arg[1][13] : 0;
  a19=(a4*a18);
  a22=(a22+a19);
  a21=(a21-a22);
  a21=(-a21);
  if (res[1]!=0) res[1][12]=a21;
  a21=(a0*a17);
  a22=arg[1]? arg[1][12] : 0;
  a19=(a4*a22);
  a21=(a21+a19);
  a19=arg[1]? arg[1][15] : 0;
  a16=(a2*a19);
  a15=(a6*a15);
  a16=(a16+a15);
  a21=(a21-a16);
  a21=(-a21);
  if (res[1]!=0) res[1][13]=a21;
  a21=(a5*a19);
  a18=(a6*a18);
  a21=(a21+a18);
  a18=(a0*a20);
  a22=(a1*a22);
  a18=(a18+a22);
  a21=(a21-a18);
  a21=(-a21);
  if (res[1]!=0) res[1][14]=a21;
  a21=(a8*a20);
  a18=(a7*a17);
  a22=(a1*a18);
  a21=(a21+a22);
  a22=(a11*a17);
  a16=(a10*a20);
  a15=(a4*a16);
  a22=(a22+a15);
  a21=(a21-a22);
  a21=(a3*a21);
  a21=(-a21);
  if (res[1]!=0) res[1][15]=a21;
  a17=(a13*a17);
  a21=(a12*a19);
  a22=(a4*a21);
  a17=(a17+a22);
  a22=(a8*a19);
  a18=(a6*a18);
  a22=(a22+a18);
  a17=(a17-a22);
  a17=(a9*a17);
  a17=(-a17);
  if (res[1]!=0) res[1][16]=a17;
  a19=(a11*a19);
  a16=(a6*a16);
  a19=(a19+a16);
  a20=(a13*a20);
  a21=(a1*a21);
  a20=(a20+a21);
  a19=(a19-a20);
  a19=(a14*a19);
  a19=(-a19);
  if (res[1]!=0) res[1][17]=a19;
  a19=arg[1]? arg[1][22] : 0;
  a20=(a2*a19);
  a21=arg[1]? arg[1][20] : 0;
  a16=(a1*a21);
  a20=(a20+a16);
  a16=arg[1]? arg[1][23] : 0;
  a17=(a5*a16);
  a22=arg[1]? arg[1][19] : 0;
  a18=(a4*a22);
  a17=(a17+a18);
  a20=(a20-a17);
  a20=(-a20);
  if (res[1]!=0) res[1][18]=a20;
  a20=(a0*a16);
  a17=arg[1]? arg[1][18] : 0;
  a18=(a4*a17);
  a20=(a20+a18);
  a18=arg[1]? arg[1][21] : 0;
  a15=(a2*a18);
  a21=(a6*a21);
  a15=(a15+a21);
  a20=(a20-a15);
  a20=(-a20);
  if (res[1]!=0) res[1][19]=a20;
  a20=(a5*a18);
  a22=(a6*a22);
  a20=(a20+a22);
  a22=(a0*a19);
  a17=(a1*a17);
  a22=(a22+a17);
  a20=(a20-a22);
  a20=(-a20);
  if (res[1]!=0) res[1][20]=a20;
  a20=(a8*a19);
  a22=(a7*a16);
  a17=(a1*a22);
  a20=(a20+a17);
  a17=(a11*a16);
  a15=(a10*a19);
  a21=(a4*a15);
  a17=(a17+a21);
  a20=(a20-a17);
  a20=(a3*a20);
  a20=(-a20);
  if (res[1]!=0) res[1][21]=a20;
  a16=(a13*a16);
  a20=(a12*a18);
  a17=(a4*a20);
  a16=(a16+a17);
  a17=(a8*a18);
  a22=(a6*a22);
  a17=(a17+a22);
  a16=(a16-a17);
  a16=(a9*a16);
  a16=(-a16);
  if (res[1]!=0) res[1][22]=a16;
  a18=(a11*a18);
  a15=(a6*a15);
  a18=(a18+a15);
  a19=(a13*a19);
  a20=(a1*a20);
  a19=(a19+a20);
  a18=(a18-a19);
  a18=(a14*a18);
  a18=(-a18);
  if (res[1]!=0) res[1][23]=a18;
  a18=arg[1]? arg[1][28] : 0;
  a19=(a2*a18);
  a20=arg[1]? arg[1][26] : 0;
  a15=(a1*a20);
  a19=(a19+a15);
  a15=arg[1]? arg[1][29] : 0;
  a16=(a5*a15);
  a17=arg[1]? arg[1][25] : 0;
  a22=(a4*a17);
  a16=(a16+a22);
  a19=(a19-a16);
  a19=(-a19);
  if (res[1]!=0) res[1][24]=a19;
  a19=(a0*a15);
  a16=arg[1]? arg[1][24] : 0;
  a22=(a4*a16);
  a19=(a19+a22);
  a22=arg[1]? arg[1][27] : 0;
  a21=(a2*a22);
  a20=(a6*a20);
  a21=(a21+a20);
  a19=(a19-a21);
  a19=(-a19);
  if (res[1]!=0) res[1][25]=a19;
  a19=(a5*a22);
  a17=(a6*a17);
  a19=(a19+a17);
  a17=(a0*a18);
  a16=(a1*a16);
  a17=(a17+a16);
  a19=(a19-a17);
  a19=(-a19);
  if (res[1]!=0) res[1][26]=a19;
  a19=(a8*a18);
  a17=(a7*a15);
  a16=(a1*a17);
  a19=(a19+a16);
  a16=(a11*a15);
  a21=(a10*a18);
  a20=(a4*a21);
  a16=(a16+a20);
  a19=(a19-a16);
  a19=(a3*a19);
  a19=(-a19);
  if (res[1]!=0) res[1][27]=a19;
  a15=(a13*a15);
  a19=(a12*a22);
  a16=(a4*a19);
  a15=(a15+a16);
  a16=(a8*a22);
  a17=(a6*a17);
  a16=(a16+a17);
  a15=(a15-a16);
  a15=(a9*a15);
  a15=(-a15);
  if (res[1]!=0) res[1][28]=a15;
  a22=(a11*a22);
  a21=(a6*a21);
  a22=(a22+a21);
  a18=(a13*a18);
  a19=(a1*a19);
  a18=(a18+a19);
  a22=(a22-a18);
  a22=(a14*a22);
  a22=(-a22);
  if (res[1]!=0) res[1][29]=a22;
  a22=arg[1]? arg[1][34] : 0;
  a18=(a2*a22);
  a19=arg[1]? arg[1][32] : 0;
  a21=(a1*a19);
  a18=(a18+a21);
  a21=arg[1]? arg[1][35] : 0;
  a15=(a5*a21);
  a16=arg[1]? arg[1][31] : 0;
  a17=(a4*a16);
  a15=(a15+a17);
  a18=(a18-a15);
  a18=(-a18);
  if (res[1]!=0) res[1][30]=a18;
  a18=(a0*a21);
  a15=arg[1]? arg[1][30] : 0;
  a17=(a4*a15);
  a18=(a18+a17);
  a17=arg[1]? arg[1][33] : 0;
  a20=(a2*a17);
  a19=(a6*a19);
  a20=(a20+a19);
  a18=(a18-a20);
  a18=(-a18);
  if (res[1]!=0) res[1][31]=a18;
  a18=(a5*a17);
  a16=(a6*a16);
  a18=(a18+a16);
  a16=(a0*a22);
  a15=(a1*a15);
  a16=(a16+a15);
  a18=(a18-a16);
  a18=(-a18);
  if (res[1]!=0) res[1][32]=a18;
  a18=(a8*a22);
  a16=(a7*a21);
  a15=(a1*a16);
  a18=(a18+a15);
  a15=(a11*a21);
  a20=(a10*a22);
  a19=(a4*a20);
  a15=(a15+a19);
  a18=(a18-a15);
  a18=(a3*a18);
  a18=(-a18);
  if (res[1]!=0) res[1][33]=a18;
  a21=(a13*a21);
  a18=(a12*a17);
  a15=(a4*a18);
  a21=(a21+a15);
  a15=(a8*a17);
  a16=(a6*a16);
  a15=(a15+a16);
  a21=(a21-a15);
  a21=(a9*a21);
  a21=(-a21);
  if (res[1]!=0) res[1][34]=a21;
  a17=(a11*a17);
  a20=(a6*a20);
  a17=(a17+a20);
  a22=(a13*a22);
  a18=(a1*a18);
  a22=(a22+a18);
  a17=(a17-a22);
  a17=(a14*a17);
  a17=(-a17);
  if (res[1]!=0) res[1][35]=a17;
  a17=3.6496350364963508e-02;
  a22=arg[2]? arg[2][4] : 0;
  a18=(a2*a22);
  a20=arg[2]? arg[2][2] : 0;
  a21=(a1*a20);
  a18=(a18+a21);
  a21=arg[2]? arg[2][5] : 0;
  a15=(a5*a21);
  a16=arg[2]? arg[2][1] : 0;
  a19=(a4*a16);
  a15=(a15+a19);
  a18=(a18-a15);
  a17=(a17-a18);
  if (res[2]!=0) res[2][0]=a17;
  a17=(a0*a21);
  a18=arg[2]? arg[2][0] : 0;
  a15=(a4*a18);
  a17=(a17+a15);
  a15=arg[2]? arg[2][3] : 0;
  a19=(a2*a15);
  a20=(a6*a20);
  a19=(a19+a20);
  a17=(a17-a19);
  a17=(-a17);
  if (res[2]!=0) res[2][1]=a17;
  a17=(a5*a15);
  a16=(a6*a16);
  a17=(a17+a16);
  a16=(a0*a22);
  a18=(a1*a18);
  a16=(a16+a18);
  a17=(a17-a16);
  a17=(-a17);
  if (res[2]!=0) res[2][2]=a17;
  a17=(a8*a22);
  a16=(a7*a21);
  a18=(a1*a16);
  a17=(a17+a18);
  a18=(a11*a21);
  a19=(a10*a22);
  a20=(a4*a19);
  a18=(a18+a20);
  a17=(a17-a18);
  a17=(a3*a17);
  a17=(-a17);
  if (res[2]!=0) res[2][3]=a17;
  a21=(a13*a21);
  a17=(a12*a15);
  a18=(a4*a17);
  a21=(a21+a18);
  a18=(a8*a15);
  a16=(a6*a16);
  a18=(a18+a16);
  a21=(a21-a18);
  a21=(a9*a21);
  a21=(-a21);
  if (res[2]!=0) res[2][4]=a21;
  a15=(a11*a15);
  a19=(a6*a19);
  a15=(a15+a19);
  a22=(a13*a22);
  a17=(a1*a17);
  a22=(a22+a17);
  a15=(a15-a22);
  a15=(a14*a15);
  a15=(-a15);
  if (res[2]!=0) res[2][5]=a15;
  a15=arg[2]? arg[2][10] : 0;
  a22=(a2*a15);
  a17=arg[2]? arg[2][8] : 0;
  a19=(a1*a17);
  a22=(a22+a19);
  a19=arg[2]? arg[2][11] : 0;
  a21=(a5*a19);
  a18=arg[2]? arg[2][7] : 0;
  a16=(a4*a18);
  a21=(a21+a16);
  a22=(a22-a21);
  a22=(-a22);
  if (res[2]!=0) res[2][6]=a22;
  a22=(a0*a19);
  a21=arg[2]? arg[2][6] : 0;
  a16=(a4*a21);
  a22=(a22+a16);
  a16=arg[2]? arg[2][9] : 0;
  a2=(a2*a16);
  a17=(a6*a17);
  a2=(a2+a17);
  a22=(a22-a2);
  a22=(-a22);
  if (res[2]!=0) res[2][7]=a22;
  a5=(a5*a16);
  a18=(a6*a18);
  a5=(a5+a18);
  a0=(a0*a15);
  a21=(a1*a21);
  a0=(a0+a21);
  a5=(a5-a0);
  a5=(-a5);
  if (res[2]!=0) res[2][8]=a5;
  a5=(a8*a15);
  a7=(a7*a19);
  a0=(a1*a7);
  a5=(a5+a0);
  a0=(a11*a19);
  a10=(a10*a15);
  a21=(a4*a10);
  a0=(a0+a21);
  a5=(a5-a0);
  a3=(a3*a5);
  a3=(-a3);
  if (res[2]!=0) res[2][9]=a3;
  a19=(a13*a19);
  a12=(a12*a16);
  a4=(a4*a12);
  a19=(a19+a4);
  a8=(a8*a16);
  a7=(a6*a7);
  a8=(a8+a7);
  a19=(a19-a8);
  a9=(a9*a19);
  a9=(-a9);
  if (res[2]!=0) res[2][10]=a9;
  a11=(a11*a16);
  a6=(a6*a10);
  a11=(a11+a6);
  a13=(a13*a15);
  a1=(a1*a12);
  a13=(a13+a1);
  a11=(a11-a13);
  a11=(a14*a11);
  a14=(a14-a11);
  if (res[2]!=0) res[2][11]=a14;
  return 0;
}

CASADI_SYMBOL_EXPORT int dynamics_expl_vde_forw(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int dynamics_expl_vde_forw_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int dynamics_expl_vde_forw_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void dynamics_expl_vde_forw_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int dynamics_expl_vde_forw_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void dynamics_expl_vde_forw_release(int mem) {
}

CASADI_SYMBOL_EXPORT void dynamics_expl_vde_forw_incref(void) {
}

CASADI_SYMBOL_EXPORT void dynamics_expl_vde_forw_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int dynamics_expl_vde_forw_n_in(void) { return 5;}

CASADI_SYMBOL_EXPORT casadi_int dynamics_expl_vde_forw_n_out(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_real dynamics_expl_vde_forw_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* dynamics_expl_vde_forw_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    case 4: return "i4";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* dynamics_expl_vde_forw_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* dynamics_expl_vde_forw_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s3;
    case 4: return casadi_s4;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* dynamics_expl_vde_forw_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int dynamics_expl_vde_forw_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 5;
  if (sz_res) *sz_res = 3;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

CASADI_SYMBOL_EXPORT int dynamics_expl_vde_forw_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 5*sizeof(const casadi_real*);
  if (sz_res) *sz_res = 3*sizeof(casadi_real*);
  if (sz_iw) *sz_iw = 0*sizeof(casadi_int);
  if (sz_w) *sz_w = 0*sizeof(casadi_real);
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
