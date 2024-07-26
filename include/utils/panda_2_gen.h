/* This file was automatically generated by CasADi 3.6.5+.
 *  It consists of: 
 *   1) content generated by CasADi runtime: not copyrighted
 *   2) template code copied from CasADi source: permissively licensed (MIT-0)
 *   3) user code: owned by the user
 *
 */
#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

extern "C" int dotJac_fun(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
extern "C" int dotJac_fun_alloc_mem(void);
extern "C" int dotJac_fun_init_mem(int mem);
extern "C" void dotJac_fun_free_mem(int mem);
extern "C" int dotJac_fun_checkout(void);
extern "C" void dotJac_fun_release(int mem);
extern "C" void dotJac_fun_incref(void);
extern "C" void dotJac_fun_decref(void);
extern "C" casadi_int dotJac_fun_n_in(void);
extern "C" casadi_int dotJac_fun_n_out(void);
extern "C" casadi_real dotJac_fun_default_in(casadi_int i);
extern "C" const char* dotJac_fun_name_in(casadi_int i);
extern "C" const char* dotJac_fun_name_out(casadi_int i);
extern "C" const casadi_int* dotJac_fun_sparsity_in(casadi_int i);
extern "C" const casadi_int* dotJac_fun_sparsity_out(casadi_int i);
extern "C" int dotJac_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
extern "C" int dotJac_fun_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define dotJac_fun_SZ_ARG 2
#define dotJac_fun_SZ_RES 1
#define dotJac_fun_SZ_IW 0
#define dotJac_fun_SZ_W 117
extern "C" int pinvJac_fun(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
extern "C" int pinvJac_fun_alloc_mem(void);
extern "C" int pinvJac_fun_init_mem(int mem);
extern "C" void pinvJac_fun_free_mem(int mem);
extern "C" int pinvJac_fun_checkout(void);
extern "C" void pinvJac_fun_release(int mem);
extern "C" void pinvJac_fun_incref(void);
extern "C" void pinvJac_fun_decref(void);
extern "C" casadi_int pinvJac_fun_n_in(void);
extern "C" casadi_int pinvJac_fun_n_out(void);
extern "C" casadi_real pinvJac_fun_default_in(casadi_int i);
extern "C" const char* pinvJac_fun_name_in(casadi_int i);
extern "C" const char* pinvJac_fun_name_out(casadi_int i);
extern "C" const casadi_int* pinvJac_fun_sparsity_in(casadi_int i);
extern "C" const casadi_int* pinvJac_fun_sparsity_out(casadi_int i);
extern "C" int pinvJac_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
extern "C" int pinvJac_fun_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define pinvJac_fun_SZ_ARG 1
#define pinvJac_fun_SZ_RES 1
#define pinvJac_fun_SZ_IW 0
#define pinvJac_fun_SZ_W 98
extern "C" int pinvJacPos_fun(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
extern "C" int pinvJacPos_fun_alloc_mem(void);
extern "C" int pinvJacPos_fun_init_mem(int mem);
extern "C" void pinvJacPos_fun_free_mem(int mem);
extern "C" int pinvJacPos_fun_checkout(void);
extern "C" void pinvJacPos_fun_release(int mem);
extern "C" void pinvJacPos_fun_incref(void);
extern "C" void pinvJacPos_fun_decref(void);
extern "C" casadi_int pinvJacPos_fun_n_in(void);
extern "C" casadi_int pinvJacPos_fun_n_out(void);
extern "C" casadi_real pinvJacPos_fun_default_in(casadi_int i);
extern "C" const char* pinvJacPos_fun_name_in(casadi_int i);
extern "C" const char* pinvJacPos_fun_name_out(casadi_int i);
extern "C" const casadi_int* pinvJacPos_fun_sparsity_in(casadi_int i);
extern "C" const casadi_int* pinvJacPos_fun_sparsity_out(casadi_int i);
extern "C" int pinvJacPos_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
extern "C" int pinvJacPos_fun_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define pinvJacPos_fun_SZ_ARG 1
#define pinvJacPos_fun_SZ_RES 1
#define pinvJacPos_fun_SZ_IW 0
#define pinvJacPos_fun_SZ_W 62
extern "C" int dotPinvJac_fun(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
extern "C" int dotPinvJac_fun_alloc_mem(void);
extern "C" int dotPinvJac_fun_init_mem(int mem);
extern "C" void dotPinvJac_fun_free_mem(int mem);
extern "C" int dotPinvJac_fun_checkout(void);
extern "C" void dotPinvJac_fun_release(int mem);
extern "C" void dotPinvJac_fun_incref(void);
extern "C" void dotPinvJac_fun_decref(void);
extern "C" casadi_int dotPinvJac_fun_n_in(void);
extern "C" casadi_int dotPinvJac_fun_n_out(void);
extern "C" casadi_real dotPinvJac_fun_default_in(casadi_int i);
extern "C" const char* dotPinvJac_fun_name_in(casadi_int i);
extern "C" const char* dotPinvJac_fun_name_out(casadi_int i);
extern "C" const casadi_int* dotPinvJac_fun_sparsity_in(casadi_int i);
extern "C" const casadi_int* dotPinvJac_fun_sparsity_out(casadi_int i);
extern "C" int dotPinvJac_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
extern "C" int dotPinvJac_fun_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define dotPinvJac_fun_SZ_ARG 2
#define dotPinvJac_fun_SZ_RES 1
#define dotPinvJac_fun_SZ_IW 0
#define dotPinvJac_fun_SZ_W 353
extern "C" int dotPinvJacPos_fun(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
extern "C" int dotPinvJacPos_fun_alloc_mem(void);
extern "C" int dotPinvJacPos_fun_init_mem(int mem);
extern "C" void dotPinvJacPos_fun_free_mem(int mem);
extern "C" int dotPinvJacPos_fun_checkout(void);
extern "C" void dotPinvJacPos_fun_release(int mem);
extern "C" void dotPinvJacPos_fun_incref(void);
extern "C" void dotPinvJacPos_fun_decref(void);
extern "C" casadi_int dotPinvJacPos_fun_n_in(void);
extern "C" casadi_int dotPinvJacPos_fun_n_out(void);
extern "C" casadi_real dotPinvJacPos_fun_default_in(casadi_int i);
extern "C" const char* dotPinvJacPos_fun_name_in(casadi_int i);
extern "C" const char* dotPinvJacPos_fun_name_out(casadi_int i);
extern "C" const casadi_int* dotPinvJacPos_fun_sparsity_in(casadi_int i);
extern "C" const casadi_int* dotPinvJacPos_fun_sparsity_out(casadi_int i);
extern "C" int dotPinvJacPos_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
extern "C" int dotPinvJacPos_fun_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define dotPinvJacPos_fun_SZ_ARG 2
#define dotPinvJacPos_fun_SZ_RES 1
#define dotPinvJacPos_fun_SZ_IW 0
#define dotPinvJacPos_fun_SZ_W 184
extern "C" int kin_fun(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
extern "C" int kin_fun_alloc_mem(void);
extern "C" int kin_fun_init_mem(int mem);
extern "C" void kin_fun_free_mem(int mem);
extern "C" int kin_fun_checkout(void);
extern "C" void kin_fun_release(int mem);
extern "C" void kin_fun_incref(void);
extern "C" void kin_fun_decref(void);
extern "C" casadi_int kin_fun_n_in(void);
extern "C" casadi_int kin_fun_n_out(void);
extern "C" casadi_real kin_fun_default_in(casadi_int i);
extern "C" const char* kin_fun_name_in(casadi_int i);
extern "C" const char* kin_fun_name_out(casadi_int i);
extern "C" const casadi_int* kin_fun_sparsity_in(casadi_int i);
extern "C" const casadi_int* kin_fun_sparsity_out(casadi_int i);
extern "C" int kin_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
extern "C" int kin_fun_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define kin_fun_SZ_ARG 1
#define kin_fun_SZ_RES 1
#define kin_fun_SZ_IW 0
#define kin_fun_SZ_W 44
extern "C" int jac_fun(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
extern "C" int jac_fun_alloc_mem(void);
extern "C" int jac_fun_init_mem(int mem);
extern "C" void jac_fun_free_mem(int mem);
extern "C" int jac_fun_checkout(void);
extern "C" void jac_fun_release(int mem);
extern "C" void jac_fun_incref(void);
extern "C" void jac_fun_decref(void);
extern "C" casadi_int jac_fun_n_in(void);
extern "C" casadi_int jac_fun_n_out(void);
extern "C" casadi_real jac_fun_default_in(casadi_int i);
extern "C" const char* jac_fun_name_in(casadi_int i);
extern "C" const char* jac_fun_name_out(casadi_int i);
extern "C" const casadi_int* jac_fun_sparsity_in(casadi_int i);
extern "C" const casadi_int* jac_fun_sparsity_out(casadi_int i);
extern "C" int jac_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
extern "C" int jac_fun_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define jac_fun_SZ_ARG 1
#define jac_fun_SZ_RES 1
#define jac_fun_SZ_IW 0
#define jac_fun_SZ_W 61
extern "C" int regr_fun(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
extern "C" int regr_fun_alloc_mem(void);
extern "C" int regr_fun_init_mem(int mem);
extern "C" void regr_fun_free_mem(int mem);
extern "C" int regr_fun_checkout(void);
extern "C" void regr_fun_release(int mem);
extern "C" void regr_fun_incref(void);
extern "C" void regr_fun_decref(void);
extern "C" casadi_int regr_fun_n_in(void);
extern "C" casadi_int regr_fun_n_out(void);
extern "C" casadi_real regr_fun_default_in(casadi_int i);
extern "C" const char* regr_fun_name_in(casadi_int i);
extern "C" const char* regr_fun_name_out(casadi_int i);
extern "C" const casadi_int* regr_fun_sparsity_in(casadi_int i);
extern "C" const casadi_int* regr_fun_sparsity_out(casadi_int i);
extern "C" int regr_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
extern "C" int regr_fun_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define regr_fun_SZ_ARG 4
#define regr_fun_SZ_RES 1
#define regr_fun_SZ_IW 0
#define regr_fun_SZ_W 475
extern "C" int mass_fun(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
extern "C" int mass_fun_alloc_mem(void);
extern "C" int mass_fun_init_mem(int mem);
extern "C" void mass_fun_free_mem(int mem);
extern "C" int mass_fun_checkout(void);
extern "C" void mass_fun_release(int mem);
extern "C" void mass_fun_incref(void);
extern "C" void mass_fun_decref(void);
extern "C" casadi_int mass_fun_n_in(void);
extern "C" casadi_int mass_fun_n_out(void);
extern "C" casadi_real mass_fun_default_in(casadi_int i);
extern "C" const char* mass_fun_name_in(casadi_int i);
extern "C" const char* mass_fun_name_out(casadi_int i);
extern "C" const casadi_int* mass_fun_sparsity_in(casadi_int i);
extern "C" const casadi_int* mass_fun_sparsity_out(casadi_int i);
extern "C" int mass_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
extern "C" int mass_fun_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define mass_fun_SZ_ARG 2
#define mass_fun_SZ_RES 1
#define mass_fun_SZ_IW 0
#define mass_fun_SZ_W 247
extern "C" int coriolis_fun(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
extern "C" int coriolis_fun_alloc_mem(void);
extern "C" int coriolis_fun_init_mem(int mem);
extern "C" void coriolis_fun_free_mem(int mem);
extern "C" int coriolis_fun_checkout(void);
extern "C" void coriolis_fun_release(int mem);
extern "C" void coriolis_fun_incref(void);
extern "C" void coriolis_fun_decref(void);
extern "C" casadi_int coriolis_fun_n_in(void);
extern "C" casadi_int coriolis_fun_n_out(void);
extern "C" casadi_real coriolis_fun_default_in(casadi_int i);
extern "C" const char* coriolis_fun_name_in(casadi_int i);
extern "C" const char* coriolis_fun_name_out(casadi_int i);
extern "C" const casadi_int* coriolis_fun_sparsity_in(casadi_int i);
extern "C" const casadi_int* coriolis_fun_sparsity_out(casadi_int i);
extern "C" int coriolis_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
extern "C" int coriolis_fun_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define coriolis_fun_SZ_ARG 3
#define coriolis_fun_SZ_RES 1
#define coriolis_fun_SZ_IW 0
#define coriolis_fun_SZ_W 1343
extern "C" int gravity_fun(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
extern "C" int gravity_fun_alloc_mem(void);
extern "C" int gravity_fun_init_mem(int mem);
extern "C" void gravity_fun_free_mem(int mem);
extern "C" int gravity_fun_checkout(void);
extern "C" void gravity_fun_release(int mem);
extern "C" void gravity_fun_incref(void);
extern "C" void gravity_fun_decref(void);
extern "C" casadi_int gravity_fun_n_in(void);
extern "C" casadi_int gravity_fun_n_out(void);
extern "C" casadi_real gravity_fun_default_in(casadi_int i);
extern "C" const char* gravity_fun_name_in(casadi_int i);
extern "C" const char* gravity_fun_name_out(casadi_int i);
extern "C" const casadi_int* gravity_fun_sparsity_in(casadi_int i);
extern "C" const casadi_int* gravity_fun_sparsity_out(casadi_int i);
extern "C" int gravity_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
extern "C" int gravity_fun_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define gravity_fun_SZ_ARG 2
#define gravity_fun_SZ_RES 1
#define gravity_fun_SZ_IW 0
#define gravity_fun_SZ_W 54