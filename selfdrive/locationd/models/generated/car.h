#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_6978687084586341298);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8854305474282830070);
void car_H_mod_fun(double *state, double *out_3156633871176746401);
void car_f_fun(double *state, double dt, double *out_4802494839670766233);
void car_F_fun(double *state, double dt, double *out_5891574291073418751);
void car_h_25(double *state, double *unused, double *out_985558434282023497);
void car_H_25(double *state, double *unused, double *out_6036554354735490836);
void car_h_24(double *state, double *unused, double *out_1166412536478823260);
void car_H_24(double *state, double *unused, double *out_225168723712713871);
void car_h_30(double *state, double *unused, double *out_192172075693954169);
void car_H_30(double *state, double *unused, double *out_880135986756125919);
void car_h_26(double *state, double *unused, double *out_4617532990992185702);
void car_H_26(double *state, double *unused, double *out_8668686400100004556);
void car_h_27(double *state, double *unused, double *out_6901189188015885833);
void car_H_27(double *state, double *unused, double *out_1294627325044298992);
void car_h_29(double *state, double *unused, double *out_8597828895338180822);
void car_H_29(double *state, double *unused, double *out_1390367331070518103);
void car_h_28(double *state, double *unused, double *out_3864403136571553148);
void car_H_28(double *state, double *unused, double *out_3692031685999012471);
void car_h_31(double *state, double *unused, double *out_1962167556073461464);
void car_H_31(double *state, double *unused, double *out_6005908392858530408);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}