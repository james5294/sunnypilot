#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_7424980701830897997);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_8948729870032527372);
void pose_H_mod_fun(double *state, double *out_7614750544922110094);
void pose_f_fun(double *state, double dt, double *out_4078122102263429604);
void pose_F_fun(double *state, double dt, double *out_8150619080602528479);
void pose_h_4(double *state, double *unused, double *out_2389356497093565065);
void pose_H_4(double *state, double *unused, double *out_4340528466131160841);
void pose_h_10(double *state, double *unused, double *out_8795577960664591671);
void pose_H_10(double *state, double *unused, double *out_8489014100523960796);
void pose_h_13(double *state, double *unused, double *out_2851896194690110696);
void pose_H_13(double *state, double *unused, double *out_6495584399261689846);
void pose_h_14(double *state, double *unused, double *out_7309375191689700906);
void pose_H_14(double *state, double *unused, double *out_8303769322470645370);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}