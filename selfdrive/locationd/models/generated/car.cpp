#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.13.2                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_6978687084586341298) {
   out_6978687084586341298[0] = delta_x[0] + nom_x[0];
   out_6978687084586341298[1] = delta_x[1] + nom_x[1];
   out_6978687084586341298[2] = delta_x[2] + nom_x[2];
   out_6978687084586341298[3] = delta_x[3] + nom_x[3];
   out_6978687084586341298[4] = delta_x[4] + nom_x[4];
   out_6978687084586341298[5] = delta_x[5] + nom_x[5];
   out_6978687084586341298[6] = delta_x[6] + nom_x[6];
   out_6978687084586341298[7] = delta_x[7] + nom_x[7];
   out_6978687084586341298[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8854305474282830070) {
   out_8854305474282830070[0] = -nom_x[0] + true_x[0];
   out_8854305474282830070[1] = -nom_x[1] + true_x[1];
   out_8854305474282830070[2] = -nom_x[2] + true_x[2];
   out_8854305474282830070[3] = -nom_x[3] + true_x[3];
   out_8854305474282830070[4] = -nom_x[4] + true_x[4];
   out_8854305474282830070[5] = -nom_x[5] + true_x[5];
   out_8854305474282830070[6] = -nom_x[6] + true_x[6];
   out_8854305474282830070[7] = -nom_x[7] + true_x[7];
   out_8854305474282830070[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_3156633871176746401) {
   out_3156633871176746401[0] = 1.0;
   out_3156633871176746401[1] = 0.0;
   out_3156633871176746401[2] = 0.0;
   out_3156633871176746401[3] = 0.0;
   out_3156633871176746401[4] = 0.0;
   out_3156633871176746401[5] = 0.0;
   out_3156633871176746401[6] = 0.0;
   out_3156633871176746401[7] = 0.0;
   out_3156633871176746401[8] = 0.0;
   out_3156633871176746401[9] = 0.0;
   out_3156633871176746401[10] = 1.0;
   out_3156633871176746401[11] = 0.0;
   out_3156633871176746401[12] = 0.0;
   out_3156633871176746401[13] = 0.0;
   out_3156633871176746401[14] = 0.0;
   out_3156633871176746401[15] = 0.0;
   out_3156633871176746401[16] = 0.0;
   out_3156633871176746401[17] = 0.0;
   out_3156633871176746401[18] = 0.0;
   out_3156633871176746401[19] = 0.0;
   out_3156633871176746401[20] = 1.0;
   out_3156633871176746401[21] = 0.0;
   out_3156633871176746401[22] = 0.0;
   out_3156633871176746401[23] = 0.0;
   out_3156633871176746401[24] = 0.0;
   out_3156633871176746401[25] = 0.0;
   out_3156633871176746401[26] = 0.0;
   out_3156633871176746401[27] = 0.0;
   out_3156633871176746401[28] = 0.0;
   out_3156633871176746401[29] = 0.0;
   out_3156633871176746401[30] = 1.0;
   out_3156633871176746401[31] = 0.0;
   out_3156633871176746401[32] = 0.0;
   out_3156633871176746401[33] = 0.0;
   out_3156633871176746401[34] = 0.0;
   out_3156633871176746401[35] = 0.0;
   out_3156633871176746401[36] = 0.0;
   out_3156633871176746401[37] = 0.0;
   out_3156633871176746401[38] = 0.0;
   out_3156633871176746401[39] = 0.0;
   out_3156633871176746401[40] = 1.0;
   out_3156633871176746401[41] = 0.0;
   out_3156633871176746401[42] = 0.0;
   out_3156633871176746401[43] = 0.0;
   out_3156633871176746401[44] = 0.0;
   out_3156633871176746401[45] = 0.0;
   out_3156633871176746401[46] = 0.0;
   out_3156633871176746401[47] = 0.0;
   out_3156633871176746401[48] = 0.0;
   out_3156633871176746401[49] = 0.0;
   out_3156633871176746401[50] = 1.0;
   out_3156633871176746401[51] = 0.0;
   out_3156633871176746401[52] = 0.0;
   out_3156633871176746401[53] = 0.0;
   out_3156633871176746401[54] = 0.0;
   out_3156633871176746401[55] = 0.0;
   out_3156633871176746401[56] = 0.0;
   out_3156633871176746401[57] = 0.0;
   out_3156633871176746401[58] = 0.0;
   out_3156633871176746401[59] = 0.0;
   out_3156633871176746401[60] = 1.0;
   out_3156633871176746401[61] = 0.0;
   out_3156633871176746401[62] = 0.0;
   out_3156633871176746401[63] = 0.0;
   out_3156633871176746401[64] = 0.0;
   out_3156633871176746401[65] = 0.0;
   out_3156633871176746401[66] = 0.0;
   out_3156633871176746401[67] = 0.0;
   out_3156633871176746401[68] = 0.0;
   out_3156633871176746401[69] = 0.0;
   out_3156633871176746401[70] = 1.0;
   out_3156633871176746401[71] = 0.0;
   out_3156633871176746401[72] = 0.0;
   out_3156633871176746401[73] = 0.0;
   out_3156633871176746401[74] = 0.0;
   out_3156633871176746401[75] = 0.0;
   out_3156633871176746401[76] = 0.0;
   out_3156633871176746401[77] = 0.0;
   out_3156633871176746401[78] = 0.0;
   out_3156633871176746401[79] = 0.0;
   out_3156633871176746401[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_4802494839670766233) {
   out_4802494839670766233[0] = state[0];
   out_4802494839670766233[1] = state[1];
   out_4802494839670766233[2] = state[2];
   out_4802494839670766233[3] = state[3];
   out_4802494839670766233[4] = state[4];
   out_4802494839670766233[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_4802494839670766233[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_4802494839670766233[7] = state[7];
   out_4802494839670766233[8] = state[8];
}
void F_fun(double *state, double dt, double *out_5891574291073418751) {
   out_5891574291073418751[0] = 1;
   out_5891574291073418751[1] = 0;
   out_5891574291073418751[2] = 0;
   out_5891574291073418751[3] = 0;
   out_5891574291073418751[4] = 0;
   out_5891574291073418751[5] = 0;
   out_5891574291073418751[6] = 0;
   out_5891574291073418751[7] = 0;
   out_5891574291073418751[8] = 0;
   out_5891574291073418751[9] = 0;
   out_5891574291073418751[10] = 1;
   out_5891574291073418751[11] = 0;
   out_5891574291073418751[12] = 0;
   out_5891574291073418751[13] = 0;
   out_5891574291073418751[14] = 0;
   out_5891574291073418751[15] = 0;
   out_5891574291073418751[16] = 0;
   out_5891574291073418751[17] = 0;
   out_5891574291073418751[18] = 0;
   out_5891574291073418751[19] = 0;
   out_5891574291073418751[20] = 1;
   out_5891574291073418751[21] = 0;
   out_5891574291073418751[22] = 0;
   out_5891574291073418751[23] = 0;
   out_5891574291073418751[24] = 0;
   out_5891574291073418751[25] = 0;
   out_5891574291073418751[26] = 0;
   out_5891574291073418751[27] = 0;
   out_5891574291073418751[28] = 0;
   out_5891574291073418751[29] = 0;
   out_5891574291073418751[30] = 1;
   out_5891574291073418751[31] = 0;
   out_5891574291073418751[32] = 0;
   out_5891574291073418751[33] = 0;
   out_5891574291073418751[34] = 0;
   out_5891574291073418751[35] = 0;
   out_5891574291073418751[36] = 0;
   out_5891574291073418751[37] = 0;
   out_5891574291073418751[38] = 0;
   out_5891574291073418751[39] = 0;
   out_5891574291073418751[40] = 1;
   out_5891574291073418751[41] = 0;
   out_5891574291073418751[42] = 0;
   out_5891574291073418751[43] = 0;
   out_5891574291073418751[44] = 0;
   out_5891574291073418751[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_5891574291073418751[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_5891574291073418751[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5891574291073418751[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5891574291073418751[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_5891574291073418751[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_5891574291073418751[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_5891574291073418751[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_5891574291073418751[53] = -9.8000000000000007*dt;
   out_5891574291073418751[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_5891574291073418751[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_5891574291073418751[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5891574291073418751[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5891574291073418751[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_5891574291073418751[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_5891574291073418751[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_5891574291073418751[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5891574291073418751[62] = 0;
   out_5891574291073418751[63] = 0;
   out_5891574291073418751[64] = 0;
   out_5891574291073418751[65] = 0;
   out_5891574291073418751[66] = 0;
   out_5891574291073418751[67] = 0;
   out_5891574291073418751[68] = 0;
   out_5891574291073418751[69] = 0;
   out_5891574291073418751[70] = 1;
   out_5891574291073418751[71] = 0;
   out_5891574291073418751[72] = 0;
   out_5891574291073418751[73] = 0;
   out_5891574291073418751[74] = 0;
   out_5891574291073418751[75] = 0;
   out_5891574291073418751[76] = 0;
   out_5891574291073418751[77] = 0;
   out_5891574291073418751[78] = 0;
   out_5891574291073418751[79] = 0;
   out_5891574291073418751[80] = 1;
}
void h_25(double *state, double *unused, double *out_985558434282023497) {
   out_985558434282023497[0] = state[6];
}
void H_25(double *state, double *unused, double *out_6036554354735490836) {
   out_6036554354735490836[0] = 0;
   out_6036554354735490836[1] = 0;
   out_6036554354735490836[2] = 0;
   out_6036554354735490836[3] = 0;
   out_6036554354735490836[4] = 0;
   out_6036554354735490836[5] = 0;
   out_6036554354735490836[6] = 1;
   out_6036554354735490836[7] = 0;
   out_6036554354735490836[8] = 0;
}
void h_24(double *state, double *unused, double *out_1166412536478823260) {
   out_1166412536478823260[0] = state[4];
   out_1166412536478823260[1] = state[5];
}
void H_24(double *state, double *unused, double *out_225168723712713871) {
   out_225168723712713871[0] = 0;
   out_225168723712713871[1] = 0;
   out_225168723712713871[2] = 0;
   out_225168723712713871[3] = 0;
   out_225168723712713871[4] = 1;
   out_225168723712713871[5] = 0;
   out_225168723712713871[6] = 0;
   out_225168723712713871[7] = 0;
   out_225168723712713871[8] = 0;
   out_225168723712713871[9] = 0;
   out_225168723712713871[10] = 0;
   out_225168723712713871[11] = 0;
   out_225168723712713871[12] = 0;
   out_225168723712713871[13] = 0;
   out_225168723712713871[14] = 1;
   out_225168723712713871[15] = 0;
   out_225168723712713871[16] = 0;
   out_225168723712713871[17] = 0;
}
void h_30(double *state, double *unused, double *out_192172075693954169) {
   out_192172075693954169[0] = state[4];
}
void H_30(double *state, double *unused, double *out_880135986756125919) {
   out_880135986756125919[0] = 0;
   out_880135986756125919[1] = 0;
   out_880135986756125919[2] = 0;
   out_880135986756125919[3] = 0;
   out_880135986756125919[4] = 1;
   out_880135986756125919[5] = 0;
   out_880135986756125919[6] = 0;
   out_880135986756125919[7] = 0;
   out_880135986756125919[8] = 0;
}
void h_26(double *state, double *unused, double *out_4617532990992185702) {
   out_4617532990992185702[0] = state[7];
}
void H_26(double *state, double *unused, double *out_8668686400100004556) {
   out_8668686400100004556[0] = 0;
   out_8668686400100004556[1] = 0;
   out_8668686400100004556[2] = 0;
   out_8668686400100004556[3] = 0;
   out_8668686400100004556[4] = 0;
   out_8668686400100004556[5] = 0;
   out_8668686400100004556[6] = 0;
   out_8668686400100004556[7] = 1;
   out_8668686400100004556[8] = 0;
}
void h_27(double *state, double *unused, double *out_6901189188015885833) {
   out_6901189188015885833[0] = state[3];
}
void H_27(double *state, double *unused, double *out_1294627325044298992) {
   out_1294627325044298992[0] = 0;
   out_1294627325044298992[1] = 0;
   out_1294627325044298992[2] = 0;
   out_1294627325044298992[3] = 1;
   out_1294627325044298992[4] = 0;
   out_1294627325044298992[5] = 0;
   out_1294627325044298992[6] = 0;
   out_1294627325044298992[7] = 0;
   out_1294627325044298992[8] = 0;
}
void h_29(double *state, double *unused, double *out_8597828895338180822) {
   out_8597828895338180822[0] = state[1];
}
void H_29(double *state, double *unused, double *out_1390367331070518103) {
   out_1390367331070518103[0] = 0;
   out_1390367331070518103[1] = 1;
   out_1390367331070518103[2] = 0;
   out_1390367331070518103[3] = 0;
   out_1390367331070518103[4] = 0;
   out_1390367331070518103[5] = 0;
   out_1390367331070518103[6] = 0;
   out_1390367331070518103[7] = 0;
   out_1390367331070518103[8] = 0;
}
void h_28(double *state, double *unused, double *out_3864403136571553148) {
   out_3864403136571553148[0] = state[0];
}
void H_28(double *state, double *unused, double *out_3692031685999012471) {
   out_3692031685999012471[0] = 1;
   out_3692031685999012471[1] = 0;
   out_3692031685999012471[2] = 0;
   out_3692031685999012471[3] = 0;
   out_3692031685999012471[4] = 0;
   out_3692031685999012471[5] = 0;
   out_3692031685999012471[6] = 0;
   out_3692031685999012471[7] = 0;
   out_3692031685999012471[8] = 0;
}
void h_31(double *state, double *unused, double *out_1962167556073461464) {
   out_1962167556073461464[0] = state[8];
}
void H_31(double *state, double *unused, double *out_6005908392858530408) {
   out_6005908392858530408[0] = 0;
   out_6005908392858530408[1] = 0;
   out_6005908392858530408[2] = 0;
   out_6005908392858530408[3] = 0;
   out_6005908392858530408[4] = 0;
   out_6005908392858530408[5] = 0;
   out_6005908392858530408[6] = 0;
   out_6005908392858530408[7] = 0;
   out_6005908392858530408[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_6978687084586341298) {
  err_fun(nom_x, delta_x, out_6978687084586341298);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8854305474282830070) {
  inv_err_fun(nom_x, true_x, out_8854305474282830070);
}
void car_H_mod_fun(double *state, double *out_3156633871176746401) {
  H_mod_fun(state, out_3156633871176746401);
}
void car_f_fun(double *state, double dt, double *out_4802494839670766233) {
  f_fun(state,  dt, out_4802494839670766233);
}
void car_F_fun(double *state, double dt, double *out_5891574291073418751) {
  F_fun(state,  dt, out_5891574291073418751);
}
void car_h_25(double *state, double *unused, double *out_985558434282023497) {
  h_25(state, unused, out_985558434282023497);
}
void car_H_25(double *state, double *unused, double *out_6036554354735490836) {
  H_25(state, unused, out_6036554354735490836);
}
void car_h_24(double *state, double *unused, double *out_1166412536478823260) {
  h_24(state, unused, out_1166412536478823260);
}
void car_H_24(double *state, double *unused, double *out_225168723712713871) {
  H_24(state, unused, out_225168723712713871);
}
void car_h_30(double *state, double *unused, double *out_192172075693954169) {
  h_30(state, unused, out_192172075693954169);
}
void car_H_30(double *state, double *unused, double *out_880135986756125919) {
  H_30(state, unused, out_880135986756125919);
}
void car_h_26(double *state, double *unused, double *out_4617532990992185702) {
  h_26(state, unused, out_4617532990992185702);
}
void car_H_26(double *state, double *unused, double *out_8668686400100004556) {
  H_26(state, unused, out_8668686400100004556);
}
void car_h_27(double *state, double *unused, double *out_6901189188015885833) {
  h_27(state, unused, out_6901189188015885833);
}
void car_H_27(double *state, double *unused, double *out_1294627325044298992) {
  H_27(state, unused, out_1294627325044298992);
}
void car_h_29(double *state, double *unused, double *out_8597828895338180822) {
  h_29(state, unused, out_8597828895338180822);
}
void car_H_29(double *state, double *unused, double *out_1390367331070518103) {
  H_29(state, unused, out_1390367331070518103);
}
void car_h_28(double *state, double *unused, double *out_3864403136571553148) {
  h_28(state, unused, out_3864403136571553148);
}
void car_H_28(double *state, double *unused, double *out_3692031685999012471) {
  H_28(state, unused, out_3692031685999012471);
}
void car_h_31(double *state, double *unused, double *out_1962167556073461464) {
  h_31(state, unused, out_1962167556073461464);
}
void car_H_31(double *state, double *unused, double *out_6005908392858530408) {
  H_31(state, unused, out_6005908392858530408);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)
