#include "pose.h"

namespace {
#define DIM 18
#define EDIM 18
#define MEDIM 18
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_4 = 7.814727903251177;
const static double MAHA_THRESH_10 = 7.814727903251177;
const static double MAHA_THRESH_13 = 7.814727903251177;
const static double MAHA_THRESH_14 = 7.814727903251177;

/******************************************************************************
 *                      Code generated with SymPy 1.13.2                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_7424980701830897997) {
   out_7424980701830897997[0] = delta_x[0] + nom_x[0];
   out_7424980701830897997[1] = delta_x[1] + nom_x[1];
   out_7424980701830897997[2] = delta_x[2] + nom_x[2];
   out_7424980701830897997[3] = delta_x[3] + nom_x[3];
   out_7424980701830897997[4] = delta_x[4] + nom_x[4];
   out_7424980701830897997[5] = delta_x[5] + nom_x[5];
   out_7424980701830897997[6] = delta_x[6] + nom_x[6];
   out_7424980701830897997[7] = delta_x[7] + nom_x[7];
   out_7424980701830897997[8] = delta_x[8] + nom_x[8];
   out_7424980701830897997[9] = delta_x[9] + nom_x[9];
   out_7424980701830897997[10] = delta_x[10] + nom_x[10];
   out_7424980701830897997[11] = delta_x[11] + nom_x[11];
   out_7424980701830897997[12] = delta_x[12] + nom_x[12];
   out_7424980701830897997[13] = delta_x[13] + nom_x[13];
   out_7424980701830897997[14] = delta_x[14] + nom_x[14];
   out_7424980701830897997[15] = delta_x[15] + nom_x[15];
   out_7424980701830897997[16] = delta_x[16] + nom_x[16];
   out_7424980701830897997[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8948729870032527372) {
   out_8948729870032527372[0] = -nom_x[0] + true_x[0];
   out_8948729870032527372[1] = -nom_x[1] + true_x[1];
   out_8948729870032527372[2] = -nom_x[2] + true_x[2];
   out_8948729870032527372[3] = -nom_x[3] + true_x[3];
   out_8948729870032527372[4] = -nom_x[4] + true_x[4];
   out_8948729870032527372[5] = -nom_x[5] + true_x[5];
   out_8948729870032527372[6] = -nom_x[6] + true_x[6];
   out_8948729870032527372[7] = -nom_x[7] + true_x[7];
   out_8948729870032527372[8] = -nom_x[8] + true_x[8];
   out_8948729870032527372[9] = -nom_x[9] + true_x[9];
   out_8948729870032527372[10] = -nom_x[10] + true_x[10];
   out_8948729870032527372[11] = -nom_x[11] + true_x[11];
   out_8948729870032527372[12] = -nom_x[12] + true_x[12];
   out_8948729870032527372[13] = -nom_x[13] + true_x[13];
   out_8948729870032527372[14] = -nom_x[14] + true_x[14];
   out_8948729870032527372[15] = -nom_x[15] + true_x[15];
   out_8948729870032527372[16] = -nom_x[16] + true_x[16];
   out_8948729870032527372[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_7614750544922110094) {
   out_7614750544922110094[0] = 1.0;
   out_7614750544922110094[1] = 0.0;
   out_7614750544922110094[2] = 0.0;
   out_7614750544922110094[3] = 0.0;
   out_7614750544922110094[4] = 0.0;
   out_7614750544922110094[5] = 0.0;
   out_7614750544922110094[6] = 0.0;
   out_7614750544922110094[7] = 0.0;
   out_7614750544922110094[8] = 0.0;
   out_7614750544922110094[9] = 0.0;
   out_7614750544922110094[10] = 0.0;
   out_7614750544922110094[11] = 0.0;
   out_7614750544922110094[12] = 0.0;
   out_7614750544922110094[13] = 0.0;
   out_7614750544922110094[14] = 0.0;
   out_7614750544922110094[15] = 0.0;
   out_7614750544922110094[16] = 0.0;
   out_7614750544922110094[17] = 0.0;
   out_7614750544922110094[18] = 0.0;
   out_7614750544922110094[19] = 1.0;
   out_7614750544922110094[20] = 0.0;
   out_7614750544922110094[21] = 0.0;
   out_7614750544922110094[22] = 0.0;
   out_7614750544922110094[23] = 0.0;
   out_7614750544922110094[24] = 0.0;
   out_7614750544922110094[25] = 0.0;
   out_7614750544922110094[26] = 0.0;
   out_7614750544922110094[27] = 0.0;
   out_7614750544922110094[28] = 0.0;
   out_7614750544922110094[29] = 0.0;
   out_7614750544922110094[30] = 0.0;
   out_7614750544922110094[31] = 0.0;
   out_7614750544922110094[32] = 0.0;
   out_7614750544922110094[33] = 0.0;
   out_7614750544922110094[34] = 0.0;
   out_7614750544922110094[35] = 0.0;
   out_7614750544922110094[36] = 0.0;
   out_7614750544922110094[37] = 0.0;
   out_7614750544922110094[38] = 1.0;
   out_7614750544922110094[39] = 0.0;
   out_7614750544922110094[40] = 0.0;
   out_7614750544922110094[41] = 0.0;
   out_7614750544922110094[42] = 0.0;
   out_7614750544922110094[43] = 0.0;
   out_7614750544922110094[44] = 0.0;
   out_7614750544922110094[45] = 0.0;
   out_7614750544922110094[46] = 0.0;
   out_7614750544922110094[47] = 0.0;
   out_7614750544922110094[48] = 0.0;
   out_7614750544922110094[49] = 0.0;
   out_7614750544922110094[50] = 0.0;
   out_7614750544922110094[51] = 0.0;
   out_7614750544922110094[52] = 0.0;
   out_7614750544922110094[53] = 0.0;
   out_7614750544922110094[54] = 0.0;
   out_7614750544922110094[55] = 0.0;
   out_7614750544922110094[56] = 0.0;
   out_7614750544922110094[57] = 1.0;
   out_7614750544922110094[58] = 0.0;
   out_7614750544922110094[59] = 0.0;
   out_7614750544922110094[60] = 0.0;
   out_7614750544922110094[61] = 0.0;
   out_7614750544922110094[62] = 0.0;
   out_7614750544922110094[63] = 0.0;
   out_7614750544922110094[64] = 0.0;
   out_7614750544922110094[65] = 0.0;
   out_7614750544922110094[66] = 0.0;
   out_7614750544922110094[67] = 0.0;
   out_7614750544922110094[68] = 0.0;
   out_7614750544922110094[69] = 0.0;
   out_7614750544922110094[70] = 0.0;
   out_7614750544922110094[71] = 0.0;
   out_7614750544922110094[72] = 0.0;
   out_7614750544922110094[73] = 0.0;
   out_7614750544922110094[74] = 0.0;
   out_7614750544922110094[75] = 0.0;
   out_7614750544922110094[76] = 1.0;
   out_7614750544922110094[77] = 0.0;
   out_7614750544922110094[78] = 0.0;
   out_7614750544922110094[79] = 0.0;
   out_7614750544922110094[80] = 0.0;
   out_7614750544922110094[81] = 0.0;
   out_7614750544922110094[82] = 0.0;
   out_7614750544922110094[83] = 0.0;
   out_7614750544922110094[84] = 0.0;
   out_7614750544922110094[85] = 0.0;
   out_7614750544922110094[86] = 0.0;
   out_7614750544922110094[87] = 0.0;
   out_7614750544922110094[88] = 0.0;
   out_7614750544922110094[89] = 0.0;
   out_7614750544922110094[90] = 0.0;
   out_7614750544922110094[91] = 0.0;
   out_7614750544922110094[92] = 0.0;
   out_7614750544922110094[93] = 0.0;
   out_7614750544922110094[94] = 0.0;
   out_7614750544922110094[95] = 1.0;
   out_7614750544922110094[96] = 0.0;
   out_7614750544922110094[97] = 0.0;
   out_7614750544922110094[98] = 0.0;
   out_7614750544922110094[99] = 0.0;
   out_7614750544922110094[100] = 0.0;
   out_7614750544922110094[101] = 0.0;
   out_7614750544922110094[102] = 0.0;
   out_7614750544922110094[103] = 0.0;
   out_7614750544922110094[104] = 0.0;
   out_7614750544922110094[105] = 0.0;
   out_7614750544922110094[106] = 0.0;
   out_7614750544922110094[107] = 0.0;
   out_7614750544922110094[108] = 0.0;
   out_7614750544922110094[109] = 0.0;
   out_7614750544922110094[110] = 0.0;
   out_7614750544922110094[111] = 0.0;
   out_7614750544922110094[112] = 0.0;
   out_7614750544922110094[113] = 0.0;
   out_7614750544922110094[114] = 1.0;
   out_7614750544922110094[115] = 0.0;
   out_7614750544922110094[116] = 0.0;
   out_7614750544922110094[117] = 0.0;
   out_7614750544922110094[118] = 0.0;
   out_7614750544922110094[119] = 0.0;
   out_7614750544922110094[120] = 0.0;
   out_7614750544922110094[121] = 0.0;
   out_7614750544922110094[122] = 0.0;
   out_7614750544922110094[123] = 0.0;
   out_7614750544922110094[124] = 0.0;
   out_7614750544922110094[125] = 0.0;
   out_7614750544922110094[126] = 0.0;
   out_7614750544922110094[127] = 0.0;
   out_7614750544922110094[128] = 0.0;
   out_7614750544922110094[129] = 0.0;
   out_7614750544922110094[130] = 0.0;
   out_7614750544922110094[131] = 0.0;
   out_7614750544922110094[132] = 0.0;
   out_7614750544922110094[133] = 1.0;
   out_7614750544922110094[134] = 0.0;
   out_7614750544922110094[135] = 0.0;
   out_7614750544922110094[136] = 0.0;
   out_7614750544922110094[137] = 0.0;
   out_7614750544922110094[138] = 0.0;
   out_7614750544922110094[139] = 0.0;
   out_7614750544922110094[140] = 0.0;
   out_7614750544922110094[141] = 0.0;
   out_7614750544922110094[142] = 0.0;
   out_7614750544922110094[143] = 0.0;
   out_7614750544922110094[144] = 0.0;
   out_7614750544922110094[145] = 0.0;
   out_7614750544922110094[146] = 0.0;
   out_7614750544922110094[147] = 0.0;
   out_7614750544922110094[148] = 0.0;
   out_7614750544922110094[149] = 0.0;
   out_7614750544922110094[150] = 0.0;
   out_7614750544922110094[151] = 0.0;
   out_7614750544922110094[152] = 1.0;
   out_7614750544922110094[153] = 0.0;
   out_7614750544922110094[154] = 0.0;
   out_7614750544922110094[155] = 0.0;
   out_7614750544922110094[156] = 0.0;
   out_7614750544922110094[157] = 0.0;
   out_7614750544922110094[158] = 0.0;
   out_7614750544922110094[159] = 0.0;
   out_7614750544922110094[160] = 0.0;
   out_7614750544922110094[161] = 0.0;
   out_7614750544922110094[162] = 0.0;
   out_7614750544922110094[163] = 0.0;
   out_7614750544922110094[164] = 0.0;
   out_7614750544922110094[165] = 0.0;
   out_7614750544922110094[166] = 0.0;
   out_7614750544922110094[167] = 0.0;
   out_7614750544922110094[168] = 0.0;
   out_7614750544922110094[169] = 0.0;
   out_7614750544922110094[170] = 0.0;
   out_7614750544922110094[171] = 1.0;
   out_7614750544922110094[172] = 0.0;
   out_7614750544922110094[173] = 0.0;
   out_7614750544922110094[174] = 0.0;
   out_7614750544922110094[175] = 0.0;
   out_7614750544922110094[176] = 0.0;
   out_7614750544922110094[177] = 0.0;
   out_7614750544922110094[178] = 0.0;
   out_7614750544922110094[179] = 0.0;
   out_7614750544922110094[180] = 0.0;
   out_7614750544922110094[181] = 0.0;
   out_7614750544922110094[182] = 0.0;
   out_7614750544922110094[183] = 0.0;
   out_7614750544922110094[184] = 0.0;
   out_7614750544922110094[185] = 0.0;
   out_7614750544922110094[186] = 0.0;
   out_7614750544922110094[187] = 0.0;
   out_7614750544922110094[188] = 0.0;
   out_7614750544922110094[189] = 0.0;
   out_7614750544922110094[190] = 1.0;
   out_7614750544922110094[191] = 0.0;
   out_7614750544922110094[192] = 0.0;
   out_7614750544922110094[193] = 0.0;
   out_7614750544922110094[194] = 0.0;
   out_7614750544922110094[195] = 0.0;
   out_7614750544922110094[196] = 0.0;
   out_7614750544922110094[197] = 0.0;
   out_7614750544922110094[198] = 0.0;
   out_7614750544922110094[199] = 0.0;
   out_7614750544922110094[200] = 0.0;
   out_7614750544922110094[201] = 0.0;
   out_7614750544922110094[202] = 0.0;
   out_7614750544922110094[203] = 0.0;
   out_7614750544922110094[204] = 0.0;
   out_7614750544922110094[205] = 0.0;
   out_7614750544922110094[206] = 0.0;
   out_7614750544922110094[207] = 0.0;
   out_7614750544922110094[208] = 0.0;
   out_7614750544922110094[209] = 1.0;
   out_7614750544922110094[210] = 0.0;
   out_7614750544922110094[211] = 0.0;
   out_7614750544922110094[212] = 0.0;
   out_7614750544922110094[213] = 0.0;
   out_7614750544922110094[214] = 0.0;
   out_7614750544922110094[215] = 0.0;
   out_7614750544922110094[216] = 0.0;
   out_7614750544922110094[217] = 0.0;
   out_7614750544922110094[218] = 0.0;
   out_7614750544922110094[219] = 0.0;
   out_7614750544922110094[220] = 0.0;
   out_7614750544922110094[221] = 0.0;
   out_7614750544922110094[222] = 0.0;
   out_7614750544922110094[223] = 0.0;
   out_7614750544922110094[224] = 0.0;
   out_7614750544922110094[225] = 0.0;
   out_7614750544922110094[226] = 0.0;
   out_7614750544922110094[227] = 0.0;
   out_7614750544922110094[228] = 1.0;
   out_7614750544922110094[229] = 0.0;
   out_7614750544922110094[230] = 0.0;
   out_7614750544922110094[231] = 0.0;
   out_7614750544922110094[232] = 0.0;
   out_7614750544922110094[233] = 0.0;
   out_7614750544922110094[234] = 0.0;
   out_7614750544922110094[235] = 0.0;
   out_7614750544922110094[236] = 0.0;
   out_7614750544922110094[237] = 0.0;
   out_7614750544922110094[238] = 0.0;
   out_7614750544922110094[239] = 0.0;
   out_7614750544922110094[240] = 0.0;
   out_7614750544922110094[241] = 0.0;
   out_7614750544922110094[242] = 0.0;
   out_7614750544922110094[243] = 0.0;
   out_7614750544922110094[244] = 0.0;
   out_7614750544922110094[245] = 0.0;
   out_7614750544922110094[246] = 0.0;
   out_7614750544922110094[247] = 1.0;
   out_7614750544922110094[248] = 0.0;
   out_7614750544922110094[249] = 0.0;
   out_7614750544922110094[250] = 0.0;
   out_7614750544922110094[251] = 0.0;
   out_7614750544922110094[252] = 0.0;
   out_7614750544922110094[253] = 0.0;
   out_7614750544922110094[254] = 0.0;
   out_7614750544922110094[255] = 0.0;
   out_7614750544922110094[256] = 0.0;
   out_7614750544922110094[257] = 0.0;
   out_7614750544922110094[258] = 0.0;
   out_7614750544922110094[259] = 0.0;
   out_7614750544922110094[260] = 0.0;
   out_7614750544922110094[261] = 0.0;
   out_7614750544922110094[262] = 0.0;
   out_7614750544922110094[263] = 0.0;
   out_7614750544922110094[264] = 0.0;
   out_7614750544922110094[265] = 0.0;
   out_7614750544922110094[266] = 1.0;
   out_7614750544922110094[267] = 0.0;
   out_7614750544922110094[268] = 0.0;
   out_7614750544922110094[269] = 0.0;
   out_7614750544922110094[270] = 0.0;
   out_7614750544922110094[271] = 0.0;
   out_7614750544922110094[272] = 0.0;
   out_7614750544922110094[273] = 0.0;
   out_7614750544922110094[274] = 0.0;
   out_7614750544922110094[275] = 0.0;
   out_7614750544922110094[276] = 0.0;
   out_7614750544922110094[277] = 0.0;
   out_7614750544922110094[278] = 0.0;
   out_7614750544922110094[279] = 0.0;
   out_7614750544922110094[280] = 0.0;
   out_7614750544922110094[281] = 0.0;
   out_7614750544922110094[282] = 0.0;
   out_7614750544922110094[283] = 0.0;
   out_7614750544922110094[284] = 0.0;
   out_7614750544922110094[285] = 1.0;
   out_7614750544922110094[286] = 0.0;
   out_7614750544922110094[287] = 0.0;
   out_7614750544922110094[288] = 0.0;
   out_7614750544922110094[289] = 0.0;
   out_7614750544922110094[290] = 0.0;
   out_7614750544922110094[291] = 0.0;
   out_7614750544922110094[292] = 0.0;
   out_7614750544922110094[293] = 0.0;
   out_7614750544922110094[294] = 0.0;
   out_7614750544922110094[295] = 0.0;
   out_7614750544922110094[296] = 0.0;
   out_7614750544922110094[297] = 0.0;
   out_7614750544922110094[298] = 0.0;
   out_7614750544922110094[299] = 0.0;
   out_7614750544922110094[300] = 0.0;
   out_7614750544922110094[301] = 0.0;
   out_7614750544922110094[302] = 0.0;
   out_7614750544922110094[303] = 0.0;
   out_7614750544922110094[304] = 1.0;
   out_7614750544922110094[305] = 0.0;
   out_7614750544922110094[306] = 0.0;
   out_7614750544922110094[307] = 0.0;
   out_7614750544922110094[308] = 0.0;
   out_7614750544922110094[309] = 0.0;
   out_7614750544922110094[310] = 0.0;
   out_7614750544922110094[311] = 0.0;
   out_7614750544922110094[312] = 0.0;
   out_7614750544922110094[313] = 0.0;
   out_7614750544922110094[314] = 0.0;
   out_7614750544922110094[315] = 0.0;
   out_7614750544922110094[316] = 0.0;
   out_7614750544922110094[317] = 0.0;
   out_7614750544922110094[318] = 0.0;
   out_7614750544922110094[319] = 0.0;
   out_7614750544922110094[320] = 0.0;
   out_7614750544922110094[321] = 0.0;
   out_7614750544922110094[322] = 0.0;
   out_7614750544922110094[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_4078122102263429604) {
   out_4078122102263429604[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_4078122102263429604[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_4078122102263429604[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_4078122102263429604[3] = dt*state[12] + state[3];
   out_4078122102263429604[4] = dt*state[13] + state[4];
   out_4078122102263429604[5] = dt*state[14] + state[5];
   out_4078122102263429604[6] = state[6];
   out_4078122102263429604[7] = state[7];
   out_4078122102263429604[8] = state[8];
   out_4078122102263429604[9] = state[9];
   out_4078122102263429604[10] = state[10];
   out_4078122102263429604[11] = state[11];
   out_4078122102263429604[12] = state[12];
   out_4078122102263429604[13] = state[13];
   out_4078122102263429604[14] = state[14];
   out_4078122102263429604[15] = state[15];
   out_4078122102263429604[16] = state[16];
   out_4078122102263429604[17] = state[17];
}
void F_fun(double *state, double dt, double *out_8150619080602528479) {
   out_8150619080602528479[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8150619080602528479[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8150619080602528479[2] = 0;
   out_8150619080602528479[3] = 0;
   out_8150619080602528479[4] = 0;
   out_8150619080602528479[5] = 0;
   out_8150619080602528479[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8150619080602528479[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8150619080602528479[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8150619080602528479[9] = 0;
   out_8150619080602528479[10] = 0;
   out_8150619080602528479[11] = 0;
   out_8150619080602528479[12] = 0;
   out_8150619080602528479[13] = 0;
   out_8150619080602528479[14] = 0;
   out_8150619080602528479[15] = 0;
   out_8150619080602528479[16] = 0;
   out_8150619080602528479[17] = 0;
   out_8150619080602528479[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_8150619080602528479[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_8150619080602528479[20] = 0;
   out_8150619080602528479[21] = 0;
   out_8150619080602528479[22] = 0;
   out_8150619080602528479[23] = 0;
   out_8150619080602528479[24] = 0;
   out_8150619080602528479[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_8150619080602528479[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_8150619080602528479[27] = 0;
   out_8150619080602528479[28] = 0;
   out_8150619080602528479[29] = 0;
   out_8150619080602528479[30] = 0;
   out_8150619080602528479[31] = 0;
   out_8150619080602528479[32] = 0;
   out_8150619080602528479[33] = 0;
   out_8150619080602528479[34] = 0;
   out_8150619080602528479[35] = 0;
   out_8150619080602528479[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8150619080602528479[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8150619080602528479[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8150619080602528479[39] = 0;
   out_8150619080602528479[40] = 0;
   out_8150619080602528479[41] = 0;
   out_8150619080602528479[42] = 0;
   out_8150619080602528479[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8150619080602528479[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8150619080602528479[45] = 0;
   out_8150619080602528479[46] = 0;
   out_8150619080602528479[47] = 0;
   out_8150619080602528479[48] = 0;
   out_8150619080602528479[49] = 0;
   out_8150619080602528479[50] = 0;
   out_8150619080602528479[51] = 0;
   out_8150619080602528479[52] = 0;
   out_8150619080602528479[53] = 0;
   out_8150619080602528479[54] = 0;
   out_8150619080602528479[55] = 0;
   out_8150619080602528479[56] = 0;
   out_8150619080602528479[57] = 1;
   out_8150619080602528479[58] = 0;
   out_8150619080602528479[59] = 0;
   out_8150619080602528479[60] = 0;
   out_8150619080602528479[61] = 0;
   out_8150619080602528479[62] = 0;
   out_8150619080602528479[63] = 0;
   out_8150619080602528479[64] = 0;
   out_8150619080602528479[65] = 0;
   out_8150619080602528479[66] = dt;
   out_8150619080602528479[67] = 0;
   out_8150619080602528479[68] = 0;
   out_8150619080602528479[69] = 0;
   out_8150619080602528479[70] = 0;
   out_8150619080602528479[71] = 0;
   out_8150619080602528479[72] = 0;
   out_8150619080602528479[73] = 0;
   out_8150619080602528479[74] = 0;
   out_8150619080602528479[75] = 0;
   out_8150619080602528479[76] = 1;
   out_8150619080602528479[77] = 0;
   out_8150619080602528479[78] = 0;
   out_8150619080602528479[79] = 0;
   out_8150619080602528479[80] = 0;
   out_8150619080602528479[81] = 0;
   out_8150619080602528479[82] = 0;
   out_8150619080602528479[83] = 0;
   out_8150619080602528479[84] = 0;
   out_8150619080602528479[85] = dt;
   out_8150619080602528479[86] = 0;
   out_8150619080602528479[87] = 0;
   out_8150619080602528479[88] = 0;
   out_8150619080602528479[89] = 0;
   out_8150619080602528479[90] = 0;
   out_8150619080602528479[91] = 0;
   out_8150619080602528479[92] = 0;
   out_8150619080602528479[93] = 0;
   out_8150619080602528479[94] = 0;
   out_8150619080602528479[95] = 1;
   out_8150619080602528479[96] = 0;
   out_8150619080602528479[97] = 0;
   out_8150619080602528479[98] = 0;
   out_8150619080602528479[99] = 0;
   out_8150619080602528479[100] = 0;
   out_8150619080602528479[101] = 0;
   out_8150619080602528479[102] = 0;
   out_8150619080602528479[103] = 0;
   out_8150619080602528479[104] = dt;
   out_8150619080602528479[105] = 0;
   out_8150619080602528479[106] = 0;
   out_8150619080602528479[107] = 0;
   out_8150619080602528479[108] = 0;
   out_8150619080602528479[109] = 0;
   out_8150619080602528479[110] = 0;
   out_8150619080602528479[111] = 0;
   out_8150619080602528479[112] = 0;
   out_8150619080602528479[113] = 0;
   out_8150619080602528479[114] = 1;
   out_8150619080602528479[115] = 0;
   out_8150619080602528479[116] = 0;
   out_8150619080602528479[117] = 0;
   out_8150619080602528479[118] = 0;
   out_8150619080602528479[119] = 0;
   out_8150619080602528479[120] = 0;
   out_8150619080602528479[121] = 0;
   out_8150619080602528479[122] = 0;
   out_8150619080602528479[123] = 0;
   out_8150619080602528479[124] = 0;
   out_8150619080602528479[125] = 0;
   out_8150619080602528479[126] = 0;
   out_8150619080602528479[127] = 0;
   out_8150619080602528479[128] = 0;
   out_8150619080602528479[129] = 0;
   out_8150619080602528479[130] = 0;
   out_8150619080602528479[131] = 0;
   out_8150619080602528479[132] = 0;
   out_8150619080602528479[133] = 1;
   out_8150619080602528479[134] = 0;
   out_8150619080602528479[135] = 0;
   out_8150619080602528479[136] = 0;
   out_8150619080602528479[137] = 0;
   out_8150619080602528479[138] = 0;
   out_8150619080602528479[139] = 0;
   out_8150619080602528479[140] = 0;
   out_8150619080602528479[141] = 0;
   out_8150619080602528479[142] = 0;
   out_8150619080602528479[143] = 0;
   out_8150619080602528479[144] = 0;
   out_8150619080602528479[145] = 0;
   out_8150619080602528479[146] = 0;
   out_8150619080602528479[147] = 0;
   out_8150619080602528479[148] = 0;
   out_8150619080602528479[149] = 0;
   out_8150619080602528479[150] = 0;
   out_8150619080602528479[151] = 0;
   out_8150619080602528479[152] = 1;
   out_8150619080602528479[153] = 0;
   out_8150619080602528479[154] = 0;
   out_8150619080602528479[155] = 0;
   out_8150619080602528479[156] = 0;
   out_8150619080602528479[157] = 0;
   out_8150619080602528479[158] = 0;
   out_8150619080602528479[159] = 0;
   out_8150619080602528479[160] = 0;
   out_8150619080602528479[161] = 0;
   out_8150619080602528479[162] = 0;
   out_8150619080602528479[163] = 0;
   out_8150619080602528479[164] = 0;
   out_8150619080602528479[165] = 0;
   out_8150619080602528479[166] = 0;
   out_8150619080602528479[167] = 0;
   out_8150619080602528479[168] = 0;
   out_8150619080602528479[169] = 0;
   out_8150619080602528479[170] = 0;
   out_8150619080602528479[171] = 1;
   out_8150619080602528479[172] = 0;
   out_8150619080602528479[173] = 0;
   out_8150619080602528479[174] = 0;
   out_8150619080602528479[175] = 0;
   out_8150619080602528479[176] = 0;
   out_8150619080602528479[177] = 0;
   out_8150619080602528479[178] = 0;
   out_8150619080602528479[179] = 0;
   out_8150619080602528479[180] = 0;
   out_8150619080602528479[181] = 0;
   out_8150619080602528479[182] = 0;
   out_8150619080602528479[183] = 0;
   out_8150619080602528479[184] = 0;
   out_8150619080602528479[185] = 0;
   out_8150619080602528479[186] = 0;
   out_8150619080602528479[187] = 0;
   out_8150619080602528479[188] = 0;
   out_8150619080602528479[189] = 0;
   out_8150619080602528479[190] = 1;
   out_8150619080602528479[191] = 0;
   out_8150619080602528479[192] = 0;
   out_8150619080602528479[193] = 0;
   out_8150619080602528479[194] = 0;
   out_8150619080602528479[195] = 0;
   out_8150619080602528479[196] = 0;
   out_8150619080602528479[197] = 0;
   out_8150619080602528479[198] = 0;
   out_8150619080602528479[199] = 0;
   out_8150619080602528479[200] = 0;
   out_8150619080602528479[201] = 0;
   out_8150619080602528479[202] = 0;
   out_8150619080602528479[203] = 0;
   out_8150619080602528479[204] = 0;
   out_8150619080602528479[205] = 0;
   out_8150619080602528479[206] = 0;
   out_8150619080602528479[207] = 0;
   out_8150619080602528479[208] = 0;
   out_8150619080602528479[209] = 1;
   out_8150619080602528479[210] = 0;
   out_8150619080602528479[211] = 0;
   out_8150619080602528479[212] = 0;
   out_8150619080602528479[213] = 0;
   out_8150619080602528479[214] = 0;
   out_8150619080602528479[215] = 0;
   out_8150619080602528479[216] = 0;
   out_8150619080602528479[217] = 0;
   out_8150619080602528479[218] = 0;
   out_8150619080602528479[219] = 0;
   out_8150619080602528479[220] = 0;
   out_8150619080602528479[221] = 0;
   out_8150619080602528479[222] = 0;
   out_8150619080602528479[223] = 0;
   out_8150619080602528479[224] = 0;
   out_8150619080602528479[225] = 0;
   out_8150619080602528479[226] = 0;
   out_8150619080602528479[227] = 0;
   out_8150619080602528479[228] = 1;
   out_8150619080602528479[229] = 0;
   out_8150619080602528479[230] = 0;
   out_8150619080602528479[231] = 0;
   out_8150619080602528479[232] = 0;
   out_8150619080602528479[233] = 0;
   out_8150619080602528479[234] = 0;
   out_8150619080602528479[235] = 0;
   out_8150619080602528479[236] = 0;
   out_8150619080602528479[237] = 0;
   out_8150619080602528479[238] = 0;
   out_8150619080602528479[239] = 0;
   out_8150619080602528479[240] = 0;
   out_8150619080602528479[241] = 0;
   out_8150619080602528479[242] = 0;
   out_8150619080602528479[243] = 0;
   out_8150619080602528479[244] = 0;
   out_8150619080602528479[245] = 0;
   out_8150619080602528479[246] = 0;
   out_8150619080602528479[247] = 1;
   out_8150619080602528479[248] = 0;
   out_8150619080602528479[249] = 0;
   out_8150619080602528479[250] = 0;
   out_8150619080602528479[251] = 0;
   out_8150619080602528479[252] = 0;
   out_8150619080602528479[253] = 0;
   out_8150619080602528479[254] = 0;
   out_8150619080602528479[255] = 0;
   out_8150619080602528479[256] = 0;
   out_8150619080602528479[257] = 0;
   out_8150619080602528479[258] = 0;
   out_8150619080602528479[259] = 0;
   out_8150619080602528479[260] = 0;
   out_8150619080602528479[261] = 0;
   out_8150619080602528479[262] = 0;
   out_8150619080602528479[263] = 0;
   out_8150619080602528479[264] = 0;
   out_8150619080602528479[265] = 0;
   out_8150619080602528479[266] = 1;
   out_8150619080602528479[267] = 0;
   out_8150619080602528479[268] = 0;
   out_8150619080602528479[269] = 0;
   out_8150619080602528479[270] = 0;
   out_8150619080602528479[271] = 0;
   out_8150619080602528479[272] = 0;
   out_8150619080602528479[273] = 0;
   out_8150619080602528479[274] = 0;
   out_8150619080602528479[275] = 0;
   out_8150619080602528479[276] = 0;
   out_8150619080602528479[277] = 0;
   out_8150619080602528479[278] = 0;
   out_8150619080602528479[279] = 0;
   out_8150619080602528479[280] = 0;
   out_8150619080602528479[281] = 0;
   out_8150619080602528479[282] = 0;
   out_8150619080602528479[283] = 0;
   out_8150619080602528479[284] = 0;
   out_8150619080602528479[285] = 1;
   out_8150619080602528479[286] = 0;
   out_8150619080602528479[287] = 0;
   out_8150619080602528479[288] = 0;
   out_8150619080602528479[289] = 0;
   out_8150619080602528479[290] = 0;
   out_8150619080602528479[291] = 0;
   out_8150619080602528479[292] = 0;
   out_8150619080602528479[293] = 0;
   out_8150619080602528479[294] = 0;
   out_8150619080602528479[295] = 0;
   out_8150619080602528479[296] = 0;
   out_8150619080602528479[297] = 0;
   out_8150619080602528479[298] = 0;
   out_8150619080602528479[299] = 0;
   out_8150619080602528479[300] = 0;
   out_8150619080602528479[301] = 0;
   out_8150619080602528479[302] = 0;
   out_8150619080602528479[303] = 0;
   out_8150619080602528479[304] = 1;
   out_8150619080602528479[305] = 0;
   out_8150619080602528479[306] = 0;
   out_8150619080602528479[307] = 0;
   out_8150619080602528479[308] = 0;
   out_8150619080602528479[309] = 0;
   out_8150619080602528479[310] = 0;
   out_8150619080602528479[311] = 0;
   out_8150619080602528479[312] = 0;
   out_8150619080602528479[313] = 0;
   out_8150619080602528479[314] = 0;
   out_8150619080602528479[315] = 0;
   out_8150619080602528479[316] = 0;
   out_8150619080602528479[317] = 0;
   out_8150619080602528479[318] = 0;
   out_8150619080602528479[319] = 0;
   out_8150619080602528479[320] = 0;
   out_8150619080602528479[321] = 0;
   out_8150619080602528479[322] = 0;
   out_8150619080602528479[323] = 1;
}
void h_4(double *state, double *unused, double *out_2389356497093565065) {
   out_2389356497093565065[0] = state[6] + state[9];
   out_2389356497093565065[1] = state[7] + state[10];
   out_2389356497093565065[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_4340528466131160841) {
   out_4340528466131160841[0] = 0;
   out_4340528466131160841[1] = 0;
   out_4340528466131160841[2] = 0;
   out_4340528466131160841[3] = 0;
   out_4340528466131160841[4] = 0;
   out_4340528466131160841[5] = 0;
   out_4340528466131160841[6] = 1;
   out_4340528466131160841[7] = 0;
   out_4340528466131160841[8] = 0;
   out_4340528466131160841[9] = 1;
   out_4340528466131160841[10] = 0;
   out_4340528466131160841[11] = 0;
   out_4340528466131160841[12] = 0;
   out_4340528466131160841[13] = 0;
   out_4340528466131160841[14] = 0;
   out_4340528466131160841[15] = 0;
   out_4340528466131160841[16] = 0;
   out_4340528466131160841[17] = 0;
   out_4340528466131160841[18] = 0;
   out_4340528466131160841[19] = 0;
   out_4340528466131160841[20] = 0;
   out_4340528466131160841[21] = 0;
   out_4340528466131160841[22] = 0;
   out_4340528466131160841[23] = 0;
   out_4340528466131160841[24] = 0;
   out_4340528466131160841[25] = 1;
   out_4340528466131160841[26] = 0;
   out_4340528466131160841[27] = 0;
   out_4340528466131160841[28] = 1;
   out_4340528466131160841[29] = 0;
   out_4340528466131160841[30] = 0;
   out_4340528466131160841[31] = 0;
   out_4340528466131160841[32] = 0;
   out_4340528466131160841[33] = 0;
   out_4340528466131160841[34] = 0;
   out_4340528466131160841[35] = 0;
   out_4340528466131160841[36] = 0;
   out_4340528466131160841[37] = 0;
   out_4340528466131160841[38] = 0;
   out_4340528466131160841[39] = 0;
   out_4340528466131160841[40] = 0;
   out_4340528466131160841[41] = 0;
   out_4340528466131160841[42] = 0;
   out_4340528466131160841[43] = 0;
   out_4340528466131160841[44] = 1;
   out_4340528466131160841[45] = 0;
   out_4340528466131160841[46] = 0;
   out_4340528466131160841[47] = 1;
   out_4340528466131160841[48] = 0;
   out_4340528466131160841[49] = 0;
   out_4340528466131160841[50] = 0;
   out_4340528466131160841[51] = 0;
   out_4340528466131160841[52] = 0;
   out_4340528466131160841[53] = 0;
}
void h_10(double *state, double *unused, double *out_8795577960664591671) {
   out_8795577960664591671[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_8795577960664591671[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_8795577960664591671[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_8489014100523960796) {
   out_8489014100523960796[0] = 0;
   out_8489014100523960796[1] = 9.8100000000000005*cos(state[1]);
   out_8489014100523960796[2] = 0;
   out_8489014100523960796[3] = 0;
   out_8489014100523960796[4] = -state[8];
   out_8489014100523960796[5] = state[7];
   out_8489014100523960796[6] = 0;
   out_8489014100523960796[7] = state[5];
   out_8489014100523960796[8] = -state[4];
   out_8489014100523960796[9] = 0;
   out_8489014100523960796[10] = 0;
   out_8489014100523960796[11] = 0;
   out_8489014100523960796[12] = 1;
   out_8489014100523960796[13] = 0;
   out_8489014100523960796[14] = 0;
   out_8489014100523960796[15] = 1;
   out_8489014100523960796[16] = 0;
   out_8489014100523960796[17] = 0;
   out_8489014100523960796[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_8489014100523960796[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_8489014100523960796[20] = 0;
   out_8489014100523960796[21] = state[8];
   out_8489014100523960796[22] = 0;
   out_8489014100523960796[23] = -state[6];
   out_8489014100523960796[24] = -state[5];
   out_8489014100523960796[25] = 0;
   out_8489014100523960796[26] = state[3];
   out_8489014100523960796[27] = 0;
   out_8489014100523960796[28] = 0;
   out_8489014100523960796[29] = 0;
   out_8489014100523960796[30] = 0;
   out_8489014100523960796[31] = 1;
   out_8489014100523960796[32] = 0;
   out_8489014100523960796[33] = 0;
   out_8489014100523960796[34] = 1;
   out_8489014100523960796[35] = 0;
   out_8489014100523960796[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_8489014100523960796[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_8489014100523960796[38] = 0;
   out_8489014100523960796[39] = -state[7];
   out_8489014100523960796[40] = state[6];
   out_8489014100523960796[41] = 0;
   out_8489014100523960796[42] = state[4];
   out_8489014100523960796[43] = -state[3];
   out_8489014100523960796[44] = 0;
   out_8489014100523960796[45] = 0;
   out_8489014100523960796[46] = 0;
   out_8489014100523960796[47] = 0;
   out_8489014100523960796[48] = 0;
   out_8489014100523960796[49] = 0;
   out_8489014100523960796[50] = 1;
   out_8489014100523960796[51] = 0;
   out_8489014100523960796[52] = 0;
   out_8489014100523960796[53] = 1;
}
void h_13(double *state, double *unused, double *out_2851896194690110696) {
   out_2851896194690110696[0] = state[3];
   out_2851896194690110696[1] = state[4];
   out_2851896194690110696[2] = state[5];
}
void H_13(double *state, double *unused, double *out_6495584399261689846) {
   out_6495584399261689846[0] = 0;
   out_6495584399261689846[1] = 0;
   out_6495584399261689846[2] = 0;
   out_6495584399261689846[3] = 1;
   out_6495584399261689846[4] = 0;
   out_6495584399261689846[5] = 0;
   out_6495584399261689846[6] = 0;
   out_6495584399261689846[7] = 0;
   out_6495584399261689846[8] = 0;
   out_6495584399261689846[9] = 0;
   out_6495584399261689846[10] = 0;
   out_6495584399261689846[11] = 0;
   out_6495584399261689846[12] = 0;
   out_6495584399261689846[13] = 0;
   out_6495584399261689846[14] = 0;
   out_6495584399261689846[15] = 0;
   out_6495584399261689846[16] = 0;
   out_6495584399261689846[17] = 0;
   out_6495584399261689846[18] = 0;
   out_6495584399261689846[19] = 0;
   out_6495584399261689846[20] = 0;
   out_6495584399261689846[21] = 0;
   out_6495584399261689846[22] = 1;
   out_6495584399261689846[23] = 0;
   out_6495584399261689846[24] = 0;
   out_6495584399261689846[25] = 0;
   out_6495584399261689846[26] = 0;
   out_6495584399261689846[27] = 0;
   out_6495584399261689846[28] = 0;
   out_6495584399261689846[29] = 0;
   out_6495584399261689846[30] = 0;
   out_6495584399261689846[31] = 0;
   out_6495584399261689846[32] = 0;
   out_6495584399261689846[33] = 0;
   out_6495584399261689846[34] = 0;
   out_6495584399261689846[35] = 0;
   out_6495584399261689846[36] = 0;
   out_6495584399261689846[37] = 0;
   out_6495584399261689846[38] = 0;
   out_6495584399261689846[39] = 0;
   out_6495584399261689846[40] = 0;
   out_6495584399261689846[41] = 1;
   out_6495584399261689846[42] = 0;
   out_6495584399261689846[43] = 0;
   out_6495584399261689846[44] = 0;
   out_6495584399261689846[45] = 0;
   out_6495584399261689846[46] = 0;
   out_6495584399261689846[47] = 0;
   out_6495584399261689846[48] = 0;
   out_6495584399261689846[49] = 0;
   out_6495584399261689846[50] = 0;
   out_6495584399261689846[51] = 0;
   out_6495584399261689846[52] = 0;
   out_6495584399261689846[53] = 0;
}
void h_14(double *state, double *unused, double *out_7309375191689700906) {
   out_7309375191689700906[0] = state[6];
   out_7309375191689700906[1] = state[7];
   out_7309375191689700906[2] = state[8];
}
void H_14(double *state, double *unused, double *out_8303769322470645370) {
   out_8303769322470645370[0] = 0;
   out_8303769322470645370[1] = 0;
   out_8303769322470645370[2] = 0;
   out_8303769322470645370[3] = 0;
   out_8303769322470645370[4] = 0;
   out_8303769322470645370[5] = 0;
   out_8303769322470645370[6] = 1;
   out_8303769322470645370[7] = 0;
   out_8303769322470645370[8] = 0;
   out_8303769322470645370[9] = 0;
   out_8303769322470645370[10] = 0;
   out_8303769322470645370[11] = 0;
   out_8303769322470645370[12] = 0;
   out_8303769322470645370[13] = 0;
   out_8303769322470645370[14] = 0;
   out_8303769322470645370[15] = 0;
   out_8303769322470645370[16] = 0;
   out_8303769322470645370[17] = 0;
   out_8303769322470645370[18] = 0;
   out_8303769322470645370[19] = 0;
   out_8303769322470645370[20] = 0;
   out_8303769322470645370[21] = 0;
   out_8303769322470645370[22] = 0;
   out_8303769322470645370[23] = 0;
   out_8303769322470645370[24] = 0;
   out_8303769322470645370[25] = 1;
   out_8303769322470645370[26] = 0;
   out_8303769322470645370[27] = 0;
   out_8303769322470645370[28] = 0;
   out_8303769322470645370[29] = 0;
   out_8303769322470645370[30] = 0;
   out_8303769322470645370[31] = 0;
   out_8303769322470645370[32] = 0;
   out_8303769322470645370[33] = 0;
   out_8303769322470645370[34] = 0;
   out_8303769322470645370[35] = 0;
   out_8303769322470645370[36] = 0;
   out_8303769322470645370[37] = 0;
   out_8303769322470645370[38] = 0;
   out_8303769322470645370[39] = 0;
   out_8303769322470645370[40] = 0;
   out_8303769322470645370[41] = 0;
   out_8303769322470645370[42] = 0;
   out_8303769322470645370[43] = 0;
   out_8303769322470645370[44] = 1;
   out_8303769322470645370[45] = 0;
   out_8303769322470645370[46] = 0;
   out_8303769322470645370[47] = 0;
   out_8303769322470645370[48] = 0;
   out_8303769322470645370[49] = 0;
   out_8303769322470645370[50] = 0;
   out_8303769322470645370[51] = 0;
   out_8303769322470645370[52] = 0;
   out_8303769322470645370[53] = 0;
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

void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_4, H_4, NULL, in_z, in_R, in_ea, MAHA_THRESH_4);
}
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_10, H_10, NULL, in_z, in_R, in_ea, MAHA_THRESH_10);
}
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_13, H_13, NULL, in_z, in_R, in_ea, MAHA_THRESH_13);
}
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_14, H_14, NULL, in_z, in_R, in_ea, MAHA_THRESH_14);
}
void pose_err_fun(double *nom_x, double *delta_x, double *out_7424980701830897997) {
  err_fun(nom_x, delta_x, out_7424980701830897997);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_8948729870032527372) {
  inv_err_fun(nom_x, true_x, out_8948729870032527372);
}
void pose_H_mod_fun(double *state, double *out_7614750544922110094) {
  H_mod_fun(state, out_7614750544922110094);
}
void pose_f_fun(double *state, double dt, double *out_4078122102263429604) {
  f_fun(state,  dt, out_4078122102263429604);
}
void pose_F_fun(double *state, double dt, double *out_8150619080602528479) {
  F_fun(state,  dt, out_8150619080602528479);
}
void pose_h_4(double *state, double *unused, double *out_2389356497093565065) {
  h_4(state, unused, out_2389356497093565065);
}
void pose_H_4(double *state, double *unused, double *out_4340528466131160841) {
  H_4(state, unused, out_4340528466131160841);
}
void pose_h_10(double *state, double *unused, double *out_8795577960664591671) {
  h_10(state, unused, out_8795577960664591671);
}
void pose_H_10(double *state, double *unused, double *out_8489014100523960796) {
  H_10(state, unused, out_8489014100523960796);
}
void pose_h_13(double *state, double *unused, double *out_2851896194690110696) {
  h_13(state, unused, out_2851896194690110696);
}
void pose_H_13(double *state, double *unused, double *out_6495584399261689846) {
  H_13(state, unused, out_6495584399261689846);
}
void pose_h_14(double *state, double *unused, double *out_7309375191689700906) {
  h_14(state, unused, out_7309375191689700906);
}
void pose_H_14(double *state, double *unused, double *out_8303769322470645370) {
  H_14(state, unused, out_8303769322470645370);
}
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF pose = {
  .name = "pose",
  .kinds = { 4, 10, 13, 14 },
  .feature_kinds = {  },
  .f_fun = pose_f_fun,
  .F_fun = pose_F_fun,
  .err_fun = pose_err_fun,
  .inv_err_fun = pose_inv_err_fun,
  .H_mod_fun = pose_H_mod_fun,
  .predict = pose_predict,
  .hs = {
    { 4, pose_h_4 },
    { 10, pose_h_10 },
    { 13, pose_h_13 },
    { 14, pose_h_14 },
  },
  .Hs = {
    { 4, pose_H_4 },
    { 10, pose_H_10 },
    { 13, pose_H_13 },
    { 14, pose_H_14 },
  },
  .updates = {
    { 4, pose_update_4 },
    { 10, pose_update_10 },
    { 13, pose_update_13 },
    { 14, pose_update_14 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_lib_init(pose)
