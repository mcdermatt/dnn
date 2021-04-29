/* Simscape target specific file.
 * This file is generated for the Simscape network associated with the solver block 'human9DOF/Solver Configuration'.
 */

#include <math.h>
#include <string.h>
#include "pm_std.h"
#include "sm_std.h"
#include "ne_std.h"
#include "ne_dae.h"
#include "sm_ssci_run_time_errors.h"
#include "sm_RuntimeDerivedValuesBundle.h"

void human9DOF_836bb176_1_computeRuntimeParameters(real_T (*t0)[19], real_T
  (*out)[19])
{
  (*out)[0] = (*t0)[5ULL];
  (*out)[1] = (*t0)[0ULL];
  (*out)[2] = (*t0)[1ULL];
  (*out)[3] = (*t0)[2ULL];
  (*out)[4] = (*t0)[18ULL];
  (*out)[5] = (*t0)[13ULL];
  (*out)[6] = (*t0)[12ULL];
  (*out)[7] = (*t0)[6ULL];
  (*out)[8] = (*t0)[4ULL];
  (*out)[9] = (*t0)[8ULL];
  (*out)[10] = (*t0)[14ULL];
  (*out)[11] = (*t0)[7ULL];
  (*out)[12] = (*t0)[3ULL];
  (*out)[13] = (*t0)[9ULL];
  (*out)[14] = (*t0)[10ULL];
  (*out)[15] = (*t0)[11ULL];
  (*out)[16] = (*t0)[15ULL];
  (*out)[17] = (*t0)[16ULL];
  (*out)[18] = (*t0)[17ULL];
}

void human9DOF_836bb176_1_computeAsmRuntimeDerivedValuesDoubles(const double
  *rtp, double *rtdvd)
{
  double xx[89];
  xx[0] = 0.5;
  xx[1] = 0.0174532925199433;
  xx[2] = 0.0;
  xx[3] = xx[0] * (!pmf_is_inf(rtp[5]) && !pmf_is_nan(rtp[5]) ? xx[1] * rtp[5] :
                   xx[2]);
  xx[4] = cos(xx[3]);
  xx[5] = sin(xx[3]);
  xx[3] = 0.7071067811865476;
  xx[6] = xx[3] * xx[4];
  xx[7] = xx[3] * xx[5];
  xx[8] = 2.0;
  xx[9] = 1.0;
  xx[10] = xx[0] * (!pmf_is_inf(rtp[6]) && !pmf_is_nan(rtp[6]) ? xx[1] * rtp[6] :
                    xx[2]);
  xx[11] = cos(xx[10]);
  xx[12] = sin(xx[10]);
  xx[10] = xx[0] * xx[12];
  xx[13] = xx[0] * xx[11];
  xx[14] = xx[10] - xx[13];
  xx[15] = xx[0] * (!pmf_is_inf(rtp[7]) && !pmf_is_nan(rtp[7]) ? xx[1] * rtp[7] :
                    xx[2]);
  xx[16] = cos(xx[15]);
  xx[17] = sin(xx[15]);
  xx[15] = 0.7068317195536079;
  xx[18] = 0.01887037739875302;
  xx[19] = 0.7068703770056876;
  xx[20] = 0.01915983302525687;
  xx[21] = 2.046760363534724e-4;
  xx[22] = xx[0] * (!pmf_is_inf(rtp[8]) && !pmf_is_nan(rtp[8]) ? xx[1] * rtp[8] :
                    xx[2]);
  xx[23] = sin(xx[22]);
  xx[24] = 0.9996383390547171;
  xx[25] = cos(xx[22]);
  xx[22] = xx[21] * xx[23] + xx[24] * xx[25];
  xx[26] = 0.02689141968076872;
  xx[27] = 2.733494650891524e-5;
  xx[28] = xx[26] * xx[25] + xx[27] * xx[23];
  xx[29] = xx[27] * xx[25] - xx[26] * xx[23];
  xx[26] = xx[21] * xx[25] - xx[24] * xx[23];
  xx[21] = - xx[26];
  xx[30] = xx[22];
  xx[31] = xx[28];
  xx[32] = xx[29];
  xx[33] = xx[21];
  xx[23] = - 0.2244123868924967;
  xx[24] = 9.872944805650204e-3;
  xx[25] = 1.177299620836221e-3;
  pm_math_Quaternion_inverseXform_ra(xx + 30, xx + 23, xx + 34);
  xx[27] = xx[3] * xx[22];
  xx[30] = xx[3] * xx[29];
  xx[31] = xx[28] * xx[3];
  xx[32] = xx[26] * xx[3];
  xx[33] = - ((xx[22] * xx[29] - xx[28] * xx[26]) * xx[8]);
  xx[37] = xx[8] * (xx[28] * xx[22] + xx[26] * xx[29]);
  xx[26] = (xx[28] * xx[28] + xx[29] * xx[29]) * xx[8] - xx[9];
  xx[38] = xx[33];
  xx[39] = xx[37];
  xx[40] = xx[26];
  pm_math_Vector3_cross_ra(xx + 23, xx + 38, xx + 41);
  xx[23] = xx[0] * (!pmf_is_inf(rtp[9]) && !pmf_is_nan(rtp[9]) ? xx[1] * rtp[9] :
                    xx[2]);
  xx[24] = xx[3] * cos(xx[23]);
  xx[25] = xx[3] * sin(xx[23]);
  xx[23] = xx[3] * xx[24];
  xx[38] = xx[3] * xx[25];
  xx[39] = xx[24] * xx[25];
  xx[40] = (xx[24] * xx[24] + xx[25] * xx[25]) * xx[8];
  xx[44] = xx[0] * (!pmf_is_inf(rtp[10]) && !pmf_is_nan(rtp[10]) ? xx[1] * rtp
                    [10] : xx[2]);
  xx[45] = cos(xx[44]);
  xx[46] = sin(xx[44]);
  xx[44] = 0.1119320701420158;
  xx[47] = 0.06364430550912828;
  xx[48] = 0.1907086909414691;
  xx[49] = 0.9731653555534001;
  xx[50] = 0.9731653555533999;
  xx[51] = xx[0] * (!pmf_is_inf(rtp[11]) && !pmf_is_nan(rtp[11]) ? xx[1] * rtp
                    [11] : xx[2]);
  xx[52] = cos(xx[51]);
  xx[53] = 0.190708690941469;
  xx[54] = sin(xx[51]);
  xx[51] = xx[50] * xx[52] - xx[53] * xx[54];
  xx[55] = 0.1119320701420158;
  xx[56] = 0.06364430550912827;
  xx[57] = xx[55] * xx[54] - xx[56] * xx[52];
  xx[58] = - xx[57];
  xx[59] = xx[56] * xx[54] + xx[55] * xx[52];
  xx[55] = - xx[59];
  xx[56] = xx[50] * xx[54] + xx[53] * xx[52];
  xx[60] = xx[51];
  xx[61] = xx[58];
  xx[62] = xx[55];
  xx[63] = xx[56];
  xx[52] = 0.0475545670786721;
  xx[53] = - 0.1051997654804633;
  xx[54] = - 0.01172847704075845;
  pm_math_Quaternion_inverseXform_ra(xx + 60, xx + 52, xx + 64);
  xx[67] = 0.6380800411073502;
  xx[68] = 0.3315952130861324;
  xx[69] = 0.6825039733402078;
  xx[70] = - 0.1307164954152066;
  pm_math_Quaternion_composeInverse_ra(xx + 60, xx + 67, xx + 71);
  xx[50] = (xx[59] * xx[51] + xx[57] * xx[56]) * xx[8];
  xx[60] = - (xx[8] * (xx[57] * xx[51] - xx[56] * xx[59]));
  xx[61] = (xx[57] * xx[57] + xx[59] * xx[59]) * xx[8] - xx[9];
  xx[67] = xx[50];
  xx[68] = xx[60];
  xx[69] = xx[61];
  pm_math_Vector3_cross_ra(xx + 52, xx + 67, xx + 75);
  xx[67] = 0.9337939117424219;
  xx[68] = - 0.1420427034611695;
  xx[69] = - 0.03141246372882533;
  xx[70] = 0.3269037441032352;
  xx[52] = xx[0] * (!pmf_is_inf(rtp[12]) && !pmf_is_nan(rtp[12]) ? xx[1] * rtp
                    [12] : xx[2]);
  xx[0] = - (xx[3] * cos(xx[52]));
  xx[53] = xx[3] * sin(xx[52]);
  xx[78] = xx[0];
  xx[79] = xx[0];
  xx[80] = xx[53];
  xx[81] = - xx[53];
  pm_math_Quaternion_inverseCompose_ra(xx + 67, xx + 78, xx + 82);
  xx[52] = - 0.08481305455953883;
  xx[53] = - 0.07715695137174808;
  xx[54] = 4.397388070152032e-4;
  pm_math_Quaternion_inverseXform_ra(xx + 82, xx + 52, xx + 67);
  xx[0] = xx[3] * xx[82];
  xx[57] = xx[3] * xx[83];
  xx[59] = xx[3] * xx[84];
  xx[62] = xx[3] * xx[85];
  xx[3] = - ((xx[82] * xx[84] + xx[83] * xx[85]) * xx[8]);
  xx[63] = - (xx[8] * (xx[84] * xx[85] - xx[82] * xx[83]));
  xx[70] = (xx[83] * xx[83] + xx[84] * xx[84]) * xx[8] - xx[9];
  xx[78] = xx[3];
  xx[79] = xx[63];
  xx[80] = xx[70];
  pm_math_Vector3_cross_ra(xx + 52, xx + 78, xx + 86);
  xx[52] = !pmf_is_inf(rtp[3]) && !pmf_is_nan(rtp[3]) ? xx[1] * rtp[3] : xx[2];
  xx[53] = !pmf_is_inf(rtp[4]) && !pmf_is_nan(rtp[4]) ? xx[1] * rtp[4] : xx[2];
  xx[54] = !pmf_is_inf(rtp[15]) && !pmf_is_nan(rtp[15]) ? xx[1] * rtp[15] : xx[2];
  xx[78] = !pmf_is_inf(rtp[18]) && !pmf_is_nan(rtp[18]) ? xx[1] * rtp[18] : xx[2];
  xx[79] = 57.29577951308232;
  rtdvd[0] = xx[4];
  rtdvd[1] = xx[5];
  rtdvd[2] = xx[6];
  rtdvd[3] = - xx[6];
  rtdvd[4] = xx[7];
  rtdvd[5] = xx[7];
  rtdvd[6] = - (xx[8] * xx[4] * xx[5]);
  rtdvd[7] = xx[8] * xx[5] * xx[5] - xx[9];
  rtdvd[8] = xx[11];
  rtdvd[9] = xx[12];
  rtdvd[10] = xx[14];
  rtdvd[11] = - (xx[13] + xx[10]);
  rtdvd[12] = - (xx[13] + xx[10]);
  rtdvd[13] = xx[14];
  rtdvd[14] = - (xx[8] * xx[11] * xx[12]);
  rtdvd[15] = xx[8] * xx[12] * xx[12] - xx[9];
  rtdvd[16] = xx[16];
  rtdvd[17] = xx[17];
  rtdvd[18] = xx[15] * xx[16] + xx[18] * xx[17];
  rtdvd[19] = xx[19] * xx[17] - xx[20] * xx[16];
  rtdvd[20] = - (xx[19] * xx[16] + xx[20] * xx[17]);
  rtdvd[21] = xx[15] * xx[17] - xx[18] * xx[16];
  rtdvd[22] = xx[22];
  rtdvd[23] = xx[28];
  rtdvd[24] = xx[29];
  rtdvd[25] = xx[21];
  rtdvd[26] = !pmf_is_inf(rtp[0]) && !pmf_is_nan(rtp[0]) ? xx[1] * rtp[0] : xx[2];
  rtdvd[27] = - xx[34];
  rtdvd[28] = - xx[35];
  rtdvd[29] = - xx[36];
  rtdvd[30] = xx[27] + xx[30];
  rtdvd[31] = xx[31] - xx[32];
  rtdvd[32] = xx[30] - xx[27];
  rtdvd[33] = - (xx[32] + xx[31]);
  rtdvd[34] = xx[41];
  rtdvd[35] = xx[42];
  rtdvd[36] = xx[43];
  rtdvd[37] = xx[33];
  rtdvd[38] = xx[37];
  rtdvd[39] = xx[26];
  rtdvd[40] = xx[24];
  rtdvd[41] = xx[24];
  rtdvd[42] = - xx[25];
  rtdvd[43] = xx[25];
  rtdvd[44] = xx[23] + xx[23];
  rtdvd[45] = xx[23] - xx[23];
  rtdvd[46] = - (xx[38] + xx[38]);
  rtdvd[47] = xx[38] - xx[38];
  rtdvd[48] = - (xx[8] * (xx[39] - xx[39]));
  rtdvd[49] = xx[40];
  rtdvd[50] = xx[40] - xx[9];
  rtdvd[51] = xx[45];
  rtdvd[52] = xx[46];
  rtdvd[53] = !pmf_is_inf(rtp[1]) && !pmf_is_nan(rtp[1]) ? xx[1] * rtp[1] : xx[2];
  rtdvd[54] = !pmf_is_inf(rtp[2]) && !pmf_is_nan(rtp[2]) ? xx[1] * rtp[2] : xx[2];
  rtdvd[55] = xx[44] * xx[45] + xx[47] * xx[46];
  rtdvd[56] = xx[48] * xx[45] + xx[49] * xx[46];
  rtdvd[57] = xx[48] * xx[46] - xx[49] * xx[45];
  rtdvd[58] = xx[44] * xx[46] - xx[47] * xx[45];
  rtdvd[59] = xx[51];
  rtdvd[60] = xx[58];
  rtdvd[61] = xx[55];
  rtdvd[62] = xx[56];
  rtdvd[63] = !pmf_is_inf(rtp[13]) && !pmf_is_nan(rtp[13]) ? xx[1] * rtp[13] :
    xx[2];
  rtdvd[64] = !pmf_is_inf(rtp[14]) && !pmf_is_nan(rtp[14]) ? xx[1] * rtp[14] :
    xx[2];
  rtdvd[65] = - xx[64];
  rtdvd[66] = - xx[65];
  rtdvd[67] = - xx[66];
  rtdvd[68] = xx[71];
  rtdvd[69] = xx[72];
  rtdvd[70] = xx[73];
  rtdvd[71] = xx[74];
  rtdvd[72] = xx[75];
  rtdvd[73] = xx[76];
  rtdvd[74] = xx[77];
  rtdvd[75] = xx[50];
  rtdvd[76] = xx[60];
  rtdvd[77] = xx[61];
  rtdvd[78] = xx[82];
  rtdvd[79] = xx[83];
  rtdvd[80] = xx[84];
  rtdvd[81] = xx[85];
  rtdvd[82] = !pmf_is_inf(rtp[16]) && !pmf_is_nan(rtp[16]) ? xx[1] * rtp[16] :
    xx[2];
  rtdvd[83] = !pmf_is_inf(rtp[17]) && !pmf_is_nan(rtp[17]) ? xx[1] * rtp[17] :
    xx[2];
  rtdvd[84] = - xx[67];
  rtdvd[85] = - xx[68];
  rtdvd[86] = - xx[69];
  rtdvd[87] = xx[0] + xx[57];
  rtdvd[88] = xx[57] - xx[0];
  rtdvd[89] = xx[59] - xx[62];
  rtdvd[90] = xx[62] + xx[59];
  rtdvd[91] = xx[86];
  rtdvd[92] = xx[87];
  rtdvd[93] = xx[88];
  rtdvd[94] = xx[3];
  rtdvd[95] = xx[63];
  rtdvd[96] = xx[70];
  rtdvd[97] = xx[52];
  rtdvd[98] = xx[53];
  rtdvd[99] = xx[54];
  rtdvd[100] = xx[78];
  rtdvd[101] = xx[79] * xx[53];
  rtdvd[102] = xx[79] * xx[52];
  rtdvd[103] = xx[79] * xx[54];
  rtdvd[104] = xx[79] * xx[78];
}

void human9DOF_836bb176_1_computeAsmRuntimeDerivedValuesInts(const double *rtp,
  int *rtdvi)
{
  (void) rtp;
  (void) rtdvi;
}

void human9DOF_836bb176_1_computeAsmRuntimeDerivedValues(const double *rtp,
  RuntimeDerivedValuesBundle *rtdv)
{
  human9DOF_836bb176_1_computeAsmRuntimeDerivedValuesDoubles(rtp,
    rtdv->mDoubles.mValues);
  human9DOF_836bb176_1_computeAsmRuntimeDerivedValuesInts(rtp,
    rtdv->mInts.mValues);
}

void human9DOF_836bb176_1_computeSimRuntimeDerivedValuesDoubles(const double
  *rtp, double *rtdvd)
{
  double xx[89];
  xx[0] = 0.5;
  xx[1] = 0.0174532925199433;
  xx[2] = 0.0;
  xx[3] = xx[0] * (!pmf_is_inf(rtp[5]) && !pmf_is_nan(rtp[5]) ? xx[1] * rtp[5] :
                   xx[2]);
  xx[4] = cos(xx[3]);
  xx[5] = sin(xx[3]);
  xx[3] = 0.7071067811865476;
  xx[6] = xx[3] * xx[4];
  xx[7] = xx[3] * xx[5];
  xx[8] = 2.0;
  xx[9] = 1.0;
  xx[10] = xx[0] * (!pmf_is_inf(rtp[6]) && !pmf_is_nan(rtp[6]) ? xx[1] * rtp[6] :
                    xx[2]);
  xx[11] = cos(xx[10]);
  xx[12] = sin(xx[10]);
  xx[10] = xx[0] * xx[12];
  xx[13] = xx[0] * xx[11];
  xx[14] = xx[10] - xx[13];
  xx[15] = xx[0] * (!pmf_is_inf(rtp[7]) && !pmf_is_nan(rtp[7]) ? xx[1] * rtp[7] :
                    xx[2]);
  xx[16] = cos(xx[15]);
  xx[17] = sin(xx[15]);
  xx[15] = 0.7068317195536079;
  xx[18] = 0.01887037739875302;
  xx[19] = 0.7068703770056876;
  xx[20] = 0.01915983302525687;
  xx[21] = 2.046760363534724e-4;
  xx[22] = xx[0] * (!pmf_is_inf(rtp[8]) && !pmf_is_nan(rtp[8]) ? xx[1] * rtp[8] :
                    xx[2]);
  xx[23] = sin(xx[22]);
  xx[24] = 0.9996383390547171;
  xx[25] = cos(xx[22]);
  xx[22] = xx[21] * xx[23] + xx[24] * xx[25];
  xx[26] = 0.02689141968076872;
  xx[27] = 2.733494650891524e-5;
  xx[28] = xx[26] * xx[25] + xx[27] * xx[23];
  xx[29] = xx[27] * xx[25] - xx[26] * xx[23];
  xx[26] = xx[21] * xx[25] - xx[24] * xx[23];
  xx[21] = - xx[26];
  xx[30] = xx[22];
  xx[31] = xx[28];
  xx[32] = xx[29];
  xx[33] = xx[21];
  xx[23] = - 0.2244123868924967;
  xx[24] = 9.872944805650204e-3;
  xx[25] = 1.177299620836221e-3;
  pm_math_Quaternion_inverseXform_ra(xx + 30, xx + 23, xx + 34);
  xx[27] = xx[3] * xx[22];
  xx[30] = xx[3] * xx[29];
  xx[31] = xx[28] * xx[3];
  xx[32] = xx[26] * xx[3];
  xx[33] = - ((xx[22] * xx[29] - xx[28] * xx[26]) * xx[8]);
  xx[37] = xx[8] * (xx[28] * xx[22] + xx[26] * xx[29]);
  xx[26] = (xx[28] * xx[28] + xx[29] * xx[29]) * xx[8] - xx[9];
  xx[38] = xx[33];
  xx[39] = xx[37];
  xx[40] = xx[26];
  pm_math_Vector3_cross_ra(xx + 23, xx + 38, xx + 41);
  xx[23] = xx[0] * (!pmf_is_inf(rtp[9]) && !pmf_is_nan(rtp[9]) ? xx[1] * rtp[9] :
                    xx[2]);
  xx[24] = xx[3] * cos(xx[23]);
  xx[25] = xx[3] * sin(xx[23]);
  xx[23] = xx[3] * xx[24];
  xx[38] = xx[3] * xx[25];
  xx[39] = xx[24] * xx[25];
  xx[40] = (xx[24] * xx[24] + xx[25] * xx[25]) * xx[8];
  xx[44] = xx[0] * (!pmf_is_inf(rtp[10]) && !pmf_is_nan(rtp[10]) ? xx[1] * rtp
                    [10] : xx[2]);
  xx[45] = cos(xx[44]);
  xx[46] = sin(xx[44]);
  xx[44] = 0.1119320701420158;
  xx[47] = 0.06364430550912828;
  xx[48] = 0.1907086909414691;
  xx[49] = 0.9731653555534001;
  xx[50] = 0.9731653555533999;
  xx[51] = xx[0] * (!pmf_is_inf(rtp[11]) && !pmf_is_nan(rtp[11]) ? xx[1] * rtp
                    [11] : xx[2]);
  xx[52] = cos(xx[51]);
  xx[53] = 0.190708690941469;
  xx[54] = sin(xx[51]);
  xx[51] = xx[50] * xx[52] - xx[53] * xx[54];
  xx[55] = 0.1119320701420158;
  xx[56] = 0.06364430550912827;
  xx[57] = xx[55] * xx[54] - xx[56] * xx[52];
  xx[58] = - xx[57];
  xx[59] = xx[56] * xx[54] + xx[55] * xx[52];
  xx[55] = - xx[59];
  xx[56] = xx[50] * xx[54] + xx[53] * xx[52];
  xx[60] = xx[51];
  xx[61] = xx[58];
  xx[62] = xx[55];
  xx[63] = xx[56];
  xx[52] = 0.0475545670786721;
  xx[53] = - 0.1051997654804633;
  xx[54] = - 0.01172847704075845;
  pm_math_Quaternion_inverseXform_ra(xx + 60, xx + 52, xx + 64);
  xx[67] = 0.6380800411073502;
  xx[68] = 0.3315952130861324;
  xx[69] = 0.6825039733402078;
  xx[70] = - 0.1307164954152066;
  pm_math_Quaternion_composeInverse_ra(xx + 60, xx + 67, xx + 71);
  xx[50] = (xx[59] * xx[51] + xx[57] * xx[56]) * xx[8];
  xx[60] = - (xx[8] * (xx[57] * xx[51] - xx[56] * xx[59]));
  xx[61] = (xx[57] * xx[57] + xx[59] * xx[59]) * xx[8] - xx[9];
  xx[67] = xx[50];
  xx[68] = xx[60];
  xx[69] = xx[61];
  pm_math_Vector3_cross_ra(xx + 52, xx + 67, xx + 75);
  xx[67] = 0.9337939117424219;
  xx[68] = - 0.1420427034611695;
  xx[69] = - 0.03141246372882533;
  xx[70] = 0.3269037441032352;
  xx[52] = xx[0] * (!pmf_is_inf(rtp[12]) && !pmf_is_nan(rtp[12]) ? xx[1] * rtp
                    [12] : xx[2]);
  xx[0] = - (xx[3] * cos(xx[52]));
  xx[53] = xx[3] * sin(xx[52]);
  xx[78] = xx[0];
  xx[79] = xx[0];
  xx[80] = xx[53];
  xx[81] = - xx[53];
  pm_math_Quaternion_inverseCompose_ra(xx + 67, xx + 78, xx + 82);
  xx[52] = - 0.08481305455953883;
  xx[53] = - 0.07715695137174808;
  xx[54] = 4.397388070152032e-4;
  pm_math_Quaternion_inverseXform_ra(xx + 82, xx + 52, xx + 67);
  xx[0] = xx[3] * xx[82];
  xx[57] = xx[3] * xx[83];
  xx[59] = xx[3] * xx[84];
  xx[62] = xx[3] * xx[85];
  xx[3] = - ((xx[82] * xx[84] + xx[83] * xx[85]) * xx[8]);
  xx[63] = - (xx[8] * (xx[84] * xx[85] - xx[82] * xx[83]));
  xx[70] = (xx[83] * xx[83] + xx[84] * xx[84]) * xx[8] - xx[9];
  xx[78] = xx[3];
  xx[79] = xx[63];
  xx[80] = xx[70];
  pm_math_Vector3_cross_ra(xx + 52, xx + 78, xx + 86);
  rtdvd[0] = xx[4];
  rtdvd[1] = xx[5];
  rtdvd[2] = xx[6];
  rtdvd[3] = - xx[6];
  rtdvd[4] = xx[7];
  rtdvd[5] = xx[7];
  rtdvd[6] = - (xx[8] * xx[4] * xx[5]);
  rtdvd[7] = xx[8] * xx[5] * xx[5] - xx[9];
  rtdvd[8] = xx[11];
  rtdvd[9] = xx[12];
  rtdvd[10] = xx[14];
  rtdvd[11] = - (xx[13] + xx[10]);
  rtdvd[12] = - (xx[13] + xx[10]);
  rtdvd[13] = xx[14];
  rtdvd[14] = - (xx[8] * xx[11] * xx[12]);
  rtdvd[15] = xx[8] * xx[12] * xx[12] - xx[9];
  rtdvd[16] = xx[16];
  rtdvd[17] = xx[17];
  rtdvd[18] = xx[15] * xx[16] + xx[18] * xx[17];
  rtdvd[19] = xx[19] * xx[17] - xx[20] * xx[16];
  rtdvd[20] = - (xx[19] * xx[16] + xx[20] * xx[17]);
  rtdvd[21] = xx[15] * xx[17] - xx[18] * xx[16];
  rtdvd[22] = xx[22];
  rtdvd[23] = xx[28];
  rtdvd[24] = xx[29];
  rtdvd[25] = xx[21];
  rtdvd[26] = !pmf_is_inf(rtp[0]) && !pmf_is_nan(rtp[0]) ? xx[1] * rtp[0] : xx[2];
  rtdvd[27] = - xx[34];
  rtdvd[28] = - xx[35];
  rtdvd[29] = - xx[36];
  rtdvd[30] = xx[27] + xx[30];
  rtdvd[31] = xx[31] - xx[32];
  rtdvd[32] = xx[30] - xx[27];
  rtdvd[33] = - (xx[32] + xx[31]);
  rtdvd[34] = xx[41];
  rtdvd[35] = xx[42];
  rtdvd[36] = xx[43];
  rtdvd[37] = xx[33];
  rtdvd[38] = xx[37];
  rtdvd[39] = xx[26];
  rtdvd[40] = xx[24];
  rtdvd[41] = xx[24];
  rtdvd[42] = - xx[25];
  rtdvd[43] = xx[25];
  rtdvd[44] = xx[23] + xx[23];
  rtdvd[45] = xx[23] - xx[23];
  rtdvd[46] = - (xx[38] + xx[38]);
  rtdvd[47] = xx[38] - xx[38];
  rtdvd[48] = - (xx[8] * (xx[39] - xx[39]));
  rtdvd[49] = xx[40];
  rtdvd[50] = xx[40] - xx[9];
  rtdvd[51] = xx[45];
  rtdvd[52] = xx[46];
  rtdvd[53] = !pmf_is_inf(rtp[1]) && !pmf_is_nan(rtp[1]) ? xx[1] * rtp[1] : xx[2];
  rtdvd[54] = !pmf_is_inf(rtp[2]) && !pmf_is_nan(rtp[2]) ? xx[1] * rtp[2] : xx[2];
  rtdvd[55] = xx[44] * xx[45] + xx[47] * xx[46];
  rtdvd[56] = xx[48] * xx[45] + xx[49] * xx[46];
  rtdvd[57] = xx[48] * xx[46] - xx[49] * xx[45];
  rtdvd[58] = xx[44] * xx[46] - xx[47] * xx[45];
  rtdvd[59] = xx[51];
  rtdvd[60] = xx[58];
  rtdvd[61] = xx[55];
  rtdvd[62] = xx[56];
  rtdvd[63] = !pmf_is_inf(rtp[13]) && !pmf_is_nan(rtp[13]) ? xx[1] * rtp[13] :
    xx[2];
  rtdvd[64] = !pmf_is_inf(rtp[14]) && !pmf_is_nan(rtp[14]) ? xx[1] * rtp[14] :
    xx[2];
  rtdvd[65] = - xx[64];
  rtdvd[66] = - xx[65];
  rtdvd[67] = - xx[66];
  rtdvd[68] = xx[71];
  rtdvd[69] = xx[72];
  rtdvd[70] = xx[73];
  rtdvd[71] = xx[74];
  rtdvd[72] = xx[75];
  rtdvd[73] = xx[76];
  rtdvd[74] = xx[77];
  rtdvd[75] = xx[50];
  rtdvd[76] = xx[60];
  rtdvd[77] = xx[61];
  rtdvd[78] = xx[82];
  rtdvd[79] = xx[83];
  rtdvd[80] = xx[84];
  rtdvd[81] = xx[85];
  rtdvd[82] = !pmf_is_inf(rtp[16]) && !pmf_is_nan(rtp[16]) ? xx[1] * rtp[16] :
    xx[2];
  rtdvd[83] = !pmf_is_inf(rtp[17]) && !pmf_is_nan(rtp[17]) ? xx[1] * rtp[17] :
    xx[2];
  rtdvd[84] = - xx[67];
  rtdvd[85] = - xx[68];
  rtdvd[86] = - xx[69];
  rtdvd[87] = xx[0] + xx[57];
  rtdvd[88] = xx[57] - xx[0];
  rtdvd[89] = xx[59] - xx[62];
  rtdvd[90] = xx[62] + xx[59];
  rtdvd[91] = xx[86];
  rtdvd[92] = xx[87];
  rtdvd[93] = xx[88];
  rtdvd[94] = xx[3];
  rtdvd[95] = xx[63];
  rtdvd[96] = xx[70];
}

void human9DOF_836bb176_1_computeSimRuntimeDerivedValuesInts(const double *rtp,
  int *rtdvi)
{
  (void) rtp;
  (void) rtdvi;
}

void human9DOF_836bb176_1_computeSimRuntimeDerivedValues(const double *rtp,
  RuntimeDerivedValuesBundle *rtdv)
{
  human9DOF_836bb176_1_computeSimRuntimeDerivedValuesDoubles(rtp,
    rtdv->mDoubles.mValues);
  human9DOF_836bb176_1_computeSimRuntimeDerivedValuesInts(rtp,
    rtdv->mInts.mValues);
}
