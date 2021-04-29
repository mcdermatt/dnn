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
#include "human9DOF_836bb176_1_geometries.h"

PmfMessageId human9DOF_836bb176_1_compDerivs(const RuntimeDerivedValuesBundle
  *rtdv, const int *eqnEnableFlags, const double *state, const int *modeVector,
  const double *input, const double *inputDot, const double *inputDdot, const
  double *discreteState, double *deriv, double *errorResult,
  NeuDiagnosticManager *neDiagMgr)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  int ii[1];
  double xx[345];
  (void) rtdvi;
  (void) eqnEnableFlags;
  (void) modeVector;
  (void) inputDot;
  (void) inputDdot;
  (void) discreteState;
  (void) neDiagMgr;
  xx[0] = 0.0;
  xx[1] = state[0] + 0.5629965224107606;
  if (xx[0] < xx[1])
    xx[1] = xx[0];
  xx[2] = 1.74532925199433e-3;
  xx[3] = 1.0;
  xx[4] = - (xx[1] / xx[2]);
  if (xx[3] < xx[4])
    xx[4] = xx[3];
  xx[5] = 3.0;
  xx[6] = 2.0;
  xx[7] = 572.9577951308232;
  xx[8] = xx[7] * state[1];
  xx[9] = 5.729577951308233e6;
  xx[10] = xx[4] * xx[4] * (xx[5] - xx[6] * xx[4]) * ((- xx[1] == xx[0] ? xx[0] :
    - xx[8]) - xx[9] * xx[1]);
  if (xx[0] > xx[10])
    xx[10] = xx[0];
  xx[1] = 2864.788975654116;
  xx[4] = 286.4788975654116;
  xx[11] = state[0] - 0.6167645713769713;
  if (xx[0] > xx[11])
    xx[11] = xx[0];
  xx[12] = xx[11] / xx[2];
  if (xx[3] < xx[12])
    xx[12] = xx[3];
  xx[13] = (xx[9] * xx[11] + (xx[11] == xx[0] ? xx[0] : xx[8])) * xx[12] * xx[12]
    * (xx[5] - xx[6] * xx[12]);
  if (xx[0] > xx[13])
    xx[13] = xx[0];
  xx[8] = 0.5;
  xx[9] = xx[8] * state[2];
  xx[11] = cos(xx[9]);
  xx[12] = sin(xx[9]);
  xx[9] = xx[11] * rtdvd[10] - xx[12] * rtdvd[12];
  xx[14] = xx[11] * rtdvd[11] - xx[12] * rtdvd[13];
  xx[15] = xx[12] * rtdvd[10] + xx[11] * rtdvd[12];
  xx[16] = xx[11] * rtdvd[13] + xx[12] * rtdvd[11];
  xx[17] = xx[9];
  xx[18] = xx[14];
  xx[19] = xx[15];
  xx[20] = xx[16];
  xx[21] = rtdvd[18];
  xx[22] = rtdvd[19];
  xx[23] = rtdvd[20];
  xx[24] = rtdvd[21];
  xx[11] = xx[8] * state[4];
  xx[12] = 0.9999999147210419;
  xx[25] = sin(xx[11]);
  xx[26] = 4.077338750123304e-4;
  xx[27] = 6.565817943715579e-5;
  xx[28] = cos(xx[11]);
  xx[29] = xx[12] * xx[25];
  xx[30] = - (xx[26] * xx[25]);
  xx[31] = - (xx[27] * xx[25]);
  pm_math_Quaternion_compose_ra(xx + 21, xx + 28, xx + 32);
  xx[11] = xx[16] * state[1];
  xx[21] = xx[14] * state[1];
  xx[22] = xx[11] * xx[9] + xx[15] * xx[21];
  xx[23] = xx[21] * xx[9] - xx[15] * xx[11];
  xx[28] = - (xx[22] * xx[6]);
  xx[29] = (xx[16] * xx[11] + xx[21] * xx[14]) * xx[6] - state[1] + state[3];
  xx[30] = xx[6] * xx[23];
  pm_math_Quaternion_inverseXform_ra(xx + 32, xx + 28, xx + 36);
  xx[11] = xx[12] * state[5];
  xx[21] = xx[36] + xx[11];
  xx[24] = xx[26] * state[5];
  xx[25] = xx[37] - xx[24];
  xx[31] = xx[27] * state[5];
  xx[39] = xx[38] - xx[31];
  xx[40] = xx[21];
  xx[41] = xx[25];
  xx[42] = xx[39];
  xx[43] = 0.02373992835662075;
  xx[44] = 1.025609865480195;
  xx[45] = 1.028237576055439;
  xx[46] = xx[21] * xx[43];
  xx[47] = xx[25] * xx[44];
  xx[48] = xx[39] * xx[45];
  pm_math_Vector3_cross_ra(xx + 40, xx + 46, xx + 49);
  xx[52] = 0.997812314335627;
  xx[53] = 3.317527921544825e-3;
  xx[54] = 0.03372656826312366;
  xx[55] = - 0.05676352668626715;
  xx[46] = xx[8] * state[18];
  xx[47] = 0.1161231239188075;
  xx[48] = sin(xx[46]);
  xx[56] = 0.9922934657394255;
  xx[57] = 0.0432330653802957;
  xx[58] = cos(xx[46]);
  xx[59] = xx[47] * xx[48];
  xx[60] = - (xx[56] * xx[48]);
  xx[61] = - (xx[57] * xx[48]);
  pm_math_Quaternion_compose_ra(xx + 52, xx + 58, xx + 62);
  pm_math_Quaternion_inverseXform_ra(xx + 62, xx + 40, xx + 52);
  xx[46] = xx[47] * state[19];
  xx[48] = xx[52] + xx[46];
  xx[55] = xx[56] * state[19];
  xx[58] = xx[53] - xx[55];
  xx[59] = xx[57] * state[19];
  xx[60] = xx[54] - xx[59];
  xx[66] = xx[48];
  xx[67] = xx[58];
  xx[68] = xx[60];
  xx[61] = 0.01189550881342706;
  xx[69] = 0.01007487515622759;
  xx[70] = 0.01140985073840049;
  xx[71] = xx[48] * xx[61];
  xx[72] = xx[58] * xx[69];
  xx[73] = xx[60] * xx[70];
  pm_math_Vector3_cross_ra(xx + 66, xx + 71, xx + 74);
  xx[66] = xx[46];
  xx[67] = - xx[55];
  xx[68] = - xx[59];
  pm_math_Vector3_cross_ra(xx + 52, xx + 66, xx + 71);
  xx[46] = xx[74] + xx[61] * xx[71];
  xx[55] = 1.381343644018857e-3;
  xx[59] = 0.3490658503988659;
  xx[66] = state[18] + xx[59];
  if (xx[0] < xx[66])
    xx[66] = xx[0];
  xx[67] = - (xx[66] / xx[2]);
  if (xx[3] < xx[67])
    xx[67] = xx[3];
  xx[68] = xx[7] * state[19];
  xx[77] = 5.729577951308233e5;
  xx[78] = xx[67] * xx[67] * (xx[5] - xx[6] * xx[67]) * ((- xx[66] == xx[0] ?
    xx[0] : - xx[68]) - xx[77] * xx[66]);
  if (xx[0] > xx[78])
    xx[78] = xx[0];
  xx[66] = state[18] - xx[59];
  if (xx[0] > xx[66])
    xx[66] = xx[0];
  xx[59] = xx[66] / xx[2];
  if (xx[3] < xx[59])
    xx[59] = xx[3];
  xx[67] = (xx[77] * xx[66] + (xx[66] == xx[0] ? xx[0] : xx[68])) * xx[59] * xx
    [59] * (xx[5] - xx[6] * xx[59]);
  if (xx[0] > xx[67])
    xx[67] = xx[0];
  xx[79] = xx[47];
  xx[80] = - xx[56];
  xx[81] = - xx[57];
  xx[47] = xx[75] + xx[69] * xx[72];
  xx[56] = xx[76] + xx[70] * xx[73];
  xx[71] = xx[46];
  xx[72] = xx[47];
  xx[73] = xx[56];
  xx[57] = 0.01029859628626135;
  xx[59] = 8.928983924324474e-3;
  xx[66] = 0.1772779043919793;
  xx[74] = - xx[57];
  xx[75] = - xx[59];
  xx[76] = xx[66];
  xx[82] = xx[52] + xx[48];
  xx[83] = xx[53] + xx[58];
  xx[84] = xx[54] + xx[60];
  xx[52] = - (xx[57] * state[19]);
  xx[53] = - (xx[59] * state[19]);
  xx[54] = xx[66] * state[19];
  pm_math_Vector3_cross_ra(xx + 82, xx + 52, xx + 57);
  xx[52] = - 0.1774392005774167;
  xx[53] = - 0.0103867778636817;
  xx[54] = - 0.01083111892178359;
  pm_math_Quaternion_xform_ra(xx + 62, xx + 52, xx + 82);
  xx[52] = - (0.02621557995213412 + xx[82]);
  xx[53] = 0.1918176122920127 - xx[83];
  xx[54] = 0.04085299662695828 - xx[84];
  pm_math_Vector3_cross_ra(xx + 40, xx + 52, xx + 82);
  pm_math_Vector3_cross_ra(xx + 40, xx + 82, xx + 85);
  pm_math_Quaternion_inverseXform_ra(xx + 62, xx + 85, xx + 82);
  xx[48] = 2.26796185;
  xx[60] = (xx[57] + xx[82]) * xx[48];
  xx[66] = (xx[58] + xx[83]) * xx[48];
  xx[57] = (xx[59] + xx[84]) * xx[48];
  xx[82] = xx[60];
  xx[83] = xx[66];
  xx[84] = xx[57];
  xx[58] = 0.08179955042544698;
  ii[0] = factorSymmetricPosDef(xx + 58, 1, xx + 59);
  if (ii[0] != 0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:DegenerateMass",
      "'human9DOF/Revolute Joint' has a degenerate mass distribution on its follower side.",
      neDiagMgr);
  }

  xx[59] = (xx[78] - 143.2394487827058 * state[19] - xx[67] -
            (pm_math_Vector3_dot_ra(xx + 79, xx + 71) + pm_math_Vector3_dot_ra
             (xx + 74, xx + 82))) / xx[58];
  xx[67] = 9.997232785665111e-3;
  xx[68] = 4.932828229526836e-4;
  xx[71] = xx[46] + xx[55] * xx[59];
  xx[72] = xx[47] - xx[67] * xx[59];
  xx[73] = xx[56] - xx[68] * xx[59];
  pm_math_Quaternion_xform_ra(xx + 62, xx + 71, xx + 74);
  xx[46] = 0.02335682348579243;
  xx[47] = 0.0202505948996312;
  xx[56] = 0.4020595240089566;
  xx[71] = xx[60] - xx[46] * xx[59];
  xx[72] = xx[66] - xx[47] * xx[59];
  xx[73] = xx[57] + xx[56] * xx[59];
  pm_math_Quaternion_xform_ra(xx + 62, xx + 71, xx + 78);
  pm_math_Vector3_cross_ra(xx + 52, xx + 78, xx + 71);
  xx[57] = xx[8] * state[6];
  xx[60] = cos(xx[57]);
  xx[66] = sin(xx[57]);
  xx[57] = xx[60] * rtdvd[30] - xx[66] * rtdvd[31];
  xx[81] = xx[66] * rtdvd[30] + xx[60] * rtdvd[31];
  xx[82] = xx[60] * rtdvd[32] + xx[66] * rtdvd[33];
  xx[83] = xx[60] * rtdvd[33] - xx[66] * rtdvd[32];
  xx[84] = xx[57];
  xx[85] = xx[81];
  xx[86] = xx[82];
  xx[87] = xx[83];
  xx[60] = xx[8] * state[8];
  xx[66] = cos(xx[60]);
  xx[88] = sin(xx[60]);
  xx[60] = xx[66] * rtdvd[44] + xx[88] * rtdvd[46];
  xx[89] = xx[66] * rtdvd[45] + xx[88] * rtdvd[47];
  xx[90] = xx[66] * rtdvd[46] - xx[88] * rtdvd[44];
  xx[91] = xx[66] * rtdvd[47] - xx[88] * rtdvd[45];
  xx[92] = xx[60];
  xx[93] = xx[89];
  xx[94] = xx[90];
  xx[95] = xx[91];
  xx[96] = rtdvd[55];
  xx[97] = rtdvd[56];
  xx[98] = rtdvd[57];
  xx[99] = rtdvd[58];
  xx[66] = xx[8] * state[10];
  xx[88] = 0.1935817812961164;
  xx[100] = sin(xx[66]);
  xx[101] = 0.1665657035417852;
  xx[102] = 0.966841228099969;
  xx[103] = cos(xx[66]);
  xx[104] = xx[88] * xx[100];
  xx[105] = xx[101] * xx[100];
  xx[106] = - (xx[102] * xx[100]);
  pm_math_Quaternion_compose_ra(xx + 96, xx + 103, xx + 107);
  pm_math_Quaternion_inverseXform_ra(xx + 84, xx + 40, xx + 96);
  xx[103] = xx[96] + state[7];
  xx[104] = xx[97];
  xx[105] = xx[98];
  pm_math_Quaternion_inverseXform_ra(xx + 92, xx + 103, xx + 111);
  xx[103] = xx[111];
  xx[104] = xx[112] - state[9];
  xx[105] = xx[113];
  pm_math_Quaternion_inverseXform_ra(xx + 107, xx + 103, xx + 114);
  xx[66] = xx[88] * state[11];
  xx[96] = xx[114] + xx[66];
  xx[99] = xx[101] * state[11];
  xx[100] = xx[115] + xx[99];
  xx[106] = xx[102] * state[11];
  xx[112] = xx[116] - xx[106];
  xx[117] = xx[96];
  xx[118] = xx[100];
  xx[119] = xx[112];
  xx[120] = 0.01298224526242276;
  xx[121] = 7.743843344108615e-3;
  xx[122] = 0.01674583373577103;
  xx[123] = xx[96] * xx[120];
  xx[124] = xx[100] * xx[121];
  xx[125] = xx[112] * xx[122];
  pm_math_Vector3_cross_ra(xx + 117, xx + 123, xx + 126);
  xx[129] = rtdvd[68];
  xx[130] = rtdvd[69];
  xx[131] = rtdvd[70];
  xx[132] = rtdvd[71];
  xx[123] = xx[8] * state[12];
  xx[124] = 0.7842943984275436;
  xx[125] = sin(xx[123]);
  xx[133] = 0.6015976293979709;
  xx[134] = 0.1515341179336174;
  xx[135] = cos(xx[123]);
  xx[136] = xx[124] * xx[125];
  xx[137] = - (xx[133] * xx[125]);
  xx[138] = - (xx[134] * xx[125]);
  pm_math_Quaternion_compose_ra(xx + 129, xx + 135, xx + 139);
  pm_math_Quaternion_inverseXform_ra(xx + 139, xx + 117, xx + 129);
  xx[123] = xx[124] * state[13];
  xx[125] = xx[129] + xx[123];
  xx[132] = xx[133] * state[13];
  xx[135] = xx[130] - xx[132];
  xx[136] = xx[134] * state[13];
  xx[137] = xx[131] - xx[136];
  xx[143] = xx[125];
  xx[144] = xx[135];
  xx[145] = xx[137];
  xx[138] = 0.01657663176842608;
  xx[146] = 3.879462597833555e-3;
  xx[147] = 7.692913897095051e-3;
  xx[148] = xx[125] * xx[138];
  xx[149] = xx[135] * xx[146];
  xx[150] = xx[137] * xx[147];
  pm_math_Vector3_cross_ra(xx + 143, xx + 148, xx + 151);
  xx[148] = xx[8] * state[14];
  xx[149] = cos(xx[148]);
  xx[150] = sin(xx[148]);
  xx[148] = xx[149] * rtdvd[87] + xx[150] * rtdvd[89];
  xx[154] = xx[149] * rtdvd[88] + xx[150] * rtdvd[90];
  xx[155] = xx[149] * rtdvd[89] - xx[150] * rtdvd[87];
  xx[156] = xx[149] * rtdvd[90] - xx[150] * rtdvd[88];
  xx[157] = xx[148];
  xx[158] = xx[154];
  xx[159] = xx[155];
  xx[160] = xx[156];
  xx[161] = - 0.979885074593562;
  xx[162] = 0.1454472155865731;
  xx[163] = 2.824882685237451e-3;
  xx[164] = - 0.1366102781810241;
  xx[149] = xx[8] * state[16];
  xx[150] = 0.03420304840239036;
  xx[165] = sin(xx[149]);
  xx[166] = 0.2858149274078682;
  xx[167] = 0.9576742550318551;
  xx[168] = cos(xx[149]);
  xx[169] = - (xx[150] * xx[165]);
  xx[170] = - (xx[166] * xx[165]);
  xx[171] = xx[167] * xx[165];
  pm_math_Quaternion_compose_ra(xx + 161, xx + 168, xx + 172);
  pm_math_Quaternion_inverseXform_ra(xx + 157, xx + 143, xx + 161);
  xx[168] = xx[161];
  xx[169] = xx[162] - state[15];
  xx[170] = xx[163];
  pm_math_Quaternion_inverseXform_ra(xx + 172, xx + 168, xx + 176);
  xx[149] = xx[150] * state[17];
  xx[162] = xx[176] - xx[149];
  xx[164] = xx[166] * state[17];
  xx[165] = xx[177] - xx[164];
  xx[171] = xx[167] * state[17];
  xx[179] = xx[178] + xx[171];
  xx[180] = xx[162];
  xx[181] = xx[165];
  xx[182] = xx[179];
  xx[183] = 0.0165766317684261;
  xx[184] = 3.87946259783357e-3;
  xx[185] = 7.692913897095077e-3;
  xx[186] = xx[162] * xx[183];
  xx[187] = xx[165] * xx[184];
  xx[188] = xx[179] * xx[185];
  pm_math_Vector3_cross_ra(xx + 180, xx + 186, xx + 189);
  xx[180] = - 7.359574475229558e-3;
  xx[181] = - 0.0873999301960896;
  xx[182] = - 0.02634708456854787;
  xx[186] = xx[8] * state[0];
  xx[8] = cos(xx[186]);
  xx[187] = sin(xx[186]);
  xx[192] = xx[8] * rtdvd[2] + xx[187] * rtdvd[4];
  xx[193] = xx[8] * rtdvd[3] + xx[187] * rtdvd[5];
  xx[194] = xx[8] * rtdvd[4] - xx[187] * rtdvd[2];
  xx[195] = xx[8] * rtdvd[5] - xx[187] * rtdvd[3];
  pm_math_Quaternion_compose_ra(xx + 192, xx + 17, xx + 196);
  pm_math_Quaternion_compose_ra(xx + 196, xx + 32, xx + 192);
  pm_math_Quaternion_compose_ra(xx + 192, xx + 84, xx + 196);
  pm_math_Quaternion_compose_ra(xx + 196, xx + 92, xx + 192);
  pm_math_Quaternion_compose_ra(xx + 192, xx + 107, xx + 196);
  pm_math_Quaternion_compose_ra(xx + 196, xx + 139, xx + 192);
  pm_math_Quaternion_compose_ra(xx + 192, xx + 157, xx + 196);
  pm_math_Quaternion_compose_ra(xx + 196, xx + 172, xx + 192);
  xx[186] = input[0];
  xx[187] = input[1];
  xx[188] = input[2];
  pm_math_Quaternion_inverseXform_ra(xx + 192, xx + 186, xx + 196);
  pm_math_Vector3_cross_ra(xx + 180, xx + 196, xx + 186);
  xx[180] = - xx[149];
  xx[181] = - xx[164];
  xx[182] = xx[171];
  pm_math_Vector3_cross_ra(xx + 176, xx + 180, xx + 192);
  xx[8] = xx[189] - xx[186] + xx[183] * xx[192];
  xx[149] = 5.669713387240796e-4;
  xx[164] = state[16] + 0.5634801614971511;
  if (xx[0] < xx[164])
    xx[164] = xx[0];
  xx[171] = - (xx[164] / xx[2]);
  if (xx[3] < xx[171])
    xx[171] = xx[3];
  xx[180] = xx[7] * state[17];
  xx[181] = xx[171] * xx[171] * (xx[5] - xx[6] * xx[171]) * ((- xx[164] == xx[0]
    ? xx[0] : - xx[180]) - xx[77] * xx[164]);
  if (xx[0] > xx[181])
    xx[181] = xx[0];
  xx[164] = state[16] - 1.356382015696612;
  if (xx[0] > xx[164])
    xx[164] = xx[0];
  xx[171] = xx[164] / xx[2];
  if (xx[3] < xx[171])
    xx[171] = xx[3];
  xx[182] = (xx[77] * xx[164] + (xx[164] == xx[0] ? xx[0] : xx[180])) * xx[171] *
    xx[171] * (xx[5] - xx[6] * xx[171]);
  if (xx[0] > xx[182])
    xx[182] = xx[0];
  xx[199] = - xx[150];
  xx[200] = - xx[166];
  xx[201] = xx[167];
  xx[150] = xx[190] - xx[187] + xx[184] * xx[193];
  xx[164] = xx[191] - xx[188] + xx[185] * xx[194];
  xx[186] = xx[8];
  xx[187] = xx[150];
  xx[188] = xx[164];
  xx[166] = 0.03146781994356587;
  xx[167] = 0.03789251374844847;
  xx[171] = 0.01243276758501344;
  xx[189] = - xx[166];
  xx[190] = - xx[167];
  xx[191] = - xx[171];
  xx[192] = xx[176] + xx[162];
  xx[193] = xx[177] + xx[165];
  xx[194] = xx[178] + xx[179];
  xx[176] = - (xx[166] * state[17]);
  xx[177] = - (xx[167] * state[17]);
  xx[178] = - (xx[171] * state[17]);
  pm_math_Vector3_cross_ra(xx + 192, xx + 176, xx + 165);
  xx[176] = 0.03984215544011924;
  xx[177] = - 0.03056115957341688;
  xx[178] = - 7.697933191027721e-3;
  pm_math_Quaternion_xform_ra(xx + 172, xx + 176, xx + 192);
  xx[176] = - xx[192];
  xx[177] = - xx[193];
  xx[178] = - xx[194];
  pm_math_Vector3_cross_ra(xx + 168, xx + 176, xx + 192);
  pm_math_Vector3_cross_ra(xx + 168, xx + 192, xx + 202);
  pm_math_Quaternion_inverseXform_ra(xx + 172, xx + 202, xx + 168);
  xx[162] = 0.90718474;
  xx[171] = (xx[165] + xx[168]) * xx[162] - xx[196];
  xx[179] = (xx[166] + xx[169]) * xx[162] - xx[197];
  xx[165] = (xx[167] + xx[170]) * xx[162] - xx[198];
  xx[166] = xx[171];
  xx[167] = xx[179];
  xx[168] = xx[165];
  xx[169] = 9.732902233401763e-3;
  ii[0] = factorSymmetricPosDef(xx + 169, 1, xx + 170);
  if (ii[0] != 0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:DegenerateMass",
      "'human9DOF/j8' has a degenerate mass distribution on its follower side.",
      neDiagMgr);
  }

  xx[170] = (xx[181] - 5.729577951308233 * state[17] - xx[182] -
             (pm_math_Vector3_dot_ra(xx + 199, xx + 186) +
              pm_math_Vector3_dot_ra(xx + 189, xx + 166))) / xx[169];
  xx[166] = 1.108808320781342e-3;
  xx[167] = 7.367305585424733e-3;
  xx[180] = xx[8] - xx[149] * xx[170];
  xx[181] = xx[150] - xx[166] * xx[170];
  xx[182] = xx[164] + xx[167] * xx[170];
  pm_math_Quaternion_xform_ra(xx + 172, xx + 180, xx + 186);
  xx[8] = 0.02854712605387062;
  xx[150] = 0.03437551023283265;
  xx[164] = 0.01127881702909084;
  xx[180] = xx[171] - xx[8] * xx[170];
  xx[181] = xx[179] - xx[150] * xx[170];
  xx[182] = xx[165] - xx[164] * xx[170];
  pm_math_Quaternion_xform_ra(xx + 172, xx + 180, xx + 189);
  pm_math_Vector3_cross_ra(xx + 176, xx + 189, xx + 179);
  xx[165] = xx[163] * state[15];
  xx[163] = xx[172] * xx[172];
  xx[168] = xx[173] * xx[174];
  xx[171] = xx[172] * xx[175];
  xx[182] = xx[173] * xx[175];
  xx[192] = xx[172] * xx[174];
  xx[193] = xx[174] * xx[175];
  xx[194] = xx[172] * xx[173];
  xx[195] = (xx[163] + xx[173] * xx[173]) * xx[6] - xx[3];
  xx[196] = xx[6] * (xx[168] - xx[171]);
  xx[197] = (xx[182] + xx[192]) * xx[6];
  xx[198] = (xx[168] + xx[171]) * xx[6];
  xx[199] = (xx[163] + xx[174] * xx[174]) * xx[6] - xx[3];
  xx[200] = xx[6] * (xx[193] - xx[194]);
  xx[201] = xx[6] * (xx[182] - xx[192]);
  xx[202] = (xx[193] + xx[194]) * xx[6];
  xx[203] = (xx[163] + xx[175] * xx[175]) * xx[6] - xx[3];
  xx[163] = xx[149] / xx[169];
  xx[168] = - (xx[166] * xx[163]);
  xx[171] = xx[167] * xx[163];
  xx[182] = xx[166] / xx[169];
  xx[192] = xx[167] * xx[182];
  xx[204] = xx[183] - xx[149] * xx[163];
  xx[205] = xx[168];
  xx[206] = xx[171];
  xx[207] = xx[168];
  xx[208] = xx[184] - xx[166] * xx[182];
  xx[209] = xx[192];
  xx[210] = xx[171];
  xx[211] = xx[192];
  xx[212] = xx[185] - 5.427719158903047e-5 / xx[169];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 204, xx + 195, xx + 213);
  pm_math_Matrix3x3_compose_ra(xx + 195, xx + 213, xx + 204);
  xx[168] = xx[8] / xx[169];
  xx[171] = xx[150] / xx[169];
  xx[183] = xx[164] / xx[169];
  xx[213] = - (xx[149] * xx[168]);
  xx[214] = - (xx[149] * xx[171]);
  xx[215] = - (xx[149] * xx[183]);
  xx[216] = - (xx[166] * xx[168]);
  xx[217] = - (xx[166] * xx[171]);
  xx[218] = - (xx[166] * xx[183]);
  xx[219] = xx[167] * xx[168];
  xx[220] = xx[167] * xx[171];
  xx[221] = xx[167] * xx[183];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 213, xx + 195, xx + 222);
  pm_math_Matrix3x3_compose_ra(xx + 195, xx + 222, xx + 213);
  pm_math_Matrix3x3_postCross_ra(xx + 213, xx + 176, xx + 222);
  xx[149] = - (xx[150] * xx[168]);
  xx[166] = - (xx[164] * xx[168]);
  xx[184] = - (xx[164] * xx[171]);
  xx[231] = xx[162] - xx[8] * xx[168];
  xx[232] = xx[149];
  xx[233] = xx[166];
  xx[234] = xx[149];
  xx[235] = xx[162] - xx[150] * xx[171];
  xx[236] = xx[184];
  xx[237] = xx[166];
  xx[238] = xx[184];
  xx[239] = xx[162] - xx[164] * xx[183];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 231, xx + 195, xx + 240);
  pm_math_Matrix3x3_compose_ra(xx + 195, xx + 240, xx + 231);
  pm_math_Matrix3x3_postCross_ra(xx + 231, xx + 176, xx + 192);
  pm_math_Matrix3x3_preCross_ra(xx + 192, xx + 176, xx + 240);
  xx[8] = xx[204] - xx[222] - xx[222] - xx[240];
  xx[149] = xx[161] * state[15];
  xx[150] = xx[206] - xx[224] - xx[228] - xx[242];
  xx[161] = xx[213] - xx[192];
  xx[162] = xx[214] - xx[195];
  xx[164] = xx[215] - xx[198];
  xx[166] = xx[216] - xx[193];
  xx[184] = xx[217] - xx[196];
  xx[185] = xx[218] - xx[199];
  xx[192] = xx[219] - xx[194];
  xx[193] = xx[220] - xx[197];
  xx[194] = xx[221] - xx[200];
  xx[195] = xx[161];
  xx[196] = xx[162];
  xx[197] = xx[164];
  xx[198] = xx[166];
  xx[199] = xx[184];
  xx[200] = xx[185];
  xx[201] = xx[192];
  xx[202] = xx[193];
  xx[203] = xx[194];
  xx[213] = - 0.08481305455953883;
  xx[214] = - 0.07715695137174808;
  xx[215] = 4.397388070152032e-4;
  pm_math_Vector3_cross_ra(xx + 143, xx + 213, xx + 216);
  pm_math_Vector3_cross_ra(xx + 143, xx + 216, xx + 219);
  pm_math_Quaternion_inverseXform_ra(xx + 157, xx + 219, xx + 143);
  pm_math_Matrix3x3_xform_ra(xx + 195, xx + 143, xx + 216);
  xx[195] = state[14] - rtdvd[82];
  if (xx[0] < xx[195])
    xx[195] = xx[0];
  xx[196] = - (xx[195] / xx[2]);
  if (xx[3] < xx[196])
    xx[196] = xx[3];
  xx[197] = xx[7] * state[15];
  xx[198] = xx[196] * xx[196] * (xx[5] - xx[6] * xx[196]) * (- xx[195] == xx[0] ?
    xx[0] : - xx[197]);
  if (xx[0] > xx[198])
    xx[198] = xx[0];
  xx[195] = state[14] - rtdvd[83];
  if (xx[0] > xx[195])
    xx[195] = xx[0];
  xx[196] = xx[195] / xx[2];
  if (xx[3] < xx[196])
    xx[196] = xx[3];
  xx[199] = xx[196] * xx[196] * (xx[5] - xx[6] * xx[196]) * (xx[195] == xx[0] ?
    xx[0] : xx[197]);
  if (xx[0] > xx[199])
    xx[199] = xx[0];
  xx[195] = xx[207] - xx[225] - xx[223] - xx[243];
  xx[196] = xx[209] - xx[227] - xx[229] - xx[245];
  xx[197] = xx[187] + xx[180] + xx[165] * xx[195] - xx[149] * xx[196] + xx[217];
  xx[200] = xx[208] - xx[226] - xx[226] - xx[244];
  memcpy(xx + 201, xx + 200, 1 * sizeof(double));
  ii[0] = factorSymmetricPosDef(xx + 201, 1, xx + 202);
  if (ii[0] != 0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:DegenerateMass",
      "'human9DOF/j7' has a degenerate mass distribution on its follower side.",
      neDiagMgr);
  }

  xx[202] = (xx[198] - 0.1 * state[15] - xx[199] + xx[197]) / xx[201];
  xx[198] = xx[205] - xx[223] - xx[225] - xx[241];
  xx[180] = xx[210] - xx[228] - xx[224] - xx[246];
  xx[187] = xx[212] - xx[230] - xx[230] - xx[248];
  xx[199] = xx[211] - xx[229] - xx[227] - xx[247];
  xx[203] = xx[186] + xx[179] + xx[165] * xx[8] - xx[149] * xx[150] + xx[216] -
    xx[202] * xx[198];
  xx[204] = xx[197] - xx[202] * xx[200];
  xx[205] = xx[188] + xx[181] + xx[165] * xx[180] - xx[149] * xx[187] + xx[218]
    - xx[202] * xx[199];
  pm_math_Quaternion_xform_ra(xx + 157, xx + 203, xx + 206);
  pm_math_Matrix3x3_xform_ra(xx + 231, xx + 143, xx + 203);
  xx[209] = xx[189] + xx[165] * xx[161] - xx[149] * xx[192] + xx[203] - xx[202] *
    xx[166];
  xx[210] = xx[190] + xx[165] * xx[162] - xx[149] * xx[193] + xx[204] - xx[202] *
    xx[184];
  xx[211] = xx[191] + xx[165] * xx[164] - xx[149] * xx[194] + xx[205] - xx[202] *
    xx[185];
  pm_math_Quaternion_xform_ra(xx + 157, xx + 209, xx + 188);
  pm_math_Vector3_cross_ra(xx + 213, xx + 188, xx + 203);
  xx[179] = xx[148] * xx[148];
  xx[181] = xx[154] * xx[155];
  xx[186] = xx[148] * xx[156];
  xx[191] = xx[154] * xx[156];
  xx[197] = xx[148] * xx[155];
  xx[209] = xx[155] * xx[156];
  xx[210] = xx[148] * xx[154];
  xx[216] = (xx[179] + xx[154] * xx[154]) * xx[6] - xx[3];
  xx[217] = xx[6] * (xx[181] - xx[186]);
  xx[218] = (xx[191] + xx[197]) * xx[6];
  xx[219] = (xx[181] + xx[186]) * xx[6];
  xx[220] = (xx[179] + xx[155] * xx[155]) * xx[6] - xx[3];
  xx[221] = xx[6] * (xx[209] - xx[210]);
  xx[222] = xx[6] * (xx[191] - xx[197]);
  xx[223] = (xx[209] + xx[210]) * xx[6];
  xx[224] = (xx[179] + xx[156] * xx[156]) * xx[6] - xx[3];
  xx[148] = xx[198] / xx[201];
  xx[154] = xx[148] * xx[200];
  xx[155] = xx[148] * xx[199];
  xx[156] = xx[200] / xx[201];
  xx[179] = xx[156] * xx[199];
  xx[181] = xx[199] / xx[201];
  xx[240] = xx[8] - xx[148] * xx[198];
  xx[241] = xx[198] - xx[154];
  xx[242] = xx[150] - xx[155];
  xx[243] = xx[195] - xx[154];
  xx[244] = xx[200] - xx[156] * xx[200];
  xx[245] = xx[196] - xx[179];
  xx[246] = xx[180] - xx[155];
  xx[247] = xx[199] - xx[179];
  xx[248] = xx[187] - xx[181] * xx[199];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 240, xx + 216, xx + 249);
  pm_math_Matrix3x3_compose_ra(xx + 216, xx + 249, xx + 240);
  xx[8] = xx[166] / xx[201];
  xx[150] = xx[184] / xx[201];
  xx[154] = xx[185] / xx[201];
  xx[249] = xx[161] - xx[8] * xx[198];
  xx[250] = xx[162] - xx[150] * xx[198];
  xx[251] = xx[164] - xx[154] * xx[198];
  xx[252] = xx[166] - xx[8] * xx[200];
  xx[253] = xx[184] - xx[150] * xx[200];
  xx[254] = xx[185] - xx[154] * xx[200];
  xx[255] = xx[192] - xx[8] * xx[199];
  xx[256] = xx[193] - xx[150] * xx[199];
  xx[257] = xx[194] - xx[154] * xx[199];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 249, xx + 216, xx + 191);
  pm_math_Matrix3x3_compose_ra(xx + 216, xx + 191, xx + 249);
  pm_math_Matrix3x3_postCross_ra(xx + 249, xx + 213, xx + 191);
  xx[155] = xx[8] * xx[184];
  xx[161] = xx[8] * xx[185];
  xx[162] = xx[150] * xx[185];
  xx[258] = xx[231] - xx[8] * xx[166];
  xx[259] = xx[232] - xx[155];
  xx[260] = xx[233] - xx[161];
  xx[261] = xx[234] - xx[155];
  xx[262] = xx[235] - xx[150] * xx[184];
  xx[263] = xx[236] - xx[162];
  xx[264] = xx[237] - xx[161];
  xx[265] = xx[238] - xx[162];
  xx[266] = xx[239] - xx[154] * xx[185];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 258, xx + 216, xx + 225);
  pm_math_Matrix3x3_compose_ra(xx + 216, xx + 225, xx + 258);
  pm_math_Matrix3x3_postCross_ra(xx + 258, xx + 213, xx + 216);
  pm_math_Matrix3x3_preCross_ra(xx + 216, xx + 213, xx + 225);
  xx[155] = xx[240] - xx[191] - xx[191] - xx[225];
  xx[161] = xx[241] - xx[192] - xx[194] - xx[226];
  xx[162] = xx[242] - xx[193] - xx[197] - xx[227];
  xx[164] = xx[243] - xx[194] - xx[192] - xx[228];
  xx[166] = xx[244] - xx[195] - xx[195] - xx[229];
  xx[179] = xx[245] - xx[196] - xx[198] - xx[230];
  xx[180] = xx[246] - xx[197] - xx[193] - xx[231];
  xx[184] = xx[247] - xx[198] - xx[196] - xx[232];
  xx[185] = xx[248] - xx[199] - xx[199] - xx[233];
  xx[191] = xx[138] + xx[155];
  xx[192] = xx[161];
  xx[193] = xx[162];
  xx[194] = xx[164];
  xx[195] = xx[146] + xx[166];
  xx[196] = xx[179];
  xx[197] = xx[180];
  xx[198] = xx[184];
  xx[199] = xx[147] + xx[185];
  xx[209] = xx[123];
  xx[210] = - xx[132];
  xx[211] = - xx[136];
  pm_math_Vector3_cross_ra(xx + 129, xx + 209, xx + 225);
  pm_math_Matrix3x3_xform_ra(xx + 191, xx + 225, xx + 209);
  xx[123] = xx[249] - xx[216];
  xx[132] = xx[250] - xx[219];
  xx[136] = xx[251] - xx[222];
  xx[186] = xx[252] - xx[217];
  xx[187] = xx[253] - xx[220];
  xx[200] = xx[254] - xx[223];
  xx[201] = xx[255] - xx[218];
  xx[212] = xx[256] - xx[221];
  xx[216] = xx[257] - xx[224];
  xx[228] = xx[123];
  xx[229] = xx[132];
  xx[230] = xx[136];
  xx[231] = xx[186];
  xx[232] = xx[187];
  xx[233] = xx[200];
  xx[234] = xx[201];
  xx[235] = xx[212];
  xx[236] = xx[216];
  xx[217] = xx[129] + xx[125];
  xx[218] = xx[130] + xx[135];
  xx[219] = xx[131] + xx[137];
  xx[125] = 0.0115241621900204;
  xx[129] = 0.05252542267538761;
  xx[130] = 0.1488828669086876;
  xx[220] = xx[125] * state[13];
  xx[221] = xx[129] * state[13];
  xx[222] = - (xx[130] * state[13]);
  pm_math_Vector3_cross_ra(xx + 217, xx + 220, xx + 237);
  xx[217] = 0.1174480511045049;
  xx[218] = 0.09974111499921998;
  xx[219] = 0.04427933683814569;
  pm_math_Quaternion_xform_ra(xx + 139, xx + 217, xx + 220);
  xx[217] = 0.0475545670786721 - xx[220];
  xx[218] = - (0.1051997654804633 + xx[221]);
  xx[219] = - (0.01172847704075845 + xx[222]);
  pm_math_Vector3_cross_ra(xx + 117, xx + 217, xx + 220);
  pm_math_Vector3_cross_ra(xx + 117, xx + 220, xx + 240);
  pm_math_Quaternion_inverseXform_ra(xx + 139, xx + 240, xx + 117);
  xx[131] = xx[237] + xx[117];
  xx[135] = xx[238] + xx[118];
  xx[117] = xx[239] + xx[119];
  xx[220] = xx[131];
  xx[221] = xx[135];
  xx[222] = xx[117];
  pm_math_Matrix3x3_xform_ra(xx + 228, xx + 220, xx + 237);
  xx[118] = xx[151] + xx[206] + xx[203] + xx[209] + xx[237];
  xx[240] = xx[124];
  xx[241] = - xx[133];
  xx[242] = - xx[134];
  pm_math_Matrix3x3_xform_ra(xx + 191, xx + 240, xx + 243);
  xx[191] = xx[125];
  xx[192] = xx[129];
  xx[193] = - xx[130];
  pm_math_Matrix3x3_xform_ra(xx + 228, xx + 191, xx + 194);
  xx[119] = xx[243] + xx[194];
  xx[137] = state[12] - rtdvd[63];
  if (xx[0] < xx[137])
    xx[137] = xx[0];
  xx[197] = - (xx[137] / xx[2]);
  if (xx[3] < xx[197])
    xx[197] = xx[3];
  xx[198] = xx[7] * state[13];
  xx[199] = xx[197] * xx[197] * (xx[5] - xx[6] * xx[197]) * (- xx[137] == xx[0] ?
    xx[0] : - xx[198]);
  if (xx[0] > xx[199])
    xx[199] = xx[0];
  xx[137] = state[12] - rtdvd[64];
  if (xx[0] > xx[137])
    xx[137] = xx[0];
  xx[197] = xx[137] / xx[2];
  if (xx[3] < xx[197])
    xx[197] = xx[3];
  xx[223] = xx[197] * xx[197] * (xx[5] - xx[6] * xx[197]) * (xx[137] == xx[0] ?
    xx[0] : xx[198]);
  if (xx[0] > xx[223])
    xx[223] = xx[0];
  xx[137] = xx[152] + xx[207] + xx[204] + xx[210] + xx[238];
  xx[151] = xx[153] + xx[208] + xx[205] + xx[211] + xx[239];
  xx[203] = xx[118];
  xx[204] = xx[137];
  xx[205] = xx[151];
  pm_math_Matrix3x3_transposeXform_ra(xx + 228, xx + 225, xx + 206);
  xx[152] = 2.72155422;
  xx[246] = xx[152] + xx[258];
  xx[247] = xx[259];
  xx[248] = xx[260];
  xx[249] = xx[261];
  xx[250] = xx[152] + xx[262];
  xx[251] = xx[263];
  xx[252] = xx[264];
  xx[253] = xx[265];
  xx[254] = xx[152] + xx[266];
  pm_math_Matrix3x3_xform_ra(xx + 246, xx + 220, xx + 209);
  xx[153] = xx[188] + xx[206] + xx[209];
  xx[188] = xx[189] + xx[207] + xx[210];
  xx[189] = xx[190] + xx[208] + xx[211];
  xx[206] = xx[153];
  xx[207] = xx[188];
  xx[208] = xx[189];
  xx[190] = xx[244] + xx[195];
  xx[194] = xx[245] + xx[196];
  xx[195] = xx[119];
  xx[196] = xx[190];
  xx[197] = xx[194];
  pm_math_Matrix3x3_transposeXform_ra(xx + 228, xx + 240, xx + 209);
  pm_math_Matrix3x3_xform_ra(xx + 246, xx + 191, xx + 220);
  xx[198] = xx[209] + xx[220];
  xx[224] = xx[210] + xx[221];
  xx[209] = xx[211] + xx[222];
  xx[220] = xx[198];
  xx[221] = xx[224];
  xx[222] = xx[209];
  xx[210] = pm_math_Vector3_dot_ra(xx + 240, xx + 195) + pm_math_Vector3_dot_ra
    (xx + 191, xx + 220);
  ii[0] = factorSymmetricPosDef(xx + 210, 1, xx + 195);
  if (ii[0] != 0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:DegenerateMass",
      "'human9DOF/j6' has a degenerate mass distribution on its follower side.",
      neDiagMgr);
  }

  xx[195] = (xx[199] - xx[223] - (pm_math_Vector3_dot_ra(xx + 240, xx + 203) +
              pm_math_Vector3_dot_ra(xx + 191, xx + 206))) / xx[210];
  xx[191] = xx[118] + xx[119] * xx[195];
  xx[192] = xx[137] + xx[190] * xx[195];
  xx[193] = xx[151] + xx[194] * xx[195];
  pm_math_Quaternion_xform_ra(xx + 139, xx + 191, xx + 203);
  xx[191] = xx[153] + xx[198] * xx[195];
  xx[192] = xx[188] + xx[224] * xx[195];
  xx[193] = xx[189] + xx[209] * xx[195];
  pm_math_Quaternion_xform_ra(xx + 139, xx + 191, xx + 206);
  pm_math_Vector3_cross_ra(xx + 217, xx + 206, xx + 191);
  xx[118] = xx[139] * xx[139];
  xx[137] = xx[140] * xx[141];
  xx[151] = xx[139] * xx[142];
  xx[153] = xx[140] * xx[142];
  xx[188] = xx[139] * xx[141];
  xx[189] = xx[141] * xx[142];
  xx[196] = xx[139] * xx[140];
  xx[228] = (xx[118] + xx[140] * xx[140]) * xx[6] - xx[3];
  xx[229] = xx[6] * (xx[137] - xx[151]);
  xx[230] = (xx[153] + xx[188]) * xx[6];
  xx[231] = (xx[137] + xx[151]) * xx[6];
  xx[232] = (xx[118] + xx[141] * xx[141]) * xx[6] - xx[3];
  xx[233] = xx[6] * (xx[189] - xx[196]);
  xx[234] = xx[6] * (xx[153] - xx[188]);
  xx[235] = (xx[189] + xx[196]) * xx[6];
  xx[236] = (xx[118] + xx[142] * xx[142]) * xx[6] - xx[3];
  xx[118] = xx[119] / xx[210];
  xx[137] = xx[190] * xx[118];
  xx[151] = xx[194] * xx[118];
  xx[153] = xx[190] / xx[210];
  xx[188] = xx[194] * xx[153];
  xx[189] = xx[194] / xx[210];
  xx[237] = xx[155] - xx[119] * xx[118] + xx[138];
  xx[238] = xx[161] - xx[137];
  xx[239] = xx[162] - xx[151];
  xx[240] = xx[164] - xx[137];
  xx[241] = xx[166] - xx[190] * xx[153] + xx[146];
  xx[242] = xx[179] - xx[188];
  xx[243] = xx[180] - xx[151];
  xx[244] = xx[184] - xx[188];
  xx[245] = xx[185] - xx[194] * xx[189] + xx[147];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 237, xx + 228, xx + 246);
  pm_math_Matrix3x3_compose_ra(xx + 228, xx + 246, xx + 237);
  xx[137] = xx[198] / xx[210];
  xx[138] = xx[224] / xx[210];
  xx[146] = xx[209] / xx[210];
  xx[246] = xx[123] - xx[119] * xx[137];
  xx[247] = xx[132] - xx[119] * xx[138];
  xx[248] = xx[136] - xx[119] * xx[146];
  xx[249] = xx[186] - xx[190] * xx[137];
  xx[250] = xx[187] - xx[190] * xx[138];
  xx[251] = xx[200] - xx[190] * xx[146];
  xx[252] = xx[201] - xx[194] * xx[137];
  xx[253] = xx[212] - xx[194] * xx[138];
  xx[254] = xx[216] - xx[194] * xx[146];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 246, xx + 228, xx + 267);
  pm_math_Matrix3x3_compose_ra(xx + 228, xx + 267, xx + 246);
  pm_math_Matrix3x3_postCross_ra(xx + 246, xx + 217, xx + 267);
  xx[119] = xx[224] * xx[137];
  xx[123] = xx[209] * xx[137];
  xx[132] = xx[209] * xx[138];
  xx[276] = xx[258] - xx[198] * xx[137] + xx[152];
  xx[277] = xx[259] - xx[119];
  xx[278] = xx[260] - xx[123];
  xx[279] = xx[261] - xx[119];
  xx[280] = xx[262] - xx[224] * xx[138] + xx[152];
  xx[281] = xx[263] - xx[132];
  xx[282] = xx[264] - xx[123];
  xx[283] = xx[265] - xx[132];
  xx[284] = xx[266] - xx[209] * xx[146] + xx[152];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 276, xx + 228, xx + 255);
  pm_math_Matrix3x3_compose_ra(xx + 228, xx + 255, xx + 276);
  pm_math_Matrix3x3_postCross_ra(xx + 276, xx + 217, xx + 228);
  pm_math_Matrix3x3_preCross_ra(xx + 228, xx + 217, xx + 255);
  xx[119] = xx[237] - xx[267] - xx[267] - xx[255];
  xx[123] = xx[238] - xx[268] - xx[270] - xx[256];
  xx[132] = xx[239] - xx[269] - xx[273] - xx[257];
  xx[136] = xx[240] - xx[270] - xx[268] - xx[258];
  xx[147] = xx[241] - xx[271] - xx[271] - xx[259];
  xx[151] = xx[242] - xx[272] - xx[274] - xx[260];
  xx[152] = xx[243] - xx[273] - xx[269] - xx[261];
  xx[155] = xx[244] - xx[274] - xx[272] - xx[262];
  xx[161] = xx[245] - xx[275] - xx[275] - xx[263];
  xx[237] = xx[120] + xx[119];
  xx[238] = xx[123];
  xx[239] = xx[132];
  xx[240] = xx[136];
  xx[241] = xx[121] + xx[147];
  xx[242] = xx[151];
  xx[243] = xx[152];
  xx[244] = xx[155];
  xx[245] = xx[122] + xx[161];
  xx[184] = xx[66];
  xx[185] = xx[99];
  xx[186] = - xx[106];
  pm_math_Vector3_cross_ra(xx + 114, xx + 184, xx + 196);
  pm_math_Matrix3x3_xform_ra(xx + 237, xx + 196, xx + 184);
  xx[66] = xx[246] - xx[228];
  xx[99] = xx[247] - xx[231];
  xx[106] = xx[248] - xx[234];
  xx[162] = xx[249] - xx[229];
  xx[164] = xx[250] - xx[232];
  xx[166] = xx[251] - xx[235];
  xx[179] = xx[252] - xx[230];
  xx[180] = xx[253] - xx[233];
  xx[187] = xx[254] - xx[236];
  xx[228] = xx[66];
  xx[229] = xx[99];
  xx[230] = xx[106];
  xx[231] = xx[162];
  xx[232] = xx[164];
  xx[233] = xx[166];
  xx[234] = xx[179];
  xx[235] = xx[180];
  xx[236] = xx[187];
  xx[199] = xx[114] + xx[96];
  xx[200] = xx[115] + xx[100];
  xx[201] = xx[116] + xx[112];
  xx[96] = 0.1232317192063306;
  xx[100] = 0.07120994256178509;
  xx[112] = 0.03694148414451474;
  xx[114] = - (xx[96] * state[11]);
  xx[115] = - (xx[100] * state[11]);
  xx[116] = - (xx[112] * state[11]);
  pm_math_Vector3_cross_ra(xx + 199, xx + 114, xx + 209);
  xx[114] = - 0.0786896255494641;
  xx[115] = 0.1231236283902652;
  xx[116] = 0.02515955587646605;
  pm_math_Quaternion_xform_ra(xx + 107, xx + 114, xx + 199);
  xx[114] = - xx[199];
  xx[115] = - xx[200];
  xx[116] = - xx[201];
  pm_math_Vector3_cross_ra(xx + 103, xx + 114, xx + 199);
  pm_math_Vector3_cross_ra(xx + 103, xx + 199, xx + 220);
  pm_math_Quaternion_inverseXform_ra(xx + 107, xx + 220, xx + 103);
  xx[188] = xx[209] + xx[103];
  xx[190] = xx[210] + xx[104];
  xx[103] = xx[211] + xx[105];
  xx[199] = xx[188];
  xx[200] = xx[190];
  xx[201] = xx[103];
  pm_math_Matrix3x3_xform_ra(xx + 228, xx + 199, xx + 209);
  xx[104] = xx[126] + xx[203] + xx[191] + xx[184] + xx[209];
  xx[220] = xx[88];
  xx[221] = xx[101];
  xx[222] = - xx[102];
  pm_math_Matrix3x3_xform_ra(xx + 237, xx + 220, xx + 246);
  xx[237] = - xx[96];
  xx[238] = - xx[100];
  xx[239] = - xx[112];
  pm_math_Matrix3x3_xform_ra(xx + 228, xx + 237, xx + 240);
  xx[105] = xx[246] + xx[240];
  xx[194] = state[10] - rtdvd[53];
  if (xx[0] < xx[194])
    xx[194] = xx[0];
  xx[212] = - (xx[194] / xx[2]);
  if (xx[3] < xx[212])
    xx[212] = xx[3];
  xx[216] = xx[7] * state[11];
  xx[223] = xx[212] * xx[212] * (xx[5] - xx[6] * xx[212]) * ((- xx[194] == xx[0]
    ? xx[0] : - xx[216]) - xx[77] * xx[194]);
  if (xx[0] > xx[223])
    xx[223] = xx[0];
  xx[194] = state[10] - rtdvd[54];
  if (xx[0] > xx[194])
    xx[194] = xx[0];
  xx[212] = xx[194] / xx[2];
  if (xx[3] < xx[212])
    xx[212] = xx[3];
  xx[224] = (xx[77] * xx[194] + (xx[194] == xx[0] ? xx[0] : xx[216])) * xx[212] *
    xx[212] * (xx[5] - xx[6] * xx[212]);
  if (xx[0] > xx[224])
    xx[224] = xx[0];
  xx[194] = xx[127] + xx[204] + xx[192] + xx[185] + xx[210];
  xx[126] = xx[128] + xx[205] + xx[193] + xx[186] + xx[211];
  xx[184] = xx[104];
  xx[185] = xx[194];
  xx[186] = xx[126];
  pm_math_Matrix3x3_transposeXform_ra(xx + 228, xx + 196, xx + 191);
  xx[127] = 3.62873896;
  xx[249] = xx[127] + xx[276];
  xx[250] = xx[277];
  xx[251] = xx[278];
  xx[252] = xx[279];
  xx[253] = xx[127] + xx[280];
  xx[254] = xx[281];
  xx[255] = xx[282];
  xx[256] = xx[283];
  xx[257] = xx[127] + xx[284];
  pm_math_Matrix3x3_xform_ra(xx + 249, xx + 199, xx + 203);
  xx[128] = xx[206] + xx[191] + xx[203];
  xx[199] = xx[207] + xx[192] + xx[204];
  xx[191] = xx[208] + xx[193] + xx[205];
  xx[203] = xx[128];
  xx[204] = xx[199];
  xx[205] = xx[191];
  xx[192] = xx[247] + xx[241];
  xx[193] = xx[248] + xx[242];
  xx[206] = xx[105];
  xx[207] = xx[192];
  xx[208] = xx[193];
  pm_math_Matrix3x3_transposeXform_ra(xx + 228, xx + 220, xx + 209);
  pm_math_Matrix3x3_xform_ra(xx + 249, xx + 237, xx + 228);
  xx[200] = xx[209] + xx[228];
  xx[201] = xx[210] + xx[229];
  xx[209] = xx[211] + xx[230];
  xx[210] = xx[200];
  xx[211] = xx[201];
  xx[212] = xx[209];
  xx[216] = pm_math_Vector3_dot_ra(xx + 220, xx + 206) + pm_math_Vector3_dot_ra
    (xx + 237, xx + 210);
  ii[0] = factorSymmetricPosDef(xx + 216, 1, xx + 206);
  if (ii[0] != 0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:DegenerateMass",
      "'human9DOF/curl' has a degenerate mass distribution on its follower side.",
      neDiagMgr);
  }

  xx[206] = (xx[223] - state[11] - xx[224] - (pm_math_Vector3_dot_ra(xx + 220,
    xx + 184) + pm_math_Vector3_dot_ra(xx + 237, xx + 203))) / xx[216];
  xx[184] = xx[104] + xx[105] * xx[206];
  xx[185] = xx[194] + xx[192] * xx[206];
  xx[186] = xx[126] + xx[193] * xx[206];
  pm_math_Quaternion_xform_ra(xx + 107, xx + 184, xx + 203);
  xx[184] = xx[128] + xx[200] * xx[206];
  xx[185] = xx[199] + xx[201] * xx[206];
  xx[186] = xx[191] + xx[209] * xx[206];
  pm_math_Quaternion_xform_ra(xx + 107, xx + 184, xx + 210);
  pm_math_Vector3_cross_ra(xx + 114, xx + 210, xx + 184);
  xx[104] = xx[113] * state[9];
  xx[113] = xx[107] * xx[107];
  xx[126] = xx[108] * xx[109];
  xx[128] = xx[107] * xx[110];
  xx[191] = xx[108] * xx[110];
  xx[194] = xx[107] * xx[109];
  xx[199] = xx[109] * xx[110];
  xx[207] = xx[107] * xx[108];
  xx[228] = (xx[113] + xx[108] * xx[108]) * xx[6] - xx[3];
  xx[229] = xx[6] * (xx[126] - xx[128]);
  xx[230] = (xx[191] + xx[194]) * xx[6];
  xx[231] = (xx[126] + xx[128]) * xx[6];
  xx[232] = (xx[113] + xx[109] * xx[109]) * xx[6] - xx[3];
  xx[233] = xx[6] * (xx[199] - xx[207]);
  xx[234] = xx[6] * (xx[191] - xx[194]);
  xx[235] = (xx[199] + xx[207]) * xx[6];
  xx[236] = (xx[113] + xx[110] * xx[110]) * xx[6] - xx[3];
  xx[113] = xx[105] / xx[216];
  xx[126] = xx[192] * xx[113];
  xx[128] = xx[193] * xx[113];
  xx[191] = xx[192] / xx[216];
  xx[194] = xx[193] * xx[191];
  xx[199] = xx[193] / xx[216];
  xx[237] = xx[119] - xx[105] * xx[113] + xx[120];
  xx[238] = xx[123] - xx[126];
  xx[239] = xx[132] - xx[128];
  xx[240] = xx[136] - xx[126];
  xx[241] = xx[147] - xx[192] * xx[191] + xx[121];
  xx[242] = xx[151] - xx[194];
  xx[243] = xx[152] - xx[128];
  xx[244] = xx[155] - xx[194];
  xx[245] = xx[161] - xx[193] * xx[199] + xx[122];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 237, xx + 228, xx + 246);
  pm_math_Matrix3x3_compose_ra(xx + 228, xx + 246, xx + 237);
  xx[119] = xx[200] / xx[216];
  xx[120] = xx[201] / xx[216];
  xx[121] = xx[209] / xx[216];
  xx[246] = xx[66] - xx[105] * xx[119];
  xx[247] = xx[99] - xx[105] * xx[120];
  xx[248] = xx[106] - xx[105] * xx[121];
  xx[249] = xx[162] - xx[192] * xx[119];
  xx[250] = xx[164] - xx[192] * xx[120];
  xx[251] = xx[166] - xx[192] * xx[121];
  xx[252] = xx[179] - xx[193] * xx[119];
  xx[253] = xx[180] - xx[193] * xx[120];
  xx[254] = xx[187] - xx[193] * xx[121];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 246, xx + 228, xx + 255);
  pm_math_Matrix3x3_compose_ra(xx + 228, xx + 255, xx + 246);
  pm_math_Matrix3x3_postCross_ra(xx + 246, xx + 114, xx + 255);
  xx[66] = xx[201] * xx[119];
  xx[99] = xx[209] * xx[119];
  xx[105] = xx[209] * xx[120];
  xx[264] = xx[276] - xx[200] * xx[119] + xx[127];
  xx[265] = xx[277] - xx[66];
  xx[266] = xx[278] - xx[99];
  xx[267] = xx[279] - xx[66];
  xx[268] = xx[280] - xx[201] * xx[120] + xx[127];
  xx[269] = xx[281] - xx[105];
  xx[270] = xx[282] - xx[99];
  xx[271] = xx[283] - xx[105];
  xx[272] = xx[284] - xx[209] * xx[121] + xx[127];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 264, xx + 228, xx + 273);
  pm_math_Matrix3x3_compose_ra(xx + 228, xx + 273, xx + 264);
  pm_math_Matrix3x3_postCross_ra(xx + 264, xx + 114, xx + 228);
  pm_math_Matrix3x3_preCross_ra(xx + 228, xx + 114, xx + 273);
  xx[66] = xx[237] - xx[255] - xx[255] - xx[273];
  xx[99] = xx[111] * state[9];
  xx[105] = xx[239] - xx[257] - xx[261] - xx[275];
  xx[106] = state[8] + 1.91488952190423;
  if (xx[0] < xx[106])
    xx[106] = xx[0];
  xx[111] = - (xx[106] / xx[2]);
  if (xx[3] < xx[111])
    xx[111] = xx[3];
  xx[122] = xx[7] * state[9];
  xx[123] = xx[111] * xx[111] * (xx[5] - xx[6] * xx[111]) * ((- xx[106] == xx[0]
    ? xx[0] : - xx[122]) - xx[77] * xx[106]);
  if (xx[0] > xx[123])
    xx[123] = xx[0];
  xx[106] = 57.29577951308232;
  xx[111] = state[8] - 0.1795055804889654;
  if (xx[0] > xx[111])
    xx[111] = xx[0];
  xx[126] = xx[111] / xx[2];
  if (xx[3] < xx[126])
    xx[126] = xx[3];
  xx[127] = (xx[77] * xx[111] + (xx[111] == xx[0] ? xx[0] : xx[122])) * xx[126] *
    xx[126] * (xx[5] - xx[6] * xx[126]);
  if (xx[0] > xx[127])
    xx[127] = xx[0];
  xx[111] = xx[240] - xx[258] - xx[256] - xx[276];
  xx[122] = xx[242] - xx[260] - xx[262] - xx[278];
  xx[126] = xx[204] + xx[185] + xx[104] * xx[111] - xx[99] * xx[122];
  xx[128] = xx[241] - xx[259] - xx[259] - xx[277];
  memcpy(xx + 132, xx + 128, 1 * sizeof(double));
  ii[0] = factorSymmetricPosDef(xx + 132, 1, xx + 136);
  if (ii[0] != 0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:DegenerateMass",
      "'human9DOF/butterfly' has a degenerate mass distribution on its follower side.",
      neDiagMgr);
  }

  xx[136] = (xx[123] - (xx[106] * state[8] + xx[106] * state[9]) - xx[127] + xx
             [126]) / xx[132];
  xx[106] = xx[238] - xx[256] - xx[258] - xx[274];
  xx[123] = xx[243] - xx[261] - xx[257] - xx[279];
  xx[127] = xx[245] - xx[263] - xx[263] - xx[281];
  xx[147] = xx[244] - xx[262] - xx[260] - xx[280];
  xx[192] = xx[203] + xx[184] + xx[104] * xx[66] - xx[99] * xx[105] - xx[136] *
    xx[106];
  xx[193] = xx[126] - xx[136] * xx[128];
  xx[194] = xx[205] + xx[186] + xx[104] * xx[123] - xx[99] * xx[127] - xx[136] *
    xx[147];
  pm_math_Quaternion_xform_ra(xx + 92, xx + 192, xx + 184);
  xx[126] = xx[60] * xx[60];
  xx[151] = xx[89] * xx[90];
  xx[152] = xx[60] * xx[91];
  xx[155] = xx[89] * xx[91];
  xx[161] = xx[60] * xx[90];
  xx[162] = xx[90] * xx[91];
  xx[164] = xx[60] * xx[89];
  xx[237] = (xx[126] + xx[89] * xx[89]) * xx[6] - xx[3];
  xx[238] = xx[6] * (xx[151] - xx[152]);
  xx[239] = (xx[155] + xx[161]) * xx[6];
  xx[240] = (xx[151] + xx[152]) * xx[6];
  xx[241] = (xx[126] + xx[90] * xx[90]) * xx[6] - xx[3];
  xx[242] = xx[6] * (xx[162] - xx[164]);
  xx[243] = xx[6] * (xx[155] - xx[161]);
  xx[244] = (xx[162] + xx[164]) * xx[6];
  xx[245] = (xx[126] + xx[91] * xx[91]) * xx[6] - xx[3];
  xx[60] = xx[106] / xx[132];
  xx[89] = xx[60] * xx[128];
  xx[90] = xx[60] * xx[147];
  xx[91] = xx[128] / xx[132];
  xx[126] = xx[91] * xx[147];
  xx[151] = xx[147] / xx[132];
  xx[255] = xx[66] - xx[60] * xx[106];
  xx[256] = xx[106] - xx[89];
  xx[257] = xx[105] - xx[90];
  xx[258] = xx[111] - xx[89];
  xx[259] = xx[128] - xx[91] * xx[128];
  xx[260] = xx[122] - xx[126];
  xx[261] = xx[123] - xx[90];
  xx[262] = xx[147] - xx[126];
  xx[263] = xx[127] - xx[151] * xx[147];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 255, xx + 237, xx + 273);
  pm_math_Matrix3x3_compose_ra(xx + 237, xx + 273, xx + 255);
  xx[66] = xx[98] * state[7];
  xx[89] = xx[97] * state[7];
  xx[90] = xx[246] - xx[228];
  xx[97] = xx[249] - xx[229];
  xx[98] = xx[97] / xx[132];
  xx[105] = xx[247] - xx[231];
  xx[111] = xx[250] - xx[232];
  xx[122] = xx[111] / xx[132];
  xx[123] = xx[248] - xx[234];
  xx[126] = xx[251] - xx[235];
  xx[127] = xx[126] / xx[132];
  xx[132] = xx[252] - xx[230];
  xx[152] = xx[253] - xx[233];
  xx[155] = xx[254] - xx[236];
  xx[228] = xx[90] - xx[98] * xx[106];
  xx[229] = xx[105] - xx[122] * xx[106];
  xx[230] = xx[123] - xx[127] * xx[106];
  xx[231] = xx[97] - xx[98] * xx[128];
  xx[232] = xx[111] - xx[122] * xx[128];
  xx[233] = xx[126] - xx[127] * xx[128];
  xx[234] = xx[132] - xx[98] * xx[147];
  xx[235] = xx[152] - xx[122] * xx[147];
  xx[236] = xx[155] - xx[127] * xx[147];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 228, xx + 237, xx + 246);
  pm_math_Matrix3x3_compose_ra(xx + 237, xx + 246, xx + 228);
  xx[192] = - 0.2244123868924967;
  xx[193] = 9.872944805650204e-3;
  xx[194] = 1.177299620836221e-3;
  pm_math_Vector3_cross_ra(xx + 40, xx + 192, xx + 203);
  pm_math_Vector3_cross_ra(xx + 40, xx + 203, xx + 207);
  pm_math_Quaternion_inverseXform_ra(xx + 84, xx + 207, xx + 40);
  pm_math_Matrix3x3_xform_ra(xx + 228, xx + 40, xx + 203);
  xx[106] = xx[184] + xx[256] * xx[66] - xx[257] * xx[89] + xx[203];
  xx[128] = state[6] - rtdvd[26];
  if (xx[0] < xx[128])
    xx[128] = xx[0];
  xx[147] = - (xx[128] / xx[2]);
  if (xx[3] < xx[147])
    xx[147] = xx[3];
  xx[161] = xx[7] * state[7];
  xx[162] = xx[147] * xx[147] * (xx[5] - xx[6] * xx[147]) * ((- xx[128] == xx[0]
    ? xx[0] : - xx[161]) - xx[77] * xx[128]);
  if (xx[0] > xx[162])
    xx[162] = xx[0];
  xx[128] = state[6] - 1.398482201733458;
  if (xx[0] > xx[128])
    xx[128] = xx[0];
  xx[147] = xx[128] / xx[2];
  if (xx[3] < xx[147])
    xx[147] = xx[3];
  xx[164] = (xx[77] * xx[128] + (xx[128] == xx[0] ? xx[0] : xx[161])) * xx[147] *
    xx[147] * (xx[5] - xx[6] * xx[147]);
  if (xx[0] > xx[164])
    xx[164] = xx[0];
  memcpy(xx + 128, xx + 255, 1 * sizeof(double));
  ii[0] = factorSymmetricPosDef(xx + 128, 1, xx + 147);
  if (ii[0] != 0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:DegenerateMass",
      "'human9DOF/Chicken Wing' has a degenerate mass distribution on its follower side.",
      neDiagMgr);
  }

  xx[147] = (xx[162] - xx[164] - xx[106]) / xx[128];
  xx[207] = xx[106] + xx[147] * xx[255];
  xx[208] = xx[185] + xx[259] * xx[66] - xx[260] * xx[89] + xx[204] + xx[147] *
    xx[258];
  xx[209] = xx[186] + xx[262] * xx[66] - xx[263] * xx[89] + xx[205] + xx[147] *
    xx[261];
  pm_math_Quaternion_xform_ra(xx + 84, xx + 207, xx + 184);
  xx[203] = xx[210] + xx[104] * xx[90] - xx[99] * xx[132] - xx[136] * xx[97];
  xx[204] = xx[211] + xx[104] * xx[105] - xx[99] * xx[152] - xx[136] * xx[111];
  xx[205] = xx[212] + xx[104] * xx[123] - xx[99] * xx[155] - xx[136] * xx[126];
  pm_math_Quaternion_xform_ra(xx + 92, xx + 203, xx + 207);
  xx[90] = xx[98] * xx[111];
  xx[105] = xx[98] * xx[126];
  xx[106] = xx[122] * xx[126];
  xx[246] = xx[264] - xx[98] * xx[97];
  xx[247] = xx[265] - xx[90];
  xx[248] = xx[266] - xx[105];
  xx[249] = xx[267] - xx[90];
  xx[250] = xx[268] - xx[122] * xx[111];
  xx[251] = xx[269] - xx[106];
  xx[252] = xx[270] - xx[105];
  xx[253] = xx[271] - xx[106];
  xx[254] = xx[272] - xx[127] * xx[126];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 246, xx + 237, xx + 264);
  pm_math_Matrix3x3_compose_ra(xx + 237, xx + 264, xx + 246);
  pm_math_Matrix3x3_xform_ra(xx + 246, xx + 40, xx + 203);
  xx[210] = xx[207] + xx[231] * xx[66] - xx[234] * xx[89] + xx[203] + xx[147] *
    xx[228];
  xx[211] = xx[208] + xx[232] * xx[66] - xx[235] * xx[89] + xx[204] + xx[147] *
    xx[229];
  xx[212] = xx[209] + xx[233] * xx[66] - xx[236] * xx[89] + xx[205] + xx[147] *
    xx[230];
  pm_math_Quaternion_xform_ra(xx + 84, xx + 210, xx + 203);
  pm_math_Vector3_cross_ra(xx + 192, xx + 203, xx + 207);
  xx[90] = xx[62] * xx[62];
  xx[97] = xx[63] * xx[64];
  xx[105] = xx[62] * xx[65];
  xx[106] = xx[63] * xx[65];
  xx[111] = xx[62] * xx[64];
  xx[123] = xx[64] * xx[65];
  xx[126] = xx[62] * xx[63];
  xx[237] = (xx[90] + xx[63] * xx[63]) * xx[6] - xx[3];
  xx[238] = xx[6] * (xx[97] - xx[105]);
  xx[239] = (xx[106] + xx[111]) * xx[6];
  xx[240] = (xx[97] + xx[105]) * xx[6];
  xx[241] = (xx[90] + xx[64] * xx[64]) * xx[6] - xx[3];
  xx[242] = xx[6] * (xx[123] - xx[126]);
  xx[243] = xx[6] * (xx[106] - xx[111]);
  xx[244] = (xx[123] + xx[126]) * xx[6];
  xx[245] = (xx[90] + xx[65] * xx[65]) * xx[6] - xx[3];
  xx[90] = xx[55] / xx[58];
  xx[97] = xx[67] * xx[90];
  xx[105] = xx[68] * xx[90];
  xx[106] = xx[67] / xx[58];
  xx[111] = - (xx[68] * xx[106]);
  xx[264] = xx[61] - xx[55] * xx[90];
  xx[265] = xx[97];
  xx[266] = xx[105];
  xx[267] = xx[97];
  xx[268] = xx[69] - xx[67] * xx[106];
  xx[269] = xx[111];
  xx[270] = xx[105];
  xx[271] = xx[111];
  xx[272] = xx[70] - 2.433279434201686e-7 / xx[58];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 264, xx + 237, xx + 273);
  pm_math_Matrix3x3_compose_ra(xx + 237, xx + 273, xx + 264);
  xx[61] = xx[46] / xx[58];
  xx[69] = xx[47] / xx[58];
  xx[70] = xx[56] / xx[58];
  xx[273] = xx[55] * xx[61];
  xx[274] = xx[55] * xx[69];
  xx[275] = - (xx[55] * xx[70]);
  xx[276] = - (xx[67] * xx[61]);
  xx[277] = - (xx[67] * xx[69]);
  xx[278] = xx[67] * xx[70];
  xx[279] = - (xx[68] * xx[61]);
  xx[280] = - (xx[68] * xx[69]);
  xx[281] = xx[68] * xx[70];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 273, xx + 237, xx + 282);
  pm_math_Matrix3x3_compose_ra(xx + 237, xx + 282, xx + 273);
  pm_math_Matrix3x3_postCross_ra(xx + 273, xx + 52, xx + 282);
  xx[55] = - (xx[47] * xx[61]);
  xx[67] = xx[56] * xx[61];
  xx[97] = xx[56] * xx[69];
  xx[291] = xx[48] - xx[46] * xx[61];
  xx[292] = xx[55];
  xx[293] = xx[67];
  xx[294] = xx[55];
  xx[295] = xx[48] - xx[47] * xx[69];
  xx[296] = xx[97];
  xx[297] = xx[67];
  xx[298] = xx[97];
  xx[299] = xx[48] - xx[56] * xx[70];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 291, xx + 237, xx + 300);
  pm_math_Matrix3x3_compose_ra(xx + 237, xx + 300, xx + 291);
  pm_math_Matrix3x3_postCross_ra(xx + 291, xx + 52, xx + 237);
  pm_math_Matrix3x3_preCross_ra(xx + 237, xx + 52, xx + 300);
  xx[46] = xx[57] * xx[57];
  xx[47] = xx[82] * xx[81];
  xx[48] = xx[57] * xx[83];
  xx[55] = xx[81] * xx[83];
  xx[56] = xx[82] * xx[57];
  xx[67] = xx[82] * xx[83];
  xx[97] = xx[81] * xx[57];
  xx[309] = (xx[46] + xx[81] * xx[81]) * xx[6] - xx[3];
  xx[310] = xx[6] * (xx[47] - xx[48]);
  xx[311] = (xx[55] + xx[56]) * xx[6];
  xx[312] = (xx[47] + xx[48]) * xx[6];
  xx[313] = (xx[46] + xx[82] * xx[82]) * xx[6] - xx[3];
  xx[314] = xx[6] * (xx[67] - xx[97]);
  xx[315] = xx[6] * (xx[55] - xx[56]);
  xx[316] = (xx[67] + xx[97]) * xx[6];
  xx[317] = (xx[46] + xx[83] * xx[83]) * xx[6] - xx[3];
  xx[46] = xx[255] / xx[128];
  xx[47] = xx[46] * xx[258];
  xx[48] = xx[46] * xx[261];
  xx[55] = xx[258] / xx[128];
  xx[56] = xx[55] * xx[261];
  xx[57] = xx[261] / xx[128];
  xx[318] = xx[255] - xx[46] * xx[255];
  xx[319] = xx[256] - xx[47];
  xx[320] = xx[257] - xx[48];
  xx[321] = xx[258] - xx[47];
  xx[322] = xx[259] - xx[55] * xx[258];
  xx[323] = xx[260] - xx[56];
  xx[324] = xx[261] - xx[48];
  xx[325] = xx[262] - xx[56];
  xx[326] = xx[263] - xx[57] * xx[261];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 318, xx + 309, xx + 327);
  pm_math_Matrix3x3_compose_ra(xx + 309, xx + 327, xx + 318);
  xx[47] = xx[228] / xx[128];
  xx[48] = xx[229] / xx[128];
  xx[56] = xx[230] / xx[128];
  xx[327] = xx[228] - xx[47] * xx[255];
  xx[328] = xx[229] - xx[48] * xx[255];
  xx[329] = xx[230] - xx[56] * xx[255];
  xx[330] = xx[231] - xx[47] * xx[258];
  xx[331] = xx[232] - xx[48] * xx[258];
  xx[332] = xx[233] - xx[56] * xx[258];
  xx[333] = xx[234] - xx[47] * xx[261];
  xx[334] = xx[235] - xx[48] * xx[261];
  xx[335] = xx[236] - xx[56] * xx[261];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 327, xx + 309, xx + 255);
  pm_math_Matrix3x3_compose_ra(xx + 309, xx + 255, xx + 327);
  pm_math_Matrix3x3_postCross_ra(xx + 327, xx + 192, xx + 255);
  xx[67] = xx[47] * xx[229];
  xx[81] = xx[47] * xx[230];
  xx[82] = xx[48] * xx[230];
  xx[336] = xx[246] - xx[47] * xx[228];
  xx[337] = xx[247] - xx[67];
  xx[338] = xx[248] - xx[81];
  xx[339] = xx[249] - xx[67];
  xx[340] = xx[250] - xx[48] * xx[229];
  xx[341] = xx[251] - xx[82];
  xx[342] = xx[252] - xx[81];
  xx[343] = xx[253] - xx[82];
  xx[344] = xx[254] - xx[56] * xx[230];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 336, xx + 309, xx + 228);
  pm_math_Matrix3x3_compose_ra(xx + 309, xx + 228, xx + 246);
  pm_math_Matrix3x3_postCross_ra(xx + 246, xx + 192, xx + 228);
  pm_math_Matrix3x3_preCross_ra(xx + 228, xx + 192, xx + 309);
  xx[67] = xx[43] + xx[264] - xx[282] - xx[282] - xx[300] + xx[318] - xx[255] -
    xx[255] - xx[309];
  xx[43] = xx[265] - xx[283] - xx[285] - xx[301] + xx[319] - xx[256] - xx[258] -
    xx[310];
  xx[81] = xx[266] - xx[284] - xx[288] - xx[302] + xx[320] - xx[257] - xx[261] -
    xx[311];
  xx[82] = xx[267] - xx[285] - xx[283] - xx[303] + xx[321] - xx[258] - xx[256] -
    xx[312];
  xx[83] = xx[44] + xx[268] - xx[286] - xx[286] - xx[304] + xx[322] - xx[259] -
    xx[259] - xx[313];
  xx[44] = xx[269] - xx[287] - xx[289] - xx[305] + xx[323] - xx[260] - xx[262] -
    xx[314];
  xx[97] = xx[270] - xx[288] - xx[284] - xx[306] + xx[324] - xx[261] - xx[257] -
    xx[315];
  xx[105] = xx[271] - xx[289] - xx[287] - xx[307] + xx[325] - xx[262] - xx[260]
    - xx[316];
  xx[111] = xx[45] + xx[272] - xx[290] - xx[290] - xx[308] + xx[326] - xx[263] -
    xx[263] - xx[317];
  xx[255] = xx[67];
  xx[256] = xx[43];
  xx[257] = xx[81];
  xx[258] = xx[82];
  xx[259] = xx[83];
  xx[260] = xx[44];
  xx[261] = xx[97];
  xx[262] = xx[105];
  xx[263] = xx[111];
  xx[210] = xx[11];
  xx[211] = - xx[24];
  xx[212] = - xx[31];
  pm_math_Vector3_cross_ra(xx + 36, xx + 210, xx + 220);
  pm_math_Matrix3x3_xform_ra(xx + 255, xx + 220, xx + 210);
  xx[11] = xx[273] - xx[237] + xx[327] - xx[228];
  xx[24] = xx[274] - xx[240] + xx[328] - xx[231];
  xx[31] = xx[275] - xx[243] + xx[329] - xx[234];
  xx[45] = xx[276] - xx[238] + xx[330] - xx[229];
  xx[123] = xx[277] - xx[241] + xx[331] - xx[232];
  xx[126] = xx[278] - xx[244] + xx[332] - xx[235];
  xx[128] = xx[279] - xx[239] + xx[333] - xx[230];
  xx[132] = xx[280] - xx[242] + xx[334] - xx[233];
  xx[152] = xx[281] - xx[245] + xx[335] - xx[236];
  xx[228] = xx[11];
  xx[229] = xx[24];
  xx[230] = xx[31];
  xx[231] = xx[45];
  xx[232] = xx[123];
  xx[233] = xx[126];
  xx[234] = xx[128];
  xx[235] = xx[132];
  xx[236] = xx[152];
  xx[237] = xx[36] + xx[21];
  xx[238] = xx[37] + xx[25];
  xx[239] = xx[38] + xx[39];
  xx[21] = 3.88810641306223e-5;
  xx[25] = 0.03617154622233763;
  xx[36] = 0.3675504912767608;
  xx[37] = xx[21] * state[5];
  xx[38] = xx[25] * state[5];
  xx[39] = xx[36] * state[5];
  pm_math_Vector3_cross_ra(xx + 237, xx + 37, xx + 240);
  xx[37] = - 0.02644395617389824;
  xx[38] = - 0.3675397405234455;
  xx[39] = 0.03617328556917723;
  pm_math_Quaternion_xform_ra(xx + 32, xx + 37, xx + 237);
  xx[37] = - xx[237];
  xx[38] = - xx[238];
  xx[39] = - xx[239];
  pm_math_Vector3_cross_ra(xx + 28, xx + 37, xx + 237);
  pm_math_Vector3_cross_ra(xx + 28, xx + 237, xx + 243);
  pm_math_Quaternion_inverseXform_ra(xx + 32, xx + 243, xx + 28);
  xx[155] = xx[240] + xx[28];
  xx[161] = xx[241] + xx[29];
  xx[28] = xx[242] + xx[30];
  xx[237] = xx[155];
  xx[238] = xx[161];
  xx[239] = xx[28];
  pm_math_Matrix3x3_xform_ra(xx + 228, xx + 237, xx + 240);
  xx[29] = xx[49] + xx[74] + xx[71] + xx[184] + xx[207] + xx[210] + xx[240];
  xx[243] = xx[12];
  xx[244] = - xx[26];
  xx[245] = - xx[27];
  pm_math_Matrix3x3_xform_ra(xx + 255, xx + 243, xx + 264);
  xx[255] = xx[21];
  xx[256] = xx[25];
  xx[257] = xx[36];
  pm_math_Matrix3x3_xform_ra(xx + 228, xx + 255, xx + 258);
  xx[30] = xx[264] + xx[258];
  xx[162] = state[4] + 0.0449217015611241;
  if (xx[0] < xx[162])
    xx[162] = xx[0];
  xx[164] = - (xx[162] / xx[2]);
  if (xx[3] < xx[164])
    xx[164] = xx[3];
  xx[166] = xx[7] * state[5];
  xx[179] = xx[164] * xx[164] * (xx[5] - xx[6] * xx[164]) * ((- xx[162] == xx[0]
    ? xx[0] : - xx[166]) - xx[77] * xx[162]);
  if (xx[0] > xx[179])
    xx[179] = xx[0];
  xx[162] = 1.133175543535048;
  xx[164] = state[4] - xx[162];
  if (xx[0] > xx[164])
    xx[164] = xx[0];
  xx[180] = xx[164] / xx[2];
  if (xx[3] < xx[180])
    xx[180] = xx[3];
  xx[187] = (xx[77] * xx[164] + (xx[164] == xx[0] ? xx[0] : xx[166])) * xx[180] *
    xx[180] * (xx[5] - xx[6] * xx[180]);
  if (xx[0] > xx[187])
    xx[187] = xx[0];
  xx[164] = xx[50] + xx[75] + xx[72] + xx[185] + xx[208] + xx[211] + xx[241];
  xx[49] = xx[51] + xx[76] + xx[73] + xx[186] + xx[209] + xx[212] + xx[242];
  xx[71] = xx[29];
  xx[72] = xx[164];
  xx[73] = xx[49];
  pm_math_Matrix3x3_transposeXform_ra(xx + 228, xx + 220, xx + 74);
  xx[50] = 61.68856232;
  xx[51] = xx[50] + xx[291] + xx[246];
  xx[166] = xx[292] + xx[247];
  xx[180] = xx[293] + xx[248];
  xx[184] = xx[294] + xx[249];
  xx[185] = xx[50] + xx[295] + xx[250];
  xx[186] = xx[296] + xx[251];
  xx[200] = xx[297] + xx[252];
  xx[201] = xx[298] + xx[253];
  xx[207] = xx[50] + xx[299] + xx[254];
  xx[246] = xx[51];
  xx[247] = xx[166];
  xx[248] = xx[180];
  xx[249] = xx[184];
  xx[250] = xx[185];
  xx[251] = xx[186];
  xx[252] = xx[200];
  xx[253] = xx[201];
  xx[254] = xx[207];
  pm_math_Matrix3x3_xform_ra(xx + 246, xx + 237, xx + 208);
  xx[50] = xx[78] + xx[203] + xx[74] + xx[208];
  xx[78] = xx[79] + xx[204] + xx[75] + xx[209];
  xx[74] = xx[80] + xx[205] + xx[76] + xx[210];
  xx[203] = xx[50];
  xx[204] = xx[78];
  xx[205] = xx[74];
  xx[75] = xx[265] + xx[259];
  xx[76] = xx[266] + xx[260];
  xx[208] = xx[30];
  xx[209] = xx[75];
  xx[210] = xx[76];
  pm_math_Matrix3x3_transposeXform_ra(xx + 228, xx + 243, xx + 237);
  pm_math_Matrix3x3_xform_ra(xx + 246, xx + 255, xx + 228);
  xx[79] = xx[237] + xx[228];
  xx[80] = xx[238] + xx[229];
  xx[211] = xx[239] + xx[230];
  xx[228] = xx[79];
  xx[229] = xx[80];
  xx[230] = xx[211];
  xx[212] = pm_math_Vector3_dot_ra(xx + 243, xx + 208) + pm_math_Vector3_dot_ra
    (xx + 255, xx + 228);
  ii[0] = factorSymmetricPosDef(xx + 212, 1, xx + 208);
  if (ii[0] != 0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:DegenerateMass",
      "'human9DOF/HipsForward' has a degenerate mass distribution on its follower side.",
      neDiagMgr);
  }

  xx[208] = (xx[179] - (xx[1] * state[4] + xx[4] * state[5]) - xx[187] -
             (pm_math_Vector3_dot_ra(xx + 243, xx + 71) + pm_math_Vector3_dot_ra
              (xx + 255, xx + 203))) / xx[212];
  xx[71] = xx[29] + xx[30] * xx[208];
  xx[72] = xx[164] + xx[75] * xx[208];
  xx[73] = xx[49] + xx[76] * xx[208];
  pm_math_Quaternion_xform_ra(xx + 32, xx + 71, xx + 203);
  xx[71] = xx[50] + xx[79] * xx[208];
  xx[72] = xx[78] + xx[80] * xx[208];
  xx[73] = xx[74] + xx[211] * xx[208];
  pm_math_Quaternion_xform_ra(xx + 32, xx + 71, xx + 228);
  pm_math_Vector3_cross_ra(xx + 37, xx + 228, xx + 71);
  xx[29] = xx[6] * xx[23] * state[3];
  xx[23] = xx[32] * xx[32];
  xx[49] = xx[33] * xx[34];
  xx[50] = xx[32] * xx[35];
  xx[74] = xx[33] * xx[35];
  xx[78] = xx[32] * xx[34];
  xx[164] = xx[34] * xx[35];
  xx[179] = xx[32] * xx[33];
  xx[228] = (xx[23] + xx[33] * xx[33]) * xx[6] - xx[3];
  xx[229] = xx[6] * (xx[49] - xx[50]);
  xx[230] = (xx[74] + xx[78]) * xx[6];
  xx[231] = (xx[49] + xx[50]) * xx[6];
  xx[232] = (xx[23] + xx[34] * xx[34]) * xx[6] - xx[3];
  xx[233] = xx[6] * (xx[164] - xx[179]);
  xx[234] = xx[6] * (xx[74] - xx[78]);
  xx[235] = (xx[164] + xx[179]) * xx[6];
  xx[236] = (xx[23] + xx[35] * xx[35]) * xx[6] - xx[3];
  xx[23] = xx[30] / xx[212];
  xx[49] = xx[75] * xx[23];
  xx[50] = xx[76] * xx[23];
  xx[74] = xx[75] / xx[212];
  xx[78] = xx[76] * xx[74];
  xx[164] = xx[76] / xx[212];
  xx[237] = xx[67] - xx[30] * xx[23];
  xx[238] = xx[43] - xx[49];
  xx[239] = xx[81] - xx[50];
  xx[240] = xx[82] - xx[49];
  xx[241] = xx[83] - xx[75] * xx[74];
  xx[242] = xx[44] - xx[78];
  xx[243] = xx[97] - xx[50];
  xx[244] = xx[105] - xx[78];
  xx[245] = xx[111] - xx[76] * xx[164];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 237, xx + 228, xx + 246);
  pm_math_Matrix3x3_compose_ra(xx + 228, xx + 246, xx + 237);
  xx[43] = xx[79] / xx[212];
  xx[44] = xx[80] / xx[212];
  xx[49] = xx[211] / xx[212];
  xx[246] = xx[11] - xx[30] * xx[43];
  xx[247] = xx[24] - xx[30] * xx[44];
  xx[248] = xx[31] - xx[30] * xx[49];
  xx[249] = xx[45] - xx[75] * xx[43];
  xx[250] = xx[123] - xx[75] * xx[44];
  xx[251] = xx[126] - xx[75] * xx[49];
  xx[252] = xx[128] - xx[76] * xx[43];
  xx[253] = xx[132] - xx[76] * xx[44];
  xx[254] = xx[152] - xx[76] * xx[49];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 246, xx + 228, xx + 255);
  pm_math_Matrix3x3_compose_ra(xx + 228, xx + 255, xx + 246);
  pm_math_Matrix3x3_postCross_ra(xx + 246, xx + 37, xx + 255);
  xx[11] = xx[80] * xx[43];
  xx[24] = xx[211] * xx[43];
  xx[30] = xx[211] * xx[44];
  xx[246] = xx[51] - xx[79] * xx[43];
  xx[247] = xx[166] - xx[11];
  xx[248] = xx[180] - xx[24];
  xx[249] = xx[184] - xx[11];
  xx[250] = xx[185] - xx[80] * xx[44];
  xx[251] = xx[186] - xx[30];
  xx[252] = xx[200] - xx[24];
  xx[253] = xx[201] - xx[30];
  xx[254] = xx[207] - xx[211] * xx[49];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 246, xx + 228, xx + 264);
  pm_math_Matrix3x3_compose_ra(xx + 228, xx + 264, xx + 246);
  pm_math_Matrix3x3_postCross_ra(xx + 246, xx + 37, xx + 228);
  pm_math_Matrix3x3_preCross_ra(xx + 228, xx + 37, xx + 246);
  xx[11] = xx[237] - xx[255] - xx[255] - xx[246];
  xx[24] = xx[6] * xx[22] * state[3];
  xx[22] = xx[239] - xx[257] - xx[261] - xx[248];
  xx[30] = state[2] + 0.4304329798196265;
  if (xx[0] < xx[30])
    xx[30] = xx[0];
  xx[31] = - (xx[30] / xx[2]);
  if (xx[3] < xx[31])
    xx[31] = xx[3];
  xx[45] = xx[7] * state[3];
  xx[7] = xx[31] * xx[31] * (xx[5] - xx[6] * xx[31]) * ((- xx[30] == xx[0] ? xx
    [0] : - xx[45]) - xx[77] * xx[30]);
  if (xx[0] > xx[7])
    xx[7] = xx[0];
  xx[30] = state[2] - xx[162];
  if (xx[0] > xx[30])
    xx[30] = xx[0];
  xx[31] = xx[30] / xx[2];
  if (xx[3] < xx[31])
    xx[31] = xx[3];
  xx[2] = (xx[77] * xx[30] + (xx[30] == xx[0] ? xx[0] : xx[45])) * xx[31] * xx
    [31] * (xx[5] - xx[6] * xx[31]);
  if (xx[0] > xx[2])
    xx[2] = xx[0];
  xx[5] = xx[240] - xx[258] - xx[256] - xx[249];
  xx[30] = xx[242] - xx[260] - xx[262] - xx[251];
  xx[31] = xx[204] + xx[72] - (xx[29] * xx[5] + xx[24] * xx[30]);
  xx[45] = xx[241] - xx[259] - xx[259] - xx[250];
  memcpy(xx + 50, xx + 45, 1 * sizeof(double));
  ii[0] = factorSymmetricPosDef(xx + 50, 1, xx + 51);
  if (ii[0] != 0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:DegenerateMass",
      "'human9DOF/hipside' has a degenerate mass distribution on its follower side.",
      neDiagMgr);
  }

  xx[51] = (xx[7] - (xx[1] * state[2] + 5.0 * state[3]) - xx[2] - xx[31]) / xx
    [50];
  xx[2] = xx[238] - xx[256] - xx[258] - xx[247];
  xx[7] = xx[243] - xx[261] - xx[257] - xx[252];
  xx[67] = xx[245] - xx[263] - xx[263] - xx[254];
  xx[72] = xx[244] - xx[262] - xx[260] - xx[253];
  xx[75] = xx[203] + xx[71] - (xx[29] * xx[11] + xx[24] * xx[22]) + xx[51] * xx
    [2];
  xx[76] = xx[31] + xx[51] * xx[45];
  xx[77] = xx[205] + xx[73] - (xx[29] * xx[7] + xx[24] * xx[67]) + xx[51] * xx
    [72];
  pm_math_Quaternion_xform_ra(xx + 17, xx + 75, xx + 78);
  xx[17] = xx[9] * xx[9];
  xx[18] = xx[15] * xx[14];
  xx[19] = xx[16] * xx[9];
  xx[20] = xx[16] * xx[14];
  xx[31] = xx[15] * xx[9];
  xx[71] = xx[16] * xx[15];
  xx[73] = xx[9] * xx[14];
  xx[228] = (xx[17] + xx[14] * xx[14]) * xx[6] - xx[3];
  xx[229] = xx[6] * (xx[18] - xx[19]);
  xx[230] = (xx[20] + xx[31]) * xx[6];
  xx[231] = (xx[18] + xx[19]) * xx[6];
  xx[232] = (xx[17] + xx[15] * xx[15]) * xx[6] - xx[3];
  xx[233] = xx[6] * (xx[71] - xx[73]);
  xx[234] = xx[6] * (xx[20] - xx[31]);
  xx[235] = (xx[71] + xx[73]) * xx[6];
  xx[236] = (xx[17] + xx[16] * xx[16]) * xx[6] - xx[3];
  xx[3] = xx[2] / xx[50];
  xx[17] = xx[3] * xx[45];
  xx[18] = xx[3] * xx[72];
  xx[19] = xx[45] / xx[50];
  xx[20] = xx[19] * xx[72];
  xx[31] = xx[72] / xx[50];
  xx[237] = xx[11] - xx[3] * xx[2];
  xx[238] = xx[2] - xx[17];
  xx[239] = xx[22] - xx[18];
  xx[240] = xx[5] - xx[17];
  xx[241] = xx[45] - xx[19] * xx[45];
  xx[242] = xx[30] - xx[20];
  xx[243] = xx[7] - xx[18];
  xx[244] = xx[72] - xx[20];
  xx[245] = xx[67] - xx[31] * xx[72];
  pm_math_Matrix3x3_composeTranspose_ra(xx + 237, xx + 228, xx + 246);
  pm_math_Matrix3x3_compose_ra(xx + 228, xx + 246, xx + 237);
  ii[0] = factorSymmetricPosDef(xx + 241, 1, xx + 2);
  if (ii[0] != 0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:DegenerateMass",
      "'human9DOF/spine twist' has a degenerate mass distribution on its follower side.",
      neDiagMgr);
  }

  xx[2] = (xx[10] - (xx[1] * state[0] + xx[4] * state[1]) - xx[13] + xx[79]) /
    xx[241];
  xx[71] = xx[3];
  xx[72] = xx[19];
  xx[73] = xx[31];
  xx[1] = xx[16] * xx[2];
  xx[3] = xx[2] * xx[14];
  xx[4] = (xx[1] * xx[9] + xx[15] * xx[3]) * xx[6];
  xx[5] = (xx[16] * xx[1] + xx[3] * xx[14]) * xx[6] - xx[2];
  xx[7] = xx[6] * (xx[3] * xx[9] - xx[15] * xx[1]);
  xx[9] = - xx[4];
  xx[10] = xx[5];
  xx[11] = xx[7];
  xx[1] = xx[51] - pm_math_Vector3_dot_ra(xx + 71, xx + 9);
  xx[9] = xx[23];
  xx[10] = xx[74];
  xx[11] = xx[164];
  xx[13] = - (xx[4] + xx[29]);
  xx[14] = xx[5] + xx[1];
  xx[15] = xx[7] - xx[24];
  pm_math_Quaternion_inverseXform_ra(xx + 32, xx + 13, xx + 3);
  xx[16] = xx[43];
  xx[17] = xx[44];
  xx[18] = xx[49];
  pm_math_Vector3_cross_ra(xx + 13, xx + 37, xx + 22);
  pm_math_Quaternion_inverseXform_ra(xx + 32, xx + 22, xx + 13);
  xx[6] = xx[208] - (pm_math_Vector3_dot_ra(xx + 9, xx + 3) +
                     pm_math_Vector3_dot_ra(xx + 16, xx + 13));
  xx[9] = xx[46];
  xx[10] = xx[55];
  xx[11] = xx[57];
  xx[16] = xx[3] + xx[12] * xx[6] + xx[220];
  xx[17] = xx[4] - xx[26] * xx[6] + xx[221];
  xx[18] = xx[5] - xx[27] * xx[6] + xx[222];
  pm_math_Quaternion_inverseXform_ra(xx + 84, xx + 16, xx + 3);
  xx[22] = xx[47];
  xx[23] = xx[48];
  xx[24] = xx[56];
  xx[7] = xx[13] + xx[21] * xx[6] + xx[155];
  pm_math_Vector3_cross_ra(xx + 16, xx + 192, xx + 19);
  xx[12] = xx[14] + xx[25] * xx[6] + xx[161];
  xx[13] = xx[15] + xx[36] * xx[6] + xx[28];
  xx[25] = xx[7] + xx[19];
  xx[26] = xx[12] + xx[20];
  xx[27] = xx[13] + xx[21];
  pm_math_Quaternion_inverseXform_ra(xx + 84, xx + 25, xx + 19);
  xx[14] = xx[147] - (pm_math_Vector3_dot_ra(xx + 9, xx + 3) +
                      pm_math_Vector3_dot_ra(xx + 22, xx + 19));
  xx[9] = - xx[60];
  xx[10] = - xx[91];
  xx[11] = - xx[151];
  xx[22] = xx[3] + xx[14];
  xx[23] = xx[4] + xx[66];
  xx[24] = xx[5] - xx[89];
  pm_math_Quaternion_inverseXform_ra(xx + 92, xx + 22, xx + 3);
  xx[22] = - xx[98];
  xx[23] = - xx[122];
  xx[24] = - xx[127];
  xx[25] = xx[19] + xx[40];
  xx[26] = xx[20] + xx[41];
  xx[27] = xx[21] + xx[42];
  pm_math_Quaternion_inverseXform_ra(xx + 92, xx + 25, xx + 19);
  xx[15] = xx[136] - (pm_math_Vector3_dot_ra(xx + 9, xx + 3) +
                      pm_math_Vector3_dot_ra(xx + 22, xx + 19));
  xx[9] = xx[113];
  xx[10] = xx[191];
  xx[11] = xx[199];
  xx[22] = xx[3] + xx[104];
  xx[23] = xx[4] - xx[15];
  xx[24] = xx[5] - xx[99];
  pm_math_Quaternion_inverseXform_ra(xx + 107, xx + 22, xx + 3);
  pm_math_Vector3_cross_ra(xx + 22, xx + 114, xx + 25);
  xx[22] = xx[19] + xx[25];
  xx[23] = xx[20] + xx[26];
  xx[24] = xx[21] + xx[27];
  pm_math_Quaternion_inverseXform_ra(xx + 107, xx + 22, xx + 19);
  xx[22] = xx[206] - (pm_math_Vector3_dot_ra(xx + 9, xx + 3) +
                      pm_math_Vector3_dot_ra(xx + 119, xx + 19));
  xx[9] = xx[118];
  xx[10] = xx[153];
  xx[11] = xx[189];
  xx[23] = xx[3] + xx[88] * xx[22] + xx[196];
  xx[24] = xx[4] + xx[101] * xx[22] + xx[197];
  xx[25] = xx[5] - xx[102] * xx[22] + xx[198];
  pm_math_Quaternion_inverseXform_ra(xx + 139, xx + 23, xx + 3);
  xx[26] = xx[137];
  xx[27] = xx[138];
  xx[28] = xx[146];
  pm_math_Vector3_cross_ra(xx + 23, xx + 217, xx + 29);
  xx[23] = xx[19] - xx[96] * xx[22] + xx[188] + xx[29];
  xx[24] = xx[20] - xx[100] * xx[22] + xx[190] + xx[30];
  xx[25] = xx[21] - xx[112] * xx[22] + xx[103] + xx[31];
  pm_math_Quaternion_inverseXform_ra(xx + 139, xx + 23, xx + 19);
  xx[23] = xx[195] - (pm_math_Vector3_dot_ra(xx + 9, xx + 3) +
                      pm_math_Vector3_dot_ra(xx + 26, xx + 19));
  xx[9] = - xx[148];
  xx[10] = - xx[156];
  xx[11] = - xx[181];
  xx[24] = xx[3] + xx[124] * xx[23] + xx[225];
  xx[25] = xx[4] - xx[133] * xx[23] + xx[226];
  xx[26] = xx[5] - xx[134] * xx[23] + xx[227];
  pm_math_Quaternion_inverseXform_ra(xx + 157, xx + 24, xx + 3);
  xx[27] = - xx[8];
  xx[28] = - xx[150];
  xx[29] = - xx[154];
  pm_math_Vector3_cross_ra(xx + 24, xx + 213, xx + 30);
  xx[24] = xx[19] + xx[125] * xx[23] + xx[131] + xx[30];
  xx[25] = xx[20] + xx[129] * xx[23] + xx[135] + xx[31];
  xx[26] = xx[21] - xx[130] * xx[23] + xx[117] + xx[32];
  pm_math_Quaternion_inverseXform_ra(xx + 157, xx + 24, xx + 19);
  xx[8] = xx[202] - (pm_math_Vector3_dot_ra(xx + 9, xx + 3) +
                     pm_math_Vector3_dot_ra(xx + 27, xx + 19));
  xx[9] = - xx[163];
  xx[10] = - xx[182];
  xx[11] = xx[167] / xx[169];
  xx[24] = xx[3] + xx[165];
  xx[25] = xx[4] - xx[8];
  xx[26] = xx[5] - xx[149];
  pm_math_Quaternion_inverseXform_ra(xx + 172, xx + 24, xx + 3);
  xx[27] = - xx[168];
  xx[28] = - xx[171];
  xx[29] = - xx[183];
  pm_math_Vector3_cross_ra(xx + 24, xx + 176, xx + 30);
  xx[24] = xx[19] + xx[143] + xx[30];
  xx[25] = xx[20] + xx[144] + xx[31];
  xx[26] = xx[21] + xx[145] + xx[32];
  pm_math_Quaternion_inverseXform_ra(xx + 172, xx + 24, xx + 19);
  xx[24] = xx[90];
  xx[25] = - xx[106];
  xx[26] = - (xx[68] / xx[58]);
  pm_math_Quaternion_inverseXform_ra(xx + 62, xx + 16, xx + 30);
  xx[33] = - xx[61];
  xx[34] = - xx[69];
  xx[35] = xx[70];
  pm_math_Vector3_cross_ra(xx + 16, xx + 52, xx + 36);
  xx[16] = xx[7] + xx[36];
  xx[17] = xx[12] + xx[37];
  xx[18] = xx[13] + xx[38];
  pm_math_Quaternion_inverseXform_ra(xx + 62, xx + 16, xx + 36);
  deriv[0] = state[1];
  deriv[1] = xx[2];
  deriv[2] = state[3];
  deriv[3] = xx[1];
  deriv[4] = state[5];
  deriv[5] = xx[6];
  deriv[6] = state[7];
  deriv[7] = xx[14];
  deriv[8] = state[9];
  deriv[9] = xx[15];
  deriv[10] = state[11];
  deriv[11] = xx[22];
  deriv[12] = state[13];
  deriv[13] = xx[23];
  deriv[14] = state[15];
  deriv[15] = xx[8];
  deriv[16] = state[17];
  deriv[17] = xx[170] - (pm_math_Vector3_dot_ra(xx + 9, xx + 3) +
    pm_math_Vector3_dot_ra(xx + 27, xx + 19));
  deriv[18] = state[19];
  deriv[19] = xx[59] - (pm_math_Vector3_dot_ra(xx + 24, xx + 30) +
                        pm_math_Vector3_dot_ra(xx + 33, xx + 36));
  errorResult[0] = xx[0];
  return NULL;
}

PmfMessageId human9DOF_836bb176_1_numJacPerturbLoBounds(const
  RuntimeDerivedValuesBundle *rtdv, const int *eqnEnableFlags, const double
  *state, const int *modeVector, const double *input, const double *inputDot,
  const double *inputDdot, const double *discreteState, double *bounds, double
  *errorResult, NeuDiagnosticManager *neDiagMgr)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[1];
  (void) rtdvd;
  (void) rtdvi;
  (void) eqnEnableFlags;
  (void) state;
  (void) modeVector;
  (void) input;
  (void) inputDot;
  (void) inputDdot;
  (void) discreteState;
  (void) neDiagMgr;
  xx[0] = 1.0e-8;
  bounds[0] = xx[0];
  bounds[1] = xx[0];
  bounds[2] = xx[0];
  bounds[3] = xx[0];
  bounds[4] = xx[0];
  bounds[5] = xx[0];
  bounds[6] = xx[0];
  bounds[7] = xx[0];
  bounds[8] = xx[0];
  bounds[9] = xx[0];
  bounds[10] = xx[0];
  bounds[11] = xx[0];
  bounds[12] = xx[0];
  bounds[13] = xx[0];
  bounds[14] = xx[0];
  bounds[15] = xx[0];
  bounds[16] = xx[0];
  bounds[17] = xx[0];
  bounds[18] = xx[0];
  bounds[19] = xx[0];
  errorResult[0] = 0.0;
  return NULL;
}

PmfMessageId human9DOF_836bb176_1_numJacPerturbHiBounds(const
  RuntimeDerivedValuesBundle *rtdv, const int *eqnEnableFlags, const double
  *state, const int *modeVector, const double *input, const double *inputDot,
  const double *inputDdot, const double *discreteState, double *bounds, double
  *errorResult, NeuDiagnosticManager *neDiagMgr)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[2];
  (void) rtdvd;
  (void) rtdvi;
  (void) eqnEnableFlags;
  (void) state;
  (void) modeVector;
  (void) input;
  (void) inputDot;
  (void) inputDdot;
  (void) discreteState;
  (void) neDiagMgr;
  xx[0] = 1.0;
  xx[1] = +pmf_get_inf();
  bounds[0] = xx[0];
  bounds[1] = xx[1];
  bounds[2] = xx[0];
  bounds[3] = xx[1];
  bounds[4] = xx[0];
  bounds[5] = xx[1];
  bounds[6] = xx[0];
  bounds[7] = xx[1];
  bounds[8] = xx[0];
  bounds[9] = xx[1];
  bounds[10] = xx[0];
  bounds[11] = xx[1];
  bounds[12] = xx[0];
  bounds[13] = xx[1];
  bounds[14] = xx[0];
  bounds[15] = xx[1];
  bounds[16] = xx[0];
  bounds[17] = xx[1];
  bounds[18] = xx[0];
  bounds[19] = xx[1];
  errorResult[0] = 0.0;
  return NULL;
}
