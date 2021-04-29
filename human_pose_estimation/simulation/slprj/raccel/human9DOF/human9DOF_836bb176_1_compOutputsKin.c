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

PmfMessageId human9DOF_836bb176_1_compOutputsKin(const
  RuntimeDerivedValuesBundle *rtdv, const double *state, const int *modeVector,
  const double *input, const double *inputDot, const double *inputDdot, const
  double *discreteState, double *output, NeuDiagnosticManager *neDiagMgr)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[59];
  (void) rtdvi;
  (void) modeVector;
  (void) input;
  (void) inputDot;
  (void) inputDdot;
  (void) discreteState;
  (void) neDiagMgr;
  xx[0] = 0.5;
  xx[1] = xx[0] * state[0];
  xx[2] = cos(xx[1]);
  xx[3] = sin(xx[1]);
  xx[4] = xx[2] * rtdvd[2] + xx[3] * rtdvd[4];
  xx[5] = xx[2] * rtdvd[3] + xx[3] * rtdvd[5];
  xx[6] = xx[2] * rtdvd[4] - xx[3] * rtdvd[2];
  xx[7] = xx[2] * rtdvd[5] - xx[3] * rtdvd[3];
  xx[1] = xx[0] * state[2];
  xx[2] = cos(xx[1]);
  xx[3] = sin(xx[1]);
  xx[8] = xx[2] * rtdvd[10] - xx[3] * rtdvd[12];
  xx[9] = xx[2] * rtdvd[11] - xx[3] * rtdvd[13];
  xx[10] = xx[3] * rtdvd[10] + xx[2] * rtdvd[12];
  xx[11] = xx[2] * rtdvd[13] + xx[3] * rtdvd[11];
  pm_math_Quaternion_compose_ra(xx + 4, xx + 8, xx + 12);
  xx[1] = rtdvd[18];
  xx[2] = rtdvd[19];
  xx[3] = rtdvd[20];
  xx[4] = rtdvd[21];
  xx[5] = xx[0] * state[4];
  xx[6] = sin(xx[5]);
  xx[7] = cos(xx[5]);
  xx[8] = 0.9999999147210419 * xx[6];
  xx[9] = - (4.077338750123304e-4 * xx[6]);
  xx[10] = - (6.565817943715579e-5 * xx[6]);
  pm_math_Quaternion_compose_ra(xx + 1, xx + 7, xx + 16);
  pm_math_Quaternion_compose_ra(xx + 12, xx + 16, xx + 1);
  xx[5] = xx[0] * state[6];
  xx[6] = cos(xx[5]);
  xx[7] = sin(xx[5]);
  xx[8] = xx[6] * rtdvd[30] - xx[7] * rtdvd[31];
  xx[9] = xx[7] * rtdvd[30] + xx[6] * rtdvd[31];
  xx[10] = xx[6] * rtdvd[32] + xx[7] * rtdvd[33];
  xx[11] = xx[6] * rtdvd[33] - xx[7] * rtdvd[32];
  pm_math_Quaternion_compose_ra(xx + 1, xx + 8, xx + 20);
  xx[5] = xx[0] * state[8];
  xx[6] = cos(xx[5]);
  xx[7] = sin(xx[5]);
  xx[8] = xx[6] * rtdvd[44] + xx[7] * rtdvd[46];
  xx[9] = xx[6] * rtdvd[45] + xx[7] * rtdvd[47];
  xx[10] = xx[6] * rtdvd[46] - xx[7] * rtdvd[44];
  xx[11] = xx[6] * rtdvd[47] - xx[7] * rtdvd[45];
  pm_math_Quaternion_compose_ra(xx + 20, xx + 8, xx + 24);
  xx[5] = rtdvd[55];
  xx[6] = rtdvd[56];
  xx[7] = rtdvd[57];
  xx[8] = rtdvd[58];
  xx[9] = xx[0] * state[10];
  xx[10] = sin(xx[9]);
  xx[20] = cos(xx[9]);
  xx[21] = 0.1935817812961164 * xx[10];
  xx[22] = 0.1665657035417852 * xx[10];
  xx[23] = - (0.966841228099969 * xx[10]);
  pm_math_Quaternion_compose_ra(xx + 5, xx + 20, xx + 28);
  pm_math_Quaternion_compose_ra(xx + 24, xx + 28, xx + 5);
  xx[20] = rtdvd[68];
  xx[21] = rtdvd[69];
  xx[22] = rtdvd[70];
  xx[23] = rtdvd[71];
  xx[9] = xx[0] * state[12];
  xx[10] = sin(xx[9]);
  xx[32] = cos(xx[9]);
  xx[33] = 0.7842943984275436 * xx[10];
  xx[34] = - (0.6015976293979709 * xx[10]);
  xx[35] = - (0.1515341179336174 * xx[10]);
  pm_math_Quaternion_compose_ra(xx + 20, xx + 32, xx + 36);
  pm_math_Quaternion_compose_ra(xx + 5, xx + 36, xx + 20);
  xx[9] = xx[0] * state[14];
  xx[10] = cos(xx[9]);
  xx[11] = sin(xx[9]);
  xx[32] = xx[10] * rtdvd[87] + xx[11] * rtdvd[89];
  xx[33] = xx[10] * rtdvd[88] + xx[11] * rtdvd[90];
  xx[34] = xx[10] * rtdvd[89] - xx[11] * rtdvd[87];
  xx[35] = xx[10] * rtdvd[90] - xx[11] * rtdvd[88];
  pm_math_Quaternion_compose_ra(xx + 20, xx + 32, xx + 40);
  xx[32] = - 0.979885074593562;
  xx[33] = 0.1454472155865731;
  xx[34] = 2.824882685237451e-3;
  xx[35] = - 0.1366102781810241;
  xx[9] = xx[0] * state[16];
  xx[0] = sin(xx[9]);
  xx[44] = cos(xx[9]);
  xx[45] = - (0.03420304840239036 * xx[0]);
  xx[46] = - (0.2858149274078682 * xx[0]);
  xx[47] = 0.9576742550318551 * xx[0];
  pm_math_Quaternion_compose_ra(xx + 32, xx + 44, xx + 48);
  pm_math_Quaternion_compose_ra(xx + 40, xx + 48, xx + 32);
  xx[44] = 0.9337939117424221;
  xx[45] = 0.1420427034611693;
  xx[46] = 0.03141246372882511;
  xx[47] = - 0.3269037441032348;
  pm_math_Quaternion_compose_ra(xx + 32, xx + 44, xx + 52);
  xx[0] = sqrt(xx[53] * xx[53] + xx[54] * xx[54] + xx[55] * xx[55]);
  xx[0] = xx[0] == 0.0 ? 0.0 : 1.0 / xx[0];
  xx[9] = ((- xx[52]) < 0.0 ? -1.0 : +1.0) * xx[0];
  xx[44] = 0.03984215544011924;
  xx[45] = - 0.03056115957341688;
  xx[46] = - 7.697933191027721e-3;
  pm_math_Quaternion_xform_ra(xx + 32, xx + 44, xx + 56);
  pm_math_Quaternion_xform_ra(xx + 48, xx + 44, xx + 32);
  xx[44] = - xx[32];
  xx[45] = - xx[33];
  xx[46] = - xx[34];
  pm_math_Quaternion_xform_ra(xx + 40, xx + 44, xx + 32);
  xx[40] = - 0.08481305455953883;
  xx[41] = - 0.07715695137174808;
  xx[42] = 4.397388070152032e-4;
  pm_math_Quaternion_xform_ra(xx + 20, xx + 40, xx + 43);
  xx[20] = 0.1174480511045049;
  xx[21] = 0.09974111499921998;
  xx[22] = 0.04427933683814569;
  pm_math_Quaternion_xform_ra(xx + 36, xx + 20, xx + 40);
  xx[20] = 0.0475545670786721 - xx[40];
  xx[21] = - (0.1051997654804633 + xx[41]);
  xx[22] = - (0.01172847704075845 + xx[42]);
  pm_math_Quaternion_xform_ra(xx + 5, xx + 20, xx + 35);
  xx[5] = - 0.0786896255494641;
  xx[6] = 0.1231236283902652;
  xx[7] = 0.02515955587646605;
  pm_math_Quaternion_xform_ra(xx + 28, xx + 5, xx + 20);
  xx[5] = - xx[20];
  xx[6] = - xx[21];
  xx[7] = - xx[22];
  pm_math_Quaternion_xform_ra(xx + 24, xx + 5, xx + 20);
  xx[5] = - 0.2244123868924967;
  xx[6] = 9.872944805650204e-3;
  xx[7] = 1.177299620836221e-3;
  pm_math_Quaternion_xform_ra(xx + 1, xx + 5, xx + 23);
  xx[0] = - 0.02644395617389824;
  xx[1] = - 0.3675397405234455;
  xx[2] = 0.03617328556917723;
  pm_math_Quaternion_xform_ra(xx + 16, xx + 0, xx + 3);
  xx[0] = - xx[3];
  xx[1] = - xx[4];
  xx[2] = - xx[5];
  pm_math_Quaternion_xform_ra(xx + 12, xx + 0, xx + 3);
  xx[0] = 0.0;
  output[0] = state[6];
  output[1] = state[7];
  output[2] = state[4];
  output[3] = state[5];
  output[4] = state[8];
  output[5] = state[9];
  output[6] = state[10];
  output[7] = state[11];
  output[8] = state[2];
  output[9] = state[3];
  output[10] = state[12];
  output[11] = state[13];
  output[12] = state[14];
  output[13] = state[15];
  output[14] = state[16];
  output[15] = state[17];
  output[16] = state[0];
  output[17] = state[1];
  output[18] = - (xx[53] * xx[9]);
  output[19] = - (xx[54] * xx[9]);
  output[20] = - (xx[55] * xx[9]);
  output[21] = xx[56] + xx[32] + xx[43] + xx[35] + xx[20] + xx[23] + xx[3];
  output[22] = xx[57] + xx[33] + xx[44] + xx[36] + xx[21] + xx[24] + xx[4];
  output[23] = xx[58] + xx[34] + xx[45] + xx[37] + xx[22] + xx[25] + xx[5];
  return NULL;
}
