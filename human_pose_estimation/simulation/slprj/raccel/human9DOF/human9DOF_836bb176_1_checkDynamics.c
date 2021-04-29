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

PmfMessageId human9DOF_836bb176_1_checkDynamics(const RuntimeDerivedValuesBundle
  *rtdv, const double *state, const double *input, const double *inputDot, const
  double *inputDdot, const double *discreteState, double *result,
  NeuDiagnosticManager *neDiagMgr)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[23];
  (void) rtdvi;
  (void) inputDot;
  (void) inputDdot;
  (void) discreteState;
  (void) neDiagMgr;
  xx[0] = - 7.359574475229558e-3;
  xx[1] = - 0.0873999301960896;
  xx[2] = - 0.02634708456854787;
  xx[3] = 0.5;
  xx[4] = xx[3] * state[0];
  xx[5] = cos(xx[4]);
  xx[6] = sin(xx[4]);
  xx[7] = xx[5] * rtdvd[2] + xx[6] * rtdvd[4];
  xx[8] = xx[5] * rtdvd[3] + xx[6] * rtdvd[5];
  xx[9] = xx[5] * rtdvd[4] - xx[6] * rtdvd[2];
  xx[10] = xx[5] * rtdvd[5] - xx[6] * rtdvd[3];
  xx[4] = xx[3] * state[2];
  xx[5] = cos(xx[4]);
  xx[6] = sin(xx[4]);
  xx[11] = xx[5] * rtdvd[10] - xx[6] * rtdvd[12];
  xx[12] = xx[5] * rtdvd[11] - xx[6] * rtdvd[13];
  xx[13] = xx[6] * rtdvd[10] + xx[5] * rtdvd[12];
  xx[14] = xx[5] * rtdvd[13] + xx[6] * rtdvd[11];
  pm_math_Quaternion_compose_ra(xx + 7, xx + 11, xx + 15);
  xx[4] = rtdvd[18];
  xx[5] = rtdvd[19];
  xx[6] = rtdvd[20];
  xx[7] = rtdvd[21];
  xx[8] = xx[3] * state[4];
  xx[9] = sin(xx[8]);
  xx[10] = cos(xx[8]);
  xx[11] = 0.9999999147210419 * xx[9];
  xx[12] = - (4.077338750123304e-4 * xx[9]);
  xx[13] = - (6.565817943715579e-5 * xx[9]);
  pm_math_Quaternion_compose_ra(xx + 4, xx + 10, xx + 19);
  pm_math_Quaternion_compose_ra(xx + 15, xx + 19, xx + 4);
  xx[8] = xx[3] * state[6];
  xx[9] = cos(xx[8]);
  xx[10] = sin(xx[8]);
  xx[11] = xx[9] * rtdvd[30] - xx[10] * rtdvd[31];
  xx[12] = xx[10] * rtdvd[30] + xx[9] * rtdvd[31];
  xx[13] = xx[9] * rtdvd[32] + xx[10] * rtdvd[33];
  xx[14] = xx[9] * rtdvd[33] - xx[10] * rtdvd[32];
  pm_math_Quaternion_compose_ra(xx + 4, xx + 11, xx + 15);
  xx[4] = xx[3] * state[8];
  xx[5] = cos(xx[4]);
  xx[6] = sin(xx[4]);
  xx[7] = xx[5] * rtdvd[44] + xx[6] * rtdvd[46];
  xx[8] = xx[5] * rtdvd[45] + xx[6] * rtdvd[47];
  xx[9] = xx[5] * rtdvd[46] - xx[6] * rtdvd[44];
  xx[10] = xx[5] * rtdvd[47] - xx[6] * rtdvd[45];
  pm_math_Quaternion_compose_ra(xx + 15, xx + 7, xx + 11);
  xx[4] = rtdvd[55];
  xx[5] = rtdvd[56];
  xx[6] = rtdvd[57];
  xx[7] = rtdvd[58];
  xx[8] = xx[3] * state[10];
  xx[9] = sin(xx[8]);
  xx[15] = cos(xx[8]);
  xx[16] = 0.1935817812961164 * xx[9];
  xx[17] = 0.1665657035417852 * xx[9];
  xx[18] = - (0.966841228099969 * xx[9]);
  pm_math_Quaternion_compose_ra(xx + 4, xx + 15, xx + 19);
  pm_math_Quaternion_compose_ra(xx + 11, xx + 19, xx + 4);
  xx[8] = rtdvd[68];
  xx[9] = rtdvd[69];
  xx[10] = rtdvd[70];
  xx[11] = rtdvd[71];
  xx[12] = xx[3] * state[12];
  xx[13] = sin(xx[12]);
  xx[14] = cos(xx[12]);
  xx[15] = 0.7842943984275436 * xx[13];
  xx[16] = - (0.6015976293979709 * xx[13]);
  xx[17] = - (0.1515341179336174 * xx[13]);
  pm_math_Quaternion_compose_ra(xx + 8, xx + 14, xx + 18);
  pm_math_Quaternion_compose_ra(xx + 4, xx + 18, xx + 8);
  xx[4] = xx[3] * state[14];
  xx[5] = cos(xx[4]);
  xx[6] = sin(xx[4]);
  xx[12] = xx[5] * rtdvd[87] + xx[6] * rtdvd[89];
  xx[13] = xx[5] * rtdvd[88] + xx[6] * rtdvd[90];
  xx[14] = xx[5] * rtdvd[89] - xx[6] * rtdvd[87];
  xx[15] = xx[5] * rtdvd[90] - xx[6] * rtdvd[88];
  pm_math_Quaternion_compose_ra(xx + 8, xx + 12, xx + 4);
  xx[8] = - 0.979885074593562;
  xx[9] = 0.1454472155865731;
  xx[10] = 2.824882685237451e-3;
  xx[11] = - 0.1366102781810241;
  xx[12] = xx[3] * state[16];
  xx[3] = sin(xx[12]);
  xx[13] = cos(xx[12]);
  xx[14] = - (0.03420304840239036 * xx[3]);
  xx[15] = - (0.2858149274078682 * xx[3]);
  xx[16] = 0.9576742550318551 * xx[3];
  pm_math_Quaternion_compose_ra(xx + 8, xx + 13, xx + 17);
  pm_math_Quaternion_compose_ra(xx + 4, xx + 17, xx + 8);
  xx[3] = input[0];
  xx[4] = input[1];
  xx[5] = input[2];
  pm_math_Quaternion_inverseXform_ra(xx + 8, xx + 3, xx + 12);
  pm_math_Vector3_cross_ra(xx + 0, xx + 12, xx + 3);
  result[0] = xx[3] * xx[3] + xx[4] * xx[4] + xx[5] * xx[5] + xx[12] * xx[12] +
    xx[13] * xx[13] + xx[14] * xx[14];
  return NULL;
}
