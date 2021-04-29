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
#include "sm_CTarget.h"

static void setTargets_9(const RuntimeDerivedValuesBundle *rtdv, double *values,
  double *auxData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvi;
  (void) auxData;
  values[0] = rtdvd[97];
}

static void setTargets_11(const RuntimeDerivedValuesBundle *rtdv, double *values,
  double *auxData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvi;
  (void) auxData;
  values[0] = rtdvd[98];
}

static void setTargets_13(const RuntimeDerivedValuesBundle *rtdv, double *values,
  double *auxData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvi;
  (void) auxData;
  values[0] = rtdvd[99];
}

static void setTargets_15(const RuntimeDerivedValuesBundle *rtdv, double *values,
  double *auxData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvi;
  (void) auxData;
  values[0] = rtdvd[100];
}

void human9DOF_836bb176_1_setTargets(const RuntimeDerivedValuesBundle *rtdv,
  CTarget *targets)
{
  setTargets_9(rtdv, targets[9].mValue, targets[9].mAuxiliaryTargetData);
  setTargets_11(rtdv, targets[11].mValue, targets[11].mAuxiliaryTargetData);
  setTargets_13(rtdv, targets[13].mValue, targets[13].mAuxiliaryTargetData);
  setTargets_15(rtdv, targets[15].mValue, targets[15].mAuxiliaryTargetData);
}

void human9DOF_836bb176_1_resetAsmStateVector(const void *mech, double *state)
{
  double xx[1];
  (void) mech;
  xx[0] = 0.0;
  state[0] = xx[0];
  state[1] = xx[0];
  state[2] = xx[0];
  state[3] = xx[0];
  state[4] = xx[0];
  state[5] = xx[0];
  state[6] = xx[0];
  state[7] = xx[0];
  state[8] = xx[0];
  state[9] = xx[0];
  state[10] = xx[0];
  state[11] = xx[0];
  state[12] = xx[0];
  state[13] = xx[0];
  state[14] = xx[0];
  state[15] = xx[0];
  state[16] = xx[0];
  state[17] = xx[0];
  state[18] = xx[0];
  state[19] = xx[0];
}

void human9DOF_836bb176_1_initializeTrackedAngleState(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, const int *modeVector, const double
  *motionData, double *state, void *neDiagMgr0)
{
  NeuDiagnosticManager *neDiagMgr = (NeuDiagnosticManager *) neDiagMgr0;
  (void) mech;
  (void) rtdv;
  (void) modeVector;
  (void) motionData;
  (void) state;
  (void) neDiagMgr;
}

void human9DOF_836bb176_1_computeDiscreteState(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, double *state)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  (void) state;
}

void human9DOF_836bb176_1_adjustPosition(const void *mech, const double
  *dofDeltas, double *state)
{
  (void) mech;
  state[0] = state[0] + dofDeltas[0];
  state[2] = state[2] + dofDeltas[1];
  state[4] = state[4] + dofDeltas[2];
  state[6] = state[6] + dofDeltas[3];
  state[8] = state[8] + dofDeltas[4];
  state[10] = state[10] + dofDeltas[5];
  state[12] = state[12] + dofDeltas[6];
  state[14] = state[14] + dofDeltas[7];
  state[16] = state[16] + dofDeltas[8];
  state[18] = state[18] + dofDeltas[9];
}

static void perturbAsmJointPrimitiveState_0_0(double mag, double *state)
{
  state[0] = state[0] + mag;
}

static void perturbAsmJointPrimitiveState_0_0v(double mag, double *state)
{
  state[0] = state[0] + mag;
  state[1] = state[1] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_1_0(double mag, double *state)
{
  state[2] = state[2] + mag;
}

static void perturbAsmJointPrimitiveState_1_0v(double mag, double *state)
{
  state[2] = state[2] + mag;
  state[3] = state[3] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_2_0(double mag, double *state)
{
  state[4] = state[4] + mag;
}

static void perturbAsmJointPrimitiveState_2_0v(double mag, double *state)
{
  state[4] = state[4] + mag;
  state[5] = state[5] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_3_0(double mag, double *state)
{
  state[6] = state[6] + mag;
}

static void perturbAsmJointPrimitiveState_3_0v(double mag, double *state)
{
  state[6] = state[6] + mag;
  state[7] = state[7] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_4_0(double mag, double *state)
{
  state[8] = state[8] + mag;
}

static void perturbAsmJointPrimitiveState_4_0v(double mag, double *state)
{
  state[8] = state[8] + mag;
  state[9] = state[9] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_5_0(double mag, double *state)
{
  state[10] = state[10] + mag;
}

static void perturbAsmJointPrimitiveState_5_0v(double mag, double *state)
{
  state[10] = state[10] + mag;
  state[11] = state[11] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_6_0(double mag, double *state)
{
  state[12] = state[12] + mag;
}

static void perturbAsmJointPrimitiveState_6_0v(double mag, double *state)
{
  state[12] = state[12] + mag;
  state[13] = state[13] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_7_0(double mag, double *state)
{
  state[14] = state[14] + mag;
}

static void perturbAsmJointPrimitiveState_7_0v(double mag, double *state)
{
  state[14] = state[14] + mag;
  state[15] = state[15] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_8_0(double mag, double *state)
{
  state[16] = state[16] + mag;
}

static void perturbAsmJointPrimitiveState_8_0v(double mag, double *state)
{
  state[16] = state[16] + mag;
  state[17] = state[17] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_9_0(double mag, double *state)
{
  state[18] = state[18] + mag;
}

static void perturbAsmJointPrimitiveState_9_0v(double mag, double *state)
{
  state[18] = state[18] + mag;
  state[19] = state[19] - 0.875 * mag;
}

void human9DOF_836bb176_1_perturbAsmJointPrimitiveState(const void *mech, size_t
  stageIdx, size_t primIdx, double mag, boolean_T doPerturbVelocity, double
  *state)
{
  (void) mech;
  (void) stageIdx;
  (void) primIdx;
  (void) mag;
  (void) doPerturbVelocity;
  (void) state;
  switch ((stageIdx * 6 + primIdx) * 2 + (doPerturbVelocity ? 1 : 0))
  {
   case 0:
    perturbAsmJointPrimitiveState_0_0(mag, state);
    break;

   case 1:
    perturbAsmJointPrimitiveState_0_0v(mag, state);
    break;

   case 12:
    perturbAsmJointPrimitiveState_1_0(mag, state);
    break;

   case 13:
    perturbAsmJointPrimitiveState_1_0v(mag, state);
    break;

   case 24:
    perturbAsmJointPrimitiveState_2_0(mag, state);
    break;

   case 25:
    perturbAsmJointPrimitiveState_2_0v(mag, state);
    break;

   case 36:
    perturbAsmJointPrimitiveState_3_0(mag, state);
    break;

   case 37:
    perturbAsmJointPrimitiveState_3_0v(mag, state);
    break;

   case 48:
    perturbAsmJointPrimitiveState_4_0(mag, state);
    break;

   case 49:
    perturbAsmJointPrimitiveState_4_0v(mag, state);
    break;

   case 60:
    perturbAsmJointPrimitiveState_5_0(mag, state);
    break;

   case 61:
    perturbAsmJointPrimitiveState_5_0v(mag, state);
    break;

   case 72:
    perturbAsmJointPrimitiveState_6_0(mag, state);
    break;

   case 73:
    perturbAsmJointPrimitiveState_6_0v(mag, state);
    break;

   case 84:
    perturbAsmJointPrimitiveState_7_0(mag, state);
    break;

   case 85:
    perturbAsmJointPrimitiveState_7_0v(mag, state);
    break;

   case 96:
    perturbAsmJointPrimitiveState_8_0(mag, state);
    break;

   case 97:
    perturbAsmJointPrimitiveState_8_0v(mag, state);
    break;

   case 108:
    perturbAsmJointPrimitiveState_9_0(mag, state);
    break;

   case 109:
    perturbAsmJointPrimitiveState_9_0v(mag, state);
    break;
  }
}

void human9DOF_836bb176_1_computePosDofBlendMatrix(const void *mech, size_t
  stageIdx, size_t primIdx, const double *state, int partialType, double *matrix)
{
  (void) mech;
  (void) stageIdx;
  (void) primIdx;
  (void) state;
  (void) partialType;
  (void) matrix;
  switch ((stageIdx * 6 + primIdx))
  {
  }
}

void human9DOF_836bb176_1_computeVelDofBlendMatrix(const void *mech, size_t
  stageIdx, size_t primIdx, const double *state, int partialType, double *matrix)
{
  (void) mech;
  (void) stageIdx;
  (void) primIdx;
  (void) state;
  (void) partialType;
  (void) matrix;
  switch ((stageIdx * 6 + primIdx))
  {
  }
}

void human9DOF_836bb176_1_projectPartiallyTargetedPos(const void *mech, size_t
  stageIdx, size_t primIdx, const double *origState, int partialType, double
  *state)
{
  (void) mech;
  (void) stageIdx;
  (void) primIdx;
  (void) origState;
  (void) partialType;
  (void) state;
  switch ((stageIdx * 6 + primIdx))
  {
  }
}

void human9DOF_836bb176_1_propagateMotion(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, const double *state, double *motionData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[179];
  (void) mech;
  (void) rtdvi;
  xx[0] = 0.5;
  xx[1] = xx[0] * state[0];
  xx[2] = cos(xx[1]);
  xx[3] = sin(xx[1]);
  xx[1] = xx[2] * rtdvd[2] + xx[3] * rtdvd[4];
  xx[4] = xx[2] * rtdvd[3] + xx[3] * rtdvd[5];
  xx[5] = xx[2] * rtdvd[4] - xx[3] * rtdvd[2];
  xx[6] = xx[2] * rtdvd[5] - xx[3] * rtdvd[3];
  xx[2] = 0.0;
  xx[3] = xx[0] * state[2];
  xx[7] = cos(xx[3]);
  xx[8] = sin(xx[3]);
  xx[3] = xx[7] * rtdvd[10] - xx[8] * rtdvd[12];
  xx[9] = xx[7] * rtdvd[11] - xx[8] * rtdvd[13];
  xx[10] = xx[8] * rtdvd[10] + xx[7] * rtdvd[12];
  xx[11] = xx[7] * rtdvd[13] + xx[8] * rtdvd[11];
  xx[12] = rtdvd[18];
  xx[13] = rtdvd[19];
  xx[14] = rtdvd[20];
  xx[15] = rtdvd[21];
  xx[7] = xx[0] * state[4];
  xx[8] = 0.9999999147210419;
  xx[16] = sin(xx[7]);
  xx[17] = 4.077338750123304e-4;
  xx[18] = 6.565817943715579e-5;
  xx[19] = cos(xx[7]);
  xx[20] = xx[8] * xx[16];
  xx[21] = - (xx[17] * xx[16]);
  xx[22] = - (xx[18] * xx[16]);
  pm_math_Quaternion_compose_ra(xx + 12, xx + 19, xx + 23);
  xx[12] = - 0.02644395617389824;
  xx[13] = - 0.3675397405234455;
  xx[14] = 0.03617328556917723;
  pm_math_Quaternion_xform_ra(xx + 23, xx + 12, xx + 19);
  xx[7] = - xx[19];
  xx[12] = - xx[20];
  xx[13] = - xx[21];
  xx[14] = xx[0] * state[6];
  xx[15] = cos(xx[14]);
  xx[16] = sin(xx[14]);
  xx[14] = xx[15] * rtdvd[30] - xx[16] * rtdvd[31];
  xx[19] = xx[16] * rtdvd[30] + xx[15] * rtdvd[31];
  xx[20] = xx[15] * rtdvd[32] + xx[16] * rtdvd[33];
  xx[21] = xx[15] * rtdvd[33] - xx[16] * rtdvd[32];
  xx[15] = - 0.2244123868924967;
  xx[16] = 9.872944805650204e-3;
  xx[22] = 1.177299620836221e-3;
  xx[27] = xx[0] * state[8];
  xx[28] = cos(xx[27]);
  xx[29] = sin(xx[27]);
  xx[27] = xx[28] * rtdvd[44] + xx[29] * rtdvd[46];
  xx[30] = xx[28] * rtdvd[45] + xx[29] * rtdvd[47];
  xx[31] = xx[28] * rtdvd[46] - xx[29] * rtdvd[44];
  xx[32] = xx[28] * rtdvd[47] - xx[29] * rtdvd[45];
  xx[33] = rtdvd[55];
  xx[34] = rtdvd[56];
  xx[35] = rtdvd[57];
  xx[36] = rtdvd[58];
  xx[28] = xx[0] * state[10];
  xx[29] = 0.1935817812961164;
  xx[37] = sin(xx[28]);
  xx[38] = 0.1665657035417852;
  xx[39] = 0.966841228099969;
  xx[40] = cos(xx[28]);
  xx[41] = xx[29] * xx[37];
  xx[42] = xx[38] * xx[37];
  xx[43] = - (xx[39] * xx[37]);
  pm_math_Quaternion_compose_ra(xx + 33, xx + 40, xx + 44);
  xx[33] = - 0.0786896255494641;
  xx[34] = 0.1231236283902652;
  xx[35] = 0.02515955587646605;
  pm_math_Quaternion_xform_ra(xx + 44, xx + 33, xx + 40);
  xx[28] = - xx[40];
  xx[33] = - xx[41];
  xx[34] = - xx[42];
  xx[40] = rtdvd[68];
  xx[41] = rtdvd[69];
  xx[42] = rtdvd[70];
  xx[43] = rtdvd[71];
  xx[35] = xx[0] * state[12];
  xx[36] = 0.7842943984275436;
  xx[37] = sin(xx[35]);
  xx[48] = 0.6015976293979709;
  xx[49] = 0.1515341179336174;
  xx[50] = cos(xx[35]);
  xx[51] = xx[36] * xx[37];
  xx[52] = - (xx[48] * xx[37]);
  xx[53] = - (xx[49] * xx[37]);
  pm_math_Quaternion_compose_ra(xx + 40, xx + 50, xx + 54);
  xx[40] = 0.1174480511045049;
  xx[41] = 0.09974111499921998;
  xx[42] = 0.04427933683814569;
  pm_math_Quaternion_xform_ra(xx + 54, xx + 40, xx + 50);
  xx[35] = 0.0475545670786721 - xx[50];
  xx[37] = - (0.1051997654804633 + xx[51]);
  xx[40] = - (0.01172847704075845 + xx[52]);
  xx[41] = xx[0] * state[14];
  xx[42] = cos(xx[41]);
  xx[43] = sin(xx[41]);
  xx[41] = xx[42] * rtdvd[87] + xx[43] * rtdvd[89];
  xx[50] = xx[42] * rtdvd[88] + xx[43] * rtdvd[90];
  xx[51] = xx[42] * rtdvd[89] - xx[43] * rtdvd[87];
  xx[52] = xx[42] * rtdvd[90] - xx[43] * rtdvd[88];
  xx[42] = - 0.08481305455953883;
  xx[43] = - 0.07715695137174808;
  xx[53] = 4.397388070152032e-4;
  xx[58] = - 0.979885074593562;
  xx[59] = 0.1454472155865731;
  xx[60] = 2.824882685237451e-3;
  xx[61] = - 0.1366102781810241;
  xx[62] = xx[0] * state[16];
  xx[63] = 0.03420304840239036;
  xx[64] = sin(xx[62]);
  xx[65] = 0.2858149274078682;
  xx[66] = 0.9576742550318551;
  xx[67] = cos(xx[62]);
  xx[68] = - (xx[63] * xx[64]);
  xx[69] = - (xx[65] * xx[64]);
  xx[70] = xx[66] * xx[64];
  pm_math_Quaternion_compose_ra(xx + 58, xx + 67, xx + 71);
  xx[58] = 0.03984215544011924;
  xx[59] = - 0.03056115957341688;
  xx[60] = - 7.697933191027721e-3;
  pm_math_Quaternion_xform_ra(xx + 71, xx + 58, xx + 67);
  xx[58] = - xx[67];
  xx[59] = - xx[68];
  xx[60] = - xx[69];
  xx[67] = 0.997812314335627;
  xx[68] = 3.317527921544825e-3;
  xx[69] = 0.03372656826312366;
  xx[70] = - 0.05676352668626715;
  xx[61] = xx[0] * state[18];
  xx[0] = 0.1161231239188075;
  xx[62] = sin(xx[61]);
  xx[64] = 0.9922934657394255;
  xx[75] = 0.0432330653802957;
  xx[76] = cos(xx[61]);
  xx[77] = xx[0] * xx[62];
  xx[78] = - (xx[64] * xx[62]);
  xx[79] = - (xx[75] * xx[62]);
  pm_math_Quaternion_compose_ra(xx + 67, xx + 76, xx + 80);
  xx[67] = - 0.1774392005774167;
  xx[68] = - 0.0103867778636817;
  xx[69] = - 0.01083111892178359;
  pm_math_Quaternion_xform_ra(xx + 80, xx + 67, xx + 76);
  xx[61] = - (0.02621557995213412 + xx[76]);
  xx[62] = 0.1918176122920127 - xx[77];
  xx[67] = 0.04085299662695828 - xx[78];
  xx[76] = xx[1];
  xx[77] = xx[4];
  xx[78] = xx[5];
  xx[79] = xx[6];
  xx[84] = xx[3];
  xx[85] = xx[9];
  xx[86] = xx[10];
  xx[87] = xx[11];
  pm_math_Quaternion_compose_ra(xx + 76, xx + 84, xx + 88);
  pm_math_Quaternion_compose_ra(xx + 88, xx + 23, xx + 76);
  xx[68] = xx[7];
  xx[69] = xx[12];
  xx[70] = xx[13];
  pm_math_Quaternion_xform_ra(xx + 88, xx + 68, xx + 84);
  xx[92] = xx[14];
  xx[93] = xx[19];
  xx[94] = xx[20];
  xx[95] = xx[21];
  pm_math_Quaternion_compose_ra(xx + 76, xx + 92, xx + 96);
  xx[100] = xx[15];
  xx[101] = xx[16];
  xx[102] = xx[22];
  pm_math_Quaternion_xform_ra(xx + 76, xx + 100, xx + 103);
  xx[87] = xx[103] + xx[84];
  xx[106] = xx[104] + xx[85];
  xx[103] = xx[105] + xx[86];
  xx[107] = xx[27];
  xx[108] = xx[30];
  xx[109] = xx[31];
  xx[110] = xx[32];
  pm_math_Quaternion_compose_ra(xx + 96, xx + 107, xx + 111);
  pm_math_Quaternion_compose_ra(xx + 111, xx + 44, xx + 115);
  xx[119] = xx[28];
  xx[120] = xx[33];
  xx[121] = xx[34];
  pm_math_Quaternion_xform_ra(xx + 111, xx + 119, xx + 122);
  xx[104] = xx[122] + xx[87];
  xx[105] = xx[123] + xx[106];
  xx[122] = xx[124] + xx[103];
  pm_math_Quaternion_compose_ra(xx + 115, xx + 54, xx + 123);
  xx[127] = xx[35];
  xx[128] = xx[37];
  xx[129] = xx[40];
  pm_math_Quaternion_xform_ra(xx + 115, xx + 127, xx + 130);
  xx[133] = xx[130] + xx[104];
  xx[134] = xx[131] + xx[105];
  xx[130] = xx[132] + xx[122];
  xx[135] = xx[41];
  xx[136] = xx[50];
  xx[137] = xx[51];
  xx[138] = xx[52];
  pm_math_Quaternion_compose_ra(xx + 123, xx + 135, xx + 139);
  xx[143] = xx[42];
  xx[144] = xx[43];
  xx[145] = xx[53];
  pm_math_Quaternion_xform_ra(xx + 123, xx + 143, xx + 146);
  xx[131] = xx[146] + xx[133];
  xx[132] = xx[147] + xx[134];
  xx[146] = xx[148] + xx[130];
  pm_math_Quaternion_compose_ra(xx + 139, xx + 71, xx + 147);
  pm_math_Quaternion_xform_ra(xx + 139, xx + 58, xx + 151);
  xx[154] = xx[11] * state[1];
  xx[155] = xx[9] * state[1];
  xx[156] = 2.0;
  xx[157] = - ((xx[154] * xx[3] + xx[10] * xx[155]) * xx[156]);
  xx[158] = (xx[11] * xx[154] + xx[155] * xx[9]) * xx[156] - state[1] + state[3];
  xx[159] = xx[156] * (xx[155] * xx[3] - xx[10] * xx[154]);
  pm_math_Quaternion_inverseXform_ra(xx + 23, xx + 157, xx + 154);
  xx[160] = xx[154] + xx[8] * state[5];
  xx[8] = xx[155] - xx[17] * state[5];
  xx[17] = xx[156] - xx[18] * state[5];
  pm_math_Vector3_cross_ra(xx + 157, xx + 68, xx + 154);
  pm_math_Quaternion_inverseXform_ra(xx + 23, xx + 154, xx + 68);
  xx[18] = xx[68] + 3.88810641306223e-5 * state[5];
  xx[154] = xx[69] + 0.03617154622233763 * state[5];
  xx[68] = xx[70] + 0.3675504912767608 * state[5];
  xx[161] = xx[160];
  xx[162] = xx[8];
  xx[163] = xx[17];
  pm_math_Quaternion_inverseXform_ra(xx + 92, xx + 161, xx + 164);
  xx[69] = xx[164] + state[7];
  pm_math_Vector3_cross_ra(xx + 161, xx + 100, xx + 167);
  xx[100] = xx[167] + xx[18];
  xx[101] = xx[168] + xx[154];
  xx[102] = xx[169] + xx[68];
  pm_math_Quaternion_inverseXform_ra(xx + 92, xx + 100, xx + 167);
  xx[92] = xx[69];
  xx[93] = xx[165];
  xx[94] = xx[166];
  pm_math_Quaternion_inverseXform_ra(xx + 107, xx + 92, xx + 100);
  xx[70] = xx[101] - state[9];
  pm_math_Quaternion_inverseXform_ra(xx + 107, xx + 167, xx + 92);
  xx[107] = xx[100];
  xx[108] = xx[70];
  xx[109] = xx[102];
  pm_math_Quaternion_inverseXform_ra(xx + 44, xx + 107, xx + 170);
  xx[95] = xx[170] + xx[29] * state[11];
  xx[29] = xx[171] + xx[38] * state[11];
  xx[38] = xx[172] - xx[39] * state[11];
  pm_math_Vector3_cross_ra(xx + 107, xx + 119, xx + 170);
  xx[107] = xx[170] + xx[92];
  xx[108] = xx[171] + xx[93];
  xx[109] = xx[172] + xx[94];
  pm_math_Quaternion_inverseXform_ra(xx + 44, xx + 107, xx + 119);
  xx[39] = xx[119] - 0.1232317192063306 * state[11];
  xx[101] = xx[120] - 0.07120994256178509 * state[11];
  xx[107] = xx[121] - 0.03694148414451474 * state[11];
  xx[108] = xx[95];
  xx[109] = xx[29];
  xx[110] = xx[38];
  pm_math_Quaternion_inverseXform_ra(xx + 54, xx + 108, xx + 119);
  xx[155] = xx[119] + xx[36] * state[13];
  xx[36] = xx[120] - xx[48] * state[13];
  xx[48] = xx[121] - xx[49] * state[13];
  pm_math_Vector3_cross_ra(xx + 108, xx + 127, xx + 119);
  xx[108] = xx[119] + xx[39];
  xx[109] = xx[120] + xx[101];
  xx[110] = xx[121] + xx[107];
  pm_math_Quaternion_inverseXform_ra(xx + 54, xx + 108, xx + 119);
  xx[49] = xx[119] + 0.0115241621900204 * state[13];
  xx[108] = xx[120] + 0.05252542267538761 * state[13];
  xx[109] = xx[121] - 0.1488828669086876 * state[13];
  xx[119] = xx[155];
  xx[120] = xx[36];
  xx[121] = xx[48];
  pm_math_Quaternion_inverseXform_ra(xx + 135, xx + 119, xx + 127);
  xx[110] = xx[128] - state[15];
  pm_math_Vector3_cross_ra(xx + 119, xx + 143, xx + 170);
  xx[119] = xx[170] + xx[49];
  xx[120] = xx[171] + xx[108];
  xx[121] = xx[172] + xx[109];
  pm_math_Quaternion_inverseXform_ra(xx + 135, xx + 119, xx + 143);
  xx[119] = xx[127];
  xx[120] = xx[110];
  xx[121] = xx[129];
  pm_math_Quaternion_inverseXform_ra(xx + 71, xx + 119, xx + 135);
  pm_math_Vector3_cross_ra(xx + 119, xx + 58, xx + 170);
  xx[119] = xx[170] + xx[143];
  xx[120] = xx[171] + xx[144];
  xx[121] = xx[172] + xx[145];
  pm_math_Quaternion_inverseXform_ra(xx + 71, xx + 119, xx + 170);
  pm_math_Quaternion_inverseXform_ra(xx + 80, xx + 161, xx + 119);
  xx[173] = xx[61];
  xx[174] = xx[62];
  xx[175] = xx[67];
  pm_math_Vector3_cross_ra(xx + 161, xx + 173, xx + 176);
  xx[161] = xx[176] + xx[18];
  xx[162] = xx[177] + xx[154];
  xx[163] = xx[178] + xx[68];
  pm_math_Quaternion_inverseXform_ra(xx + 80, xx + 161, xx + 173);
  motionData[0] = xx[1];
  motionData[1] = xx[4];
  motionData[2] = xx[5];
  motionData[3] = xx[6];
  motionData[4] = xx[2];
  motionData[5] = xx[2];
  motionData[6] = xx[2];
  motionData[7] = xx[3];
  motionData[8] = xx[9];
  motionData[9] = xx[10];
  motionData[10] = xx[11];
  motionData[11] = xx[2];
  motionData[12] = xx[2];
  motionData[13] = xx[2];
  motionData[14] = xx[23];
  motionData[15] = xx[24];
  motionData[16] = xx[25];
  motionData[17] = xx[26];
  motionData[18] = xx[7];
  motionData[19] = xx[12];
  motionData[20] = xx[13];
  motionData[21] = xx[14];
  motionData[22] = xx[19];
  motionData[23] = xx[20];
  motionData[24] = xx[21];
  motionData[25] = xx[15];
  motionData[26] = xx[16];
  motionData[27] = xx[22];
  motionData[28] = xx[27];
  motionData[29] = xx[30];
  motionData[30] = xx[31];
  motionData[31] = xx[32];
  motionData[32] = xx[2];
  motionData[33] = xx[2];
  motionData[34] = xx[2];
  motionData[35] = xx[44];
  motionData[36] = xx[45];
  motionData[37] = xx[46];
  motionData[38] = xx[47];
  motionData[39] = xx[28];
  motionData[40] = xx[33];
  motionData[41] = xx[34];
  motionData[42] = xx[54];
  motionData[43] = xx[55];
  motionData[44] = xx[56];
  motionData[45] = xx[57];
  motionData[46] = xx[35];
  motionData[47] = xx[37];
  motionData[48] = xx[40];
  motionData[49] = xx[41];
  motionData[50] = xx[50];
  motionData[51] = xx[51];
  motionData[52] = xx[52];
  motionData[53] = xx[42];
  motionData[54] = xx[43];
  motionData[55] = xx[53];
  motionData[56] = xx[71];
  motionData[57] = xx[72];
  motionData[58] = xx[73];
  motionData[59] = xx[74];
  motionData[60] = xx[58];
  motionData[61] = xx[59];
  motionData[62] = xx[60];
  motionData[63] = xx[80];
  motionData[64] = xx[81];
  motionData[65] = xx[82];
  motionData[66] = xx[83];
  motionData[67] = xx[61];
  motionData[68] = xx[62];
  motionData[69] = xx[67];
  motionData[70] = xx[88];
  motionData[71] = xx[89];
  motionData[72] = xx[90];
  motionData[73] = xx[91];
  motionData[74] = xx[2];
  motionData[75] = xx[2];
  motionData[76] = xx[2];
  motionData[77] = xx[76];
  motionData[78] = xx[77];
  motionData[79] = xx[78];
  motionData[80] = xx[79];
  motionData[81] = xx[84];
  motionData[82] = xx[85];
  motionData[83] = xx[86];
  motionData[84] = xx[96];
  motionData[85] = xx[97];
  motionData[86] = xx[98];
  motionData[87] = xx[99];
  motionData[88] = xx[87];
  motionData[89] = xx[106];
  motionData[90] = xx[103];
  motionData[91] = xx[111];
  motionData[92] = xx[112];
  motionData[93] = xx[113];
  motionData[94] = xx[114];
  motionData[95] = xx[87];
  motionData[96] = xx[106];
  motionData[97] = xx[103];
  motionData[98] = xx[115];
  motionData[99] = xx[116];
  motionData[100] = xx[117];
  motionData[101] = xx[118];
  motionData[102] = xx[104];
  motionData[103] = xx[105];
  motionData[104] = xx[122];
  motionData[105] = xx[123];
  motionData[106] = xx[124];
  motionData[107] = xx[125];
  motionData[108] = xx[126];
  motionData[109] = xx[133];
  motionData[110] = xx[134];
  motionData[111] = xx[130];
  motionData[112] = xx[139];
  motionData[113] = xx[140];
  motionData[114] = xx[141];
  motionData[115] = xx[142];
  motionData[116] = xx[131];
  motionData[117] = xx[132];
  motionData[118] = xx[146];
  motionData[119] = xx[147];
  motionData[120] = xx[148];
  motionData[121] = xx[149];
  motionData[122] = xx[150];
  motionData[123] = xx[151] + xx[131];
  motionData[124] = xx[152] + xx[132];
  motionData[125] = xx[153] + xx[146];
  motionData[126] = xx[2];
  motionData[127] = - state[1];
  motionData[128] = xx[2];
  motionData[129] = xx[2];
  motionData[130] = xx[2];
  motionData[131] = xx[2];
  motionData[132] = xx[157];
  motionData[133] = xx[158];
  motionData[134] = xx[159];
  motionData[135] = xx[2];
  motionData[136] = xx[2];
  motionData[137] = xx[2];
  motionData[138] = xx[160];
  motionData[139] = xx[8];
  motionData[140] = xx[17];
  motionData[141] = xx[18];
  motionData[142] = xx[154];
  motionData[143] = xx[68];
  motionData[144] = xx[69];
  motionData[145] = xx[165];
  motionData[146] = xx[166];
  motionData[147] = xx[167];
  motionData[148] = xx[168];
  motionData[149] = xx[169];
  motionData[150] = xx[100];
  motionData[151] = xx[70];
  motionData[152] = xx[102];
  motionData[153] = xx[92];
  motionData[154] = xx[93];
  motionData[155] = xx[94];
  motionData[156] = xx[95];
  motionData[157] = xx[29];
  motionData[158] = xx[38];
  motionData[159] = xx[39];
  motionData[160] = xx[101];
  motionData[161] = xx[107];
  motionData[162] = xx[155];
  motionData[163] = xx[36];
  motionData[164] = xx[48];
  motionData[165] = xx[49];
  motionData[166] = xx[108];
  motionData[167] = xx[109];
  motionData[168] = xx[127];
  motionData[169] = xx[110];
  motionData[170] = xx[129];
  motionData[171] = xx[143];
  motionData[172] = xx[144];
  motionData[173] = xx[145];
  motionData[174] = xx[135] - xx[63] * state[17];
  motionData[175] = xx[136] - xx[65] * state[17];
  motionData[176] = xx[137] + xx[66] * state[17];
  motionData[177] = xx[170] - 0.03146781994356587 * state[17];
  motionData[178] = xx[171] - 0.03789251374844847 * state[17];
  motionData[179] = xx[172] - 0.01243276758501344 * state[17];
  motionData[180] = xx[119] + xx[0] * state[19];
  motionData[181] = xx[120] - xx[64] * state[19];
  motionData[182] = xx[121] - xx[75] * state[19];
  motionData[183] = xx[173] - 0.01029859628626135 * state[19];
  motionData[184] = xx[174] - 8.928983924324474e-3 * state[19];
  motionData[185] = xx[175] + 0.1772779043919793 * state[19];
}

size_t human9DOF_836bb176_1_computeAssemblyError(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, size_t constraintIdx, const double *state,
  const int *modeVector, const double *motionData, double *error)
{
  (void) mech;
  (void)rtdv;
  (void) state;
  (void) modeVector;
  (void) motionData;
  (void) error;
  switch (constraintIdx)
  {
  }

  return 0;
}

size_t human9DOF_836bb176_1_computeAssemblyJacobian(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, size_t constraintIdx, boolean_T
  forVelocitySatisfaction, const double *state, const int *modeVector, const
  double *motionData, double *J)
{
  (void) mech;
  (void) rtdv;
  (void) state;
  (void) modeVector;
  (void) forVelocitySatisfaction;
  (void) motionData;
  (void) J;
  switch (constraintIdx)
  {
  }

  return 0;
}

size_t human9DOF_836bb176_1_computeFullAssemblyJacobian(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, const double *state, const int *modeVector,
  const double *motionData, double *J)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  (void) state;
  (void) modeVector;
  (void) motionData;
  (void) J;
  return 0;
}

int human9DOF_836bb176_1_isInKinematicSingularity(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, size_t constraintIdx, const int *modeVector,
  const double *motionData)
{
  (void) mech;
  (void) rtdv
    ;
  (void) modeVector;
  (void) motionData;
  switch (constraintIdx)
  {
  }

  return 0;
}

PmfMessageId human9DOF_836bb176_1_convertStateVector(const void *asmMech, const
  RuntimeDerivedValuesBundle *rtdv, const void *simMech, const double *asmState,
  const int *asmModeVector, const int *simModeVector, double *simState, void
  *neDiagMgr0)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  NeuDiagnosticManager *neDiagMgr = (NeuDiagnosticManager *) neDiagMgr0;
  (void) asmMech;
  (void) rtdvd;
  (void) rtdvi;
  (void) simMech;
  (void) asmModeVector;
  (void) simModeVector;
  (void) neDiagMgr;
  simState[0] = asmState[0];
  simState[1] = asmState[1];
  simState[2] = asmState[2];
  simState[3] = asmState[3];
  simState[4] = asmState[4];
  simState[5] = asmState[5];
  simState[6] = asmState[6];
  simState[7] = asmState[7];
  simState[8] = asmState[8];
  simState[9] = asmState[9];
  simState[10] = asmState[10];
  simState[11] = asmState[11];
  simState[12] = asmState[12];
  simState[13] = asmState[13];
  simState[14] = asmState[14];
  simState[15] = asmState[15];
  simState[16] = asmState[16];
  simState[17] = asmState[17];
  simState[18] = asmState[18];
  simState[19] = asmState[19];
  return NULL;
}
