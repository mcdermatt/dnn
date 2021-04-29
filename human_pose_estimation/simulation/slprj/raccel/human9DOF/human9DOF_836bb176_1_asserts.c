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

void human9DOF_836bb176_1_validateRuntimeParameters(const double *rtp, int32_T
  *satFlags)
{
  boolean_T bb[21];
  double xx[2];
  bb[0] = !pmf_is_inf(rtp[0]);
  bb[1] = !pmf_is_nan(rtp[0]);
  bb[2] = bb[0] && bb[1];
  xx[0] = 0.0174532925199433;
  xx[1] = 0.0;
  bb[3] = !pmf_is_inf(rtp[1]);
  bb[4] = !pmf_is_nan(rtp[1]);
  bb[5] = !pmf_is_inf(rtp[2]);
  bb[6] = !pmf_is_nan(rtp[2]);
  bb[7] = bb[3] && bb[4];
  bb[8] = bb[5] && bb[6];
  bb[9] = !pmf_is_inf(rtp[13]);
  bb[10] = !pmf_is_nan(rtp[13]);
  bb[11] = !pmf_is_inf(rtp[14]);
  bb[12] = !pmf_is_nan(rtp[14]);
  bb[13] = bb[9] && bb[10];
  bb[14] = bb[11] && bb[12];
  bb[15] = !pmf_is_inf(rtp[16]);
  bb[16] = !pmf_is_nan(rtp[16]);
  bb[17] = !pmf_is_inf(rtp[17]);
  bb[18] = !pmf_is_nan(rtp[17]);
  bb[19] = bb[15] && bb[16];
  bb[20] = bb[17] && bb[18];
  satFlags[0] = bb[0] ? 1 : 0;
  satFlags[1] = bb[1] ? 1 : 0;
  satFlags[2] = !bb[2] || (bb[2] ? xx[0] * rtp[0] : xx[1]) < 1.398482201733458 ?
    1 : 0;
  satFlags[3] = !pmf_is_inf(rtp[3]) ? 1 : 0;
  satFlags[4] = !pmf_is_nan(rtp[3]) ? 1 : 0;
  satFlags[5] = bb[3] ? 1 : 0;
  satFlags[6] = bb[4] ? 1 : 0;
  satFlags[7] = bb[5] ? 1 : 0;
  satFlags[8] = bb[6] ? 1 : 0;
  satFlags[9] = !(bb[7] && bb[8]) || (bb[7] ? xx[0] * rtp[1] : xx[1]) < (bb[8] ?
    xx[0] * rtp[2] : xx[1]) ? 1 : 0;
  satFlags[10] = !pmf_is_inf(rtp[4]) ? 1 : 0;
  satFlags[11] = !pmf_is_nan(rtp[4]) ? 1 : 0;
  satFlags[12] = !pmf_is_inf(rtp[5]) ? 1 : 0;
  satFlags[13] = !pmf_is_nan(rtp[5]) ? 1 : 0;
  satFlags[14] = !pmf_is_inf(rtp[6]) ? 1 : 0;
  satFlags[15] = !pmf_is_nan(rtp[6]) ? 1 : 0;
  satFlags[16] = !pmf_is_inf(rtp[7]) ? 1 : 0;
  satFlags[17] = !pmf_is_nan(rtp[7]) ? 1 : 0;
  satFlags[18] = !pmf_is_inf(rtp[8]) ? 1 : 0;
  satFlags[19] = !pmf_is_nan(rtp[8]) ? 1 : 0;
  satFlags[20] = !pmf_is_inf(rtp[9]) ? 1 : 0;
  satFlags[21] = !pmf_is_nan(rtp[9]) ? 1 : 0;
  satFlags[22] = !pmf_is_inf(rtp[10]) ? 1 : 0;
  satFlags[23] = !pmf_is_nan(rtp[10]) ? 1 : 0;
  satFlags[24] = !pmf_is_inf(rtp[11]) ? 1 : 0;
  satFlags[25] = !pmf_is_nan(rtp[11]) ? 1 : 0;
  satFlags[26] = !pmf_is_inf(rtp[12]) ? 1 : 0;
  satFlags[27] = !pmf_is_nan(rtp[12]) ? 1 : 0;
  satFlags[28] = !pmf_is_inf(rtp[15]) ? 1 : 0;
  satFlags[29] = !pmf_is_nan(rtp[15]) ? 1 : 0;
  satFlags[30] = bb[9] ? 1 : 0;
  satFlags[31] = bb[10] ? 1 : 0;
  satFlags[32] = bb[11] ? 1 : 0;
  satFlags[33] = bb[12] ? 1 : 0;
  satFlags[34] = !(bb[13] && bb[14]) || (bb[13] ? xx[0] * rtp[13] : xx[1]) <
    (bb[14] ? xx[0] * rtp[14] : xx[1]) ? 1 : 0;
  satFlags[35] = !pmf_is_inf(rtp[18]) ? 1 : 0;
  satFlags[36] = !pmf_is_nan(rtp[18]) ? 1 : 0;
  satFlags[37] = bb[15] ? 1 : 0;
  satFlags[38] = bb[16] ? 1 : 0;
  satFlags[39] = bb[17] ? 1 : 0;
  satFlags[40] = bb[18] ? 1 : 0;
  satFlags[41] = !(bb[19] && bb[20]) || (bb[19] ? xx[0] * rtp[16] : xx[1]) <
    (bb[20] ? xx[0] * rtp[17] : xx[1]) ? 1 : 0;
}

const NeAssertData human9DOF_836bb176_1_assertData[42] = {
  { "human9DOF/Chicken Wing", 0, 0, "Chicken_Wing", "", false,
    "The parameter Rz/Lower Limit/Bound contains an Inf value, which is not allowed.",
    "sm:model:evaluate:InvalidValueInf" },

  { "human9DOF/Chicken Wing", 0, 0, "Chicken_Wing", "", false,
    "The parameter Rz/Lower Limit/Bound contains a NaN value, which is not allowed.",
    "sm:model:evaluate:InvalidValueNaN" },

  { "human9DOF/Chicken Wing", 0, 0, "Chicken_Wing", "", false,
    "The joint limits of primitive Rz have invalid bounds: the lower bound must be strictly less than the upper bound.",
    "sm:model:jointPrimitive:NullLimitRange" },

  { "human9DOF/curl", 0, 0, "curl", "", false,
    "The parameter Rz/Velocity contains an Inf value, which is not allowed.",
    "sm:model:evaluate:InvalidValueInf" },

  { "human9DOF/curl", 0, 0, "curl", "", false,
    "The parameter Rz/Velocity contains a NaN value, which is not allowed.",
    "sm:model:evaluate:InvalidValueNaN" },

  { "human9DOF/curl", 0, 0, "curl", "", false,
    "The parameter Rz/Lower Limit/Bound contains an Inf value, which is not allowed.",
    "sm:model:evaluate:InvalidValueInf" },

  { "human9DOF/curl", 0, 0, "curl", "", false,
    "The parameter Rz/Lower Limit/Bound contains a NaN value, which is not allowed.",
    "sm:model:evaluate:InvalidValueNaN" },

  { "human9DOF/curl", 0, 0, "curl", "", false,
    "The parameter Rz/Upper Limit/Bound contains an Inf value, which is not allowed.",
    "sm:model:evaluate:InvalidValueInf" },

  { "human9DOF/curl", 0, 0, "curl", "", false,
    "The parameter Rz/Upper Limit/Bound contains a NaN value, which is not allowed.",
    "sm:model:evaluate:InvalidValueNaN" },

  { "human9DOF/curl", 0, 0, "curl", "", false,
    "The joint limits of primitive Rz have invalid bounds: the lower bound must be strictly less than the upper bound.",
    "sm:model:jointPrimitive:NullLimitRange" },

  { "human9DOF/hipside", 0, 0, "hipside", "", false,
    "The parameter Rz/Velocity contains an Inf value, which is not allowed.",
    "sm:model:evaluate:InvalidValueInf" },

  { "human9DOF/hipside", 0, 0, "hipside", "", false,
    "The parameter Rz/Velocity contains a NaN value, which is not allowed.",
    "sm:model:evaluate:InvalidValueNaN" },

  { "human9DOF/initial position j0", 0, 0, "initial_position_j0", "", false,
    "The parameter Angle contains an Inf value, which is not allowed.",
    "sm:model:evaluate:InvalidValueInf" },

  { "human9DOF/initial position j0", 0, 0, "initial_position_j0", "", false,
    "The parameter Angle contains a NaN value, which is not allowed.",
    "sm:model:evaluate:InvalidValueNaN" },

  { "human9DOF/initial position j1", 0, 0, "initial_position_j1", "", false,
    "The parameter Angle contains an Inf value, which is not allowed.",
    "sm:model:evaluate:InvalidValueInf" },

  { "human9DOF/initial position j1", 0, 0, "initial_position_j1", "", false,
    "The parameter Angle contains a NaN value, which is not allowed.",
    "sm:model:evaluate:InvalidValueNaN" },

  { "human9DOF/initialpositionj2", 0, 0, "initialpositionj2", "", false,
    "The parameter Angle contains an Inf value, which is not allowed.",
    "sm:model:evaluate:InvalidValueInf" },

  { "human9DOF/initialpositionj2", 0, 0, "initialpositionj2", "", false,
    "The parameter Angle contains a NaN value, which is not allowed.",
    "sm:model:evaluate:InvalidValueNaN" },

  { "human9DOF/initialpositionj3", 0, 0, "initialpositionj3", "", false,
    "The parameter Angle contains an Inf value, which is not allowed.",
    "sm:model:evaluate:InvalidValueInf" },

  { "human9DOF/initialpositionj3", 0, 0, "initialpositionj3", "", false,
    "The parameter Angle contains a NaN value, which is not allowed.",
    "sm:model:evaluate:InvalidValueNaN" },

  { "human9DOF/initialpositionj4", 0, 0, "initialpositionj4", "", false,
    "The parameter Angle contains an Inf value, which is not allowed.",
    "sm:model:evaluate:InvalidValueInf" },

  { "human9DOF/initialpositionj4", 0, 0, "initialpositionj4", "", false,
    "The parameter Angle contains a NaN value, which is not allowed.",
    "sm:model:evaluate:InvalidValueNaN" },

  { "human9DOF/initialpositionj5", 0, 0, "initialpositionj5", "", false,
    "The parameter Angle contains an Inf value, which is not allowed.",
    "sm:model:evaluate:InvalidValueInf" },

  { "human9DOF/initialpositionj5", 0, 0, "initialpositionj5", "", false,
    "The parameter Angle contains a NaN value, which is not allowed.",
    "sm:model:evaluate:InvalidValueNaN" },

  { "human9DOF/initialpositionj6", 0, 0, "initialpositionj6", "", false,
    "The parameter Angle contains an Inf value, which is not allowed.",
    "sm:model:evaluate:InvalidValueInf" },

  { "human9DOF/initialpositionj6", 0, 0, "initialpositionj6", "", false,
    "The parameter Angle contains a NaN value, which is not allowed.",
    "sm:model:evaluate:InvalidValueNaN" },

  { "human9DOF/initialpositionj7", 0, 0, "initialpositionj7", "", false,
    "The parameter Angle contains an Inf value, which is not allowed.",
    "sm:model:evaluate:InvalidValueInf" },

  { "human9DOF/initialpositionj7", 0, 0, "initialpositionj7", "", false,
    "The parameter Angle contains a NaN value, which is not allowed.",
    "sm:model:evaluate:InvalidValueNaN" },

  { "human9DOF/j6", 0, 0, "j6", "", false,
    "The parameter Rz/Velocity contains an Inf value, which is not allowed.",
    "sm:model:evaluate:InvalidValueInf" },

  { "human9DOF/j6", 0, 0, "j6", "", false,
    "The parameter Rz/Velocity contains a NaN value, which is not allowed.",
    "sm:model:evaluate:InvalidValueNaN" },

  { "human9DOF/j6", 0, 0, "j6", "", false,
    "The parameter Rz/Lower Limit/Bound contains an Inf value, which is not allowed.",
    "sm:model:evaluate:InvalidValueInf" },

  { "human9DOF/j6", 0, 0, "j6", "", false,
    "The parameter Rz/Lower Limit/Bound contains a NaN value, which is not allowed.",
    "sm:model:evaluate:InvalidValueNaN" },

  { "human9DOF/j6", 0, 0, "j6", "", false,
    "The parameter Rz/Upper Limit/Bound contains an Inf value, which is not allowed.",
    "sm:model:evaluate:InvalidValueInf" },

  { "human9DOF/j6", 0, 0, "j6", "", false,
    "The parameter Rz/Upper Limit/Bound contains a NaN value, which is not allowed.",
    "sm:model:evaluate:InvalidValueNaN" },

  { "human9DOF/j6", 0, 0, "j6", "", false,
    "The joint limits of primitive Rz have invalid bounds: the lower bound must be strictly less than the upper bound.",
    "sm:model:jointPrimitive:NullLimitRange" },

  { "human9DOF/j7", 0, 0, "j7", "", false,
    "The parameter Rz/Velocity contains an Inf value, which is not allowed.",
    "sm:model:evaluate:InvalidValueInf" },

  { "human9DOF/j7", 0, 0, "j7", "", false,
    "The parameter Rz/Velocity contains a NaN value, which is not allowed.",
    "sm:model:evaluate:InvalidValueNaN" },

  { "human9DOF/j7", 0, 0, "j7", "", false,
    "The parameter Rz/Lower Limit/Bound contains an Inf value, which is not allowed.",
    "sm:model:evaluate:InvalidValueInf" },

  { "human9DOF/j7", 0, 0, "j7", "", false,
    "The parameter Rz/Lower Limit/Bound contains a NaN value, which is not allowed.",
    "sm:model:evaluate:InvalidValueNaN" },

  { "human9DOF/j7", 0, 0, "j7", "", false,
    "The parameter Rz/Upper Limit/Bound contains an Inf value, which is not allowed.",
    "sm:model:evaluate:InvalidValueInf" },

  { "human9DOF/j7", 0, 0, "j7", "", false,
    "The parameter Rz/Upper Limit/Bound contains a NaN value, which is not allowed.",
    "sm:model:evaluate:InvalidValueNaN" },

  { "human9DOF/j7", 0, 0, "j7", "", false,
    "The joint limits of primitive Rz have invalid bounds: the lower bound must be strictly less than the upper bound.",
    "sm:model:jointPrimitive:NullLimitRange" }
};
