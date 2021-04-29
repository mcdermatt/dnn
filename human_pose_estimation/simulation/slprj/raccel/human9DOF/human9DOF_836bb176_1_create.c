/* Simscape target specific file.
 * This file is generated for the Simscape network associated with the solver block 'human9DOF/Solver Configuration'.
 */

#include "pm_std.h"
#include "ne_std.h"
#include "ne_dae.h"
#include "pm_default_allocator.h"
#include "sm_ssci_NeDaePrivateData.h"
#include "sm_CTarget.h"

PmfMessageId sm_ssci_recordRunTimeError(
  const char *errorId, const char *errorMsg, NeuDiagnosticManager* mgr);

#define pm_allocator_alloc(_allocator, _m, _n) ((_allocator)->mCallocFcn((_allocator), (_m), (_n)))
#define PM_ALLOCATE_ARRAY(_name, _type, _size, _allocator)\
 _name = (_type *) pm_allocator_alloc(_allocator, sizeof(_type), _size)
#define pm_size_to_int(_size)          ((int32_T) (_size))

PmIntVector *pm_create_int_vector(size_t, PmAllocator *);
int_T pm_create_int_vector_fields (PmIntVector *, size_t, PmAllocator *);
int_T pm_create_real_vector_fields(PmRealVector *, size_t, PmAllocator *);
int_T pm_create_char_vector_fields(PmCharVector *, size_t, PmAllocator *);
int_T pm_create_bool_vector_fields(PmBoolVector *, size_t, PmAllocator *);
void pm_rv_equals_rv(const PmRealVector *, const PmRealVector *);
void sm_ssci_setupLoggerFcn_codeGen(const NeDae *dae,
  NeLoggerBuilder *neLoggerBuilder);
int32_T sm_ssci_logFcn_codeGen(const NeDae *dae,
  const NeSystemInput *systemInput,
  PmRealVector *output);
extern const NeAssertData human9DOF_836bb176_1_assertData[];
extern const NeZCData human9DOF_836bb176_1_ZCData[];
void human9DOF_836bb176_1_computeRuntimeParameters(
  const double *runtimeRootVariables,
  double *runtimeParameters);
void human9DOF_836bb176_1_validateRuntimeParameters(
  const double *runtimeParameters,
  int32_T *assertSatisfactionFlags);
void human9DOF_836bb176_1_computeAsmRuntimeDerivedValues(
  const double *runtimeParameters,
  RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle);
void human9DOF_836bb176_1_computeSimRuntimeDerivedValues(
  const double *runtimeParameters,
  RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle);
PmfMessageId human9DOF_836bb176_1_compDerivs(
  const RuntimeDerivedValuesBundle *,
  const int *,
  const double *,
  const int *,
  const double *, const double *, const double *,
  const double *,
  double *,
  double *,
  NeuDiagnosticManager *neDiagMgr);
PmfMessageId human9DOF_836bb176_1_numJacPerturbLoBounds(
  const RuntimeDerivedValuesBundle *,
  const int *,
  const double *,
  const int *,
  const double *, const double *, const double *,
  const double *,
  double *,
  double *,
  NeuDiagnosticManager *neDiagMgr);
PmfMessageId human9DOF_836bb176_1_numJacPerturbHiBounds(
  const RuntimeDerivedValuesBundle *,
  const int *,
  const double *,
  const int *,
  const double *, const double *, const double *,
  const double *,
  double *,
  double *,
  NeuDiagnosticManager *neDiagMgr);
PmfMessageId human9DOF_836bb176_1_checkDynamics(
  const RuntimeDerivedValuesBundle *,
  const double *,
  const double *, const double *, const double *,
  const double *,
  double *,
  NeuDiagnosticManager *neDiagMgr);
PmfMessageId human9DOF_836bb176_1_compOutputsDyn(
  const RuntimeDerivedValuesBundle *,
  const int *,
  const double *,
  const int *,
  const double *, const double *, const double *,
  const double *,
  double *,
  double *,
  int *,
  double *,
  NeuDiagnosticManager *neDiagMgr);
PmfMessageId human9DOF_836bb176_1_compOutputsKin(
  const RuntimeDerivedValuesBundle *,
  const double *,
  const int *,
  const double *, const double *, const double *,
  const double *,
  double *,
  NeuDiagnosticManager *neDiagMgr);
PmfMessageId human9DOF_836bb176_1_compOutputs (
  const RuntimeDerivedValuesBundle *,
  const int *,
  const double *,
  const int *,
  const double *, const double *, const double *,
  const double *,
  double *,
  double *,
  int *,
  double *,
  NeuDiagnosticManager *neDiagMgr);
PmfMessageId human9DOF_836bb176_1_computeAsmModeVector(
  const double *, const double *, const double *,
  int *,
  double *,
  NeuDiagnosticManager *neDiagMgr);
PmfMessageId human9DOF_836bb176_1_computeSimModeVector(
  const double *, const double *, const double *,
  int *,
  double *,
  NeuDiagnosticManager *neDiagMgr);
PmfMessageId human9DOF_836bb176_1_computeZeroCrossings(
  const RuntimeDerivedValuesBundle *,
  const double *,
  const double *, const double *, const double *,
  const double *,
  double *,
  double *,
  NeuDiagnosticManager *neDiagMgr);
void human9DOF_836bb176_1_setTargets(
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  CTarget *targets);
void human9DOF_836bb176_1_resetAsmStateVector(const void *mech, double
  *stateVector);
void human9DOF_836bb176_1_resetSimStateVector(const void *mech, double
  *stateVector);
void human9DOF_836bb176_1_initializeTrackedAngleState(
  const void *mech,
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  const int *modeVector,
  const double *motionData,
  double *stateVector,
  void *neDiagMgr);
void human9DOF_836bb176_1_computeDiscreteState(
  const void *mech,
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  double *stateVector);
void human9DOF_836bb176_1_adjustPosition(
  const void *mech,
  const double *dofDeltas,
  double *stateVector);
void human9DOF_836bb176_1_perturbAsmJointPrimitiveState(
  const void *mech,
  size_t stageIdx,
  size_t primitiveIdx,
  double magnitude,
  boolean_T doPerturbVelocity,
  double *stateVector);
void human9DOF_836bb176_1_perturbSimJointPrimitiveState(
  const void *mech,
  size_t stageIdx,
  size_t primitiveIdx,
  double magnitude,
  boolean_T doPerturbVelocity,
  double *stateVector);
void human9DOF_836bb176_1_perturbFlexibleBodyState(
  const void *mech,
  size_t stageIdx,
  double magnitude,
  boolean_T doPerturbVelocity,
  double *stateVector);
void human9DOF_836bb176_1_computePosDofBlendMatrix(
  const void *mech,
  size_t stageIdx,
  size_t primitiveIdx,
  const double *stateVector,
  int partialType,
  double *matrix);
void human9DOF_836bb176_1_computeVelDofBlendMatrix(
  const void *mech,
  size_t stageIdx,
  size_t primitiveIdx,
  const double *stateVector,
  int partialType,
  double *matrix);
void human9DOF_836bb176_1_projectPartiallyTargetedPos(
  const void *mech,
  size_t stageIdx,
  size_t primitiveIdx,
  const double *origStateVector,
  int partialType,
  double *stateVector);
void human9DOF_836bb176_1_propagateMotion(
  const void *mech,
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  const double *stateVector,
  double *motionData);
size_t human9DOF_836bb176_1_computeAssemblyError(
  const void *mech,
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  size_t constraintIdx,
  const double *stateVector,
  const int *modeVector,
  const double *motionData,
  double *error);
size_t human9DOF_836bb176_1_computeAssemblyJacobian(
  const void *mech,
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  size_t constraintIdx,
  boolean_T forVelocitySatisfaction,
  const double *stateVector,
  const int *modeVector,
  const double *motionData,
  double *J);
size_t human9DOF_836bb176_1_computeFullAssemblyJacobian(
  const void *mech,
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  const double *stateVector,
  const int *modeVector,
  const double *motionData,
  double *J);
int human9DOF_836bb176_1_isInKinematicSingularity(
  const void *mech,
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  size_t constraintIdx,
  const int *modeVector,
  const double *motionData);
PmfMessageId human9DOF_836bb176_1_convertStateVector(
  const void *asmMech,
  const RuntimeDerivedValuesBundle *asmRuntimeDerivedValuesBundle,
  const void *simMech,
  const double *asmStateVector,
  const int *asmModeVector,
  const int *simModeVector,
  double *simStateVector,
  void *neDiagMgr);
void human9DOF_836bb176_1_constructStateVector(
  const void *mech,
  const double *solverStateVector,
  const double *u,
  const double *uDot,
  const double *discreteStateVector,
  double *fullStateVector);
void human9DOF_836bb176_1_extractSolverStateVector(
  const void *mech,
  const double *fullStateVector,
  double *solverStateVector);
int human9DOF_836bb176_1_isPositionViolation(
  const void *mech,
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  const int *constraintEqnEnableFlags,
  const double *stateVector,
  const int *modeVector);
int human9DOF_836bb176_1_isVelocityViolation(
  const void *mech,
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  const int *constraintEqnEnableFlags,
  const double *stateVector,
  const int *modeVector);
PmfMessageId human9DOF_836bb176_1_projectStateSim(
  const void *mech,
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  const int *constraintEqnEnableFlags,
  const int *modeVector,
  const double *inputVector,
  double *stateVector,
  void *neDiagMgr);
void human9DOF_836bb176_1_computeConstraintError(
  const void *mech,
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  const double *stateVector,
  const int *modeVector,
  double *error);
void human9DOF_836bb176_1_resetModeVector(const void *mech, int *modeVector);
boolean_T human9DOF_836bb176_1_hasJointDisToNormModeChange(
  const void *mech,
  const int *prevModeVector,
  const int *modeVector);
PmfMessageId human9DOF_836bb176_1_performJointDisToNormModeChange(
  const void *mech,
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  const int *prevModeVector,
  const int *modeVector,
  const double *inputVector,
  double *stateVector,
  void *neDiagMgr);
PmfMessageId human9DOF_836bb176_1_onModeChangedCutJoints(
  const void *mech,
  const int *prevModeVector,
  int *modeVector,
  double *stateVector,
  void *neDiagMgr);
PmfMessageId human9DOF_836bb176_1_assemble(const double *u, double *udot, double
  *x,
  NeuDiagnosticManager *neDiagMgr)
{
  (void) x;
  (void) u;
  (void) udot;
  (void) neDiagMgr;
  return NULL;
}

static
  void dae_cg_setParameters_function(const NeDae *dae,
  const NeParameterBundle *paramBundle)
{
  const NeDaePrivateData *smData = dae->mPrivateData;
  const double *runtimeRootVariables = paramBundle->mRealParameters.mX;
  if (smData->mRuntimeParameterScalars.mN == 0)
    return;
  human9DOF_836bb176_1_computeRuntimeParameters(
    runtimeRootVariables,
    smData->mRuntimeParameterScalars.mX);
  human9DOF_836bb176_1_computeAsmRuntimeDerivedValues(
    smData->mRuntimeParameterScalars.mX,
    &dae->mPrivateData->mAsmRuntimeDerivedValuesBundle);
  human9DOF_836bb176_1_computeSimRuntimeDerivedValues(
    smData->mRuntimeParameterScalars.mX,
    &dae->mPrivateData->mSimRuntimeDerivedValuesBundle);
  sm_core_computeRedundantConstraintEquations(
    &dae->mPrivateData->mSimulationDelegate,
    &smData->mSimRuntimeDerivedValuesBundle);

#if 0

  {
    size_t i;
    const size_t n = smData->mSimulationDelegate.mRunTimeEnabledEquations.mSize;
    pmf_printf("\nRuntime Enabled Equations (%lu)\n", n);
    for (i = 0; i < n; ++i)
      pmf_printf("  %2lu:  %d\n", i,
                 smData->mSimulationDelegate.mRunTimeEnabledEquations.mValues[i]);
  }

#endif

}

static
  PmfMessageId dae_cg_pAssert_method(const NeDae *dae,
  const NeSystemInput *systemInput,
  NeDaeMethodOutput *daeMethodOutput,
  NeuDiagnosticManager *neDiagMgr)
{
  const NeDaePrivateData *smData = dae->mPrivateData;
  const double *runtimeParams = smData->mRuntimeParameterScalars.mX;
  int32_T *assertSatisfactionFlags = daeMethodOutput->mPASSERT.mX;
  (void) systemInput;
  (void) neDiagMgr;
  human9DOF_836bb176_1_validateRuntimeParameters(
    runtimeParams, assertSatisfactionFlags);
  return NULL;
}

static
  PmfMessageId dae_cg_deriv_method(const NeDae *dae,
  const NeSystemInput *systemInput,
  NeDaeMethodOutput *daeMethodOutput,
  NeuDiagnosticManager *neDiagMgr)
{
  const NeDaePrivateData *smData = dae->mPrivateData;
  PmfMessageId errorId = NULL;
  double errorResult = 0.0;
  if (smData->mCachedDerivativesAvailable)
    memcpy(daeMethodOutput->mXP0.mX, smData->mCachedDerivatives.mX,
           20 * sizeof(real_T));
  else
    errorId = human9DOF_836bb176_1_compDerivs(
      &smData->mSimRuntimeDerivedValuesBundle,
      smData->mSimulationDelegate
      .mRunTimeEnabledEquations.mValues,
      systemInput->mX.mX,
      systemInput->mM.mX,
      systemInput->mU.mX,
      systemInput->mU.mX + 3,
      systemInput->mV.mX + 3,
      systemInput->mD.mX,
      daeMethodOutput->mXP0.mX,
      &errorResult,
      neDiagMgr);
  return errorId;
}

static
  PmfMessageId dae_cg_numJacPerturbLoBounds_method(
  const NeDae *dae,
  const NeSystemInput *systemInput,
  NeDaeMethodOutput *daeMethodOutput,
  NeuDiagnosticManager *neDiagMgr)
{
  const NeDaePrivateData *smData = dae->mPrivateData;
  PmfMessageId errorId = NULL;
  double errorResult = 0.0;
  errorId = human9DOF_836bb176_1_numJacPerturbLoBounds(
    &smData->mSimRuntimeDerivedValuesBundle,
    smData->mSimulationDelegate
    .mRunTimeEnabledEquations.mValues,
    systemInput->mX.mX,
    systemInput->mM.mX,
    systemInput->mU.mX,
    systemInput->mU.mX + 3,
    systemInput->mV.mX + 3,
    systemInput->mD.mX,
    daeMethodOutput->mNUMJAC_DX_LO.mX,
    &errorResult,
    neDiagMgr);
  return errorId;
}

static
  PmfMessageId dae_cg_numJacPerturbHiBounds_method(
  const NeDae *dae,
  const NeSystemInput *systemInput,
  NeDaeMethodOutput *daeMethodOutput,
  NeuDiagnosticManager *neDiagMgr)
{
  const NeDaePrivateData *smData = dae->mPrivateData;
  PmfMessageId errorId = NULL;
  double errorResult = 0.0;
  errorId = human9DOF_836bb176_1_numJacPerturbHiBounds(
    &smData->mSimRuntimeDerivedValuesBundle,
    smData->mSimulationDelegate
    .mRunTimeEnabledEquations.mValues,
    systemInput->mX.mX,
    systemInput->mM.mX,
    systemInput->mU.mX,
    systemInput->mU.mX + 3,
    systemInput->mV.mX + 3,
    systemInput->mD.mX,
    daeMethodOutput->mNUMJAC_DX_HI.mX,
    &errorResult,
    neDiagMgr);
  return errorId;
}

static
  PmfMessageId dae_cg_compOutputs_method(const NeDae *dae,
  const NeSystemInput *systemInput,
  NeDaeMethodOutput *daeMethodOutput,
  NeuDiagnosticManager *neDiagMgr)
{
  PmfMessageId errorId = NULL;
  NeDaePrivateData *smData = dae->mPrivateData;
  if (smData->mDoComputeDynamicOutputs) {
    int derivErr = 0;
    double errorResult = 0.0;
    errorId = human9DOF_836bb176_1_compOutputsDyn(
      &smData->mSimRuntimeDerivedValuesBundle,
      smData->mSimulationDelegate
      .mRunTimeEnabledEquations.mValues,
      systemInput->mX.mX,
      systemInput->mM.mX,
      systemInput->mU.mX,
      systemInput->mU.mX + 3,
      systemInput->mV.mX + 3,
      systemInput->mD.mX,
      smData->mCachedDerivatives.mX, daeMethodOutput->mY.mX,
      &derivErr, &errorResult, neDiagMgr);
    smData->mCachedDerivativesAvailable = (derivErr == 0);
  } else
    errorId = human9DOF_836bb176_1_compOutputsKin(
      &smData->mSimRuntimeDerivedValuesBundle,
      systemInput->mX.mX,
      systemInput->mM.mX,
      systemInput->mU.mX,
      systemInput->mU.mX + 3,
      systemInput->mV.mX + 3,
      systemInput->mD.mX,
      daeMethodOutput->mY.mX, neDiagMgr);
  return errorId;
}

static
  PmfMessageId dae_cg_mode_method(const NeDae *dae,
  const NeSystemInput *systemInput,
  NeDaeMethodOutput *daeMethodOutput,
  NeuDiagnosticManager *neDiagMgr)
{
  const NeDaePrivateData *smData = dae->mPrivateData;
  PmfMessageId errorId = NULL;
  double errorResult = 0.0;
  errorId = human9DOF_836bb176_1_computeSimModeVector(
    systemInput->mU.mX,
    systemInput->mU.mX + 3,
    systemInput->mV.mX + 3,
    daeMethodOutput->mMODE.mX,
    &errorResult,
    neDiagMgr);
  memcpy(smData->mCachedModeVector.mX, daeMethodOutput->mMODE.mX,
         0 * sizeof(int32_T));
  return errorId;
}

static
  PmfMessageId dae_cg_zeroCrossing_method(const NeDae *dae,
  const NeSystemInput *systemInput,
  NeDaeMethodOutput *daeMethodOutput,
  NeuDiagnosticManager *neDiagMgr)
{
  const NeDaePrivateData *smData = dae->mPrivateData;
  double errorResult = 0.0;
  return
    human9DOF_836bb176_1_computeZeroCrossings(
    &smData->mSimRuntimeDerivedValuesBundle,
    systemInput->mX.mX,
    systemInput->mU.mX,
    systemInput->mU.mX + 3,
    systemInput->mV.mX + 3,
    systemInput->mD.mX,
    daeMethodOutput->mZC.mX,
    &errorResult,
    neDiagMgr);
}

static
  PmfMessageId dae_cg_project_solve(const NeDae *dae,
  const NeSystemInput *systemInput,
  NeuDiagnosticManager *neDiagMgr)
{
  NeDaePrivateData *smData = dae->mPrivateData;
  return
    sm_core_projectState(
    false,
    &smData->mSimulationDelegate,
    &smData->mSimRuntimeDerivedValuesBundle,
    systemInput->mM.mX,
    systemInput->mU.mX,
    systemInput->mU.mX + 3,
    systemInput->mD.mX,
    systemInput->mX.mX, neDiagMgr);
}

static
  PmfMessageId dae_cg_check_solve(const NeDae *dae,
  const NeSystemInput *systemInput,
  NeuDiagnosticManager *neDiagMgr)
{
  NeDaePrivateData *smData = dae->mPrivateData;
  PmfMessageId errorId = NULL;
  if (smData->mNumConstraintEqns > 0)
    errorId = sm_core_projectState(
      false,
      &smData->mSimulationDelegate,
      &smData->mSimRuntimeDerivedValuesBundle,
      systemInput->mM.mX,
      systemInput->mU.mX,
      systemInput->mU.mX + 3,
      systemInput->mD.mX,
      systemInput->mX.mX, neDiagMgr);
  if (errorId == NULL && smData->mDoCheckDynamics) {
    double result = 0.0;
    errorId = human9DOF_836bb176_1_checkDynamics(
      &smData->mSimRuntimeDerivedValuesBundle,
      systemInput->mX.mX,
      systemInput->mU.mX,
      systemInput->mU.mX + 3,
      systemInput->mV.mX + 3,
      systemInput->mD.mX,
      &result, neDiagMgr);
  }

  return errorId;
}

static
  PmfMessageId dae_cg_CIC_MODE_solve(const NeDae *dae,
  const NeSystemInput *systemInput,
  NeuDiagnosticManager *neDiagMgr)
{
  NeDaePrivateData *smData = dae->mPrivateData;
  PmfMessageId errorId = NULL;
  double errorResult = 0.0;
  const size_t mvSize = smData->mModeVectorSize;
  boolean_T modeChanged = false;
  if (mvSize > 0) {
    errorId = human9DOF_836bb176_1_computeSimModeVector(
      systemInput->mU.mX,
      systemInput->mU.mX + 3,
      systemInput->mV.mX + 3,
      systemInput->mM.mX,
      &errorResult,
      neDiagMgr);
    if (errorId != NULL)
      return errorId;

    {
      size_t i;
      for (i = 0; i < mvSize; ++i)
        if (systemInput->mM.mX[i] != smData->mCachedModeVector.mX[i]) {
          modeChanged = true;
          break;
        }
    }
  }

  if (modeChanged) {
    errorId = sm_core_onModeChanged(
      &smData->mSimulationDelegate,
      &smData->mSimRuntimeDerivedValuesBundle,
      systemInput->mU.mX,
      systemInput->mU.mX + 3,
      systemInput->mD.mX,
      smData->mCachedModeVector.mX,
      systemInput->mM.mX,
      systemInput->mX.mX,
      neDiagMgr);
    if (errorId != NULL)
      return errorId;
    memcpy(smData->mCachedModeVector.mX, systemInput->mM.mX,
           0 * sizeof(int32_T));
  }

  errorId =
    sm_core_projectState(
    true,
    &smData->mSimulationDelegate,
    &smData->mSimRuntimeDerivedValuesBundle,
    systemInput->mM.mX,
    systemInput->mU.mX,
    systemInput->mU.mX + 3,
    systemInput->mD.mX,
    systemInput->mX.mX, neDiagMgr);
  return errorId;
}

static
  PmfMessageId dae_cg_assemble_solve(const NeDae *dae,
  const NeSystemInput *systemInput,
  NeuDiagnosticManager *neDiagMgr)
{
  NeDaePrivateData *smData = dae->mPrivateData;
  const SmMechanismDelegate *delegate = &smData->mAssemblyDelegate;
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle =
    &smData->mAsmRuntimeDerivedValuesBundle;
  PmfMessageId errorId = NULL;
  size_t i;
  double errorResult = 0.0;
  const size_t numTargets = 20;
  unsigned int asmStatus = 0;
  double *assemblyFullStateVector = smData->mAssemblyFullStateVector.mX;
  double *simulationFullStateVector = smData->mSimulationFullStateVector.mX;
  (*delegate->mSetTargets)(runtimeDerivedValuesBundle, smData->mTargets);

  {
    const double *u = systemInput->mU.mX;
    const double *uDot = u + smData->mInputVectorSize;
    CTarget *target = smData->mTargets + smData->mNumInternalTargets;
    for (i = 0; i < smData->mNumInputMotionPrimitives; ++i) {
      const size_t inputOffset = smData->mMotionInputOffsets.mX[i];
      (target++)->mValue[0] = u [inputOffset];
      (target++)->mValue[0] = uDot[inputOffset];
    }
  }

  if (smData->mAssemblyModeVector.mN > 0) {
    errorId = human9DOF_836bb176_1_computeAsmModeVector(
      systemInput->mU.mX,
      systemInput->mU.mX + 3,
      systemInput->mV.mX + 3,
      smData->mAssemblyModeVector.mX,
      &errorResult,
      neDiagMgr);
    if (errorId != NULL)
      return errorId;
  }

  errorId = sm_core_computeStateVector(
    delegate, runtimeDerivedValuesBundle, smData->mAssemblyModeVector.mX,
    numTargets, smData->mTargets,
    assemblyFullStateVector, neDiagMgr);
  if (errorId != NULL)
    return errorId;
  asmStatus = sm_core_checkAssembly(
    delegate, runtimeDerivedValuesBundle, assemblyFullStateVector,
    smData->mAssemblyModeVector.mX,
    NULL, NULL, NULL);
  if (asmStatus != 1) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:AssemblyFailure",
      asmStatus == 2 ?
      "Model not assembled due to a position violation. The failure occurred during the attempt to assemble all joints in the system and satisfy any motion inputs. If an Update Diagram operation completes successfully, the failure is likely caused by motion inputs. Consider adjusting the motion inputs to specify a different starting configuration. Also consider adjusting or adding joint targets to better guide the assembly."
      :
      (asmStatus == 3 ?
       "Model not assembled due to a velocity violation. The failure occurred during the attempt to assemble all joints in the system and satisfy any motion inputs. If an Update Diagram operation completes successfully, the failure is likely caused by motion inputs. Consider adjusting the motion inputs to specify a different starting configuration. Also consider adjusting or adding joint targets to better guide the assembly."
       :
       "Model not assembled due to a singularity violation. The failure occurred during the attempt to assemble all joints in the system and satisfy any motion inputs. If an Update Diagram operation completes successfully, the failure is likely caused by motion inputs. Consider adjusting the motion inputs to specify a different starting configuration. Also consider adjusting or adding joint targets to better guide the assembly."),
      neDiagMgr);
  }

#if 0

  human9DOF_836bb176_1_checkTargets(
    &smData->mSimRuntimeDerivedValuesBundle,
    assemblyFullStateVector);

#endif

  if (smData->mModeVectorSize > 0) {
    errorId = human9DOF_836bb176_1_computeSimModeVector(
      systemInput->mU.mX,
      systemInput->mU.mX + 3,
      systemInput->mV.mX + 3,
      systemInput->mM.mX,
      &errorResult,
      neDiagMgr);
    if (errorId != NULL)
      return errorId;
    memcpy(smData->mCachedModeVector.mX, systemInput->mM.mX,
           0 * sizeof(int32_T));
  }

  errorId =
    (*delegate->mConvertStateVector)(
    NULL, runtimeDerivedValuesBundle, NULL, assemblyFullStateVector,
    smData->mAssemblyModeVector.mX, systemInput->mM.mX,
    simulationFullStateVector, neDiagMgr);
  for (i = 0; i < smData->mStateVectorSize; ++i)
    systemInput->mX.mX[i] = simulationFullStateVector[smData->
      mStateVectorMap.mX[i]];
  memcpy(systemInput->mD.mX,
         simulationFullStateVector +
         smData->mFullStateVectorSize - smData->mDiscreteStateSize,
         smData->mDiscreteStateSize * sizeof(double));
  return errorId;
}

typedef struct {
  size_t first;
  size_t second;
} SizePair;

static void checkMemAllocStatus(int_T status)
{
  (void) status;
}

static
  PmCharVector cStringToCharVector(const char *src)
{
  const size_t n = strlen(src);
  PmCharVector charVect;
  const int_T status =
    pm_create_char_vector_fields(&charVect, n + 1, pm_default_allocator());
  checkMemAllocStatus(status);
  strcpy(charVect.mX, src);
  return charVect;
}

static
  void initBasicAttributes(NeDaePrivateData *smData)
{
  size_t i;
  smData->mStateVectorSize = 20;
  smData->mFullStateVectorSize = 20;
  smData->mDiscreteStateSize = 0;
  smData->mModeVectorSize = 0;
  smData->mNumZeroCrossings = 0;
  smData->mInputVectorSize = 3;
  smData->mOutputVectorSize = 27;
  smData->mNumConstraintEqns = 0;
  smData->mDoCheckDynamics = true;
  for (i = 0; i < 4; ++i)
    smData->mChecksum[i] = 0;
}

static
  void initStateVector(NeDaePrivateData *smData)
{
  PmAllocator *alloc = pm_default_allocator();
  const int32_T stateVectorMap[20] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
    10, 11, 12, 13, 14, 15, 16, 17, 18, 19
  };

  const CTarget targets[20] = {
    { 0, 41, 0, false, 0, 0, "1", false, true, +1.000000000000000000e+00, true,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 41, 0, false, 0, 2, "deg/s", true, true, +1.000000000000000000e+00,
      true, 1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 44, 0, false, 0, 0, "1", false, true, +1.000000000000000000e+00, true,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 44, 0, false, 0, 2, "deg/s", true, true, +1.000000000000000000e+00,
      true, 1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 49, 0, false, 0, 0, "1", false, true, +1.000000000000000000e+00, true,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 49, 0, false, 0, 0, "1", true, true, +1.000000000000000000e+00, true, 1,
      { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 74, 0, false, 0, 0, "1", false, true, +1.000000000000000000e+00, true,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 74, 0, false, 0, 2, "deg/s", true, true, +1.000000000000000000e+00,
      true, 1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 75, 0, false, 0, 0, "1", false, true, +1.000000000000000000e+00, true,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 75, 0, false, 0, 2, "deg/s", true, true, +1.000000000000000000e+00,
      true, 1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 77, 0, false, 0, 0, "1", false, true, +1.000000000000000000e+00, true,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 77, 0, false, 0, 2, "deg/s", true, true, +1.000000000000000000e+00,
      true, 1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 87, 0, false, 0, 0, "1", false, true, +1.000000000000000000e+00, true,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 87, 0, false, 0, 2, "deg/s", true, true, +1.000000000000000000e+00,
      true, 1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 88, 0, false, 0, 0, "1", false, true, +1.000000000000000000e+00, true,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 88, 0, false, 0, 2, "deg/s", true, true, +1.000000000000000000e+00,
      true, 1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 89, 0, false, 0, 0, "1", false, true, +1.000000000000000000e+00, true,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 89, 0, false, 0, 0, "1", true, true, +1.000000000000000000e+00, true, 1,
      { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 90, 0, false, 0, 0, "1", false, true, +1.000000000000000000e+00, true,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 90, 0, false, 0, 2, "deg/s", true, true, +1.000000000000000000e+00,
      true, 1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } }
  };

  const size_t numTargets = 20;
  int_T status;
  size_t i;
  status = pm_create_real_vector_fields(
    &smData->mAssemblyFullStateVector, 20, alloc);
  checkMemAllocStatus(status);
  status = pm_create_real_vector_fields(
    &smData->mSimulationFullStateVector, 20, alloc);
  checkMemAllocStatus(status);
  status = pm_create_int_vector_fields(
    &smData->mStateVectorMap, smData->mStateVectorSize, alloc);
  checkMemAllocStatus(status);
  memcpy(smData->mStateVectorMap.mX, stateVectorMap,
         smData->mStateVectorSize * sizeof(int32_T));
  smData->mNumInternalTargets = 20;
  smData->mNumInputMotionPrimitives = 0;
  PM_ALLOCATE_ARRAY(smData->mTargets, CTarget, numTargets, alloc);
  for (i = 0; i < numTargets; ++i)
    sm_compiler_CTarget_copy(targets + i, smData->mTargets + i);
}

static void initAsserts(NeDaePrivateData *smData)
{
  PmAllocator *alloc = pm_default_allocator();
  int_T status = 0;
  smData->mNumParamAsserts = 42;
  smData->mParamAssertObjects = NULL;
  smData->mParamAssertPaths = NULL;
  smData->mParamAssertDescriptors = NULL;
  smData->mParamAssertMessages = NULL;
  smData->mParamAssertMessageIds = NULL;
  status = pm_create_bool_vector_fields(
    &smData->mParamAssertIsWarnings, smData->mNumParamAsserts, alloc);
  checkMemAllocStatus(status);
  if (smData->mNumParamAsserts > 0) {
    const NeAssertData *ad = human9DOF_836bb176_1_assertData;
    size_t i;
    PM_ALLOCATE_ARRAY(smData->mParamAssertObjects,
                      PmCharVector, 42, alloc);
    PM_ALLOCATE_ARRAY(smData->mParamAssertPaths,
                      PmCharVector, 42, alloc);
    PM_ALLOCATE_ARRAY(smData->mParamAssertDescriptors,
                      PmCharVector, 42, alloc);
    PM_ALLOCATE_ARRAY(smData->mParamAssertMessages,
                      PmCharVector, 42, alloc);
    PM_ALLOCATE_ARRAY(smData->mParamAssertMessageIds,
                      PmCharVector, 42, alloc);
    for (i = 0; i < smData->mNumParamAsserts; ++i, ++ad) {
      smData->mParamAssertObjects [i] = cStringToCharVector(ad->mObject );
      smData->mParamAssertPaths [i] = cStringToCharVector(ad->mPath );
      smData->mParamAssertDescriptors[i] = cStringToCharVector(ad->mDescriptor);
      smData->mParamAssertMessages [i] = cStringToCharVector(ad->mMessage );
      smData->mParamAssertMessageIds [i] = cStringToCharVector(ad->mMessageID );
      smData->mParamAssertIsWarnings.mX[i] = ad->mIsWarn;
    }
  }
}

static
  void initModeVector(NeDaePrivateData *smData)
{
  {
    size_t i;
    const int_T status = pm_create_int_vector_fields(
      &smData->mAssemblyModeVector, 0,
      pm_default_allocator());
    checkMemAllocStatus(status);
    for (i = 0; i < smData->mAssemblyModeVector.mN; ++i)
      smData->mAssemblyModeVector.mX[i] = 0;
  }

  {
    size_t i;
    const int_T status = pm_create_int_vector_fields(
      &smData->mCachedModeVector, 0, pm_default_allocator());
    checkMemAllocStatus(status);
    for (i = 0; i < smData->mModeVectorSize; ++i)
      smData->mCachedModeVector.mX[i] = 0;
  }
}

static void initZeroCrossings(NeDaePrivateData *smData)
{
  PmAllocator *alloc = pm_default_allocator();
  int_T status = 0;
  smData->mZeroCrossingObjects = NULL;
  smData->mZeroCrossingPaths = NULL;
  smData->mZeroCrossingDescriptors = NULL;
  status = pm_create_int_vector_fields(
    &smData->mZeroCrossingTypes, 0, alloc);
  checkMemAllocStatus(status);
  if (smData->mNumZeroCrossings > 0) {
    const NeZCData *zcd = human9DOF_836bb176_1_ZCData;
    size_t i;
    PM_ALLOCATE_ARRAY(smData->mZeroCrossingObjects,
                      PmCharVector, 0, alloc);
    PM_ALLOCATE_ARRAY(smData->mZeroCrossingPaths,
                      PmCharVector, 0, alloc);
    PM_ALLOCATE_ARRAY(smData->mZeroCrossingDescriptors,
                      PmCharVector, 0, alloc);
    for (i = 0; i < smData->mNumZeroCrossings; ++i, ++zcd) {
      smData->mZeroCrossingObjects [i] = cStringToCharVector(zcd->mObject);
      smData->mZeroCrossingPaths [i] = cStringToCharVector(zcd->mPath );
      smData->mZeroCrossingDescriptors[i] = cStringToCharVector(zcd->mDescriptor);
      smData->mZeroCrossingTypes.mX[i] = zcd->mType;
    }
  }
}

static
  void initVariables(NeDaePrivateData *smData)
{
  const char *varFullPaths[20] = {
    "spine_twist.Rz.q",
    "spine_twist.Rz.w",
    "hipside.Rz.q",
    "hipside.Rz.w",
    "HipsForward.Rz.q",
    "HipsForward.Rz.w",
    "Chicken_Wing.Rz.q",
    "Chicken_Wing.Rz.w",
    "butterfly.Rz.q",
    "butterfly.Rz.w",
    "curl.Rz.q",
    "curl.Rz.w",
    "j6.Rz.q",
    "j6.Rz.w",
    "j7.Rz.q",
    "j7.Rz.w",
    "j8.Rz.q",
    "j8.Rz.w",
    "Revolute_Joint.Rz.q",
    "Revolute_Joint.Rz.w"
  };

  const char *varObjects[20] = {
    "human9DOF/spine twist",
    "human9DOF/spine twist",
    "human9DOF/hipside",
    "human9DOF/hipside",
    "human9DOF/HipsForward",
    "human9DOF/HipsForward",
    "human9DOF/Chicken Wing",
    "human9DOF/Chicken Wing",
    "human9DOF/butterfly",
    "human9DOF/butterfly",
    "human9DOF/curl",
    "human9DOF/curl",
    "human9DOF/j6",
    "human9DOF/j6",
    "human9DOF/j7",
    "human9DOF/j7",
    "human9DOF/j8",
    "human9DOF/j8",
    "human9DOF/Revolute Joint",
    "human9DOF/Revolute Joint"
  };

  smData->mNumVarScalars = 20;
  smData->mVarFullPaths = NULL;
  smData->mVarObjects = NULL;
  if (smData->mNumVarScalars > 0) {
    size_t s;
    PmAllocator *alloc = pm_default_allocator();
    PM_ALLOCATE_ARRAY(smData->mVarFullPaths, PmCharVector, 20, alloc);
    PM_ALLOCATE_ARRAY(smData->mVarObjects, PmCharVector, 20, alloc);
    for (s = 0; s < smData->mNumVarScalars; ++s) {
      smData->mVarFullPaths[s] = cStringToCharVector(varFullPaths[s]);
      smData->mVarObjects[s] = cStringToCharVector(varObjects[s]);
    }
  }
}

static
  void initRuntimeParameters(NeDaePrivateData *smData)
{
  PmAllocator *alloc = pm_default_allocator();
  int_T status = 0;
  size_t i = 0;
  const int32_T rtpRootVarRows[19] = {
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1
  };

  const int32_T rtpRootVarCols[19] = {
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1
  };

  const char *rtpFullPaths [19] = {
    "RTP_37C73AD1_LowerLimitBound",
    "RTP_37C73AD1_UpperLimitBound",
    "RTP_37C73AD1_VelocityTargetValue",
    "RTP_3BFD495C_RotationAngle",
    "RTP_3C908D45_RotationAngle",
    "RTP_3E0457AF_LowerLimitBound",
    "RTP_4B97BDD3_RotationAngle",
    "RTP_4CFA79CA_RotationAngle",
    "RTP_A2F418E6_RotationAngle",
    "RTP_AC370233_LowerLimitBound",
    "RTP_AC370233_UpperLimitBound",
    "RTP_AC370233_VelocityTargetValue",
    "RTP_BDAECBDC_RotationAngle",
    "RTP_CAA9FB4A_RotationAngle",
    "RTP_D5F32870_RotationAngle",
    "RTP_DB3032A5_LowerLimitBound",
    "RTP_DB3032A5_UpperLimitBound",
    "RTP_DB3032A5_VelocityTargetValue",
    "RTP_E22C841E_VelocityTargetValue"
  };

  smData->mNumRtpRootVars = 19;
  status = pm_create_int_vector_fields(
    &smData->mRtpRootVarRows, smData->mNumRtpRootVars, alloc);
  checkMemAllocStatus(status);
  memcpy(smData->mRtpRootVarRows.mX, rtpRootVarRows,
         smData->mNumRtpRootVars * sizeof(int32_T));
  status = pm_create_int_vector_fields(
    &smData->mRtpRootVarCols, smData->mNumRtpRootVars, alloc);
  checkMemAllocStatus(status);
  memcpy(smData->mRtpRootVarCols.mX, rtpRootVarCols,
         smData->mNumRtpRootVars * sizeof(int32_T));
  smData->mRtpFullPaths = NULL;
  if (smData->mNumRtpRootVars > 0) {
    size_t v;
    PM_ALLOCATE_ARRAY(smData->mRtpFullPaths, PmCharVector, 19, alloc);
    for (v = 0; v < smData->mNumRtpRootVars; ++v) {
      smData->mRtpFullPaths[v] = cStringToCharVector(rtpFullPaths[v]);
    }
  }

  smData->mNumRuntimeRootVarScalars = 19;
  status = pm_create_real_vector_fields(
    &smData->mRuntimeParameterScalars, 19,
    alloc);
  checkMemAllocStatus(status);
  for (i = 0; i < smData->mRuntimeParameterScalars.mN; ++i)
    smData->mRuntimeParameterScalars.mX[i] = 0.0;
  sm_core_RuntimeDerivedValuesBundle_create(
    &smData->mAsmRuntimeDerivedValuesBundle,
    105,
    0);
  sm_core_RuntimeDerivedValuesBundle_create(
    &smData->mSimRuntimeDerivedValuesBundle,
    97,
    0);
}

static
  void initIoInfoHelper(
  size_t n,
  const char *portPathsSource[],
  const char *unitsSource[],
  const SizePair dimensions[],
  boolean_T doInputs,
  NeDaePrivateData *smData)
{
  PmCharVector *portPaths = NULL;
  PmCharVector *units = NULL;
  NeDsIoInfo *infos = NULL;
  if (n > 0) {
    size_t s;
    PmAllocator *alloc = pm_default_allocator();
    PM_ALLOCATE_ARRAY(portPaths, PmCharVector, n, alloc);
    PM_ALLOCATE_ARRAY(units, PmCharVector, n, alloc);
    PM_ALLOCATE_ARRAY(infos, NeDsIoInfo, n, alloc);
    for (s = 0; s < n; ++s) {
      portPaths[s] = cStringToCharVector(portPathsSource[s]);
      units[s] = cStringToCharVector(unitsSource[s]);

      {
        NeDsIoInfo *info = infos + s;
        info->mName = info->mIdentifier = portPaths[s].mX;
        info->mM = dimensions[s].first;
        info->mN = dimensions[s].second;
        info->mUnit = units[s].mX;
      }
    }
  }

  if (doInputs) {
    smData->mNumInputs = n;
    smData->mInputPortPaths = portPaths;
    smData->mInputUnits = units;
    smData->mInputInfos = infos;
  } else {
    smData->mNumOutputs = n;
    smData->mOutputPortPaths = portPaths;
    smData->mOutputUnits = units;
    smData->mOutputInfos = infos;
  }
}

static
  void initIoInfo(NeDaePrivateData *smData)
{
  const char *inputPortPaths[3] = {
    "External_Force_and_Torque.fx",
    "External_Force_and_Torque.fy",
    "External_Force_and_Torque.fz"
  };

  const char *inputUnits[3] = {
    "m*kg/s^2",
    "m*kg/s^2",
    "m*kg/s^2"
  };

  const SizePair inputDimensions[3] = {
    { 1, 1 }, { 1, 1 }, { 1, 1 }
  };

  const char *outputPortPaths[25] = {
    "Chicken_Wing.q",
    "Chicken_Wing.w",
    "HipsForward.q",
    "HipsForward.w",
    "butterfly.q",
    "butterfly.w",
    "curl.q",
    "curl.w",
    "hipside.q",
    "hipside.w",
    "j6.q",
    "j6.w",
    "j7.q",
    "j7.w",
    "j8.q",
    "j8.w",
    "spine_twist.q",
    "spine_twist.w",
    "Transform_Sensor.axs",
    "Transform_Sensor.x",
    "Transform_Sensor.y",
    "Transform_Sensor.z",
    "Transform_Sensor.ax",
    "Transform_Sensor.ay",
    "Transform_Sensor.az"
  };

  const char *outputUnits[25] = {
    "rad",
    "rad/s",
    "rad",
    "rad/s",
    "rad",
    "rad/s",
    "rad",
    "rad/s",
    "rad",
    "rad/s",
    "rad",
    "rad/s",
    "rad",
    "rad/s",
    "rad",
    "rad/s",
    "rad",
    "rad/s",
    "1",
    "m",
    "m",
    "m",
    "m/s^2",
    "m/s^2",
    "m/s^2"
  };

  const SizePair outputDimensions[25] = {
    { 1, 1 }, { 1, 1 }, { 1, 1 }, { 1, 1 },

    { 1, 1 }, { 1, 1 }, { 1, 1 }, { 1, 1 },

    { 1, 1 }, { 1, 1 }, { 1, 1 }, { 1, 1 },

    { 1, 1 }, { 1, 1 }, { 1, 1 }, { 1, 1 },

    { 1, 1 }, { 1, 1 }, { 3, 1 }, { 1, 1 },

    { 1, 1 }, { 1, 1 }, { 1, 1 }, { 1, 1 },

    { 1, 1 }
  };

  initIoInfoHelper(3, inputPortPaths, inputUnits, inputDimensions,
                   true, smData);
  initIoInfoHelper(25, outputPortPaths, outputUnits, outputDimensions,
                   false, smData);
}

static
  void initInputDerivs(NeDaePrivateData *smData)
{
  const int32_T numInputDerivs[3] = {
    0, 0, 0
  };

  PmAllocator *alloc = pm_default_allocator();
  const int_T status = pm_create_int_vector_fields(
    &smData->mNumInputDerivs, smData->mInputVectorSize, alloc);
  checkMemAllocStatus(status);
  memcpy(smData->mNumInputDerivs.mX, numInputDerivs,
         3 * sizeof(int32_T));
  smData->mInputOrder = 1;
}

static
  void initDirectFeedthrough(NeDaePrivateData *smData)
{
  const boolean_T directFeedthroughVector[3] = {
    false, false, false
  };

  const boolean_T directFeedthroughMatrix[81] = {
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, true, true, true, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, true, true, true, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, true, true,
    true
  };

  PmAllocator *alloc = pm_default_allocator();

  {
    const int_T status = pm_create_bool_vector_fields(
      &smData->mDirectFeedthroughVector, 3, alloc);
    checkMemAllocStatus(status);
    memcpy(smData->mDirectFeedthroughVector.mX, directFeedthroughVector,
           3 * sizeof(boolean_T));
  }

  {
    const int_T status = pm_create_bool_vector_fields(
      &smData->mDirectFeedthroughMatrix, 81, alloc);
    checkMemAllocStatus(status);
    memcpy(smData->mDirectFeedthroughMatrix.mX, directFeedthroughMatrix,
           81 * sizeof(boolean_T));
  }
}

static
  void initOutputDerivProc(NeDaePrivateData *smData)
{
  PmAllocator *alloc = pm_default_allocator();
  const int32_T outputFunctionMap[27] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 1, 1
  };

  smData->mOutputFunctionMap = pm_create_int_vector(27, alloc);
  memcpy(smData->mOutputFunctionMap->mX, outputFunctionMap,
         27 * sizeof(int32_T));
  smData->mNumOutputClasses = 2;
  smData->mHasKinematicOutputs = true;
  smData->mHasDynamicOutputs = true;
  smData->mIsOutputClass0Dynamic = false;
  smData->mDoComputeDynamicOutputs = false;
  smData->mCachedDerivativesAvailable = false;

  {
    size_t i = 0;
    const int_T status = pm_create_real_vector_fields(
      &smData->mCachedDerivatives, 20, pm_default_allocator());
    checkMemAllocStatus(status);
    for (i = 0; i < smData->mCachedDerivatives.mN; ++i)
      smData->mCachedDerivatives.mX[i] = 0.0;
  }
}

#if 0

static void initializeSizePairVector(const SmSizePair *data,
  SmSizePairVector *vector)
{
  const size_t n = sm_core_SmSizePairVector_size(vector);
  size_t i;
  for (i = 0; i < n; ++i, ++data)
    sm_core_SmSizePairVector_setValue(vector, i, data++);
}

#endif

static
  void initAssemblyDelegate(SmMechanismDelegate *delegate)
{
  SmMechanismDelegateScratchpad *scratchpad = NULL;
  const SmSizePair jointToStageIdx[10] = {
    { 41, 3 }, { 44, 2 }, { 49, 9 }, { 74, 4 }, { 75, 5 }, { 77, 1 },

    { 87, 6 }, { 88, 7 }, { 89, 8 }, { 90, 0 }
  };

  const size_t primitiveIndices[10 + 1] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
    10
  };

  const SmSizePair stateOffsets[10] = {
    { 0, 1 }, { 2, 3 }, { 4, 5 }, { 6, 7 }, { 8, 9 }, { 10, 11 },

    { 12, 13 }, { 14, 15 }, { 16, 17 }, { 18, 19 }
  };

  const SmSizePair dofOffsets[10] = {
    { 0, 1 }, { 1, 2 }, { 2, 3 }, { 3, 4 }, { 4, 5 }, { 5, 6 },

    { 6, 7 }, { 7, 8 }, { 8, 9 }, { 9, 10 }
  };

  const size_t *flexibleStages = NULL;
  const size_t remodIndices[10] = {
    0, 2, 4, 6, 8, 10, 12, 14, 16, 18
  };

  const size_t *equationsPerConstraint = NULL;
  const size_t dofToVelSlot[10] = {
    1, 3, 5, 7, 9, 11, 13, 15, 17, 19
  };

  const size_t *constraintDofs = NULL;
  const size_t constraintDofOffsets[0 + 1] = {
    0
  };

  const size_t Jm = 0;
  const size_t Jn = 10;
  SmSizePair zeroSizePair;
  zeroSizePair.mFirst = zeroSizePair.mSecond = 0;
  sm_core_MechanismDelegate_allocScratchpad(delegate);
  scratchpad = delegate->mScratchpad;
  delegate->mTargetStrengthFree = 0;
  delegate->mTargetStrengthSuggested = 1;
  delegate->mTargetStrengthDesired = 2;
  delegate->mTargetStrengthRequired = 3;
  delegate->mConsistencyTol = +1.000000000000000062e-09;
  delegate->mTreeJointDof = 10;
  delegate->mDof = 10;
  delegate->mStateSize = 20;
  delegate->mContinuousStateSize = 20;
  delegate->mModeVectorSize = 0;
  delegate->mNumStages = 10;
  delegate->mNumConstraints = 0;
  delegate->mNumAllConstraintEquations = 0;
  sm_core_SmSizePairVector_create(
    &delegate->mJointToStageIdx, 10, &zeroSizePair);
  memcpy(sm_core_SmSizePairVector_nonConstValues(&delegate->mJointToStageIdx),
         jointToStageIdx, 10 * sizeof(SmSizePair));
  sm_core_SmSizeTVector_create(
    &delegate->mPrimitiveIndices, delegate->mNumStages + 1, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mPrimitiveIndices),
         primitiveIndices, (delegate->mNumStages + 1) * sizeof(size_t));
  sm_core_SmSizePairVector_create(
    &delegate->mStateOffsets, 10, &zeroSizePair);
  memcpy(sm_core_SmSizePairVector_nonConstValues(&delegate->mStateOffsets),
         stateOffsets, 10 * sizeof(SmSizePair));
  sm_core_SmSizePairVector_create(
    &delegate->mDofOffsets, 10, &zeroSizePair);
  memcpy(sm_core_SmSizePairVector_nonConstValues(&delegate->mDofOffsets),
         dofOffsets, 10 * sizeof(SmSizePair));
  sm_core_SmSizeTVector_create(
    &delegate->mFlexibleStages, 0, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mFlexibleStages),
         flexibleStages, 0 * sizeof(size_t));
  sm_core_SmSizeTVector_create(
    &delegate->mRemodIndices, 10, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mRemodIndices),
         remodIndices, 10 * sizeof(size_t));
  sm_core_SmSizeTVector_create(
    &delegate->mEquationsPerConstraint, delegate->mNumConstraints, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mEquationsPerConstraint),
         equationsPerConstraint, delegate->mNumConstraints * sizeof(size_t));
  sm_core_SmIntVector_create(
    &delegate->mRunTimeEnabledEquations,
    delegate->mNumAllConstraintEquations, 1);
  sm_core_SmSizeTVector_create(
    &delegate->mDofToVelSlot, delegate->mDof, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mDofToVelSlot),
         dofToVelSlot, delegate->mDof * sizeof(size_t));
  sm_core_SmSizeTVector_create(
    &delegate->mConstraintDofs, 0, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mConstraintDofs),
         constraintDofs, 0 * sizeof(size_t));
  sm_core_SmSizeTVector_create(
    &delegate->mConstraintDofOffsets, delegate->mNumConstraints + 1, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mConstraintDofOffsets),
         constraintDofOffsets, (delegate->mNumConstraints + 1) * sizeof(size_t));
  sm_core_SmBoundedSet_create(&scratchpad->mPosRequired, 10);
  sm_core_SmBoundedSet_create(&scratchpad->mPosDesired, 10);
  sm_core_SmBoundedSet_create(&scratchpad->mPosSuggested, 10);
  sm_core_SmBoundedSet_create(&scratchpad->mPosFree, 10);
  sm_core_SmBoundedSet_create(&scratchpad->mPosNonRequired, 10);
  sm_core_SmBoundedSet_create(&scratchpad->mPosSuggAndFree, 10);
  sm_core_SmBoundedSet_create(&scratchpad->mVelRequired, 10);
  sm_core_SmBoundedSet_create(&scratchpad->mVelDesired, 10);
  sm_core_SmBoundedSet_create(&scratchpad->mVelSuggested, 10);
  sm_core_SmBoundedSet_create(&scratchpad->mVelFree, 10);
  sm_core_SmBoundedSet_create(&scratchpad->mVelNonRequired, 10);
  sm_core_SmBoundedSet_create(&scratchpad->mVelSuggAndFree, 10);
  sm_core_SmBoundedSet_create(&scratchpad->mConstraintFilter, 0);
  sm_core_SmBoundedSet_create(&scratchpad->mActiveConstraints, 0);
  sm_core_SmBoundedSet_create(&scratchpad->mActiveDofs, 10);
  sm_core_SmBoundedSet_create(&scratchpad->mActiveDofs0, 10);
  sm_core_SmBoundedSet_create(&scratchpad->mNewConstraints, 0);
  sm_core_SmBoundedSet_create(&scratchpad->mNewDofs, 10);
  sm_core_SmBoundedSet_create(&scratchpad->mUnsatisfiedConstraints, 0);
  sm_core_SmSizeTVector_create(&scratchpad->mActiveConstraintsVect,
    0, 0);
  sm_core_SmSizeTVector_create(&scratchpad->mActiveDofsVect, 10, 0);
  sm_core_SmSizeTVector_create(&scratchpad->mFullDofToActiveDof, 10, 0);
  sm_core_SmSizePairVector_create(
    &scratchpad->mPartiallyPosTargetedPrims, 10, &zeroSizePair);
  sm_core_SmSizePairVector_create(
    &scratchpad->mPartiallyVelTargetedPrims, 10, &zeroSizePair);
  sm_core_SmSizeTVector_create(&scratchpad->mPosPartialTypes, 10, 0);
  sm_core_SmSizeTVector_create(&scratchpad->mVelPartialTypes, 10, 0);
  sm_core_SmSizeTVector_create(&scratchpad->mPartiallyActivePrims, 10, 0);
  sm_core_SmSizePairVector_create(
    &scratchpad->mBaseFrameVelOffsets, 0, &zeroSizePair);
  sm_core_SmSizePairVector_create(
    &scratchpad->mCvVelOffsets, 10, &zeroSizePair);
  sm_core_SmRealVector_create(
    &scratchpad->mCvAzimuthValues, 10, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mInitialState, 20, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mStartState, 20, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mTestState, 20, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mFullStateVector, 20, 0.0);
  sm_core_SmIntVector_create(&scratchpad->mModeVector, 0, 0);
  sm_core_SmRealVector_create(&scratchpad->mJacobianRowMaj, Jm * Jn, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mJacobian, Jm * Jn, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mJacobianPrimSubmatrix, Jm * 6, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mConstraintNonhomoTerms, Jm, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mConstraintError, Jm, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mBestConstraintError, Jm, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mDeltas,
    Jn * (Jm <= Jn ? Jm : Jn), 0.0);
  sm_core_SmRealVector_create(&scratchpad->mSvdWork, 121, 0.0);
  sm_core_SmRealVector_create(
    &scratchpad->mLineSearchScaledDeltaVect, 10, 0.0);
  sm_core_SmRealVector_create(
    &scratchpad->mLineSearchTestStateVect, 20, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mLineSearchErrorVect, Jm, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mActiveDofVelsVect, 10, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mVelSystemRhs, Jm, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mMotionData, 186, 0.0);
  delegate->mSetTargets = human9DOF_836bb176_1_setTargets;
  delegate->mResetStateVector = human9DOF_836bb176_1_resetAsmStateVector;
  delegate->mInitializeTrackedAngleState =
    human9DOF_836bb176_1_initializeTrackedAngleState;
  delegate->mComputeDiscreteState = human9DOF_836bb176_1_computeDiscreteState;
  delegate->mAdjustPosition = human9DOF_836bb176_1_adjustPosition;
  delegate->mPerturbJointPrimitiveState =
    human9DOF_836bb176_1_perturbAsmJointPrimitiveState;
  delegate->mPerturbFlexibleBodyState = NULL;
  delegate->mComputePosDofBlendMatrix =
    human9DOF_836bb176_1_computePosDofBlendMatrix;
  delegate->mComputeVelDofBlendMatrix =
    human9DOF_836bb176_1_computeVelDofBlendMatrix;
  delegate->mProjectPartiallyTargetedPos =
    human9DOF_836bb176_1_projectPartiallyTargetedPos;
  delegate->mPropagateMotion = human9DOF_836bb176_1_propagateMotion;
  delegate->mComputeAssemblyError = human9DOF_836bb176_1_computeAssemblyError;
  delegate->mComputeAssemblyJacobian =
    human9DOF_836bb176_1_computeAssemblyJacobian;
  delegate->mComputeFullAssemblyJacobian =
    human9DOF_836bb176_1_computeFullAssemblyJacobian;
  delegate->mIsInKinematicSingularity =
    human9DOF_836bb176_1_isInKinematicSingularity;
  delegate->mConvertStateVector = human9DOF_836bb176_1_convertStateVector;
  delegate->mConstructStateVector = NULL;
  delegate->mExtractSolverStateVector = NULL;
  delegate->mIsPositionViolation = NULL;
  delegate->mIsVelocityViolation = NULL;
  delegate->mProjectStateSim = NULL;
  delegate->mComputeConstraintError = NULL;
  delegate->mResetModeVector = NULL;
  delegate->mHasJointDisToNormModeChange = NULL;
  delegate->mPerformJointDisToNormModeChange = NULL;
  delegate->mOnModeChangedCutJoints = NULL;
  delegate->mMech = NULL;
}

static
  void initSimulationDelegate(SmMechanismDelegate *delegate)
{
  SmMechanismDelegateScratchpad *scratchpad = NULL;
  const SmSizePair jointToStageIdx[10] = {
    { 41, 3 }, { 44, 2 }, { 49, 9 }, { 74, 4 }, { 75, 5 }, { 77, 1 },

    { 87, 6 }, { 88, 7 }, { 89, 8 }, { 90, 0 }
  };

  const size_t primitiveIndices[10 + 1] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
    10
  };

  const SmSizePair stateOffsets[10] = {
    { 0, 1 }, { 2, 3 }, { 4, 5 }, { 6, 7 }, { 8, 9 }, { 10, 11 },

    { 12, 13 }, { 14, 15 }, { 16, 17 }, { 18, 19 }
  };

  const SmSizePair dofOffsets[10] = {
    { 0, 1 }, { 1, 2 }, { 2, 3 }, { 3, 4 }, { 4, 5 }, { 5, 6 },

    { 6, 7 }, { 7, 8 }, { 8, 9 }, { 9, 10 }
  };

  const size_t *flexibleStages = NULL;
  const size_t remodIndices[10] = {
    0, 2, 4, 6, 8, 10, 12, 14, 16, 18
  };

  const size_t *equationsPerConstraint = NULL;
  const size_t dofToVelSlot[10] = {
    1, 3, 5, 7, 9, 11, 13, 15, 17, 19
  };

  const size_t *constraintDofs = NULL;
  const size_t constraintDofOffsets[0 + 1] = {
    0
  };

  const size_t Jm = 0;
  const size_t Jn = 10;
  SmSizePair zeroSizePair;
  zeroSizePair.mFirst = zeroSizePair.mSecond = 0;
  sm_core_MechanismDelegate_allocScratchpad(delegate);
  scratchpad = delegate->mScratchpad;
  delegate->mTargetStrengthFree = 0;
  delegate->mTargetStrengthSuggested = 1;
  delegate->mTargetStrengthDesired = 2;
  delegate->mTargetStrengthRequired = 3;
  delegate->mConsistencyTol = +1.000000000000000062e-09;
  delegate->mTreeJointDof = 10;
  delegate->mDof = 10;
  delegate->mStateSize = 20;
  delegate->mContinuousStateSize = 20;
  delegate->mModeVectorSize = 0;
  delegate->mNumStages = 10;
  delegate->mNumConstraints = 0;
  delegate->mNumAllConstraintEquations = 0;
  sm_core_SmSizePairVector_create(
    &delegate->mJointToStageIdx, 10, &zeroSizePair);
  memcpy(sm_core_SmSizePairVector_nonConstValues(&delegate->mJointToStageIdx),
         jointToStageIdx, 10 * sizeof(SmSizePair));
  sm_core_SmSizeTVector_create(
    &delegate->mPrimitiveIndices, delegate->mNumStages + 1, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mPrimitiveIndices),
         primitiveIndices, (delegate->mNumStages + 1) * sizeof(size_t));
  sm_core_SmSizePairVector_create(
    &delegate->mStateOffsets, 10, &zeroSizePair);
  memcpy(sm_core_SmSizePairVector_nonConstValues(&delegate->mStateOffsets),
         stateOffsets, 10 * sizeof(SmSizePair));
  sm_core_SmSizePairVector_create(
    &delegate->mDofOffsets, 10, &zeroSizePair);
  memcpy(sm_core_SmSizePairVector_nonConstValues(&delegate->mDofOffsets),
         dofOffsets, 10 * sizeof(SmSizePair));
  sm_core_SmSizeTVector_create(
    &delegate->mFlexibleStages, 0, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mFlexibleStages),
         flexibleStages, 0 * sizeof(size_t));
  sm_core_SmSizeTVector_create(
    &delegate->mRemodIndices, 10, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mRemodIndices),
         remodIndices, 10 * sizeof(size_t));
  sm_core_SmSizeTVector_create(
    &delegate->mEquationsPerConstraint, delegate->mNumConstraints, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mEquationsPerConstraint),
         equationsPerConstraint, delegate->mNumConstraints * sizeof(size_t));
  sm_core_SmIntVector_create(
    &delegate->mRunTimeEnabledEquations,
    delegate->mNumAllConstraintEquations, 1);
  sm_core_SmSizeTVector_create(
    &delegate->mDofToVelSlot, delegate->mDof, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mDofToVelSlot),
         dofToVelSlot, delegate->mDof * sizeof(size_t));
  sm_core_SmSizeTVector_create(
    &delegate->mConstraintDofs, 0, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mConstraintDofs),
         constraintDofs, 0 * sizeof(size_t));
  sm_core_SmSizeTVector_create(
    &delegate->mConstraintDofOffsets, delegate->mNumConstraints + 1, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mConstraintDofOffsets),
         constraintDofOffsets, (delegate->mNumConstraints + 1) * sizeof(size_t));
  sm_core_SmBoundedSet_create(&scratchpad->mPosRequired, 10);
  sm_core_SmBoundedSet_create(&scratchpad->mPosDesired, 10);
  sm_core_SmBoundedSet_create(&scratchpad->mPosSuggested, 10);
  sm_core_SmBoundedSet_create(&scratchpad->mPosFree, 10);
  sm_core_SmBoundedSet_create(&scratchpad->mPosNonRequired, 10);
  sm_core_SmBoundedSet_create(&scratchpad->mPosSuggAndFree, 10);
  sm_core_SmBoundedSet_create(&scratchpad->mVelRequired, 10);
  sm_core_SmBoundedSet_create(&scratchpad->mVelDesired, 10);
  sm_core_SmBoundedSet_create(&scratchpad->mVelSuggested, 10);
  sm_core_SmBoundedSet_create(&scratchpad->mVelFree, 10);
  sm_core_SmBoundedSet_create(&scratchpad->mVelNonRequired, 10);
  sm_core_SmBoundedSet_create(&scratchpad->mVelSuggAndFree, 10);
  sm_core_SmBoundedSet_create(&scratchpad->mConstraintFilter, 0);
  sm_core_SmBoundedSet_create(&scratchpad->mActiveConstraints, 0);
  sm_core_SmBoundedSet_create(&scratchpad->mActiveDofs, 10);
  sm_core_SmBoundedSet_create(&scratchpad->mActiveDofs0, 10);
  sm_core_SmBoundedSet_create(&scratchpad->mNewConstraints, 0);
  sm_core_SmBoundedSet_create(&scratchpad->mNewDofs, 10);
  sm_core_SmBoundedSet_create(&scratchpad->mUnsatisfiedConstraints, 0);
  sm_core_SmSizeTVector_create(&scratchpad->mActiveConstraintsVect,
    0, 0);
  sm_core_SmSizeTVector_create(&scratchpad->mActiveDofsVect, 10, 0);
  sm_core_SmSizeTVector_create(&scratchpad->mFullDofToActiveDof, 10, 0);
  sm_core_SmSizePairVector_create(
    &scratchpad->mPartiallyPosTargetedPrims, 10, &zeroSizePair);
  sm_core_SmSizePairVector_create(
    &scratchpad->mPartiallyVelTargetedPrims, 10, &zeroSizePair);
  sm_core_SmSizeTVector_create(&scratchpad->mPosPartialTypes, 10, 0);
  sm_core_SmSizeTVector_create(&scratchpad->mVelPartialTypes, 10, 0);
  sm_core_SmSizeTVector_create(&scratchpad->mPartiallyActivePrims, 10, 0);
  sm_core_SmSizePairVector_create(
    &scratchpad->mBaseFrameVelOffsets, 0, &zeroSizePair);
  sm_core_SmSizePairVector_create(
    &scratchpad->mCvVelOffsets, 10, &zeroSizePair);
  sm_core_SmRealVector_create(
    &scratchpad->mCvAzimuthValues, 10, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mInitialState, 20, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mStartState, 20, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mTestState, 20, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mFullStateVector, 20, 0.0);
  sm_core_SmIntVector_create(&scratchpad->mModeVector, 0, 0);
  sm_core_SmRealVector_create(&scratchpad->mJacobianRowMaj, Jm * Jn, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mJacobian, Jm * Jn, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mJacobianPrimSubmatrix, Jm * 6, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mConstraintNonhomoTerms, Jm, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mConstraintError, Jm, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mBestConstraintError, Jm, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mDeltas,
    Jn * (Jm <= Jn ? Jm : Jn), 0.0);
  sm_core_SmRealVector_create(&scratchpad->mSvdWork, 121, 0.0);
  sm_core_SmRealVector_create(
    &scratchpad->mLineSearchScaledDeltaVect, 10, 0.0);
  sm_core_SmRealVector_create(
    &scratchpad->mLineSearchTestStateVect, 20, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mLineSearchErrorVect, Jm, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mActiveDofVelsVect, 10, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mVelSystemRhs, Jm, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mMotionData, 186, 0.0);
  delegate->mSetTargets = NULL;
  delegate->mResetStateVector = human9DOF_836bb176_1_resetSimStateVector;
  delegate->mInitializeTrackedAngleState = NULL;
  delegate->mComputeDiscreteState = NULL;
  delegate->mAdjustPosition = NULL;
  delegate->mPerturbJointPrimitiveState =
    human9DOF_836bb176_1_perturbSimJointPrimitiveState;
  delegate->mPerturbFlexibleBodyState =
    human9DOF_836bb176_1_perturbFlexibleBodyState;
  delegate->mComputePosDofBlendMatrix = NULL;
  delegate->mComputeVelDofBlendMatrix = NULL;
  delegate->mProjectPartiallyTargetedPos = NULL;
  delegate->mPropagateMotion = NULL;
  delegate->mComputeAssemblyError = NULL;
  delegate->mComputeAssemblyJacobian = NULL;
  delegate->mComputeFullAssemblyJacobian = NULL;
  delegate->mIsInKinematicSingularity = NULL;
  delegate->mConvertStateVector = NULL;
  delegate->mConstructStateVector = human9DOF_836bb176_1_constructStateVector;
  delegate->mExtractSolverStateVector =
    human9DOF_836bb176_1_extractSolverStateVector;
  delegate->mIsPositionViolation = human9DOF_836bb176_1_isPositionViolation;
  delegate->mIsVelocityViolation = human9DOF_836bb176_1_isVelocityViolation;
  delegate->mProjectStateSim = human9DOF_836bb176_1_projectStateSim;
  delegate->mComputeConstraintError =
    human9DOF_836bb176_1_computeConstraintError;
  delegate->mResetModeVector = human9DOF_836bb176_1_resetModeVector;
  delegate->mHasJointDisToNormModeChange =
    human9DOF_836bb176_1_hasJointDisToNormModeChange;
  delegate->mPerformJointDisToNormModeChange =
    human9DOF_836bb176_1_performJointDisToNormModeChange;
  delegate->mOnModeChangedCutJoints =
    human9DOF_836bb176_1_onModeChangedCutJoints;
  delegate->mMech = NULL;
}

static
  void initMechanismDelegates(NeDaePrivateData *smData)
{
  PmAllocator *alloc = pm_default_allocator();
  const int32_T *motionInputOffsets = NULL;
  int_T status = 0;
  initAssemblyDelegate(&smData->mAssemblyDelegate);
  initSimulationDelegate(&smData->mSimulationDelegate);
  status = pm_create_int_vector_fields(
    &smData->mMotionInputOffsets, smData->mNumInputMotionPrimitives, alloc);
  checkMemAllocStatus(status);
  memcpy(smData->mMotionInputOffsets.mX, motionInputOffsets,
         0 * sizeof(int32_T));
}

static
  void initComputationFcnPtrs(NeDaePrivateData *smData)
{
  smData->mSetParametersFcn = dae_cg_setParameters_function;
  smData->mPAssertFcn = dae_cg_pAssert_method;
  smData->mDerivativeFcn = dae_cg_deriv_method;
  smData->mNumJacPerturbLoBoundsFcn = dae_cg_numJacPerturbLoBounds_method;
  smData->mNumJacPerturbHiBoundsFcn = dae_cg_numJacPerturbHiBounds_method;
  smData->mOutputFcn = dae_cg_compOutputs_method;
  smData->mModeFcn = dae_cg_mode_method;
  smData->mZeroCrossingFcn = dae_cg_zeroCrossing_method;
  smData->mProjectionFcn = dae_cg_project_solve;
  smData->mCIC_MODE_Fcn = dae_cg_CIC_MODE_solve;
  smData->mCheckFcn =
    (smData->mStateVectorSize == 0) ? dae_cg_check_solve : NULL;
  smData->mAssemblyFcn = dae_cg_assemble_solve;
  smData->mSetupLoggerFcn = sm_ssci_setupLoggerFcn_codeGen;
  smData->mLogFcn = sm_ssci_logFcn_codeGen;
  smData->mResidualsFcn = NULL;
  smData->mLinearizeFcn = NULL;
  smData->mGenerateFcn = NULL;
}

static
  void initLiveLinkToSm(NeDaePrivateData *smData)
{
  smData->mLiveSmLink = NULL;
  smData->mLiveSmLink_destroy = NULL;
  smData->mLiveSmLink_copy = NULL;
}

void human9DOF_836bb176_1_NeDaePrivateData_create(NeDaePrivateData *smData)
{
  initBasicAttributes (smData);
  initStateVector (smData);
  initAsserts (smData);
  initModeVector (smData);
  initZeroCrossings (smData);
  initVariables (smData);
  initRuntimeParameters (smData);
  initIoInfo (smData);
  initInputDerivs (smData);
  initDirectFeedthrough (smData);
  initOutputDerivProc (smData);
  initMechanismDelegates (smData);
  initComputationFcnPtrs (smData);
  initLiveLinkToSm (smData);
}
