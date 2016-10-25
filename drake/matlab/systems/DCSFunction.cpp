
#define S_FUNCTION_NAME DCSFunction  // for Drake C S-Function
#define S_FUNCTION_LEVEL 2
#include <simstruc.h>

#include <math.h>

// indices of the IWork vector
#define IC_IDX 0
#define IS_HYBRID_IDX 1
#define IS_STOCHASTIC_IDX 2
#define DT_SAMPLE_TIME_IDX 3

// mxIsClass seems to not be able to handle derived classes. so i'll implement
// what I need by calling back to matlab
bool isa(const mxArray *mxa, const char *class_str) {
  mxArray *plhs;
  mxArray *prhs[2];
  prhs[0] = const_cast<mxArray *>(mxa);
  prhs[1] = mxCreateString(class_str);
  mexCallMATLAB(1, &plhs, 2, prhs, "isa");
  bool tf = (mxGetScalar(plhs) != 0.0);
  mxDestroyArray(plhs);
  mxDestroyArray(prhs[1]);
  return tf;
}

bool mexCallMATLABsafe(SimStruct *S, int nlhs, mxArray *plhs[], int nrhs,
                       mxArray *prhs[], const char *filename) {
  int i;
  mxArray *ex = mexCallMATLABWithTrap(nlhs, plhs, nrhs, prhs, filename);
  if (ex) {
    mexPrintf(
        "DrakeSystem S-Function: error when calling ''%s'' with the following "
        "arguments:\n",
        filename);
    for (i = 0; i < nrhs; i++) mexCallMATLAB(0, NULL, 1, &prhs[i], "disp");
    mxArray *report;
    mexCallMATLAB(1, &report, 1, &ex, "getReport");
    char *errmsg = mxArrayToString(report);
    mexPrintf(errmsg);
    mxFree(errmsg);
    mxDestroyArray(report);
    mxArray *identifier = mxGetProperty(ex, 0, "identifier");
    char buffer[200];
    errmsg = mxArrayToString(identifier);
    snprintf(buffer, sizeof(buffer),
             "\n\nDrakeSystem S-Function: error %s in MATLAB callback.\nSee "
             "additional debugging information above",
             errmsg);
    ssSetErrorStatus(S, buffer);
    mxFree(errmsg);
    //    mxFree(identifier);  // it appears I'm not supposed to free this
    mxDestroyArray(ex);
    return true;
  }
  for (i = 0; i < nlhs; i++)
    if (!plhs[i]) {
      mexPrintf(
          "DrakeSystem S-Function: error when calling ''%s'' with the "
          "following arguments:\n",
          filename);
      for (i = 0; i < nrhs; i++) mexCallMATLAB(0, NULL, 1, &prhs[i], "disp");
      mexPrintf("Asked for %d outputs, but function only returned %d\n", nrhs,
                i);
      ssSetErrorStatus(S, "DrakeSystem S-Function: not enough outputs");
      return true;
    }
  return false;
}

#define MDL_CHECK_PARAMETERS
#if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
static void mdlCheckParameters(SimStruct *S) {
  const mxArray *psys = ssGetSFcnParam(S, 0);
  if (!isa(psys, "DrakeSystem"))
    ssSetErrorStatus(S,
                     "First dialog parameter must be a DrakeSystem of "
                     "HybridDrakeSystem class.");
}
#endif /* MDL_CHECK_PARAMETERS */

static void mdlInitializeSizes(SimStruct *S) {
  ssSetNumSFcnParams(S, 1); /* Number of expected parameters */

#if defined(MATLAB_MEX_FILE)
  if (ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S)) {
    mdlCheckParameters(S);
    if (ssGetErrorStatus(S) != NULL) {
      return;
    }
  } else {
    return;
  }
#endif

  mxArray *psys = const_cast<mxArray *>(ssGetSFcnParam(S, 0));
  mxArray *plhs[1];

  if (mexCallMATLABsafe(S, 1, plhs, 1, &psys, "getNumContStates")) return;
  ssSetNumContStates(S, (int)mxGetScalar(plhs[0]));
  mxDestroyArray(plhs[0]);

  if (mexCallMATLABsafe(S, 1, plhs, 1, &psys, "getNumDiscStates")) return;
  ssSetNumDiscStates(S, (int)mxGetScalar(plhs[0]));
  mxDestroyArray(plhs[0]);

  if (mexCallMATLABsafe(S, 1, plhs, 1, &psys, "getNumInputs")) return;
  int num_u = (int)mxGetScalar(plhs[0]);
  mxDestroyArray(plhs[0]);

  int num_w = 0;
  if (isa(psys, "StochasticDrakeSystem")) {
    if (mexCallMATLABsafe(S, 1, plhs, 1, &psys, "getNumDisturbances")) return;
    num_w = static_cast<int>(mxGetScalar(plhs[0]));
    mxDestroyArray(plhs[0]);
  }

  if (!ssSetNumInputPorts(S, (int)(num_u > 0) + (int)(num_w > 0))) return;

  if (num_u > 0) {
    ssSetInputPortWidth(S, 0, num_u);

    // note: can actually be directfeedthrough even if num_y == 0 (e.g., for the
    // visualizer)
    if (mexCallMATLABsafe(S, 1, plhs, 1, &psys, "isDirectFeedthrough")) return;
    ssSetInputPortDirectFeedThrough(S, 0, (int)mxGetScalar(plhs[0]));
    mxDestroyArray(plhs[0]);

    ssSetInputPortRequiredContiguous(S, 0, 1);
  }

  if (num_w > 0) {
    int portnum = ssGetNumInputPorts(S) - 1;  // portnums start from zero
    ssSetInputPortWidth(S, portnum, num_w);
    ssSetInputPortDirectFeedThrough(S, portnum, 1);  // go ahead and say yes to
                                                     // direct feed-through.
                                                     // there is no threat of
                                                     // algebraic loops here
    ssSetInputPortRequiredContiguous(S, portnum, 1);
  }

  if (mexCallMATLABsafe(S, 1, plhs, 1, &psys, "getNumOutputs")) return;
  int num_y = (int)mxGetScalar(plhs[0]);
  mxDestroyArray(plhs[0]);

  if (num_y > 0) {
    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, num_y);
  } else {
    if (!ssSetNumOutputPorts(S, 0)) return;
  }

  if (mexCallMATLABsafe(S, 1, plhs, 1, &psys, "getSampleTime")) return;
  ssSetNumSampleTimes(S, static_cast<int_T>(mxGetN(plhs[0])));
  mxDestroyArray(plhs[0]);

  if (mexCallMATLABsafe(S, 1, plhs, 1, &psys, "getNumZeroCrossings")) return;
  ssSetNumNonsampledZCs(S, (int)mxGetScalar(plhs[0]));
  mxDestroyArray(plhs[0]);

  ssSetNumModes(S, 0);
  ssSetNumRWork(S, 0);
  ssSetNumIWork(S, 4);  // see #define for indices at top
  ssSetNumPWork(S, 0);

  ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);

  ssSetOptions(S, 0);
}

static void mdlInitializeSampleTimes(SimStruct *S) {
  mxArray *psys = const_cast<mxArray *>(ssGetSFcnParam(S, 0));
  mxArray *plhs[1];
  if (mexCallMATLABsafe(S, 1, plhs, 1, &psys, "getSampleTime")) return;
  double *pts = mxGetPr(plhs[0]);
  for (int i = 0; i < ssGetNumSampleTimes(S); i++) {
    ssSetSampleTime(S, i, *pts++);
    ssSetOffsetTime(S, i, *pts++);
  }
}

#define MDL_INITIALIZE_CONDITIONS
#if defined(MDL_INITIALIZE_CONDITIONS)
static void mdlInitializeConditions(SimStruct *S) {
  mxArray *psys = const_cast<mxArray *>(ssGetSFcnParam(S, 0));
  mxArray *plhs[1];
  if (mexCallMATLABsafe(S, 1, plhs, 1, &psys, "getInitialState")) return;

  real_T *xc0 = ssGetContStates(S);
  real_T *xd0 = ssGetDiscStates(S);
  real_T *x = mxGetPr(plhs[0]);
  int_T num_xd = ssGetNumDiscStates(S);
  int_T num_xc = ssGetNumContStates(S);

  if (isa(psys, "HybridDrakeSystem")) {
    ssSetIWorkValue(S, IS_HYBRID_IDX, 1);
  } else {
    ssSetIWorkValue(S, IS_HYBRID_IDX, 0);
  }

  if (num_xd) {
    memcpy(xd0, x, sizeof(real_T) * num_xd);
    x += num_xd;
  }
  if (num_xc) {
    memcpy(xc0, x, sizeof(real_T) * num_xc);
  }

  ssSetIWorkValue(S, 0, 1);

  mxDestroyArray(plhs[0]);

  if (isa(psys, "StochasticDrakeSystem")) {
    if (mexCallMATLABsafe(S, 1, plhs, 1, &psys, "getNumDisturbances")) return;
    int num_w = static_cast<int>(mxGetScalar(plhs[0]));
    mxDestroyArray(plhs[0]);
    ssSetIWorkValue(S, IS_STOCHASTIC_IDX, num_w);
  } else {
    ssSetIWorkValue(S, IS_STOCHASTIC_IDX, 0);
  }

  // setup work vector with sample time information
  // note: wanted to do this in initializeSampleTimes, but you're not allowed to
  // touch work vectors there
  ssSetIWorkValue(S, DT_SAMPLE_TIME_IDX, -1);
  for (int i = 0; i < ssGetNumSampleTimes(S); i++) {
    if (ssGetSampleTime(S, i) > 0) {
      if (ssGetIWorkValue(S, DT_SAMPLE_TIME_IDX) >= 0)
        ssSetErrorStatus(
            S, "DrakeSystems may only have one discrete sample time (for now)");
      ssSetIWorkValue(S, DT_SAMPLE_TIME_IDX, i);
    }
  }
}
#endif /* MDL_INITIALIZE_CONDITIONS */

static void setScopeEnable(SimStruct *S) {
  mxArray *pEnable = mxCreateLogicalScalar(ssIsMajorTimeStep(S));
  mexPutVariable("global", "g_scope_enable", pEnable);
  mxDestroyArray(pEnable);
}

static void mdlOutputs(SimStruct *S, int_T tid) {
  int num_xd = ssGetNumDiscStates(S);
  int num_xc = ssGetNumContStates(S);
  int num_w = ssGetIWorkValue(S, IS_STOCHASTIC_IDX);
  bool sds = num_w > 0;
  int num_u = (ssGetNumInputPorts(S) - sds) ? ssGetInputPortWidth(S, 0) : 0;
  int num_y = ssGetNumOutputPorts(S) ? ssGetOutputPortWidth(S, 0) : 0;

  setScopeEnable(S);

  mxArray *psys = const_cast<mxArray *>(ssGetSFcnParam(S, 0));
  time_T t = ssGetT(S);
  real_T *xd = ssGetDiscStates(S);
  real_T *xc = ssGetContStates(S);
  const real_T *u;
  const real_T *w =
      num_w > 0 ? ssGetInputPortRealSignal(S, num_u > 0 ? 1 : 0) : NULL;
  real_T *y = (num_y > 0) ? ssGetOutputPortRealSignal(S, 0) : NULL;
  mxArray *plhs[2];
  bool fsm = (ssGetIWorkValue(S, IS_HYBRID_IDX) != 0);

  mxArray *prhs[5];
  prhs[0] = psys;                                              // obj
  prhs[1] = mxCreateDoubleScalar(t);                           // t
  prhs[2] = mxCreateDoubleMatrix(num_xd + num_xc, 1, mxREAL);  // x
  real_T *px = mxGetPr(prhs[2]);
  real_T *pxtmp = px;
  if (num_xd > 0) {
    memcpy(pxtmp, xd, sizeof(real_T) * num_xd);
    pxtmp += num_xd;
  }
  if (num_xc > 0) {
    memcpy(pxtmp, xc, sizeof(real_T) * num_xc);
  }

  prhs[3] = mxCreateDoubleMatrix(num_u, 1, mxREAL);  // u
  if (num_u > 0 && ssGetInputPortDirectFeedThrough(
                       S, 0)) {  // not allowed to reference the input port
                                 // signal unless it's feedthrough
    u = ssGetInputPortRealSignal(S, 0);
    memcpy(mxGetPr(prhs[3]), u, sizeof(real_T) * num_u);
  }  // else: leave u at random values (shouldn't be used)

  prhs[4] = mxCreateDoubleMatrix(num_w, 1, mxREAL);  // w
  if (num_w > 0) memcpy(mxGetPr(prhs[4]), w, sizeof(real_T) * num_w);

  if (ssGetIWorkValue(S, IC_IDX) == 1) {
    if (mexCallMATLABsafe(S, 1, plhs, 4, prhs, "getInitialStateWInput")) return;
    real_T *px0 = mxGetPr(plhs[0]);  // copy over to the state input used for
                                     // the rest of this function
    memcpy(px, px0, sizeof(real_T) * (num_xd + num_xc));
    // update the actual state vectors:
    if (num_xd) {
      memcpy(xd, px0, sizeof(real_T) * num_xd);
      px0 += num_xd;
    }
    if (num_xc) {
      memcpy(xc, px0, sizeof(real_T) * num_xc);
    }
    ssSetIWorkValue(S, IC_IDX, 2);
    mxDestroyArray(plhs[0]);
  }

  if (fsm && ssIsMajorTimeStep(S)) {
    // then check for zero-crossing events and apply and discontinuous changes
    if (mexCallMATLABsafe(S, 2, plhs, 4, prhs, "transitionUpdate")) return;
    real_T *pxn = mxGetPr(plhs[0]);
    memcpy(px, pxn, sizeof(real_T) * (num_xd + num_xc));  // copy over to the
                                                          // state input used
                                                          // for the rest of
                                                          // this function
    // update the actual state vectors:
    // if (num_xd) {  // for fsm's, it's guaranteed that num_xd>0
    memcpy(xd, pxn, sizeof(real_T) * num_xd);
    pxn += num_xd;
    //}
    if (num_xc) {
      memcpy(xc, pxn, sizeof(real_T) * num_xc);
    }

    if (mxGetScalar(plhs[1]) > 0) ssSetStopRequested(S, 1);
    mxDestroyArray(plhs[0]);
    mxDestroyArray(plhs[1]);
  }

  if (mexCallMATLABsafe(S, 1, plhs, sds ? 5 : 4, prhs,
                        sds ? "stochasticOutput" : "output"))
    return;  // call it even if there are no outputs (e.g., for visualizers)
  if (num_y > 0) {
    memcpy(y, mxGetPr(plhs[0]), sizeof(real_T) * num_y);
    mxDestroyArray(plhs[0]);
  }

  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  mxDestroyArray(prhs[4]);
}

#define MDL_ZERO_CROSSINGS /* Change to #undef to remove function */
#if defined(MDL_ZERO_CROSSINGS) && (defined(MATLAB_MEX_FILE) || defined(NRT))
static void mdlZeroCrossings(SimStruct *S) {
  int num_xd = ssGetNumDiscStates(S);
  int num_xc = ssGetNumContStates(S);
  int num_w = ssGetIWorkValue(S, IS_STOCHASTIC_IDX);
  bool sds = num_w > 0;
  int num_u = (ssGetNumInputPorts(S) - sds) ? ssGetInputPortWidth(S, 0) : 0;
  int num_zcs = ssGetNumNonsampledZCs(S);

  mxArray *psys = const_cast<mxArray *>(ssGetSFcnParam(S, 0));
  time_T t = ssGetT(S);
  real_T *xc = num_xc > 0 ? ssGetContStates(S) : NULL;
  real_T *xd = num_xd > 0 ? ssGetDiscStates(S) : NULL;
  const real_T *u = num_u > 0 ? ssGetInputPortRealSignal(S, 0) : NULL;
  real_T *zcs = ssGetNonsampledZCs(S);

  mxArray *plhs[1];
  mxArray *prhs[4];
  prhs[0] = psys;                     // obj
  prhs[1] = mxCreateDoubleScalar(t);  // t

  prhs[2] = mxCreateDoubleMatrix(num_xd + num_xc, 1, mxREAL);  // x
  real_T *px = mxGetPr(prhs[2]);
  real_T *pxtmp = px;

  if (num_xd) {
    memcpy(pxtmp, xd, sizeof(real_T) * num_xd);
    pxtmp += num_xd;
  }

  if (num_xc > 0) memcpy(pxtmp, xc, sizeof(real_T) * num_xc);

  prhs[3] = mxCreateDoubleMatrix(num_u, 1, mxREAL);  // u
  if (num_u > 0) memcpy(mxGetPr(prhs[3]), u, sizeof(real_T) * num_u);

  if (mexCallMATLABsafe(S, 1, plhs, 4, prhs, "zeroCrossings")) return;
  memcpy(zcs, mxGetPr(plhs[0]), sizeof(real_T) * num_zcs);
  mxDestroyArray(plhs[0]);

  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
}
#endif /* MDL_ZERO_CROSSINGS */

#define MDL_DERIVATIVES
#if defined(MDL_DERIVATIVES)
static void mdlDerivatives(SimStruct *S) {
  int num_xc = ssGetNumContStates(S);
  if (num_xc < 1) return;

  int num_xd = ssGetNumDiscStates(S);
  int num_w = ssGetIWorkValue(S, IS_STOCHASTIC_IDX);
  bool sds = num_w > 0;
  int num_u = (ssGetNumInputPorts(S) - sds) ? ssGetInputPortWidth(S, 0) : 0;

  setScopeEnable(S);

  mxArray *psys = const_cast<mxArray *>(ssGetSFcnParam(S, 0));
  time_T t = ssGetT(S);
  real_T *xd = ssGetDiscStates(S);
  real_T *xc = ssGetContStates(S);
  const real_T *u = num_u > 0 ? ssGetInputPortRealSignal(S, 0) : NULL;
  const real_T *w =
      num_w > 0 ? ssGetInputPortRealSignal(S, num_u > 0 ? 1 : 0) : NULL;
  real_T *xcdot = ssGetdX(S);
  mxArray *plhs[1];

  mxArray *prhs[5];
  prhs[0] = psys;                     // obj
  prhs[1] = mxCreateDoubleScalar(t);  // t

  prhs[2] = mxCreateDoubleMatrix(num_xd + num_xc, 1, mxREAL);  // x
  real_T *px = mxGetPr(prhs[2]);
  real_T *pxtmp = px;
  if (num_xd) {
    memcpy(pxtmp, xd, sizeof(real_T) * num_xd);
    pxtmp += num_xd;
  }
  if (num_xc) {
    memcpy(pxtmp, xc, sizeof(real_T) * num_xc);
  }

  prhs[3] = mxCreateDoubleMatrix(num_u, 1, mxREAL);  // u
  if (num_u > 0) memcpy(mxGetPr(prhs[3]), u, sizeof(real_T) * num_u);

  prhs[4] = mxCreateDoubleMatrix(num_w, 1, mxREAL);  // w
  if (num_w > 0) memcpy(mxGetPr(prhs[4]), w, sizeof(real_T) * num_w);

  if (mexCallMATLABsafe(S, 1, plhs, sds ? 5 : 4, prhs,
                        sds ? "stochasticDynamics" : "dynamics"))
    return;
  memcpy(xcdot, mxGetPr(plhs[0]), sizeof(real_T) * num_xc);
  mxDestroyArray(plhs[0]);

  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  mxDestroyArray(prhs[4]);
}
#endif /* MDL_DERIVATIVES */

#define MDL_UPDATE
#if defined(MDL_UPDATE)
static void mdlUpdate(SimStruct *S, int_T tid) {
  int sample_time_idx = ssGetIWorkValue(S, DT_SAMPLE_TIME_IDX);
  if (sample_time_idx < 0 || !ssIsSampleHit(S, sample_time_idx, tid)) return;

  int num_xd = ssGetNumDiscStates(S);
  if (num_xd < 1) return;
  int num_xc = ssGetNumContStates(S);
  int num_w = ssGetIWorkValue(S, IS_STOCHASTIC_IDX);
  bool sds = num_w > 0;
  int num_u = (ssGetNumInputPorts(S) - sds) ? ssGetInputPortWidth(S, 0) : 0;

  setScopeEnable(S);

  mxArray *psys = const_cast<mxArray *>(ssGetSFcnParam(S, 0));
  time_T t = ssGetT(S);
  real_T *xd = ssGetDiscStates(S);
  real_T *xc = ssGetContStates(S);
  const real_T *u = num_u > 0 ? ssGetInputPortRealSignal(S, 0) : NULL;
  const real_T *w =
      num_w > 0 ? ssGetInputPortRealSignal(S, num_u > 0 ? 1 : 0) : NULL;
  mxArray *plhs[1];

  mxArray *prhs[5];
  prhs[0] = psys;                     // obj
  prhs[1] = mxCreateDoubleScalar(t);  // t

  prhs[2] = mxCreateDoubleMatrix(num_xd + num_xc, 1, mxREAL);  // x
  real_T *px = mxGetPr(prhs[2]);
  real_T *pxtmp = px;
  if (num_xd) {
    memcpy(pxtmp, xd, sizeof(real_T) * num_xd);
    pxtmp += num_xd;
  }
  if (num_xc) {
    memcpy(pxtmp, xc, sizeof(real_T) * num_xc);
  }

  prhs[3] = mxCreateDoubleMatrix(num_u, 1, mxREAL);  // u
  if (num_u > 0) memcpy(mxGetPr(prhs[3]), u, sizeof(real_T) * num_u);

  prhs[4] = mxCreateDoubleMatrix(num_w, 1, mxREAL);  // w
  if (num_w > 0) memcpy(mxGetPr(prhs[4]), w, sizeof(real_T) * num_w);

  if (mexCallMATLABsafe(S, 1, plhs, sds ? 5 : 4, prhs,
                        sds ? "stochasticUpdate" : "update"))
    return;
  if (!mxIsDouble(plhs[0]) || (
          static_cast<int>(mxGetNumberOfElements(plhs[0])) != num_xd)) {
    mexPrintf("I expected xdn to have %d elements, but got  %d.\n", num_xd,
              mxGetNumberOfElements(plhs[0]));
    ssSetErrorStatus(S,
                     "\n\nDrakeSystem S-Function: error in MATLAB update "
                     "callback: update did not return a double with the "
                     "correct number of elements.");
  }

  memcpy(xd, mxGetPr(plhs[0]), sizeof(real_T) * num_xd);
  mxDestroyArray(plhs[0]);

  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  mxDestroyArray(prhs[4]);
}
#endif /* MDL_UPDATE */

static void mdlTerminate(SimStruct *S) {}

#ifdef MATLAB_MEX_FILE /* Is this file being compiled as a MEX-file? */
#include "simulink.c"  /* MEX-file interface mechanism */
#else
// NOLINTNEXTLINE(build/include)
#include "cg_sfun.h" /* Code generation registration function */
#endif
