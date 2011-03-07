
#define S_FUNCTION_NAME  RLCSFunction
#define S_FUNCTION_LEVEL 2

#include <math.h>
#include "simstruc.h"

bool isa(const mxArray* mxa, const char* class_str)
// mxIsClass seems to not be able to handle derived classes. so i'll implement what I need by calling back to matlab
{
  mxArray* plhs;
  mxArray* prhs[2];
  prhs[0] = const_cast<mxArray*>(mxa);
  prhs[1] = mxCreateString(class_str);
  mexCallMATLAB(1,&plhs,2,prhs,"isa");
  bool tf = (bool) mxGetScalar(plhs);
  mxDestroyArray(plhs);
  mxDestroyArray(prhs[1]);
  return tf;
}

#define MDL_CHECK_PARAMETERS  
#if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
static void mdlCheckParameters(SimStruct *S)
{
  const mxArray *psys = ssGetSFcnParam(S,0);
  if (!isa(psys,"RobotLibSystem"))
    ssSetErrorStatus(S,"First dialog parameter must be a RobotLibSystem of HybridRobotLibSystem class.");
}
#endif /* MDL_CHECK_PARAMETERS */

static void mdlInitializeSizes(SimStruct *S)
{
  ssSetNumSFcnParams(S, 1);  /* Number of expected parameters */

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
  
  mxArray *psys = const_cast<mxArray*>(ssGetSFcnParam(S, 0));
  mxArray* plhs[1];

  if (mexCallMATLAB(1,plhs,1,&psys,"getNumContStates")) return;
  ssSetNumContStates(S, (int)mxGetScalar(plhs[0]));
  mxDestroyArray(plhs[0]);
  
  if (mexCallMATLAB(1,plhs,1,&psys,"getNumDiscStates")) return;
  ssSetNumDiscStates(S, (int)mxGetScalar(plhs[0]));
  mxDestroyArray(plhs[0]);
  

  if (mexCallMATLAB(1,plhs,1,&psys,"getNumInputs")) return;
  int num_u = (int)mxGetScalar(plhs[0]);
  mxDestroyArray(plhs[0]);

  if (num_u>0) {
    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S,0,num_u);

    // note: can actually be directfeedthrough even if num_y == 0 (e.g., for the visualizer)
    if (mexCallMATLAB(1,plhs,1,&psys,"isDirectFeedthrough")) return;
    ssSetInputPortDirectFeedThrough(S, 0, (int)mxGetScalar(plhs[0]));
    mxDestroyArray(plhs[0]);

    ssSetInputPortRequiredContiguous(S,0,1);
  } else {
    if (!ssSetNumInputPorts(S, 0)) return;
  }
  
  if (mexCallMATLAB(1,plhs,1,&psys,"getNumOutputs")) return;
  int num_y = (int)mxGetScalar(plhs[0]);
  mxDestroyArray(plhs[0]);
  
  if (num_y>0) {
    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S,0,num_y);
  } else {
    if (!ssSetNumOutputPorts(S, 0)) return;
  }

  if (mexCallMATLAB(1,plhs,1,&psys,"getSampleTime")) return;
  ssSetNumSampleTimes(S, mxGetN(plhs[0]));
  mxDestroyArray(plhs[0]);

  ssSetNumRWork(S, 0);
  ssSetNumIWork(S, 0);
  ssSetNumPWork(S, 0);
  
  if (isa(psys,"HybridRobotLibSystem")) {
    ssSetNumModes(S, 1);

    if (mexCallMATLAB(1,plhs,1,&psys,"getNumZeroCrossings")) return;
    ssSetNumNonsampledZCs(S, (int)mxGetScalar(plhs[0]));
    mxDestroyArray(plhs[0]);

    // i need a memoryless fsm for many of the hybrid controllers.  i now think this is not required: 
    //    if (ssGetNumContStates(S)<1) 
    //     ssSetErrorStatus(S,"HybridRobotLibSystems must have numContStates>0");
  } else {    
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);
  }

  ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);

  ssSetOptions(S, 0);
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
  mxArray *psys = const_cast<mxArray*>(ssGetSFcnParam(S, 0));
  mxArray* plhs[1];
  if (mexCallMATLAB(1,plhs,1,&psys,"getSampleTime")) return;
  double* pts = mxGetPr(plhs[0]);
  for (int i=0; i<ssGetNumSampleTimes(S); i++) {
    ssSetSampleTime(S, 0, *pts++);
    ssSetOffsetTime(S, 0, *pts++);
  }
}

#define MDL_INITIALIZE_CONDITIONS   
#if defined(MDL_INITIALIZE_CONDITIONS)
static void mdlInitializeConditions(SimStruct *S)
{
  mxArray *psys = const_cast<mxArray*>(ssGetSFcnParam(S, 0));
  mxArray* plhs[1];
  if (mexCallMATLAB(1,plhs,1,&psys,"getInitialState")) return;

  real_T* x0 = ssGetContStates(S);
  real_T* x = mxGetPr(plhs[0]);

  if (isa(psys,"HybridRobotLibSystem")) {
    int_T* mode = ssGetModeVector(S);
    mode[0] = (int) x[0];
    x++;
  }

  memcpy(x0,x,sizeof(real_T)*ssGetNumContStates(S));
  mxDestroyArray(plhs[0]);
}
#endif /* MDL_INITIALIZE_CONDITIONS */

static void setScopeEnable(SimStruct* S)
{
  mxArray* pEnable = mxCreateLogicalScalar(ssIsMajorTimeStep(S));
  mexPutVariable("global","g_scope_enable",pEnable);
  mxDestroyArray(pEnable);
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
  int num_xc = ssGetNumContStates(S);
  int num_u = ssGetNumInputPorts(S) ? ssGetInputPortWidth(S,0) : 0;
  int num_y = ssGetNumOutputPorts(S) ? ssGetOutputPortWidth(S,0) : 0;

  setScopeEnable(S);
  
  mxArray *psys = const_cast<mxArray*>(ssGetSFcnParam(S, 0));
  time_T t = ssGetT(S);
  real_T *x = ssGetContStates(S);
  const real_T *u; 
  real_T *y = (num_y>0) ? ssGetOutputPortRealSignal(S, 0) : NULL;
  mxArray* plhs[2];
  bool fsm = isa(psys,"HybridRobotLibSystem");
  int_T *mode = (fsm ? ssGetModeVector(S) : NULL);

  mxArray *prhs[4];
  prhs[0] = psys;                                  // obj
  prhs[1] = mxCreateDoubleScalar(t);               // t

  prhs[2] = mxCreateDoubleMatrix(num_xc + (fsm ? 1 : 0),1,mxREAL); // x
  real_T* px = mxGetPr(prhs[2]);
  if (fsm) { 
    px[0] = (real_T) mode[0];
    if (num_xc>0) memcpy(&px[1],x,sizeof(real_T)*num_xc); 
  } else if (num_xc>0) {
    real_T* px = mxGetPr(prhs[2]);
    memcpy(px,x,sizeof(real_T)*num_xc); 
  }

  prhs[3] = mxCreateDoubleMatrix(num_u, 1, mxREAL);  // u
  if (mexCallMATLAB(1, plhs, 1, &psys, "isDirectFeedthrough")) return;
  if (num_u>0 && (int)mxGetScalar(plhs[0])) { // not allowed to reference the input port signal unless it's feedthrough
    u = ssGetInputPortRealSignal(S,0);
    memcpy(mxGetPr(prhs[3]), u, sizeof(real_T)*num_u);
  } // else: leave u at random values (shouldn't be used)
  mxDestroyArray(plhs[0]);

  if (fsm && ssIsMajorTimeStep(S)) {
    // then check for zero-crossing events and apply and discontinuous changes 
    if (mexCallMATLAB(2, plhs, 4, prhs, "transitionUpdate")) return;
    real_T *pxn = mxGetPr(plhs[0]);
    memcpy(px,pxn,sizeof(real_T)*(num_xc+1));  // copy over to the state input used for the rest of this function
    // update the actual state vectors:
    mode[0] = (int) pxn[0];
    if (num_xc>0) memcpy(x,&pxn[1],sizeof(real_T)*num_xc);
    if (mxGetScalar(plhs[1])>0) 
      ssSetStopRequested(S,1);
    mxDestroyArray(plhs[0]);
    mxDestroyArray(plhs[1]);
  }

  if (mexCallMATLAB(1, plhs, 4, prhs, "output")) return;  // call it even if there are no outputs (e.g., for visualizers)
  if (num_y>0) {
    memcpy(y, mxGetPr(plhs[0]), sizeof(real_T)*num_y);
    mxDestroyArray(plhs[0]);
  }
  
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
}

#define MDL_ZERO_CROSSINGS   /* Change to #undef to remove function */
#if defined(MDL_ZERO_CROSSINGS) && (defined(MATLAB_MEX_FILE) || defined(NRT))
static void mdlZeroCrossings(SimStruct *S)
{ 
  int num_xc = ssGetNumContStates(S);
  int num_u = ssGetNumInputPorts(S) ? ssGetInputPortWidth(S,0) : 0;
  int num_zcs = ssGetNumNonsampledZCs(S);

  mxArray *psys = const_cast<mxArray*>(ssGetSFcnParam(S, 0));
  time_T t = ssGetT(S);
  real_T *x = num_xc>0 ? ssGetContStates(S) : NULL;
  const real_T *u = num_u>0 ? ssGetInputPortRealSignal(S,0) : NULL;
  int_T *mode = ssGetModeVector(S);
  real_T *zcs = ssGetNonsampledZCs(S);  

  mxArray* plhs[1];
  mxArray *prhs[4];
  prhs[0] = psys;                                  // obj
  prhs[1] = mxCreateDoubleScalar(t);               // t

  prhs[2] = mxCreateDoubleMatrix(num_xc + 1,1,mxREAL); // x
  real_T* px = mxGetPr(prhs[2]);
  px[0] = (real_T) mode[0];
  if (num_xc>0) memcpy(&px[1],x,sizeof(real_T)*num_xc); 

  prhs[3] = mxCreateDoubleMatrix(num_u, 1, mxREAL);  // u
  if (num_u>0) memcpy(mxGetPr(prhs[3]), u, sizeof(real_T)*num_u);

  if (mexCallMATLAB(1,plhs,4,prhs,"guards")) return;
  memcpy(zcs,mxGetPr(plhs[0]),sizeof(real_T)*num_zcs);
  mxDestroyArray(plhs[0]);

  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
}
#endif /* MDL_ZERO_CROSSINGS */

#define MDL_DERIVATIVES 
#if defined(MDL_DERIVATIVES)
static void mdlDerivatives(SimStruct *S)
{
  int num_xc = ssGetNumContStates(S);
  if (num_xc<1) return;
  int num_u = ssGetNumInputPorts(S) ? ssGetInputPortWidth(S,0) : 0;

  setScopeEnable(S);

  mxArray *psys = const_cast<mxArray*>(ssGetSFcnParam(S, 0));
  time_T t = ssGetT(S);
  real_T *x = ssGetContStates(S);
  const real_T *u = num_u>0 ? ssGetInputPortRealSignal(S,0) : NULL;
  real_T *xcdot = ssGetdX(S);
  mxArray* plhs[1];
  bool fsm = isa(psys,"HybridRobotLibSystem");
  int_T *mode = (fsm ? ssGetModeVector(S) : NULL);

  mxArray *prhs[4];
  prhs[0] = psys;                                  // obj
  prhs[1] = mxCreateDoubleScalar(t);               // t

  prhs[2] = mxCreateDoubleMatrix(num_xc + (fsm ? 1 : 0),1,mxREAL); // x
  real_T* px = mxGetPr(prhs[2]);
  if (fsm) { 
    px[0] = (real_T) mode[0];
    memcpy(&px[1],x,sizeof(real_T)*num_xc); 
  } else {    
    memcpy(px,x,sizeof(real_T)*num_xc); 
  }  
  
  prhs[3] = mxCreateDoubleMatrix(num_u,1,mxREAL);  // u
  if (num_u>0) memcpy(mxGetPr(prhs[3]),u,sizeof(real_T)*num_u); 
  
  mexCallMATLAB(1,plhs,4,prhs,"dynamics");
  memcpy(xcdot,mxGetPr(plhs[0]),sizeof(real_T)*num_xc);
  mxDestroyArray(plhs[0]);

  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
}
#endif /* MDL_DERIVATIVES */

static void mdlTerminate(SimStruct *S)
{
}


#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
