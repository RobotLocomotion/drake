// GENERATED FILE DO NOT EDIT
// -----------------------------------------------------------------------------
// File: MGChaoticBabyboot.cc created Aug 30 2017 by MotionGenesis 5.9.
// Portions copyright (c) 2009-2017 Motion Genesis LLC. are licensed under
// the 3-clause BSD license.  https://opensource.org/licenses/BSD-3-Clause
// This copyright notice must appear in all copies and distributions.
// MotionGenesis Professional Licensee: Toyota Research Institute.
// -----------------------------------------------------------------------------
#include "drake/multibody/benchmarks/chaotic_babyboot/MG/MG_chaotic_babyboot_auto_generated.h"

#include <cmath>

//------------------------------------------------------------------------------
namespace MotionGenesis {
namespace MGChaoticBabyboot_ {
const double DEGtoRAD =  0.0174532925199432957692369;
const double RADtoDEG = 57.2957795130823208767981548;


// -----------------------------------------------------------------------------
void  MGChaoticBabyboot::SetInputValues()
{
  g              =  9.81;              // m/s^2    Constant
  IAx            =  50;                // g*cm^2   Constant
  IBx            =  2500;              // g*cm^2   Constant
  IBy            =  500;               // g*cm^2   Constant
  IBz            =  2000;              // g*cm^2   Constant
  LA             =  7.5;               // cm       Constant
  LB             =  20;                // cm       Constant
  mA             =  10;                // grams    Constant
  mB             =  100;               // grams    Constant

  qA             =  90;                // deg      Initial Value
  qB             =  1.0;               // deg      Initial Value
  qAp            =  0.0;               // rad/sec  Initial Value of qA'
  qBp            =  0.0;               // rad/sec  Initial Value of qB'

  tInitial       =  0.0;               // sec      Initial time
  tFinal         =  10;                // sec      Final time
  tStepMax       =  0.01;              // sec      Maximum integration step
  absError       =  1.0E-12;           //          Absolute error
  relError       =  1.0E-12;           //          Relative error

  // Unit conversions to UnitSystem( kilogram, meter, second ).
  IAx *= 1.0E-07;                      //  Converted from g*cm^2
  IBx *= 1.0E-07;                      //  Converted from g*cm^2
  IBy *= 1.0E-07;                      //  Converted from g*cm^2
  IBz *= 1.0E-07;                      //  Converted from g*cm^2
  LA *= 0.01;                          //  Converted from cm
  LB *= 0.01;                          //  Converted from cm
  mA *= 0.001;                         //  Converted from grams
  mB *= 0.001;                         //  Converted from grams
  qA *= DEGtoRAD;                      //  Converted from deg
  qB *= DEGtoRAD;                      //  Converted from deg
}


// -----------------------------------------------------------------------------
void  MGChaoticBabyboot::CalculateQuantitiesThatDependOnTXEtc( double t, bool isIntegratorBoundary )
{
  qApp = -(g*(mA*LA+mB*LB)*sin(qA)-2*(IBx-IBy)*sin(qB)*cos(qB)*qAp*qBp)/
          (IAx+IBx+mA*pow(LA,2)+mB*pow(LB,2)-(IBx-IBy)*pow(sin(qB),2));
  qBpp = -(IBx-IBy)*sin(qB)*cos(qB)*pow(qAp,2)/IBz;
}


// -----------------------------------------------------------------------------
void  MGChaoticBabyboot::SetArrayFromVariables( double VAR[myNumberOfODEs] )
{
  VAR[0] = qA;
  VAR[1] = qB;
  VAR[2] = qAp;
  VAR[3] = qBp;
}


// -----------------------------------------------------------------------------
void  MGChaoticBabyboot::SetDerivativeArray( double VARp[myNumberOfODEs] )
{
  VARp[0] = qAp;
  VARp[1] = qBp;
  VARp[2] = qApp;
  VARp[3] = qBpp;
}


// -----------------------------------------------------------------------------
void  MGChaoticBabyboot::SetVariablesFromArray( const double VAR[myNumberOfODEs] )
{
  qA = VAR[0];
  qB = VAR[1];
  qAp = VAR[2];
  qBp = VAR[3];
}


// -----------------------------------------------------------------------------
const char*  MGChaoticBabyboot::MGeqns( double t, double VAR[], double VARp[], bool isIntegratorBoundary )
{
  // Keep track of number of calls to this method.
  ++myNumberOfCallsToMGeqns;

  // Update named variables from variables passed to this method.
  SetVariablesFromArray( VAR );

  // Calculate quantities that depend on t, variables, etc.
  CalculateQuantitiesThatDependOnTXEtc( t, isIntegratorBoundary );

  // Update derivative array prior to integration step.
  SetDerivativeArray( VARp );
  return NULL;
}


// -----------------------------------------------------------------------------
void  MGChaoticBabyboot::CalculateOutput( double t )
{
  KE = 0.5*IAx*pow(qAp,2) + 0.5*IBx*pow(qAp,2) + 0.5*IBz*pow(qBp,2)
     + 0.5*mA*pow(LA,2)*pow(qAp,2) + 0.5*mB*pow(LB,2)*pow(qAp,2)
     - 0.5*(IBx-IBy)*pow(sin(qB),2)*pow(qAp,2);
  PE = -g*(mA*LA+mB*LB)*cos(qA);
  Energy = PE + KE;

  Output[0] = t;
  Output[1] = qA*RADtoDEG;
  Output[2] = qB*RADtoDEG;
  Output[3] = qAp;
  Output[4] = qBp;
  Output[5] = qApp;
  Output[6] = qBpp;
  Output[7] = Energy;
}


// -----------------------------------------------------------------------------
// PURPOSE:  Solve a set of first order ordinary differential equations of    **
//           the form dy(i)/dt = F(t, y(1), ..., y(n)) (i = 1, ..., n)  with  **
//           a Kutta-Merson algorithm that numerically integrates the differ- **
//           ential equations from tStart to tStart + hEntry.                 **
//           (L. Fox, Numerical Solutions of Ordinary and Partial Differ-     **
//           ential Equations, Palo Alto: Addison-Wesley, 1962, pp. 24-25)    **
//                                                                            **
// INPUT:                                                                     **
//   MGeqns: Method that evaluates dy(i)/dt (i = 1, ..., n),                  **
//           the first derivatives of y(1), ..., y(n) with respect to t.      **
//                                                                            **
//        y: One-dimensional array whose elements are y(1), ..., y(n).        **
//                                                                            **
//   tStart: Value of independent variable t on entry to this method.         **
//                                                                            **
//   hEntry: Suggested numerical integration stepsize for this method.        **
//           On exit, hEntry is set to actual stepsize used by integrator.    **
//           If hEntry == NULL, the method MGeqns is called and dy(i)/dt      **
//           (i = 1, ... n) are evaluated, but no integration is performed.   **
//                                                                            **
// OUTPUT:   The actual stepsize used by integrator is returned in hEntry.    **
//           The values of y(i) (i = 1 ... n) at t+hEntry are returned in y.  **
//           The return value is the integrator's estimate of the next step-  **
//           size or 0 if an error was encountered and integration stopped.   **
//                                                                            **
// Copyright (c) 2009-2017 by Paul Mitiguy.  Use permitted under the 3-clause **
//           BSD license: https://opensource.org/licenses/BSD-3-Clause.       **
//           This copyright notice must appear in all copies & distributions. **
//           This particular copyright notice applies only to the methods:    **
//           KMIntegrator, IntegratorStep, IntegrateForwardOrBackward         **
// -----------------------------------------------------------------------------
double  MGIntegrator::KMIntegrator( double y[], double tStart, double* hEntry )
{
  double f0[myNumberOfODEs], f1[myNumberOfODEs], f2[myNumberOfODEs];
  double y1[myNumberOfODEs], y2[myNumberOfODEs];
  double h = hEntry ? *hEntry : 0;               // Dereferenced local copy.
  int   i, errorVarIndex = 0;                    // Variable that failed.
  const char* errorMessage = NULL;

  // Always need to calculate derivatives at tStart (always boundary here).
  // If hEntry == NULL, just call method MGeqns at tStart and return.
  errorMessage = MGeqns(tStart, y, f0, true);    // Integration boundary.
  if( !errorMessage  &&  hEntry == NULL ) return 1;

  while( tStart + h != tStart && !errorMessage )      // Avoid round-off.
  {
    double h2 = h * 0.5;                              // Half    of h.
    double h3 = h / 3.0;                              // Third   of h.
    double h6 = h / 6.0;                              // Sixth   of h.
    double h8 = h * 0.125;                            // Eighth  of h.
    for(i=0; i < myNumberOfODEs; i++)  y1[i] = y[i] + h3*f0[i];
    if( (errorMessage = MGeqns(tStart+h3, y1, f1, false)) != NULL ) break;
    for(i=0; i < myNumberOfODEs; i++)  y1[i] = y[i] + h6*(f0[i] + f1[i]);
    if( (errorMessage = MGeqns(tStart+h3, y1, f1, false)) != NULL ) break;
    for(i=0; i < myNumberOfODEs; i++)  y1[i] = y[i] + h8*(f0[i] + 3*f1[i]);
    if( (errorMessage = MGeqns(tStart+h2, y1, f2, false)) != NULL ) break;
    for(i=0; i < myNumberOfODEs; i++)  y1[i] = y[i] + h2*(f0[i] - 3*f1[i] + 4*f2[i]);
    if( (errorMessage = MGeqns(tStart+h,  y1, f1, false)) != NULL)  break;
    for(i=0; i < myNumberOfODEs; i++)  y2[i] = y[i] + h6*(f0[i] + 4*f2[i] + f1[i]);

    // Both y1[i] and y2[i] provide estimates to the new value of y.
    // Decide if the relative and absolute error tolerances are met.
    // If they are not, reduce the stepsize and restart the integration.
    double errorRatioMax = 0;                         // Error criterion.
    for(i=0; i < myNumberOfODEs; i++)                 // Check all variables.
    {
      double errorInEstimate = 0.2 * fabs( y2[i] - y1[i] );
      double relTest  = fabs( y2[i] ) * relError;
      double largerOfAbsErrOrRelTest = absError > relTest ? absError : relTest;
      double errorRatio = errorInEstimate / largerOfAbsErrOrRelTest;
      if( errorRatio > errorRatioMax ) { errorRatioMax = errorRatio;  errorVarIndex = i; }
    }

    // If the errorRatioMax >= 1, either absError or relTest failed, i.e.,
    // errorInEstimate >= largerOfAbsErrOrRelTest.  Halve stepsize and restart.
    if( errorRatioMax >= 1 )
    {
      if( fabs(h=h2) > fabs(mySmallestAllowableStepsize) )  continue;
      static char stepsizeCutMessage[80];
      sprintf(stepsizeCutMessage, "Error: Stepsize has been cut too many times for variable %d.", errorVarIndex);
      errorMessage = stepsizeCutMessage;  break;
    }

    // Integration succeeded.  Update the values of y and t before return.
    for(i=0; i < myNumberOfODEs; i++)  y[i] = y2[i];

    // Return actual stepsize and estimate for next integration stepsize.
    *hEntry = h;   return errorRatioMax < 1.0/64.0 ? h+h : h;
  }

  // Append error messages that numerical integrator failed.
  if( errorMessage ) AddErrorMessage( errorMessage );
  else AddErrorMessage( "Error: Numerical round-off makes stepsize h too small so (tStart + h == tStart)." );
  static char failureTimeMessage[80];
  sprintf( failureTimeMessage, "Error: Numerical integration failed at t = %15.8E.", tStart );
  AddErrorMessage( failureTimeMessage );

  MGeqns(tStart, y, f0, true);         // Fill for error display.
  return hEntry ? (*hEntry = 0) : 0;
}


// -----------------------------------------------------------------------------
// PURPOSE:  Controls the independent variable, e.g., t while solving a set   **
//           of first order ordinary differential equations of the form       **
//           dy(i)/dt = F(t, y(1), ..., y(n)) (i = 1, ..., n).                **
//           Discontinuities in functions and t should be handled here.       **
//                                                                            **
// INPUT:                                                                     **
//       t:  Independent variable.                                            **
//       y:  One-dimensional array whose elements are y(1), ..., y(n).        **
//   tStep:  Maximum integration stepsize for this step.                      **
//           If isFirstCall = true, the method MGeqns is called and dy(i)/dt  **
//           (i = 1, ..., n) are evaluated, but no integration is performed.  **
//           If isFirstCall = false, integration is performed.                **
//                                                                            **
// OUTPUT:   The value of  t + tStep  is returned in t.                       **
//           The values of y(i) at  t + tStep  are returned in y.             **
//           The return value is false if the algorithm fails.                **
// -----------------------------------------------------------------------------
bool  MGIntegrator::IntegratorStep( double y[], double* t, double tStep, bool isFirstCall )
{
  double h;                                     // Current stepsize.
  double hAccumulated = 0;                      // How far to tStep.

  // If isFirstCall, just call method MGeqns at exactly *t.
  if( isFirstCall ) return (bool)KMIntegrator( y, *t, NULL );

  // Upon entry, initialize the current stepsize to the stored value of hc.
  if( tStep == 0 || absError==0 ) return false; // Cannot integrate.
  if( fabs(myPreviousStepsize) > fabs(tStep) ) myPreviousStepsize = tStep;
  h = myPreviousStepsize;                       // Current stepsize.

  // Make as many little integration steps as necessary to get to end.
  // Each integration step starts at *t and ends at *t + h.
  while( *t + h != *t )                         // Check round-off problems.
  {
    // Numerically integrate y[i] and maybe get shorter value of h.
    // Set hNext to integrator's estimate of the next integration stepsize
    // or 0 if an error was encountered and integration stopped.
    double hToIntegrator = h;                  // Suggested stepsize.
    double hNext = KMIntegrator( y, *t, &h );
    bool isIntegrationFinished = fabs( (hAccumulated+=h) + mySmallestAllowableStepsize) > fabs(tStep);
    if( h==0 || hNext==0 ) return false;       // Integration failed.

    // Any time/function discontinuities should be handled here.
    // Increment value of t using stepsize returned by the integrator.
    *t += h;                                   // Prepare for next step.

    // Update the value of myPreviousStepsize for next call to integrator.
    // Always change the stepsize if not taking the last step.
    // Always reduce the stepsize if the integrator reduced the stepsize.
    // Could always increase stepsize if integrator suggests larger one,
    // e.g., add  || fabs(hNext) > fabs(myPreviousStepsize)   to logic below.
    if( !isIntegrationFinished || fabs(hNext) < fabs(hToIntegrator) || fabs(h) < fabs(hToIntegrator) )  h = myPreviousStepsize = hNext;

    // If finished, calculate derivatives of y at the final value of *t.
    if( isIntegrationFinished ) return (bool)KMIntegrator( y, *t, NULL );

    // Integrator may suggest a stepsize that is bigger than allowed.
    // If next jump is past or very close to end of tStep, adjust h.
    if( fabs(hAccumulated + 1.1*h) > fabs(tStep) )  h = tStep - hAccumulated;
  }
  AddErrorMessage( "Error: Numerical round-off makes stepsize h too small so (t + h == t)." );
  return false;
}


// -----------------------------------------------------------------------------
// PURPOSE:  Numerically integrates from tInitial to tFinal a set of first-   **
//           order ordinary differential equations of the form                **
//           dy(i)/dt = F(t, y(1), ..., y(n)) (i = 1, ..., n).                **
//                                                                            **
// INPUT:    varArrayToIntegrate contains the values of y(1), ... y(n) at     **
//           t = tInitial (it is a n-dimensional array of initial values).    **
//                                                                            **
// OUTPUT:   If integration succeeds, varArrayToIntegrate contains the values **
//           of y(1), ..., y(n) at t = tFinal and the method returns true.    **
//           If integration fails, varArrayToIntegrate contains the values    **
//           of y(1), ..., y(n) at the value of t at which integration failed.**
// -----------------------------------------------------------------------------
bool  MGIntegrator::IntegrateForwardOrBackward( double varArrayToIntegrate[], double& t )
{
  mySmallestAllowableStepsize = 1.0E-7 * (myPreviousStepsize = tStepMax);
  double tFinalMinusEpsilon = tFinal - mySmallestAllowableStepsize;
  const bool isIntegrateForward = (tFinal >= tInitial);

  // Initialize numerical integrator at t = tInitial, thereafter integrate.
  bool isIntegrationFinished = false, isFirstCall = true, isIntegrationOK = true;
  while( !isIntegrationFinished )
  {
    const bool isFullStep = (isIntegrateForward && t+tStepMax <= tFinal) ||
                           (!isIntegrateForward && t+tStepMax >= tFinal);
    const double tStepMaxThisStep = isFullStep ? tStepMax : tFinal - t;
    isIntegrationOK = IntegratorStep( varArrayToIntegrate, &t, tStepMaxThisStep, isFirstCall );
    isIntegrationFinished = !isIntegrationOK || (isIntegrateForward && t > tFinalMinusEpsilon) ||
                                               (!isIntegrateForward && t < tFinalMinusEpsilon);
    isFirstCall = false;
  }

  if( !isIntegrationOK ) AddErrorMessage( "Error: Numerical method failed to converge.\n" );
  return isIntegrationOK;
}


//------------------------------------------------------------------------------
}        // End of namespace MGChaoticBabyboot_.
}        // End of namespace MotionGenesis.

