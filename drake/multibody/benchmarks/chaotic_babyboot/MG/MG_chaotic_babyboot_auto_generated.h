// GENERATED FILE DO NOT EDIT
// -----------------------------------------------------------------------------
// File: MGChaoticBabyboot.h created Aug 30 2017 by MotionGenesis 5.9.
// Portions copyright (c) 2009-2017 Motion Genesis LLC. are licensed under
// the 3-clause BSD license.  https://opensource.org/licenses/BSD-3-Clause
// This copyright notice must appear in all copies and distributions.
// MotionGenesis Professional Licensee: Toyota Research Institute.
//------------------------------------------------------------------------------
#include <cstdio>

//------------------------------------------------------------------------------
#ifndef MGCHAOTICBABYBOOT_
#define MGCHAOTICBABYBOOT_

//------------------------------------------------------------------------------
namespace MotionGenesis {
namespace MGChaoticBabyboot_ {
static const int myNumberOfODEs = 4;
static const int myErrorBufferSize = 9;


//------------------------------------------------------------------------------
class MGIntegrator {
 public:
  // User-defined numerical integrator data.
  double tInitial, tFinal, tStepMax, absError, relError;

 protected:
  // Pure virtual methods that must be overridden in derived class.
  virtual const char*  MGeqns( double t, double VAR[], double VARp[], bool isIntegratorBoundary ) = 0;
  virtual void  CalculateOutput( double t ) = 0;

  // Numerical integration constructor and methods that are only called by derived class.
           MGIntegrator() { SetErrorMessagesToNull(); }
  virtual ~MGIntegrator() {};
  bool  IntegrateForwardOrBackward( double varArrayToIntegrate[] );
  const char*  GetErrorMessage( const int i )  { return (i >= 0 && i < myErrorBufferSize) ? myErrorMessages[i] : NULL; }

 private:
  // Private numerical integrator methods.
  double  KMIntegrator( double y[], double tStart, double* hEntry );
  bool    IntegratorStep( double y[], double* t, double tStep, bool calcDerivOrIntegrate );

  // Private numerical integrator data.
  double mySmallestAllowableStepsize, myPreviousStepsize;

  // Method and data for error messages.
  void  SetErrorMessagesToNull()  { for(int i=0;  i < myErrorBufferSize;  i++)  myErrorMessages[i] = NULL; }
  void  AddErrorMessage( const char* errorMessage )  { for(int i=0;  i < myErrorBufferSize;  i++) if( !myErrorMessages[i] ) {myErrorMessages[i] = errorMessage; break;} }
  const char* myErrorMessages[myErrorBufferSize];
};


//------------------------------------------------------------------------------
class MGChaoticBabyboot : public MGIntegrator {
 public:
  // Constructor with built-in numerical integrator.
  MGChaoticBabyboot() : myNumberOfCallsToMGeqns(0) { SetInputValues(); }

  // This method starts the simulation - and returns false if algorithm fails.
  bool  MGSimulate( void )
  {
    double varArrayToIntegrate[myNumberOfODEs];
    SetArrayFromVariables( varArrayToIntegrate );
    return IntegrateForwardOrBackward( varArrayToIntegrate );
  }

  // User-defined class data for this simulation.
  double g, IAx, IBx, IBy, IBz, LA, LB, mA, mB;
  double qA, qB, qAp, qBp;
  double qApp, qBpp, Energy, KE, PE;
  double Output[8];

 protected:
  // Override pure virtual methods in base class MGIntegrator.
  const char*  MGeqns( double t, double VAR[], double VARp[], bool isIntegratorBoundary );
  void  CalculateOutput( double t );

 private:
  // Private methods for simulation.
  void  SetInputValues();
  void  SetArrayFromVariables( double VAR[] );
  void  SetDerivativeArray( double VARp[] );
  void  SetVariablesFromArray( const double VAR[] );
  void  CalculateQuantitiesThatDependOnTXEtc( double t, bool isIntegratorBoundary );

  // Private data for simulation.
  unsigned long myNumberOfCallsToMGeqns;
};


//------------------------------------------------------------------------------
}        // End of namespace MGChaoticBabyboot_.
}        // End of namespace MotionGenesis.

//------------------------------------------------------------------------------
#endif   // End of #ifndef MGCHAOTICBABYBOOT_


