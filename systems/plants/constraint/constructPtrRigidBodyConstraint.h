#ifndef  __CONSTRUCTPTRRIGIDBODYCONSTRAINT_H__
#define  __CONSTRUCTPTRRIGIDBODYCONSTRAINT_H__
#include "mex.h"
#include <Eigen/Dense>

#if defined(WIN32) || defined(WIN64)
  #if defined(drakeRigidBodyConstraint_EXPORTS)
    #define DLLEXPORT __declspec( dllexport )
  #else
    #define DLLEXPORT __declspec( dllimport )
  #endif
#else
  #define DLLEXPORT
#endif

DLLEXPORT mxArray* createDrakeConstraintMexPointer(void* ptr, const char* name);
DLLEXPORT void rigidBodyConstraintParseTspan(const mxArray* pm,Eigen::Vector2d &tspan);
DLLEXPORT void rigidBodyConstraintParse3dUnitVector(const mxArray* pm, Eigen::Vector3d &unit_vec);
DLLEXPORT void rigidBodyConstraintParseQuat(const mxArray* pm, Eigen::Vector4d &quat);
DLLEXPORT double rigidBodyConstraintParseGazeConethreshold(const mxArray* pm);
DLLEXPORT double rigidBodyConstraintParseGazeThreshold(const mxArray* pm);
#endif
