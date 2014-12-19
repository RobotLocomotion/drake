#ifndef  __CONSTRUCTPTRRIGIDBODYCONSTRAINT_H__
#define  __CONSTRUCTPTRRIGIDBODYCONSTRAINT_H__
#include "mex.h"
#include <Eigen/Dense>

#if defined(WIN32) || defined(WIN64)
  #if defined(drakeRigidBodyConstraint_EXPORTS)
    #define CONSTRAINT_DLLEXPORT __declspec( dllexport )
  #else
    #define CONSTRAINT_DLLEXPORT __declspec( dllimport )
  #endif
#else
  #define CONSTRAINT_DLLEXPORT
#endif

CONSTRAINT_DLLEXPORT mxArray* createDrakeConstraintMexPointer(void* ptr, const char* name);
CONSTRAINT_DLLEXPORT void rigidBodyConstraintParseTspan(const mxArray* pm,Eigen::Vector2d &tspan);
CONSTRAINT_DLLEXPORT void rigidBodyConstraintParse3dUnitVector(const mxArray* pm, Eigen::Vector3d &unit_vec);
CONSTRAINT_DLLEXPORT void rigidBodyConstraintParseQuat(const mxArray* pm, Eigen::Vector4d &quat);
CONSTRAINT_DLLEXPORT double rigidBodyConstraintParseGazeConethreshold(const mxArray* pm);
CONSTRAINT_DLLEXPORT double rigidBodyConstraintParseGazeThreshold(const mxArray* pm);
#endif
