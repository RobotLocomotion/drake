#ifndef  __CONSTRUCTPTRRIGIDBODYCONSTRAINT_H__
#define  __CONSTRUCTPTRRIGIDBODYCONSTRAINT_H__
#include "mex.h"
#include <Eigen/Dense>

#if defined(WIN32) || defined(WIN64)
  #if defined(drakeConstructRigidBodyConstraint_EXPORTS)
    #define CONSTRUCT_CONSTRAINT_DLLEXPORT __declspec( dllexport )
  #else
    #define CONSTRUCT_CONSTRAINT_DLLEXPORT __declspec( dllimport )
  #endif
#else
  #define CONSTRUCT_CONSTRAINT_DLLEXPORT
#endif

CONSTRUCT_CONSTRAINT_DLLEXPORT mxArray* createDrakeConstraintMexPointer(void* ptr, const char* name);
CONSTRUCT_CONSTRAINT_DLLEXPORT void rigidBodyConstraintParseTspan(const mxArray* pm,Eigen::Vector2d &tspan);
CONSTRUCT_CONSTRAINT_DLLEXPORT void rigidBodyConstraintParse3dUnitVector(const mxArray* pm, Eigen::Vector3d &unit_vec);
CONSTRUCT_CONSTRAINT_DLLEXPORT void rigidBodyConstraintParseQuat(const mxArray* pm, Eigen::Vector4d &quat);
CONSTRUCT_CONSTRAINT_DLLEXPORT double rigidBodyConstraintParseGazeConethreshold(const mxArray* pm);
CONSTRUCT_CONSTRAINT_DLLEXPORT double rigidBodyConstraintParseGazeThreshold(const mxArray* pm);
#endif
