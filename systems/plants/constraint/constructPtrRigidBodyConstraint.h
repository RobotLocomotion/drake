#ifndef  __CONSTRUCTPTRRIGIDBODYCONSTRAINT_H__
#define  __CONSTRUCTPTRRIGIDBODYCONSTRAINT_H__
#include "mex.h"
#include "drakeUtil.h"
#include "RigidBodyManipulator.h"
#include "RigidBodyConstraint.h"
#include <iostream>
#include <Eigen/Dense>
#include <cstdio>

mxArray* createDrakeConstraintMexPointer(void* ptr, const char* deleteMethod, const char* name);
void rigidBodyConstraintParseTspan(const mxArray* pm,Eigen::Vector2d &tspan);
void rigidBodyConstraintParse3dUnitVector(const mxArray* pm, Vector3d &unit_vec);
void rigidBodyConstraintParseQuat(const mxArray* pm, Vector4d &quat);
double rigidBodyConstraintParseGazeConethreshold(const mxArray* pm);
double rigidBodyConstraintParseGazeThreshold(const mxArray* pm);
#endif
