#pragma once

#include <mex.h>

#include <Eigen/Dense>

DLL_EXPORT_SYM mxArray* createDrakeConstraintMexPointer(void* ptr,
                                                        const char* name);
DLL_EXPORT_SYM void rigidBodyConstraintParseTspan(const mxArray* pm,
                                                  Eigen::Vector2d& tspan);
DLL_EXPORT_SYM void rigidBodyConstraintParse3dUnitVector(
    const mxArray* pm, Eigen::Vector3d& unit_vec);
DLL_EXPORT_SYM void rigidBodyConstraintParseQuat(const mxArray* pm,
                                                 Eigen::Vector4d& quat);
DLL_EXPORT_SYM double rigidBodyConstraintParseGazeConethreshold(
    const mxArray* pm);
DLL_EXPORT_SYM double rigidBodyConstraintParseGazeThreshold(const mxArray* pm);
