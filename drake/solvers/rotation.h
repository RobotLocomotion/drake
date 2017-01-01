#pragma once

#include <string>

#include "drake/solvers/decision_variable.h"
#include "drake/solvers/mathematical_program.h"

// TODO(all): Add PSDlift, McCormick envelope, PSDlift w/ SOCP relaxation...

namespace drake {
namespace solvers {

/// Add constraint (10) from https://arxiv.org/pdf/1403.4914.pdf ,
/// which exactly represents the convex hull of all rotation matrices in 3D.
DecisionVariableMatrixX NewRotationMatrixSpectrahedralSdpRelaxation(
    MathematicalProgram* prog, const std::string& name = "R");
  
/// Creates a rotation matrix R = [R1, R2, R3], and adds the bilinear constraints
/// that the each column Ri has length <= 1, R2'*R1 = 0, and R3 = cross(R1, R2)
/// using a second-order-cone relaxation.
DecisionVariableMatrixX NewRotationMatrixOrthonormalSocpRelaxation(
    MathematicalProgram* prog, const std::string& name = "R");
  
}  // namespace solvers
}  // namespace drake
