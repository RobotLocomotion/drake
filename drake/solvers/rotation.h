#pragma once

#include <string>

#include "drake/solvers/decision_variable.h"
#include "drake/solvers/mathematical_program.h"

// TODO(all): Add PSDlift, more finely parameterized McCormick envelope,
// Principle minor SOCP relaxations...

namespace drake {
namespace solvers {

/// Allocate a 3x3 matrix of decision variables with the trivial bounding
/// box constraint ensuring all elements are [-1,1].
DecisionVariableMatrixX NewRotationMatrixVars(MathematicalProgram* prog,
                                              const std::string& name = "R");

enum RollPitchYawLimits {
  kNoLimits = 0,
  kRPYError = 1 << 0,  ///< Do not use, to avoid & vs. && typos.
  kRoll_NegPI_2_to_PI_2 = 1 << 1,
  kRoll_0_to_PI = 1 << 2,
  kPitch_NegPI_2_to_PI_2 = 1 << 3,
  kPitch_0_to_PI = 1 << 4,
  kYaw_NegPI_2_to_PI_2 = 1 << 5,
  kYaw_0_to_PI = 1 << 6,
  kRoll_0_to_PI_2 = (1 << 1) | (1 << 2),
  kPitch_0_to_PI_2 = (1 << 3) | (1 << 4),
  kYaw_0_to_PI_2 = (1 << 5) | (1 << 6)
};

/// Applies conservative limits on the entries of R for the cases when rotations
/// can be limited.
/// Note: For simple rotational symmetry over PI, prefer
///   kPitch_NegPI_2_to_PI_2 (over 0_to_PI)
/// because it adds one more constraint (when combined with constraints on roll
/// and yaw).
void AddRollPitchYawLimitBoundingBoxConstraints(
    MathematicalProgram* prog, const DecisionVariableMatrixX& R,
    RollPitchYawLimits limits = kNoLimits);

/// Add constraint (10) from https://arxiv.org/pdf/1403.4914.pdf ,
/// which exactly represents the convex hull of all rotation matrices in 3D.
void AddRotationMatrixSpectrahedralSdpConstraint(
    MathematicalProgram* prog, const DecisionVariableMatrixX& R);

/// Adds the bilinear constraints that the each column Ri has length <= 1
/// and that Ri'Rj approx 0 via
///    -2 + |Ri|^2 + |Rj|^2 <= Ri'*Rj <= 2 - |Ri|^2 - |Rj|^2 (for all i!=j),
/// using a second-order-cone relaxation.
/// Note: Consider calling this method twice (once on R and once on R^T).
/// Note: Also creates a dummy slack variable 1 <= one <= 1.
void AddRotationMatrixOrthonormalSocpConstraint(
    MathematicalProgram* prog, const DecisionVariableMatrixX& R);

/// Adds the non-convex constraint that the L1 norm each column Ri is *greater*
/// than 1, e.g. forall i |R1i|+|R2i|+|R3i|>=1, as an MILP.
/// Note: Creates binary variables "bR*".
/// Note: Consider calling this method twice (once on R and once on R^T).
///
/// TODO(russt): Take in RPY limits (to avoid allocating those binary
/// variables).
/// TODO(russt): Take advantage of R3 = cross(R1,R2).
/// TODO(russt): Support tighter constraints -- simply rotate by e.g. PI/4 and
///   apply the constraint again (but it will make a lot of faces!).
void AddRotationMatrixL1NormMilpConstraint(
    MathematicalProgram* prog, const DecisionVariableMatrixX& R,
    RollPitchYawLimits limits = kNoLimits);

}  // namespace solvers
}  // namespace drake
