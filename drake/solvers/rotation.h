#pragma once

#include <string>

#include "drake/solvers/decision_variable.h"
#include "drake/solvers/mathematical_program.h"

// TODO(all): Other concepts to potentially implement:
//  - PSDlift from section 1.2 of https://arxiv.org/pdf/1403.4914.pdf .  This
//    is waiting on the symbolic expression tools.
//  - Principle minor SOCP relaxations of the SDP conditions
//  - MILP constraints that push outside the cube contained by the L2 ball,
//    with vertices at (sqrt(3)/3, sqrt(3)/3, sqrt(3)/3), (-sqrt(3)/3,
//    sqrt(3)/3, sqrt(3)/3), ... this would have 6 faces/binary variables per
//    column of R instead of the 8 in the L1 norm constraint.
//  - Axis angle and/or gaze cone constraints, as mentioned here:
// https://reviewable.io/reviews/robotlocomotion/drake/4653#-K_YGsAOBY5ooX3OBqo7

namespace drake {
namespace solvers {

/// Allocates a 3x3 matrix of decision variables with the trivial bounding
/// box constraint ensuring all elements are [-1,1].
DecisionVariableMatrix<3, 3> NewRotationMatrixVars(
    MathematicalProgram* prog, const std::string& name = "R");

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
    MathematicalProgram* prog,
    const Eigen::Ref<const DecisionVariableMatrix<3, 3>>& R,
    RollPitchYawLimits limits = kNoLimits);

/// Adds constraint (10) from https://arxiv.org/pdf/1403.4914.pdf ,
/// which exactly represents the convex hull of all rotation matrices in 3D.
void AddRotationMatrixSpectrahedralSdpConstraint(
    MathematicalProgram* prog,
    const Eigen::Ref<const DecisionVariableMatrix<3, 3>>& R);

/// Adds the bilinear constraints that the each column Ri has length <= 1
/// and that Ri'Rj approx 0 via
///    -2 + |Ri|^2 + |Rj|^2 <= 2Ri'Rj <= 2 - |Ri|^2 - |Rj|^2 (for all i!=j),
/// using a second-order-cone relaxation.
/// Note: Consider calling this method twice (once on R and once on R^T).
/// Note: Also creates a dummy slack variable 1 <= one <= 1.
void AddRotationMatrixOrthonormalSocpConstraint(
    MathematicalProgram* prog,
    const Eigen::Ref<const DecisionVariableMatrix<3, 3>>& R);

/// Adds binary variables that constrain the value of the column vectors of R
/// based on which orthant of R^3 they lie in, in order to add the following
/// (in some cases non-convex) constraints as an MILP:
///   - the L1 norm of each column Rj is *greater* than 1, and less than
///     sqrt(3), e.g.
///          forall j, 1 <= |R(0,j)|+|R(1,j)|+|R(2,j)| <= sqrt(3).
///   - the Linf norm of each column Rj is less than 1, e.g.
///          forall j, max_i |R(i,j)| <= 1.
///   - R1 cannot lie in the same orthant as R0 nor -R0.
///   - R2 is in the orthant implied by R2 = cross(R0,R1).
///
/// Note: Creates binary variables "bR*".
/// Note: Consider calling this method twice (once on R and once on R^T).
///
/// TODO(russt): Take in RPY limits (to avoid allocating those binary
/// variables).
/// TODO(russt): Support tighter constraints -- simply rotate by e.g. PI/4 and
///   apply the constraint again (but it will make a lot of faces!).
void AddRotationMatrixOrthantMilpConstraints(
    MathematicalProgram* prog,
    const Eigen::Ref<const DecisionVariableMatrix<3, 3>>& R);

}  // namespace solvers
}  // namespace drake
