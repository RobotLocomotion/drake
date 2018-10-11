#pragma once

#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/common/symbolic.h"
#include "drake/solvers/decision_variable.h"
#include "drake/solvers/mathematical_program.h"

/// @file Functions for reasoning about 3D rotations in a @MathematicalProgram.
///
/// There are a number of choices for representing 3D rotations in a
/// mathematical program -- many of these choices involve using more than
/// the minimal three parameters and therefore require additional constraints
/// For example:
///
/// - the 4 parameters of a quaternion should form a vector with unit length.
/// - the 9 parameters of a rotation matrix should form a matrix which is
///   orthonormal (R.transpose() = R.inverse()) and det(R)=1.
///
/// Unfortunately, in the context of mathematical programming, most of these
/// constraints are non-convex.  The methods below include convex relaxations
/// of these non-convex constraints.

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
/// box constraint ensuring all elements are [-1,1], and the linear constraint
/// imposing -1 <= trace(R) <= 3.
MatrixDecisionVariable<3, 3> NewRotationMatrixVars(
    MathematicalProgram* prog, const std::string& name = "R");

enum RollPitchYawLimitOptions {
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
typedef uint32_t RollPitchYawLimits;

/// Applies *very conservative* limits on the entries of R for the cases when
/// rotations can be limited (for instance, if you want to search over
/// rotations, but there is an obvious symmetry in the problem so that e.g.
/// 0 < pitch < PI need not be considered).  A matrix so constrained may still
/// contain rotations outside of this envelope.
/// Note: For simple rotational symmetry over PI, prefer
///   kPitch_NegPI_2_to_PI_2 (over 0_to_PI)
/// because it adds one more constraint (when combined with constraints on roll
/// and yaw).
/// Note: The Roll-Pitch-Yaw angles follow the convention in RollPitchYaw,
/// namely extrinsic rotations about Space-fixed x-y-z axes, respectively.
void AddBoundingBoxConstraintsImpliedByRollPitchYawLimits(
    MathematicalProgram* prog,
    const Eigen::Ref<const MatrixDecisionVariable<3, 3>>& R,
    RollPitchYawLimits limits = kNoLimits);

/// Adds constraint (10) from https://arxiv.org/pdf/1403.4914.pdf ,
/// which exactly represents the convex hull of all rotation matrices in 3D.
void AddRotationMatrixSpectrahedralSdpConstraint(
    MathematicalProgram* prog,
    const Eigen::Ref<const MatrixDecisionVariable<3, 3>>& R);

/// Adds a set of convex constraints which approximate the set of orthogonal
/// matrices, O(3).  Adds the bilinear constraints that the each column Ri has
/// length <= 1 and that Ri'Rj approx 0 via
///    -2 + |Ri|^2 + |Rj|^2 <= 2Ri'Rj <= 2 - |Ri|^2 - |Rj|^2 (for all i!=j),
/// using a second-order-cone relaxation.  Additionally, the same constraints
/// are applied to all of the rows.
void AddRotationMatrixOrthonormalSocpConstraint(
    MathematicalProgram* prog,
    const Eigen::Ref<const MatrixDecisionVariable<3, 3>>& R);

}  // namespace solvers
}  // namespace drake
