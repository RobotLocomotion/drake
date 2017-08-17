#pragma once

#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/common/symbolic.h"
#include "drake/solvers/decision_variable.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mixed_integer_optimization_util.h"

/// @file Functions for reasoning about 3D rotations in a @MathematicalProgram.
///
/// There are a number of choices for representing 3D rotations in a
/// mathematical program -- many of these choices involve using more than
/// the minimal three parameters and therefore require additional constraints
/// For example:
///  - the 4 parameters of a quaternion should form a vector with unit length.
///  - the 9 parameters of a rotation matrix should form a matrix which is
///    orthonormal (R.transpose() = R.inverse()) and det(R)=1.
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
/// Note: The Roll-Pitch-Yaw coordinates follow the convention in @rpy2rotmat -
/// representing extrinsic rotations about Space-fixed x-y-z axes,
/// respectively.
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

using AddRotationMatrixMcCormickEnvelopeReturnType =
std::tuple<std::vector<Matrix3<symbolic::Expression>>,
           std::vector<Matrix3<symbolic::Expression>>,
           std::vector<MatrixDecisionVariable<3, 3>>,
           std::vector<MatrixDecisionVariable<3, 3>>>;
/**
 * Adds binary variables that constrain the value of the column *and* row
 * vectors of R, in order to add the following (in some cases non-convex)
 * constraints as an MILP.  Specifically, for column vectors Ri, we constrain:
 * - forall i, |Ri| = 1 ± envelope,
 * - forall i,j. i ≠ j, Ri.dot(Rj) = 0 ± envelope,
 * - R2 = R0.cross(R1) ± envelope,
 *      and again for R0=R1.cross(R2), and R1=R2.cross(R0).
 * Then all of the same constraints are also added to R^T.  The size of the
 * envelope decreases quickly as num_binary_variables_per_half_axis is
 * is increased.
 *
 * Note: Creates `9*2*num_binary_variables_per_half_axis binary` variables named
 * "BRpos*(*,*)" and "BRneg*(*,*)", and the same number of continuous variables
 * named "CRpos*(*,*)" and "CRneg*(*,*)".
 *
 * Note: The particular representation/algorithm here was developed in an
 * attempt:
 *  - to enable efficient reuse of the variables between the constraints
 *    between multiple rows/columns (e.g. the constraints on R^T use the same
 *    variables as the constraints on R), and
 *  - to facilitate branch-and-bound solution techniques -- binary regions are
 *    layered so that constraining one region establishes constraints
 *    on large portions of SO(3), and confers hopefully "useful" constraints
 *    the on other binary variables.
 * @param prog The mathematical program to which the constraints are added.
 * @param R The rotation matrix
 * @param num_binary_vars_per_half_axis number of binary variables for a half
 * axis.
 * @param limits The angle joints for space fixed z-y-x representation of the
 * rotation. @default is no constraint. @see RollPitchYawLimitOptions
 * @retval NewVars  Included the newly added variables
 * <CRpos, CRneg, BRpos, BRneg>. All new variables can only take values either
 * 0 or 1. `CRpos` and `CRneg` are declared as continuous variables, while
 * `BRpos` and `BRneg` are declared as binary variables.
 * The definition for these variables are
 * <pre>
 *   CRpos[k](i, j) = 1 => k / N <= R(i, j) <= (k+1) / N
 *   CRneg[k](i, j) = 1 => -(k+1) / N <= R(i, j) <= -k / N
 *   BRpos[k](i, j) = 1 => R(i, j) >= k / N
 *   BRneg[k](i, j) = 1 => R(i, j) <= -k / N
 * </pre>
 * where `N` is `num_binary_vars_per_half_axis`.
 */

AddRotationMatrixMcCormickEnvelopeReturnType
AddRotationMatrixMcCormickEnvelopeMilpConstraints(
    MathematicalProgram* prog,
    const Eigen::Ref<const MatrixDecisionVariable<3, 3>>& R,
    int num_binary_vars_per_half_axis = 2,
    RollPitchYawLimits limits = kNoLimits);

/**
 * The return types from AddRotationMatrixBilinearMcComickMilpConstraints.
 * We return both the newly added binary variables, and the vector φ.
 * @see AddRotationMatrixBilinearMcCormickMilpConstraints for more details.
 * @tparam kNumIntervalsPerHalfAxis Number of intervals per half axis in the
 * compile time.
 */
template <int kNumIntervalsPerHalfAxis>
struct AddRotationMatrixBilinearMcCormickMilpConstraintsReturn {
  static constexpr int kPhiRows = kNumIntervalsPerHalfAxis == Eigen::Dynamic
                                     ? Eigen::Dynamic
                                     : 1 + 2 * kNumIntervalsPerHalfAxis;
  static constexpr int kNumBinaryVars =
      kPhiRows == Eigen::Dynamic ? Eigen::Dynamic : CeilLog2(kPhiRows - 1);
  typedef std::array<std::array<VectorDecisionVariable<kNumBinaryVars>, 3>, 3>
      BinaryVarType;
  typedef Eigen::Matrix<double, kPhiRows, 1> PhiType;
  typedef std::pair<BinaryVarType, PhiType> type;
};

/**
 * Relax the SO(3) constraint on a rotation matrix R = [R₀ R₁ R₂]
 * <pre>
 * Rᵢᵀ * Rᵢ = 1
 * Rᵢᵀ * Rⱼ = 0
 * Rᵢ x Rⱼ = Rₖ
 * </pre>
 * to a set of mixed-integer linear constraints. This is achieved by replacing
 * the bilinear product terms in the SO(3) constraint, with a new auxiliary
 * variable in the McCormick envelope of bilinear product. For more details,
 * please refer to
 * Global Inverse Kinematics via Mixed-Integer Convex Optimization
 * by Hongkai Dai, Gregory Izatt and Russ Tedrake, 2017.
 * @tparam kNumIntervalsPerHalfAxis We cut the interval [-1, 1] evenly into
 * KNumIntervalsPerHalfAxis * 2 intervals. Then depending on in which interval
 * R(i, j) is, we impose corresponding linear constraints. Currently only
 * certain kNumIntervalsPerHalfAxis are accepted, including Eigen::Dynamic, and
 * integer 1 to 4. If the user wants to support more kNumIntervalsPerHalfAxis,
 * please instantiate them explicitly in rotation_constraint.cc.
 * @param prog The optimization program to which the SO(3) relaxation is added.
 * @param R The rotation matrix.
 * @param num_intervals_per_half_axis Same as NumIntervalsPerHalfAxis, use this
 * variable when NumIntervalsPerHalfAxis is dynamic.
 * @return pair. pair = (B, φ). B[i][j] is a column vector. If B[i][j]
 * represents integer M in the reflected Gray code, then R(i, j) is in the
 * interval [φ(M), φ(M+1)]. φ contains the end points of the all the intervals,
 * namely φ(i) = -1 + 1 / num_intervals_per_half_axis * i.
 */
template <int kNumIntervalsPerHalfAxis = Eigen::Dynamic>
typename std::enable_if<
    kNumIntervalsPerHalfAxis == Eigen::Dynamic ||
        (kNumIntervalsPerHalfAxis >= 1 && kNumIntervalsPerHalfAxis <= 4),
    typename AddRotationMatrixBilinearMcCormickMilpConstraintsReturn<
        kNumIntervalsPerHalfAxis>::type>::type
AddRotationMatrixBilinearMcCormickMilpConstraints(
    MathematicalProgram* prog,
    const Eigen::Ref<const MatrixDecisionVariable<3, 3>>& R,
    int num_intervals_per_half_axis = kNumIntervalsPerHalfAxis);
}  // namespace solvers
}  // namespace drake
