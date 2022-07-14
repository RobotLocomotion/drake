#pragma once

#include <unordered_map>
#include <vector>

#include "drake/common/symbolic/expression.h"
#include "drake/common/symbolic/polynomial.h"
#include "drake/common/symbolic/rational_function.h"
#include "drake/common/symbolic/trigonometric_polynomial.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/multibody_tree.h"

namespace drake {
namespace multibody {
/**
 * For certain robots (whose joint transforms are algebraic functions of joint
 * variables, for example revolute/prismatic/floating-base joints), we can
 * represent the pose (position, orientation) of each link, as rational
 * functions, namely n(s) / d(s) where both the numerator n(s) and denominator
 * d(s) are polynomials of s, and s is some variable related to the generalized
 * position.
 *
 * One example is that for a rotation matrix with angle θ and axis a, the
 * rotation matrix can be written as I + sinθ A + (1-cosθ) A², where A is the
 * skew-symmetric matrix from axis a. We can use the stereographic projection to
 * substitute the trigonometric function sinθ and cosθ as
 * cosθ = cos(θ*+Δθ) = cos(θ*)•cos(Δθ) - sin(θ*)•sin(Δθ)
 *      = (1-s²)/(1+s²) cos(θ*)- 2s/(1+s²) sin(θ*)     (1)
 * sinθ = sin(θ*+Δθ) = sin(θ*)•cos(Δθ) - cos(θ*)•sin(Δθ)
 *      = (1-s²)/(1+s²) sin(θ*)- 2s/(1+s²) cos(θ*)     (2)
 * where θ* is the angle around which we take the stereographic projection.
 * θ = θ*+Δθ, and s = tan(Δθ/2).
 * With (1) and (2), both sinθ and cosθ are written as a rational function of s.
 * Thus the rotation matrix can be written as rational functions of s.
 *
 * We will use q_star to denote the nominal posture. For a revolute joint, the
 * q_star is the variable θ*. Note that the stereographic projection has a
 * singularity at θ = θ* ± π, as s = tan(Δθ/2) is infinity.
 *
 * Throughout this file, we use the following convention
 * 1. q denotes the generalized position of the entire robot. It includes the
 * angle of a revolute joint, the displacement of a prismatic joint, etc.
 * 2. θ denotes the revolute joint angle.
 * 3. t=tan((θ-θ*)/2)
 * 4. s is the vector of variables, such that the robot link pose is written as
 * an algebraic function (rational function) of s.
 */
class RationalForwardKinematics {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RationalForwardKinematics)

  /**
   * @param plant The plant for which we compute its forward kinematics. `plant`
   * should outlive this RationalForwardKinematics object.
   */
  explicit RationalForwardKinematics(const MultibodyPlant<double>* plant);

  /**
   * @tparam T can be double, symbolic::Expression or symbolic::Polynomial.
   */
  template <typename T>
  struct Pose {
    // I do not use RigidTransform<T> here because we want to support
    // T=symbolic::Polynomial or T=symbolic::RationalFunction, which haven't
    // been fully tested with RigidTransform.
    Vector3<T> p_AB;
    Matrix3<T> R_AB;
  };

  /**
   * Computes the pose X_AB as a multilinear polynomial function. The
   * indeterminates of the polynomials are cos_delta and sin_delta() .
   * We will later convert these polynomials to rational function with
   * indeterminates s().
   * @param q_star The nominal posture
   * @param link_index Frame B, the link whose pose is computed.
   * @param expressed_body_index Frame A, the link in whose frame the pose is
   * expressed.
   * @return pose The pose of link `link_index` expressed in
   * `expressed_body_index`.
   */
  [[nodiscard]] Pose<symbolic::Polynomial> CalcLinkPoseAsMultilinearPolynomial(
      const Eigen::Ref<const Eigen::VectorXd>& q_star, BodyIndex link_index,
      BodyIndex expressed_body_index) const;

  /**
   * Given a polynomial whose indeterminates are cos_delta() and sin_delta()
   * (typically this polynomial is obtained from calling
   * CalcLinkPoseAsMultilinearPolynomial()), converts this polynomial to a
   * rational function with indeterminates s().
   */
  [[nodiscard]] symbolic::RationalFunction
  ConvertMultilinearPolynomialToRationalFunction(
      const symbolic::Polynomial& e) const;

  /**
   * Computes s from q_val and q_star_val, while handling the index
   * matching between q and s (we don't guarantee that s(i) is computed from
   * q(i)). If @p clamp_angle = true and the joint is revolute, then s =
   * infinity if q_val >= q_star_val + pi, and s = -infinity if q_val <=
   * q_star_val - pi.
   */
  [[nodiscard]] Eigen::VectorXd ComputeSValue(
      const Eigen::Ref<const Eigen::VectorXd>& q_val,
      const Eigen::Ref<const Eigen::VectorXd>& q_star_val,
      bool clamp_angle = false) const;

  const MultibodyPlant<double>& plant() const { return *plant_; }

  const VectorX<symbolic::Variable>& s() const { return s_; }

  /**
   * Each s(i) is associated with a mobilizer. We return the mapping from the s
   * variable to the mobilizer.
   */
  const std::unordered_map<symbolic::Variable::Id, internal::MobilizerIndex>&
  map_s_to_mobilizer() const {
    return map_s_to_mobilizer_;
  }

  /**
   * The symbolic variables representing cos(Δθ) for all revolute joint angles
   * θ.
   */
  const VectorX<symbolic::Variable>& cos_delta() const { return cos_delta_; }

  /**
   * The symbolic variables representing sin(Δθ) for all revolute joint angles
   * θ.
   */
  const VectorX<symbolic::Variable>& sin_delta() const { return sin_delta_; }

 private:
  // Computes the pose of the link, connected to its parent link through a
  // revolute joint.
  // We will first compute the link pose as multilinear polynomials, with
  // indeterminates cos_delta and sin_delta, representing cos(Δθ) and sin(Δθ)
  // respectively. We will then replace cos_delta and sin_delta in the link
  // pose with rational functions (1-s^2)/(1+s^2) and 2s/(1+s^2) respectively.
  // The reason why we don't use RationalFunction directly, is that currently
  // our rational function can't find the common factor in the denominator,
  // namely the sum between rational functions p1(x) / (q1(x) * r(x)) + p2(x) /
  // r(x) is computed as (p1(x) * r(x) + p2(x) * q1(x) * r(x)) / (q1(x) * r(x) *
  // r(x)), without handling the common factor r(x) in the denominator.
  template <typename T>
  void CalcRevoluteJointChildLinkPoseAsMultilinearPolynomial(
      const Eigen::Ref<const Eigen::Vector3d>& axis_F,
      const math::RigidTransformd& X_PF, const math::RigidTransformd& X_MC,
      const Pose<T>& X_AP, double theta_star,
      const symbolic::Variable& cos_delta, const symbolic::Variable& sin_delta,
      Pose<T>* X_AC) const;

  // Computes the pose of the link, connected to its parent link through a
  // weld joint.
  template <typename T>
  void CalcWeldJointChildLinkPose(const math::RigidTransformd& X_FM,
                                  const math::RigidTransformd& X_PF,
                                  const math::RigidTransformd& X_MC,
                                  const Pose<T>& X_AP, Pose<T>* X_AC) const;

  /**
   * Given the pose of the parent frame X_AP measured in a frame A, calculates
   * the pose of `child` measured as expressed in frame A as a multilinear
   * polynomial, with indeterminates being s. s includes s_q and c_q for
   * revolute joint, where s_q represents sin(q - q_star) and c_q represents
   * cos(q - q_star), and other variable for other types of joints (like
   * prismatic/floating-base joints).
   *
   * We explain the terminology on @p parent and @p child, which is different
   * from that in MultibodyTreeTopology. If we designate a certain link as the
   * root link, then between two adjacent links, the one closer to this root
   * link is the parent, the one farther from the root link is the child.
   * MultibodyTreeTopology usually designates the world as the root link, but we
   * don't follow this convention here, and we regard the link attached with the
   * expressed frame A as the root link, hence the parent/child relationship
   * might be flipped from that defined in MultibodyTreeTopology.
   */
  void CalcChildLinkPoseAsMultilinearPolynomial(
      const Eigen::Ref<const Eigen::VectorXd>& q_star, BodyIndex parent,
      BodyIndex child, const Pose<symbolic::Polynomial>& X_AP,
      Pose<symbolic::Polynomial>* X_AC) const;

  const MultibodyPlant<double>* plant_;
  // The variables used in computing the pose as rational functions. s_ are the
  // indeterminates in the rational functions.
  VectorX<symbolic::Variable> s_;
  // Each s(i) is associated with a mobilizer.
  std::unordered_map<symbolic::Variable::Id, internal::MobilizerIndex>
      map_s_to_mobilizer_;
  // Given a mobilizer, returns the index of the mobilizer's variable in
  // s_.
  std::unordered_map<internal::MobilizerIndex, int> map_mobilizer_to_s_index_;

  // The variables used to represent tan(Δθ / 2). Note that s_angles_ only
  // include s for the revolute joint, it doesn't include s for other joint
  // types (like prismatic joints).
  VectorX<symbolic::Variable> s_angles_;
  std::vector<symbolic::SinCos> sin_cos_;
  VectorX<symbolic::Polynomial> one_plus_s_angles_squared_;
  VectorX<symbolic::Polynomial> two_s_angles_;
  VectorX<symbolic::Polynomial> one_minus_s_angles_squared_;

  VectorX<symbolic::Variable> cos_delta_;
  VectorX<symbolic::Variable> sin_delta_;
  // s_ could contain both prismatic s and revolute s.
  // TODO(hongkai.dai): support prismatic joint.
  // s_angles_[map_s_index_to_angle_index_[i]] = s_[i]
  std::unordered_map<int, int> map_s_index_to_angle_index_;
  // s_[map_angle_index_to_s_index_[i]] = s_angles_[i]
  std::unordered_map<int, int> map_angle_index_to_s_index_;
  // s_id_to_index_(s_(i).get_id()) = i
  std::unordered_map<symbolic::Variable::Id, int> s_id_to_index_;
  symbolic::Variables s_variables_;
};
}  // namespace multibody
}  // namespace drake
