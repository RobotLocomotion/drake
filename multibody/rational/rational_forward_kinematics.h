#pragma once

#include <limits>
#include <unordered_map>
#include <vector>

#include "drake/common/symbolic/expression.h"
#include "drake/common/symbolic/polynomial.h"
#include "drake/common/symbolic/rational_function.h"
#include "drake/common/symbolic/trigonometric_polynomial.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/mobilizer.h"
#include "drake/multibody/tree/multibody_tree.h"

namespace drake {
namespace multibody {

/** For certain robots (whose joint transforms are algebraic functions of joint
 variables, for example revolute/prismatic/floating-base joints), we can
 represent the pose (position, orientation) of each body, as rational
 functions, namely n(s) / d(s) where both the numerator n(s) and denominator
 d(s) are polynomials of s, and s is some variable related to the generalized
 position.

 One example is that for a rotation matrix with angle θ and axis a, the
 rotation matrix can be written as I + sinθ A + (1-cosθ) A², where A is the
 skew-symmetric matrix from axis a. We can use the stereographic projection to
 substitute the trigonometric function sinθ and cosθ as
 cosθ = cos(θ*+Δθ) = cos(θ*)•cos(Δθ) - sin(θ*)•sin(Δθ)
      = (1-s²)/(1+s²) cos(θ*)- 2s/(1+s²) sin(θ*)     (1)
 sinθ = sin(θ*+Δθ) = sin(θ*)•cos(Δθ) - cos(θ*)•sin(Δθ)
      = (1-s²)/(1+s²) sin(θ*)- 2s/(1+s²) cos(θ*)     (2)
 where θ* is the angle around which we take the stereographic projection.
 θ = θ*+Δθ, and s = tan(Δθ/2).
 With (1) and (2), both sinθ and cosθ are written as a rational function of s.
 Thus the rotation matrix can be written as rational functions of s.

 We will use q* to denote the nominal posture. For a revolute joint, the
 q* is the variable θ*. Note that the stereographic projection has a
 singularity at θ = θ* ± π, as s = tan(Δθ/2)=tan(±π/2) is ±infinity.

 For prismatic joint, we define s = d - d*, where d is the displacement of the
 joint, and d* is the joint value in q*.

 Currently we only support robots with revolute, prismatic and weld joints.

 Throughout this file, we use the following convention
 1. q denotes the generalized position of the entire robot. It includes the
 angle of a revolute joint, the displacement of a prismatic joint, etc.
 2. θ denotes the revolute joint angle.
 3. t=tan((θ-θ*)/2)
 4. s is the vector of variables, such that the robot body pose is written as
 an algebraic function (rational function) of s.
 */
class RationalForwardKinematics {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RationalForwardKinematics);

  /** @param plant The plant for which we compute forward kinematics.
   `plant` should outlive this %RationalForwardKinematics object.
   */
  explicit RationalForwardKinematics(const MultibodyPlant<double>* plant);

  /** This is a proxy for math::RigidTransform. It captures the rigid pose of
   one frame w.r.t another.
   @tparam T The scalar type, which must be one of double,
   symbolic::Expression, symbolic::Polynomial or symbolic::RationalFunction.
   */
  template <typename T>
  struct Pose {
    // I do not use RigidTransform<T> here because we want to support
    // T=symbolic::Polynomial or T=symbolic::RationalFunction, which haven't
    // been fully tested with RigidTransform.
    Vector3<T> position;
    Matrix3<T> rotation;
  };

  /** Computes the pose X_AB as a multilinear polynomial function. The
   indeterminates of the polynomials are cos_delta() and sin_delta().
   To compute the pose as a rational function of indeterminates s(), we
   recommend to first call this function to compute the pose as a multilinear
   polynomial functions, and then call
   ConvertMultilinearPolynomialToRationalFunction() to convert these polynomials
   to rational function with indeterminates s().
   @param q_star The nominal posture q*.
   @param body_index Frame B, the body whose pose is computed.
   @param expressed_body_index Frame A, the body in whose frame the pose is
   expressed.
   @return X_AB The pose of body `body_index` expressed in
   `expressed_body_index`.
   */
  // TODO(hongkai.dai): support computing the pose of any arbitrary frame
  // (instead of only the body frames).
  [[nodiscard]] Pose<symbolic::Polynomial> CalcBodyPoseAsMultilinearPolynomial(
      const Eigen::Ref<const Eigen::VectorXd>& q_star, BodyIndex body_index,
      BodyIndex expressed_body_index) const;

  /** Given a polynomial whose indeterminates are cos_delta() and sin_delta()
   (typically this polynomial is obtained from calling
   CalcBodyPoseAsMultilinearPolynomial()), converts this polynomial to a
   rational function with indeterminates s().
   */
  [[nodiscard]] symbolic::RationalFunction
  ConvertMultilinearPolynomialToRationalFunction(
      const symbolic::Polynomial& e) const;

  /** Computes values of s from q_val and q_star_val, while handling the index
   matching between q and s (we don't guarantee that s(i) is computed from
   q(i)).
   @param angles_wrap_to_inf If set to True, then for a revolute joint whose (θ
   −θ*) >=π/2 (or <= −π/2), we set the corresponding s to ∞ (or −∞), @default is
   false.
   */
  template <typename Derived>
  [[nodiscard]] std::enable_if_t<is_eigen_vector<Derived>::value,
                                 VectorX<typename Derived::Scalar>>
  ComputeSValue(const Eigen::MatrixBase<Derived>& q_val,
                const Eigen::Ref<const Eigen::VectorXd>& q_star_val,
                bool angles_wrap_to_inf = false) const {
    VectorX<typename Derived::Scalar> s_val(s_.size());
    using T = typename Derived::Scalar;
    for (int i = 0; i < s_val.size(); ++i) {
      const internal::Mobilizer<double>& mobilizer =
          GetInternalTree(plant_).get_mobilizer(
              map_s_to_mobilizer_.at(s_[i].get_id()));
      // the mobilizer cannot be a weld joint since weld joint doesn't introduce
      // a variable into s_.
      if (IsRevolute(mobilizer)) {
        const int q_index = mobilizer.position_start_in_q();
        const auto delta = (q_val(q_index) - q_star_val(q_index)) / 2;
        if (angles_wrap_to_inf) {
          if (delta >= T(M_PI / 2)) {
            s_val(i) = T(std::numeric_limits<double>::infinity());
          } else if (delta <= -T(M_PI / 2)) {
            s_val(i) = T(-std::numeric_limits<double>::infinity());
          } else {
            s_val(i) = tan(delta);
          }
        } else {
          s_val(i) = tan(delta);
        }
      } else if (IsPrismatic(mobilizer)) {
        const int q_index = mobilizer.position_start_in_q();
        s_val(i) = q_val(q_index) - q_star_val(q_index);
      } else {
        // Successful construction guarantees nothing but supported mobilizer
        // types.
        DRAKE_UNREACHABLE();
      }
    }
    return s_val;
  }

  /** Computes values of q from s_val and q_star_val, while handling the index
   matching between q and s (we don't guarantee that s(i) is computed from
   q(i)).
   */
  template <typename Derived>
  [[nodiscard]] std::enable_if_t<is_eigen_vector<Derived>::value,
                                 VectorX<typename Derived::Scalar>>
  ComputeQValue(const Eigen::MatrixBase<Derived>& s_val,
                const Eigen::Ref<const Eigen::VectorXd>& q_star_val) const {
    VectorX<typename Derived::Scalar> q_val(s_.size());
    for (int i = 0; i < s_val.size(); ++i) {
      const internal::Mobilizer<double>& mobilizer =
          GetInternalTree(plant_).get_mobilizer(
              map_s_to_mobilizer_.at(s_[i].get_id()));
      // the mobilizer cannot be a weld joint since weld joint doesn't introduce
      // a variable into s_.
      const int q_index = mobilizer.position_start_in_q();
      using std::atan2;
      using std::pow;
      if (IsRevolute(mobilizer)) {
        q_val(q_index) =
            atan2(2 * s_val(i), 1 - pow(s_val(i), 2)) + q_star_val(q_index);
      } else if (IsPrismatic(mobilizer)) {
        q_val(q_index) = s_val(i) + q_star_val(q_index);
      } else {
        // Successful construction guarantees nothing but supported mobilizer
        // types.
        DRAKE_UNREACHABLE();
      }
    }
    return q_val;
  }

  const MultibodyPlant<double>& plant() const { return plant_; }

  Eigen::Map<const VectorX<symbolic::Variable>> s() const {
    return EigenMapView(s_);
  }

  /** map_mobilizer_to_s_index_[mobilizer_index] returns the starting index of
   the mobilizer's variable in s_ (the variable will be contiguous in s for
   the same mobilizer). If this mobilizer doesn't have a variable in s_ (like
   the weld joint), then the s index is set to -1.
   */
  [[nodiscard]] const std::vector<int> map_mobilizer_to_s_index() const {
    return map_mobilizer_to_s_index_;
  }

 private:
  /* Computes the pose of a body, connected to its parent body through a
  revolute joint.
  We will first compute the body pose as multilinear polynomials, with
  indeterminates cos_delta and sin_delta, representing cos(Δθ) and sin(Δθ)
  respectively. We will then replace cos_delta and sin_delta in the body
  pose with rational functions (1-s^2)/(1+s^2) and 2s/(1+s^2) respectively.
  The reason why we don't use RationalFunction directly, is that currently
  our rational function can't find the common factor in the denominator,
  namely the sum between rational functions p1(x) / (q1(x) * r(x)) + p2(x) /
  r(x) is computed as (p1(x) * r(x) + p2(x) * q1(x) * r(x)) / (q1(x) * r(x) *
  r(x)), without handling the common factor r(x) in the denominator.
  We use P to denote the parent frame. F is the inboard frame. M is the outboard
  frame, and C is the child frame.
  @param axis_F The revolute axis in the frame F.
  @param X_PF The pose of frame F in P.
  @param X_MC The pose of frame C in M.
  @param X_AP The pose of the parent frame P in A.
  @param theta_star θ* as explained in the class documentation.
  @param cos_delta cos(Δθ)
  @param sin_delta sin(Δθ)
  @param[out] X_AC The child pose in A.
  */
  template <typename T>
  Pose<T> CalcRevoluteJointChildBodyPoseAsMultilinearPolynomial(
      const Eigen::Ref<const Eigen::Vector3d>& axis_F,
      const math::RigidTransformd& X_PF, const math::RigidTransformd& X_MC,
      const Pose<T>& X_AP, double theta_star,
      const symbolic::Variable& cos_delta,
      const symbolic::Variable& sin_delta) const;

  // Computes the pose of the body, connected to its parent body through a
  // weld joint.
  template <typename T>
  Pose<T> CalcWeldJointChildBodyPose(const math::RigidTransformd& X_PF,
                                     const math::RigidTransformd& X_MC,
                                     const Pose<T>& X_AP) const;

  // Computes the pose of the link, connected to its parent link through a
  // prismatic joint. The displacement of the prismatic joint is d_star + s
  // along `axis_F`.
  template <typename T>
  Pose<T> CalcPrismaticJointChildLinkPose(
      const Eigen::Ref<const Eigen::Vector3d>& axis_F,
      const math::RigidTransformd& X_PF, const math::RigidTransformd& X_MC,
      const Pose<T>& X_AP, double d_star, const symbolic::Variable& d) const;
  /* Given the pose of the parent frame X_AP measured in a frame A, calculates
   the pose of `child` measured as expressed in frame A as a multilinear
   polynomial, with indeterminates being s. s includes s_q and c_q for
   revolute joint, where s_q represents sin(q - q*) and c_q represents
   cos(q - q*), and other variable for other types of joints (like
   prismatic/floating-base joints).

   We explain the terminology on `parent` and `child`, which is different
   from that in MultibodyTreeTopology. If we designate a certain body as the
   root body, then between two adjacent bodies, the one closer to this root
   body is the parent, the one farther from the root body is the child.
   MultibodyTreeTopology usually designates the world as the root body, but we
   don't follow this convention here, and we regard the body attached with the
   expressed frame A as the root body, hence the parent/child relationship
   might be flipped from that defined in MultibodyTreeTopology.
   This convention is used to be consistent with the motivating paper "Finding
   and Optimizing Certified, Collision-Free Regions in Configuration Space for
   Robot Manipulators", where we often need to compute the pose of the world
   body expressed in a robot body (rather than computing the pose of a robot
   body in the world).
   */
  Pose<symbolic::Polynomial> CalcChildBodyPoseAsMultilinearPolynomial(
      const Eigen::Ref<const Eigen::VectorXd>& q_star, BodyIndex parent,
      BodyIndex child, const Pose<symbolic::Polynomial>& X_AP) const;

  /* Determines whether the current joint is revolute. */
  bool IsRevolute(const internal::Mobilizer<double>& mobilizer) const;

  /* Determines whether the current joint is a weld. */
  bool IsWeld(const internal::Mobilizer<double>& mobilizer) const;

  /* Determines whether the current joint is prismatic. */
  bool IsPrismatic(const internal::Mobilizer<double>& mobilizer) const;

  const MultibodyPlant<double>& plant_;
  // The variables used in computing the pose as rational functions. s_ are the
  // indeterminates in the rational functions.
  std::vector<symbolic::Variable> s_;
  // Each s(i) is associated with a mobilizer.
  std::unordered_map<symbolic::Variable::Id, internal::MobodIndex>
      map_s_to_mobilizer_;
  // map_mobilizer_to_s_index_[mobilizer_index] returns the starting index of
  // the mobilizer's variable in s_ (the variable will be contiguous in s for
  // the same mobilizer). If this mobilizer doesn't have a variable in s_ (like
  // the weld joint), then the s index is set to -1.
  std::vector<int> map_mobilizer_to_s_index_;

  // The variables used to represent tan(Δθ / 2). Note that s_angles_ only
  // include s for the revolute joint, it doesn't include s for other joint
  // types (like prismatic joints). See map_s_index_to_angle_index_ and
  // map_angle_to_s_index_ below for how to relate s_angles_ to s.
  std::vector<symbolic::Variable> s_angles_;
  std::vector<symbolic::SinCos> sin_cos_;
  symbolic::Variables sin_cos_set_;
  VectorX<symbolic::Polynomial> one_plus_s_angles_squared_;
  VectorX<symbolic::Polynomial> two_s_angles_;
  VectorX<symbolic::Polynomial> one_minus_s_angles_squared_;

  std::vector<symbolic::Variable> cos_delta_;
  std::vector<symbolic::Variable> sin_delta_;
  // s_ could contain both prismatic s and revolute s.
  // s_angles_[map_s_index_to_angle_index_[i]] = s_[i]
  std::unordered_map<int, int> map_s_index_to_angle_index_;
  symbolic::Variables s_variables_;
  symbolic::Variables s_angle_variables_;
};
}  // namespace multibody
}  // namespace drake
