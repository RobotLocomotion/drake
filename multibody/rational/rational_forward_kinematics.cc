#include "drake/multibody/rational/rational_forward_kinematics.h"

#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/multibody/rational/rational_forward_kinematics_internal.h"
#include "drake/multibody/tree/prismatic_mobilizer.h"
#include "drake/multibody/tree/revolute_mobilizer.h"
#include "drake/multibody/tree/weld_mobilizer.h"

namespace drake {
namespace multibody {
namespace {

RationalForwardKinematics::Pose<symbolic::Polynomial> GetIdentityPose() {
  RationalForwardKinematics::Pose<symbolic::Polynomial> pose;
  const symbolic::Polynomial poly_zero{0};
  const symbolic::Polynomial poly_one{1};
  // clang-format off
  pose.rotation <<
    poly_one, poly_zero, poly_zero,
    poly_zero, poly_one, poly_zero,
    poly_zero, poly_zero, poly_one;
  pose.position<< poly_zero, poly_zero, poly_zero;
  // clang-format on
  return pose;
}

/*
 * Compute the child body pose in frame A given the parent body pose (R_AP,
 * p_AP) and the mobilizer transform.
 * P is the parent frame, F is the inboard frame, M is the outboard frame and C
 * is the child frame. In this case, "parent" and "child" is agnostic of
 * MultibodyTree's or RationalForwardKinematics's definition; this function
 * simply combines transforms based on the caller's provided descriptions.
 */
template <typename Scalar1, typename Scalar2>
RationalForwardKinematics::Pose<Scalar2> CalcChildPose(
    const Matrix3<Scalar2>& R_AP, const Vector3<Scalar2>& p_WP,
    const math::RigidTransform<double>& X_PF,
    const math::RigidTransform<double>& X_MC, const Matrix3<Scalar1>& R_FM,
    const Vector3<Scalar1>& p_FM) {
  // Frame F is the inboard frame (attached to the parent body), and frame
  // M is the outboard frame (attached to the child body).
  const Matrix3<Scalar2> R_AF = R_AP * X_PF.rotation().matrix();
  const Vector3<Scalar2> p_AF = R_AP * X_PF.translation() + p_WP;
  const Matrix3<Scalar2> R_AM = R_AF * R_FM;
  const Vector3<Scalar2> p_AM = R_AF * p_FM + p_AF;
  const Matrix3<double> R_MC = X_MC.rotation().matrix();
  const Vector3<double> p_MC = X_MC.translation();
  RationalForwardKinematics::Pose<Scalar2> X_AC;
  X_AC.rotation = R_AM * R_MC;
  X_AC.position = R_AM * p_MC + p_AM;
  return X_AC;
}
}  // namespace

RationalForwardKinematics::RationalForwardKinematics(
    const MultibodyPlant<double>* plant)
    : plant_(DRAKE_DEREF(plant)) {
  const internal::MultibodyTree<double>& tree = GetInternalTree(plant_);
  const internal::SpanningForest& forest = tree.forest();
  // Initialize map_mobilizer_to_s_index_ to -1, where -1 indicates "no s
  // variable".
  map_mobilizer_to_s_index_ = std::vector<int>(tree.num_mobilizers(), -1);
  for (BodyIndex body_index(1); body_index < plant_.num_bodies();
       ++body_index) {
    // Note: we're assuming a 1:1 correspondence between bodies and mobilizers.
    // This won't work if there are merged link composites since multiple
    // bodies will map to the same mobilizer.
    const internal::MobodIndex mobod_index =
        forest.link_by_index(body_index).mobod_index();
    const internal::SpanningForest::Mobod& mobod = forest.mobods(mobod_index);
    // Confirm that there is only one body following this Mobod.
    DRAKE_DEMAND(ssize(mobod.follower_link_ordinals()) == 1);
    const internal::Mobilizer<double>* mobilizer =
        &(tree.get_mobilizer(mobod_index));
    if (IsRevolute(*mobilizer)) {
      const symbolic::Variable s_angle(fmt::format("s[{}]", s_.size()));
      s_.push_back(s_angle);
      s_angles_.push_back(s_angle);
      cos_delta_.emplace_back(fmt::format("cos_delta[{}]", cos_delta_.size()));
      sin_delta_.emplace_back(fmt::format("sin_delta[{}]", sin_delta_.size()));
      sin_cos_.emplace_back(sin_delta_.back(), cos_delta_.back());
      sin_cos_set_.insert(sin_delta_.back());
      sin_cos_set_.insert(cos_delta_.back());
      map_s_index_to_angle_index_.emplace(s_.size() - 1, s_angles_.size() - 1);
      map_s_to_mobilizer_.emplace(s_.back().get_id(), mobilizer->index());
      map_mobilizer_to_s_index_[mobilizer->index()] = s_.size() - 1;
    } else if (IsWeld(*mobilizer)) {
      // Do nothing for a weld joint.
    } else if (IsPrismatic(*mobilizer)) {
      s_.emplace_back("s[" + std::to_string(s_.size()) + "]");
      map_mobilizer_to_s_index_[mobilizer->index()] = s_.size() - 1;
      map_s_to_mobilizer_.emplace(s_.back().get_id(), mobilizer->index());
    } else {
      throw std::runtime_error(
          "RationalForwardKinematics: encountered an unidentified, unsupported "
          "Joint type; only weld, revolute and prismatic joints are "
          "supported.");
    }
  }
  const symbolic::Monomial monomial_one{};
  one_plus_s_angles_squared_.resize(s_angles_.size());
  two_s_angles_.resize(s_angles_.size());
  one_minus_s_angles_squared_.resize(s_angles_.size());
  for (int i = 0; i < static_cast<int>(s_angles_.size()); ++i) {
    one_minus_s_angles_squared_(i) = symbolic::Polynomial(
        {{monomial_one, 1}, {symbolic::Monomial(s_angles_[i], 2), -1}});
    two_s_angles_(i) =
        symbolic::Polynomial({{symbolic::Monomial(s_angles_[i], 1), 2}});
    one_plus_s_angles_squared_(i) = symbolic::Polynomial(
        {{monomial_one, 1}, {symbolic::Monomial(s_angles_[i], 2), 1}});
  }
  s_angle_variables_ = symbolic::Variables(EigenMapView(s_angles_));
  s_variables_ = symbolic::Variables(EigenMapView(s_));
}

RationalForwardKinematics::Pose<symbolic::Polynomial>
RationalForwardKinematics::CalcBodyPoseAsMultilinearPolynomial(
    const Eigen::Ref<const Eigen::VectorXd>& q_star, BodyIndex body_index,
    BodyIndex expressed_body_index) const {
  // First find the path from expressed_body_index to body_index.
  const std::vector<BodyIndex> path =
      internal::FindPath(plant_, expressed_body_index, body_index);
  std::vector<Pose<symbolic::Polynomial>> poses;
  poses.reserve(path.size());
  poses.push_back(GetIdentityPose());
  for (int i = 1; i < static_cast<int>(path.size()); ++i) {
    poses.push_back(CalcChildBodyPoseAsMultilinearPolynomial(
        q_star, path[i - 1], path[i], poses[i - 1]));
  }
  return std::move(poses.back());
}

symbolic::RationalFunction
RationalForwardKinematics::ConvertMultilinearPolynomialToRationalFunction(
    const symbolic::Polynomial& e) const {
  const symbolic::RationalFunction e_rational =
      symbolic::internal::SubstituteStereographicProjectionImpl(
          e, sin_cos_, sin_cos_set_,
          Eigen::Map<const VectorX<symbolic::Variable>>(s_angles_.data(),
                                                        s_angles_.size()),
          s_variables_, one_plus_s_angles_squared_, two_s_angles_,
          one_minus_s_angles_squared_);
  return e_rational;
}

template <typename T>
RationalForwardKinematics::Pose<T> RationalForwardKinematics::
    CalcRevoluteJointChildBodyPoseAsMultilinearPolynomial(
        const Eigen::Ref<const Eigen::Vector3d>& axis_F,
        const math::RigidTransformd& X_PF, const math::RigidTransformd& X_MC,
        const Pose<T>& X_AP, double theta_star,
        const symbolic::Variable& cos_delta_theta,
        const symbolic::Variable& sin_delta_theta) const {
  // clang-format off
      const Eigen::Matrix3d A_F =
          (Eigen::Matrix3d() << 0, -axis_F(2), axis_F(1),
                                axis_F(2), 0, -axis_F(0),
                                -axis_F(1), axis_F(0), 0).finished();
  // clang-format on
  const symbolic::Variables cos_sin_delta({cos_delta_theta, sin_delta_theta});
  const double cos_theta_star = cos(theta_star);
  const double sin_theta_star = sin(theta_star);
  const symbolic::Polynomial cos_angle(
      {{symbolic::Monomial(cos_delta_theta, 1), cos_theta_star},
       {symbolic::Monomial(sin_delta_theta, 1), -sin_theta_star}});
  const symbolic::Polynomial sin_angle(
      {{symbolic::Monomial(cos_delta_theta, 1), sin_theta_star},
       {symbolic::Monomial(sin_delta_theta, 1), cos_theta_star}});
  // Frame F is the inboard frame (attached to the parent body), and frame
  // M is the outboard frame (attached to the child body).
  const Matrix3<symbolic::Polynomial> R_FM = Eigen::Matrix3d::Identity() +
                                             sin_angle * A_F +
                                             (1 - cos_angle) * A_F * A_F;
  const symbolic::Polynomial poly_zero{};
  const Vector3<symbolic::Polynomial> p_FM(poly_zero, poly_zero, poly_zero);
  const Matrix3<T>& R_AP = X_AP.rotation;
  const Vector3<T>& p_AP = X_AP.position;
  return CalcChildPose(R_AP, p_AP, X_PF, X_MC, R_FM, p_FM);
}

template <typename T>
RationalForwardKinematics::Pose<T>
RationalForwardKinematics::CalcWeldJointChildBodyPose(
    const math::RigidTransformd& X_PF, const math::RigidTransformd& X_MC,
    const Pose<T>& X_AP) const {
  // X_FM is always identity for a Weld mobilizer.
  const Matrix3<double> R_FM = Matrix3<double>::Identity();
  const Vector3<double> p_FM = Vector3<double>::Zero();
  const Matrix3<T>& R_AP = X_AP.rotation;
  const Vector3<T>& p_AP = X_AP.position;
  return CalcChildPose(R_AP, p_AP, X_PF, X_MC, R_FM, p_FM);
}

template <typename T>
RationalForwardKinematics::Pose<T>
RationalForwardKinematics::CalcPrismaticJointChildLinkPose(
    const Eigen::Ref<const Eigen::Vector3d>& axis_F,
    const math::RigidTransformd& X_PF, const math::RigidTransformd& X_MC,
    const Pose<T>& X_AP, double d_star, const symbolic::Variable& d) const {
  const symbolic::Polynomial poly_one(1);
  const symbolic::Polynomial poly_zero(0);
  Matrix3<symbolic::Polynomial> R_FM;
  // clang-format off
  R_FM << poly_one, poly_zero, poly_zero,
          poly_zero, poly_one, poly_zero,
          poly_zero, poly_zero, poly_one;
  // clang-format on
  const Vector3<symbolic::Polynomial> p_FM =
      axis_F * symbolic::Polynomial(d_star + d);
  const Matrix3<T>& R_AP = X_AP.rotation;
  const Vector3<T>& p_AP = X_AP.position;
  return CalcChildPose(R_AP, p_AP, X_PF, X_MC, R_FM, p_FM);
}

RationalForwardKinematics::Pose<symbolic::Polynomial>
RationalForwardKinematics::CalcChildBodyPoseAsMultilinearPolynomial(
    const Eigen::Ref<const Eigen::VectorXd>& q_star, BodyIndex parent,
    BodyIndex child,
    const RationalForwardKinematics::Pose<symbolic::Polynomial>& X_AP) const {
  // if child was a child of parent in the
  // original tree before changing the root, then is_order_reversed = false;
  // otherwise it is true.
  // If we denote the frames related to the two adjacent bodies connected
  // by a mobilizer in the original tree as P->F->M->C, then after reversing
  // the order, the new frames should reverse the order, namely
  // P' = C, F' = M, M' = F, C' = P, and hence we know that
  // X_P'F' = X_MC.inverse()
  // X_F'M' = X_FM.inverse()
  // X_M'C' = X_PF.inverse()
  const internal::MultibodyTree<double>& tree = GetInternalTree(plant_);
  const internal::SpanningForest& forest = tree.forest();

  const internal::MobodIndex parent_mobod_index =
      forest.link_by_index(parent).mobod_index();
  const internal::SpanningForest::Mobod& parent_mobod =
      forest.mobods(parent_mobod_index);

  const internal::MobodIndex child_mobod_index =
      forest.link_by_index(child).mobod_index();
  const internal::SpanningForest::Mobod& child_mobod =
      forest.mobods(child_mobod_index);

  internal::MobodIndex mobilizer_index;
  bool is_order_reversed{};
  if (parent_mobod.inboard().is_valid() &&
      parent_mobod.inboard() == child_mobod_index) {
    is_order_reversed = true;
    mobilizer_index = parent_mobod_index;
  } else if (child_mobod.inboard().is_valid() &&
             child_mobod.inboard() == parent_mobod_index) {
    is_order_reversed = false;
    mobilizer_index = child_mobod_index;
  } else {
    throw std::invalid_argument(fmt::format(
        "CalcChildBodyPoseAsMultilinearPolynomial: the pair of body indices "
        "({}, {}) do not have a parent-child relationship.",
        parent, child));
  }
  const internal::Mobilizer<double>* mobilizer =
      &(tree.get_mobilizer(mobilizer_index));
  math::RigidTransformd X_PF, X_MC;
  if (!is_order_reversed) {
    X_PF = mobilizer->inboard_frame().GetFixedPoseInBodyFrame();
    X_MC = mobilizer->outboard_frame().GetFixedPoseInBodyFrame().inverse();
  } else {
    X_PF = mobilizer->outboard_frame().GetFixedPoseInBodyFrame();
    X_MC = mobilizer->inboard_frame().GetFixedPoseInBodyFrame().inverse();
  }
  if (IsRevolute(*mobilizer)) {
    // A revolute joint.
    const internal::RevoluteMobilizer<double>* revolute_mobilizer =
        static_cast<const internal::RevoluteMobilizer<double>*>(mobilizer);
    const int s_index = map_mobilizer_to_s_index_[mobilizer->index()];
    const int q_index = revolute_mobilizer->position_start_in_q();
    const int s_angle_index = map_s_index_to_angle_index_.at(s_index);
    Eigen::Vector3d axis_F;
    if (!is_order_reversed) {
      axis_F = revolute_mobilizer->revolute_axis();
    } else {
      // By negating the revolute axis, we know that R(a, θ)⁻¹ = R(-a, θ)
      axis_F = -revolute_mobilizer->revolute_axis();
    }
    return CalcRevoluteJointChildBodyPoseAsMultilinearPolynomial(
        axis_F, X_PF, X_MC, X_AP, q_star(q_index), cos_delta_[s_angle_index],
        sin_delta_[s_angle_index]);
  } else if (IsPrismatic(*mobilizer)) {
    // A prismatic joint.
    const internal::PrismaticMobilizer<double>* prismatic_mobilizer =
        static_cast<const internal::PrismaticMobilizer<double>*>(mobilizer);
    const int s_index =
        map_mobilizer_to_s_index_.at(prismatic_mobilizer->index());
    Eigen::Vector3d axis_F;
    if (!is_order_reversed) {
      axis_F = prismatic_mobilizer->translation_axis();
    } else {
      axis_F = -prismatic_mobilizer->translation_axis();
    }
    return CalcPrismaticJointChildLinkPose(axis_F, X_PF, X_MC, X_AP,
                                           q_star(s_index), s_[s_index]);
  } else if (IsWeld(*mobilizer)) {
    return CalcWeldJointChildBodyPose(X_PF, X_MC, X_AP);
  }
  // Successful construction guarantees that all supported mobilizers are
  // handled.
  DRAKE_UNREACHABLE();
}

// TODO(hongkai.dai): determine the joint type through a Reifier.
bool RationalForwardKinematics::IsRevolute(
    const internal::Mobilizer<double>& mobilizer) const {
  const bool is_revolute =
      (mobilizer.num_positions() == 1 && mobilizer.num_velocities() == 1 &&
       mobilizer.can_rotate() && !mobilizer.can_translate());
  if (is_revolute) {
    DRAKE_THROW_UNLESS(dynamic_cast<const internal::RevoluteMobilizer<double>*>(
                           &mobilizer) != nullptr);
  }
  return is_revolute;
}

// TODO(hongkai.dai): determine the joint type through a Reifier.
bool RationalForwardKinematics::IsWeld(
    const internal::Mobilizer<double>& mobilizer) const {
  const bool is_weld =
      (mobilizer.num_positions() == 0 && mobilizer.num_velocities() == 0 &&
       !mobilizer.can_rotate() && !mobilizer.can_translate());
  if (is_weld) {
    DRAKE_THROW_UNLESS(dynamic_cast<const internal::WeldMobilizer<double>*>(
                           &mobilizer) != nullptr);
  }
  return is_weld;
}

// TODO(hongkai.dai): determine the joint type through a Reifier.
bool RationalForwardKinematics::IsPrismatic(
    const internal::Mobilizer<double>& mobilizer) const {
  const bool is_prismatic =
      (mobilizer.num_positions() == 1 && mobilizer.num_velocities() == 1 &&
       !mobilizer.can_rotate() && mobilizer.can_translate());
  if (is_prismatic) {
    DRAKE_THROW_UNLESS(
        dynamic_cast<const internal::PrismaticMobilizer<double>*>(&mobilizer) !=
        nullptr);
  }
  return is_prismatic;
}

}  // namespace multibody
}  // namespace drake
