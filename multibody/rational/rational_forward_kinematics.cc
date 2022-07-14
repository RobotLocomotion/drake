#include "drake/multibody/rational/rational_forward_kinematics.h"

#include <limits>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/multibody/rational/rational_forward_kinematics_internal.h"
#include "drake/multibody/tree/mobilizer.h"
#include "drake/multibody/tree/multibody_tree_topology.h"
#include "drake/multibody/tree/prismatic_mobilizer.h"
#include "drake/multibody/tree/revolute_mobilizer.h"
#include "drake/multibody/tree/space_xyz_floating_mobilizer.h"
#include "drake/multibody/tree/space_xyz_mobilizer.h"

namespace drake {
namespace multibody {
namespace {

void SetPoseToIdentity(
    RationalForwardKinematics::Pose<symbolic::Polynomial>* pose) {
  const symbolic::Polynomial poly_zero{0};
  const symbolic::Polynomial poly_one{1};
  // clang-format off
  pose->rotation <<
    poly_one, poly_zero, poly_zero,
    poly_zero, poly_one, poly_zero,
    poly_zero, poly_zero, poly_one;
  pose->position<< poly_zero, poly_zero, poly_zero;
  // clang-format on
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
    : plant_(plant) {
  DRAKE_DEMAND(plant_ != nullptr);
  int num_s = 0;
  const internal::MultibodyTree<double>& tree = GetInternalTree(*plant_);
  for (BodyIndex body_index(1); body_index < plant_->num_bodies();
       ++body_index) {
    const internal::BodyTopology& body_topology =
        tree.get_topology().get_body(body_index);
    const internal::Mobilizer<double>* mobilizer =
        &(tree.get_mobilizer(body_topology.inboard_mobilizer));
    // TODO(hongkai.dai): do not use the dynamic_cast, but implement this as a
    // virtual function in the parent mobilizer class.
    if (dynamic_cast<const internal::RevoluteMobilizer<double>*>(mobilizer) !=
        nullptr) {
      const symbolic::Variable s_angle(fmt::format("s[{}]", num_s));
      s_.conservativeResize(s_.rows() + 1);
      s_angles_.conservativeResize(s_angles_.rows() + 1);
      cos_delta_.conservativeResize(cos_delta_.rows() + 1);
      sin_delta_.conservativeResize(sin_delta_.rows() + 1);
      s_(s_.rows() - 1) = s_angle;
      s_angles_(s_angles_.rows() - 1) = s_angle;
      cos_delta_(cos_delta_.rows() - 1) = symbolic::Variable(
          fmt::format("cos_delta[{}]", cos_delta_.rows() - 1));
      sin_delta_(sin_delta_.rows() - 1) = symbolic::Variable(
          fmt::format("sin_delta[{}]", sin_delta_.rows() - 1));
      sin_cos_.emplace_back(sin_delta_(sin_delta_.rows() - 1),
                            cos_delta_(cos_delta_.rows() - 1));
      num_s += 1;
      map_s_index_to_angle_index_.emplace(s_.rows() - 1, s_angles_.rows() - 1);
      map_angle_index_to_s_index_.emplace(s_angles_.rows() - 1, s_.rows() - 1);
      map_s_to_mobilizer_.emplace(s_(s_.rows() - 1).get_id(),
                                  mobilizer->index());
      map_mobilizer_to_s_index_.emplace(mobilizer->index(), s_.rows() - 1);
      s_id_to_index_.emplace(s_angle.get_id(), s_.rows() - 1);
    } else if (dynamic_cast<const internal::WeldMobilizer<double>*>(
                   mobilizer) != nullptr) {
    } else if (dynamic_cast<const internal::SpaceXYZMobilizer<double>*>(
                   mobilizer) != nullptr) {
      // TODO(hongkai.dai): throw the error "RationalForwardKinematics:
      // encountered an unidentified, unsupported Joint type; only weld and
      // revolute joints are supported." for all unsuppored joint types.
      throw std::runtime_error("BallRpyJoint has not been handled yet.");
    } else if (dynamic_cast<const internal::PrismaticMobilizer<double>*>(
                   mobilizer) != nullptr) {
      throw std::runtime_error("Prismatic joint has not been handled yet.");
    }
  }
  const symbolic::Monomial monomial_one{};
  one_plus_s_angles_squared_.resize(s_angles_.rows());
  two_s_angles_.resize(s_angles_.rows());
  one_minus_s_angles_squared_.resize(s_angles_.rows());
  for (int i = 0; i < s_angles_.rows(); ++i) {
    one_minus_s_angles_squared_(i) = symbolic::Polynomial(
        {{monomial_one, 1}, {symbolic::Monomial(s_angles_(i), 2), -1}});
    two_s_angles_(i) =
        symbolic::Polynomial({{symbolic::Monomial(s_angles_(i), 1), 2}});
    one_plus_s_angles_squared_(i) = symbolic::Polynomial(
        {{monomial_one, 1}, {symbolic::Monomial(s_angles_(i), 2), 1}});
  }
  s_variables_ = symbolic::Variables(s_);
}

RationalForwardKinematics::Pose<symbolic::Polynomial>
RationalForwardKinematics::CalcBodyPoseAsMultilinearPolynomial(
    const Eigen::Ref<const Eigen::VectorXd>& q_star, BodyIndex body_index,
    BodyIndex expressed_body_index) const {
  // First find the path from expressed_body_index to body_index.
  const std::vector<BodyIndex> path =
      internal::FindPath(*plant_, expressed_body_index, body_index);
  std::vector<Pose<symbolic::Polynomial>> poses(path.size());
  SetPoseToIdentity(&(poses[0]));
  for (int i = 1; i < static_cast<int>(path.size()); ++i) {
    poses[i] = CalcChildBodyPoseAsMultilinearPolynomial(q_star, path[i - 1],
                                                        path[i], poses[i - 1]);
  }
  return std::move(poses.back());
}

symbolic::RationalFunction
RationalForwardKinematics::ConvertMultilinearPolynomialToRationalFunction(
    const symbolic::Polynomial& e) const {
  const symbolic::RationalFunction e_rational =
      symbolic::internal::SubstituteStereographicProjectionImpl(
          e, sin_cos_, s_angles_, s_variables_, one_plus_s_angles_squared_,
          two_s_angles_, one_minus_s_angles_squared_);
  return e_rational;
}

Eigen::VectorXd RationalForwardKinematics::ComputeSValue(
    const Eigen::Ref<const Eigen::VectorXd>& q_val,
    const Eigen::Ref<const Eigen::VectorXd>& q_star_val,
    bool clamp_angle) const {
  Eigen::VectorXd s_val(s_.size());
  const double kInf = std::numeric_limits<double>::infinity();
  for (int i = 0; i < s_val.size(); ++i) {
    const internal::Mobilizer<double>& mobilizer =
        GetInternalTree(*plant_).get_mobilizer(
            map_s_to_mobilizer_.at(s_(i).get_id()));
    // the mobilizer cannot be a weld joint since weld joint doesn't introduce a
    // variable into s_.
    if (dynamic_cast<const internal::RevoluteMobilizer<double>*>(&mobilizer) !=
        nullptr) {
      const int q_index = mobilizer.position_start_in_q();
      s_val(i) = std::tan((q_val(q_index) - q_star_val(q_index)) / 2);
      if (clamp_angle) {
        if (q_val(q_index) - q_star_val(q_index) >= M_PI) {
          s_val(i) = kInf;
        } else if (q_val(q_index) - q_star_val(q_index) <= -M_PI) {
          s_val(i) = -kInf;
        }
      }
    } else {
      throw std::runtime_error(
          "Only revolute and weld joints are currently supported.");
    }
  }
  return s_val;
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
    const math::RigidTransformd& X_FM, const math::RigidTransformd& X_PF,
    const math::RigidTransformd& X_MC, const Pose<T>& X_AP) const {
  const Matrix3<double> R_FM = X_FM.rotation().matrix();
  const Vector3<double> p_FM = X_FM.translation();
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
  const internal::MultibodyTree<double>& tree = GetInternalTree(*plant_);
  const internal::BodyTopology& parent_topology =
      tree.get_topology().get_body(parent);
  const internal::BodyTopology& child_topology =
      tree.get_topology().get_body(child);
  internal::MobilizerIndex mobilizer_index;
  bool is_order_reversed;
  if (parent_topology.parent_body.is_valid() &&
      parent_topology.parent_body == child) {
    is_order_reversed = true;
    mobilizer_index = parent_topology.inboard_mobilizer;
  } else if (child_topology.parent_body.is_valid() &&
             child_topology.parent_body == parent) {
    is_order_reversed = false;
    mobilizer_index = child_topology.inboard_mobilizer;
  } else {
    throw std::invalid_argument(fmt::format(
        "CalcChildLinkPoseAsMultilinearPolynomial: the pair of body indices "
        "({}, {}) do not have a parent-child relationship.",
        parent, child));
  }
  const internal::Mobilizer<double>* mobilizer =
      &(tree.get_mobilizer(mobilizer_index));
  if (dynamic_cast<const internal::RevoluteMobilizer<double>*>(mobilizer) !=
      nullptr) {
    // A revolute joint.
    const internal::RevoluteMobilizer<double>* revolute_mobilizer =
        dynamic_cast<const internal::RevoluteMobilizer<double>*>(mobilizer);
    const int s_index = map_mobilizer_to_s_index_.at(mobilizer->index());
    const int q_index = revolute_mobilizer->position_start_in_q();
    const int s_angle_index = map_s_index_to_angle_index_.at(s_index);
    Eigen::Vector3d axis_F;
    math::RigidTransformd X_PF, X_MC;
    if (!is_order_reversed) {
      axis_F = revolute_mobilizer->revolute_axis();
      const Frame<double>& frame_F = mobilizer->inboard_frame();
      const Frame<double>& frame_M = mobilizer->outboard_frame();
      X_PF = frame_F.GetFixedPoseInBodyFrame();
      X_MC = frame_M.GetFixedPoseInBodyFrame();
    } else {
      // By negating the revolute axis, we know that R(a, θ)⁻¹ = R(-a, θ)
      axis_F = -revolute_mobilizer->revolute_axis();
      X_PF = mobilizer->outboard_frame().GetFixedPoseInBodyFrame().inverse();
      X_MC = mobilizer->inboard_frame().GetFixedPoseInBodyFrame().inverse();
    }
    return CalcRevoluteJointChildBodyPoseAsMultilinearPolynomial(
        axis_F, X_PF, X_MC, X_AP, q_star(q_index), cos_delta_(s_angle_index),
        sin_delta_(s_angle_index));
  } else if (dynamic_cast<const internal::PrismaticMobilizer<double>*>(
                 mobilizer) != nullptr) {
    throw std::runtime_error(
        "RationalForwardKinematics: prismatic joint is not supported yet.");
  } else if (dynamic_cast<const internal::WeldMobilizer<double>*>(mobilizer) !=
             nullptr) {
    const internal::WeldMobilizer<double>* weld_mobilizer =
        dynamic_cast<const internal::WeldMobilizer<double>*>(mobilizer);
    math::RigidTransformd X_FM, X_PF, X_MC;
    if (!is_order_reversed) {
      X_FM = weld_mobilizer->get_X_FM();
      X_PF = mobilizer->inboard_frame().GetFixedPoseInBodyFrame();
      X_MC = mobilizer->outboard_frame().GetFixedPoseInBodyFrame();
    } else {
      X_FM = weld_mobilizer->get_X_FM().inverse();
      X_PF = mobilizer->outboard_frame().GetFixedPoseInBodyFrame().inverse();
      X_MC = mobilizer->inboard_frame().GetFixedPoseInBodyFrame().inverse();
    }
    return CalcWeldJointChildBodyPose(X_FM, X_PF, X_MC, X_AP);
  }
  // Successful construction guarantess that all supported mobilizers are
  // handled.
  DRAKE_UNREACHABLE();
}

}  // namespace multibody
}  // namespace drake
