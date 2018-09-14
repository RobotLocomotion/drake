#include "drake/manipulation/planner/rigid_body_tree_wrapper.h"

#include <cmath>
#include <limits>
#include <memory>
#include <utility>

#include <fmt/format.h>

#include "drake/multibody/constraint_wrappers.h"
#include "drake/multibody/kinematics_cache_helper.h"
#include "drake/multibody/rigid_body_constraint.h"

using drake::AutoDiffVecXd;
using drake::Vector3;
using drake::Vector4;
using drake::VectorX;
using drake::math::Transform;
using drake::solvers::Constraint;
using drake::systems::plants::KinematicsCacheHelper;
using drake::systems::plants::SingleTimeKinematicConstraintWrapper;

namespace drake {
namespace manipulation {
namespace planner {
namespace {
// Sub-class of SingleTimeKinematicConstraintWrapper that owns the
// SingleTimeKinematicConstraint and KinematicsCacheHelper objects.
class OwningSingleTimeKinematicConstraintWrapper
    : public SingleTimeKinematicConstraintWrapper {
 public:
  OwningSingleTimeKinematicConstraintWrapper(
      std::unique_ptr<SingleTimeKinematicConstraint> rigid_body_constraint,
      std::unique_ptr<KinematicsCacheHelper<double>> cache_helper)
      : SingleTimeKinematicConstraintWrapper(rigid_body_constraint.get(),
                                             cache_helper.get()),
        rigid_body_constraint_(std::move(rigid_body_constraint)),
        cache_helper_(std::move(cache_helper)) {}

 private:
  std::unique_ptr<SingleTimeKinematicConstraint> rigid_body_constraint_;
  std::unique_ptr<KinematicsCacheHelper<double>> cache_helper_;
};

// Sub-class of drake::solvers::Constraint that implements a relative pose
// constraint using RelativeQuatConstraint and RelativePositionConstraint.
class RelativePoseConstraint : public Constraint {
 public:
  // Constructs a RelativePoseConstraint given the body indices
  // (`frame_A_body_index`, `frame_B_body_index`) and frame-to-body transforms
  // (`X_AbA`, `X_BbB`) of frames A and B, the desired pose of frame B relative
  // to frame A (`X_AB_des`), and tolerances on angular and transational error
  // (`orientation_tolerance`, `position_tolerance`), along with the
  // RigidBodyTree to which frames A and B belong..
  RelativePoseConstraint(int frame_A_body_index, const Transform<double>& X_AbA,
                         int frame_B_body_index, const Transform<double>& X_BbB,
                         const drake::math::Transform<double> X_AB_des,
                         double orientation_tolerance,
                         double position_tolerance,
                         const RigidBodyTree<double>* tree)
      : Constraint(4, tree->get_num_positions(),
                   (Vector4<double>() << cos(orientation_tolerance),
                    X_AB_des.translation() -
                        Vector3<double>::Constant(position_tolerance))
                       .finished(),
                   (Vector4<double>() << 1,
                    X_AB_des.translation() +
                        Vector3<double>::Constant(position_tolerance))
                       .finished()) {
    const Eigen::Matrix3Xd p_BbB = X_BbB.translation();
    Transform<double> X_AbBb_des = X_AbA * X_AB_des * X_BbB.inverse();
    // NOTE 1: The const_casts below are required by the APIs for
    // RelativeQuatConstraint and RelativePositionConstraint, which take a
    // non-const pointer despite the fact that they do not modify the
    // RigidBodyTree.
    // NOTE 2: The bounds for the RelativeQuatConstraint and
    // RelativePositionConstraint contained in this class are unused. The bounds
    // for this class are set in the call to the base class constructor above.
    orientation_constraint_ =
        std::make_unique<OwningSingleTimeKinematicConstraintWrapper>(
            std::make_unique<RelativeQuatConstraint>(
                const_cast<RigidBodyTree<double>*>(tree), frame_B_body_index,
                frame_A_body_index,
                X_AbBb_des.rotation().ToQuaternionAsVector4(),
                0 /* orientation tolerance (unused) */),
            std::unique_ptr<KinematicsCacheHelper<double>>(
                new KinematicsCacheHelper<double>(*tree)));
    position_constraint_ =
        std::make_unique<OwningSingleTimeKinematicConstraintWrapper>(
            std::unique_ptr<RelativePositionConstraint>(
                new RelativePositionConstraint(
                    const_cast<RigidBodyTree<double>*>(tree),
                    X_BbB.translation() /* pts */,
                    Vector3<double>::Zero() /* lower bound (unused) */,
                    Vector3<double>::Zero() /* upper bound (unused) */,
                    frame_B_body_index, frame_A_body_index,
                    (VectorX<double>(7) << X_AbA.translation(),
                     X_AbA.rotation().ToQuaternionAsVector4())
                        .finished(),
                    DrakeRigidBodyConstraint::default_tspan)),
            std::unique_ptr<KinematicsCacheHelper<double>>(
                new KinematicsCacheHelper<double>(*tree)));
  }

 private:
  template <typename T>
  void DoEvalGeneric(const Eigen::Ref<const VectorX<T>>& x,
                     VectorX<T>* y) const {
    DRAKE_ASSERT(y != nullptr);
    VectorX<T> y_orientation(num_orientation_constraints_);
    orientation_constraint_->Eval(x, &y_orientation);
    VectorX<T> y_position(num_position_constraints_);
    position_constraint_->Eval(x, &y_position);

    y->resize(num_constraints());
    *y << y_orientation, y_position;
  }

  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override {
    DoEvalGeneric(x, y);
  }

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override {
    DoEvalGeneric(x, y);
  }

  void DoEval(const Eigen::Ref<const VectorX<drake::symbolic::Variable>>& x,
              VectorX<drake::symbolic::Expression>* y) const override {
    throw std::runtime_error(
        "RelativePoseConstraint on Expression not implemented");
  }

  const int num_orientation_constraints_{1};
  const int num_position_constraints_{3};
  std::unique_ptr<OwningSingleTimeKinematicConstraintWrapper>
      orientation_constraint_{};
  std::unique_ptr<OwningSingleTimeKinematicConstraintWrapper>
      position_constraint_{};
};
}  // namespace

RigidBodyTreeWrapper::RigidBodyTreeWrapper(const RigidBodyTree<double>* tree)
    : tree_(tree) {
  if (!tree_->initialized()) {
    throw std::logic_error(
        "`The RigidBodyTree is not initialized. Call compile() on the "
        "RigidBodyTree before passing it to the RigidBodyTreeWrapper "
        "constructor.");
  }
  joint_velocity_lower_limit_ = VectorX<double>::Zero(num_velocities());
  joint_velocity_upper_limit_ = VectorX<double>::Zero(num_velocities());
  joint_position_lower_limit_ = tree_->joint_limit_min;
  joint_position_upper_limit_ = tree_->joint_limit_max;
  for (int i = 0; i < num_positions(); ++i) {
    drake::log()->debug("joint_position_lower_limit_({}) = {}", i,
                        joint_position_lower_limit_(i));
    drake::log()->debug("joint_position_upper_limit_({}) = {}", i,
                        joint_position_upper_limit_(i));
    if (!std::isfinite(joint_position_lower_limit_(i)) ||
        !std::isfinite(joint_position_upper_limit_(i))) {
      throw std::logic_error(fmt::format(
          "The joint limits for position {} (and possibly others) are "
          "non-finite. Set all of the RigidBodyTree's joint limits to finite "
          "values before passing it to the RigidBodyTreeWrapper constructor.",
          i));
    }
  }
}

RigidBodyTreeWrapper::RigidBodyTreeWrapper(
    std::unique_ptr<RigidBodyTree<double>> tree)
    : RigidBodyTreeWrapper(tree.get()) {
  owned_tree_ = std::move(tree);
}

RigidBodyTreeWrapper::~RigidBodyTreeWrapper() = default;

int RigidBodyTreeWrapper::num_positions() const {
  return tree_->get_num_positions();
}

int RigidBodyTreeWrapper::num_velocities() const {
  return tree_->get_num_velocities();
}

int RigidBodyTreeWrapper::PositionStartIndexForBody(
    const std::string& body_name) const {
  return tree_->FindBody(body_name)->get_position_start_index();
}

const VectorX<double>& RigidBodyTreeWrapper::joint_position_lower_limit()
    const {
  return joint_position_lower_limit_;
}

const VectorX<double>& RigidBodyTreeWrapper::joint_position_upper_limit()
    const {
  return joint_position_upper_limit_;
}

const VectorX<double>& RigidBodyTreeWrapper::joint_velocity_lower_limit()
    const {
  return joint_velocity_lower_limit_;
}

const VectorX<double>& RigidBodyTreeWrapper::joint_velocity_upper_limit()
    const {
  return joint_velocity_upper_limit_;
}

VectorX<double> RigidBodyTreeWrapper::GetZeroConfiguration() const {
  return tree_->getZeroConfiguration();
}

VectorX<double> RigidBodyTreeWrapper::GetRandomConfiguration(
    std::default_random_engine* generator) const {
  DRAKE_ASSERT(generator != nullptr);
  VectorX<double> random_configuration(num_positions());
  for (int j = 0; j < num_positions(); ++j) {
    random_configuration(j) = std::uniform_real_distribution<double>(
        joint_position_lower_limit_(j),
        joint_position_upper_limit_(j))(*generator);
  }
  return random_configuration;
}

Transform<double> RigidBodyTreeWrapper::CalcRelativeTransform(
    const VectorX<double>& q, const std::string& frame_A_name,
    const std::string& frame_B_name) const {
  return Transform<double>(tree_->relativeTransform(
      tree_->doKinematics(q), tree_->findFrame(frame_A_name)->get_frame_index(),
      tree_->findFrame(frame_B_name)->get_frame_index()));
}

void RigidBodyTreeWrapper::DoSetJointPositionLimits(int position_index,
                                                    double lower_limit,
                                                    double upper_limit) {
  joint_position_lower_limit_(position_index) = lower_limit;
  joint_position_upper_limit_(position_index) = upper_limit;
}

void RigidBodyTreeWrapper::DoSetJointVelocityLimits(int velocity_index,
                                                    double lower_limit,
                                                    double upper_limit) {
  joint_velocity_lower_limit_(velocity_index) = lower_limit;
  joint_velocity_upper_limit_(velocity_index) = upper_limit;
}

std::shared_ptr<Constraint>
RigidBodyTreeWrapper::MakeCollisionAvoidanceConstraint(
    double collision_avoidance_threshold) const {
  // The const_cast below is required because RigidBodyTree contains the
  // collision world as hidden state that is modified by collision queries.
  return std::make_shared<OwningSingleTimeKinematicConstraintWrapper>(
      std::unique_ptr<MinDistanceConstraint>(
          new MinDistanceConstraint(const_cast<RigidBodyTree<double>*>(tree_),
                                    collision_avoidance_threshold, {}, {})),
      std::unique_ptr<KinematicsCacheHelper<double>>(
          new KinematicsCacheHelper<double>(*tree_)));
}

std::shared_ptr<drake::solvers::Constraint>
RigidBodyTreeWrapper::MakeRelativePoseConstraint(
    const std::string& frame_A_name, const std::string& frame_B_name,
    const drake::math::Transform<double>& X_AB_des,
    double orientation_tolerance, double position_tolerance) const {
  std::shared_ptr<RigidBodyFrame<double>> frame_A =
      tree_->findFrame(frame_A_name);
  std::shared_ptr<RigidBodyFrame<double>> frame_B =
      tree_->findFrame(frame_B_name);
  const int frame_A_body_index = frame_A->get_rigid_body().get_body_index();
  const int frame_B_body_index = frame_B->get_rigid_body().get_body_index();
  Transform<double> X_AbA{frame_A->get_transform_to_body()};
  Transform<double> X_BbB{frame_B->get_transform_to_body()};
  return std::make_shared<RelativePoseConstraint>(
      frame_A_body_index, X_AbA, frame_B_body_index, X_BbB, X_AB_des,
      orientation_tolerance, position_tolerance, tree_);
}

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
