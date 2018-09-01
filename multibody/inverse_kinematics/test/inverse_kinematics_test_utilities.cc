#include "drake/multibody/inverse_kinematics/test/inverse_kinematics_test_utilities.h"

#include "drake/multibody/joints/quaternion_ball_joint.h"

namespace drake {
namespace multibody {
namespace test {
template <typename T>
std::unique_ptr<MultibodyTree<T>> ConstructTwoFreeBodies() {
  auto model = std::make_unique<MultibodyTree<T>>();

  const double mass{1};
  const Eigen::Vector3d p_AoAcm_A(0, 0, 0);
  const RotationalInertia<double> I_AAcm_A{0.001, 0.001, 0.001};
  const SpatialInertia<double> M_AAo_A = SpatialInertia<double>::MakeFromCentralInertia(mass, p_AoAcm_A, I_AAcm_A);

  const RigidBody<T>& link1 = model->AddRigidBody("body1", M_AAo_A);
  const RigidBody<T>& link2 = model->AddRigidBody("body2", M_AAo_A);

  model->template AddJoint<QuaternionBallJoint>("joint1", model->world_body(), Eigen::Isometry3d::Identity(), link1, Eigen::Isometry3d::Identity());
  return model;
}

template std::unique_ptr<MultibodyTree<double>> ConstructTwoFreeBodies<double>();
template std::unique_ptr<MultibodyTree<AutoDiffXd>> ConstructTwoFreeBodies<AutoDiffXd>();
}  // namespace test
}  // namespace multibody
}  // namespace drake
