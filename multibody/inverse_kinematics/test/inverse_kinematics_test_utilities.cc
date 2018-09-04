#include "drake/multibody/inverse_kinematics/test/inverse_kinematics_test_utilities.h"

namespace drake {
namespace multibody {
namespace {
template <typename T>
std::unique_ptr<T> ConstructTwoFreeBodiesHelper() {
  auto model = std::make_unique<T>();

  const double mass{1};
  const Eigen::Vector3d p_AoAcm_A(0, 0, 0);
  const RotationalInertia<double> I_AAcm_A{0.001, 0.001, 0.001};
  const SpatialInertia<double> M_AAo_A =
      SpatialInertia<double>::MakeFromCentralInertia(mass, p_AoAcm_A, I_AAcm_A);

  model->AddRigidBody("body1", M_AAo_A);
  model->AddRigidBody("body2", M_AAo_A);

  model->Finalize();

  return model;
}
}  // namespace
template <typename T>
std::unique_ptr<MultibodyTree<T>> ConstructTwoFreeBodies() {
  return ConstructTwoFreeBodiesHelper<MultibodyTree<T>>();
}

template <typename T>
std::unique_ptr<multibody_plant::MultibodyPlant<T>>
ConstructTwoFreeBodiesPlant() {
  return ConstructTwoFreeBodiesHelper<multibody_plant::MultibodyPlant<T>>();
}

Eigen::Vector4d QuaternionToVector4(const Eigen::Quaterniond& q) {
  return Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
}

template std::unique_ptr<MultibodyTree<double>>
ConstructTwoFreeBodies<double>();
template std::unique_ptr<MultibodyTree<AutoDiffXd>>
ConstructTwoFreeBodies<AutoDiffXd>();
template std::unique_ptr<multibody_plant::MultibodyPlant<double>>
ConstructTwoFreeBodiesPlant<double>();
template std::unique_ptr<multibody_plant::MultibodyPlant<AutoDiffXd>>
ConstructTwoFreeBodiesPlant<AutoDiffXd>();
}  // namespace multibody
}  // namespace drake
