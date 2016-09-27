#include "./gravity_compensator.h"

#include "drake/systems/plants/KinematicsCache.h"
#include "drake/systems/plants/RigidBodyTree.h"

namespace drake {
namespace systems {

template <typename T>
GravityCompensator<T>::GravityCompensator(const RigidBodyTree& rigid_body_tree)
    : rigid_body_tree_(rigid_body_tree) {

  this->DeclareInputPort(kVectorValued, rigid_body_tree_.number_of_positions(),
                         kContinuousSampling);
  this->DeclareOutputPort(kVectorValued, rigid_body_tree_.actuators.size(),
                          kContinuousSampling);
}

template <typename T>
void GravityCompensator<T>::EvalOutput(const Context<T>& context,
                                       SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));

  Eigen::VectorXd x = System<T>::CopyContinuousStateVector(context);

  int number_of_velocities = rigid_body_tree_.number_of_velocities();

  Eigen::VectorXd vd = Eigen::VectorXd::Zero(number_of_velocities);

  KinematicsCache<double> cache = rigid_body_tree_.doKinematics(x);
  eigen_aligned_std_unordered_map<RigidBody const*, drake::TwistVector<double>>
      f_ext;
  f_ext.clear();

  Eigen::VectorXd G = rigid_body_tree_.inverseDynamics(cache, f_ext, vd, false);
  System<T>::GetMutableOutputVector(output, 0) = G;
}

template class DRAKESYSTEMCONTROLLERS_EXPORT GravityCompensator<double>;
// TODO(naveenoid) : get the AutoDiff working as in the line below.
// template class DRAKESYSTEMCONTROLLERS_EXPORT GravityCompensator<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
