#include "./gravity_compensator.h"

#include "drake/systems/plants/KinematicsCache.h"
#include "drake/systems/plants/RigidBodyTree.h"

#include "drake/systems/plants/KinematicsCache.h"
#include "drake/systems/plants/RigidBodySystem.h"

namespace drake {
namespace systems {

template <typename T>
GravityCompensator<T>::GravityCompensator(const RigidBodyTree& rigid_body_tree)
    : mdb_world_(rigid_body_tree) {
  int num_of_inputs = mdb_world_.number_of_positions();

  this->DeclareInputPort(kVectorValued, num_of_inputs, kContinuousSampling);
  this->DeclareOutputPort(kVectorValued, num_of_inputs, kContinuousSampling);
}

template <typename T>
void GravityCompensator<T>::EvalOutput(const Context<T>& context,
                                       SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));

  Eigen::VectorXd x = System<T>::CopyContinuousStateVector(context);

  int number_of_velocities = mdb_world_.number_of_velocities();

  Eigen::VectorXd vd = Eigen::VectorXd::Zero(number_of_velocities);

  KinematicsCache<double> cache = mdb_world_.doKinematics(x);
  eigen_aligned_std_unordered_map<RigidBody const*, drake::TwistVector<double>>
      f_ext;
  f_ext.clear();

  Eigen::VectorXd G = mdb_world_.inverseDynamics(cache, f_ext, vd, false);
  System<T>::GetMutableOutputVector(output, 0) = G;
}

template class DRAKESYSTEMCONTROLLERS_EXPORT GravityCompensator<double>;
// TODO(naveenoid) : get the AutoDiff working as in the line below.
// template class DRAKESYSTEMCONTROLLERS_EXPORT GravityCompensator<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
