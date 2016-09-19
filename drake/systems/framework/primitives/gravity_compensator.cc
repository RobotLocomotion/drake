#include "./gravity_compensator.h"

#include "drake/common/eigen_autodiff_types.h"
#include "drake/systems/plants/KinematicsCache.h"
#include "drake/systems/plants/RigidBodySystem.h"

namespace drake {
namespace systems {

template <typename T>
GravityCompensator<T>::GravityCompensator(
    const RigidBodyTree& rigid_body_tree) :
    mdb_world_(rigid_body_tree) {

  int num_of_states  = mdb_world_.number_of_positions() +
      mdb_world_.number_of_velocities();
  this->DeclareInputPort(kVectorValued, num_of_states, kContinuousSampling);
  this->DeclareOutputPort(kVectorValued, num_of_states, kContinuousSampling);
}

template <typename T>
void GravityCompensator<T>::EvalOutput(const ContextBase<T>& context,
                                        SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));

  Eigen::VectorXd x = System<T>::get_input_vector(context, 0);
  int number_of_positions = mdb_world_.number_of_positions();
  int number_of_velocities = mdb_world_.number_of_velocities();
  KinematicsCache<double> cache =
      mdb_world_.doKinematics(x.head(number_of_positions),
                                    x.tail(number_of_velocities));
  eigen_aligned_unordered_map<RigidBody const*, drake::TwistVector<double>>
      f_ext;
  f_ext.clear();
  Eigen::VectorXd vd(number_of_velocities);
  vd.setZero();

  Eigen::VectorXd G = mdb_world_.inverseDynamics(cache, f_ext, vd,
                                                            false);
  System<T>::GetMutableOutputVector(output, 0) = G;

}


template class DRAKESYSTEMFRAMEWORK_EXPORT GravityCompensator<double>;
// TODO(naveenoid) : get the AutoDiff working as in the line below.
//template class DRAKESYSTEMFRAMEWORK_EXPORT GravityCompensator<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
