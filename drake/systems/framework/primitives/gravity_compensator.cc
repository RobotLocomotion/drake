#include "./gravity_compensator.h"

#include "drake/common/eigen_autodiff_types.h"
#include "drake/systems/plants/KinematicsCache.h"
#include "drake/systems/plants/RigidBodySystem.h"

namespace drake {
namespace systems {

template <typename T>
GravityCompensator<T>::GravityCompensator(
    const RigidBodyTree& world) : mbd_world_(world) {

  int num_DoF  = mbd_world_.number_of_positions();
  this->DeclareInputPort(kVectorValued, 2 * num_DoF, kContinuousSampling);
  this->DeclareOutputPort(kVectorValued, num_DoF, kContinuousSampling);
}

template <typename T>
void GravityCompensator<T>::EvalOutput(const ContextBase<T>& context,
                                        SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));

  Eigen::VectorXd x = System<T>::get_input_vector(context, 0);
  int num_DoF = mbd_world_.number_of_positions();
  KinematicsCache<double> cache =
      mbd_world_.doKinematics(
      x.head(num_DoF), x.tail(num_DoF));
  eigen_aligned_unordered_map<RigidBody const*, drake::TwistVector<double>>
      f_ext;
  f_ext.clear();
  Eigen::VectorXd vd(num_DoF);
  vd.setZero();

  Eigen::VectorXd G = mbd_world_.inverseDynamics(cache, f_ext, vd,
                                                            false);
  System<T>::GetMutableOutputVector(output, 0) = G;
}

template class DRAKESYSTEMFRAMEWORK_EXPORT GravityCompensator<double>;

}  // namespace systems
}  // namespace drake
