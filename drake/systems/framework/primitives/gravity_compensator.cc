#include "./gravity_compensator.h"

#include "drake/common/eigen_autodiff_types.h"
#include "drake/systems/plants/KinematicsCache.h"
#include "drake/systems/plants/RigidBodySystem.h"

namespace drake {
namespace systems {

template class DRAKESYSTEMFRAMEWORK_EXPORT GravityCompensator<double>;
template class DRAKESYSTEMFRAMEWORK_EXPORT GravityCompensator<AutoDiffXd>;


template <typename T>
GravityCompensator<T>::GravityCompensator(
    const std::shared_ptr<RigidBodyTree> rigid_body_tree_ptr) :
    rigid_body_tree_ptr_(rigid_body_tree_ptr) {

  int num_DoF  = rigid_body_tree_ptr_->number_of_positions();
  this->DeclareInputPort(kVectorValued, 2 * num_DoF, kContinuousSampling);
  this->DeclareOutputPort(kVectorValued, num_DoF, kContinuousSampling);
}

template <typename T>
void GravityCompensator<T>::EvalOutput(const ContextBase<T>& context,
                                        SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));

  Eigen::VectorXd x = System<T>::get_input_vector(context, 0);
  int num_DoF = rigid_body_tree_ptr_->number_of_positions();
  KinematicsCache<double> cache =
      rigid_body_tree_ptr_->doKinematics(
      x.head(num_DoF), x.tail(num_DoF));
  eigen_aligned_unordered_map<RigidBody const*, drake::TwistVector<double>>
      f_ext;
  f_ext.clear();
  Eigen::VectorXd vd(num_DoF);
  vd.setZero();

  Eigen::VectorXd G = rigid_body_tree_ptr_->inverseDynamics(cache, f_ext, vd,
                                                            false);
  System<T>::GetMutableOutputVector(output, 0) = G;

}

}  // namespace systems
}  // namespace drake
