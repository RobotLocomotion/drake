#include "drake/systems/controllers/gravity_compensator.h"

#include "drake/systems/plants/KinematicsCache.h"
#include "drake/systems/plants/RigidBodyTree.h"

namespace drake {
namespace systems {

template <typename T>
GravityCompensator<T>::GravityCompensator(const RigidBodyTree& rigid_body_tree)
    : rigid_body_tree_(rigid_body_tree) {
  this->DeclareInputPort(kVectorValued, rigid_body_tree.get_num_positions(),
                         kContinuousSampling);
  this->DeclareOutputPort(kVectorValued, rigid_body_tree_.actuators.size(),
                          kContinuousSampling);
}

template <typename T>
void GravityCompensator<T>::EvalOutput(const Context<T>& context,
                                       SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));

  Eigen::VectorXd q = this->EvalEigenVectorInput(context, 0);

  KinematicsCache<T> cache = rigid_body_tree_.doKinematics(q);
  eigen_aligned_std_unordered_map<RigidBody const*, drake::TwistVector<T>>
      f_ext;
  f_ext.clear();

  Eigen::VectorXd g = rigid_body_tree_.dynamicsBiasTerm(cache, f_ext,
  false /* include velocity terms */);
  System<T>::GetMutableOutputVector(output, 0) = g;
}

template class DRAKE_EXPORT GravityCompensator<double>;
// TODO(naveenoid): Get the AutoDiff working as in the line below.
// template class DRAKE_EXPORT GravityCompensator<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
