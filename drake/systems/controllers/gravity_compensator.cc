#include "drake/systems/controllers/gravity_compensator.h"

#include "drake/multibody/kinematics_cache.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace systems {

template <typename T>
GravityCompensator<T>::GravityCompensator(const RigidBodyTree<T>& tree)
    : tree_(tree) {
  this->DeclareInputPort(kVectorValued, tree.get_num_positions());
  this->DeclareOutputPort(kVectorValued, tree_.get_num_actuators());
  if (tree.get_num_positions() != tree_.get_num_actuators()) {
    std::stringstream msg;
    msg << "The model is under-actuated!\n"
        << "  - size of gravity vector: " << tree.get_num_positions() << "\n"
        << "  - number of actuators: " << tree.get_num_actuators();
    DRAKE_ABORT_MSG(msg.str().c_str());
  }
}

template <typename T>
void GravityCompensator<T>::DoCalcOutput(const Context<T>& context,
                                         SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));

  Eigen::VectorXd q = this->EvalEigenVectorInput(context, 0);

  KinematicsCache<T> cache = tree_.doKinematics(q);
  eigen_aligned_std_unordered_map<RigidBody<T> const*, drake::TwistVector<T>>
      f_ext;
  f_ext.clear();

  Eigen::VectorXd g = tree_.dynamicsBiasTerm(
      cache, f_ext, false /* include velocity terms */);

  DRAKE_ASSERT(g.size() == System<T>::get_output_port(0).get_size());
  System<T>::GetMutableOutputVector(output, 0) = g;
}

template class GravityCompensator<double>;
// TODO(naveenoid): Get the AutoDiff working as in the line below.
// template class GravityCompensator<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
