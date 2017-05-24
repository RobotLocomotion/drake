#pragma once

#include <memory>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/body_node.h"
#include "drake/multibody/multibody_tree/mobilizer.h"
//#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;
#define PRINT_VARn(x) std::cout <<  #x ":\n" << x << std::endl;

namespace drake {
namespace multibody {

template <typename T, int  num_positions, int num_velocities>
class BodyNodeImpl : public BodyNode<T> {
 public:
  // static constexpr int i = 42; discouraged.
  // See answer in: http://stackoverflow.com/questions/37259807/static-constexpr-int-vs-old-fashioned-enum-when-and-why
  enum : int {nq = num_positions, nv = num_velocities};

  //using BodyNode<T>::get_X_PF;
  //using BodyNode<T>::get_X_WP;
  //using BodyNode<T>::get_X_MB;
  //using BodyNode<T>::get_X_FM;

  BodyNodeImpl(const Body<T>* body, const Mobilizer<T>* mobilizer) :
      BodyNode<T>(body, mobilizer) {}

#if 0
  /// Helper methods access the context.
  const Vector<T, nv>& get_mobilizer_velocities(
      const MultibodyTreeContext<T>& context) const
  {
    //return *reinterpret_cast<const Vector<T, nv>*>(
    //    context.get_velocities().data() + this->get_rigid_velocities_start());
    return get_mobilizer_velocities_from_pool(context.get_velocities());
  }

  /// Helper methods to access node quantities as expressions of fixed size.
  const Vector<T, nv>& get_mobilizer_velocities_from_pool(
      const VectorX<T>& vdot_pool) const {
    return *reinterpret_cast<const Vector<T, nv>*>(
        &(vdot_pool[this->get_rigid_velocities_start()]));
  };

  /// Helper Methods to access the position kinematics cache.


  // Helper methods to extract entries from the velocity kinematics cache.

  /// @returns a mutable reference to the vector of time derivatives of the
  /// mobilizer generalized positions `qm` corresponding to this node's
  /// mobilizer.
  Vector<T, nq>& get_mutable_qmdot(VelocityKinematicsCache<T>* vc) const {
    DRAKE_ASSERT(this->get_num_rigid_positions() == nq);
    return *reinterpret_cast<Vector<T, nq>*>(
        vc->get_mutable_qdot_pool().data() + this->get_rigid_positions_start());
  }

  const Vector<T, nv>& get_qmdot(const VelocityKinematicsCache<T>& vc) const
  {
    DRAKE_ASSERT(this->get_num_rigid_velocities() == nv);
    return *reinterpret_cast<const Vector<T, nv>*>(
        vc.get_qdot_pool().data() + this->get_rigid_positions_start());
  }
#endif

 protected:
  using BodyNode<T>::body_;
  using BodyNode<T>::mobilizer_;
};

}  // namespace multibody
}  // namespace drake
