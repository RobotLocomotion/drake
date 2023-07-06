#pragma once

#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/contact_solvers/contact_configuration.h"
#include "drake/multibody/contact_solvers/matrix_block.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {
namespace internal {

// Struct to store kinematics information for each contact pair. For each
// contact pair this struct stores signed distance, Jacobian w.r.t. velocities
// for each participating tree and rotation matrix between world frame and
// contact frame.
template <typename T>
struct ContactPairKinematics {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ContactPairKinematics);

  // Struct to store the block contribution from a given tree to the contact
  // Jacobian for a contact pair.
  struct JacobianTreeBlock {
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(JacobianTreeBlock);

    JacobianTreeBlock(TreeIndex tree_in,
                      contact_solvers::internal::MatrixBlock<T> J_in)
        : tree(tree_in), J(std::move(J_in)) {}

    // Index of the tree for this block.
    TreeIndex tree;

    // J.cols() must equal the number of generalized velocities for
    // the corresponding tree.
    contact_solvers::internal::MatrixBlock<T> J;
  };

  ContactPairKinematics(
      std::vector<JacobianTreeBlock> jacobian_in,
      contact_solvers::internal::ContactConfiguration<T> configuration_in)
      : jacobian(std::move(jacobian_in)),
        configuration(std::move(configuration_in)) {}

  // TODO(amcastro-tri): consider using absl::InlinedVector since here we know
  // this has a size of at most 2.
  // Jacobian for a discrete contact pair stored as individual blocks for each
  // of the trees participating in the contact. Only one or two trees can
  // participate in a given contact.
  std::vector<JacobianTreeBlock> jacobian;

  // Contact configuration specifying objects in contact, contact point
  // position, depth and contact frame.
  contact_solvers::internal::ContactConfiguration<T> configuration;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    struct ::drake::multibody::internal::ContactPairKinematics)
