#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/multibody_indexes.h"
#include "drake/multibody/multibody_tree/multibody_tree_element.h"

namespace drake {
namespace multibody {

// Forward declarations.
template<typename T> class MultibodyTree;

template <typename T>
class Body : public MultibodyTreeElement<Body<T>, BodyIndex> {
 public:
  /// Returns the number of generalized positions associated with this body.
  virtual int get_num_positions() const = 0;

  /// Returns the number of generalized velocities associatted with this body.
  virtual int get_num_velocities() const = 0;

  /// At MultibodyTree::Compile() time each body will retrieve its topology from
  /// the parent MultibodyTree.
  virtual void Compile() {}
};

}  // namespace multibody
}  // namespace drake
