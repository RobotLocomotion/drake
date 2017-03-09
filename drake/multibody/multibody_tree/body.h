#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"
#include "drake/multibody/multibody_tree/multibody_tree_element.h"

namespace drake {
namespace multibody {

// Forward declarations.
template<typename T> class MultibodyTree;

/// Bodies are a bound, interconnected aggregate of matter that moves together
/// through space by translation and rotation and may or may not undergo
/// deformations, [Mitiguy 2016].
/// This class provides the general abstraction of a body with an API that
/// makes no assumption about whether a body is rigid or deformable and neither
/// does it make any assumptions about on the underlying physical model or
/// approximation.
/// As an element or component of a MultibodyTree, a body is a
/// MultibodyTreeElement, and therefore it has a unique identifier within the
/// multibody tree it belongs to.
///
/// -[Mitiguy 2016] P Mitiguy. Advanced Dynamics and Motion Simulation, 2016.
///                 Stanford University. www.MotionGenesis.com.
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
template <typename T>
class Body : public MultibodyTreeElement<Body<T>, BodyIndex> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Body)

  /// Returns the number of generalized positions associated with this body.
  virtual int get_num_positions() const = 0;

  /// Returns the number of generalized velocities associatted with this body.
  virtual int get_num_velocities() const = 0;

  /// At MultibodyTree::Compile() time, each body will retrieve its topology
  /// from the parent MultibodyTree.
  virtual void Compile() {}
 protected:
  // Default constructor. Only sub-classes can use it.
  Body() = default;
};

}  // namespace multibody
}  // namespace drake
