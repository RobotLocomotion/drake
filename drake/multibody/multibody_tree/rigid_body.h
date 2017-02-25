#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/body.h"

namespace drake {
namespace multibody {

// Forward declarations.
template<typename T> class MultibodyTree;

template <typename T>
class RigidBody : public Body<T> {
 public:
  /// Creates a new %RigidBody and adds it to the %MultibodyTree world.
  /// The MultibodyTree @p tree takes ownership of the newly created body.
  /// @param[in] tree The parent MultibodyTree to which this body will be added.
  /// @param[in] mass_properties Default mass properties for this rigid body.
  /// @returns A reference to the newly created rigid body.
  // TODO(amcastro-tri): In a future PR this factory will take a MassProperties
  // object to:
  //   1. Force users to provide all the necessary information at creation.
  //   2. Perform all the necessary checks to ensure the supplied mass
  //      properties are physically valid.
  static RigidBody<T>& Create(MultibodyTree<T>* tree);

  int get_num_positions() const final { return 0; }

  int get_num_velocities() const final { return 0; }

 private:
  // Do not allow users to create a rigid body using its public constructors
  // but force them to use the factory method Create().
  // TODO(amcastro-tri): In a future PR this factory will take a MassProperties
  // object to:
  //   1. Force users to provide all the necessary information at creation.
  //   2. Perform all the necessary checks to ensure the supplied mass
  //      properties are physically valid.
  RigidBody();
};

}  // namespace multibody
}  // namespace drake
