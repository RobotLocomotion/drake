#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/position_kinematics_cache.h"

namespace drake {
namespace multibody {

/// Right now a dummy class to avoid dependency on systems::Context<T>.
///
template <typename T>
class MultibodyTreeContext {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultibodyTreeContext)

  // TODO(amcastro-tri): replace num_bodies by a MultibodyTreeTopology argument
  // once #5583 (introducing topologies) is merged
  MultibodyTreeContext(int num_bodies) {
    // Allocate state data.
    // TODO(amcastro-tri): Allocate this state as needed. Right now this is just
    // a proof of concept assuming all bodies are free. Number of positions
    // and velocities will come from the MultibodyTreeTopology once mobilizers
    // are introduced and the topology is compiled.
    const int num_positions = 6 * num_bodies;
    const int num_velocities = 6 * num_bodies;
    q_.resize(num_positions);
    v_.resize(num_velocities);

    // Allocate cache entries.
    position_kinematics_.Allocate(num_bodies);
  }

  const VectorX<T>& get_positions() const { return q_; }
  const VectorX<T>& get_velocities() const { return v_; }

  VectorX<T>& get_mutable_positions() { return q_; }
  VectorX<T>& get_mutable_velocities() { return v_; }

  const PositionKinematicsCache<T>& get_position_kinematics() const {
    return position_kinematics_;
  }

  PositionKinematicsCache<T>* get_mutable_position_kinematics() const {
    return &position_kinematics_;
  }

  /// Returns the current time in seconds.
  const T& get_time() const { return time_sec_; }

 private:
  T time_sec_;
  VectorX<T> q_, v_;
  // Cached entries.
  mutable PositionKinematicsCache<T> position_kinematics_;
};

}  // namespace multibody
}  // namespace drake
