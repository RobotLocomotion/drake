#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "drake/common/drake_copyable.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/lcmt_drake_signal.hpp"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/continuous_state.h"

namespace drake {
namespace systems {

/// This is a child class of RigidBodyPlant that publishes `xdot`, time
/// derivative of RBPlant's state vector `x`, on an LCM channel encoded as an
/// `lcmt_drake_signal` message.
///
/// @tparam T The scalar type. Currently, the only supported type is `double`.
/// @ingroup rigid_body_systems
template <typename T>
class RigidBodyPlantThatPublishesXdot : public RigidBodyPlant<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RigidBodyPlantThatPublishesXdot)

  /// Instantiates a %RigidBodyPlantThatPublishesXdot.
  ///
  /// @param[in] tree The RigidBodyTree. This defines the multi-body dynamics of
  /// the world. This parameter must not be `nullptr`.
  ///
  /// @param[in] channel The name of the channel on which to publish `xdot`.
  ///
  /// @param[in] lcm  A non-null pointer to the LCM subsystem to publish on.
  /// The pointer must remain valid for the lifetime of this object.
  explicit RigidBodyPlantThatPublishesXdot(
      std::unique_ptr<const RigidBodyTree<T>> tree, const std::string& channel,
      drake::lcm::DrakeLcmInterface* lcm);

  ~RigidBodyPlantThatPublishesXdot() override;

 private:
  // Publishes `xdot`, the derivative of `x`, which is this system's generalized
  // state vector. This vector contains the derivatives of the RigidBodyTree's
  // joint's positions and velocities. Thus, the units are velocities and
  // accelerations.
  void DoPublish(const Context<T>& context) const override;

  // A const pointer to an LCM subsystem. Note that while the pointer is const,
  // the LCM subsystem is not const.
  ::drake::lcm::DrakeLcmInterface* const lcm_{};

  // The LCM message upon which to publish `xdot`.
  const std::string channel_;

  // A variable and LCM message for holding the `xdot`. These class member
  // variables exist to avoid repeated memory reallocation and deallocation.
  std::unique_ptr<ContinuousState<double>> derivatives_;
  mutable lcmt_drake_signal message_;
};

}  // namespace systems
}  // namespace drake
