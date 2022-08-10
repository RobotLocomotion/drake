#pragma once

#include <string>

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"
#include "drake/common/name_value.h"

namespace anzu {
namespace sim {

/** This config struct specifies how to wire up Drake systems between an LCM
interface and the actuation input ports of a MultibodyPlant. This simulates the
role that driver software and control cabinets would take in real life.

It creates an LCM publisher on the `SCHUNK_WSG_STATUS` channel and an LCM
subscriber on the `SCHUNK_WSG_COMMAND` channel. */
struct WsgDriver {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(WsgDriver)
  WsgDriver() = default;

  /// Gains to apply to the the WSG fingers.  The p term corresponds
  /// approximately to the elastic modulus of the belt, the d term to the
  /// viscous friction of the geartrain.  The i term is nonphysical.
  Eigen::Vector3d pid_gains{7200., 0., 5.};

  std::string lcm_bus{"default"};

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(pid_gains));
    a->Visit(DRAKE_NVP(lcm_bus));
  }
};

}  // namespace sim
}  // namespace anzu
