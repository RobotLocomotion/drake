#pragma once

/// @file This file contains classes dealing with sending/receiving
/// LCM messages related to the iiwa arm.

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "robotlocomotion/plan_status_t.hpp"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace pr2 {

class PlanStatusSender : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PlanStatusSender)

  explicit PlanStatusSender();

 private:
  // This is the method to use for the output port allocator.
  robotlocomotion::plan_status_t MakeOutputStatus() const;

  // This is the calculator method for the output port.
  void OutputStatus(const systems::Context<double>& context,
                    robotlocomotion::plan_status_t* output) const;

};

}  // namespace pr2
}  // namespace examples
}  // namespace drake
