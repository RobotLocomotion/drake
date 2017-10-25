#pragma once

/// @file This file implements a system which populates optitrack_frame_t
/// messages for publishing over a message passing system. Currently we support
/// publishing over LCM, and may support other messaging protocols in the
/// future.

#include "optitrack/optitrack_frame_t.hpp"

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace sensors {


/// The system has one abstract-valued input port and one abstract-valued
/// output port. The abstract input port is templated on a std::vector of type
/// manipulation::perception::TrackedBody, and the abstract output port value
/// is templated on type `optitrack_frame_t`.
///
/// Note that this system does not actually send this message on an LCM channel.
/// To send the message, the output of this system should be connected to an
/// input port of a systems::Value objecte templated on type
/// `optitrack_frame_t`.
class OptitrackLCMFrameSender : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(OptitrackLCMFrameSender)

  explicit OptitrackLCMFrameSender(int num_rigid_bodies);

  const systems::InputPortDescriptor<double>& get_optitrack_input_port() const {
    return get_input_port(0);
  }

  const systems::OutputPort<double>& get_lcm_output_port() const {
    return get_output_port(0);
  }

 private:
  // This is the method to use for the output port allocator.
  optitrack::optitrack_frame_t CreateNewMessage() const;

  // This is the calculator method for the output port.
  void PopulateMessage(const systems::Context<double>& context,
                       optitrack::optitrack_frame_t* output) const;

  const int num_rigid_bodies_;
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
