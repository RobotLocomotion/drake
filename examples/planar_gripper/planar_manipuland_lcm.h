#pragma once

/// @file
/// This file contains classes dealing with sending/receiving
/// LCM messages related to the planar manipuland.

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/lcmt_planar_manipuland_status.hpp"
#include "drake/systems/framework/event_status.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace planar_gripper {
/// Handles lcmt_planar_manipuland_status messages from a LcmSubscriberSystem.
///
/// This system has one abstract valued input port which expects a
/// Value object templated on type `lcmt_planar_manipuland_status`.
///
/// This system has one vector valued output port which reports
/// measured pose (y, z, theta) and velocity (ydot, zdot, thetadot) of the
/// manipuland.
///
/// All ports will continue to output their initial state (typically
/// zero) until a message is received.

// This should be the update frequency of the mocap system.
constexpr double kPlanarManipulandStatusPeriod = 0.010;

class PlanarManipulandStatusDecoder : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PlanarManipulandStatusDecoder)

  PlanarManipulandStatusDecoder();

  ~PlanarManipulandStatusDecoder() {}

 private:
  void OutputStatus(const systems::Context<double>& context,
                    systems::BasicVector<double>* output) const;

  /// Event handler of the periodic discrete state update.
  systems::EventStatus UpdateDiscreteState(
      const systems::Context<double>& context,
      systems::DiscreteValues<double>* discrete_state) const;
};

/**
 * Creates and outputs lcmt_planar_manipuland_status messages.
 *
 * This system has one vector-valued input port containing the current pose
 * (y, z, theta) and velocity (ẏ, ż, thetadot) of the manipuland, in the order
 * (y, z, theta, ydot, zdot, thetadot).
 *
 * This system has one abstract valued output port that contains a Value object
 * templated on type `lcmt_planar_manipuland_status`. Note that this system
 * does NOT actually send this message on an LCM channel. To send the message,
 * the output of this system should be connected to an input port of a
 * systems::lcm::LcmPublisherSystem that accepts a Value object templated on
 * type `lcmt_planar_manipuland_status`.
 */
class PlanarManipulandStatusEncoder : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PlanarManipulandStatusEncoder)

  PlanarManipulandStatusEncoder();

 private:
  // This is the calculator method for the output port.
  void OutputStatus(const systems::Context<double>& context,
                    lcmt_planar_manipuland_status* output) const;
};

}  // namespace planar_gripper
}  // namespace examples
}  // namespace drake
