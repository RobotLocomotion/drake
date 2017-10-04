#pragma once

#include <memory>

#include <Eigen/Geometry>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace automotive {

/// This class is a first-pass model of a traffic light. It outputs
/// a vector containing its position, a radius, and whether it is open or
/// closed.
///
/// It is time-triggered, and signals "green" for half of its period and "red"
/// for the other half.
///
/// At this time, the only supported value of T is double.
// TODO(nikos-tri): Support AutoDiffXd and Symbolic.

template <typename T>
class TrafficLight : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TrafficLight)

  /// Positions and radius are in meters, period is in seconds.
  TrafficLight(T x_position, T y_position, T radius, T period);

  /// No input port, because this system is time-triggered.
  const systems::OutputPort<T>& output() const;

 private:
  void DoCalcOutput(const systems::Context<T>& context,
                    systems::BasicVector<T>* output) const;

  // Indices for input / output ports.
  const int traffic_input_index_{};
  const int output_index_{};
  int parameters_index_{};

  const VectorX<T> ReadParameters(const systems::Context<T>& context) const;
  void WriteOutput(const VectorX<T> value,
                   systems::BasicVector<T>* output) const;
};

}  // namespace automotive
}  // namespace drake
