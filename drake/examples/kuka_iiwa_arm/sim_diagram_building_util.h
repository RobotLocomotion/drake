#pragma once

#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

template <typename T>
class DesiredTrajectorySource : public systems::Diagram<T> {
 public:
  DesiredTrajectorySource(std::unique_ptr<PiecewisePolynomialTrajectory> traj,
                          int derivative_degree);

  const systems::OutputPortDescriptor<T>& get_output_port_state() const {
    return this->get_output_port(0);
  }

  const systems::OutputPortDescriptor<T>& get_output_port_acceleration() const {
    DRAKE_DEMAND(trajdd_ != nullptr);
    return this->get_output_port(1);
  }

 private:
  std::unique_ptr<PiecewisePolynomialTrajectory> traj_{nullptr};
  std::unique_ptr<Trajectory> trajd_{nullptr};
  std::unique_ptr<Trajectory> trajdd_{nullptr};
};

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
