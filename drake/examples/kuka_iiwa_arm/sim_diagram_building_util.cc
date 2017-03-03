#include "drake/examples/kuka_iiwa_arm/sim_diagram_building_util.h"

#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/multiplexer.h"
#include "drake/systems/primitives/trajectory_source.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

template <typename T>
DesiredTrajectorySource<T>::DesiredTrajectorySource(
    std::unique_ptr<PiecewisePolynomialTrajectory> traj, int derivative_degree)
    : traj_(std::move(traj)) {
  DRAKE_DEMAND(derivative_degree >= 0);
  if (derivative_degree >= 1) {
    trajd_ = traj_->derivative(1);
  }
  if (derivative_degree >= 2) {
    trajdd_ = traj_->derivative(2);
  }

  systems::DiagramBuilder<T> builder;

  // Adds desired trajectory source.
  auto traj_src =
      builder.template AddSystem<systems::TrajectorySource<T>>(*traj_);
  const int dim = traj_src->get_output_port().size();

  auto input_mux = builder.template AddSystem<systems::Multiplexer<T>>(
      std::vector<int>{dim, dim});
  builder.Connect(traj_src->get_output_port(), input_mux->get_input_port(0));

  if (trajd_) {
    auto trajd_src =
        builder.template AddSystem<systems::TrajectorySource<T>>(*trajd_);
    builder.Connect(trajd_src->get_output_port(), input_mux->get_input_port(1));
  } else {
    auto zero_src =
        builder.template AddSystem<systems::ConstantVectorSource<T>>(
            Eigen::VectorXd::Zero(dim));
    builder.Connect(zero_src->get_output_port(), input_mux->get_input_port(1));
  }

  // Expose state output.
  builder.ExportOutput(input_mux->get_output_port(0));

  // Adds desired acceleration source.
  if (trajdd_) {
    auto trajdd_src =
        builder.template AddSystem<systems::TrajectorySource<T>>(*trajdd_);
    builder.ExportOutput(trajdd_src->get_output_port());
  }

  builder.BuildInto(this);
}

template class DesiredTrajectorySource<double>;

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
