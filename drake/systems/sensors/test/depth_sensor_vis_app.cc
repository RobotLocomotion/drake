#include "drake/systems/sensors/depth_sensor_vis.h"

#include <gflags/gflags.h>

#include "drake/common/text_logging_gflags.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/sensors/depth_sensor_output.h"

DEFINE_string(spec, "octant_1", "The depth sensor specification to use. Valid "
    "values include:\n"
    "  - octant_1\n"
    "  - xy_planar\n"
    "  - xz_planar\n"
    "  - xyz_spherical\n"
    "  - x_linear");

namespace drake {
namespace systems {
namespace sensors {

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  logging::HandleSpdlogGflags();
  lcm::DrakeLcm lcm;

  DepthSensorSpecification spec;
  if (FLAGS_spec == "octant_1") {
    DepthSensorSpecification::set_octant_1_spec(&spec);
  } else if (FLAGS_spec == "xy_planar") {
    DepthSensorSpecification::set_xy_planar_spec(&spec);
  } else if (FLAGS_spec == "xz_planar") {
    DepthSensorSpecification::set_xz_planar_spec(&spec);
  } else if (FLAGS_spec == "xyz_spherical") {
    DepthSensorSpecification::set_xyz_spherical_spec(&spec);
  } else if (FLAGS_spec == "x_linear") {
    DepthSensorSpecification::set_x_linear_spec(&spec);
  } else {
    throw std::runtime_error("Unknown spec type \"" + FLAGS_spec + "\".");
  }

  DepthSensorVis vis("Foo", spec, &lcm);

  const InputPortDescriptor<double>& input_port =
      vis.depth_readings_input_port();

  auto depth_sensor_output = std::make_unique<DepthSensorOutput<double>>(spec);

  const double half_range = (spec.max_range() - spec.min_range()) / 2;
  depth_sensor_output->SetFromVector(Eigen::VectorXd::Ones(
      spec.num_depth_readings()) * half_range);

  std::unique_ptr<Context<double>> context = vis.CreateDefaultContext();
  context->FixInputPort(input_port.get_index(), std::move(depth_sensor_output));
  vis.Publish(*context);
  return 0;
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::systems::sensors::main(argc, argv);
}
