#include "drake/systems/sensors/depth_sensor_to_lcm_point_cloud_message.h"

#include "bot_core/pointcloud_t.hpp"
#include <gflags/gflags.h>

#include "drake/common/text_logging_gflags.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
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
  const std::string kSensorName = "Foo";
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  logging::HandleSpdlogGflags();
  ::drake::lcm::DrakeLcm real_lcm;

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

  systems::DiagramBuilder<double> builder;
  auto vis =
      builder.template AddSystem<DepthSensorToLcmPointCloudMessage>(spec);
  builder.ExportInput(vis->depth_readings_input_port());

  auto lcm_publisher = builder.template AddSystem(
      lcm::LcmPublisherSystem::Make<bot_core::pointcloud_t>(
          "DRAKE_POINTCLOUD_" + kSensorName, &real_lcm));
  builder.Connect(
      vis->pointcloud_message_output_port(),
      lcm_publisher->get_input_port(0));


  auto diagram = builder.Build();
  auto depth_sensor_output = std::make_unique<DepthSensorOutput<double>>(spec);
  const double half_range = (spec.max_range() - spec.min_range()) / 2;
  depth_sensor_output->SetFromVector(Eigen::VectorXd::Ones(
      spec.num_depth_readings()) * half_range);

  auto context = diagram->CreateDefaultContext();
  context->FixInputPort(0, std::move(depth_sensor_output));

  auto simulator = std::make_unique<systems::Simulator<double>>(
      *diagram, std::move(context));
  simulator->Initialize();
  simulator->StepTo(0.005);
  simulator->StepTo(0.01);
  return 0;
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::systems::sensors::main(argc, argv);
}
