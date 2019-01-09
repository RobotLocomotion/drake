#include "drake/systems/optimization/test/system_optimization_test_util.h"

#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/parser.h"

namespace drake {
namespace systems {

std::unique_ptr<multibody::MultibodyPlant<double>> ConstructIiwa(
    const std::string& sdf_file, double time_step) {
  const std::string iiwa_path = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/sdf/" + sdf_file);
  auto plant = std::make_unique<multibody::MultibodyPlant<double>>(time_step);
  multibody::Parser(plant.get()).AddModelFromFile(iiwa_path);
  plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("iiwa_link_0"));
  plant->Finalize();
  return plant;
}
}  // namespace systems
}  // namespace drake
