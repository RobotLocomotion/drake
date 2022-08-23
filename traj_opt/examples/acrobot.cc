#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/traj_opt/examples/example_base.h"

namespace drake {
namespace traj_opt {
namespace examples {
namespace acrobot {

using multibody::MultibodyPlant;
using multibody::Parser;

class AcrobotExample : public TrajOptExample {
  void CreatePlantModel(MultibodyPlant<double>* plant) const {
    const std::string urdf_file =
        FindResourceOrThrow("drake/examples/acrobot/Acrobot.urdf");
    Parser(plant).AddAllModelsFromFile(urdf_file);
  }
};

}  // namespace acrobot
}  // namespace examples
}  // namespace traj_opt
}  // namespace drake

int main() {
  drake::traj_opt::examples::acrobot::AcrobotExample acrobot_example;
  acrobot_example.SolveTrajectoryOptimization(
      "drake/traj_opt/examples/acrobot.yaml");
  return 0;
}
