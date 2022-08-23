#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/traj_opt/examples/example_base.h"

namespace drake {
namespace traj_opt {
namespace examples {
namespace pendulum {

using multibody::MultibodyPlant;
using multibody::Parser;

class PendulumExample : public TrajOptExample {
  void CreatePlantModel(MultibodyPlant<double>* plant) const {
    const std::string urdf_file =
        FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf");
    Parser(plant).AddAllModelsFromFile(urdf_file);
  }
};

}  // namespace pendulum
}  // namespace examples
}  // namespace traj_opt
}  // namespace drake

int main() {
  drake::traj_opt::examples::pendulum::PendulumExample pendulum_example;
  pendulum_example.SolveTrajectoryOptimization(
      "drake/traj_opt/examples/pendulum.yaml");
  return 0;
}
