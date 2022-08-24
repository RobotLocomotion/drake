#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/traj_opt/examples/example_base.h"

namespace drake {
namespace traj_opt {
namespace examples {
namespace two_dof_spinner {

using multibody::MultibodyPlant;
using multibody::Parser;

class TwoDofSpinnerExample : public TrajOptExample {
  void CreatePlantModel(MultibodyPlant<double>* plant) const {
    const std::string urdf_file =
        FindResourceOrThrow("drake/traj_opt/examples/2dof_spinner.urdf");
    Parser(plant).AddAllModelsFromFile(urdf_file);
  }
};

}  // namespace two_dof_spinner
}  // namespace examples
}  // namespace traj_opt
}  // namespace drake

int main() {
  drake::traj_opt::examples::two_dof_spinner::TwoDofSpinnerExample
      spinner_example;
  spinner_example.SolveTrajectoryOptimization(
      "drake/traj_opt/examples/2dof_spinner.yaml");
  return 0;
}
