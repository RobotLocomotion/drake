#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/traj_opt/examples/example_base.h"

namespace drake {
namespace traj_opt {
namespace examples {
namespace wall_ball {

using multibody::MultibodyPlant;
using multibody::Parser;

/**
 * A simple 1-DoF system with contact
 *
 */
class WallBallExample : public TrajOptExample {
  void CreatePlantModel(MultibodyPlant<double>* plant) const {
    const std::string urdf_file =
        FindResourceOrThrow("drake/traj_opt/examples/wall_ball.urdf");
    Parser(plant).AddAllModelsFromFile(urdf_file);
  }
};

}  // namespace wall_ball
}  // namespace examples
}  // namespace traj_opt
}  // namespace drake

int main() {
  drake::traj_opt::examples::wall_ball::WallBallExample wall_ball_example;
  wall_ball_example.SolveTrajectoryOptimization(
      "drake/traj_opt/examples/wall_ball.yaml");
  return 0;
}
