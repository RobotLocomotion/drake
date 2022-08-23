#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/traj_opt/examples/example_base.h"

namespace drake {
namespace traj_opt {
namespace examples {
namespace spinner {

DEFINE_string(geometry, "sphere",
              "Geometry of the spinner. Options are 'rectangle', 'capsule', "
              "'square', or 'sphere'.");

using multibody::MultibodyPlant;
using multibody::Parser;

class SpinnerExample : public TrajOptExample {
  void CreatePlantModel(MultibodyPlant<double>* plant) const {
    // N.B. geometry of the spinner is chosen via gflags rather than yaml so
    // that we can use the same yaml format for all of the examples, without
    // cluttering it with spinner-specific options.
    std::string urdf_file;
    if (FLAGS_geometry == "rectangle") {
      urdf_file =
          FindResourceOrThrow("drake/traj_opt/examples/spinner_rectangle.urdf");
    } else if (FLAGS_geometry == "capsule") {
      urdf_file =
          FindResourceOrThrow("drake/traj_opt/examples/spinner_capsule.urdf");
    } else if (FLAGS_geometry == "square") {
      urdf_file =
          FindResourceOrThrow("drake/traj_opt/examples/spinner_square.urdf");
    } else if (FLAGS_geometry == "sphere") {
      urdf_file =
          FindResourceOrThrow("drake/traj_opt/examples/spinner_sphere.urdf");
    } else {
      throw std::runtime_error(
          fmt::format("Unknown spinner geometry '{}'.", FLAGS_geometry));
    }
    Parser(plant).AddAllModelsFromFile(urdf_file);
  }
};

}  // namespace spinner
}  // namespace examples
}  // namespace traj_opt
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  drake::traj_opt::examples::spinner::SpinnerExample spinner_example;
  spinner_example.SolveTrajectoryOptimization(
      "drake/traj_opt/examples/spinner.yaml");
  return 0;
}
