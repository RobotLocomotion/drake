#include "drake/examples/Cars/car_simulation.h"
#include "drake/examples/Cars/gen/driving_command.h"
#include "drake/common/drake_path.h"
#include "drake/systems/LCMSystem.h"
#include "drake/systems/LinearSystem.h"
#include "drake/systems/pd_control_system.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/util/drakeAppUtil.h"
#include "lcmtypes/drake/lcmt_driving_command_t.hpp"

using drake::BotVisualizer;
using drake::Gain;
using drake::SimulationOptions;

using Eigen::VectorXd;

namespace drake {
namespace examples {
namespace cars {
namespace {

int do_main(int argc, const char* argv[]) {
  // Initializes the communication layer.
  std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();

  // Instantiates a rigid body system.
  auto rigid_body_sys = std::allocate_shared<RigidBodySystem>(
      Eigen::aligned_allocator<RigidBodySystem>());

  // Adds a multiple copies of the same SDF into the same rigid body system.
  const int kNumCarsPerRow = 3;
  const int kNumTotalCars = 12;

  for (int ii = 0; ii < kNumTotalCars; ++ii) {
    int x_coord = ii % kNumCarsPerRow;
    int y_coord = ii - x_coord * kNumCarsPerRow;

    auto car_offset = std::allocate_shared<RigidBodyFrame>(
        Eigen::aligned_allocator<RigidBodyFrame>(),
        // Weld the model to the world link.
        "world",

        // A pointer to a rigid body to which to weld the model is not needed
        // since the model will be welded to the world, which can be
        // automatically found within the rigid body tree.
        nullptr,

        // The following parameter specifies the X,Y,Z position of the car's
        // root link in the world's frame.
        Eigen::Vector3d(x_coord * 5, y_coord * 5, 0.378326),

        // The following parameter specifies the roll, pitch, and yaw of the
        // car's root link in the world's frame.
        Eigen::Vector3d(0, 0, 0));

    rigid_body_sys->addRobotFromFile(drake::GetDrakePath() +
      "/examples/Cars/models/prius/prius.sdf",
      DrakeJoint::QUATERNION, car_offset);
  }

  SetRigidBodySystemParameters(rigid_body_sys.get());

  double duration = ParseDuration(argc, argv);

  auto const& tree = rigid_body_sys->getRigidBodyTree();

  AddFlatTerrain(tree);

  // Initializes and cascades all of the systems.
  auto visualizer =
      std::make_shared<BotVisualizer<RigidBodySystem::StateVector>>(lcm, tree);

  auto sys = cascade(rigid_body_sys, visualizer);

  // Initializes the simulation options.
  SimulationOptions options =
      GetCarSimulationDefaultOptions();

  // Starts the simulation.
  drake::runLCM(sys, lcm, 0, duration,
                GetInitialState(*(rigid_body_sys.get())),
                options);

  return 0;
}

}  // namespace
}  // namespace cars
}  // namespace examples
}  // namespace drake

int main(int argc, const char* argv[]) {
  return drake::examples::cars::do_main(argc, argv);
}
