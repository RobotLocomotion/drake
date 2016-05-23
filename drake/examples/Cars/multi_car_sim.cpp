#include "drake/examples/Cars/car_simulation.h"
#include "drake/examples/Cars/gen/driving_command.h"
#include "drake/Path.h"
#include "drake/systems/LCMSystem.h"
#include "drake/systems/LinearSystem.h"
#include "drake/systems/pd_control_system.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/util/drakeAppUtil.h"
#include "lcmtypes/drake/lcmt_driving_command_t.hpp"

using Drake::BotVisualizer;
using Drake::Gain;
using Drake::SimulationOptions;

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

  const int kNumCarsPerRow = 3;
  for (int ii = 0; ii < 12; ii++) {

    int x_coord = ii % kNumCarsPerRow;
    int y_coord = ii - x_coord * kNumCarsPerRow;

    auto car_offset = std::allocate_shared<RigidBodyFrame>(
        Eigen::aligned_allocator<RigidBodyFrame>(),
        // Weld the model to the world link.
        "world",

        // A pointer to a rigid body to which to weld the model is not needed
        // since the model will be welded to the world, which can by automatically
        // found within the rigid body tree.
        nullptr,

        // The following parameter specifies the X,Y,Z position of the car's root
        // link in the world's frame.
        Eigen::Vector3d(x_coord * 5, y_coord * 5, 0.378326),

        // The following parameter specifies the roll, pitch, and yaw of the car's
        // root link in the world's frame.
        Eigen::Vector3d(0, 0, 0));

    std::stringstream postfix_buff;
    postfix_buff << "_" << ii;
    std::string postfix = postfix_buff.str();

    rigid_body_sys->addRobotFromFile(Drake::getDrakePath() +
      "/examples/Cars/models/prius/prius.sdf", &postfix,
      DrakeJoint::QUATERNION, car_offset);
  }

  // auto car_offset_1 = std::allocate_shared<RigidBodyFrame>(
  //     Eigen::aligned_allocator<RigidBodyFrame>(),
  //     // Weld the model to the world link.
  //     "world",

  //     // A pointer to a rigid body to which to weld the model is not needed
  //     // since the model will be welded to the world, which can by automatically
  //     // found within the rigid body tree.
  //     nullptr,

  //     // The following parameter specifies the X,Y,Z position of the car's root
  //     // link in the world's frame.
  //     Eigen::Vector3d(0, 0, 0.378326),

  //     // The following parameter specifies the roll, pitch, and yaw of the car's
  //     // root link in the world's frame.
  //     Eigen::Vector3d(0, 0, 0));

  // auto car_offset_2 = std::allocate_shared<RigidBodyFrame>(
  //     Eigen::aligned_allocator<RigidBodyFrame>(),
  //     // Weld the model to the world link.
  //     "world",

  //     // A pointer to a rigid body to which to weld the model is not needed
  //     // since the model will be welded to the world, which can by automatically
  //     // found within the rigid body tree.
  //     nullptr,

  //     // The following parameter specifies the X,Y,Z position of the car's root
  //     // link in the world's frame.
  //     Eigen::Vector3d(5, 5, 0.378326),

  //     // The following parameter specifies the roll, pitch, and yaw of the car's
  //     // root link in the world's frame.
  //     Eigen::Vector3d(0, 0, 0));

  // std::string postfix_1 = "_1";
  // std::string postfix_2 = "_2";

  // rigid_body_sys->addRobotFromFile(Drake::getDrakePath() +
  //   "/examples/Cars/models/prius/prius.sdf", &postfix_1,
  //   DrakeJoint::QUATERNION, car_offset_1);

  // rigid_body_sys->addRobotFromFile(Drake::getDrakePath() +
  //   "/examples/Cars/models/prius/prius.sdf", &postfix_2, DrakeJoint::QUATERNION,
  //   car_offset_2);

  // TODO(liangfok) move this into a method in car_simulation.h/.cc
  rigid_body_sys->penetration_stiffness = 5000.0;
  rigid_body_sys->penetration_damping = rigid_body_sys->penetration_stiffness / 10.0;
  rigid_body_sys->friction_coefficient = 10.0;  // essentially infinite friction

  // std::cout << "Number of outputs: " << rigid_body_sys->getNumOutputs() << std::endl;
  // std::cout << "Number of positions: " << rigid_body_sys->number_of_positions() << std::endl;
  // std::cout << "Number of velocities: " << rigid_body_sys->number_of_velocities() << std::endl;

  double duration = std::numeric_limits<double>::infinity();

  auto const& tree = rigid_body_sys->getRigidBodyTree();

  AddFlatTerrain(tree);

  // Initializes and cascades all of the other systems.
  // auto vehicle_sys = CreateVehicleSystem(rigid_body_sys);

  auto visualizer =
      std::make_shared<BotVisualizer<RigidBodySystem::StateVector>>(lcm, tree);

  auto sys = cascade(rigid_body_sys, visualizer);

  // Initializes the simulation options.
  SimulationOptions options =
      GetCarSimulationDefaultOptions();

  // Starts the simulation.
  Drake::runLCM(sys, lcm, 0, duration,
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
