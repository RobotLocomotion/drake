#include "drake/examples/hsr/controllers/main_controller.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/examples/hsr/parameters/robot_parameters_loader.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/parser.h"

namespace drake {
namespace examples {
namespace hsr {
namespace controllers {
namespace {

constexpr double kMaxTimeStep = 1e-3;

class MainControllerTest : public ::testing::Test {
 public:
  MainControllerTest() {
    LoadPlant();

    LoadRobotParameters();

    PopulateTestingStatesValue();
  }

  const Eigen::VectorXd& estimated_state() { return estimated_state_; }
  const Eigen::VectorXd& desired_state() { return desired_state_; }

  const multibody::MultibodyPlant<double>& plant() { return plant_; }
  const multibody::MultibodyPlant<double>& welded_plant() {
    return welded_plant_;
  }

  const parameters::RobotParameters<double>& robot_parameters() {
    return robot_parameters_;
  }

  void LoadPlant() {
    const std::string model_path = FindResourceOrThrow(model_path_);
    multibody::Parser(&plant_).AddModelFromFile(model_path);
    plant_.Finalize();

    // Create the same plant but with the base welded to the ground.
    const auto welded_robot_model =
        multibody::Parser(&welded_plant_).AddModelFromFile(model_path);

    // The welded plant is only used for the inverse dynamics controller
    // calculation purpose. Here we assume the robot only has one floating
    // body, which should be true.
    const std::unordered_set<multibody::BodyIndex> floating_base_indexes =
        plant_.GetFloatingBaseBodies();
    DRAKE_DEMAND(floating_base_indexes.size() == 1);

    welded_plant_.WeldFrames(
        welded_plant_.world_frame(),
        welded_plant_.GetFrameByName(
            plant_.get_body(*(floating_base_indexes.begin())).name(),
            welded_robot_model),
        math::RigidTransform<double>::Identity());
    welded_plant_.Finalize();
  }

  void LoadRobotParameters() {
    robot_parameters_.name = robot_name_;
    const bool load_successful = hsr::parameters::ReadParametersFromFile(
        robot_name_, filepath_prefix_, &robot_parameters_);
    DRAKE_DEMAND(load_successful);
  }

  void PopulateTestingStatesValue() {
    const int num_positions = plant_.num_positions();
    const int num_velocities = plant_.num_velocities();
    const int state_size = num_positions + num_velocities;

    estimated_state_ = Eigen::VectorXd::Zero(state_size);
    estimated_state_[0] = 1;
    desired_state_ = Eigen::VectorXd::Zero(state_size);
    desired_state_[0] = 1;

    for (int i = 4; i < state_size; ++i) {
      estimated_state_[i] = i * i;
      desired_state_[i] = i * i * i;
    }
  }

 private:
  const std::string robot_name_ = "hsr";
  const std::string filepath_prefix_ = "drake/examples/hsr/models/config/";
  const std::string model_path_ =
      "drake/examples/hsr/models/urdfs/hsrb4s_fix_free_joints.urdf";
  multibody::MultibodyPlant<double> plant_{kMaxTimeStep};
  multibody::MultibodyPlant<double> welded_plant_{kMaxTimeStep};

  parameters::RobotParameters<double> robot_parameters_;

  Eigen::VectorXd estimated_state_;
  Eigen::VectorXd desired_state_;
};

TEST_F(MainControllerTest, ConstructionTest) {
  MainController controller_test(plant(), welded_plant(), robot_parameters());

  std::unique_ptr<systems::Context<double>> context =
      controller_test.CreateDefaultContext();

  controller_test.get_desired_state_input_port().FixValue(context.get(),
                                                          desired_state());
  controller_test.get_estimated_state_input_port().FixValue(context.get(),
                                                            estimated_state());

  const auto& output_generalized_force =
      controller_test.get_generalized_force_output_port().Eval(*context);

  // It's hard to predict the values coming out from the Drake inverse dynamics
  // controller. Here, we check that at least two generalized forces should be
  // non zero value and the actuation port output should always be zero.
  // Since the first three elements are always 0 (corresponding to the
  // floating base), we start from the forth one.
  const double kErrorTol = 0.01;

  int non_zero_count = 0;
  for (int i = 3; i < plant().num_velocities(); ++i) {
    if (std::abs(output_generalized_force[i]) > kErrorTol) {
      non_zero_count += 1;
    }
  }
  EXPECT_GT(non_zero_count, 1);
}

}  // namespace
}  // namespace controllers
}  // namespace hsr
}  // namespace examples
}  // namespace drake
