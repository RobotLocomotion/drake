#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/monolithic_pick_and_place_system.h"

#include <vector>

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"
#include "drake/examples/kuka_iiwa_arm/pick_and_place/pick_and_place_configuration.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

DEFINE_bool(visualize, false, "Publish visuzliztion messages over LCM.");

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {
namespace {

using systems::RungeKutta2Integrator;
using systems::Simulator;

const char kIiwaPath[] =
    "drake/manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";
const char kEndEffectorName[] = "iiwa_link_ee";
const char kYellowPostPath[] =
    "drake/examples/kuka_iiwa_arm/models/objects/yellow_post.urdf";
const char kExtraHeavyDutyTablePath[] =
    "drake/examples/kuka_iiwa_arm/models/table/"
    "extra_heavy_duty_table_surface_only_collision.sdf";
const char kTargetPath[] =
    "drake/examples/kuka_iiwa_arm/models/objects/simple_cuboid.urdf";
const Vector3<double> kTargetDimensions{0.06, 0.06, 0.06};

const std::vector<std::string> kTableModelPaths{
    kYellowPostPath,           // position A
    kYellowPostPath,           // position B
    kExtraHeavyDutyTablePath,  // position C
    kYellowPostPath,           // position D
    kYellowPostPath,           // position E
    kYellowPostPath,           // position F
};

const std::vector<Vector3<double>> kTablePositions{
    {0.00, 1.00, 0.0},     // position A
    {0.80, 0.36, 0.0},     // position B
    {0.86, -0.36, -0.07},  // position C
    {0.30, -0.9, 0.0},     // position D
    {-0.1, -1.0, 0.0},     // position E
    {-0.47, -0.8, 0.0},    // position F
};

const std::vector<double> kTableOptitrackFrameHeights{
    1.02,    // position A
    1.02,    // position B
    0.7645,  // position C
    1.02,    // position D
    1.02,    // position E
    1.02,    // position F
};

double ExpectedObjectOrientation(Vector3<double> table_position) {
  return atan2(table_position.y(), table_position.x());
}

class SingleMoveTests : public ::testing::TestWithParam<std::tuple<int, int>> {
 protected:
  void SetUp() {
    initial_table_index_ = std::get<0>(GetParam());
    final_table_index_ = std::get<1>(GetParam());
    // Set Optitrack parameters.
    optitrack_configuration_.robot_base_optitrack_info.emplace_back();
    optitrack_configuration_.robot_base_optitrack_info.back().id = 1;
    optitrack_configuration_.table_optitrack_info.emplace_back();
    optitrack_configuration_.table_optitrack_info.back().id = 2;
    optitrack_configuration_.table_optitrack_info.back()
        .X_MF.translation()
        .z() += kTableOptitrackFrameHeights[initial_table_index_];
    optitrack_configuration_.table_optitrack_info.emplace_back();
    optitrack_configuration_.table_optitrack_info.back().id = 3;
    optitrack_configuration_.table_optitrack_info.back()
        .X_MF.translation()
        .z() += kTableOptitrackFrameHeights[final_table_index_];
    optitrack_configuration_.object_optitrack_info.emplace_back();
    optitrack_configuration_.object_optitrack_info.back().id = 4;

    // Set plant parameters.
    plant_configuration_.robot_poses.push_back(Isometry3<double>::Identity());
    plant_configuration_.robot_poses.back().translation().z() = 0.7645;
    plant_configuration_.robot_models.push_back(kIiwaPath);

    plant_configuration_.table_models.push_back(
        kTableModelPaths[initial_table_index_]);
    plant_configuration_.table_models.push_back(
        kTableModelPaths[final_table_index_]);
    plant_configuration_.table_poses.resize(2, Isometry3<double>::Identity());
    plant_configuration_.table_poses[0].translation() =
        kTablePositions[initial_table_index_];
    plant_configuration_.table_poses[1].translation() =
        kTablePositions[final_table_index_];

    plant_configuration_.object_models.push_back(kTargetPath);
    plant_configuration_.object_poses.push_back(
        plant_configuration_.table_poses[0] *
        optitrack_configuration_.table_optitrack_info[0].X_MF);
    plant_configuration_.object_poses.back().translation().z() +=
        0.5 * kTargetDimensions.z();
    plant_configuration_.object_poses.back().rotate(AngleAxis<double>(
        ExpectedObjectOrientation(kTablePositions[initial_table_index_]),
        Vector3<double>::UnitZ()));
    plant_configuration_.default_contact_material.set_youngs_modulus(3e7);
    plant_configuration_.default_contact_material.set_dissipation(5);

    // Set planner parameters
    planner_configuration_.drake_relative_model_path = kIiwaPath;
    planner_configuration_.end_effector_name = kEndEffectorName;
    planner_configuration_.target_dimensions = kTargetDimensions;
    planner_configuration_.num_tables = 2;
  }

  void ValidateObjectStateAfterSingleMove() {
    systems::DiagramBuilder<double> builder;

    std::vector<pick_and_place::PlannerConfiguration> planner_configurations{
        planner_configuration_};
    auto plant = builder.AddSystem<MonolithicPickAndPlaceSystem>(
        plant_configuration_, optitrack_configuration_, planner_configurations,
        true /*single_move*/);

    std::unique_ptr<lcm::DrakeLcm> lcm{nullptr};
    if (FLAGS_visualize) {
      lcm = std::make_unique<lcm::DrakeLcm>();
      auto drake_visualizer = builder.AddSystem<systems::DrakeVisualizer>(
          plant->get_tree(), lcm.get());
      drake_visualizer->set_publish_period(kIiwaLcmStatusPeriod);
      builder.Connect(plant->get_output_port_plant_state(),
                      drake_visualizer->get_input_port(0));
      lcm->StartReceiveThread();
    }

    auto sys = builder.Build();
    Simulator<double> simulator(*sys);
    simulator.reset_integrator<RungeKutta2Integrator<double>>(
        *sys, dt_, &simulator.get_mutable_context());
    simulator.get_mutable_integrator()->set_fixed_step_mode(true);
    simulator.set_publish_every_time_step(false);
    simulator.Initialize();

    // Initialize the plan interpolators
    plant->Initialize(&sys->GetMutableSubsystemContext(
        *plant, &simulator.get_mutable_context()));

    // Step the simulator in some small increment.  Between steps, check
    // to see if the state machine thinks we're done, and if so that the
    // object is near the target.
    const double simulation_step = 0.5;
    bool done{false};
    while (!done) {
      simulator.StepTo(simulator.get_context().get_time() + simulation_step);
      done = plant->is_done(
          sys->GetSubsystemContext(*plant, simulator.get_context()));
    }

    // Verify final state of the object.
    // Frames:
    //   - S: Sensor (Optitrack) frame
    //   - O: Object frame
    //   - T: Table frame of "place" table (center of the table top)
    const pick_and_place::WorldState& world_state = plant->world_state(
        sys->GetSubsystemContext(*plant, simulator.get_context()), 0);
    const Isometry3<double>& X_SO = world_state.get_object_pose();
    const Isometry3<double>& X_TS =
        world_state.get_table_poses().back().inverse();
    const Isometry3<double> X_TO = X_TS * X_SO;

    const Vector6<double>& object_velocity = world_state.get_object_velocity();
    Isometry3<double> X_TO_expected{Isometry3<double>::TranslationType(
        0.0, 0.0, 0.5 * kTargetDimensions.z())};
    X_TO_expected.linear() =
        Isometry3<double>::LinearMatrixType(AngleAxis<double>(
            ExpectedObjectOrientation(kTablePositions[final_table_index_]),
            Vector3<double>::UnitZ()));
    Eigen::Vector3d object_rpy = math::rotmat2rpy(X_TO.linear());
    Eigen::Vector3d X_TO_expected_rpy =
        math::rotmat2rpy(X_TO_expected.linear());

    drake::log()->info("Pose: {} {}", X_TO.translation().transpose(),
                       object_rpy.transpose());
    drake::log()->info("Velocity: {}", object_velocity.transpose());
    drake::log()->info("Goal: {} {}", X_TO_expected.translation().transpose(),
                       X_TO_expected_rpy.transpose());

    // TODO(avalenzu): Bring this back to 0.02 when planning is less brittle.
    const double position_tolerance = 0.03;
    // TODO(avalenzu): Bring this back to 5 degrees  when planning is less
    // brittle.
    const double angle_tolerance = 10.0 * M_PI / 180;
    const double linear_velocity_tolerance = 0.1;
    const double angular_velocity_tolerance = 0.1;
    EXPECT_TRUE(CompareMatrices(X_TO.translation(), X_TO_expected.translation(),
                                position_tolerance));

    EXPECT_TRUE(
        CompareMatrices(object_rpy, X_TO_expected_rpy, angle_tolerance));
    EXPECT_TRUE(CompareMatrices(object_velocity.head(3),
                                Vector3<double>::Zero(),
                                angular_velocity_tolerance));
    EXPECT_TRUE(CompareMatrices(object_velocity.tail(3),
                                Vector3<double>::Zero(),
                                linear_velocity_tolerance));
  }

  pick_and_place::SimulatedPlantConfiguration plant_configuration_;
  pick_and_place::OptitrackConfiguration optitrack_configuration_;
  pick_and_place::PlannerConfiguration planner_configuration_;
  int initial_table_index_;
  int final_table_index_;
  double dt_{3e-4};
};

TEST_P(SingleMoveTests, FinalObjectStateTest) {
  // Verify final state of the object.
  ValidateObjectStateAfterSingleMove();
}

const std::vector<int> kTableIndices{0, 1, 2, 3, 4, 5};
INSTANTIATE_TEST_CASE_P(InitialTable0, SingleMoveTests,
                        ::testing::Combine(::testing::Values(0),
                                           ::testing::Values(1, 2, 3, 4, 5)));
INSTANTIATE_TEST_CASE_P(InitialTable1, SingleMoveTests,
                        ::testing::Combine(::testing::Values(1),
                                           ::testing::Values(0, 2, 3, 4, 5)));
INSTANTIATE_TEST_CASE_P(InitialTable2, SingleMoveTests,
                        ::testing::Combine(::testing::Values(2),
                                           ::testing::Values(0, 1, 3, 4, 5)));
INSTANTIATE_TEST_CASE_P(InitialTable3, SingleMoveTests,
                        ::testing::Combine(::testing::Values(3),
                                           ::testing::Values(0, 1, 2, 4, 5)));
INSTANTIATE_TEST_CASE_P(InitialTable4, SingleMoveTests,
                        ::testing::Combine(::testing::Values(4),
                                           ::testing::Values(0, 1, 2, 3, 5)));
INSTANTIATE_TEST_CASE_P(InitialTable5, SingleMoveTests,
                        ::testing::Combine(::testing::Values(5),
                                           ::testing::Values(0, 1, 2, 3, 4)));

class SingleMoveExtraArmTests : public SingleMoveTests {
 protected:
  void SetUp() {
    SingleMoveTests::SetUp();
    // Add extra arm
    plant_configuration_.robot_models.push_back(kIiwaPath);
    plant_configuration_.robot_poses.push_back(Isometry3<double>::Identity());
    plant_configuration_.robot_poses.back().translation().y() = -2.5;
    plant_configuration_.robot_poses.back().translation().z() = 0.7645;
    optitrack_configuration_.robot_base_optitrack_info.emplace_back();
    optitrack_configuration_.robot_base_optitrack_info.back().id = 5;
  }
};

TEST_P(SingleMoveExtraArmTests, FinalObjectStateTest) {
  // Verify final state of the object.
  ValidateObjectStateAfterSingleMove();
}

// Verify that the plumbing works when there's an extra arm that's not connected
// to a planner.
INSTANTIATE_TEST_CASE_P(SelectedPairs, SingleMoveExtraArmTests,
                        ::testing::Values(std::make_tuple(0, 2)));

}  // namespace
}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
