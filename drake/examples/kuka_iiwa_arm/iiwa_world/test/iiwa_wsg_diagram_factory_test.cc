#include "drake/examples/kuka_iiwa_arm/iiwa_world/iiwa_wsg_diagram_factory.h"

#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/manipulation/util/world_sim_tree_builder.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

constexpr int kNumIiwaPositions = 7;
constexpr int kNumWsgPositions = 5;
constexpr int kNumObjectPositions = 7;
constexpr int kNumIiwaVelocities = 7;
constexpr int kNumWsgVelocities = 5;
constexpr int kNumObjectVelocities = 6;

class IiwaAndWsgPlantWithStateEstimatorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    tree_builder_.StoreModel("iiwa",
                             "drake/manipulation/models/iiwa_description/urdf/"
                             "iiwa14_polytope_collision.urdf");
    tree_builder_.StoreModel("wsg",
                             "drake/manipulation/models/wsg_50_description"
                             "/sdf/schunk_wsg_50_ball_contact.sdf");
    tree_builder_.StoreModel(
        "object",
        "drake/examples/kuka_iiwa_arm/models/objects/simple_cuboid.urdf");
  }
  void BuildDefaultPlant() { BuildPlant(1, 1, 1); }
  void BuildPlant(int num_iiwas, int num_wsgs, int num_objects) {
    for (int i = 0; i < num_iiwas; ++i) {
      int id =
          tree_builder_.AddFixedModelInstance("iiwa", Vector3<double>::Zero());
      iiwa_instances_.push_back(tree_builder_.get_model_info_for_instance(id));
    }
    for (int i = 0; i < num_wsgs; ++i) {
      int id =
          tree_builder_.AddFixedModelInstance("wsg", Vector3<double>::Zero());
      wsg_instances_.push_back(tree_builder_.get_model_info_for_instance(id));
    }
    for (int i = 0; i < num_objects; ++i) {
      int id = tree_builder_.AddFloatingModelInstance("object",
                                                      Vector3<double>::Zero());
      object_instances_.push_back(
          tree_builder_.get_model_info_for_instance(id));
    }
    combined_plant_ = std::make_unique<systems::RigidBodyPlant<double>>(
        tree_builder_.Build());
  }
  std::unique_ptr<IiwaAndWsgPlantWithStateEstimator<double>>
  MakeIiwaAndWsgPlantWithStateEstimator() {
    return std::make_unique<IiwaAndWsgPlantWithStateEstimator<double>>(
        std::move(combined_plant_), iiwa_instances_, wsg_instances_,
        object_instances_);
  }
  manipulation::util::WorldSimTreeBuilder<double> tree_builder_;
  std::unique_ptr<systems::RigidBodyPlant<double>> combined_plant_;
  std::vector<manipulation::util::ModelInstanceInfo<double>> iiwa_instances_;
  std::vector<manipulation::util::ModelInstanceInfo<double>> wsg_instances_;
  std::vector<manipulation::util::ModelInstanceInfo<double>> object_instances_;
};

class IiwaAndWsgPlantWithStateEstimatorParameterizedTest
    : public IiwaAndWsgPlantWithStateEstimatorTest,
      public ::testing::WithParamInterface<std::tuple<int, int>> {};

void CheckNumPositionsAndVelocities(
    const IiwaAndWsgPlantWithStateEstimator<double>& iiwa_and_wsg_plant,
    int num_iiwas, int num_wsgs, int num_objects) {
  // Check the number of positions and velocities in the contained tree.
  EXPECT_EQ(iiwa_and_wsg_plant.get_tree().get_num_positions(),
            num_iiwas * kNumIiwaPositions + num_wsgs * kNumWsgPositions +
                num_objects * kNumObjectPositions);
  EXPECT_EQ(iiwa_and_wsg_plant.get_tree().get_num_velocities(),
            num_iiwas * kNumIiwaVelocities + num_wsgs * kNumWsgVelocities +
                num_objects * kNumObjectVelocities);
}

void CheckNumPorts(
    const IiwaAndWsgPlantWithStateEstimator<double>& iiwa_and_wsg_plant,
    int num_iiwas, int num_wsgs, int num_objects) {
  // Check the number of input/output ports
  const int num_input_ports_per_iiwa = 2;
  const int num_input_ports_per_wsg = 1;
  EXPECT_EQ(iiwa_and_wsg_plant.get_num_input_ports(),
            num_iiwas * num_input_ports_per_iiwa +
                num_wsgs * num_input_ports_per_wsg);
  const int num_output_ports_per_iiwa = 2;
  const int num_output_ports_per_wsg = 1;
  const int num_output_ports_per_object = 1;
  // Every instance of IiwaAndWsgPlantWithStateEstimator has
  //  - a port for the full RigidBodyPlant state
  //  - a port for the contact results of the RigidBodyPlant
  //  - a port for the kinematics results of the RigidBodyPlant
  const int num_other_output_ports = 3;
  EXPECT_EQ(iiwa_and_wsg_plant.get_num_output_ports(),
            num_iiwas * num_output_ports_per_iiwa +
                num_wsgs * num_output_ports_per_wsg +
                num_objects * num_output_ports_per_object +
                num_other_output_ports);
}

// Check that calling the constructor results in a properly configured plant.
TEST_P(IiwaAndWsgPlantWithStateEstimatorParameterizedTest, Constructor) {
  int num_iiwas = std::get<0>(GetParam());
  int num_wsgs = std::get<0>(GetParam());
  int num_objects = std::get<1>(GetParam());
  BuildPlant(num_iiwas, num_wsgs, num_objects);
  auto iiwa_and_wsg_plant = MakeIiwaAndWsgPlantWithStateEstimator();
  CheckNumPositionsAndVelocities(*iiwa_and_wsg_plant, num_iiwas, num_wsgs,
                                 num_objects);
  CheckNumPorts(*iiwa_and_wsg_plant, num_iiwas, num_wsgs, num_objects);
}

INSTANTIATE_TEST_CASE_P(ValidCombinations,
                        IiwaAndWsgPlantWithStateEstimatorParameterizedTest,
                        ::testing::Combine(::testing::Values(1, 2, 3, 10),
                                           ::testing::Values(1, 2, 3, 10)));

// Check that calling the constructor with scalar inputs results in a properly
// configured plant.
TEST_F(IiwaAndWsgPlantWithStateEstimatorTest, ScalarConstructor) {
  BuildPlant(1, 1, 1);
  auto iiwa_and_wsg_plant =
      std::make_unique<IiwaAndWsgPlantWithStateEstimator<double>>(
          std::move(combined_plant_), iiwa_instances_.back(),
          wsg_instances_.back(), object_instances_.back());
  CheckNumPositionsAndVelocities(*iiwa_and_wsg_plant, 1, 1, 1);
  CheckNumPorts(*iiwa_and_wsg_plant, 1, 1, 1);
}

// Check that the constructor throws is the number of IIWAs does not match the
// number of WSGs.
TEST_F(IiwaAndWsgPlantWithStateEstimatorTest, IiwaAndWsgNumberMismatch) {
  BuildPlant(1, 2, 3);
  EXPECT_THROW(MakeIiwaAndWsgPlantWithStateEstimator(), std::logic_error);
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
