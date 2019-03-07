#include <cmath>
#include <map>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/manipulation/scene_generation/random_clutter_generator.h"
#include "drake/manipulation/scene_generation/simulate_plant_to_rest.h"
#include "drake/manipulation/util/world_sim_tree_builder.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace manipulation {
namespace scene_generation {
const Vector3<double> kClutterCenter(0.0, 0.0, 0.0);
const Vector3<double> kClutterSize(0.2, 0.4, 1.6);
const char kPath[] = "examples/kuka_iiwa_arm/models/objects/";
const char kContainer[] = "open_top_box.urdf";
const char kObject1[] = "big_robot_toy.urdf";
const char kObject2[] = "block_for_pick_and_place.urdf";

const int kNumRepetitions = 2;
const int kNumBodiesInClutter = 2 * kNumRepetitions;
const double kZHeightCost = 100.0;  // Chosen arbitrarily.

namespace {
// Verifies that the obtained clutter IK solution is a 'feasible' clutter, i.e.
// All the bodies are situated within the bounds specified by `kClutterSize`.
void VerifyClutterIk(const VectorX<double>& q, int num_elements_in_clutter) {
  Vector3<double> clutter_min = kClutterCenter - 0.5 * kClutterSize;
  Vector3<double> clutter_max = kClutterCenter + 0.5 * kClutterSize;

  const double kPositionTolerance = 1e-6;

  for (int i = 0; i < num_elements_in_clutter; ++i) {
    VectorX<double> pose = q.segment(7 * i, 7);
    for (int j = 0; j < 3; ++j) {
      EXPECT_GE(pose[j], clutter_min[j] - kPositionTolerance);
      EXPECT_LE(pose[j], clutter_max[j] + kPositionTolerance);
    }
    // Check for valid orientation
    VectorX<double> orientation = pose.tail(4);
    EXPECT_TRUE(std::abs(1 - orientation.norm()) < 1e-5);
  }
}
}  // namespace

class ClutterGeneratorTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    auto tree_builder = std::make_unique<util::WorldSimTreeBuilder<double>>();
    tree_builder->StoreModel("container",
                             std::string(kPath) + std::string(kContainer));
    tree_builder->StoreModel("object1",
                             std::string(kPath) + std::string(kObject1));
    tree_builder->StoreModel("object2",
                             std::string(kPath) + std::string(kObject2));

    tree_builder->AddFixedModelInstance("container",
                                        Eigen::Vector3d(0, 0, 0.01));
    std::set<int> clutter_instances;

    Eigen::Isometry3d origin = Eigen::Isometry3d::Identity();
    for (int i = 0; i < kNumRepetitions; ++i) {
      for (int j = 0; j < 2; ++j) {
        std::stringstream object_name;
        object_name << "object" << j + 1;
        int model_instance = tree_builder->AddFloatingModelInstance(
            object_name.str(), origin.translation());
        clutter_instances.insert(model_instance);
      }
    }

    auto scene_tree = tree_builder->Build();
    num_positions_ = scene_tree->get_num_positions();
    clutter_generator_ = std::make_unique<RandomClutterGenerator>(
        scene_tree.get(), clutter_instances, kClutterCenter, kClutterSize);

    auto scene_plant = std::make_unique<systems::RigidBodyPlant<double>>(
        std::move(scene_tree));

    plant_to_rest_ =
        std::make_unique<SimulatePlantToRest>(std::move(scene_plant));
  }

 protected:
  std::unique_ptr<RandomClutterGenerator> clutter_generator_;
  std::unique_ptr<SimulatePlantToRest> plant_to_rest_;
  int num_positions_;
};

const int kNumTrials = 2;

// This is a regression test. Verifies only valid clutter is generated.
TEST_F(ClutterGeneratorTest, TestClutterIkValidity) {
  // Uses an arbitrarily chosen seed.
  std::default_random_engine generator(42);

  // The expected number of positions corresponds to 7DoF for the configuration
  // of each of the bodies in the clutter i.e. each of the 2 types rigid bodies
  // that are repeated 4 times each within the clutter RigidBodyTree.
  EXPECT_EQ(num_positions_, 7 * kNumBodiesInClutter);
  VectorX<double> q_initial = VectorX<double>::Random(num_positions_);
  VectorX<double> q_ik, q_ik_previous = q_initial;

  for (int i = 0; i < kNumTrials; ++i) {
    q_ik = clutter_generator_->GenerateFloatingClutter(q_initial, &generator,
                                                       kZHeightCost);
    EXPECT_EQ(q_ik.size(), num_positions_);
    EXPECT_NO_THROW(VerifyClutterIk(q_ik, kNumBodiesInClutter));

    // Also verify that the resulting solution is not identical for a generous
    // tolerance.
    EXPECT_FALSE(CompareMatrices(q_ik, q_ik_previous, 1e-2));
    q_ik_previous = q_ik;
  }
}

// Verifies valid clutter is generated with or without a z_cost.
TEST_F(ClutterGeneratorTest, TestClutterIkCost) {
  EXPECT_EQ(num_positions_, 7 * kNumBodiesInClutter);
  // Uses an arbitrarily chosen seed.
  std::default_random_engine generator_1(42);
  const VectorX<double> q_initial = VectorX<double>::Random(num_positions_);
  VectorX<double> q_ik;
  std::vector<VectorX<double>> q_stored_ik_with_cost, q_stored_ik_no_cost;

  // This loop tests the repeatability.
  for (int i = 0; i < kNumTrials; ++i) {
    q_ik = clutter_generator_->GenerateFloatingClutter(q_initial, &generator_1,
                                                       kZHeightCost);
    EXPECT_EQ(q_ik.size(), num_positions_);
    EXPECT_NO_THROW(VerifyClutterIk(q_ik, kNumBodiesInClutter));

    q_stored_ik_with_cost.push_back(q_ik);
  }

  // Uses an arbitrarily chosen seed.
  std::default_random_engine generator_2(42);

  // This loop tests the repeatability.
  for (int i = 0; i < kNumTrials; ++i) {
    q_ik = clutter_generator_->GenerateFloatingClutter(q_initial, &generator_2);
    EXPECT_EQ(q_ik.size(), num_positions_);
    EXPECT_NO_THROW(VerifyClutterIk(q_ik, kNumBodiesInClutter));

    q_stored_ik_no_cost.push_back(q_ik);
  }

  // Verifies that adding or removing cost results in different but valid
  // clutter.
  for (int i = 0; i < kNumTrials; ++i) {
    EXPECT_FALSE(CompareMatrices(q_stored_ik_no_cost[i],
                                 q_stored_ik_with_cost[i], 1e-2));
  }
}

// This is a regression test. Verifies repeatability of SimulatePlantToRest.
TEST_F(ClutterGeneratorTest, TestPlantToRest) {
  std::default_random_engine generator(42);

  EXPECT_EQ(num_positions_, 7 * kNumBodiesInClutter);
  const VectorX<double> q_initial = VectorX<double>::Random(num_positions_);
  VectorX<double> q_ik, q_out, q_out_stored;
  q_ik = clutter_generator_->GenerateFloatingClutter(q_initial, &generator,
                                                     kZHeightCost);

  EXPECT_NO_THROW(q_out_stored = plant_to_rest_->Run(q_ik));

  // This loop tests the repeatability.
  for (int i = 0; i < kNumTrials; ++i) {
    EXPECT_EQ(q_ik.size(), num_positions_);
    EXPECT_NO_THROW(q_out = plant_to_rest_->Run(q_ik));

    EXPECT_TRUE(CompareMatrices(q_out_stored, q_out, 1e-3));
  }
}

}  // namespace scene_generation
}  // namespace manipulation
}  // namespace drake
