#include "planning/voxelized_environment_builder.h"

#include <map>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <voxelized_geometry_tools/collision_map.hpp>
#include <voxelized_geometry_tools/tagged_object_collision_map.hpp>

#include "drake/common/text_logging.h"
#include "drake/geometry/scene_graph.h"
#include "planning/test/planning_test_helpers.h"

namespace anzu {
namespace planning {
namespace {

using voxelized_geometry_tools::SignedDistanceFieldGenerationParameters;

GTEST_TEST(VoxelizedEnvironmentBuilderTest, Test) {
  // Assemble model directives.
  drake::multibody::parsing::ModelDirective add_model_1;
  add_model_1.add_model = drake::multibody::parsing::AddModel{
      "package://anzu/models/test/voxel_test1.sdf", "voxel_test1"};
  drake::multibody::parsing::ModelDirective add_weld_1;
  add_weld_1.add_weld = drake::multibody::parsing::AddWeld{
      "world", "voxel_test1::voxel_test1"};
  drake::multibody::parsing::ModelDirective add_model_2;
  add_model_2.add_model = drake::multibody::parsing::AddModel{
      "package://anzu/models/test/voxel_test2.sdf", "voxel_test2"};
  drake::multibody::parsing::ModelDirective add_weld_2;
  add_weld_2.add_weld = drake::multibody::parsing::AddWeld{
      "world", "voxel_test2::voxel_test2"};

  const drake::multibody::parsing::ModelDirectives directives{.directives = {
      add_model_1, add_weld_1, add_model_2, add_weld_2}};

  auto model = MakePlanningTestModel(directives);
  auto context = model->CreateDefaultContext();
  // Get the plant context
  auto& plant_context = model->mutable_plant_context(context.get());
  // Origin of grid at world origin
  const Eigen::Isometry3d X_WG = Eigen::Isometry3d::Identity();
  const Eigen::Quaterniond R_WG(X_WG.rotation());
  // Grid 1m in each axis
  const Eigen::Vector3d grid_size(1.0, 1.0, 1.0);
  // 1/8 meter resolution, so 8 cells/axis
  const double grid_resolution = 0.125;
  // Frame name
  const std::string world_frame_name = "world";
  // Build both types of grids
  const voxelized_geometry_tools::CollisionMap cmap =
      BuildCollisionMap(
          model->plant(), plant_context,
          std::unordered_set<drake::geometry::GeometryId>(),
          world_frame_name, X_WG, grid_size, grid_resolution);
  ASSERT_TRUE(cmap.IsInitialized());
  ASSERT_EQ(cmap.GetNumXCells(), 8);
  ASSERT_EQ(cmap.GetNumYCells(), 8);
  ASSERT_EQ(cmap.GetNumZCells(), 8);
  const voxelized_geometry_tools::TaggedObjectCollisionMap tocmap =
      BuildTaggedObjectCollisionMap(
          model->plant(), plant_context,
          std::unordered_set<drake::geometry::GeometryId>(),
          world_frame_name, X_WG, grid_size, grid_resolution);
  ASSERT_TRUE(tocmap.IsInitialized());
  ASSERT_EQ(tocmap.GetNumXCells(), 8);
  ASSERT_EQ(tocmap.GetNumYCells(), 8);
  ASSERT_EQ(tocmap.GetNumZCells(), 8);
  // Export to signed distance field, using default options for SDF generation.
  const SignedDistanceFieldGenerationParameters<float> sdf_gen_parameters;
  const auto sdf = cmap.ExtractSignedDistanceFieldFloat(sdf_gen_parameters);
  ASSERT_TRUE(sdf.IsInitialized());
  ASSERT_EQ(sdf.GetNumXCells(), 8);
  ASSERT_EQ(sdf.GetNumYCells(), 8);
  ASSERT_EQ(sdf.GetNumZCells(), 8);
  // Make sure the grids are properly filled
  for (int64_t xidx = 0; xidx < cmap.GetNumXCells(); xidx++) {
    for (int64_t yidx = 0; yidx < cmap.GetNumYCells(); yidx++) {
      for (int64_t zidx = 0; zidx < cmap.GetNumZCells(); zidx++) {
        // Check grid querying
        const auto cmap_query = cmap.GetIndexImmutable(xidx, yidx, zidx);
        ASSERT_TRUE(cmap_query);
        const auto tocmap_query = tocmap.GetIndexImmutable(xidx, yidx, zidx);
        ASSERT_TRUE(tocmap_query);
        const auto sdf_query = sdf.GetIndexImmutable(xidx, yidx, zidx);
        ASSERT_TRUE(sdf_query);
        // Check grid values
        const float cmap_occupancy = cmap_query.Value().Occupancy();
        const float tocmap_occupancy = tocmap_query.Value().Occupancy();
        const float sdf_distance = sdf_query.Value();
        ASSERT_EQ(cmap_occupancy, tocmap_occupancy);
        const uint32_t tocmap_object_id = tocmap_query.Value().ObjectId();
        drake::log()->info(
            "Checking ({},{},{}) (x,y,z) with occupancy {} object_id {}",
            xidx, yidx, zidx, tocmap_occupancy, tocmap_object_id);
        // The environment is split evenly into 8 blocks
        if ((xidx < 4) && (yidx < 4)) {
          if (zidx < 4) {
            ASSERT_EQ(cmap_occupancy, 1.0f);
            ASSERT_LT(sdf_distance, 0.0f);
          } else {
            ASSERT_EQ(cmap_occupancy, 0.0f);
            ASSERT_GT(sdf_distance, 0.0f);
          }
        } else if (xidx < 4) {
          if (zidx < 4) {
            ASSERT_EQ(cmap_occupancy, 0.0f);
            ASSERT_GT(sdf_distance, 0.0f);
          } else {
            ASSERT_EQ(cmap_occupancy, 1.0f);
            ASSERT_LT(sdf_distance, 0.0f);
          }
        } else if (yidx < 4) {
          if (zidx < 4) {
            ASSERT_EQ(cmap_occupancy, 0.0f);
            ASSERT_GT(sdf_distance, 0.0f);
          } else {
            ASSERT_EQ(cmap_occupancy, 1.0f);
            ASSERT_LT(sdf_distance, 0.0f);
          }
        } else {
          if (zidx < 4) {
            ASSERT_EQ(cmap_occupancy, 1.0f);
            ASSERT_LT(sdf_distance, 0.0f);
          } else {
            ASSERT_EQ(cmap_occupancy, 0.0f);
            ASSERT_GT(sdf_distance, 0.0f);
          }
        }
        if (tocmap_occupancy > 0.5f) {
          // Check for a logical object_id in filled cells
          ASSERT_GT(tocmap_object_id, 0u);
        } else {
          // Check for no object_id in free cells
          ASSERT_EQ(tocmap_object_id, 0u);
        }
      }
    }
  }
}
}  // namespace
}  // namespace planning
}  // namespace anzu
