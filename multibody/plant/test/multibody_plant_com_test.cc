/* clang-format off to disable clang-format-includes */
#include "drake/multibody/plant/multibody_plant.h"
/* clang-format on */

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/scene_graph.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace {

class EmptyMultibodyPlantCenterOfMassTest : public ::testing::Test {
 public:
  void SetUp() override {
    plant_.Finalize();
    context_ = plant_.CreateDefaultContext();
  }

 protected:
  MultibodyPlant<double> plant_;
  std::unique_ptr<systems::Context<double>> context_;
};

TEST_F(EmptyMultibodyPlantCenterOfMassTest, GetCenterOfMassPosition) {
  Eigen::Vector3d p_WBcm;
  EXPECT_THROW(plant_.CalcCenterOfMassPosition(*context_, &p_WBcm),
               std::runtime_error);
}

class MultibodyPlantCenterOfMassTest : public ::testing::Test {
 public:
  void SetUp() override {
    cubic_instance_ = plant_.AddModelInstance("Cubics");
    sphere_instance_ = plant_.AddModelInstance("Spheres");

    mass_s1 = 10.0;
    p_SoScm_s1 = Eigen::Vector3d(1.5, 2.1, 3.9);
    plant_.AddRigidBody(
        "Sphere1", sphere_instance_,
        multibody::SpatialInertia<double>::MakeFromCentralInertia(
            mass_s1, p_SoScm_s1,
            // Set random inertia tensor, which does not affect the Center of
            // Mass position.
            multibody::RotationalInertia<double>(6.0, 6.0, 6.0)));

    mass_c1 = 20.0;
    p_CoCcm_c1 = Eigen::Vector3d(3.3, 2.5, 0.7);
    plant_.AddRigidBody(
        "Cubic1", cubic_instance_,
        multibody::SpatialInertia<double>::MakeFromCentralInertia(
            mass_c1, p_CoCcm_c1,
            // Set random inertia tensor, which does not affect the Center of
            // Mass position.
            multibody::RotationalInertia<double>(5.0, 14.0, 10)));

    plant_.Finalize();
    context_ = plant_.CreateDefaultContext();
  }

 protected:
  MultibodyPlant<double> plant_;
  std::unique_ptr<systems::Context<double>> context_;
  ModelInstanceIndex cubic_instance_;
  ModelInstanceIndex sphere_instance_;
  double mass_s1;
  double mass_c1;
  Eigen::Vector3d p_SoScm_s1;
  Eigen::Vector3d p_CoCcm_c1;
};

TEST_F(MultibodyPlantCenterOfMassTest, CenterOfMassPosition) {
  Eigen::Vector3d p_WBcm;
  //  plant_.CalcCenterOfMassPosition(*context_, &p_WBcm);
  Eigen::Vector3d result =
      (p_SoScm_s1 * mass_s1 + p_CoCcm_c1 * mass_c1) / (mass_s1 + mass_c1);
  //  EXPECT_TRUE(CompareMatrices(p_WBcm, result, 1e-6));

  drake::VectorX<double> sphere_positions =
      plant_.GetPositions(*context_, sphere_instance_);
  Eigen::Vector3d p_WSo_W(1.1, 2.3, 3.7);
  sphere_positions.segment<3>(4) = p_WSo_W;
  plant_.SetPositions(context_.get(), sphere_instance_, sphere_positions);

  drake::VectorX<double> cubic_positions =
      plant_.GetPositions(*context_, cubic_instance_);
  Eigen::Vector3d p_WCo_W(-5.2, 10.4, -6.8);
  cubic_positions.segment<3>(4) = p_WCo_W;
  plant_.SetPositions(context_.get(), cubic_instance_, cubic_positions);

  result =
      ((p_WSo_W + p_SoScm_s1) * mass_s1 + (p_WCo_W + p_CoCcm_c1) * mass_c1) /
      (mass_s1 + mass_c1);
  plant_.CalcCenterOfMassPosition(*context_, &p_WBcm);
  EXPECT_TRUE(CompareMatrices(p_WBcm, result, 1e-12));

  // Try empty model_instances.
  std::unordered_set<ModelInstanceIndex> model_instances;
  EXPECT_THROW(
      plant_.CalcCenterOfMassPosition(*context_, model_instances, &p_WBcm),
      std::runtime_error);

  // Try one instance in model_instances.
  model_instances.insert(cubic_instance_);
  plant_.CalcCenterOfMassPosition(*context_, model_instances, &p_WBcm);
  EXPECT_TRUE(CompareMatrices(p_WBcm, p_WCo_W + p_CoCcm_c1, 1e-12));

  // Try all instance in model_instances.
  model_instances.insert(sphere_instance_);
  plant_.CalcCenterOfMassPosition(*context_, model_instances, &p_WBcm);
  EXPECT_TRUE(CompareMatrices(p_WBcm, result, 1e-12));

  // Try error instance in model_instances.
  ModelInstanceIndex error_index(10);
  model_instances.insert(error_index);
  EXPECT_THROW(
      plant_.CalcCenterOfMassPosition(*context_, model_instances, &p_WBcm),
      std::runtime_error);

  // Try after translation and rotation.
  Eigen::Quaterniond quat_s1(1, 2, 3, 4);
  quat_s1.normalize();
  math::RotationMatrixd rot_s1(quat_s1);
  sphere_positions[0] = quat_s1.w();
  sphere_positions.segment(1, 3) = quat_s1.vec();  // Set x, y, z.
  plant_.SetPositions(context_.get(), sphere_instance_, sphere_positions);

  Eigen::Quaterniond quat_c1(-5, 6, -7, 8);
  quat_c1.normalize();
  math::RotationMatrixd rot_c1(quat_c1);
  cubic_positions[0] = quat_c1.w();
  cubic_positions.segment(1, 3) = quat_c1.vec();  // Set x, y, z.
  plant_.SetPositions(context_.get(), cubic_instance_, cubic_positions);

  Eigen::Vector3d rotation_result =
      ((p_WSo_W + rot_s1 * p_SoScm_s1) * mass_s1 +
       (p_WCo_W + rot_c1 * p_CoCcm_c1) * mass_c1) /
      (mass_s1 + mass_c1);
  plant_.CalcCenterOfMassPosition(*context_, &p_WBcm);
  EXPECT_TRUE(CompareMatrices(p_WBcm, rotation_result, 1e-12));
}

}  // namespace
}  // namespace multibody
}  // namespace drake
