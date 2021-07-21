/* clang-format off to disable clang-format-includes */
#include "drake/multibody/plant/multibody_plant.h"
/* clang-format on */

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace {

GTEST_TEST(EmptyMultibodyPlantCenterOfMassTest, CalcCenterOfMassPosition) {
  MultibodyPlant<double> plant(0.0);
  plant.Finalize();
  auto context_ = plant.CreateDefaultContext();
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.CalcCenterOfMassPositionInWorld(*context_),
      "CalcCenterOfMassPositionInWorld\\(\\): This MultibodyPlant "
      "only contains the world_body\\(\\) so its center of mass is undefined.");

  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.CalcCenterOfMassTranslationalVelocityInWorld(*context_),
      "CalcCenterOfMassTranslationalVelocityInWorld\\(\\): This MultibodyPlant "
      "only contains the world_body\\(\\) so its center of mass is undefined.");
  }

class MultibodyPlantCenterOfMassTest : public ::testing::Test {
 public:
  void SetUp() override {
    triangle_instance_ = plant_.AddModelInstance("Triangles");
    sphere_instance_ = plant_.AddModelInstance("Spheres");

    mass_S_ = 10.0;
    p_SScm_S_ = Eigen::Vector3d(1.5, 2.1, 3.9);
    plant_.AddRigidBody(
        "Sphere1", sphere_instance_,
        multibody::SpatialInertia<double>::MakeFromCentralInertia(
            mass_S_, p_SScm_S_,
            // Set random inertia tensor, which does not affect the Center of
            // Mass position.
            multibody::RotationalInertia<double>(6.0, 6.0, 6.0)));

    mass_T_ = 20.0;
    p_TTcm_T_ = Eigen::Vector3d(3.3, 2.5, 0.7);
    plant_.AddRigidBody(
        "Triangle1", triangle_instance_,
        multibody::SpatialInertia<double>::MakeFromCentralInertia(
            mass_T_, p_TTcm_T_,
            // Set random inertia tensor, which does not affect the Center of
            // Mass position.
            multibody::RotationalInertia<double>(5.0, 14.0, 10.0)));

    plant_.Finalize();
    context_ = plant_.CreateDefaultContext();
  }

  void CheckCmPosition(const math::RigidTransform<double>& X_WS,
                       const math::RigidTransform<double>& X_WT) {
    plant_.SetFreeBodyPose(context_.get(), GetSphereBody(), X_WS);
    plant_.SetFreeBodyPose(context_.get(), GetTriangleBody(), X_WT);

    const math::RotationMatrixd& R_WS = X_WS.rotation();
    const math::RotationMatrixd& R_WT = X_WT.rotation();
    const Eigen::Vector3d& p_WSo_W = X_WS.translation();
    const Eigen::Vector3d& p_WTo_W = X_WT.translation();
    const Eigen::Vector3d p_WCcm_expected =
        ((p_WSo_W + R_WS * p_SScm_S_) * mass_S_ +
         (p_WTo_W + R_WT * p_TTcm_T_) * mass_T_) /
        (mass_S_ + mass_T_);
    Eigen::Vector3d p_WCcm = plant_.CalcCenterOfMassPositionInWorld(*context_);
    // Allow for 3 bits (2^3 = 8) of error.
    const double kTolerance = 8 * std::numeric_limits<double>::epsilon();
    EXPECT_TRUE(CompareMatrices(p_WCcm, p_WCcm_expected, kTolerance));
  }

  const RigidBody<double>& GetSphereBody() {
    return plant_.GetRigidBodyByName("Sphere1");
  }

  const RigidBody<double>& GetTriangleBody() {
    return plant_.GetRigidBodyByName("Triangle1");
  }

  void set_mass_triangle(double m) {
    mass_T_ = m;
    systems::Context<double>* context_ptr = context_.get();
    GetTriangleBody().SetMass(context_ptr, m);
  }

  void set_mass_sphere(double m) {
    mass_S_ = m;
    systems::Context<double>* context_ptr = context_.get();
    GetSphereBody().SetMass(context_ptr, m);
  }

    void CheckCmTranslationalVelocity(const SpatialVelocity<double>& V_WS_W,
                                      const SpatialVelocity<double>& V_WT_W) {
    const Body<double>& sphere = plant_.GetBodyByName("Sphere1");
    const Body<double>& triangle = plant_.GetBodyByName("Triangle1");
    plant_.SetFreeBodySpatialVelocity(context_.get(), sphere, V_WS_W);
    plant_.SetFreeBodySpatialVelocity(context_.get(), triangle, V_WT_W);

    // Denoting Scm as the center of mass of the system formed by Sphere1 and
    // Triangle1, form Scm's translational velocity in frame W, expressed in W.
    const Vector3<double> v_WScm_W =
        plant_.CalcCenterOfMassTranslationalVelocityInWorld(*context_);

    // By hand, calculate the expected result for that same quantity (v_WScm_W).
    const double mass_sphere = sphere.get_mass(*context_);
    const double mass_triangle = triangle.get_mass(*context_);
    const Vector3<double> mv_sphere = mass_sphere *
        sphere.CalcCenterOfMassTranslationalVelocityInWorld(*context_);
    const Vector3<double> mv_triangle =  mass_triangle *
        triangle.CalcCenterOfMassTranslationalVelocityInWorld(*context_);
    const Vector3<double> v_WScm_W_expected = (mv_sphere + mv_triangle) /
        (mass_sphere + mass_triangle);

    const double kTolerance = 16 * std::numeric_limits<double>::epsilon();
    EXPECT_TRUE(CompareMatrices(v_WScm_W, v_WScm_W_expected, kTolerance));
  }

 protected:
  MultibodyPlant<double> plant_{0.0};
  std::unique_ptr<systems::Context<double>> context_;
  ModelInstanceIndex triangle_instance_;
  ModelInstanceIndex sphere_instance_;
  double mass_S_;
  double mass_T_;
  Eigen::Vector3d p_SScm_S_;
  Eigen::Vector3d p_TTcm_T_;
};

TEST_F(MultibodyPlantCenterOfMassTest, CalcTotalMass) {
  // Verify the plant's total mass makes sense.
  const double mass_system = plant_.CalcTotalMass(*context_);
  EXPECT_EQ(mass_system, mass_S_ + mass_T_);

  // Verify CalcTotalMass() returns 0 if model instances only has 1 world body.
  const ModelInstanceIndex world_model_instance =
      multibody::world_model_instance();
  std::vector<ModelInstanceIndex> world_model_instance_array;
  world_model_instance_array.push_back(world_model_instance);
  double mass_zero =
      plant_.CalcTotalMass(*context_, world_model_instance_array);
  EXPECT_EQ(mass_zero, 0.0);

  // Check CalcTotalMass() returns 0 if model instances has only 2 world bodies.
  world_model_instance_array.push_back(world_model_instance);
  mass_zero = plant_.CalcTotalMass(*context_, world_model_instance_array);
  EXPECT_EQ(mass_zero, 0.0);

  // Verify CalcTotalMass() returns 0 for empty model_instances.
  std::vector<ModelInstanceIndex> model_instances;
  mass_zero = plant_.CalcTotalMass(*context_, model_instances);
  EXPECT_EQ(mass_zero, 0.0);

  // Verify CalcTotalMass() works for 1 instances in model_instances.
  model_instances.push_back(triangle_instance_);
  const double mass_triangle = plant_.CalcTotalMass(*context_, model_instances);
  EXPECT_EQ(mass_triangle, mass_T_);

  // Verify CalcTotalMass() works for 2 instances in model_instances.
  model_instances.push_back(sphere_instance_);
  double mass_2_instances = plant_.CalcTotalMass(*context_, model_instances);
  EXPECT_EQ(mass_2_instances, mass_S_ + mass_T_);

  // Verify CalcTotalMass() works for 3 instances (with 2 sphere_instance_).
  // Note: CalcTotalMass() only calculates a single sphere_instance_ !
  model_instances.push_back(sphere_instance_);
  double mass_odd_instances = plant_.CalcTotalMass(*context_, model_instances);
  EXPECT_EQ(mass_odd_instances, mass_S_ + mass_T_);  // mass_S not 2 * mass_S!

  // Verify we are able to determine if the total mass = 0.
  set_mass_sphere(0.0);
  set_mass_triangle(0.0);
  mass_zero = plant_.CalcTotalMass(*context_);
  EXPECT_EQ(mass_zero, 0.0);
  mass_zero = plant_.CalcTotalMass(*context_, model_instances);
  EXPECT_EQ(mass_zero, 0.0);

  // Verify no exception is thrown if there is an invalid ModelInstanceIndex.
  ModelInstanceIndex error_index(10);
  model_instances.push_back(error_index);
  DRAKE_EXPECT_NO_THROW(plant_.CalcTotalMass(*context_, model_instances));
}

TEST_F(MultibodyPlantCenterOfMassTest, CenterOfMassPosition) {
  // Verify the plant's default center of mass position makes sense.
  Eigen::Vector3d p_WCcm = plant_.CalcCenterOfMassPositionInWorld(*context_);
  const math::RigidTransformd X_WS0 = math::RigidTransformd::Identity();
  const math::RigidTransformd X_WT0 = math::RigidTransformd::Identity();
  Eigen::Vector3d result =
      (X_WS0 * p_SScm_S_ * mass_S_ + X_WT0 * p_TTcm_T_ * mass_T_) /
      (mass_S_ + mass_T_);
  // Allow for 3 bits (2^3 = 8) of error.
  const double kTolerance = 8 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(p_WCcm, result, kTolerance));

  // Verify the plant's center of mass location for an arbitrary input.
  const Eigen::Vector3d p_WSo_W(1.1, 2.3, 3.7);
  const Eigen::Vector3d p_WTo_W(-5.2, 10.4, -6.8);
  const math::RigidTransformd X_WS1(p_WSo_W);  // Rotation matrix is identity.
  const math::RigidTransformd X_WT1(p_WTo_W);  // Rotation matrix is identity.
  CheckCmPosition(X_WS1, X_WT1);

  // Verify center of mass translational velocity when there is no motion.
  SpatialVelocity<double> V1 = SpatialVelocity<double>::Zero();
  SpatialVelocity<double> V2 = SpatialVelocity<double>::Zero();
  CheckCmTranslationalVelocity(V1, V2);

  // Verify center of mass translational velocity for arbitrary motion.
  V1 = SpatialVelocity<double>(Vector3<double>(1, 2, 3),
                               Vector3<double>(4, 5, 6));
  V2 = SpatialVelocity<double>(Vector3<double>(3, 5, 7),
                               Vector3<double>(5, 3, 1));
  CheckCmTranslationalVelocity(V1, V2);

  // Ensure center of mass methods throw an exception for empty model_instances.
  std::vector<ModelInstanceIndex> model_instances;
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_.CalcCenterOfMassPositionInWorld(*context_, model_instances),
      "CalcCenterOfMassPositionInWorld\\(\\): There must be at "
      "least one non-world body contained in model_instances.");

  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_.CalcCenterOfMassTranslationalVelocityInWorld(*context_,
                                                          model_instances),
      "CalcCenterOfMassTranslationalVelocityInWorld\\(\\): There must be at "
      "least one non-world body contained in model_instances.");

  Eigen::MatrixXd Js_v_WCcm_W(3, plant_.num_velocities());
  const Frame<double>& frame_W = plant_.world_frame();
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_.CalcJacobianCenterOfMassTranslationalVelocity(
          *context_, model_instances, JacobianWrtVariable::kV,
          frame_W, frame_W, &Js_v_WCcm_W),
      "CalcJacobianCenterOfMassTranslationalVelocity\\(\\): There must be at "
      "least one non-world body contained in model_instances.");

  // Ensure an exception is thrown when a model instance has one world body.
  const ModelInstanceIndex world_model_instance =
      multibody::world_model_instance();
  std::vector<ModelInstanceIndex> world_model_instance_array;
  world_model_instance_array.push_back(world_model_instance);
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_.CalcCenterOfMassTranslationalVelocityInWorld(*context_,
          world_model_instance_array),
      "CalcCenterOfMassTranslationalVelocityInWorld\\(\\): There must be at "
      "least one non-world body contained in model_instances.");

  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_.CalcJacobianCenterOfMassTranslationalVelocity(
          *context_, model_instances, JacobianWrtVariable::kV,
          frame_W, frame_W, &Js_v_WCcm_W),
      "CalcJacobianCenterOfMassTranslationalVelocity\\(\\): There must be at "
      "least one non-world body contained in model_instances.");

  // Ensure an exception is thrown when a model instance has two world bodies.
  world_model_instance_array.push_back(world_model_instance);
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_.CalcCenterOfMassTranslationalVelocityInWorld(*context_,
          world_model_instance_array),
      "CalcCenterOfMassTranslationalVelocityInWorld\\(\\): There must be at "
      "least one non-world body contained in model_instances.");

  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_.CalcJacobianCenterOfMassTranslationalVelocity(
          *context_, model_instances, JacobianWrtVariable::kV,
          frame_W, frame_W, &Js_v_WCcm_W),
      "CalcJacobianCenterOfMassTranslationalVelocity\\(\\): There must be at "
      "least one non-world body contained in model_instances.");

  // Verify CalcCenterOfMassPositionInWorld() works for 1 instance in
  // model_instances.
  model_instances.push_back(triangle_instance_);
  p_WCcm = plant_.CalcCenterOfMassPositionInWorld(*context_, model_instances);
  EXPECT_TRUE(CompareMatrices(p_WCcm, p_WTo_W + p_TTcm_T_, kTolerance));

  // Verify CalcCenterOfMassPositionInWorld() works for 2 instances
  // in model_instances.
  model_instances.push_back(sphere_instance_);
  result = ((p_WSo_W + p_SScm_S_) * mass_S_ + (p_WTo_W + p_TTcm_T_) * mass_T_) /
           (mass_S_ + mass_T_);
  p_WCcm = plant_.CalcCenterOfMassPositionInWorld(*context_, model_instances);
  EXPECT_TRUE(CompareMatrices(p_WCcm, result, kTolerance));

  // Verify CalcCenterOfMassPositionInWorld() works for 2 objects in
  // model_instances, where the 2 objects have arbitrary orientation/position.
  const math::RigidTransformd X_WS2(math::RollPitchYawd(0.3, -1.5, 0.7),
                                    Eigen::Vector3d(5.2, -3.1, 10.9));
  const math::RigidTransformd X_WT2(math::RollPitchYawd(-2.3, -3.5, 1.2),
                                    Eigen::Vector3d(-70.2, 9.8, 843.1));
  CheckCmPosition(X_WS2, X_WT2);

  // Ensure center of mass methods throw an exception if total mass ≤ 0.
  set_mass_sphere(0.0);
  set_mass_triangle(0.0);
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_.CalcCenterOfMassPositionInWorld(*context_, model_instances),
      "CalcCenterOfMassPositionInWorld\\(\\): The "
      "system's total mass must be greater than zero.");

  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_.CalcCenterOfMassTranslationalVelocityInWorld(*context_,
                                                          model_instances),
      "CalcCenterOfMassTranslationalVelocityInWorld\\(\\): The "
      "system's total mass must be greater than zero.");

  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_.CalcJacobianCenterOfMassTranslationalVelocity(
          *context_, model_instances, JacobianWrtVariable::kV,
          frame_W, frame_W, &Js_v_WCcm_W),
      "CalcJacobianCenterOfMassTranslationalVelocity\\(\\): The "
      "system's total mass must be greater than zero.");

  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_.CalcJacobianCenterOfMassTranslationalVelocity(
      *context_, JacobianWrtVariable::kV, frame_W, frame_W, &Js_v_WCcm_W),
      "CalcJacobianCenterOfMassTranslationalVelocity\\(\\): The "
      "system's total mass must be greater than zero.");

  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_.CalcBiasCenterOfMassTranslationalAcceleration(
      *context_, JacobianWrtVariable::kV, frame_W, frame_W),
      "CalcBiasCenterOfMassTranslationalAcceleration\\(\\): The "
      "system's total mass must be greater than zero.");

  // Ensure an exception is thrown if there is an invalid ModelInstanceIndex.
  ModelInstanceIndex error_index(10);
  model_instances.push_back(error_index);
  EXPECT_THROW(plant_.CalcCenterOfMassPositionInWorld(
      *context_, model_instances),
          std::exception);
  EXPECT_THROW(plant_.CalcCenterOfMassTranslationalVelocityInWorld(
                   *context_, model_instances),
                       std::exception);
  EXPECT_THROW(plant_.CalcJacobianCenterOfMassTranslationalVelocity(
          *context_, model_instances, JacobianWrtVariable::kV,
          frame_W, frame_W, &Js_v_WCcm_W),
              std::exception);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
