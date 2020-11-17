/* clang-format off to disable clang-format-includes */
#include "drake/multibody/plant/multibody_plant.h"
/* clang-format on */

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace {

GTEST_TEST(EmptyMultibodyPlantCenterOfMassTest, GetCenterOfMassPosition) {
  MultibodyPlant<double> plant(0.0);
  plant.Finalize();
  auto context_ = plant.CreateDefaultContext();
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.CalcCenterOfMassPosition(*context_), std::runtime_error,
      "CalcCenterOfMassPosition\\(\\): this MultibodyPlant contains only "
      "world_body\\(\\) so its center of mass is undefined.");
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

  void CheckCom(const math::RigidTransform<double>& X_WS,
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
    Eigen::Vector3d p_WCcm = plant_.CalcCenterOfMassPosition(*context_);
    EXPECT_TRUE(CompareMatrices(p_WCcm, p_WCcm_expected, 1e-15));
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

TEST_F(MultibodyPlantCenterOfMassTest, CenterOfMassPosition) {
  // Verify the plant's default center of mass position makes sense.
  Eigen::Vector3d p_WCcm = plant_.CalcCenterOfMassPosition(*context_);
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
  CheckCom(X_WS1, X_WT1);

  // Ensure exception is thrown if model_instances is empty.
  std::vector<ModelInstanceIndex> model_instances;
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_.CalcCenterOfMassPosition(*context_, model_instances),
      std::runtime_error,
      "CalcCenterOfMassPosition\\(\\): you must provide at least one selected "
      "body.");

  // Verify CalcCenterOfMassPosition() works for 1 instance in model_instances.
  model_instances.push_back(triangle_instance_);
  p_WCcm = plant_.CalcCenterOfMassPosition(*context_, model_instances);
  EXPECT_TRUE(CompareMatrices(p_WCcm, p_WTo_W + p_TTcm_T_, kTolerance));

  // Verify CalcCenterOfMassPosition() works for 2 instances in model_instances.
  model_instances.push_back(sphere_instance_);
  result = ((p_WSo_W + p_SScm_S_) * mass_S_ + (p_WTo_W + p_TTcm_T_) * mass_T_) /
           (mass_S_ + mass_T_);
  p_WCcm = plant_.CalcCenterOfMassPosition(*context_, model_instances);
  EXPECT_TRUE(CompareMatrices(p_WCcm, result, 1e-15));

  // Verify CalcCenterOfMassPosition() works for 2 instances in model_instances,
  // where the 2 objects have arbitrary orientation and position.
  const math::RigidTransformd X_WS2(math::RollPitchYawd(0.3, -1.5, 0.7),
                                    Eigen::Vector3d(5.2, -3.1, 10.9));
  const math::RigidTransformd X_WT2(math::RollPitchYawd(-2.3, -3.5, 1.2),
                                    Eigen::Vector3d(-70.2, 9.8, 843.1));
  CheckCom(X_WS2, X_WT2);

  // Ensure CalcCenterOfMassPosition() throws an exception if total mass <= 0.
  set_mass_sphere(0.0);
  set_mass_triangle(0.0);
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_.CalcCenterOfMassPosition(*context_, model_instances),
      std::runtime_error,
      "CalcCenterOfMassPosition\\(\\): the "
      "system's total mass must be greater than zero.");

  // Ensure CalcJacobianCenterOfMassTranslationalVelocity() throws an exception
  // if total mass <= 0.
  Eigen::MatrixXd Js_v_WCcm_W(3, plant_.num_velocities());
  const Frame<double>& frame_W = plant_.world_frame();
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_.CalcJacobianCenterOfMassTranslationalVelocity(
      *context_, JacobianWrtVariable::kV, frame_W, frame_W, &Js_v_WCcm_W),
      std::runtime_error,
      "CalcJacobianCenterOfMassTranslationalVelocity\\(\\): the "
      "system's total mass must be greater than zero.");

  // Ensure CalcBiasCenterOfMassTranslationalAcceleration() throws an exception
  // if total mass <= 0.
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_.CalcBiasCenterOfMassTranslationalAcceleration(
      *context_, JacobianWrtVariable::kV, frame_W, frame_W),
      std::runtime_error,
      "CalcBiasCenterOfMassTranslationalAcceleration\\(\\): the "
      "system's total mass must be greater than zero.");

  // Ensure an exception is thrown if there is an invalid ModelInstanceIndex.
  ModelInstanceIndex error_index(10);
  model_instances.push_back(error_index);
  EXPECT_THROW(plant_.CalcCenterOfMassPosition(*context_, model_instances),
               std::runtime_error);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
