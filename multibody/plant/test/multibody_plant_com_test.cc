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
  EXPECT_THROW(plant_.CalcCenterOfMassPosition(*context_), std::runtime_error);
}

class MultibodyPlantCenterOfMassTest : public ::testing::Test {
 public:
  void SetUp() override {
    cubic_instance_ = plant_.AddModelInstance("Cubics");
    sphere_instance_ = plant_.AddModelInstance("Spheres");

    mass_So_ = 10.0;
    p_SoScm_So_ = Eigen::Vector3d(1.5, 2.1, 3.9);
    plant_.AddRigidBody(
        "Sphere1", sphere_instance_,
        multibody::SpatialInertia<double>::MakeFromCentralInertia(
            mass_So_, p_SoScm_So_,
            // Set random inertia tensor, which does not affect the Center of
            // Mass position.
            multibody::RotationalInertia<double>(6.0, 6.0, 6.0)));

    mass_Co_ = 20.0;
    p_CoCcm_Co_ = Eigen::Vector3d(3.3, 2.5, 0.7);
    plant_.AddRigidBody(
        "Cubic1", cubic_instance_,
        multibody::SpatialInertia<double>::MakeFromCentralInertia(
            mass_Co_, p_CoCcm_Co_,
            // Set random inertia tensor, which does not affect the Center of
            // Mass position.
            multibody::RotationalInertia<double>(5.0, 14.0, 10)));

    plant_.Finalize();
    context_ = plant_.CreateDefaultContext();
  }

  void CheckCOM(const math::RigidTransform<double>& X_WSo,
                const math::RigidTransform<double>& X_WCo) {
    Eigen::Matrix<double, 7, 1> q_sphere, q_cubic;
    const math::RotationMatrixd& R_WSo = X_WSo.rotation();
    const math::RotationMatrixd& R_WCo = X_WCo.rotation();
    const Eigen::Vector3d& p_WSo_W = X_WSo.translation();
    const Eigen::Vector3d& p_WCo_W = X_WCo.translation();
    q_sphere.segment<4>(0) = R_WSo.ToQuaternionAsVector4();
    q_sphere.segment<3>(4) = p_WSo_W;
    q_cubic.segment<4>(0) = R_WCo.ToQuaternionAsVector4();
    q_cubic.segment<3>(4) = p_WCo_W;
    plant_.SetPositions(context_.get(), sphere_instance_, q_sphere);
    plant_.SetPositions(context_.get(), cubic_instance_, q_cubic);

    const Eigen::Vector3d p_WCcm_expected =
        ((p_WSo_W + R_WSo * p_SoScm_So_) * mass_So_ +
         (p_WCo_W + R_WCo * p_CoCcm_Co_) * mass_Co_) /
        (mass_So_ + mass_Co_);
    Eigen::Vector3d p_WCcm = plant_.CalcCenterOfMassPosition(*context_);
    EXPECT_TRUE(CompareMatrices(p_WCcm, p_WCcm_expected, 1e-12));
  }

 protected:
  MultibodyPlant<double> plant_;
  std::unique_ptr<systems::Context<double>> context_;
  ModelInstanceIndex cubic_instance_;
  ModelInstanceIndex sphere_instance_;
  double mass_So_;
  double mass_Co_;
  Eigen::Vector3d p_SoScm_So_;
  Eigen::Vector3d p_CoCcm_Co_;
};

TEST_F(MultibodyPlantCenterOfMassTest, CenterOfMassPosition) {
  Eigen::Vector3d p_WCcm = plant_.CalcCenterOfMassPosition(*context_);
  Eigen::Vector3d result =
      (p_SoScm_So_ * mass_So_ + p_CoCcm_Co_ * mass_Co_) / (mass_So_ + mass_Co_);
  EXPECT_TRUE(CompareMatrices(p_WCcm, result, 1e-12));

  // Try compute Center of Mass at random translation.
  Eigen::Vector3d p_WSo_W(1.1, 2.3, 3.7);
  Eigen::Vector3d p_WCo_W(-5.2, 10.4, -6.8);
  math::RigidTransformd X_WSo1(
      math::RotationMatrixd(Eigen::MatrixXd::Identity(3, 3)), p_WSo_W);
  math::RigidTransformd X_WCo1(
      math::RotationMatrixd(Eigen::MatrixXd::Identity(3, 3)), p_WCo_W);
  CheckCOM(X_WSo1, X_WCo1);

  // Try empty model_instances.
  std::unordered_set<ModelInstanceIndex> model_instances;
  EXPECT_THROW(plant_.CalcCenterOfMassPosition(*context_, model_instances),
               std::runtime_error);

  // Try one instance in model_instances.
  model_instances.insert(cubic_instance_);
  p_WCcm = plant_.CalcCenterOfMassPosition(*context_, model_instances);
  EXPECT_TRUE(CompareMatrices(p_WCcm, p_WCo_W + p_CoCcm_Co_, 1e-12));

  // Try all instances in model_instances.
  model_instances.insert(sphere_instance_);
  result = ((p_WSo_W + p_SoScm_So_) * mass_So_ +
            (p_WCo_W + p_CoCcm_Co_) * mass_Co_) /
           (mass_So_ + mass_Co_);
  p_WCcm = plant_.CalcCenterOfMassPosition(*context_, model_instances);
  EXPECT_TRUE(CompareMatrices(p_WCcm, result, 1e-12));

  // Try error instance in model_instances.
  ModelInstanceIndex error_index(10);
  model_instances.insert(error_index);
  EXPECT_THROW(plant_.CalcCenterOfMassPosition(*context_, model_instances),
               std::runtime_error);

  // Try compute Center of Mass at random translation and rotation.
  math::RigidTransformd X_WSo(math::RollPitchYawd(0.3, -1.5, 0.7),
                              Eigen::Vector3d(5.2, -3.1, 10.9));
  math::RigidTransformd X_WCo(math::RollPitchYawd(-2.3, -3.5, 1.2),
                              Eigen::Vector3d(-70.2, 9.8, 843.1));
  CheckCOM(X_WSo, X_WCo);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
