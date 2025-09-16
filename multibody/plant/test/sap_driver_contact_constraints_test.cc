#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/contact_solvers/sap/sap_contact_problem.h"
#include "drake/multibody/contact_solvers/sap/sap_friction_cone_constraint.h"
#include "drake/multibody/contact_solvers/sap/sap_hunt_crossley_constraint.h"
#include "drake/multibody/plant/compliant_contact_manager.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/sap_driver.h"
#include "drake/multibody/plant/test/compliant_contact_manager_tester.h"

/* @file This file tests SapDriver's support for contact constraints. */

using drake::geometry::ProximityProperties;
using drake::math::RigidTransformd;
using drake::multibody::contact_solvers::internal::ContactConfiguration;
using drake::multibody::contact_solvers::internal::SapConstraint;
using drake::multibody::contact_solvers::internal::SapContactProblem;
using drake::multibody::contact_solvers::internal::SapFrictionConeConstraint;
using drake::multibody::contact_solvers::internal::SapHuntCrossleyApproximation;
using drake::multibody::contact_solvers::internal::SapHuntCrossleyConstraint;
using drake::systems::Context;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace drake {
namespace multibody {
namespace internal {

// Friend class used to provide access to a selection of private functions in
// SapDriver for testing purposes.
class SapDriverTest {
 public:
  static const ContactProblemCache<double>& EvalContactProblemCache(
      const SapDriver<double>& driver, const Context<double>& context) {
    return driver.EvalContactProblemCache(context);
  }
};

struct TestConfig {
  // This is a gtest test suffix; no underscores or spaces.
  std::string description;
  // Contact model approximation.
  DiscreteContactApproximation contact_approximation;
};

// This provides the suffix for each test parameter: the test config
// description.
std::ostream& operator<<(std::ostream& out, const TestConfig& c) {
  out << c.description;
  return out;
}

// Fixture that sets a MultibodyPlant model with two bodies A and B in contact.
// The purpose of this test is to verify that contact constraints are
// appropiately added by the SapDriver according to the model approximation set
// in the TestConfig.
class TwoBodiesTest : public ::testing::TestWithParam<TestConfig> {
 public:
  // Makes a model with two spherical bodies A and B in contact.
  void MakeModel() {
    systems::DiagramBuilder<double> builder;
    plant_ =
        &AddMultibodyPlantSceneGraph(&builder, 0.01 /* Discrete model */).plant;
    plant_->set_discrete_contact_approximation(
        GetParam().contact_approximation);

    // Arbitrary inertia values only used by the driver to build a valid contact
    // problem.
    const double mass = 1.5;
    const double radius = 0.1;
    const SpatialInertia<double> M_BBcm =
        SpatialInertia<double>::SolidSphereWithMass(mass, radius);

    // Both bodies have the same geometry and proximity properties.
    const ProximityProperties properties = MakeProximityProperties();
    const geometry::Sphere shape(kRadius_);

    bodyA_ = &plant_->AddRigidBody("A", M_BBcm);
    plant_->RegisterCollisionGeometry(*bodyA_, RigidTransformd(), shape,
                                      "bodyA_collision", properties);
    bodyB_ = &plant_->AddRigidBody("B", M_BBcm);
    plant_->RegisterCollisionGeometry(*bodyB_, RigidTransformd(), shape,
                                      "bodyB_collision", properties);

    plant_->Finalize();

    // Adding the manager programatically gives us access to the manager and the
    // driver.
    auto owned_contact_manager =
        std::make_unique<CompliantContactManager<double>>();
    manager_ = owned_contact_manager.get();
    plant_->SetDiscreteUpdateManager(std::move(owned_contact_manager));

    // Build Diagram with MultibodyPlant and SceneGraph connected, and create
    // context.
    diagram_ = builder.Build();
    context_ = diagram_->CreateDefaultContext();
    plant_context_ = &plant_->GetMyMutableContextFromRoot(context_.get());

    // Set the state so that the spheres are in contact.
    const RigidTransformd X_WA(Vector3d(0.0, 0.0, 0.0));
    const RigidTransformd X_WB(
        Vector3d(2.0 * kRadius_ - kPenetration_, 0.0, 0.0));
    plant_->SetFreeBodyPose(plant_context_, *bodyA_, X_WA);
    plant_->SetFreeBodyPose(plant_context_, *bodyB_, X_WB);
  }

 protected:
  ProximityProperties MakeProximityProperties() const {
    ProximityProperties properties;
    geometry::AddContactMaterial(
        kHuntCrossleyDissipation_, kStiffness_,
        CoulombFriction<double>(kFriction_, kFriction_), &properties);
    properties.AddProperty(geometry::internal::kMaterialGroup,
                           geometry::internal::kRelaxationTime,
                           kRelaxationTime_);
    return properties;
  }

  const SapDriver<double>& sap_driver() const {
    return CompliantContactManagerTester::sap_driver(*manager_);
  }

  std::unique_ptr<systems::Diagram<double>> diagram_;
  MultibodyPlant<double>* plant_{nullptr};
  const RigidBody<double>* bodyA_{nullptr};
  const RigidBody<double>* bodyB_{nullptr};
  CompliantContactManager<double>* manager_{nullptr};
  std::unique_ptr<Context<double>> context_;
  systems::Context<double>* plant_context_{nullptr};

  // Parameters of the problem.
  const double kRadius_{0.01};
  const double kPenetration_{1.0e-3};  // Penetration of the spheres in contact.
  const double kStiffness_{3.0e4};
  const double kHuntCrossleyDissipation_{25.0};
  const double kRelaxationTime_{0.01};
  const double kFriction_{1.2};
};

// This test verifies that the SapContactProblem built by the driver is
// consistent with the contact kinematics computed with CalcContactKinematics().
TEST_P(TwoBodiesTest, ConfirmContactConstraintProperties) {
  // const TestConfig& config = GetParam();
  MakeModel();

  const ContactProblemCache<double>& problem_cache =
      SapDriverTest::EvalContactProblemCache(sap_driver(), *plant_context_);
  const SapContactProblem<double>& problem = *problem_cache.sap_problem;

  // Verify the expected number of constraints and equations for a contact.
  EXPECT_EQ(problem.num_constraints(), 1);
  EXPECT_EQ(problem.num_constraint_equations(), 3);

  // Verify it is only a single point contact.
  const DiscreteContactData<DiscreteContactPair<double>>& contact_pairs =
      manager_->EvalDiscreteContactPairs(*plant_context_);
  EXPECT_EQ(contact_pairs.size(), 1);
  EXPECT_EQ(contact_pairs.num_point_contacts(), 1);

  const DiscreteContactPair<double>& discrete_pair = contact_pairs[0];
  const SapConstraint<double>& constraint = problem.get_constraint(0);

  // Check Jacobian and number of cliques.
  ASSERT_EQ(constraint.num_cliques(), 2);  // Two floating bodies.
  EXPECT_EQ(constraint.first_clique(), discrete_pair.jacobian[0].tree);
  EXPECT_EQ(constraint.first_clique_jacobian().MakeDenseMatrix(),
            discrete_pair.jacobian[0].J.MakeDenseMatrix());
  EXPECT_EQ(constraint.second_clique(), discrete_pair.jacobian[1].tree);
  EXPECT_EQ(constraint.second_clique_jacobian().MakeDenseMatrix(),
            discrete_pair.jacobian[1].J.MakeDenseMatrix());

  const ContactConfiguration<double> expected_configuration =
      multibody::contact_solvers::internal::MakeContactConfiguration(
          discrete_pair);

  // Verify constraint type, configuration and its parameters.
  if (GetParam().contact_approximation == DiscreteContactApproximation::kSap) {
    // Constraint type.
    const auto* contact_constraint =
        dynamic_cast<const SapFrictionConeConstraint<double>*>(&constraint);
    ASSERT_NE(contact_constraint, nullptr);
    // Configuration.
    EXPECT_EQ(contact_constraint->configuration(), expected_configuration);
    // Parameters.
    EXPECT_EQ(contact_constraint->parameters().mu,
              discrete_pair.friction_coefficient);
    EXPECT_EQ(contact_constraint->parameters().stiffness,
              discrete_pair.stiffness);
    EXPECT_EQ(contact_constraint->parameters().dissipation_time_scale,
              discrete_pair.dissipation_time_scale);
    EXPECT_EQ(contact_constraint->parameters().beta,
              plant_->get_sap_near_rigid_threshold());
    // This parameter sigma is for now hard-code in the manager to these value.
    // Here we simply test they are consistent with those hard-coded values.
    EXPECT_EQ(contact_constraint->parameters().sigma, 1.0e-3);
  } else {
    // Constraint type.
    const auto* contact_constraint =
        dynamic_cast<const SapHuntCrossleyConstraint<double>*>(&constraint);
    ASSERT_NE(contact_constraint, nullptr);
    // Configuration.
    EXPECT_EQ(contact_constraint->configuration(), expected_configuration);
    // Parameters.
    const SapHuntCrossleyApproximation expected_model =
        GetParam().contact_approximation ==
                DiscreteContactApproximation::kLagged
            ? SapHuntCrossleyApproximation::kLagged
            : SapHuntCrossleyApproximation::kSimilar;
    EXPECT_EQ(contact_constraint->parameters().model, expected_model);
    EXPECT_EQ(contact_constraint->parameters().friction,
              discrete_pair.friction_coefficient);
    EXPECT_EQ(contact_constraint->parameters().stiffness,
              discrete_pair.stiffness);
    EXPECT_EQ(contact_constraint->parameters().dissipation,
              discrete_pair.damping);
    EXPECT_EQ(contact_constraint->parameters().stiction_tolerance,
              plant_->stiction_tolerance());
    // This parameter sigma is for now hard-coded in the manager to this value.
    // Here we simply test it is consistent with that hard-coded value.
    EXPECT_EQ(contact_constraint->parameters().sigma, 1.0e-3);
  }
}

std::vector<TestConfig> MakeTestCases() {
  return std::vector<TestConfig>{
      {.description = "SAP",
       .contact_approximation = DiscreteContactApproximation::kSap},
      {.description = "Lagged",
       .contact_approximation = DiscreteContactApproximation::kLagged},
      {.description = "Similar",
       .contact_approximation = DiscreteContactApproximation::kSimilar},
  };
}

INSTANTIATE_TEST_SUITE_P(SapDriverContactConstraintsTests, TwoBodiesTest,
                         testing::ValuesIn(MakeTestCases()),
                         testing::PrintToStringParamName());

}  // namespace internal
}  // namespace multibody
}  // namespace drake
