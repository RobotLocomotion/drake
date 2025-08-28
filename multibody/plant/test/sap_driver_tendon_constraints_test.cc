#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/contact_solvers/sap/sap_contact_problem.h"
#include "drake/multibody/contact_solvers/sap/sap_tendon_constraint.h"
#include "drake/multibody/plant/compliant_contact_manager.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/sap_driver.h"
#include "drake/multibody/plant/test/compliant_contact_manager_tester.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/universal_joint.h"

/* @file This file tests SapDriver's support for tendon constraints.

  Constraints are only supported by the SAP solver. Therefore, to exercise the
  relevant code paths, we arbitrarily choose one contact approximation that uses
  the SAP solver. More precisely, in the unit tests below we call
  set_discrete_contact_approximation(DiscreteContactApproximation::kSap) on the
  MultibodyPlant used for testing, before constraints are added. */

using drake::math::RigidTransformd;
using drake::multibody::PrismaticJoint;
using drake::multibody::RevoluteJoint;
using drake::multibody::RigidBody;
using drake::multibody::UniversalJoint;
using drake::multibody::contact_solvers::internal::SapContactProblem;
using drake::multibody::contact_solvers::internal::SapTendonConstraint;
using drake::systems::Context;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

constexpr double kInfinity = std::numeric_limits<double>::infinity();

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
  int num_cliques{};
  double lower_limit{};
  double upper_limit{};
};

// This provides the suffix for each test parameter: the test config
// description.
std::ostream& operator<<(std::ostream& out, const TestConfig& c) {
  out << c.description;
  return out;
}

// Fixture that sets a MultibodyPlant model of two kinematic trees:
// (1) Chain of two bodies connected to world by revolute joints.
// (2) A single body connected to world by a prismatic joint.
// All of the physical properties of the bodies are arbitrary. Then
// a single tendon constraint is added coupling the joints of (1) and
// (optionally) coupling the joints of (2).
class TwoTreesTest : public ::testing::TestWithParam<TestConfig> {
 public:
  // Makes the model described in the fixture's documentation.
  void SetUp() {
    plant_.set_discrete_contact_approximation(
        DiscreteContactApproximation::kSap);

    // Arbitrary inertia values only used by the driver to build a valid contact
    // problem.
    const double mass = 1.5;
    const double radius = 0.1;
    const SpatialInertia<double> M_BBcm =
        SpatialInertia<double>::SolidSphereWithMass(mass, radius);

    const RigidBody<double>& bodyA = plant_.AddRigidBody("A", M_BBcm);
    const RigidBody<double>& bodyB = plant_.AddRigidBody("B", M_BBcm);
    const RigidBody<double>& bodyC = plant_.AddRigidBody("C", M_BBcm);

    const RevoluteJoint<double>& world_bodyA = plant_.AddJoint<RevoluteJoint>(
        "world_bodyA", plant_.world_body(), {}, bodyA, {}, Vector3d::UnitX());
    const RevoluteJoint<double>& bodyA_bodyB = plant_.AddJoint<RevoluteJoint>(
        "bodyA_bodyB", bodyA, {}, bodyB, {}, Vector3d::UnitX());
    const PrismaticJoint<double>& world_bodyC = plant_.AddJoint<PrismaticJoint>(
        "world_bodyC", plant_.world_body(), {}, bodyC, {}, Vector3d::UnitZ());

    std::vector<JointIndex> joints = {world_bodyA.index(), bodyA_bodyB.index()};
    std::vector<double> coefficients = {1.2, 3.4};

    // Optionally add (1) to the constraint.
    if (GetParam().num_cliques == 2) {
      joints.push_back(world_bodyC.index());
      coefficients.push_back(5.6);
    }

    // Copy the joints and coefficients.
    joints_ = joints;
    coefficients_ = coefficients;

    id_ = plant_.AddTendonConstraint(
        joints, coefficients, kOffset_, GetParam().lower_limit,
        GetParam().upper_limit, kStiffness_, kDamping_);

    plant_.Finalize();

    auto owned_contact_manager =
        std::make_unique<CompliantContactManager<double>>();
    manager_ = owned_contact_manager.get();
    plant_.SetDiscreteUpdateManager(std::move(owned_contact_manager));
    // Model with a single tendon constraint.
    EXPECT_EQ(plant_.num_constraints(), 1);
    context_ = plant_.CreateDefaultContext();
  }

  const SapDriver<double>& sap_driver() const {
    return CompliantContactManagerTester::sap_driver(*manager_);
  }

 protected:
  MultibodyPlant<double> plant_{0.01};  // Discrete model.
  CompliantContactManager<double>* manager_{nullptr};
  std::unique_ptr<Context<double>> context_;
  std::vector<JointIndex> joints_;
  std::vector<double> coefficients_;
  MultibodyConstraintId id_;
  // Parameters of the constraint.
  const double kStiffness_{3.0e4};
  const double kDamping_{1.5};
  const double kOffset_{1.2};
};

// This test configures a single tendon constraint in variety of ways (causing
// differing numbers of cliques and varying constraint properties). It then
// examines the newly added constraint to confirm that its instantiation
// reflects the specification.
TEST_P(TwoTreesTest, ConfirmConstraintProperties) {
  const TestConfig& config = GetParam();

  ASSERT_EQ(plant_.num_velocities(), 3);

  const bool has_lower_limit = config.lower_limit > -kInfinity;
  const bool has_upper_limit = config.upper_limit < kInfinity;

  ASSERT_TRUE(has_lower_limit || has_upper_limit);

  EXPECT_EQ(plant_.num_constraints(), 1);
  EXPECT_EQ(plant_.num_tendon_constraints(), 1);

  // Confirm the spec was added properly.
  const internal::TendonConstraintSpec& spec =
      plant_.get_tendon_constraint_specs(id_);
  EXPECT_EQ(spec.joints, joints_);
  EXPECT_EQ(spec.a, coefficients_);
  EXPECT_EQ(spec.offset, kOffset_);
  EXPECT_EQ(spec.lower_limit, config.lower_limit);
  EXPECT_EQ(spec.upper_limit, config.upper_limit);
  EXPECT_EQ(spec.stiffness, kStiffness_);
  EXPECT_EQ(spec.damping, kDamping_);
  EXPECT_EQ(spec.id, id_);

  const std::map<MultibodyConstraintId, TendonConstraintSpec>& tendon_specs =
      plant_.get_tendon_constraint_specs();
  ASSERT_EQ(ssize(tendon_specs), 1);

  const MultibodyConstraintId tendon_id_from_map = tendon_specs.begin()->first;
  const TendonConstraintSpec& tendon_spec_from_map =
      tendon_specs.begin()->second;
  EXPECT_EQ(tendon_spec_from_map, spec);

  // Check the id in the map matches the one returned.
  EXPECT_EQ(tendon_id_from_map, id_);

  // Throw on id to wrong constraint specs type.
  EXPECT_THROW(plant_.get_ball_constraint_specs(id_), std::exception);
  EXPECT_THROW(plant_.get_coupler_constraint_specs(id_), std::exception);
  EXPECT_THROW(plant_.get_weld_constraint_specs(id_), std::exception);

  // The tendon constraint is only added to the problem if it is violated in the
  // current configuration. The constraint is set up such that all coefficients
  // are positive, and either lower_limit < 0 or upper_limit > 0 when finite.
  // The constraint should be satisfied if q = 0.
  {
    plant_.SetPositions(context_.get(), Vector3d::Zero());

    const ContactProblemCache<double>& problem_cache =
        SapDriverTest::EvalContactProblemCache(sap_driver(), *context_);
    const SapContactProblem<double>& problem = *problem_cache.sap_problem;

    // Verify the expected number of constraints and equations.
    EXPECT_EQ(problem.num_constraints(), 0);
    EXPECT_EQ(problem.num_constraint_equations(), 0);
  }

  // Set the configuration such that the constraint is violated. We'll observe a
  // non-zero value when evaluating the constraint function.
  if (has_lower_limit) {
    plant_.SetPositions(context_.get(), config.lower_limit * Vector3d(1, 2, 3));
  } else {
    plant_.SetPositions(context_.get(), config.upper_limit * Vector3d(1, 2, 3));
  }

  const ContactProblemCache<double>& problem_cache =
      SapDriverTest::EvalContactProblemCache(sap_driver(), *context_);
  const SapContactProblem<double>& problem = *problem_cache.sap_problem;

  // Verify the expected number of constraints and equations for a single
  // tendon constraint.
  const int expected_num_constraint_equations =
      (has_lower_limit && has_upper_limit ? 2 : 1);
  EXPECT_EQ(problem.num_constraints(), 1);
  EXPECT_EQ(problem.num_constraint_equations(),
            expected_num_constraint_equations);

  const auto* constraint = dynamic_cast<const SapTendonConstraint<double>*>(
      &problem.get_constraint(0));
  // Verify it is a SapTendonConstraint as expected.
  ASSERT_NE(constraint, nullptr);
  EXPECT_EQ(constraint->num_cliques(), config.num_cliques);
  // There are only two cliques in the problem. We exploit internal knowledge of
  // the size and ordering of dofs in the cliques. The first tree of two bodies
  // makes clique 0 and the second tree makes clique 1.
  if (config.num_cliques == 1) {
    EXPECT_EQ(constraint->first_clique(), 0);
    EXPECT_THROW(constraint->second_clique(), std::exception);
  } else {
    EXPECT_EQ(constraint->first_clique(), 0);
    EXPECT_EQ(constraint->second_clique(), 1);
  }

  // Verify parameters.
  const SapTendonConstraint<double>::Parameters p = constraint->parameters();
  EXPECT_EQ(p.lower_limit(), config.lower_limit);
  EXPECT_EQ(p.upper_limit(), config.upper_limit);
  EXPECT_EQ(p.stiffness(), kStiffness_);
  EXPECT_EQ(p.damping(), kDamping_);
  // This value is hard-coded in the source. This test serves as a brake to
  // prevent the value changing without notification. Changing this value
  // would lead to a behavior change and shouldn't happen silently.
  EXPECT_EQ(p.beta(), 0.1);
  EXPECT_EQ(p.has_finite_lower_limit(), has_lower_limit);
  EXPECT_EQ(p.has_finite_upper_limit(), has_upper_limit);
  EXPECT_EQ(p.num_finite_limits(), expected_num_constraint_equations);

  // Verify kinematics.
  const SapTendonConstraint<double>::Kinematics k = constraint->kinematics();
  // q is expected to be partitioned as [q0, q1].
  const VectorXd q = plant_.GetPositions(*context_);
  EXPECT_EQ(k.num_cliques(), config.num_cliques);
  EXPECT_EQ(k.clique0(), 0);
  EXPECT_EQ(k.clique0_nv(), 2);
  EXPECT_EQ(k.q0(), q.segment<2>(0));
  EXPECT_THAT(k.a0(), testing::ElementsAre(coefficients_[0], coefficients_[1]));
  EXPECT_EQ(k.offset(), kOffset_);
  if (config.num_cliques == 2) {
    EXPECT_EQ(k.clique1(), 1);
    EXPECT_EQ(k.clique1_nv(), 1);
    EXPECT_EQ(k.q1(), q.segment<1>(2));
    EXPECT_THAT(k.a1(), testing::ElementsAre(coefficients_[2]));
  }
}

// While not *strictly necessary*, as the properties for number of cliques and
// number of finit limits can be checked independent of each other, we test all
// possible combinations of {1, 2} cliques and {only upper limits, only lower
// limits, both}.
std::vector<TestConfig> MakeTestCases() {
  return std::vector<TestConfig>{
      {
          .description = "SingleClique",
          .num_cliques = 1,
          .lower_limit = -1.2,
          .upper_limit = 3.4,
      },
      {
          .description = "TwoCliques",
          .num_cliques = 2,
          .lower_limit = -1.2,
          .upper_limit = 3.4,
      },
      {
          .description = "SingleCliqueLowerOnly",
          .num_cliques = 1,
          .lower_limit = -1.2,
          .upper_limit = kInfinity,
      },
      {
          .description = "TwoCliquesLowerOnly",
          .num_cliques = 2,
          .lower_limit = -1.2,
          .upper_limit = kInfinity,
      },
      {
          .description = "SingleCliqueUpperOnly",
          .num_cliques = 1,
          .lower_limit = -kInfinity,
          .upper_limit = 3.4,
      },
      {
          .description = "TwoCliquesUpperOnly",
          .num_cliques = 1,
          .lower_limit = -kInfinity,
          .upper_limit = 3.4,
      },
  };
}

INSTANTIATE_TEST_SUITE_P(SapTendonConstraintTests, TwoTreesTest,
                         testing::ValuesIn(MakeTestCases()),
                         testing::PrintToStringParamName());

// This test suite covers all expected failure modes of
// MultibodyPlant::AddTendonConstraint(), as well as validating default values
// for optional arguments.
class SimplePlant : public ::testing::Test {
 public:
  // Makes a simple model for error testing.
  void MakePlant(const double dt = 0.1) {
    plant_ = std::make_unique<MultibodyPlant<double>>(dt);
    bodyA_ = &plant_->AddRigidBody("A", SpatialInertia<double>::NaN());
    bodyB_ = &plant_->AddRigidBody("B", SpatialInertia<double>::NaN());
    single_dof_joint_ = &plant_->AddJoint<RevoluteJoint>(
        "joint0", plant_->world_body(), {}, *bodyA_, {}, Vector3d::UnitX());
    multi_dof_joint_ = &plant_->AddJoint<UniversalJoint>(
        "joint1", plant_->world_body(), {}, *bodyB_, {});
    valid_joints_.push_back(single_dof_joint_->index());
  }

 protected:
  std::unique_ptr<MultibodyPlant<double>> plant_;
  const RigidBody<double>* bodyA_{};
  const RigidBody<double>* bodyB_{};
  const RevoluteJoint<double>* single_dof_joint_{};
  const UniversalJoint<double>* multi_dof_joint_{};

  std::vector<JointIndex> valid_joints_;
  std::vector<double> valid_a_{1.0};
  double valid_offset_{1.0};
  double valid_lower_limit_{-2.0};
  double valid_upper_limit_{2.0};
  double valid_stiffness_{1e3};
  double valid_damping_{5};
};

TEST_F(SimplePlant, FailOnTAMSI) {
  MakePlant();
  plant_->set_discrete_contact_approximation(
      DiscreteContactApproximation::kTamsi);

  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_->AddTendonConstraint({single_dof_joint_->index()}, {1.0}, {}, {},
                                  {}, {}, {}),
      ".*TAMSI does not support tendon constraints.*");
}

TEST_F(SimplePlant, FailOnContinuous) {
  MakePlant(0.0);  // Continuous plant.
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_->AddTendonConstraint({single_dof_joint_->index()}, {1.0}, {}, {},
                                  {}, {}, {}),
      ".*Currently tendon constraints are only supported for discrete "
      "MultibodyPlant models.*");
}

TEST_F(SimplePlant, FailOnFinalized) {
  MakePlant();
  plant_->Finalize();
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_->AddTendonConstraint({single_dof_joint_->index()}, {1.0}, {}, {},
                                  {}, {}, {}),
      ".*Post-finalize calls to 'AddTendonConstraint\\(\\)' are not "
      "allowed.*");
}

TEST_F(SimplePlant, TestDefaults) {
  MakePlant();
  // Smoke-test for simple valid case.
  EXPECT_NO_THROW(plant_->AddTendonConstraint(
      valid_joints_, valid_a_, valid_offset_, valid_lower_limit_,
      valid_upper_limit_, valid_stiffness_, valid_damping_));

  MultibodyConstraintId id;

  // At least one of lower_limit/upper_limit mush be finite. First test infinite
  // lower_limit.
  EXPECT_NO_THROW(
      id = plant_->AddTendonConstraint(valid_joints_, valid_a_, {} /* offset */,
                                       {} /* lower_limit */, valid_upper_limit_,
                                       {} /* stiffness */, {} /* damping */));

  TendonConstraintSpec spec = plant_->get_tendon_constraint_specs(id);
  EXPECT_EQ(spec.joints, valid_joints_);
  EXPECT_EQ(spec.a, valid_a_);
  EXPECT_EQ(spec.upper_limit, valid_upper_limit_);

  // Expected defaults
  EXPECT_EQ(spec.offset, 0.0);
  EXPECT_EQ(spec.lower_limit, -kInfinity);
  EXPECT_EQ(spec.stiffness, kInfinity);
  EXPECT_EQ(spec.damping, 0.0);

  // Test infinite upper limit.
  EXPECT_NO_THROW(
      id = plant_->AddTendonConstraint(valid_joints_, valid_a_, {} /* offset */,
                                       valid_lower_limit_, {} /* upper_limit */,
                                       {} /* stiffness */, {} /* damping */));
  spec = plant_->get_tendon_constraint_specs(id);

  // Expected defaults.
  EXPECT_EQ(spec.upper_limit, kInfinity);
}

TEST_F(SimplePlant, FailOnInvalidSpecs) {
  MakePlant();

  // joints.size() == 0.
  EXPECT_THROW(plant_->AddTendonConstraint(
                   {} /* joints */, valid_a_, valid_offset_, valid_lower_limit_,
                   valid_upper_limit_, valid_stiffness_, valid_damping_),
               std::exception);

  // Duplicated joint in `joints`
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_->AddTendonConstraint(
          {single_dof_joint_->index(), multi_dof_joint_->index(),
           single_dof_joint_->index()} /* joints */,
          {valid_a_[0], valid_a_[0], valid_a_[0]}, valid_offset_,
          valid_lower_limit_, valid_upper_limit_, valid_stiffness_,
          valid_damping_),
      "AddTendonConstraint\\(\\): Duplicated joint in `joints`. `joints` must "
      "be a unique set of JointIndex.");

  // a.size() != joints.size().
  EXPECT_THROW(plant_->AddTendonConstraint(valid_joints_, {1.0, 2.0} /* a */,
                                           valid_offset_, valid_lower_limit_,
                                           valid_upper_limit_, valid_stiffness_,
                                           valid_damping_),
               std::exception);

  // joint.num_velocities() > 1.
  EXPECT_THROW(plant_->AddTendonConstraint(
                   {multi_dof_joint_->index()} /* joints */, valid_a_,
                   valid_offset_, valid_lower_limit_, valid_upper_limit_,
                   valid_stiffness_, valid_damping_),
               std::exception);

  // lower_limit == ∞.
  EXPECT_THROW(
      plant_->AddTendonConstraint(
          valid_joints_, valid_a_, valid_offset_, kInfinity /* lower_limit */,
          valid_upper_limit_, valid_stiffness_, valid_damping_),
      std::exception);

  // upper_limit == -∞.
  EXPECT_THROW(plant_->AddTendonConstraint(valid_joints_, valid_a_,
                                           valid_offset_, valid_lower_limit_,
                                           -kInfinity /* upper_limit */,
                                           valid_stiffness_, valid_damping_),
               std::exception);

  // Both lower_limit and upper_limit are infinite.
  EXPECT_THROW(plant_->AddTendonConstraint(
                   valid_joints_, valid_a_, valid_offset_, {} /* lower_limit */,
                   {} /* upper_limit */, valid_stiffness_, valid_damping_),
               std::exception);

  // @pre lower_limit > upper_limit.
  EXPECT_THROW(
      plant_->AddTendonConstraint(valid_joints_, valid_a_, valid_offset_,
                                  1.0 /* lower_limit */, -1.0 /* upper_limit */,
                                  valid_stiffness_, valid_damping_),
      std::exception);

  // stiffness <= 0.
  EXPECT_THROW(plant_->AddTendonConstraint(
                   valid_joints_, valid_a_, valid_offset_, valid_lower_limit_,
                   valid_upper_limit_, -1.0 /* stiffness */, valid_damping_),
               std::exception);

  //  damping < 0.
  EXPECT_THROW(plant_->AddTendonConstraint(
                   valid_joints_, valid_a_, valid_offset_, valid_lower_limit_,
                   valid_upper_limit_, valid_stiffness_, -1.0),
               std::exception);

  // Removed joint.
  const JointIndex removed_joint_index = single_dof_joint_->index();
  plant_->RemoveJoint(*single_dof_joint_);
  EXPECT_THROW(plant_->AddTendonConstraint({removed_joint_index}, valid_a_,
                                           valid_offset_, valid_lower_limit_,
                                           valid_upper_limit_, valid_stiffness_,
                                           valid_damping_),
               std::exception);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
