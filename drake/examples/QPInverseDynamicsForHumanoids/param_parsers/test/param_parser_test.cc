#include "drake/examples/QPInverseDynamicsForHumanoids/param_parsers/param_parser.h"

#include <set>

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {
namespace param_parsers {
namespace {

class ParamParserTests : public ::testing::Test {
 protected:
  virtual void SetUp() {
    const std::string urdf_name =
        drake::GetDrakePath() +
        "/examples/Valkyrie/urdf/urdf/"
        "valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf";
    const std::string alias_groups_config_name =
        drake::GetDrakePath() +
        "/examples/QPInverseDynamicsForHumanoids/"
        "param_parsers/test/params.alias_groups";
    const std::string controller_config_name =
        drake::GetDrakePath() +
        "/examples/QPInverseDynamicsForHumanoids/"
        "param_parsers/test/params.id_controller_config";

    robot_ = std::make_unique<RigidBodyTree<double>>();
    parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
        urdf_name, multibody::joints::kRollPitchYaw, robot_.get());

    rbt_alias_ = std::make_unique<RigidBodyTreeAliasGroups<double>>(*robot_);
    rbt_alias_->LoadFromFile(alias_groups_config_name);

    paramset_.LoadFromFile(controller_config_name, *rbt_alias_);
  }

  std::unique_ptr<RigidBodyTree<double>> robot_;
  std::unique_ptr<RigidBodyTreeAliasGroups<double>> rbt_alias_;
  ParamSet paramset_;

  const double kTolerance = 1e-20;
};

// Tests for parsing params related to ContactInformation.
TEST_F(ParamParserTests, ContactInformation) {
  // Contacts:
  //   default:
  //     weight: 1e5
  //     contact_points: [0, 0, 0]
  //     Kd: 8
  //     mu: 1
  //     num_basis_per_contact_point: 3
  //     normal: [0, 0, 1]
  //
  //   left_foot:
  //     contact_points:
  //       [[0.2, 0.05, -0.09],
  //       [0.2, -0.05, -0.09],
  //       [-0.05, -0.05, -0.09],
  //       [-0.05, 0.05, -0.09]]
  //     num_basis_per_contact_point: 5
  //     weight: -1
  std::unordered_map<std::string, ContactInformation> contacts =
      paramset_.MakeContactInformation("feet", *rbt_alias_);
  {
    // "left_foot" is specified, should match exactly.
    const ContactInformation& contact = contacts.at("leftFoot");
    EXPECT_EQ(&contact.body(), robot_->FindBody("leftFoot"));
    EXPECT_EQ(contact.mu(), 1);
    EXPECT_EQ(contact.Kd(), 8);
    EXPECT_EQ(contact.num_basis_per_contact_point(), 5);
    EXPECT_EQ(contact.weight(), -1);
    EXPECT_EQ(contact.acceleration_constraint_type(), ConstraintType::Hard);
    EXPECT_TRUE(CompareMatrices(contact.normal(), Vector3<double>(0, 0, 1),
                                kTolerance, MatrixCompareType::absolute));
    EXPECT_EQ(contact.contact_points().cols(), 4);
    EXPECT_TRUE(CompareMatrices(contact.contact_points().col(0),
                                Vector3<double>(0.2, 0.05, -0.09), kTolerance,
                                MatrixCompareType::absolute));
    EXPECT_TRUE(CompareMatrices(contact.contact_points().col(1),
                                Vector3<double>(0.2, -0.05, -0.09), kTolerance,
                                MatrixCompareType::absolute));
    EXPECT_TRUE(CompareMatrices(contact.contact_points().col(2),
                                Vector3<double>(-0.05, -0.05, -0.09),
                                kTolerance, MatrixCompareType::absolute));
    EXPECT_TRUE(CompareMatrices(contact.contact_points().col(3),
                                Vector3<double>(-0.05, 0.05, -0.09), kTolerance,
                                MatrixCompareType::absolute));
  }

  {
    // "right_foot" is not specified, should match default.
    const ContactInformation& contact = contacts.at("rightFoot");
    EXPECT_EQ(&contact.body(), robot_->FindBody("rightFoot"));
    EXPECT_EQ(contact.mu(), 1);
    EXPECT_EQ(contact.Kd(), 8);
    EXPECT_EQ(contact.num_basis_per_contact_point(), 3);
    EXPECT_EQ(contact.weight(), 1e5);
    EXPECT_EQ(contact.acceleration_constraint_type(), ConstraintType::Soft);
    EXPECT_TRUE(CompareMatrices(contact.normal(), Vector3<double>(0, 0, 1),
                                kTolerance, MatrixCompareType::absolute));
    EXPECT_EQ(contact.contact_points().cols(), 1);
    EXPECT_TRUE(CompareMatrices(contact.contact_points().col(0),
                                Vector3<double>::Zero(), kTolerance,
                                MatrixCompareType::absolute));
  }

  // "NO_SUCH_BODY_GROUP" is not a valid group name, so lookup returns empty.
  EXPECT_TRUE(
      paramset_.MakeContactInformation("NO_SUCH_BODY_GROUP", *rbt_alias_)
          .empty());
}

void TestDesiredBodyMotion(const DesiredBodyMotion& motion,
                           const RigidBody<double>* body,
                           const Vector6<double>& expected_weights,
                           double tol) {
  EXPECT_EQ(&motion.body(), body);
  EXPECT_TRUE(CompareMatrices(motion.weights(), expected_weights, tol,
                              MatrixCompareType::absolute));
  for (int i = 0; i < 6; ++i) {
    if (expected_weights(i) > 0) {
      EXPECT_EQ(motion.constraint_type(i), ConstraintType::Soft);
    } else if (expected_weights(i) < 0) {
      EXPECT_EQ(motion.constraint_type(i), ConstraintType::Hard);
    } else {
      EXPECT_EQ(motion.constraint_type(i), ConstraintType::Skip);
    }
  }
}

// Tests for parsing params related to DesiredBodyMotion.
TEST_F(ParamParserTests, BodyMotionParams) {
  // BodyMotions:
  //   default:
  //     Kp: 0
  //     Kd: 0
  //     weight: [1e-2]
  //
  //   left_foot:
  //     Kp: 20
  //     weight: [1, 1, 1, 1, 1, 2]
  //
  //   pelvis:
  //     Kp: [20, 20, 20, 0, 0, 0]
  //     Kd: [8, 8, 8, 0, 0, 0]
  //     weight: [1, 1, 1, 0, 0, 0]

  // Tests for MakeDesiredBodyMotion.
  const RigidBody<double>* body = robot_->FindBody("pelvis");
  DesiredBodyMotion motion =
      paramset_.MakeDesiredBodyMotion("pelvis", *rbt_alias_).at("pelvis");
  Vector6<double> weights;
  weights << 1, 1, 1, 0, 0, 0;
  TestDesiredBodyMotion(motion, body, weights, kTolerance);

  // Unspecified, uses "default".
  body = robot_->FindBody("rightFoot");
  motion = paramset_.MakeDesiredBodyMotion(*body);
  weights = Vector6<double>::Constant(1e-2);
  TestDesiredBodyMotion(motion, body, weights, kTolerance);

  // Partially specified.
  body = robot_->FindBody("leftFoot");
  motion = paramset_.MakeDesiredBodyMotion(*body);
  weights << 1, 1, 1, 1, 1, 2;
  TestDesiredBodyMotion(motion, body, weights, kTolerance);

  // "NO_SUCH_BODY_GROUP" is not a valid group name, so this returns emtpy.
  EXPECT_TRUE(paramset_.MakeDesiredBodyMotion("NO_SUCH_BODY_GROUP", *rbt_alias_)
                  .empty());

  // Tests for LookupDesiredBodyMotionGains.
  std::vector<Vector6<double>> Kp_vec(1), Kd_vec(1);
  Vector6<double> Kp_expected, Kd_expected;

  paramset_.LookupDesiredBodyMotionGains(*robot_->FindBody("pelvis"),
                                         &(Kp_vec[0]), &(Kd_vec[0]));
  Kp_expected << 20, 20, 20, 0, 0, 0;
  Kd_expected << 8, 8, 8, 0, 0, 0;
  EXPECT_TRUE(CompareMatrices(Kp_vec[0], Kp_expected, kTolerance,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(Kd_vec[0], Kd_expected, kTolerance,
                              MatrixCompareType::absolute));

  // Body group "feet" maps to ["leftFoot", "rightFoot"]
  paramset_.LookupDesiredBodyMotionGains("feet", *rbt_alias_, &Kp_vec, &Kd_vec);
  EXPECT_EQ(Kp_vec.size(), 2u);
  EXPECT_EQ(Kd_vec.size(), 2u);

  // Left foot
  Kp_expected << 20, 20, 20, 20, 20, 20;
  Kd_expected << 0, 0, 0, 0, 0, 0;
  EXPECT_TRUE(CompareMatrices(Kp_vec[0], Kp_expected, kTolerance,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(Kd_vec[0], Kd_expected, kTolerance,
                              MatrixCompareType::absolute));
  // Right foot (not specified, equals default)
  Kp_expected << 0, 0, 0, 0, 0, 0;
  Kd_expected << 0, 0, 0, 0, 0, 0;
  EXPECT_TRUE(CompareMatrices(Kp_vec[1], Kp_expected, kTolerance,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(Kd_vec[1], Kd_expected, kTolerance,
                              MatrixCompareType::absolute));
}

// Tests for DesiredCentroidalMomentumDot related param parsing.
TEST_F(ParamParserTests, CentroidalMomentumDotParams) {
  // Tests for MakeDesiredCentroidalMomentumDot.
  DesiredCentroidalMomentumDot cdot =
      paramset_.MakeDesiredCentroidalMomentumDot();
  Vector6<double> weights;
  weights << 0, 0, 0, 10, 10, 10;
  EXPECT_TRUE(CompareMatrices(cdot.weights(), weights, kTolerance,
                              MatrixCompareType::absolute));

  // Tests for gains.
  Vector6<double> Kp, Kd;
  Vector6<double> Kp_expected, Kd_expected;
  Kp_expected << 0, 0, 0, 40, 40, 40;
  Kd_expected << 4, 4, 4, 12, 12, 12;
  paramset_.LookupDesiredCentroidalMomentumDotGains(&Kp, &Kd);
  EXPECT_TRUE(CompareMatrices(Kp, Kp_expected, kTolerance,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(Kd, Kd_expected, kTolerance,
                              MatrixCompareType::absolute));
}

// Helper function use to test parsing for the Dof motions. Checks numerical
// equality between @p motion, @p Kp, and @p kd, and values in the config file.
void TestDofMotionsParamsHelper(
    const DesiredDofMotions& motion, const VectorX<double>& Kp,
    const VectorX<double>& Kd,
    const RigidBodyTreeAliasGroups<double>& rbt_alias, double tol) {
  // DoFMotions:
  //   default:
  //     Kp: 0
  //     Kd: 0
  //     weight: 1e-2
  //
  //   floating_base:
  //     Kd: [1, 2, 3, 4, 5, 6]
  //     weight: 0
  //
  //   left_arm:
  //     Kp: 10
  //     Kd: 3
  //     weight: -1
  //
  //   right_arm:
  //     Kp: 20
  //     Kd: 11
  //     weight: 1
  std::set<int> l_arm(rbt_alias.get_velocity_group("left_arm").begin(),
                      rbt_alias.get_velocity_group("left_arm").end());
  std::set<int> r_arm(rbt_alias.get_velocity_group("right_arm").begin(),
                      rbt_alias.get_velocity_group("right_arm").end());
  std::set<int> floating_base(
      rbt_alias.get_velocity_group("floating_base").begin(),
      rbt_alias.get_velocity_group("floating_base").end());

  const RigidBodyTree<double>& robot = rbt_alias.get_tree();
  int dim = robot.get_num_velocities();
  for (int i = 0; i < dim; ++i) {
    if (l_arm.count(i)) {
      EXPECT_EQ(motion.weight(i), -1);
      EXPECT_EQ(motion.constraint_type(i), ConstraintType::Hard);
    } else if (r_arm.count(i)) {
      EXPECT_EQ(motion.weight(i), 1);
      EXPECT_EQ(motion.constraint_type(i), ConstraintType::Soft);
    } else if (floating_base.count(i)) {
      EXPECT_EQ(motion.weight(i), 0);
      EXPECT_EQ(motion.constraint_type(i), ConstraintType::Skip);
    } else {
      EXPECT_EQ(motion.weight(i), 1e-2);
      EXPECT_EQ(motion.constraint_type(i), ConstraintType::Soft);
    }
  }

  // Check gains.
  VectorX<double> Kp_expected, Kd_expected;
  Kp_expected = VectorX<double>::Constant(robot.get_num_velocities(), 0);
  Kd_expected = VectorX<double>::Constant(robot.get_num_velocities(), 0);
  for (int i : l_arm) {
    Kp_expected(i) = 10;
    Kd_expected(i) = 3;
  }
  for (int i : r_arm) {
    Kp_expected(i) = 20;
    Kd_expected(i) = 11;
  }
  int ctr = 1;
  for (int i : floating_base) {
    Kp_expected(i) = 0;
    Kd_expected(i) = ctr++;
  }
  EXPECT_TRUE(
      CompareMatrices(Kp, Kp_expected, tol, MatrixCompareType::absolute));
  EXPECT_TRUE(
      CompareMatrices(Kd, Kd_expected, tol, MatrixCompareType::absolute));
}

// Tests for DesiredDoFMotion related param parsing.
TEST_F(ParamParserTests, DoFParams) {
  DesiredDofMotions motion = paramset_.MakeDesiredDofMotions();
  VectorX<double> Kp, Kd;
  paramset_.LookupDesiredDofMotionGains(&Kp, &Kd);

  TestDofMotionsParamsHelper(motion, Kp, Kd, *rbt_alias_, kTolerance);
}

// Tests MakeQpInput
TEST_F(ParamParserTests, MakeQpInput) {
  QpInput qp_input =
      paramset_.MakeQpInput({"pelvis"},               /* contact groups */
                            {"pelvis", "right_foot"}, /* tracked body groups */
                            *rbt_alias_);

  // Tests for body motion params.
  EXPECT_EQ(qp_input.desired_body_motions().size(), 2);

  Vector6<double> weights;
  const RigidBody<double>* body = robot_->FindBody("pelvis");
  weights << 1, 1, 1, 0, 0, 0;
  TestDesiredBodyMotion(qp_input.desired_body_motions().at("pelvis"), body,
                        weights, kTolerance);

  body = robot_->FindBody("rightFoot");
  weights = Vector6<double>::Constant(1e-2);
  TestDesiredBodyMotion(qp_input.desired_body_motions().at("rightFoot"), body,
                        weights, kTolerance);

  // Tests for contacts, pelvis should be the default contact.
  EXPECT_EQ(qp_input.contact_information().size(), 1);
  const ContactInformation& contact =
      qp_input.contact_information().at("pelvis");
  EXPECT_EQ(&contact.body(), robot_->FindBody("pelvis"));
  EXPECT_EQ(contact.mu(), 1);
  EXPECT_EQ(contact.Kd(), 8);
  EXPECT_EQ(contact.num_basis_per_contact_point(), 3);
  EXPECT_EQ(contact.weight(), 1e5);
  EXPECT_EQ(contact.acceleration_constraint_type(), ConstraintType::Soft);
  EXPECT_TRUE(CompareMatrices(contact.normal(), Vector3<double>(0, 0, 1),
                              kTolerance, MatrixCompareType::absolute));
  EXPECT_EQ(contact.contact_points().cols(), 1);
  EXPECT_TRUE(CompareMatrices(contact.contact_points().col(0),
                              Vector3<double>::Zero(), kTolerance,
                              MatrixCompareType::absolute));

  // Tests for contact force basis regularization.
  EXPECT_EQ(qp_input.w_basis_reg(), 1e-6);

  // Tests for Dof motion.
  VectorX<double> Kp, Kd;
  paramset_.LookupDesiredDofMotionGains(&Kp, &Kd);
  TestDofMotionsParamsHelper(qp_input.desired_dof_motions(), Kp, Kd,
                             *rbt_alias_, kTolerance);

  // Tests for centroidal momentum rate.
  weights << 0, 0, 0, 10, 10, 10;
  EXPECT_TRUE(
      CompareMatrices(qp_input.desired_centroidal_momentum_dot().weights(),
                      weights, kTolerance, MatrixCompareType::absolute));
}

}  // namespace
}  // namespace param_parsers
}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
