#include <cstdlib>

#include "gtest/gtest.h"

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/example_qp_input_for_valkyrie.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/lcm_utils.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {
namespace {

static std::string get_urdf_name() {
  std::string urdf =
      drake::GetDrakePath() +
      std::string(
          "/examples/Valkyrie/urdf/urdf/"
          "valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf");
  return urdf;
}
static RigidBodyTree<double> robot(
    get_urdf_name(), drake::multibody::joints::kRollPitchYaw);

// Test if an Eigen vector (row or column) equals a std vector.
template <typename Derived, typename Scalar>
void TestEigenVectorAndStdVector(const Eigen::MatrixBase<Derived>& eigvec,
                                 const std::vector<Scalar>& stdvec) {
  EXPECT_TRUE(eigvec.rows() == 1 || eigvec.cols() == 1);
  EXPECT_EQ(static_cast<size_t>(eigvec.size()), stdvec.size());
  for (size_t i = 0; i < stdvec.size(); ++i) {
    EXPECT_EQ(static_cast<Scalar>(eigvec(i)), stdvec[i]);
  }
}

// Test if an Eigen vector (row or column) equals a C array (with known fixed
// size).
template <typename Derived, typename Scalar, size_t Size>
void TestEigenVectorAndCArray(const Eigen::MatrixBase<Derived>& eigvec,
                              const Scalar(&array)[Size]) {
  EXPECT_TRUE(eigvec.rows() == 1 || eigvec.cols() == 1);
  EXPECT_EQ(static_cast<size_t>(eigvec.size()), Size);
  for (size_t i = 0; i < Size; ++i) {
    EXPECT_EQ(static_cast<Scalar>(eigvec(i)), array[i]);
  }
}

// Test if an Eigen Matrix equals a std vector of std vectors.
// M(i, j) == VoV[i][j]
template <typename Derived, typename Scalar>
void TestEigenMatrixAndStdVectorOfStdVector(
    const Eigen::MatrixBase<Derived>& eigmat,
    const std::vector<std::vector<Scalar>>& stdvecvec) {
  EXPECT_EQ(static_cast<size_t>(eigmat.rows()), stdvecvec.size());
  for (int row = 0; row < eigmat.rows(); ++row) {
    TestEigenVectorAndStdVector(eigmat.row(row), stdvecvec[row]);
  }
}

// Resize val to the given dimension, then set value and weight to alternating
// positive and negative values, finally set the constraint type based on the
// weight's sign.
static void SetConstrainedValues(ConstrainedValues* val, int dim) {
  if (!val) return;
  val->mutable_values() = VectorX<double>::Random(dim);
  val->mutable_weights() = VectorX<double>::Random(dim);
  val->mutable_constraint_types().resize(dim);
  for (int i = 0; i < dim; ++i) {
    val->mutable_value(i) = i * M_PI;
    val->mutable_weight(i) = i;
    if (i % 2 == 1) {
      val->mutable_weight(i) *= -1;
      val->mutable_value(i) *= -1;
    }

    if (val->weight(i) < 0)
      val->mutable_constraint_type(i) = ConstraintType::Hard;
    else if (val->weight(i) > 0)
      val->mutable_constraint_type(i) = ConstraintType::Soft;
    else
      val->mutable_constraint_type(i) = ConstraintType::Skip;
  }
}

// Test equality of the given ConstraintValues and lcmt_constrained_values
// message.
static void TestConstrainedValuesMsg(const ConstrainedValues& val,
                                     const lcmt_constrained_values& msg) {
  EXPECT_EQ(msg.size, val.size());
  EXPECT_EQ(msg.types.size(), msg.weights.size());
  EXPECT_EQ(msg.types.size(), msg.values.size());
  EXPECT_EQ(static_cast<int>(msg.types.size()), msg.size);
  for (int i = 0; i < val.size(); ++i) {
    EXPECT_EQ(msg.types[i], EncodeConstraintType(val.constraint_type(i)));
  }
  TestEigenVectorAndStdVector(val.weights(), msg.weights);
  TestEigenVectorAndStdVector(val.values(), msg.values);

  ConstrainedValues decoded_val;
  DecodeConstrainedValues(msg, &decoded_val);
  EXPECT_EQ(val, decoded_val);
}

// Test equality of the given ContactInformation and lcmt_contact_information
// message.
static void TestEncodeContactInformation(const ContactInformation& info,
                                         const lcmt_contact_information& msg) {
  EXPECT_EQ(msg.body_name.compare(info.body_name()), 0);
  EXPECT_EQ(msg.num_contact_points, info.num_contact_points());
  EXPECT_EQ(msg.num_basis_per_contact_point,
            info.num_basis_per_contact_point());
  TestEigenMatrixAndStdVectorOfStdVector(info.contact_points(),
                                         msg.contact_points);
  TestEigenVectorAndCArray(info.normal(), msg.normal);
  EXPECT_DOUBLE_EQ(msg.mu, info.mu());
  EXPECT_DOUBLE_EQ(msg.Kd, info.Kd());
  EXPECT_DOUBLE_EQ(msg.weight, info.weight());
  EXPECT_EQ(msg.acceleration_constraint_type,
            EncodeConstraintType(info.acceleration_constraint_type()));
}

// Test equality of the given DesiredBodyMotion and lcmt_desired_body_motion
// message.
static void TestEncodeDesiredBodyMotion(const DesiredBodyMotion& mot,
                                        const lcmt_desired_body_motion& msg) {
  EXPECT_EQ(mot.body_name().compare(msg.body_name), 0);
  EXPECT_EQ(mot.control_during_contact(), msg.control_during_contact);
  TestConstrainedValuesMsg(mot, msg.constrained_accelerations);
}

// Test equality of the given DesiredDoFMotions and lcmt_desired_dof_motions
// message.
static void TestEncodeDesiredDoFMotions(const DesiredDoFMotions& mot,
                                        const lcmt_desired_dof_motions& msg) {
  EXPECT_EQ(msg.num_dof, mot.size());
  EXPECT_EQ(static_cast<int>(msg.dof_names.size()), mot.size());
  for (int i = 0; i < mot.size(); ++i) {
    EXPECT_EQ(msg.dof_names[i].compare(mot.dof_name(i)), 0);
  }
  TestConstrainedValuesMsg(mot, msg.constrained_accelerations);
}

// Test equality of the given QPInput and lcmt_qp_input message.
static void TestEncodeQPInput(const QPInput& qp_input,
                              const lcmt_qp_input& msg) {
  // Contacts
  EXPECT_EQ(msg.num_contacts,
            static_cast<int>(qp_input.contact_information().size()));
  EXPECT_EQ(msg.contact_information.size(),
            qp_input.contact_information().size());
  for (const auto& msg_contact : msg.contact_information) {
    auto it = qp_input.contact_information().find(msg_contact.body_name);
    EXPECT_TRUE(it != qp_input.contact_information().end());
    TestEncodeContactInformation(it->second, msg_contact);
  }
  // Body motions
  EXPECT_EQ(msg.num_desired_body_motions,
            static_cast<int>(qp_input.desired_body_motions().size()));
  EXPECT_EQ(msg.desired_body_motions.size(),
            qp_input.desired_body_motions().size());
  for (const auto& msg_mot : msg.desired_body_motions) {
    auto it = qp_input.desired_body_motions().find(msg_mot.body_name);
    EXPECT_TRUE(it != qp_input.desired_body_motions().end());
    TestEncodeDesiredBodyMotion(it->second, msg_mot);
  }
  // Dof motions
  TestEncodeDesiredDoFMotions(qp_input.desired_dof_motions(),
                              msg.desired_dof_motions);

  // Centroidal momentum
  TestConstrainedValuesMsg(
      qp_input.desired_centroidal_momentum_dot(),
      msg.desired_centroidal_momentum_dot.centroidal_momentum_dot);

  // Basis regularization weight
  EXPECT_DOUBLE_EQ(msg.w_basis_reg, qp_input.w_basis_reg());
}

// Test equality of the given ResolvedContact and lcmt_resolved_contact message.
static void TestResolvedContact(const ResolvedContact& contact,
                                const lcmt_resolved_contact& msg) {
  EXPECT_EQ(contact.body_name().compare(msg.body_name), 0);
  EXPECT_EQ(contact.basis().size(), msg.num_all_basis);
  EXPECT_EQ(contact.num_basis_per_contact_point(),
            msg.num_basis_per_contact_point);
  TestEigenVectorAndStdVector(contact.basis(), msg.basis);
  TestEigenMatrixAndStdVectorOfStdVector(contact.point_forces(),
                                         msg.point_forces);
  TestEigenMatrixAndStdVectorOfStdVector(contact.contact_points(),
                                         msg.contact_points);
  TestEigenVectorAndCArray(contact.equivalent_wrench(), msg.equivalent_wrench);
  TestEigenVectorAndCArray(contact.reference_point(), msg.reference_point);
}

// Test equality of the given BodyAcceleration and lcmt_body_acceleration
// message.
static void TestBodyAcceleration(const BodyAcceleration& acc,
                                 const lcmt_body_acceleration& msg) {
  EXPECT_EQ(acc.body_name().compare(msg.body_name), 0);
  TestEigenVectorAndCArray(acc.accelerations(), msg.accelerations);
}

// Test encoding and decoding of ResolvedContact <-> lcmt_resolved_contact.
GTEST_TEST(testLcmUtils, testEncodeDecodeResolvedContact) {
  ResolvedContact contact(*robot.FindBody("leftFoot"));
  contact.mutable_num_basis_per_contact_point() = 4;
  contact.mutable_basis().resize(4);
  contact.mutable_basis() << 0.1, 0.0, 0.2, 0.3;
  contact.mutable_point_forces().resize(3, 1);
  contact.mutable_point_forces() << 0.1, 0.2, 0.3;
  contact.mutable_contact_points().resize(3, 1);
  contact.mutable_contact_points() << -0.1, -0.2, -0.3;
  contact.mutable_equivalent_wrench() << 1, 2, 3, 4, 5, 6;
  contact.mutable_reference_point() << -1, -2, -3;

  lcmt_resolved_contact msg;
  EncodeResolvedContact(contact, &msg);
  TestResolvedContact(contact, msg);

  ResolvedContact decoded_contact(*robot.FindBody("world"));
  DecodeResolvedContact(robot, msg, &decoded_contact);
  EXPECT_EQ(decoded_contact, contact);
}

// Test encoding and decoding of ResolvedContact <-> lcmt_resolved_contact.
GTEST_TEST(testLcmUtils, testEncodeDecodeBodyAcceleration) {
  BodyAcceleration acc(*robot.FindBody("leftFoot"));
  acc.mutable_accelerations() << 1, 2, 3, 4, 5, 6;

  lcmt_body_acceleration msg;
  EncodeBodyAcceleration(acc, &msg);
  TestBodyAcceleration(acc, msg);

  BodyAcceleration decoded_acc(*robot.FindBody("world"));
  DecodeBodyAcceleration(robot, msg, &decoded_acc);
  EXPECT_EQ(decoded_acc, acc);
}

// Test encoding and decoding of ResolvedContact <-> lcmt_resolved_contact.
GTEST_TEST(testLcmUtils, testEncodeDecodeConstrainedValues) {
  ConstrainedValues val;
  SetConstrainedValues(&val, 7);

  // Test encode.
  lcmt_constrained_values msg;
  EncodeConstrainedValues(val, &msg);
  TestConstrainedValuesMsg(val, msg);

  // Test decode.
  ConstrainedValues decoded_val;
  DecodeConstrainedValues(msg, &decoded_val);
  EXPECT_EQ(decoded_val, val);
}

// Test encoding and decoding of ResolvedContact <-> lcmt_resolved_contact.
GTEST_TEST(testLcmUtils, testEncodeDecodeContactInformation) {
  ContactInformation info(*robot.FindBody("leftFoot"), 5);
  info.mutable_contact_points().resize(3, 2);
  info.mutable_contact_points().col(0) = Vector3<double>(0.3, -0.1, 1);
  info.mutable_contact_points().col(1) = Vector3<double>(-0.3, 0.1, -1);
  info.mutable_acceleration_constraint_type() = ConstraintType::Soft;
  info.mutable_weight() = M_PI;
  info.mutable_Kd() = 0.3;

  // Test encode.
  lcmt_contact_information msg;
  EncodeContactInformation(info, &msg);
  TestEncodeContactInformation(info, msg);

  // Test decode.
  ContactInformation decoded_info(*robot.FindBody("world"));
  DecodeContactInformation(robot, msg, &decoded_info);
  EXPECT_EQ(decoded_info, info);
}

// Test encoding and decoding of
// DesiredBodyMotion <-> lcmt_desired_body_motion.
GTEST_TEST(testLcmUtils, testEncodeDecodeDesiredBodyMotion) {
  DesiredBodyMotion mot(*robot.FindBody("leftFoot"));
  mot.mutable_control_during_contact() = true;
  SetConstrainedValues(&mot, mot.size());

  // Test encode.
  lcmt_desired_body_motion msg;
  EncodeDesiredBodyMotion(mot, &msg);
  TestEncodeDesiredBodyMotion(mot, msg);

  // Test decode
  DesiredBodyMotion decoded_mot(*robot.FindBody("world"));
  DecodeDesiredBodyMotion(robot, msg, &decoded_mot);
  EXPECT_EQ(decoded_mot, mot);
}

// Test encoding and decoding of
// DesiredDoFMotions <-> lcmt_desired_dof_motions.
GTEST_TEST(testLcmUtils, testEncodeDecodeDesiredDoFMotions) {
  DesiredDoFMotions mot({"a", "b", "c", "d"});
  SetConstrainedValues(&mot, mot.size());

  // Test encode.
  lcmt_desired_dof_motions msg;
  EncodeDesiredDoFMotions(mot, &msg);
  TestEncodeDesiredDoFMotions(mot, msg);

  // Test decode.
  DesiredDoFMotions decoded_mot;
  DecodeDesiredDoFMotions(msg, &decoded_mot);
  EXPECT_EQ(decoded_mot, mot);
}

// Test encoding and decoding of
// DesiredCentroidalMomentumDot <-> SetConstrainedValues.
GTEST_TEST(testLcmUtils, testEncodeDecodeDesiredCentroidalMomentumDot) {
  DesiredCentroidalMomentumDot Ld;
  SetConstrainedValues(&Ld, Ld.size());

  // Test encode.
  lcmt_desired_centroidal_momentum_dot msg;
  EncodeDesiredCentroidalMomentumDot(Ld, &msg);
  TestConstrainedValuesMsg(Ld, msg.centroidal_momentum_dot);

  // Test decode.
  DesiredCentroidalMomentumDot decoded_Ld;
  DecodeDesiredCentroidalMomentumDot(msg, &decoded_Ld);
  EXPECT_EQ(Ld, decoded_Ld);
}

// Test encoding and decoding of QPInput <-> lcmt_qp_input.
GTEST_TEST(testLcmUtils, testEncodeDecodeQPInput) {
  HumanoidStatus robot_status(robot);
  QPInput qp_input = MakeExampleQPInput(robot_status);

  // Test encode.
  lcmt_qp_input msg;
  EncodeQPInput(qp_input, &msg);
  TestEncodeQPInput(qp_input, msg);

  // Test decode.
  QPInput decoded_qp_input(robot);
  DecodeQPInput(robot, msg, &decoded_qp_input);
  EXPECT_EQ(qp_input, decoded_qp_input);
}

}  // namespace
}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
