#include "drake/examples/QPInverseDynamicsForHumanoids/lcm_utils.h"

#include <cstdlib>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {
namespace {

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
                              const Scalar (&array)[Size]) {
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

// Test equality of the given DesiredDofMotions and lcmt_desired_dof_motions
// message.
static void TestEncodeDesiredDofMotions(const DesiredDofMotions& mot,
                                        const lcmt_desired_dof_motions& msg) {
  EXPECT_EQ(msg.num_dof, mot.size());
  EXPECT_EQ(static_cast<int>(msg.dof_names.size()), mot.size());
  for (int i = 0; i < mot.size(); ++i) {
    EXPECT_EQ(msg.dof_names[i].compare(mot.dof_name(i)), 0);
  }
  TestConstrainedValuesMsg(mot, msg.constrained_accelerations);
}

// Test equality of the given QpInput and lcmt_qp_input message.
static void TestEncodeQpInput(const QpInput& qp_input,
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
  TestEncodeDesiredDofMotions(qp_input.desired_dof_motions(),
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

class LcmUtilsTests : public ::testing::Test {
 protected:
  virtual void SetUp() {
    tree_ = std::make_unique<RigidBodyTree<double>>();
    parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
        GetDrakePath() +
            "/examples/Valkyrie/urdf/urdf/"
            "valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf",
        multibody::joints::kRollPitchYaw, tree_.get());
  }

  std::unique_ptr<RigidBodyTree<double>> tree_;
};

// Test encoding and decoding of ResolvedContact <-> lcmt_resolved_contact.
TEST_F(LcmUtilsTests, TestEncodeDecodeResolvedContact) {
  ResolvedContact contact(*tree_->FindBody("leftFoot"));
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

  ResolvedContact decoded_contact(*tree_->FindBody("world"));
  DecodeResolvedContact(*tree_, msg, &decoded_contact);
  EXPECT_EQ(decoded_contact, contact);
}

// Test encoding and decoding of ResolvedContact <-> lcmt_resolved_contact.
TEST_F(LcmUtilsTests, TestEncodeDecodeBodyAcceleration) {
  BodyAcceleration acc(*tree_->FindBody("leftFoot"));
  acc.mutable_accelerations() << 1, 2, 3, 4, 5, 6;

  lcmt_body_acceleration msg;
  EncodeBodyAcceleration(acc, &msg);
  TestBodyAcceleration(acc, msg);

  BodyAcceleration decoded_acc(*tree_->FindBody("world"));
  DecodeBodyAcceleration(*tree_, msg, &decoded_acc);
  EXPECT_EQ(decoded_acc, acc);
}

// Test encoding and decoding of ResolvedContact <-> lcmt_resolved_contact.
TEST_F(LcmUtilsTests, TestEncodeDecodeConstrainedValues) {
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
TEST_F(LcmUtilsTests, TestEncodeDecodeContactInformation) {
  ContactInformation info(*tree_->FindBody("leftFoot"), 5);
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
  ContactInformation decoded_info(*tree_->FindBody("world"));
  DecodeContactInformation(*tree_, msg, &decoded_info);
  EXPECT_EQ(decoded_info, info);
}

// Test encoding and decoding of
// DesiredBodyMotion <-> lcmt_desired_body_motion.
TEST_F(LcmUtilsTests, TestEncodeDecodeDesiredBodyMotion) {
  DesiredBodyMotion mot(*tree_->FindBody("leftFoot"));
  mot.mutable_control_during_contact() = true;
  SetConstrainedValues(&mot, mot.size());

  // Test encode.
  lcmt_desired_body_motion msg;
  EncodeDesiredBodyMotion(mot, &msg);
  TestEncodeDesiredBodyMotion(mot, msg);

  // Test decode
  DesiredBodyMotion decoded_mot(*tree_->FindBody("world"));
  DecodeDesiredBodyMotion(*tree_, msg, &decoded_mot);
  EXPECT_EQ(decoded_mot, mot);
}

// Test encoding and decoding of
// DesiredDofMotions <-> lcmt_desired_dof_motions.
TEST_F(LcmUtilsTests, TestEncodeDecodeDesiredDofMotions) {
  DesiredDofMotions mot({"a", "b", "c", "d"});
  SetConstrainedValues(&mot, mot.size());

  // Test encode.
  lcmt_desired_dof_motions msg;
  EncodeDesiredDofMotions(mot, &msg);
  TestEncodeDesiredDofMotions(mot, msg);

  // Test decode.
  DesiredDofMotions decoded_mot;
  DecodeDesiredDofMotions(msg, &decoded_mot);
  EXPECT_EQ(decoded_mot, mot);
}

// Test encoding and decoding of
// DesiredCentroidalMomentumDot <-> SetConstrainedValues.
TEST_F(LcmUtilsTests, TestEncodeDecodeDesiredCentroidalMomentumDot) {
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

// Test encoding and decoding of QpInput <-> lcmt_qp_input.
TEST_F(LcmUtilsTests, TestEncodeDecodeQpInput) {
  // Initialize QP input
  QpInput qp_input(GetDofNames(*tree_));
  ContactInformation contact(*tree_->FindBody("leftFoot"), 3);
  contact.mutable_contact_points() = Vector3<double>(1, 2, 3);
  contact.mutable_mu() = 0.2;
  contact.mutable_weight() = -1;
  contact.mutable_acceleration_constraint_type() = ConstraintType::Hard;
  contact.mutable_Kd() = 1;
  contact.mutable_normal() = Vector3<double>(3, 2, 1).normalized();
  qp_input.mutable_contact_information().emplace("leftFoot", contact);

  DesiredBodyMotion pelv_motion(*tree_->FindBody("pelvis"));
  pelv_motion.mutable_weights() << 1, 0, -1, 20, 0, -99;
  pelv_motion.mutable_values() << 1, 2, 3, 4, 5, 6;
  pelv_motion.SetAllConstraintTypesBasedOnWeights();
  qp_input.mutable_desired_body_motions().emplace("pelvis", pelv_motion);

  for (int i = 0; i < tree_->get_num_velocities(); ++i) {
    qp_input.mutable_desired_dof_motions().mutable_weights()[i] = i - 10;
    qp_input.mutable_desired_dof_motions().mutable_values()[i] = i;
  }
  qp_input.mutable_desired_dof_motions().SetAllConstraintTypesBasedOnWeights();

  qp_input.mutable_desired_centroidal_momentum_dot().mutable_weights() << -1, 0,
      1, 2, 3, 4;
  qp_input.mutable_desired_centroidal_momentum_dot().mutable_values() << -3, -2,
      -1, 0, 1, 2;
  qp_input.mutable_desired_centroidal_momentum_dot()
      .SetAllConstraintTypesBasedOnWeights();

  qp_input.mutable_w_basis_reg() = 1e-3;

  // Test encode.
  lcmt_qp_input msg;
  EncodeQpInput(qp_input, &msg);
  TestEncodeQpInput(qp_input, msg);

  // Test decode.
  QpInput decoded_qp_input;
  DecodeQpInput(*tree_, msg, &decoded_qp_input);
  EXPECT_EQ(qp_input, decoded_qp_input);
}

}  // namespace
}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
