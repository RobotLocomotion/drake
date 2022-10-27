#include "drake/multibody/rational/rational_forward_kinematics.h"

#include <gtest/gtest.h>

#include "drake/common/symbolic/rational_function.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/symbolic_test_util.h"
#include "drake/multibody/rational/test/rational_forward_kinematics_test_utilities.h"
#include "drake/multibody/tree/multibody_tree.h"

namespace drake {
namespace multibody {
namespace {
using math::RigidTransformd;
using math::RotationMatrixd;
using multibody::BodyIndex;
using multibody::ModelInstanceIndex;
using symbolic::Polynomial;
using symbolic::RationalFunction;

void CheckBodyKinematics(const RationalForwardKinematics& dut,
                         const Eigen::Ref<const Eigen::VectorXd>& q_val,
                         const Eigen::Ref<const Eigen::VectorXd>& q_star_val,
                         const Eigen::Ref<const Eigen::VectorXd>& s_val,
                         BodyIndex expressed_body_index) {
  DRAKE_DEMAND(s_val.rows() == dut.s().rows());
  auto context = dut.plant().CreateDefaultContext();

  dut.plant().SetPositions(context.get(), q_val);

  std::vector<math::RigidTransformd> X_WB_expected;

  const internal::MultibodyTree<double>& tree =
      internal::GetInternalTree(dut.plant());
  tree.CalcAllBodyPosesInWorld(*context, &X_WB_expected);

  symbolic::Environment env;
  for (int i = 0; i < s_val.rows(); ++i) {
    env.insert(dut.s()(i), s_val(i));
  }

  const double kTol{1E-12};
  for (int i = 0; i < dut.plant().num_bodies(); ++i) {
    const math::RigidTransformd X_AB_expected =
        X_WB_expected[expressed_body_index].inverse() * X_WB_expected[i];
    // Now check CalcBodyPoseAsMultilinearPolynomial, namely to compute a
    // single body pose.
    const RationalForwardKinematics::Pose<symbolic::Polynomial> pose_poly_i =
        dut.CalcBodyPoseAsMultilinearPolynomial(q_star_val, BodyIndex(i),
                                                expressed_body_index);
    Eigen::Matrix3d R_AB_i;
    Eigen::Vector3d p_AB_i;
    for (int m = 0; m < 3; ++m) {
      for (int n = 0; n < 3; ++n) {
        const symbolic::RationalFunction R_AB_i_mn_rational =
            dut.ConvertMultilinearPolynomialToRationalFunction(
                pose_poly_i.rotation(m, n));
        R_AB_i(m, n) = R_AB_i_mn_rational.Evaluate(env);
      }
      const symbolic::RationalFunction p_AB_i_m_rational =
          dut.ConvertMultilinearPolynomialToRationalFunction(
              pose_poly_i.position(m));
      p_AB_i(m) = p_AB_i_m_rational.Evaluate(env);
    }
    EXPECT_TRUE(
        CompareMatrices(R_AB_i, X_AB_expected.rotation().matrix(), kTol));
    EXPECT_TRUE(CompareMatrices(p_AB_i, X_AB_expected.translation(), kTol));
  }
}

TEST_F(FinalizedIiwaTest, CalcBodyPose1) {
  // I split the FinalizedIiwaTest::CalcBodyPose into smaller tests, and run the
  // tests with shard_count >= 1, so that the test takes less time with asan
  // being active.
  //
  // Test with q_val = 0 and q* = 0.
  RationalForwardKinematics dut(iiwa_.get());
  constexpr int kNumJoints = 7;
  EXPECT_EQ(dut.s().rows(), kNumJoints);

  CheckBodyKinematics(dut, Eigen::VectorXd::Zero(kNumJoints),
                      Eigen::VectorXd::Zero(kNumJoints),
                      Eigen::VectorXd::Zero(kNumJoints), world_);
  // Compute the pose in the iiwa_link[i]'s frame.
  for (int i = 0; i < 8; ++i) {
    CheckBodyKinematics(dut, Eigen::VectorXd::Zero(kNumJoints),
                        Eigen::VectorXd::Zero(kNumJoints),
                        Eigen::VectorXd::Zero(kNumJoints), iiwa_link_[i]);
  }
}

TEST_F(FinalizedIiwaTest, CalcBodyPose2) {
  // Test with non-zero q_val and q_star_val = 0.
  RationalForwardKinematics dut(iiwa_.get());
  constexpr int kNumJoints = 7;
  EXPECT_EQ(dut.s().rows(), kNumJoints);
  Eigen::VectorXd q_val(kNumJoints);
  // arbitrary value
  q_val << 0.2, 0.3, 0.5, -0.1, 1.2, 2.3, -0.5;
  Eigen::VectorXd s_val = (q_val / 2).array().tan().matrix();
  CheckBodyKinematics(dut, q_val, Eigen::VectorXd::Zero(kNumJoints), s_val,
                      world_);
  // Compute the pose in the iiwa_link[i]'s frame.
  for (int i = 0; i < 8; ++i) {
    CheckBodyKinematics(dut, q_val, Eigen::VectorXd::Zero(kNumJoints), s_val,
                        iiwa_link_[i]);
  }
}

TEST_F(FinalizedIiwaTest, CalcBodyPose3) {
  // Test with both q_val and q_star_val being non-zero.
  RationalForwardKinematics dut(iiwa_.get());
  constexpr int kNumJoints = 7;
  EXPECT_EQ(dut.s().rows(), kNumJoints);
  Eigen::VectorXd q_val(kNumJoints);
  Eigen::VectorXd q_star_val(kNumJoints);
  q_val << 0.2, 0.3, 0.5, -0.1, 1.2, 2.3, -0.5;
  q_star_val << 1.2, -0.4, 0.3, -0.5, 0.4, 1, 0.2;
  const Eigen::VectorXd s_val = dut.ComputeSValue(q_val, q_star_val);
  CheckBodyKinematics(dut, q_val, q_star_val, s_val, world_);
  // Compute the pose in the iiwa_link[i]'s frame.
  for (int i = 0; i < 8; ++i) {
    CheckBodyKinematics(dut, q_val, q_star_val, s_val, iiwa_link_[i]);
  }
}

GTEST_TEST(RationalForwardKinematicsTest, CalcBodyPosesForDualArmIiwa) {
  RigidTransformd X_WL{};
  RigidTransformd X_WR{
      RotationMatrixd(Eigen::AngleAxisd(0.2 * M_PI, Eigen::Vector3d::UnitZ())),
      {0.2, 0.8, 0.1}};
  ModelInstanceIndex left_iiwa_instance, right_iiwa_instance;
  auto iiwa_plant =
      ConstructDualArmIiwaPlant("iiwa14_no_collision.sdf", X_WL, X_WR,
                                &left_iiwa_instance, &right_iiwa_instance);

  RationalForwardKinematics dut(iiwa_plant.get());
  EXPECT_EQ(dut.s().size(), 14);

  Eigen::VectorXd q_star_val = Eigen::VectorXd::Zero(14);
  Eigen::VectorXd q_val = Eigen::VectorXd::Zero(14);

  auto calc_s_val = [&](const Eigen::VectorXd& q_left,
                        const Eigen::VectorXd& q_right,
                        const Eigen::VectorXd& q_left_star,
                        const Eigen::VectorXd& q_right_star) {
    auto context = iiwa_plant->CreateDefaultContext();
    iiwa_plant->SetPositions(context.get(), left_iiwa_instance, q_left);
    iiwa_plant->SetPositions(context.get(), right_iiwa_instance, q_right);
    q_val = iiwa_plant->GetPositions(*context);
    iiwa_plant->SetPositions(context.get(), left_iiwa_instance, q_left_star);
    iiwa_plant->SetPositions(context.get(), right_iiwa_instance, q_right_star);
    q_star_val = iiwa_plant->GetPositions(*context);

    Eigen::VectorXd s_val = dut.ComputeSValue(q_val, q_star_val);
    return s_val;
  };

  const BodyIndex world_index = iiwa_plant->world_body().index();
  Eigen::VectorXd s_val = Eigen::VectorXd::Zero(14);
  CheckBodyKinematics(dut, q_val, q_star_val, s_val, world_index);

  Eigen::VectorXd q_left(7);
  Eigen::VectorXd q_right(7);
  Eigen::VectorXd q_left_star(7);
  Eigen::VectorXd q_right_star(7);
  q_left_star.setZero();
  q_right_star.setZero();
  q_left << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
  q_right << -0.1, -0.2, -0.3, -0.4, -0.5, -0.6, -0.7;
  s_val = calc_s_val(q_left, q_right, q_left_star, q_right_star);
  CheckBodyKinematics(dut, q_val, q_star_val, s_val, world_index);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
