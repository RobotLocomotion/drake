#include "drake/multibody/plant/rational_forward_kinematics.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/symbolic_test_util.h"
#include "drake/multibody/plant/test/rational_forward_kinematics_test_utilities.h"

namespace drake {
namespace multibody {
namespace {
using math::RigidTransformd;
using math::RotationMatrixd;
using multibody::BodyIndex;
using multibody::ModelInstanceIndex;
using symbolic::Polynomial;
using symbolic::RationalFunction;

void CheckLinkKinematics(
    const RationalForwardKinematics& rational_forward_kinematics,
    const Eigen::Ref<const Eigen::VectorXd>& q_val,
    const Eigen::Ref<const Eigen::VectorXd>& q_star_val,
    const Eigen::Ref<const Eigen::VectorXd>& s_val,
    BodyIndex expressed_body_index) {
  DRAKE_DEMAND(s_val.rows() == rational_forward_kinematics.s().rows());
  auto context = rational_forward_kinematics.plant().CreateDefaultContext();

  rational_forward_kinematics.plant().SetPositions(context.get(), q_val);

  std::vector<math::RigidTransformd> X_WB_expected;

  const auto& tree =
      multibody::internal::GetInternalTree(rational_forward_kinematics.plant());
  tree.CalcAllBodyPosesInWorld(*context, &X_WB_expected);

  symbolic::Environment env;
  for (int i = 0; i < s_val.rows(); ++i) {
    env.insert(rational_forward_kinematics.s()(i), s_val(i));
  }

  const double tol{1E-12};
  for (int i = 0; i < rational_forward_kinematics.plant().num_bodies(); ++i) {
    const math::RigidTransformd X_AB_expected =
        X_WB_expected[expressed_body_index].inverse() * X_WB_expected[i];
    // Now check CalcLinkPoseAsMultilinearPolynomial, namely to compute a
    // single link pose.
    const RationalForwardKinematics::Pose<symbolic::Polynomial> pose_poly_i =
        rational_forward_kinematics.CalcLinkPoseAsMultilinearPolynomial(
            q_star_val, BodyIndex(i), expressed_body_index);
    EXPECT_EQ(pose_poly_i.frame_A_index, expressed_body_index);
    Eigen::Matrix3d R_AB_i;
    Eigen::Vector3d p_AB_i;
    for (int m = 0; m < 3; ++m) {
      for (int n = 0; n < 3; ++n) {
        const auto R_AB_i_mn_rational =
            rational_forward_kinematics
                .ConvertMultilinearPolynomialToRationalFunction(
                    pose_poly_i.R_AB(m, n));
        R_AB_i(m, n) = R_AB_i_mn_rational.Evaluate(env);
      }
      const auto p_AB_i_m_rational =
          rational_forward_kinematics
              .ConvertMultilinearPolynomialToRationalFunction(
                  pose_poly_i.p_AB(m));
      p_AB_i(m) = p_AB_i_m_rational.Evaluate(env);
    }
    EXPECT_TRUE(
        CompareMatrices(R_AB_i, X_AB_expected.rotation().matrix(), tol));
    EXPECT_TRUE(CompareMatrices(p_AB_i, X_AB_expected.translation(), tol));
  }
}

TEST_F(FinalizedIiwaTest, CalcLinkPoses) {
  RationalForwardKinematics rational_forward_kinematics(*iiwa_);
  EXPECT_EQ(rational_forward_kinematics.s().rows(), 7);

  // Call CalcLinkPoseAsMultilinearPolynomial to make sure the expressed link
  // index is correct in each pose.
  for (BodyIndex body_index{0};
       body_index < rational_forward_kinematics.plant().num_bodies();
       ++body_index) {
    const auto pose =
        rational_forward_kinematics.CalcLinkPoseAsMultilinearPolynomial(
            Eigen::VectorXd::Zero(7), body_index, iiwa_link_[2]);
    EXPECT_EQ(pose.frame_A_index, iiwa_link_[2]);
  }
  // q_val = 0 and q* = 0.
  CheckLinkKinematics(rational_forward_kinematics, Eigen::VectorXd::Zero(7),
                      Eigen::VectorXd::Zero(7), Eigen::VectorXd::Zero(7),
                      world_);
  // Compute the pose in the iiwa_link[i]'s frame.
  for (int i = 0; i < 8; ++i) {
    CheckLinkKinematics(rational_forward_kinematics, Eigen::VectorXd::Zero(7),
                        Eigen::VectorXd::Zero(7), Eigen::VectorXd::Zero(7),
                        iiwa_link_[i]);
  }

  // Non-zero q_val and zero q_star_val.
  Eigen::VectorXd q_val(7);
  // arbitrary value
  q_val << 0.2, 0.3, 0.5, -0.1, 1.2, 2.3, -0.5;
  Eigen::VectorXd s_val = (q_val / 2).array().tan().matrix();
  CheckLinkKinematics(rational_forward_kinematics, q_val,
                      Eigen::VectorXd::Zero(7), s_val, world_);
  // Compute the pose in the iiwa_link[i]'s frame.
  for (int i = 0; i < 8; ++i) {
    CheckLinkKinematics(rational_forward_kinematics, q_val,
                        Eigen::VectorXd::Zero(7), s_val, iiwa_link_[i]);
  }

  // Non-zero q_val and non-zero q_star_val.
  Eigen::VectorXd q_star_val(7);
  q_star_val << 1.2, -0.4, 0.3, -0.5, 0.4, 1, 0.2;
  s_val = ((q_val - q_star_val) / 2).array().tan().matrix();
  CheckLinkKinematics(rational_forward_kinematics, q_val, q_star_val, s_val,
                      world_);
  // Compute the pose in the iiwa_link[i]'s frame.
  for (int i = 0; i < 8; ++i) {
    CheckLinkKinematics(rational_forward_kinematics, q_val, q_star_val, s_val,
                        iiwa_link_[i]);
  }
}

GTEST_TEST(RationalForwardKinematicsTest, CalcLinkPosesForDualArmIiwa) {
  RigidTransformd X_WL{};
  RigidTransformd X_WR{
      RotationMatrixd(Eigen::AngleAxisd(0.2 * M_PI, Eigen::Vector3d::UnitZ())),
      {0.2, 0.8, 0.1}};
  ModelInstanceIndex left_iiwa_instance, right_iiwa_instance;
  auto iiwa_plant =
      ConstructDualArmIiwaPlant("iiwa14_no_collision.sdf", X_WL, X_WR,
                                &left_iiwa_instance, &right_iiwa_instance);

  RationalForwardKinematics rational_forward_kinematics(*iiwa_plant);
  EXPECT_EQ(rational_forward_kinematics.s().size(), 14);

  Eigen::VectorXd q_star_val(14);
  q_star_val.setZero();
  Eigen::VectorXd q_val(14);
  q_val.setZero();
  Eigen::VectorXd s_val(14);
  s_val.setZero();

  auto set_s_val = [&](const Eigen::VectorXd& q_left,
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

    s_val = rational_forward_kinematics.ComputeSValue(q_val, q_star_val);
  };

  const BodyIndex world_index = iiwa_plant->world_body().index();
  CheckLinkKinematics(rational_forward_kinematics, q_val, q_star_val, s_val,
                      world_index);

  Eigen::VectorXd q_left(7);
  Eigen::VectorXd q_right(7);
  Eigen::VectorXd q_left_star(7);
  Eigen::VectorXd q_right_star(7);
  q_left_star.setZero();
  q_right_star.setZero();
  q_left << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
  q_right << -0.1, -0.2, -0.3, -0.4, -0.5, -0.6, -0.7;
  set_s_val(q_left, q_right, q_left_star, q_right_star);
  CheckLinkKinematics(rational_forward_kinematics, q_val, q_star_val, s_val,
                      world_index);

  q_left_star << -0.2, 1.2, 0.3, 0.4, -2.1, 2.2, 2.3;
  q_right_star << 0.1, 0.2, 0.5, 1.1, -0.3, -0.2, 2.1;
  set_s_val(q_left, q_right, q_left_star, q_right_star);
  CheckLinkKinematics(rational_forward_kinematics, q_val, q_star_val, s_val,
                      world_index);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
