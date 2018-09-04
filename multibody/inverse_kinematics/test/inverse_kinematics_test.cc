#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/inverse_kinematics/test/inverse_kinematics_test_utilities.h"
#include "drake/solvers/create_constraint.h"

namespace drake {
namespace multibody {
Eigen::Quaterniond Vector4ToQuaternion(
    const Eigen::Ref<const Eigen::Vector4d>& q) {
  return Eigen::Quaterniond(q(0), q(1), q(2), q(3));
}

class TwoFreeBodiesTest : public ::testing::Test {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TwoFreeBodiesTest)

  TwoFreeBodiesTest()
      : two_bodies_(test::ConstructTwoFreeBodies<AutoDiffXd>()),
        body1_index_(two_bodies_->GetBodyByName("body1").body_frame().index()),
        body2_index_(two_bodies_->GetBodyByName("body2").body_frame().index()),
        ik_(*two_bodies_) {
    // TODO(hongkai.dai): The unit quaternion constraint should be added by
    // InverseKinematics automatically.
    ik_.AddConstraint(solvers::internal::ParseQuadraticConstraint(
        ik_.q().head<4>().cast<symbolic::Expression>().squaredNorm(), 1, 1));
    ik_.AddConstraint(solvers::internal::ParseQuadraticConstraint(
        ik_.q().segment<4>(7).cast<symbolic::Expression>().squaredNorm(), 1,
        1));
  }

  ~TwoFreeBodiesTest() override {}

  void RetrieveSolution() {
    const auto q_sol = ik_.GetSolution(ik_.q());
    body1_quaternion_sol_ = Vector4ToQuaternion(q_sol.head<4>());
    body1_position_sol_ = q_sol.segment<3>(4);
    body2_quaternion_sol_ = Vector4ToQuaternion(q_sol.segment<4>(7));
    body2_position_sol_ = q_sol.tail<3>();
  }

 protected:
  std::unique_ptr<MultibodyTree<AutoDiffXd>> two_bodies_;
  FrameIndex body1_index_;
  FrameIndex body2_index_;
  InverseKinematics ik_;

  Eigen::Quaterniond body1_quaternion_sol_;
  Eigen::Quaterniond body2_quaternion_sol_;
  Eigen::Vector3d body1_position_sol_;
  Eigen::Vector3d body2_position_sol_;
};

TEST_F(TwoFreeBodiesTest, PositionConstraint) {
  const Eigen::Vector3d p_BQ(0.2, 0.3, 0.5);
  const Eigen::Vector3d p_AQ_lower(-0.1, -0.2, -0.3);
  const Eigen::Vector3d p_AQ_upper(-0.05, -0.12, -0.28);
  ik_.AddPositionConstraint(body1_index_, p_BQ, body2_index_, p_AQ_lower,
                            p_AQ_upper);

  ik_.SetInitialGuess(ik_.q().head<4>(), Eigen::Vector4d(1, 0, 0, 0));
  ik_.SetInitialGuess(ik_.q().segment<4>(7), Eigen::Vector4d(1, 0, 0, 0));
  const auto result = ik_.Solve();
  EXPECT_EQ(result, solvers::SolutionResult::kSolutionFound);
  RetrieveSolution();
  const Eigen::Vector3d p_AQ = body2_quaternion_sol_.inverse() *
                               (body1_quaternion_sol_ * p_BQ +
                                body1_position_sol_ - body2_position_sol_);
  const double tol = 1E-6;
  EXPECT_TRUE(
      (p_AQ.array() <= p_AQ_upper.array() + Eigen::Array3d::Constant(tol))
          .all());
  EXPECT_TRUE(
      (p_AQ.array() >= p_AQ_lower.array() - Eigen::Array3d::Constant(tol))
          .all());
}

TEST_F(TwoFreeBodiesTest, OrientationConstraint) {
  const double angle_bound = 0.05 * M_PI;

  ik_.AddOrientationConstraint(body1_index_, body2_index_, angle_bound);

  ik_.SetInitialGuess(ik_.q().head<4>(), Eigen::Vector4d(1, 0, 0, 0));
  ik_.SetInitialGuess(ik_.q().segment<4>(7), Eigen::Vector4d(1, 0, 0, 0));
  const auto result = ik_.Solve();
  EXPECT_EQ(result, solvers::SolutionResult::kSolutionFound);
  const auto q_sol = ik_.GetSolution(ik_.q());
  RetrieveSolution();
  const double angle =
      Eigen::AngleAxisd(body1_quaternion_sol_.inverse() * body2_quaternion_sol_)
          .angle();
  EXPECT_LE(angle, angle_bound + 1E-6);
}

TEST_F(TwoFreeBodiesTest, GazeTargetConstraint) {
  const Eigen::Vector3d p_AS(0.01, 0.2, 0.4);
  const Eigen::Vector3d n_A(0.2, 0.4, -0.1);
  const Eigen::Vector3d p_BT(0.4, -0.2, 1.5);
  const double cone_half_angle{0.2 * M_PI};

  ik_.AddGazeTargetConstraint(body1_index_, p_AS, n_A, body2_index_, p_BT,
                              cone_half_angle);

  ik_.SetInitialGuess(ik_.q().head<4>(), Eigen::Vector4d(1, 0, 0, 0));
  ik_.SetInitialGuess(ik_.q().segment<4>(7), Eigen::Vector4d(1, 0, 0, 0));
  const auto result = ik_.Solve();
  EXPECT_EQ(result, solvers::SolutionResult::kSolutionFound);

  RetrieveSolution();
  const Eigen::Vector3d p_WS =
      body1_quaternion_sol_ * p_AS + body1_position_sol_;
  const Eigen::Vector3d p_WT =
      body2_quaternion_sol_ * p_BT + body2_position_sol_;
  const Eigen::Vector3d p_ST_A =
      body1_quaternion_sol_.inverse() * (p_WT - p_WS);
  const double angle =
      std::acos(p_ST_A.dot(n_A) / (p_ST_A.norm() * n_A.norm()));
  EXPECT_LE(angle, cone_half_angle + 1E-6);
}

TEST_F(TwoFreeBodiesTest, AngleBetweenVectorsConstraint) {
  const Eigen::Vector3d n_A(0.2, -0.4, 0.9);
  const Eigen::Vector3d n_B(1.4, -0.1, 1.8);

  const double angle_lower{0.2 * M_PI};
  const double angle_upper{0.2 * M_PI};

  ik_.AddAngleBetweenVectorsConstraint(body1_index_, n_A, body2_index_, n_B,
                                       angle_lower, angle_upper);
  ik_.SetInitialGuess(ik_.q().head<4>(), Eigen::Vector4d(1, 0, 0, 0));
  ik_.SetInitialGuess(ik_.q().segment<4>(7), Eigen::Vector4d(1, 0, 0, 0));
  const auto result = ik_.Solve();
  EXPECT_EQ(result, solvers::SolutionResult::kSolutionFound);

  RetrieveSolution();

  const Eigen::Vector3d n_A_W = body1_quaternion_sol_ * n_A;
  const Eigen::Vector3d n_B_W = body2_quaternion_sol_ * n_B;

  const double angle =
      std::acos(n_A_W.dot(n_B_W) / (n_A_W.norm() * n_B_W.norm()));
  EXPECT_NEAR(angle, angle_lower, 1E-6);
}
}  // namespace multibody
}  // namespace drake
