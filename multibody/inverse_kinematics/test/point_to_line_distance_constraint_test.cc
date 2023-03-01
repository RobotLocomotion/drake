#include "drake/multibody/inverse_kinematics/point_to_line_distance_constraint.h"

#include <gtest/gtest.h>

#include "drake/multibody/inverse_kinematics/test/inverse_kinematics_test_utilities.h"

using drake::systems::Context;

namespace drake {
namespace multibody {
template <typename T>
void TestIiwa(const MultibodyPlant<T>* plant,
              systems::Context<T>* plant_context,
              const MultibodyPlant<AutoDiffXd>* plant_autodiff,
              systems::Context<AutoDiffXd>* plant_context_autodiff) {
  const Eigen::Vector3d p_B1P(0.1, 0.2, 0.3);
  const Eigen::Vector3d p_B2Q(0.2, 0.4, -0.5);
  const Eigen::Vector3d n_B2(0.5, -0.1, 0.2);

  const auto frame_point_index = plant->GetFrameByName("iiwa_link_7").index();
  const auto frame_line_index = plant->GetFrameByName("iiwa_link_2").index();
  const Frame<T>& frame_point = plant->get_frame(frame_point_index);
  const Frame<T>& frame_line = plant->get_frame(frame_line_index);

  const double distance_lower = 0.1;
  const double distance_upper = 0.2;

  PointToLineDistanceConstraint constraint(
      plant, frame_point, p_B1P, frame_line, p_B2Q, n_B2, distance_lower,
      distance_upper, plant_context);
  EXPECT_EQ(constraint.num_vars(), plant->num_positions());
  EXPECT_EQ(constraint.num_constraints(), 1);
  EXPECT_EQ(constraint.lower_bound()(0), distance_lower * distance_lower);
  EXPECT_EQ(constraint.upper_bound()(0), distance_upper * distance_upper);

  // Now check if Eval function computes the right result.
  Eigen::VectorXd q(7);
  q << 0.1, 0.2, 0.3, 0.4, -0.1, -0.2, -0.4;
  Eigen::VectorXd y;
  constraint.Eval(q, &y);

  Vector3<T> p_WP, p_WQ, n_W;
  plant->SetPositions(plant_context, q.cast<T>());
  plant->CalcPointsPositions(*plant_context, frame_point, p_B1P.cast<T>(),
                             plant->world_frame(), &p_WP);
  const auto X_WB2 = plant->CalcRelativeTransform(
      *plant_context, plant->world_frame(), frame_line);
  p_WQ = X_WB2 * p_B2Q.cast<T>();
  n_W = X_WB2.rotation() * n_B2.cast<T>();
  const Vector3<T> n_W_normalized = n_W.normalized();

  // Project P onto the line to P_perp.
  const Vector3<T> p_WP_perp =
      (p_WP - p_WQ).dot(n_W_normalized) * n_W_normalized + p_WQ;
  const T distance_squared = (p_WP - p_WP_perp).squaredNorm();
  if constexpr (std::is_same_v<T, double>) {
    EXPECT_NEAR(y(0), distance_squared, 1E-12);
  } else if constexpr (std::is_same_v<T, AutoDiffXd>) {
    EXPECT_NEAR(y(0), distance_squared.value(), 1E-12);
  }

  // Check the evaluation gradient.
  Eigen::MatrixXd dqdz(7, 2);
  dqdz << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1, 1.1, 1.2, 1.3, 1.4;
  VectorX<AutoDiffXd> q_autodiff = math::InitializeAutoDiff(q, dqdz);
  AutoDiffVecXd y_autodiff;
  constraint.Eval(q_autodiff, &y_autodiff);
  plant_autodiff->SetPositions(plant_context_autodiff, q_autodiff);
  Vector3<AutoDiffXd> p_B2P_ad;
  plant_autodiff->CalcPointsPositions(
      *plant_context_autodiff, plant_autodiff->get_frame(frame_point_index),
      p_B1P.cast<AutoDiffXd>(), plant_autodiff->get_frame(frame_line_index),
      &p_B2P_ad);
  const Vector3<AutoDiffXd> p_QP_B2_ad = p_B2P_ad - p_B2Q.cast<AutoDiffXd>();
  const Vector3<AutoDiffXd> n_B2_normalized_ad =
      n_B2.normalized().cast<AutoDiffXd>();
  const AutoDiffXd distance_squared_ad =
      (p_QP_B2_ad.dot(n_B2_normalized_ad) * n_B2_normalized_ad +
       p_B2Q.cast<AutoDiffXd>() - p_B2P_ad)
          .squaredNorm();
  CompareAutoDiffVectors(y_autodiff, Vector1<AutoDiffXd>(distance_squared_ad),
                         1E-12);
}

TEST_F(IiwaKinematicConstraintTest, PointToLineDistanceConstraint) {
  // Test PointToLineDistanceConstraint constructed with MBP<double>
  TestIiwa(plant_, plant_context_, plant_autodiff_.get(),
           plant_context_autodiff_.get());
  // Test PointToLineDistanceConstraint constructed with MBP<AutoDiffXd>
  TestIiwa(plant_autodiff_.get(), plant_context_autodiff_.get(),
           plant_autodiff_.get(), plant_context_autodiff_.get());
}
}  // namespace multibody
}  // namespace drake
