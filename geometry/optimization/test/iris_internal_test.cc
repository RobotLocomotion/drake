#include "drake/geometry/optimization/iris_internal.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/optimization/test/iris_test_utilities.h"
#include "drake/planning/scene_graph_collision_checker.h"

namespace drake {
namespace geometry {
namespace optimization {
namespace internal {

const double kInf = std::numeric_limits<double>::infinity();

class A {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(A);
  A() {}
  virtual ~A() = default;
};

class B : public A {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(B);
  B() {}
  ~B() override = default;
};

TEST_F(Toy2DofRobotTest, InCollisionConstraint) {
  planning::CollisionCheckerParams params;
  const auto& plant = builder_.plant();
  params.model = builder_.Build();
  params.robot_model_instances.push_back(multibody::default_model_instance());
  params.configuration_distance_function = [](const Eigen::VectorXd& q1,
                                              const Eigen::VectorXd& q2) {
    return (q1 - q2).norm();
  };
  params.edge_step_size = 0.05;
  planning::SceneGraphCollisionChecker collision_checker(std::move(params));

  const double influence_distance = 2;
  const SortedPair<multibody::BodyIndex> body_pair(body_indices_[0],
                                                   plant.world_body().index());
  InCollisionConstraint dut(body_pair, collision_checker, influence_distance);
  EXPECT_EQ(dut.num_vars(), plant.num_positions());
  EXPECT_EQ(dut.num_constraints(), 1);
  EXPECT_TRUE(CompareMatrices(dut.upper_bound(), Vector1d(0)));
  EXPECT_TRUE(CompareMatrices(dut.lower_bound(), Vector1d(-kInf)));

  // Check Eval for double.
  Eigen::VectorXd q(2);
  q << 0.1, 0.2;
  Eigen::VectorXd y;
  dut.Eval(q, &y);
  EXPECT_EQ(y.rows(), 1);
  const planning::RobotClearance robot_clearance =
      collision_checker.CalcRobotClearance(q, influence_distance);

  double min_distance = kInf;
  Eigen::RowVectorXd dphi_dq;
  for (int i = 0; i < robot_clearance.size(); ++i) {
    if (SortedPair<multibody::BodyIndex>(robot_clearance.robot_indices()[i],
                                         robot_clearance.other_indices()[i]) ==
            body_pair &&
        robot_clearance.distances()(i) < min_distance) {
      min_distance = robot_clearance.distances()(i);
      dphi_dq = robot_clearance.jacobians().row(i);
    }
  }
  const double kTol = 1E-12;
  EXPECT_NEAR(min_distance, y(0), kTol);

  // Check Eval for autodiff.
  const Eigen::MatrixXd q_grad = Eigen::Vector2d(0.5, -2);
  const AutoDiffVecXd x_ad = math::InitializeAutoDiff(q, q_grad);
  AutoDiffVecXd y_ad;
  dut.Eval(x_ad, &y_ad);
  EXPECT_NEAR(y_ad(0).value(), y(0), kTol);
  EXPECT_TRUE(CompareMatrices(y_ad(0).derivatives(), dphi_dq * q_grad, kTol));

  // Construct with a pair of bodies between which the collision is ignored.
  // Expect an error.
  DRAKE_EXPECT_THROWS_MESSAGE(
      InCollisionConstraint(SortedPair(body_indices_[0], body_indices_[1]),
                            collision_checker, influence_distance),
      fmt::format(
          ".* collision between the body {} and {} is filtered",
          plant.get_body(SortedPair(body_indices_[0], body_indices_[1]).first())
              .name(),
          plant
              .get_body(SortedPair(body_indices_[0], body_indices_[1]).second())
              .name()));
}
}  // namespace internal
}  // namespace optimization
}  // namespace geometry
}  // namespace drake
