#include "drake/geometry/optimization/cartesian_product.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/point.h"
#include "drake/geometry/optimization/test_utilities.h"
#include "drake/geometry/optimization/vpolytope.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/rigid_transform.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace geometry {
namespace optimization {

using drake::Vector1d;
using Eigen::Matrix;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using math::RigidTransformd;

GTEST_TEST(CartesianProductTest, BasicTest) {
  const Point P1(Vector2d{1.2, 3.4}), P2(Vector2d{5.6, 7.8});
  const CartesianProduct S(P1, P2);
  EXPECT_EQ(S.num_factors(), 2);
  EXPECT_EQ(S.ambient_dimension(), 4);

  // Test PointInSet.
  Vector4d in, out;
  in << P1.x(), P2.x();
  out << P1.x(), P2.x() + Vector2d::Constant(0.01);
  EXPECT_TRUE(S.PointInSet(in));
  EXPECT_FALSE(S.PointInSet(out));

  EXPECT_TRUE(internal::CheckAddPointInSetConstraints(S, in));
  EXPECT_FALSE(internal::CheckAddPointInSetConstraints(S, out));
  {
    // Test the new variables added in AddPointInSetConstraints
    solvers::MathematicalProgram prog;
    auto x = prog.NewContinuousVariables(4);
    auto [new_vars, new_constraints] = S.AddPointInSetConstraints(&prog, x);
    EXPECT_EQ(new_vars.rows(), 0);
  }

  // Test MaybeGetPoint.
  ASSERT_TRUE(S.MaybeGetPoint().has_value());
  EXPECT_TRUE(CompareMatrices(S.MaybeGetPoint().value(), in));

  // Test IsBounded.
  EXPECT_TRUE(S.IsBounded());

  // Test ConvexSets constructor.
  ConvexSets sets;
  sets.emplace_back(P1);
  sets.emplace_back(P2);
  const CartesianProduct S2(sets);
  EXPECT_EQ(S2.num_factors(), 2);
  EXPECT_EQ(S2.ambient_dimension(), 4);
  EXPECT_TRUE(S2.PointInSet(in));
  EXPECT_FALSE(S2.PointInSet(out));
}

GTEST_TEST(CartesianProductTest, DefaultCtor) {
  const CartesianProduct dut;
  EXPECT_EQ(dut.num_factors(), 0);
  EXPECT_NO_THROW(dut.Clone());
  EXPECT_EQ(dut.ambient_dimension(), 0);
  EXPECT_FALSE(dut.IntersectsWith(dut));
  EXPECT_TRUE(dut.IsBounded());
  EXPECT_FALSE(dut.MaybeGetPoint().has_value());
  EXPECT_FALSE(dut.PointInSet(Eigen::VectorXd::Zero(0)));
}

GTEST_TEST(CartesianProductTest, Move) {
  const Point P1(Vector2d{1.2, 3.4}), P2(Vector2d{5.6, 7.8});
  CartesianProduct orig(P1, P2);

  // A move-constructed CartesianProduct takes over the original data.
  CartesianProduct dut(std::move(orig));
  EXPECT_EQ(dut.num_factors(), 2);
  EXPECT_EQ(dut.ambient_dimension(), 4);

  // The old set is in a valid but unspecified state. For convenience we'll
  // assert that it's empty, but that's not the only valid implementation,
  // just the one we happen to currently use.
  EXPECT_EQ(orig.num_factors(), 0);
  EXPECT_EQ(orig.ambient_dimension(), 0);
  EXPECT_NO_THROW(orig.Clone());
}

GTEST_TEST(CartesianProductTest, FromSceneGraph) {
  const RigidTransformd X_WG{math::RollPitchYawd(.1, .2, 3),
                             Vector3d{.5, .87, .1}};

  // Test SceneGraph constructor.
  const double kRadius = 0.2;
  const double kLength = 0.5;
  auto [scene_graph, geom_id] =
      internal::MakeSceneGraphWithShape(Cylinder(kRadius, kLength), X_WG);
  auto context = scene_graph->CreateDefaultContext();
  auto query =
      scene_graph->get_query_output_port().Eval<QueryObject<double>>(*context);

  CartesianProduct S(query, geom_id, std::nullopt);
  Matrix<double, 3, 4> in_G, out_G;
  // clang-format off
  in_G << 0, 0,           kRadius,      kRadius/std::sqrt(2.0),
          0, 0,           0,            kRadius/std::sqrt(2.0),
          0, kLength/2.0, -kLength/2.0, kLength/2.0;
  out_G << kRadius+.01, 0,     kRadius,          kRadius/std::sqrt(2.0),
           0, 0,               0,                kRadius/std::sqrt(2.0),
           0, kLength/2.0+.01, -kLength/2.0-.01, kLength/2.0+.01;
  // clang-format on
  const Matrix<double, 3, 4> in_W = X_WG * in_G, out_W = X_WG * out_G;
  const double kTol = 1e-14;
  for (int i = 0; i < 4; ++i) {
    EXPECT_LE(query.ComputeSignedDistanceToPoint(in_W.col(i))[0].distance,
              kTol);
    EXPECT_GE(query.ComputeSignedDistanceToPoint(out_W.col(i))[0].distance,
              -kTol);

    EXPECT_TRUE(S.PointInSet(in_W.col(i), kTol));
    EXPECT_FALSE(S.PointInSet(out_W.col(i), kTol));
  }
  EXPECT_FALSE(S.MaybeGetPoint().has_value());

  // Test reference_frame frame.
  SourceId source_id = scene_graph->RegisterSource("F");
  FrameId frame_id = scene_graph->RegisterFrame(source_id, GeometryFrame("F"));
  auto context2 = scene_graph->CreateDefaultContext();
  const RigidTransformd X_WF{math::RollPitchYawd(.5, .26, -3),
                             Vector3d{.9, -2., .12}};
  const FramePoseVector<double> pose_vector{{frame_id, X_WF}};
  scene_graph->get_source_pose_port(source_id).FixValue(context2.get(),
                                                        pose_vector);
  auto query2 =
      scene_graph->get_query_output_port().Eval<QueryObject<double>>(*context2);
  CartesianProduct S2(query2, geom_id, frame_id);

  const RigidTransformd X_FW = X_WF.inverse();
  const Matrix<double, 3, 4> in_F = X_FW * in_W, out_F = X_FW * out_W;
  for (int i = 0; i < 4; ++i) {
    EXPECT_TRUE(S2.PointInSet(in_F.col(i), kTol));
    EXPECT_FALSE(S2.PointInSet(out_F.col(i), kTol));
  }
}

GTEST_TEST(CartesianProductTest, TwoBoxes) {
  HPolyhedron H1 = HPolyhedron::MakeBox(Vector2d{1, 1}, Vector2d{2, 2});
  HPolyhedron H2 = HPolyhedron::MakeBox(Vector2d{-2, 2}, Vector2d{0, 4});
  CartesianProduct S(H1, H2);
  EXPECT_TRUE(S.IsBounded());
  EXPECT_FALSE(S.MaybeGetPoint().has_value());
  EXPECT_TRUE(S.PointInSet(Vector4d{1.9, 1.9, -.1, 3.9}));
  EXPECT_FALSE(S.PointInSet(Vector4d{1.9, 1.9, -.1, 4.1}));
  EXPECT_FALSE(S.PointInSet(Vector4d{2.1, 1.9, -.1, 3.9}));
}

GTEST_TEST(CartesianProductTest, UnboundedSets) {
  const Point P(Vector2d{1.2, 3.4});
  Eigen::Matrix2d A;
  A << -1, 0, 0, 1;
  const HPolyhedron H(A, Vector2d(0, 2));

  // Bounded x Unbounded -> Unbounded
  const CartesianProduct S(P, H);
  EXPECT_FALSE(S.IsBounded());

  // Unbounded x Bounded -> Unbounded
  const CartesianProduct S2(H, P);
  EXPECT_FALSE(S2.IsBounded());

  // Unbounded x Unbounded -> Unbounded
  const CartesianProduct S3(H, H);
  EXPECT_FALSE(S3.IsBounded());

  // Bounded x Unbounded -> Unbounded
  ConvexSets sets;
  sets.emplace_back(P);
  sets.emplace_back(H);
  const CartesianProduct S4(sets);
  EXPECT_FALSE(S4.IsBounded());
}

GTEST_TEST(CartesianProductTest, ScaledPoints) {
  const Point P1(Vector1d{1.2});
  const Point P2(Vector1d{3.4});
  // clang-format off
  Eigen::Matrix2d A;
  A << -2, 0,
        0, 5;
  // clang-format on
  Vector2d b{0, 3};
  const CartesianProduct S(MakeConvexSets(P1, P2), A, b);

  const double kTol = 1e-15;
  const Vector2d in{-0.6, 0.08};
  EXPECT_TRUE(S.PointInSet(in, kTol));
  ASSERT_TRUE(S.MaybeGetPoint().has_value());
  EXPECT_TRUE(CompareMatrices(S.MaybeGetPoint().value(), in, kTol));
}

GTEST_TEST(CartesianProductTest, ScaledPointAddPointInSet) {
  const Point P1(Vector1d{1.2});
  const Point P2(Vector1d{3.4});
  // clang-format off
  Eigen::Matrix2d A;
  A << -2, 0,
        0, 5;
  // clang-format on
  Vector2d b{0, 3};
  const CartesianProduct S(MakeConvexSets(P1, P2), A, b);

  solvers::MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  auto [new_vars, new_constraints] = S.AddPointInSetConstraints(&prog, x);
  EXPECT_EQ(new_vars.rows(), 2);
  EXPECT_EQ(new_constraints.size(), 3);
  auto result = solvers::Solve(prog);
  EXPECT_TRUE(result.is_success());
  const Eigen::Vector2d new_vars_val = result.GetSolution(new_vars);
  EXPECT_TRUE(CompareMatrices(new_vars_val,
                              Eigen::Vector2d(P1.x()(0), P2.x()(0)), 1E-6));
}

GTEST_TEST(CartesianProductTest, ScaledPointInjective) {
  const Point P(Vector3d{42, 42, 0});
  // clang-format off
  Eigen::MatrixXd A(3, 2);
  A << 1, 0,
       0, 1,
       0, 0;
  // clang-format on
  Vector3d b{22, 22, 0};
  const CartesianProduct S(MakeConvexSets(P), A, b);

  const double kTol = 1e-15;
  const Vector2d in{20, 20};
  EXPECT_TRUE(S.PointInSet(in, kTol));
  ASSERT_TRUE(S.MaybeGetPoint().has_value());
  EXPECT_TRUE(CompareMatrices(S.MaybeGetPoint().value(), in, kTol));
}

GTEST_TEST(CartesianProductTest, ScaledPointNotInjectiveFail) {
  const Point P(Vector1d{0});
  Eigen::MatrixXd A(1, 2);
  A << 1, 0;
  Vector1d b{0};
  DRAKE_EXPECT_THROWS_MESSAGE(CartesianProduct(MakeConvexSets(P), A, b),
                              ".*rank.*");
}

GTEST_TEST(CartesianProductTest, CloneTest) {
  const Point P1(Vector2d{1.2, 3.4}), P2(Vector2d{5.6, 7.8});
  const CartesianProduct S(P1, P2);
  std::unique_ptr<ConvexSet> clone = S.Clone();
  EXPECT_EQ(clone->ambient_dimension(), 4);

  CartesianProduct* s = dynamic_cast<CartesianProduct*>(clone.get());
  ASSERT_NE(s, nullptr);
  ASSERT_EQ(s->num_factors(), 2);
  const Point* p = dynamic_cast<const Point*>(&s->factor(0));
  ASSERT_NE(p, nullptr);
  EXPECT_TRUE(CompareMatrices(P1.x(), p->x()));
}

GTEST_TEST(CartesianProductTest, NonnegativeScalingTest) {
  const Point P1(Vector2d{1.2, 3.4}), P2(Vector2d{5.6, 7.8});
  const CartesianProduct S(P1, P2);

  solvers::MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(4, "x");
  auto t = prog.NewContinuousVariables(1, "t")[0];

  std::vector<solvers::Binding<solvers::Constraint>> constraints =
      S.AddPointInNonnegativeScalingConstraints(&prog, x, t);

  // 1 vector constraint from P1, 1 from P2, and t>=0 3 times.
  EXPECT_EQ(constraints.size(), 5);

  const double tol = 1e-16;
  Vector4d p;
  p << P1.x(), P2.x();
  for (const double scale : {0.5, 1.0, 2.0}) {
    prog.SetInitialGuess(x, scale * p);
    prog.SetInitialGuess(t, scale);
    EXPECT_TRUE(prog.CheckSatisfiedAtInitialGuess(constraints, tol));
    prog.SetInitialGuess(x, 0.99 * scale * p);
    EXPECT_FALSE(prog.CheckSatisfiedAtInitialGuess(constraints, tol));
  }
  prog.SetInitialGuess(x, p);
  prog.SetInitialGuess(t, -1.0);
  EXPECT_FALSE(prog.CheckSatisfiedAtInitialGuess(constraints, tol));
}

bool PointInScaledSet(const solvers::VectorXDecisionVariable& x_vars,
                      const solvers::VectorXDecisionVariable& t_var,
                      const Vector2d& x, const Vector2d& t,
                      solvers::MathematicalProgram* prog) {
  auto b1 = prog->AddBoundingBoxConstraint(x, x, x_vars);
  auto b2 = prog->AddBoundingBoxConstraint(t, t, t_var);
  auto result = solvers::Solve(*prog);
  prog->RemoveConstraint(b1);
  prog->RemoveConstraint(b2);
  return result.is_success();
}

GTEST_TEST(CartesianProductTest, NonnegativeScalingTest2) {
  const Point P1(Vector2d{1.2, 3.4}), P2(Vector2d{2.4, 6.8});
  const CartesianProduct S(P1, P2);

  solvers::MathematicalProgram prog;
  Eigen::MatrixXd A(4, 2);
  // clang-format off
  A << 1, 0,
       0, 1,
       2, 0,
       0, 2;
  // clang-format on
  Eigen::Vector4d b = Eigen::Vector4d::Zero();
  auto x = prog.NewContinuousVariables(2, "x");
  Eigen::Vector2d c(1, -1);
  double d = 0;
  auto t = prog.NewContinuousVariables(2, "t");

  std::vector<solvers::Binding<solvers::Constraint>> constraints =
      S.AddPointInNonnegativeScalingConstraints(&prog, A, b, c, d, x, t);

  // 1 vector constraint from P1, 1 from P2, and c'*t+d>=0 3 times.
  EXPECT_EQ(constraints.size(), 5);

  Eigen::Vector2d x_solution = P1.x();
  Eigen::Vector3d scale(0.5, 1.0, 2.0);
  Eigen::MatrixXd t_solutions(2, 9);
  // clang-format off
  t_solutions << 0.5,  0,  0.25, 1,  0,  0.5, 2,  0,  1,
                 0, -0.5, -0.25, 0, -1, -0.5, 0, -2, -1;
  // clang-format on
  for (int ii = 0; ii < t_solutions.cols(); ++ii) {
    EXPECT_TRUE(PointInScaledSet(x, t, scale[ii / 3] * x_solution,
                                 t_solutions.col(ii), &prog));
    EXPECT_FALSE(PointInScaledSet(x, t, 0.99 * scale[ii / 3] * x_solution,
                                  t_solutions.col(ii), &prog));
  }

  EXPECT_FALSE(
      PointInScaledSet(x, t, x_solution, Eigen::Vector2d(0, 1), &prog));
  EXPECT_FALSE(
      PointInScaledSet(x, t, x_solution, Eigen::Vector2d(-1, 0), &prog));
}

// Test the case where A and b are set via the constructor.  Note that the
// SceneGraph test also operates on sets that have A and b set.  In particular,
// this test exercises the case of a non-square A.
GTEST_TEST(CartesianProductTest, Rotated) {
  // Denote the ambient space of S with _S.
  const Vector2d in_S{1.23, 4.5};
  const Vector2d out_S = in_S + Vector2d{0.0, 0.01};

  Matrix<double, 4, 2> A;
  A << 1.2, 4, 2, -1.2, 4.3, -2, 7.1, 1.4;
  Vector4d b{.64, 9.2, 7.2, 1.};
  // Denote the "rotated" space, where we take product of points P, with _P.
  const Vector4d in_P = A * in_S + b;
  const Point P1(in_P.head<2>()), P2(in_P.tail<2>());

  // The set S is equivalent to a Point(in_S).
  ConvexSets sets;
  sets.emplace_back(P1);
  sets.emplace_back(P2);
  const CartesianProduct S(sets, A, b);

  EXPECT_EQ(S.ambient_dimension(), 2);
  EXPECT_EQ(S.num_factors(), 2);
  EXPECT_TRUE(S.IsBounded());

  // The set S is equivalent to Point(in_S).
  const double kTol = 1e-14;
  EXPECT_TRUE(S.PointInSet(in_S, kTol));
  EXPECT_FALSE(S.PointInSet(out_S, kTol));

  EXPECT_TRUE(internal::CheckAddPointInSetConstraints(S, in_S));
  EXPECT_FALSE(internal::CheckAddPointInSetConstraints(S, out_S));

  solvers::MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(2, "x");
  auto t = prog.NewContinuousVariables(1, "t")[0];
  std::vector<solvers::Binding<solvers::Constraint>> constraints =
      S.AddPointInNonnegativeScalingConstraints(&prog, x, t);

  // The rotation terms cause an additional y=Ax+b constraint to be added inside
  // the method, so I must solve the problem now with constraints to set x and t
  // now, instead of just checking the declared constraints.

  // Test that 0.5*in_S is in the set scaled by 0.5.
  prog.AddBoundingBoxConstraint(.5, .5, t);
  solvers::Binding<solvers::BoundingBoxConstraint> x_constraint =
      prog.AddBoundingBoxConstraint(.5 * in_S, .5 * in_S, x);
  EXPECT_TRUE(Solve(prog).is_success());

  // Test that 0.5*out_S is *not* in the set scaled by 0.5.
  prog.RemoveConstraint(x_constraint);
  prog.AddBoundingBoxConstraint(.5 * out_S, .5 * out_S, x);
  EXPECT_FALSE(Solve(prog).is_success());
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
