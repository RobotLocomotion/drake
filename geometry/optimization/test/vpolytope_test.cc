#include "drake/geometry/optimization/vpolytope.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/optimization/test_utilities.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/random_rotation.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace geometry {
namespace optimization {

using Eigen::Matrix;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using internal::CheckAddPointInSetConstraints;
using internal::MakeSceneGraphWithShape;
using math::RigidTransformd;
using math::RotationMatrixd;
using solvers::Binding;
using solvers::Constraint;
using solvers::MathematicalProgram;

GTEST_TEST(VPolytopeTest, TriangleTest) {
  Eigen::Matrix<double, 2, 3> triangle;
  // clang-format off
  triangle <<  2, 4, 3,
              -1, 2, 5;
  // clang-format on

  VPolytope V(triangle);
  EXPECT_EQ(V.ambient_dimension(), 2);
  EXPECT_TRUE(CompareMatrices(V.vertices(), triangle));

  // Check IsBounded (which is trivially true for V Polytopes).
  EXPECT_TRUE(V.IsBounded());

  const Eigen::Vector2d center = triangle.rowwise().mean();
  // Mosek worked with 1e-15.
  // Gurobi worked with 1e-15 (see the note about alpha_sol in the method).
  // CLP needed 1e-11.
  const double kTol = 1e-11;
  EXPECT_TRUE(V.PointInSet(center, kTol));
  for (int i = 0; i < 3; ++i) {
    const Eigen::Vector2d v = triangle.col(i);
    const Eigen::Vector2d at_tol =
        v + kTol * (v - center) / ((v - center).lpNorm<Eigen::Infinity>());
    EXPECT_TRUE(CompareMatrices(v, at_tol, 2.0 * kTol));
    EXPECT_FALSE(CompareMatrices(v, at_tol, .5 * kTol));

    EXPECT_TRUE(V.PointInSet(v, kTol));
    EXPECT_TRUE(V.PointInSet(at_tol, 2.0 * kTol));
    EXPECT_FALSE(V.PointInSet(at_tol, 0.5 * kTol));
  }
}

GTEST_TEST(VPolytopeTest, UnitBoxTest) {
  VPolytope V = VPolytope::MakeUnitBox(3);
  EXPECT_EQ(V.ambient_dimension(), 3);

  const Vector3d in1_W{.8, .3, -.9}, in2_W{-1.0, 1.0, 1.0},
      out_W{1.1, 1.2, 0.4};

  // Test PointInSet.
  // Mosek worked with 1e-14; CLP needed 1e-11.
  const double kTol = 1e-11;
  EXPECT_TRUE(V.PointInSet(in1_W, kTol));
  EXPECT_TRUE(V.PointInSet(in2_W, kTol));
  EXPECT_FALSE(V.PointInSet(out_W, kTol));

  // Test AddPointInSetConstraints.
  EXPECT_TRUE(CheckAddPointInSetConstraints(V, in1_W));
  EXPECT_TRUE(CheckAddPointInSetConstraints(V, in2_W));
  EXPECT_FALSE(CheckAddPointInSetConstraints(V, out_W));

  // Test SceneGraph constructor.
  auto [scene_graph, geom_id] =
      MakeSceneGraphWithShape(Box(2.0, 2.0, 2.0), RigidTransformd::Identity());
  auto context = scene_graph->CreateDefaultContext();
  auto query =
      scene_graph->get_query_output_port().Eval<QueryObject<double>>(*context);

  VPolytope V_scene_graph(query, geom_id);
  EXPECT_EQ(V.ambient_dimension(), 3);
  EXPECT_TRUE(V.PointInSet(in1_W, kTol));
  EXPECT_TRUE(V.PointInSet(in2_W, kTol));
  EXPECT_FALSE(V.PointInSet(out_W, kTol));
}

GTEST_TEST(VPolytopeTest, ArbitraryBoxTest) {
  const RigidTransformd X_WG(math::RollPitchYawd(.1, .2, 3),
                             Vector3d(-4.0, -5.0, -6.0));
  auto [scene_graph, geom_id] =
      MakeSceneGraphWithShape(Box(1.0, 2.0, 3.0), X_WG);
  auto context = scene_graph->CreateDefaultContext();
  auto query =
      scene_graph->get_query_output_port().Eval<QueryObject<double>>(*context);
  VPolytope V(query, geom_id);

  EXPECT_EQ(V.ambient_dimension(), 3);
  const Vector3d in1_G{.48, .98, -1.49}, in2_G{-0.49, .99, 1.49},
      out_G{0.51, .9, 0.4};
  const Vector3d in1_W = X_WG * in1_G, in2_W = X_WG * in2_G,
                 out_W = X_WG * out_G;

  EXPECT_LE(query.ComputeSignedDistanceToPoint(in1_W)[0].distance, 0.0);
  EXPECT_LE(query.ComputeSignedDistanceToPoint(in2_W)[0].distance, 0.0);
  EXPECT_GE(query.ComputeSignedDistanceToPoint(out_W)[0].distance, 0.0);

  const double kTol = 1e-14;
  EXPECT_TRUE(V.PointInSet(in1_W, kTol));
  EXPECT_TRUE(V.PointInSet(in2_W, kTol));
  EXPECT_FALSE(V.PointInSet(out_W, kTol));

  EXPECT_TRUE(CheckAddPointInSetConstraints(V, in1_W));
  EXPECT_TRUE(CheckAddPointInSetConstraints(V, in2_W));
  EXPECT_FALSE(CheckAddPointInSetConstraints(V, out_W));

  // Test reference_frame frame.
  SourceId source_id = scene_graph->RegisterSource("F");
  FrameId frame_id = scene_graph->RegisterFrame(source_id, GeometryFrame("F"));
  auto context2 = scene_graph->CreateDefaultContext();
  const RigidTransformd X_WF{math::RollPitchYawd(5.1, 3.2, -3),
                             Vector3d{.5, .87, .1}};
  const FramePoseVector<double> pose_vector{{frame_id, X_WF}};
  scene_graph->get_source_pose_port(source_id).FixValue(context2.get(),
                                                        pose_vector);
  auto query2 =
      scene_graph->get_query_output_port().Eval<QueryObject<double>>(*context2);
  VPolytope V_F(query2, geom_id, frame_id);

  const RigidTransformd X_FW = X_WF.inverse();
  EXPECT_TRUE(V_F.PointInSet(X_FW * in1_W, kTol));
  EXPECT_TRUE(V_F.PointInSet(X_FW * in2_W, kTol));
  EXPECT_FALSE(V_F.PointInSet(X_FW * out_W, kTol));
}

// Check if the set of vertices equals to the set of vertices_expected.
void CheckVertices(const Eigen::Ref<const Eigen::Matrix3Xd>& vertices,
                   const Eigen::Ref<const Eigen::Matrix3Xd>& vertices_expected,
                   double tol) {
  EXPECT_EQ(vertices.cols(), vertices_expected.cols());
  const int num_vertices = vertices.cols();
  for (int i = 0; i < num_vertices; ++i) {
    bool found_match = false;
    for (int j = 0; j < num_vertices; ++j) {
      if (CompareMatrices(vertices.col(i), vertices_expected.col(j), tol)) {
        found_match = true;
        break;
      }
    }
    EXPECT_TRUE(found_match);
  }
}

GTEST_TEST(VPolytopeTest, OctahedronTest) {
  const RigidTransformd X_WG(math::RollPitchYawd(.1, .2, 3),
                             Vector3d(-4.0, -5.0, -6.0));
  auto [scene_graph, geom_id] = MakeSceneGraphWithShape(
      Convex(FindResourceOrThrow("drake/geometry/test/octahedron.obj")), X_WG);
  auto context = scene_graph->CreateDefaultContext();
  auto query =
      scene_graph->get_query_output_port().Eval<QueryObject<double>>(*context);
  VPolytope V(query, geom_id);
  EXPECT_EQ(V.vertices().cols(), 6);

  EXPECT_EQ(V.ambient_dimension(), 3);

  Eigen::Matrix<double, 6, 3> p_GV_expected;
  // clang-format off
  p_GV_expected << 1, 1, 0,
                   1, -1, 0,
                   -1, 1, 0,
                   -1, -1, 0,
                   0, 0, std::sqrt(2),
                   0, 0, -std::sqrt(2);
  // clang-format on
  CheckVertices(V.vertices(), X_WG * p_GV_expected.transpose(), 1E-9);
}

GTEST_TEST(VPolytopeTest, NonconvexMesh) {
  auto [scene_graph, geom_id] = MakeSceneGraphWithShape(
      Convex(FindResourceOrThrow("drake/geometry/test/non_convex_mesh.obj")),
      RigidTransformd{});
  auto context = scene_graph->CreateDefaultContext();
  auto query =
      scene_graph->get_query_output_port().Eval<QueryObject<double>>(*context);
  VPolytope V(query, geom_id);

  // The non-convex mesh contains 5 vertices, but the convex hull contains only
  // 4 vertices.
  const int num_vertices = 4;
  EXPECT_EQ(V.vertices().cols(), num_vertices);
  EXPECT_EQ(V.ambient_dimension(), 3);
  Eigen::Matrix<double, 4, 3> vertices_expected;
  // clang-format off
  vertices_expected << 0, 0, 0,
                       1, 0, 0,
                       0, 1, 0,
                       0, 0, 1;
  // clang-format on
  const double tol = 1E-12;
  CheckVertices(V.vertices(), vertices_expected.transpose(), tol);
}

GTEST_TEST(VPolytopeTest, UnitBox6DTest) {
  VPolytope V = VPolytope::MakeUnitBox(6);
  EXPECT_EQ(V.ambient_dimension(), 6);

  Vector6d in1_W{Vector6d::Constant(-.99)}, in2_W{Vector6d::Constant(.99)},
      out1_W{Vector6d::Constant(-1.01)}, out2_W{Vector6d::Constant(1.01)};

  const double kTol = 1e-11;
  EXPECT_TRUE(V.PointInSet(in1_W, kTol));
  EXPECT_TRUE(V.PointInSet(in2_W, kTol));
  EXPECT_FALSE(V.PointInSet(out1_W, kTol));
  EXPECT_FALSE(V.PointInSet(out2_W, kTol));
}

GTEST_TEST(VPolytopeTest, FromHUnitBoxTest) {
  HPolyhedron H = HPolyhedron::MakeUnitBox(6);
  VPolytope V(H);
  EXPECT_EQ(V.ambient_dimension(), 6);
  EXPECT_EQ(V.vertices().rows(), 6);
  EXPECT_EQ(V.vertices().cols(), std::pow(2, 6));

  const Vector6d in1_W{Vector6d::Constant(-.99)},
      in2_W{Vector6d::Constant(.99)}, out1_W{Vector6d::Constant(-1.01)},
      out2_W{Vector6d::Constant(1.01)};
  Vector6d out3_W;
  out3_W << .99, 1.01, .99, 1.01, .99, 1.01;

  const double kTol = 1e-11;
  EXPECT_TRUE(V.PointInSet(in1_W, kTol));
  EXPECT_TRUE(V.PointInSet(in2_W, kTol));
  EXPECT_FALSE(V.PointInSet(out1_W, kTol));
  EXPECT_FALSE(V.PointInSet(out2_W, kTol));
  EXPECT_FALSE(V.PointInSet(out3_W, kTol));
}

GTEST_TEST(VPolytopeTest, From2DHPolytopeTest) {
  Matrix<double, 4, 2> A;
  Vector4d b;
  // clang-format off
  A <<  1, -1,  // y ≥ x
        1,  0,  // x ≤ 1
        0,  1,  // y ≤ 2
       -1,  0;  // x ≥ 0
  // clang-format on
  b << 0, 1, 2, 0;
  HPolyhedron H(A, b);
  VPolytope V(H);

  // Vertices should be (0,0), (1,1), (1,2), and (0,2). The order of the
  // returned vertices need not be unique.
  Eigen::MatrixXd vertices(2, 4);
  // clang-format off
  vertices << 0, 1, 0, 1,
              0, 1, 2, 2;
  // clang-format on
  for (int i = 0; i < 4; ++i) {
    bool found_match = false;
    for (int j = 0; j < 4; ++j) {
      if (CompareMatrices(vertices.col(i), V.vertices().col(j), 1e-11)) {
        found_match = true;
        break;
      }
    }
    EXPECT_TRUE(found_match);
  }
}

GTEST_TEST(VPolytopeTest, From3DHSimplexTest) {
  Matrix<double, 4, 3> A;
  Vector4d b;
  // clang-format off
  A << -1,  0,  0,
        0, -1,  0,
        0,  0, -1,
        1,  1,  1;
  // clang-format on
  b << 0, 0, 0, 1;
  HPolyhedron H(A, b);
  VPolytope V(H);

  // Vertices are (0,0,0), (1,0,0), (0,1,0), and (0,0,1).  The order of the
  // returned vertices need not be unique.
  Eigen::MatrixXd vertices(3, 4);
  // clang-format off
  vertices << 0, 1, 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;
  // clang-format on
  for (int i = 0; i < 4; ++i) {
    bool found_match = false;
    for (int j = 0; j < 4; ++j) {
      if (CompareMatrices(vertices.col(i), V.vertices().col(j), 1e-11)) {
        found_match = true;
        break;
      }
    }
    EXPECT_TRUE(found_match);
  }
}

// Same as FromHPolytopeTest but with two redundant constraints.
GTEST_TEST(VPolytopeTest, FromRedundantHPolytopeTest) {
  Matrix<double, 6, 2> A;
  Vector6d b;
  A << 1, -1,  // y ≥ x
      1, 0,    // x ≤ 1
      0, 1,    // y ≤ 2
      -1, 0,   // x ≥ 0
      1, 1,    // x + y ≤ 3.1   (redundant)
      -1, -1;  // x + y ≥ - 0.1 (redundant)
  b << 0, 1, 2, 0, 3.1, 0.1;
  HPolyhedron H(A, b);
  VPolytope V(H);

  // Vertices should be (0,0), (1,1), (1,2), and (0,2). The order of the
  // returned vertices need not be unique.
  Eigen::MatrixXd vertices(2, 4);
  // clang-format off
  vertices << 0, 1, 0, 1,
              0, 1, 2, 2;
  // clang-format on
  for (int i = 0; i < 4; ++i) {
    bool found_match = false;
    for (int j = 0; j < 4; ++j) {
      if (CompareMatrices(vertices.col(i), V.vertices().col(j), 1e-11)) {
        found_match = true;
        break;
      }
    }
    EXPECT_TRUE(found_match);
  }
}

GTEST_TEST(VPolytopeTest, FromUnboundedHPolytopeTest) {
  Matrix<double, 3, 2> A;
  Vector3d b;
  A << 1, -1,  // y ≥ x
      1, 0,    // x ≤ 1
      0, 1;    // y ≤ 2
  b << 0, 1, 2;
  HPolyhedron H(A, b);

  DRAKE_EXPECT_THROWS_MESSAGE(VPolytope{H}, ".*hpoly.IsBounded().*");
}

GTEST_TEST(VPolytopeTest, CloneTest) {
  VPolytope V = VPolytope::MakeBox(Vector3d{-3, -4, -5}, Vector3d{6, 7, 8});
  std::unique_ptr<ConvexSet> clone = V.Clone();
  EXPECT_EQ(clone->ambient_dimension(), V.ambient_dimension());
  VPolytope* pointer = dynamic_cast<VPolytope*>(clone.get());
  ASSERT_NE(pointer, nullptr);
  EXPECT_TRUE(CompareMatrices(V.vertices(), pointer->vertices()));
}

// Returns true iff a scaled version of the upper corner is confirmed to be
// inside the scaled box.
bool PointInScaledSet(double x_scale, double t_scale) {
  const Vector3d lb{1, 1, 1}, ub{2, 3, 4};
  VPolytope V = VPolytope::MakeBox(lb, ub);

  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(3, "x");
  auto t = prog.NewContinuousVariables(1, "t")[0];

  std::vector<Binding<Constraint>> constraints =
      V.AddPointInNonnegativeScalingConstraints(&prog, x, t);

  EXPECT_EQ(constraints.size(), 4);

  prog.AddBoundingBoxConstraint(x_scale * ub, x_scale * ub, x);
  prog.AddBoundingBoxConstraint(t_scale, t_scale, t);
  auto result = solvers::Solve(prog);
  return result.is_success();
}

GTEST_TEST(VPolytopeTest, NonnegativeScalingTest) {
  // Note: I have to call Solve on these (not just check the constraints)
  // because of the slack variables.
  EXPECT_TRUE(PointInScaledSet(0.99, 1.0));
  EXPECT_FALSE(PointInScaledSet(1.01, 1.0));
  EXPECT_FALSE(PointInScaledSet(0.99, -0.01));
  EXPECT_TRUE(PointInScaledSet(0.49, 0.5));
  EXPECT_FALSE(PointInScaledSet(0.51, 0.5));
  EXPECT_TRUE(PointInScaledSet(1.99, 2.0));
  EXPECT_FALSE(PointInScaledSet(2.01, 2.0));
}

// Returns true iff a scaled version of the upper corner is confirmed to be
// inside the scaled box.
bool PointInTransformedScaledSet(double x_scale, Vector2d t_guess) {
  const Vector3d lb{1, 1, 1}, ub{2, 3, 4};
  VPolytope V = VPolytope::MakeBox(lb, ub);

  MathematicalProgram prog;
  Eigen::MatrixXd A(3, 2);
  // clang-format off
  A << 1, 0,
       0, 1,
       2, 0;
  // clang-format on
  Eigen::Vector3d b = Eigen::Vector3d::Zero();
  auto x = prog.NewContinuousVariables(2, "x");
  Eigen::Vector2d c(1, -1);
  double d = 0;
  auto t = prog.NewContinuousVariables(2, "t");

  std::vector<Binding<Constraint>> constraints =
      V.AddPointInNonnegativeScalingConstraints(&prog, A, b, c, d, x, t);

  EXPECT_EQ(constraints.size(), 4);

  prog.AddBoundingBoxConstraint(x_scale * ub.head(2), x_scale * ub.head(2), x);
  prog.AddBoundingBoxConstraint(t_guess, t_guess, t);
  auto result = solvers::Solve(prog);
  return result.is_success();
}

GTEST_TEST(VPolytopeTest, NonnegativeScalingTest2) {
  // Note: I have to call Solve on these (not just check the constraints)
  // because of the slack variables.
  EXPECT_TRUE(PointInTransformedScaledSet(0.99, Vector2d(1.0, 0)));
  EXPECT_TRUE(PointInTransformedScaledSet(0.99, Vector2d(0, -1.0)));
  EXPECT_FALSE(PointInTransformedScaledSet(1.01, Vector2d(1.0, 0)));
  EXPECT_FALSE(PointInTransformedScaledSet(1.01, Vector2d(0, -1.0)));
  EXPECT_FALSE(PointInTransformedScaledSet(0.99, Vector2d(-0.01, 0)));
  EXPECT_FALSE(PointInTransformedScaledSet(0.99, Vector2d(0, -0.01)));
  EXPECT_TRUE(PointInTransformedScaledSet(0.49, Vector2d(0.5, 0)));
  EXPECT_TRUE(PointInTransformedScaledSet(0.49, Vector2d(0, -0.5)));
  EXPECT_FALSE(PointInTransformedScaledSet(0.51, Vector2d(0.5, 0)));
  EXPECT_FALSE(PointInTransformedScaledSet(0.51, Vector2d(0, -0.5)));
  EXPECT_TRUE(PointInTransformedScaledSet(1.99, Vector2d(2.0, 0)));
  EXPECT_TRUE(PointInTransformedScaledSet(1.99, Vector2d(0, -2.0)));
  EXPECT_FALSE(PointInTransformedScaledSet(2.01, Vector2d(2.0, 0)));
  EXPECT_FALSE(PointInTransformedScaledSet(2.01, Vector2d(0, -2.0)));
}

GTEST_TEST(VPolytopeTest, CalcVolume) {
  const double tol{1E-6};
  EXPECT_NEAR(VPolytope::MakeUnitBox(3).CalcVolume(), 8, tol);

  Eigen::Matrix<double, 2, 5> vertices_2d;
  // clang-format off
  vertices_2d << 1, -1, 0, 0, 0,
                 0, 0, 1, -1, 0;
  // clang-format on
  EXPECT_NEAR(VPolytope(vertices_2d).CalcVolume(), 2, tol);

  Eigen::Matrix<double, 3, 5> vertices_3d;
  // clang-format off
  vertices_3d << 1, 0, 0, 0, 0.25,
                 0, 1, 0, 0, 0.25,
                 0, 0, 1, 0, 0.25;
  // clang-format on
  EXPECT_NEAR(VPolytope(vertices_3d).CalcVolume(), 1.0 / 6, tol);

  // A degenerate case where al the 3d vertices are on a 2d plane.
  Eigen::Matrix<double, 3, 4> vertices_3d_planar;
  // clang-format off
  vertices_3d_planar << 1, -1, 0, 0,
                        0, 0, 1, -1,
                        0, 0, 0, 0;
  // clang-format on
  EXPECT_NEAR(VPolytope(vertices_3d_planar).CalcVolume(), 0., tol);
}

double CalcPathLength(const Eigen::MatrixXd& vertices) {
  DRAKE_DEMAND(vertices.rows() == 2);

  size_t n = vertices.cols();
  double length = 0;

  for (size_t i = 0; i < n; ++i) {
    auto j = (i + 1) % n;

    auto diff = vertices.col(i) - vertices.col(j);
    length += sqrt(diff.dot(diff));
  }

  return length;
}

GTEST_TEST(VPolytopeTest, GetMinimalRepresentationTest) {
  const double tol{1E-6};

  // 2D: Square plus some points inside
  {
    Eigen::Matrix<double, 2, 6> vertices;
    const double l = 2.0;
    // clang-format off
    vertices << 0, l, 0, l, l/4, l/2,
                0, 0, l, l, l/5, l/3;
    // clang-format on
    auto vpoly = VPolytope(vertices).GetMinimalRepresentation();
    EXPECT_EQ(vpoly.vertices().cols(), 4);
    EXPECT_NEAR(vpoly.CalcVolume(), l * l, tol);
    // Calculate the length of the path that visits all the vertices
    // sequentially.
    // If the vertices are in clockwise/counter-clockwise order,
    // the length of the path will coincide with the perimeter of a
    // square.
    EXPECT_EQ(CalcPathLength(vpoly.vertices()), l * 4);

    // Test PointInSet with points nearby the four edges.
    const double d = 10 * tol;
    for (int axis = 0; axis < 2; ++axis) {
      for (int face = 0; face < 2; ++face) {
        for (int side = 0; side < 2; ++side) {
          Vector2d point(l / 2, l / 2);
          point[axis] = l * face + d * (side * 2 - 1);
          if (face + side == 1) {
            EXPECT_TRUE(vpoly.PointInSet(point, tol));
          } else {
            EXPECT_FALSE(vpoly.PointInSet(point, tol));
          }
        }
      }
    }
  }

  // 3D: Box plus some points inside
  {
    Eigen::Matrix<double, 3, 10> vertices;
    const double l = 2.0;
    // clang-format off
    vertices << 0, l, 0, l, 0, l, 0, l, l/4, l/2,
                0, 0, l, l, 0, 0, l, l, l/5, l/3,
                0, 0, 0, 0, l, l, l, l, l/6, l/4;
    // clang-format on
    auto vpoly = VPolytope(vertices).GetMinimalRepresentation();
    EXPECT_EQ(vpoly.vertices().cols(), 8);
    EXPECT_NEAR(vpoly.CalcVolume(), l * l * l, tol);

    // Test PointInSet with points nearby the six faces.
    const double d = 10 * tol;
    for (int axis = 0; axis < 3; ++axis) {
      for (int face = 0; face < 2; ++face) {
        for (int side = 0; side < 2; ++side) {
          Vector3d point(l / 2, l / 2, l / 2);
          point[axis] = l * face + d * (side * 2 - 1);
          if (face + side == 1) {
            EXPECT_TRUE(vpoly.PointInSet(point, tol));
          } else {
            EXPECT_FALSE(vpoly.PointInSet(point, tol));
          }
        }
      }
    }
  }
}

// Confirm that WriteObj generates an Obj file that can be read back in to
// obtain the same VPolytope. All of the geometry work is done by qhull; this
// test simply covers the data flow.
GTEST_TEST(VPolytopeTest, WriteObjTest) {
  VPolytope V = VPolytope::MakeUnitBox(3);

  const std::string filename = temp_directory() + "/vpolytope.obj";
  V.WriteObj(filename);

  auto [scene_graph, geom_id] =
      MakeSceneGraphWithShape(Convex(filename, 1), RigidTransformd::Identity());
  auto context = scene_graph->CreateDefaultContext();
  auto query =
      scene_graph->get_query_output_port().Eval<QueryObject<double>>(*context);

  VPolytope V_scene_graph(query, geom_id);
  CheckVertices(V.vertices(), V_scene_graph.vertices(), 1e-6);
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
