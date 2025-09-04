#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/test_utilities/symbolic_test_util.h"
#include "drake/geometry/collision_filter_declaration.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/optimization/cspace_free_polytope.h"  // NOLINT
#include "drake/geometry/optimization/test/c_iris_test_utilities.h"
#include "drake/multibody/rational/rational_forward_kinematics.h"
#include "drake/multibody/rational/rational_forward_kinematics_internal.h"
#include "drake/solvers/common_solver_option.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace geometry {
namespace optimization {
// Only use two threads during testing. This value should match the "cpu" tag
// in BUILD.bazel defining this test.
constexpr int kTestConcurrency = 2;

const double kInf = std::numeric_limits<double>::infinity();

// Project s_sample to the polytope {s | C*s<=d, s_lower<=s<=s_upper}.
[[nodiscard]] Eigen::VectorXd ProjectToPolytope(
    const Eigen::VectorXd& s_sample, const Eigen::Ref<const Eigen::MatrixXd>& C,
    const Eigen::Ref<const Eigen::VectorXd>& d,
    const Eigen::Ref<const Eigen::VectorXd>& s_lower,
    const Eigen::Ref<const Eigen::VectorXd>& s_upper) {
  DRAKE_DEMAND(s_sample.rows() == C.cols());
  if (((C * s_sample).array() <= d.array()).all() &&
      (s_sample.array() >= s_lower.array()).all() &&
      (s_sample.array() <= s_upper.array()).all()) {
    return s_sample;
  } else {
    solvers::MathematicalProgram prog;
    auto s = prog.NewContinuousVariables(s_sample.rows());
    prog.AddBoundingBoxConstraint(s_lower, s_upper, s);
    prog.AddLinearConstraint(C, Eigen::VectorXd::Constant(C.rows(), -kInf), d,
                             s);
    prog.AddQuadraticErrorCost(Eigen::MatrixXd::Identity(s.rows(), s.rows()),
                               s_sample, s);
    const auto result = solvers::Solve(prog);
    DRAKE_DEMAND(result.is_success());
    return result.GetSolution(s);
  }
}

void SetupPolytope(const CspaceFreePolytopeTester& tester,
                   const Eigen::Ref<const Eigen::MatrixXd>& C,
                   const Eigen::Ref<const Eigen::VectorXd>& d,
                   VectorX<symbolic::Polynomial>* d_minus_Cs,
                   std::unordered_set<int>* C_redundant_indices,
                   std::unordered_set<int>* s_lower_redundant_indices,
                   std::unordered_set<int>* s_upper_redundant_indices) {
  *d_minus_Cs = tester.CalcDminusCs<double>(C, d);
  tester.FindRedundantInequalities(
      C, d, tester.s_lower(), tester.s_upper(), 0., C_redundant_indices,
      s_lower_redundant_indices, s_upper_redundant_indices);
}

// Lagrangian should be positive if it is irredundant, otherwise it should be
// zero.
void CheckLagrangians(const VectorX<symbolic::Polynomial>& lagrangians,
                      const std::unordered_set<int> redundant_indices,
                      const Eigen::MatrixXd& indeterminates_samples,
                      const VectorX<symbolic::Variable>& indeterminates) {
  for (int i = 0; i < lagrangians.rows(); ++i) {
    if (!redundant_indices.contains(i)) {
      CheckPositivePolynomialBySamples(lagrangians(i), indeterminates,
                                       indeterminates_samples.transpose());
    } else {
      EXPECT_PRED2(symbolic::test::PolyEqual, lagrangians(i),
                   symbolic::Polynomial());
    }
  }
}

// @param s_samples Each row of s_samples is a sampled configuration s.
// @param C The c-space polytope is {s | C*s<=d, s_lower<=s<=s_upper}.
// @param d The c-space polytope is {s | C*s<=d, s_lower<=s<=s_upper}.
// @param a Maps the plane index to the separating plane parameter `a` in {x|
// aᵀx+b=0}
// @param b Maps the plane index to the separating plane parameter `b` in {x|
// aᵀx+b=0}
void CheckSeparationBySamples(
    const CspaceFreePolytopeTester& tester,
    const systems::Diagram<double>& diagram,
    const Eigen::Ref<const Eigen::MatrixXd>& s_samples,
    const Eigen::MatrixXd& C, const Eigen::VectorXd& d,
    const std::unordered_map<int, Vector3<symbolic::Polynomial>>& a,
    const std::unordered_map<int, symbolic::Polynomial>& b,
    const Eigen::Ref<const Eigen::VectorXd>& q_star,
    const CspaceFreePolytope::IgnoredCollisionPairs& ignored_collision_pairs) {
  auto diagram_context = diagram.CreateDefaultContext();
  const auto& plant =
      tester.cspace_free_polytope().rational_forward_kin().plant();
  auto& plant_context =
      plant.GetMyMutableContextFromRoot(diagram_context.get());

  for (int i = 0; i < s_samples.rows(); ++i) {
    const Eigen::Vector3d s_val = ProjectToPolytope(
        s_samples.row(i).transpose(), C, d, tester.s_lower(), tester.s_upper());
    symbolic::Environment env;
    env.insert(tester.cspace_free_polytope().rational_forward_kin().s(), s_val);
    const Eigen::VectorXd q_val =
        tester.cspace_free_polytope().rational_forward_kin().ComputeQValue(
            s_val, q_star);
    plant.SetPositions(&plant_context, q_val);
    for (int plane_index = 0;
         plane_index <
         static_cast<int>(
             tester.cspace_free_polytope().separating_planes().size());
         ++plane_index) {
      const auto& plane =
          tester.cspace_free_polytope().separating_planes()[plane_index];
      if (!ignored_collision_pairs.contains(SortedPair<geometry::GeometryId>(
              plane.positive_side_geometry->id(),
              plane.negative_side_geometry->id())) &&
          a.contains(plane_index) && b.contains(plane_index)) {
        Eigen::Vector3d a_val;
        for (int j = 0; j < 3; ++j) {
          a_val(j) = a.at(plane_index)(j).Evaluate(env);
        }
        const double b_val = b.at(plane_index).Evaluate(env);
        EXPECT_GE(
            DistanceToHalfspace(*plane.positive_side_geometry, a_val, b_val,
                                plane.expressed_body, PlaneSide::kPositive,
                                plant, plant_context),
            0);
        EXPECT_GE(
            DistanceToHalfspace(*plane.negative_side_geometry, a_val, b_val,
                                plane.expressed_body, PlaneSide::kNegative,
                                plant, plant_context),
            0);
      }
    }
  }
}

void CheckSosLagrangians(
    const std::vector<CspaceFreePolytope::SeparatingPlaneLagrangians>&
        lagrangians_vec) {
  for (const auto& lagrangians : lagrangians_vec) {
    for (int i = 0; i < lagrangians.polytope().rows(); ++i) {
      EXPECT_TRUE(IsPolynomialSos(lagrangians.polytope()(i), 0.0));
    }
    for (int i = 0; i < lagrangians.s_lower().rows(); ++i) {
      EXPECT_TRUE(IsPolynomialSos(lagrangians.s_lower()(i), 0.0));
    }
    for (int i = 0; i < lagrangians.s_upper().rows(); ++i) {
      EXPECT_TRUE(IsPolynomialSos(lagrangians.s_upper()(i), 0.0));
    }
  }
}

// Check if rationals are all positive in the C-space polytope. Note that for
// some reason Mosek doesn't solve to a very high precision (the constraint
// violation can be in the order of 1E-3, even if Mosek reports success), so we
// could use a pretty large tolerance `tol`.
void CheckRationalsPositiveInCspacePolytope(
    const std::vector<symbolic::RationalFunction>& rationals,
    const std::vector<CspaceFreePolytope::SeparatingPlaneLagrangians>&
        lagrangians_vec,
    const VectorX<symbolic::Variable>& plane_decision_vars,
    const Eigen::VectorXd& plane_decision_var_vals, const Eigen::MatrixXd& C,
    const Eigen::VectorXd& d, const CspaceFreePolytopeTester& tester,
    double tol) {
  const VectorX<symbolic::Polynomial> d_minus_Cs =
      tester.CalcDminusCs<double>(C, d);
  symbolic::Environment env;
  env.insert(plane_decision_vars, plane_decision_var_vals);
  EXPECT_EQ(rationals.size(), lagrangians_vec.size());
  for (int i = 0; i < static_cast<int>(rationals.size()); ++i) {
    const symbolic::Polynomial rational_numerator =
        rationals[i].numerator().EvaluatePartial(env);
    const symbolic::Polynomial sos_poly =
        rational_numerator - lagrangians_vec[i].polytope().dot(d_minus_Cs) -
        lagrangians_vec[i].s_lower().dot(tester.s_minus_s_lower()) -
        lagrangians_vec[i].s_upper().dot(tester.s_upper_minus_s());
    EXPECT_TRUE(IsPolynomialSos(sos_poly, tol));
  }
  CheckSosLagrangians(lagrangians_vec);
}

void TestConstructPlaneSearchProgram(
    const systems::Diagram<double>& diagram,
    const multibody::MultibodyPlant<double>& plant,
    const SceneGraph<double>& scene_graph,
    const SortedPair<geometry::GeometryId>& geometry_pair, bool with_cross_y) {
  const Eigen::Vector3d q_star(0, 0, 0);
  CspaceFreePolytope::Options options;
  options.with_cross_y = with_cross_y;
  CspaceFreePolytopeTester tester(
      &plant, &scene_graph, SeparatingPlaneOrder::kAffine, q_star, options);
  Eigen::Matrix<double, 9, 3> C;
  // clang-format off
  C <<  1,  1,  0,
       -1, -1,  0,
       -1,  0,  1,
        1,  0, -1,
        0,  1,  1,
        0, -1, -1,
        1,  0,  1,
        1,  1, -1,
        1, -1,  1;
  // clang-format on
  Eigen::Matrix<double, 9, 1> d;
  d << 0.1, 0.2, 0.3, 0.2, 0.2, 0.2, 0.1, 0.1, 0.2;

  VectorX<symbolic::Polynomial> d_minus_Cs;
  std::unordered_set<int> C_redundant_indices;
  std::unordered_set<int> s_lower_redundant_indices;
  std::unordered_set<int> s_upper_redundant_indices;
  SetupPolytope(tester, C, d, &d_minus_Cs, &C_redundant_indices,
                &s_lower_redundant_indices, &s_upper_redundant_indices);

  // Consider the plane between world_box_ and body3_box_.
  // Notice that this chain only has one DOF, hence one of the rationals is
  // actually a constant.
  int plane_geometries_index = -1;
  for (int i = 0; i < static_cast<int>(tester.plane_geometries().size()); ++i) {
    const auto& plane =
        tester.cspace_free_polytope()
            .separating_planes()[tester.plane_geometries()[i].plane_index];
    if (SortedPair<geometry::GeometryId>(plane.positive_side_geometry->id(),
                                         plane.negative_side_geometry->id()) ==
        geometry_pair) {
      plane_geometries_index = i;
      break;
    }
  }
  const auto& plane_geometries =
      tester.plane_geometries()[plane_geometries_index];
  const auto& plane = tester.cspace_free_polytope()
                          .separating_planes()[plane_geometries.plane_index];
  auto ret = tester.ConstructPlaneSearchProgram(
      plane_geometries, d_minus_Cs, C_redundant_indices,
      s_lower_redundant_indices, s_upper_redundant_indices);
  EXPECT_EQ(ret.certificate.positive_side_rational_lagrangians.size(),
            plane_geometries.positive_side_rationals.size());
  EXPECT_EQ(ret.certificate.negative_side_rational_lagrangians.size(),
            plane_geometries.negative_side_rationals.size());

  solvers::MosekSolver solver;
  solvers::SolverOptions solver_options;
  const auto result = solver.Solve(*(ret.prog), std::nullopt, solver_options);
  EXPECT_TRUE(result.is_success());

  auto get_lagrangian_result =
      [](const solvers::MathematicalProgramResult& prog_result,
         const std::vector<CspaceFreePolytope::SeparatingPlaneLagrangians>&
             lagrangians_vec)
      -> std::vector<CspaceFreePolytope::SeparatingPlaneLagrangians> {
    std::vector<CspaceFreePolytope::SeparatingPlaneLagrangians>
        lagrangians_result;
    for (const auto& lagrangian : lagrangians_vec) {
      lagrangians_result.push_back(lagrangian.GetSolution(prog_result));
    }
    return lagrangians_result;
  };
  const auto positive_side_rational_lagrangians_result = get_lagrangian_result(
      result, ret.certificate.positive_side_rational_lagrangians);
  const auto negative_side_rational_lagrangians_result = get_lagrangian_result(
      result, ret.certificate.negative_side_rational_lagrangians);

  Vector3<symbolic::Polynomial> a_result;
  for (int i = 0; i < 3; ++i) {
    a_result(i) = result.GetSolution(plane.a(i));
  }
  const symbolic::Polynomial b_result = result.GetSolution(plane.b);
  const Eigen::VectorXd plane_decision_var_vals =
      result.GetSolution(plane.decision_variables);

  CheckRationalsPositiveInCspacePolytope(
      plane_geometries.positive_side_rationals,
      positive_side_rational_lagrangians_result, plane.decision_variables,
      plane_decision_var_vals, C, d, tester, 1E-5);
  CheckRationalsPositiveInCspacePolytope(
      plane_geometries.negative_side_rationals,
      negative_side_rational_lagrangians_result, plane.decision_variables,
      plane_decision_var_vals, C, d, tester, 1E-5);

  Eigen::Matrix<double, 10, 3> s_samples;
  // clang-format off
  s_samples <<   1,    2,   -1,
              -0.5,  0.3,  0.2,
               0.2,  0.1,  0.4,
               0.5, -1.2,  0.3,
               0.2,  0.5, -0.4,
              -0.3,  1.5,    2,
               0.5,  0.2,    1,
              -0.4,  0.5,    1,
                 0,    0,    0,
               0.2, -1.5,    1;
  // clang-format on

  CheckSeparationBySamples(tester, diagram, s_samples, C, d,
                           {{plane_geometries.plane_index, a_result}},
                           {{plane_geometries.plane_index, b_result}}, q_star,
                           {});
}

TEST_F(CIrisToyRobotTest, ConstructPlaneSearchProgram1) {
  // Test ConstructPlaneSearchProgram with both geometries being polytopes.
  for (bool with_cross_y : {false, true}) {
    TestConstructPlaneSearchProgram(
        *diagram_, *plant_, *scene_graph_,
        SortedPair<geometry::GeometryId>(world_box_, body3_box_), with_cross_y);
  }
}

TEST_F(CIrisToyRobotTest, ConstructPlaneSearchProgram2) {
  // Test ConstructPlaneSearchProgram with neither geometries being polytope.
  for (bool with_cross_y : {false, true}) {
    TestConstructPlaneSearchProgram(
        *diagram_, *plant_, *scene_graph_,
        SortedPair<geometry::GeometryId>(world_box_, body2_capsule_),
        with_cross_y);
  }
}

TEST_F(CIrisToyRobotTest, ConstructPlaneSearchProgram3) {
  // Test ConstructPlaneSearchProgram with one geometry being polytope and the
  // other not.
  for (bool with_cross_y : {false, true}) {
    TestConstructPlaneSearchProgram(
        *diagram_, *plant_, *scene_graph_,
        SortedPair<geometry::GeometryId>(body1_convex_, body3_cylinder_),
        with_cross_y);
  }
}

TEST_F(CIrisToyRobotTest, MakeAndSolveIsGeometrySeparableProgram) {
  const Eigen::Vector3d q_star(0, 0, 0);
  CspaceFreePolytope::Options options;
  options.with_cross_y = false;
  CspaceFreePolytopeTester tester(
      plant_, scene_graph_, SeparatingPlaneOrder::kAffine, q_star, options);
  Eigen::Matrix<double, 9, 3> C_good;
  // clang-format off
  C_good << 1, 1, 0,
            -1, -1, 0,
            -1, 0, 1,
            1, 0, -1,
            0, 1, 1,
            0, -1, -1,
            1, 0, 1,
            1, 1, -1,
            1, -1, 1;
  // clang-format on
  Eigen::Matrix<double, 9, 1> d_good;
  d_good << 0.1, 0.2, 0.3, 0.2, 0.2, 0.2, 0.1, 0.1, 0.2;

  // First test the geometry pair that doesn't require a separation certificate.
  {
    CspaceFreePolytope::SeparationCertificateProgram ret;
    // These two geometries are on the same body.
    DRAKE_EXPECT_THROWS_MESSAGE(
        ret = tester.cspace_free_polytope().MakeIsGeometrySeparableProgram(
            SortedPair<geometry::GeometryId>(body0_box_, body0_sphere_), C_good,
            d_good),
        ".*does not need a separation certificate");
  }
  CspaceFreePolytope::FindSeparationCertificateGivenPolytopeOptions
      find_certificate_options;
  find_certificate_options.verbose = false;
  solvers::MosekSolver solver;
  find_certificate_options.solver_id = solver.id();

  const SortedPair<geometry::GeometryId> geometry_pair{body0_box_,
                                                       body2_sphere_};
  {
    auto separation_certificate_program =
        tester.cspace_free_polytope().MakeIsGeometrySeparableProgram(
            geometry_pair, C_good, d_good);
    auto separation_certificate_result =
        tester.cspace_free_polytope().SolveSeparationCertificateProgram(
            separation_certificate_program, find_certificate_options);
    const auto plane_index =
        tester.cspace_free_polytope().map_geometries_to_separating_planes().at(
            geometry_pair);
    const auto& plane_geometry = tester.plane_geometries()[plane_index];
    CheckSosLagrangians(
        separation_certificate_result->positive_side_rational_lagrangians);
    CheckSosLagrangians(
        separation_certificate_result->negative_side_rational_lagrangians);
    CheckRationalsPositiveInCspacePolytope(
        plane_geometry.positive_side_rationals,
        separation_certificate_result->positive_side_rational_lagrangians,
        tester.cspace_free_polytope()
            .separating_planes()[plane_index]
            .decision_variables,
        separation_certificate_result->plane_decision_var_vals, C_good, d_good,
        tester, 1E-5);
    CheckRationalsPositiveInCspacePolytope(
        plane_geometry.negative_side_rationals,
        separation_certificate_result->negative_side_rational_lagrangians,
        tester.cspace_free_polytope()
            .separating_planes()[plane_index]
            .decision_variables,
        separation_certificate_result->plane_decision_var_vals, C_good, d_good,
        tester, 1E-5);
    EXPECT_TRUE(separation_certificate_result.has_value());
    Eigen::Matrix<double, 10, 3> s_samples;
    // clang-format off
    s_samples << 1, 2, -1,
               -0.5, 0.3, 0.2,
               0.2, 0.1, 0.4,
               0.5, -1.2, 0.3,
               0.2, 0.5, -0.4,
               -0.3, 1.5, 2,
               0.5, 0.2, 1,
               -0.4, 0.5, 1,
               0, 0, 0,
               0.2, -1.5, 1;
    // clang-format on
    CheckSeparationBySamples(tester, *diagram_, s_samples, C_good, d_good,
                             {{separation_certificate_result->plane_index,
                               separation_certificate_result->a}},
                             {{separation_certificate_result->plane_index,
                               separation_certificate_result->b}},
                             q_star, {});
  }

  {
    // This C-space polytope is NOT collision free.
    Eigen::Matrix<double, 4, 3> C_bad;
    // clang-format off
    C_bad << 1, 1, 0,
            -1, -1, 0,
            -1, 0, 1,
          1, 0, -1;
    // clang-format on
    Eigen::Matrix<double, 4, 1> d_bad;
    d_bad << 10.8, 20, 34, 22;
    auto separation_certificate_program =
        tester.cspace_free_polytope().MakeIsGeometrySeparableProgram(
            geometry_pair, C_bad, d_bad);
    auto separation_certificate_result =
        tester.cspace_free_polytope().SolveSeparationCertificateProgram(
            separation_certificate_program, find_certificate_options);
    EXPECT_FALSE(separation_certificate_result.has_value());
  }
}

TEST_F(CIrisToyRobotTest, FindSeparationCertificateGivenPolytopeSuccess) {
  // Test CspaceFreePolytope::FindSeparationCertificateGivenPolytope for a
  // collision-free C-space polytope.
  const Eigen::Vector3d q_star(0, 0, 0);
  CspaceFreePolytopeTester tester(plant_, scene_graph_,
                                  SeparatingPlaneOrder::kAffine, q_star);
  // This C-space polytope is collision free.
  Eigen::Matrix<double, 9, 3> C;
  // clang-format off
  C << 1, 1, 0,
       -1, -1, 0,
       -1, 0, 1,
       1, 0, -1,
       0, 1, 1,
       0, -1, -1,
       1, 0, 1,
       1, 1, -1,
       1, -1, 1;
  // clang-format on
  Eigen::Matrix<double, 9, 1> d;
  d << 0.1, 0.1, 0.1, 0.02, 0.02, 0.2, 0.1, 0.1, 0.2;

  // We sample some s and project them to the polytope {s | C*s<=d, s_lower <=s
  // <=s_upper}, then compute the corresponding q.
  Eigen::Matrix<double, 10, 3> s_samples;
  // clang-format off
  s_samples << 1, 2, -1,
             -0.5, 0.3, 0.2,
             0.2, 0.1, 0.4,
             0.5, -1.2, 0.3,
             0.2, 0.5, -0.4,
             -0.3, 1.5, 2,
             0.5, 0.2, 1,
             -0.4, 0.5, 1,
             0, 0, 0,
             0.2, -1.5, 1;
  // clang-format on

  const CspaceFreePolytope::IgnoredCollisionPairs ignored_collision_pairs{
      SortedPair<geometry::GeometryId>(world_box_, body2_sphere_)};
  CspaceFreePolytope::FindSeparationCertificateGivenPolytopeOptions options;
  options.verbose = false;
  solvers::MosekSolver solver;
  options.solver_id = solver.id();
  for (int num_threads : {1, kTestConcurrency}) {
    options.parallelism = num_threads;

    const auto certificates_result =
        tester.FindSeparationCertificateGivenPolytope(ignored_collision_pairs,
                                                      C, d, options);
    EXPECT_EQ(certificates_result.size(),
              tester.cspace_free_polytope().separating_planes().size() -
                  ignored_collision_pairs.size());
    for (const auto& certificate : certificates_result) {
      EXPECT_TRUE(certificate.has_value());
      const auto& plane = tester.cspace_free_polytope()
                              .separating_planes()[certificate->plane_index];
      EXPECT_FALSE(
          ignored_collision_pairs.contains(SortedPair<geometry::GeometryId>(
              plane.positive_side_geometry->id(),
              plane.negative_side_geometry->id())));

      CheckSeparationBySamples(tester, *diagram_, s_samples, C, d,
                               {{certificate->plane_index, certificate->a}},
                               {{certificate->plane_index, certificate->b}},
                               q_star, ignored_collision_pairs);

      // Now check if the polynomials are actually sos.
      const auto& plane_geometries =
          tester.plane_geometries()[certificate->plane_index];
      CheckRationalsPositiveInCspacePolytope(
          plane_geometries.positive_side_rationals,
          certificate->positive_side_rational_lagrangians,
          plane.decision_variables, certificate->plane_decision_var_vals, C, d,
          tester, 1E-4);
      CheckRationalsPositiveInCspacePolytope(
          plane_geometries.negative_side_rationals,
          certificate->negative_side_rational_lagrangians,
          plane.decision_variables, certificate->plane_decision_var_vals, C, d,
          tester, 1E-4);
    }
  }
  std::unordered_map<SortedPair<geometry::GeometryId>,
                     CspaceFreePolytope::SeparationCertificateResult>
      certificates_map;
  bool is_success =
      tester.cspace_free_polytope().FindSeparationCertificateGivenPolytope(
          C, d, ignored_collision_pairs, options, &certificates_map);
  EXPECT_EQ(certificates_map.size(),
            tester.cspace_free_polytope().separating_planes().size() -
                ignored_collision_pairs.size());
  EXPECT_TRUE(is_success);

  // Check the separating plane does separate the geometries.
  std::unordered_map<int, Vector3<symbolic::Polynomial>> a_map;
  std::unordered_map<int, symbolic::Polynomial> b_map;
  for (const auto& [geometry_pair, certificate] : certificates_map) {
    a_map.emplace(certificate.plane_index, certificate.a);
    b_map.emplace(certificate.plane_index, certificate.b);
  }
  CheckSeparationBySamples(tester, *diagram_, s_samples, C, d, a_map, b_map,
                           q_star, ignored_collision_pairs);
}

TEST_F(CIrisToyRobotTest, FindSeparationCertificateGivenPolytopeFailure) {
  // Test CspaceFreePolytope::FindSeparationCertificateGivenPolytope. The
  // C-space polytope is NOT collision free.
  const Eigen::Vector3d q_star(0, 0, 0);
  CspaceFreePolytopeTester tester(plant_, scene_graph_,
                                  SeparatingPlaneOrder::kAffine, q_star);

  // This C-space polytope is NOT collision free.
  Eigen::Matrix<double, 4, 3> C;
  // clang-format off
  C << 1, 1, 0,
       -1, -1, 0,
       -1, 0, 1,
       1, 0, -1;
  // clang-format on
  Eigen::Matrix<double, 4, 1> d;
  d << 10.8, 20, 34, 22;

  const CspaceFreePolytope::IgnoredCollisionPairs ignored_collision_pairs{
      SortedPair<geometry::GeometryId>(world_box_, body2_sphere_)};
  CspaceFreePolytope::FindSeparationCertificateGivenPolytopeOptions options;
  options.verbose = false;
  solvers::MosekSolver solver;
  options.solver_id = solver.id();
  for (int num_threads : {1, kTestConcurrency}) {
    options.parallelism = num_threads;

    const auto certificates_result =
        tester.FindSeparationCertificateGivenPolytope(ignored_collision_pairs,
                                                      C, d, options);
    EXPECT_EQ(certificates_result.size(),
              tester.cspace_free_polytope().separating_planes().size() -
                  ignored_collision_pairs.size());
    EXPECT_TRUE(std::any_of(
        certificates_result.begin(), certificates_result.end(),
        [](const std::optional<CspaceFreePolytope::SeparationCertificateResult>&
               certificate_result) {
          return !certificate_result.has_value();
        }));
  }
  // Test with terminate_at_failure=false. This will run the sos for every
  // separating plane.
  options.terminate_at_failure = false;
  const auto certificates_result =
      tester.FindSeparationCertificateGivenPolytope(ignored_collision_pairs, C,
                                                    d, options);
  EXPECT_EQ(certificates_result.size(),
            tester.cspace_free_polytope().separating_planes().size() -
                ignored_collision_pairs.size());
  std::unordered_map<SortedPair<geometry::GeometryId>,
                     CspaceFreePolytope::SeparationCertificateResult>
      certificates_map;
  const bool is_success =
      tester.cspace_free_polytope().FindSeparationCertificateGivenPolytope(
          C, d, ignored_collision_pairs, options, &certificates_map);
  EXPECT_FALSE(is_success);

  // These are the geometries that currently we can't certify being separable. I
  // haven't searched for a configuration that the geometries are in collision
  // yet.
  std::unordered_set<SortedPair<geometry::GeometryId>> inseparable_geometries;
  inseparable_geometries.emplace(body0_box_, body2_sphere_);
  for (const auto& certificate : certificates_result) {
    if (certificate.has_value()) {
      const auto& plane = tester.cspace_free_polytope()
                              .separating_planes()[certificate->plane_index];
      const SortedPair<geometry::GeometryId> geometry_pair(
          plane.positive_side_geometry->id(),
          plane.negative_side_geometry->id());
      EXPECT_FALSE(inseparable_geometries.contains(geometry_pair));
    }
  }
  for (const auto& [geometry_pair, certificate] : certificates_map) {
    EXPECT_FALSE(inseparable_geometries.contains(geometry_pair));
  }
}

void CompareLagrangians(
    const std::vector<CspaceFreePolytope::SeparatingPlaneLagrangians>&
        lagrangians_vec1,
    const std::vector<CspaceFreePolytope::SeparatingPlaneLagrangians>&
        lagrangians_vec2,
    bool compare_s_bounds) {
  EXPECT_EQ(lagrangians_vec1.size(), lagrangians_vec2.size());
  for (int i = 0; i < static_cast<int>(lagrangians_vec1.size()); ++i) {
    EXPECT_EQ(lagrangians_vec1[i].polytope().size(),
              lagrangians_vec2[i].polytope().size());
    for (int j = 0; j < lagrangians_vec1[i].polytope().rows(); ++j) {
      EXPECT_PRED2(symbolic::test::PolyEqual, lagrangians_vec1[i].polytope()(j),
                   lagrangians_vec2[i].polytope()(j));
    }
    if (compare_s_bounds) {
      EXPECT_EQ(lagrangians_vec1[i].s_lower().size(),
                lagrangians_vec2[i].s_lower().size());
      EXPECT_EQ(lagrangians_vec1[i].s_upper().size(),
                lagrangians_vec2[i].s_upper().size());
      for (int j = 0; j < lagrangians_vec1[i].s_lower().rows(); ++j) {
        EXPECT_PRED2(symbolic::test::PolyEqual,
                     lagrangians_vec1[i].s_lower()(j),
                     lagrangians_vec2[i].s_lower()(j));
        EXPECT_PRED2(symbolic::test::PolyEqual,
                     lagrangians_vec1[i].s_upper()(j),
                     lagrangians_vec2[i].s_upper()(j));
      }
    }
  }
}

void CheckPolytopeSearchResult(
    const CspaceFreePolytopeTester& tester, const Eigen::MatrixXd& C_sol,
    const Eigen::VectorXd& d_sol,
    const solvers::MathematicalProgramResult& result,
    const std::vector<
        std::optional<CspaceFreePolytope::SeparationCertificateResult>>&
        certificates_result,
    const std::unordered_map<int, CspaceFreePolytope::SeparationCertificate>&
        new_certificates,
    bool search_s_bounds_lagrangians, double tol) {
  // Now check rationals.
  std::unordered_map<int, CspaceFreePolytope::SeparationCertificateResult>
      new_certificates_result;
  for (const auto& [plane_index, new_certificate] : new_certificates) {
    const auto& plane =
        tester.cspace_free_polytope().separating_planes()[plane_index];
    const auto& plane_geometries = tester.plane_geometries()[plane_index];
    const CspaceFreePolytope::SeparationCertificateResult
        new_certificate_result = new_certificate.GetSolution(
            plane_index, plane.a, plane.b, plane.decision_variables, result);
    new_certificates_result.emplace(plane_index, new_certificate_result);
    CheckRationalsPositiveInCspacePolytope(
        plane_geometries.positive_side_rationals,
        new_certificate_result.positive_side_rational_lagrangians,
        plane.decision_variables,
        new_certificate_result.plane_decision_var_vals, C_sol, d_sol, tester,
        tol);
    CheckRationalsPositiveInCspacePolytope(
        plane_geometries.negative_side_rationals,
        new_certificate_result.negative_side_rational_lagrangians,
        plane.decision_variables,
        new_certificate_result.plane_decision_var_vals, C_sol, d_sol, tester,
        tol);
  }
  // Check if the Lagrangians are set correctly.
  for (const auto& old_certificate : certificates_result) {
    const int plane_index = old_certificate->plane_index;
    const auto& new_certificate_result =
        new_certificates_result.at(plane_index);
    CompareLagrangians(
        old_certificate->positive_side_rational_lagrangians,
        new_certificate_result.positive_side_rational_lagrangians,
        !search_s_bounds_lagrangians);
    CompareLagrangians(
        old_certificate->negative_side_rational_lagrangians,
        new_certificate_result.negative_side_rational_lagrangians,
        !search_s_bounds_lagrangians);
  }
}

// Test the private InitializePolytopeSearchProgram function
TEST_F(CIrisRobotPolytopicGeometryTest, InitializePolytopeSearchProgram) {
  const Eigen::Vector4d q_star(0, 0, 0, 0);
  CspaceFreePolytopeTester tester(plant_, scene_graph_,
                                  SeparatingPlaneOrder::kAffine, q_star);
  Eigen::Matrix<double, 12, 4> C;
  // clang-format off
  C << 1, 1, 0, 0,
       -1, -1, 0, 1,
       -1, 0, 1, -1,
       1, 0, -1, 0,
       0, 1, 1, 0,
       0, -1, -1, 1,
       1, 0, 1, 0,
       1, 1, -1, 1,
       1, -1, 1, -1,
       0, 1, 1, 1,
       1, 1, 1, -1,
       1, 0, 0, 1;
  // clang-format on
  Eigen::Matrix<double, 12, 1> d;
  d << 0.1, 0.1, 0.1, 0.02, 0.02, 0.2, 0.1, 0.1, 0.2, 0.1, 0.1, 0.02;
  for (int i = 0; i < C.rows(); ++i) {
    const double C_row_norm = C.row(i).norm();
    C.row(i) = C.row(i) / C_row_norm;
    d(i) = d(i) / C_row_norm;
  }

  CspaceFreePolytope::IgnoredCollisionPairs ignored_collision_pairs{};
  CspaceFreePolytope::FindSeparationCertificateGivenPolytopeOptions options;
  options.verbose = false;
  solvers::MosekSolver solver;
  options.solver_id = solver.id();
  options.parallelism = kTestConcurrency;
  const auto certificates_result =
      tester.FindSeparationCertificateGivenPolytope(ignored_collision_pairs, C,
                                                    d, options);
  ASSERT_TRUE(std::all_of(
      certificates_result.begin(), certificates_result.end(),
      [](const std::optional<CspaceFreePolytope::SeparationCertificateResult>&
             certificate) {
        return certificate.has_value();
      }));

  for (bool search_s_bounds_lagrangians : {false, true}) {
    const int gram_total_size = tester.GetGramVarSizeForPolytopeSearchProgram(
        ignored_collision_pairs, search_s_bounds_lagrangians);
    MatrixX<symbolic::Variable> C_var(C.rows(), C.cols());
    VectorX<symbolic::Variable> d_var(d.rows());
    for (int i = 0; i < C.rows(); ++i) {
      d_var(i) = symbolic::Variable("d" + std::to_string(i));
      for (int j = 0; j < C.cols(); ++j) {
        C_var(i, j) = symbolic::Variable(fmt::format("C({},{})", i, j));
      }
    }
    const VectorX<symbolic::Polynomial> d_minus_Cs =
        tester.CalcDminusCs<symbolic::Variable>(C_var, d_var);

    std::unordered_map<int, CspaceFreePolytope::SeparationCertificate>
        new_certificates;
    auto prog = tester.InitializePolytopeSearchProgram(
        ignored_collision_pairs, C_var, d_var, d_minus_Cs, certificates_result,
        search_s_bounds_lagrangians, gram_total_size, &new_certificates);
    solvers::SolverOptions solver_options;
    solver_options.SetOption(solvers::CommonSolverOption::kPrintToConsole, 0);
    const auto result = solver.Solve(*prog, std::nullopt, solver_options);
    ASSERT_TRUE(result.is_success());
    const auto C_sol = result.GetSolution(C_var);
    const auto d_sol = result.GetSolution(d_var);
    CheckPolytopeSearchResult(tester, C_sol, d_sol, result, certificates_result,
                              new_certificates, search_s_bounds_lagrangians,
                              2E-2);
  }

  // Now test the public InitializePolytopeSearchProgram function.
  // First find the separation certificate with a fixed C-space polytope using
  // the public FindSeparationCertificateGivenPolytope function.
  std::unordered_map<SortedPair<geometry::GeometryId>,
                     CspaceFreePolytope::SeparationCertificateResult>
      certificates;
  const bool find_separation_success =
      tester.cspace_free_polytope().FindSeparationCertificateGivenPolytope(
          C, d, ignored_collision_pairs, options, &certificates);
  ASSERT_TRUE(find_separation_success);
  for (bool search_s_bounds_lagrangians : {false, true}) {
    MatrixX<symbolic::Variable> C_var;
    VectorX<symbolic::Variable> d_var;
    std::unordered_map<int, CspaceFreePolytope::SeparationCertificate>
        new_certificates;
    auto prog = tester.cspace_free_polytope().InitializePolytopeSearchProgram(
        ignored_collision_pairs, certificates, search_s_bounds_lagrangians,
        &C_var, &d_var, &new_certificates);
    solvers::SolverOptions solver_options;
    solver_options.SetOption(solvers::CommonSolverOption::kPrintToConsole, 0);
    const auto result = solver.Solve(*prog, std::nullopt, solver_options);
    ASSERT_TRUE(result.is_success());
    const auto C_sol = result.GetSolution(C_var);
    const auto d_sol = result.GetSolution(d_var);
    CheckPolytopeSearchResult(tester, C_sol, d_sol, result, certificates_result,
                              new_certificates, search_s_bounds_lagrangians,
                              2E-2);
  }
}

class CIrisToyRobotInitializePolytopeSearchProgramTest
    : public CIrisToyRobotTest {
 public:
  void Test(bool with_cross_y,
            const std::vector<bool>& search_s_bounds_lagrangians_options) {
    const Eigen::Vector3d q_star(0, 0, 0);
    CspaceFreePolytope::Options cspace_free_polytope_options;
    cspace_free_polytope_options.with_cross_y = with_cross_y;
    CspaceFreePolytopeTester tester(plant_, scene_graph_,
                                    SeparatingPlaneOrder::kAffine, q_star,
                                    cspace_free_polytope_options);
    Eigen::Matrix<double, 9, 3> C;
    // clang-format off
  C << 1, 1, 0,
       -1, -1, 0,
       -1, 0, 1,
       1, 0, -1,
       0, 1, 1,
       0, -1, -1,
       1, 0, 1,
       1, 1, -1,
       1, -1, 1;
    // clang-format on
    Eigen::Matrix<double, 9, 1> d;
    d << 0.1, 0.1, 0.1, 0.02, 0.02, 0.2, 0.1, 0.1, 0.2;
    for (int i = 0; i < C.rows(); ++i) {
      const double C_row_norm = C.row(i).norm();
      C.row(i) = C.row(i) / C_row_norm;
      d(i) = d(i) / C_row_norm;
    }

    CspaceFreePolytope::IgnoredCollisionPairs ignored_collision_pairs{
        SortedPair<geometry::GeometryId>(world_box_, body2_sphere_),
        SortedPair<geometry::GeometryId>(world_box_, body2_capsule_),
        SortedPair<geometry::GeometryId>(body0_box_, body2_capsule_)};
    CspaceFreePolytope::FindSeparationCertificateGivenPolytopeOptions options;
    options.verbose = false;
    solvers::MosekSolver solver;
    options.solver_id = solver.id();
    options.solver_options = solvers::SolverOptions();
    options.solver_options->SetOption(
        solvers::CommonSolverOption::kPrintToConsole, 0);
    options.parallelism = kTestConcurrency;
    const auto certificates_result =
        tester.FindSeparationCertificateGivenPolytope(ignored_collision_pairs,
                                                      C, d, options);
    ASSERT_TRUE(std::all_of(
        certificates_result.begin(), certificates_result.end(),
        [](const std::optional<CspaceFreePolytope::SeparationCertificateResult>&
               certificate) {
          return certificate.has_value();
        }));

    for (bool search_s_bounds_lagrangians :
         search_s_bounds_lagrangians_options) {
      const int gram_total_size = tester.GetGramVarSizeForPolytopeSearchProgram(
          ignored_collision_pairs, search_s_bounds_lagrangians);
      MatrixX<symbolic::Variable> C_var(C.rows(), C.cols());
      VectorX<symbolic::Variable> d_var(d.rows());
      for (int i = 0; i < C.rows(); ++i) {
        d_var(i) = symbolic::Variable("d" + std::to_string(i));
        for (int j = 0; j < C.cols(); ++j) {
          C_var(i, j) = symbolic::Variable(fmt::format("C({},{})", i, j));
        }
      }
      const VectorX<symbolic::Polynomial> d_minus_Cs =
          tester.CalcDminusCs<symbolic::Variable>(C_var, d_var);

      std::unordered_map<int, CspaceFreePolytope::SeparationCertificate>
          new_certificates;
      auto prog = tester.InitializePolytopeSearchProgram(
          ignored_collision_pairs, C_var, d_var, d_minus_Cs,
          certificates_result, search_s_bounds_lagrangians, gram_total_size,
          &new_certificates);
      solvers::SolverOptions solver_options;
      solver_options.SetOption(solvers::CommonSolverOption::kPrintToConsole, 0);
      const auto result = solver.Solve(*prog, std::nullopt, solver_options);
      ASSERT_TRUE(result.is_success());
      const auto C_sol = result.GetSolution(C_var);
      const auto d_sol = result.GetSolution(d_var);
      CheckPolytopeSearchResult(tester, C_sol, d_sol, result,
                                certificates_result, new_certificates,
                                search_s_bounds_lagrangians, 1E-3);
    }
  }
};

TEST_F(CIrisToyRobotInitializePolytopeSearchProgramTest, WithoutCrossY) {
  Test(false /* with_cross_y */,
       {true} /* search_s_bounds_lagrangians_options */);
}

TEST_F(CIrisToyRobotInitializePolytopeSearchProgramTest, WithCrossY) {
  Test(true /* with_cross_y */,
       {true} /* search_s_bounds_lagrangians_options */);
}

TEST_F(CIrisToyRobotTest, FindPolytopeGivenLagrangian) {
  const Eigen::Vector3d q_star(0, 0, 0);
  CspaceFreePolytopeTester tester(plant_, scene_graph_,
                                  SeparatingPlaneOrder::kAffine, q_star);
  Eigen::Matrix<double, 9, 3> C;
  // clang-format off
  C << 1, 1, 0,
       -1, -1, 0,
       -1, 0, 1,
       1, 0, -1,
       0, 1, 1,
       0, -1, -1,
       1, 0, 1,
       1, 1, -1,
       1, -1, 1;
  // clang-format on
  Eigen::Matrix<double, 9, 1> d;
  d << 0.1, 0.1, 0.1, 0.02, 0.02, 0.2, 0.1, 0.1, 0.2;
  for (int i = 0; i < C.rows(); ++i) {
    const double C_row_norm = C.row(i).norm();
    C.row(i) = C.row(i) / C_row_norm;
    d(i) = d(i) / C_row_norm;
  }

  CspaceFreePolytope::IgnoredCollisionPairs ignored_collision_pairs{
      SortedPair<geometry::GeometryId>(world_box_, body2_sphere_),
      SortedPair<geometry::GeometryId>(world_box_, body2_capsule_),
      SortedPair<geometry::GeometryId>(body0_box_, body2_capsule_),
  };

  CspaceFreePolytope::FindSeparationCertificateGivenPolytopeOptions options;
  options.verbose = false;
  solvers::MosekSolver solver;
  options.solver_id = solver.id();
  options.parallelism = kTestConcurrency;

  const auto certificates_result =
      tester.FindSeparationCertificateGivenPolytope(ignored_collision_pairs, C,
                                                    d, options);
  ASSERT_TRUE(std::all_of(
      certificates_result.begin(), certificates_result.end(),
      [](const std::optional<CspaceFreePolytope::SeparationCertificateResult>&
             certificate) {
        return certificate.has_value();
      }));
  for (bool search_s_bounds_lagrangians : {true}) {
    const int gram_total_size = tester.GetGramVarSizeForPolytopeSearchProgram(
        ignored_collision_pairs, search_s_bounds_lagrangians);
    MatrixX<symbolic::Variable> C_var(C.rows(), C.cols());
    VectorX<symbolic::Variable> d_var(d.rows());
    for (int i = 0; i < C.rows(); ++i) {
      d_var(i) = symbolic::Variable("d" + std::to_string(i));
      for (int j = 0; j < C.cols(); ++j) {
        C_var(i, j) = symbolic::Variable(fmt::format("C({},{})", i, j));
      }
    }
    const VectorX<symbolic::Polynomial> d_minus_Cs =
        tester.CalcDminusCs<symbolic::Variable>(C_var, d_var);

    VectorX<symbolic::Variable> ellipsoid_margins(C.rows());
    for (int i = 0; i < ellipsoid_margins.rows(); ++i) {
      ellipsoid_margins(i) =
          symbolic::Variable("ellipsoid_margin" + std::to_string(i));
    }
    CspaceFreePolytope::FindPolytopeGivenLagrangianOptions polytope_options;
    polytope_options.solver_options.emplace(solvers::SolverOptions{});
    polytope_options.solver_options->SetOption(
        solvers::CommonSolverOption::kPrintToConsole, 0);
    polytope_options.search_s_bounds_lagrangians = search_s_bounds_lagrangians;
    polytope_options.backoff_scale = 0.05;
    polytope_options.ellipsoid_margin_epsilon = 1E-4;
    polytope_options.ellipsoid_margin_cost =
        CspaceFreePolytope::EllipsoidMarginCost::kGeometricMean;

    const HPolyhedron cspace_h_polyhedron =
        tester.GetPolyhedronWithJointLimits(C, d);
    const auto ellipsoid =
        cspace_h_polyhedron.MaximumVolumeInscribedEllipsoid();
    const Eigen::MatrixXd ellipsoid_Q = 0.98 * ellipsoid.A().inverse();
    std::unordered_map<int, CspaceFreePolytope::SeparationCertificateResult>
        new_certificates_result;
    const auto polytope_result = tester.FindPolytopeGivenLagrangian(
        ignored_collision_pairs, C_var, d_var, d_minus_Cs, certificates_result,
        ellipsoid_Q, ellipsoid.center(), ellipsoid_margins, gram_total_size,
        polytope_options, &new_certificates_result);
    ASSERT_TRUE(polytope_result.has_value());
    // Check the new separation certificates.
    EXPECT_EQ(new_certificates_result.size(),
              tester.cspace_free_polytope().separating_planes().size() -
                  ignored_collision_pairs.size());
    for (const auto& [plane_index, certificate_result] :
         new_certificates_result) {
      const auto& plane_geometries = tester.plane_geometries()[plane_index];
      const auto& plane =
          tester.cspace_free_polytope().separating_planes()[plane_index];
      CheckRationalsPositiveInCspacePolytope(
          plane_geometries.positive_side_rationals,
          certificate_result.positive_side_rational_lagrangians,
          plane.decision_variables, certificate_result.plane_decision_var_vals,
          polytope_result->C, polytope_result->d, tester, 1E-3);
      CheckRationalsPositiveInCspacePolytope(
          plane_geometries.negative_side_rationals,
          certificate_result.negative_side_rational_lagrangians,
          plane.decision_variables, certificate_result.plane_decision_var_vals,
          polytope_result->C, polytope_result->d, tester, 1E-3);
    }

    // Now check if the C-space polytope {s | C*s<=d, s_lower<=s<=s_upper} is
    // collision free.
    // First sample many s values, and project these values into the polytope
    // {s | C*s<=d, s_lower<=s<=s_upper}, then check if the plane separates
    // the geometries at the corresponding configurations.
    Eigen::Matrix<double, 10, 3> s_samples;
    // clang-format off
      s_samples << 1, 2, -1,
                 -0.5, 0.3, 0.2,
                 0.2, 0.9, 0.4,
                 0.5, -1.2, 0.3,
                 0.2, 2.5, -0.4,
                 -1.3, 1.5, 2,
                 0.5, 0.2, 1,
                 -0.4, 0.5, 1,
                 0, 0, 0,
                 0.2, -1.5, 1;
    // clang-format on
    EXPECT_EQ(polytope_result->a.size(),
              tester.cspace_free_polytope().separating_planes().size() -
                  ignored_collision_pairs.size());
    EXPECT_EQ(polytope_result->a.size(),
              tester.cspace_free_polytope().separating_planes().size() -
                  ignored_collision_pairs.size());
    CheckSeparationBySamples(tester, *diagram_, s_samples, polytope_result->C,
                             polytope_result->d, polytope_result->a,
                             polytope_result->b, q_star,
                             ignored_collision_pairs);

    // Check the margin between the inscribed ellipsoid and the polytope
    // faces.
    EXPECT_TRUE((polytope_result->ellipsoid_margins.array() >= 0).all());
    for (int i = 0; i < polytope_result->C.rows(); ++i) {
      EXPECT_LE(polytope_result->C.row(i).norm(), 1);
      EXPECT_GE((polytope_result->d(i) -
                 polytope_result->C.row(i).dot(ellipsoid.center()) -
                 (polytope_result->C.row(i) * ellipsoid_Q).norm()) /
                    polytope_result->C.row(i).norm(),
                polytope_result->ellipsoid_margins(i));
    }
    if (polytope_options.backoff_scale.has_value()) {
      // Check the inequality constraint |Qcᵢ|₂ ≤ dᵢ − δᵢ − cᵢᵀs₀ is satisfied
      EXPECT_TRUE(((ellipsoid_Q * polytope_result->C.transpose())
                       .colwise()
                       .norm()
                       .transpose()
                       .array() <=
                   (polytope_result->d - polytope_result->ellipsoid_margins -
                    polytope_result->C * ellipsoid.center())
                       .array())
                      .all());
    } else {
      // Check that the inequality constraint |Qcᵢ|₂ ≤ dᵢ − δᵢ − cᵢᵀs₀ is
      // active at the optimal solution.
      EXPECT_TRUE(CompareMatrices(
          (ellipsoid_Q * polytope_result->C.transpose()).colwise().norm(),
          (polytope_result->d - polytope_result->ellipsoid_margins -
           polytope_result->C * ellipsoid.center())
              .transpose(),
          1E-5));
    }
    // Check that the norm of each row in C is <= 1.
    EXPECT_TRUE((polytope_result->C.rowwise().norm().array() <= 1).all());
  }
}

TEST_F(CIrisToyRobotTest, SearchWithBilinearAlternation) {
  const Eigen::Vector3d q_star(0, 0, 0);
  CspaceFreePolytopeTester tester(plant_, scene_graph_,
                                  SeparatingPlaneOrder::kAffine, q_star);
  // Test with initial C and d that is collision-free.
  Eigen::Matrix<double, 9, 3> C;
  // clang-format off
  C << 1, 1, 0,
       -1, -1, 0,
       -1, 0, 1,
       1, 0, -1,
       0, 1, 1,
       0, -1, -1,
       1, 0, 1,
       1, 1, -1,
       1, -1, 1;
  // clang-format on
  Eigen::Matrix<double, 9, 1> d;
  d << 0.1, 0.1, 0.1, 0.02, 0.02, 0.2, 0.1, 0.1, 0.2;

  CspaceFreePolytope::IgnoredCollisionPairs ignored_collision_pairs{
      SortedPair<geometry::GeometryId>(world_box_, body2_sphere_)};

  CspaceFreePolytope::BilinearAlternationOptions bilinear_alternation_options;
  bilinear_alternation_options.max_iter = 5;
  bilinear_alternation_options.convergence_tol = 1E-5;
  bilinear_alternation_options.find_lagrangian_options.parallelism =
      kTestConcurrency;
  bilinear_alternation_options.find_polytope_options.solver_options =
      solvers::SolverOptions();
  bilinear_alternation_options.find_polytope_options.solver_options->SetOption(
      solvers::CommonSolverOption::kPrintToConsole, 0);
  bilinear_alternation_options.find_polytope_options.backoff_scale = 0.04;
  bilinear_alternation_options.find_polytope_options
      .search_s_bounds_lagrangians = true;
  bilinear_alternation_options.ellipsoid_scaling = 1;

  solvers::MosekSolver solver;
  auto bilinear_alternation_results =
      tester.cspace_free_polytope().SearchWithBilinearAlternation(
          ignored_collision_pairs, C, d, bilinear_alternation_options);
  ASSERT_FALSE(bilinear_alternation_results.empty());
  ASSERT_EQ(bilinear_alternation_results.size(),
            bilinear_alternation_results.back().num_iter() + 1);
  EXPECT_EQ(bilinear_alternation_results.back()
                .certified_polytope()
                .ambient_dimension(),
            C.cols());
  // Sample many s_values, project to {s | C*s <= d}. And then make sure that
  // the corresponding configurations are collision free.
  Eigen::Matrix<double, 10, 3> s_samples;
  // clang-format off
    s_samples << 1, 2, -1,
               -0.5, 0.3, 0.2,
               0.2, 0.9, 0.4,
               0.5, -1.2, 0.3,
               0.2, 2.5, -0.4,
               -1.3, 1.5, 2,
               0.5, 0.2, 1,
               -0.4, 0.5, 1,
               0, 0, 0,
               0.2, -1.5, 1;
  // clang-format on
  for (const auto& result : bilinear_alternation_results) {
    EXPECT_EQ(result.a().size(),
              tester.cspace_free_polytope().separating_planes().size() -
                  ignored_collision_pairs.size());
    EXPECT_EQ(result.a().size(),
              tester.cspace_free_polytope().separating_planes().size() -
                  ignored_collision_pairs.size());
    CheckSeparationBySamples(tester, *diagram_, s_samples, result.C(),
                             result.d(), result.a(), result.b(), q_star,
                             ignored_collision_pairs);
  }

  // Test the case that bilinear alternation fails.
  // The initial C-space polytope is too huge.
  auto bilinear_alternation_results_failure =
      tester.cspace_free_polytope().SearchWithBilinearAlternation(
          ignored_collision_pairs, C,
          d + Eigen::VectorXd::Constant(d.rows(), 100),
          bilinear_alternation_options);
  EXPECT_TRUE(bilinear_alternation_results_failure.empty());
}

TEST_F(CIrisToyRobotTest, BinarySearch) {
  const Eigen::Vector3d q_star(0, 0, 0);
  CspaceFreePolytopeTester tester(plant_, scene_graph_,
                                  SeparatingPlaneOrder::kAffine, q_star);
  // Test with initial C and d that is collision-free.
  Eigen::Matrix<double, 9, 3> C;
  // clang-format off
  C << 1, 1, 0,
       -1, -1, 0,
       -1, 0, 1,
       1, 0, -1,
       0, 1, 1,
       0, -1, -1,
       1, 0, 1,
       1, 1, -1,
       1, -1, 1;
  // clang-format on
  Eigen::Matrix<double, 9, 1> d;
  d << 0.1, 0.1, 0.1, 0.02, 0.02, 0.2, 0.1, 0.1, 0.2;

  CspaceFreePolytope::IgnoredCollisionPairs ignored_collision_pairs{
      SortedPair<geometry::GeometryId>(world_box_, body2_sphere_)};

  CspaceFreePolytope::BinarySearchOptions options;
  options.scale_min = 1;
  options.scale_max = 100;
  options.convergence_tol = 1E-1;
  options.find_lagrangian_options.parallelism = kTestConcurrency;
  solvers::MosekSolver solver;
  const Eigen::Vector3d s_center(0.01, 0, 0.01);
  auto result = tester.cspace_free_polytope().BinarySearch(
      ignored_collision_pairs, C, d, s_center, options);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->certified_polytope().ambient_dimension(), C.cols());
  EXPECT_GT(result->num_iter(), 0);
  Eigen::Matrix<double, 10, 3> s_samples;
  // clang-format off
  s_samples << 1, 2, -1,
             -0.5, 0.3, 0.2,
             0.2, 0.9, 0.4,
             0.5, -1.2, 0.3,
             0.2, 2.5, -0.4,
             -1.3, 1.5, 2,
             0.5, 0.2, 1,
             -0.4, 0.5, 1,
             0, 0, 0,
             0.2, -1.5, 1;
  // clang-format on
  EXPECT_EQ(result->a().size(),
            tester.cspace_free_polytope().separating_planes().size() -
                ignored_collision_pairs.size());
  EXPECT_EQ(result->a().size(),
            tester.cspace_free_polytope().separating_planes().size() -
                ignored_collision_pairs.size());
  CheckSeparationBySamples(tester, *diagram_, s_samples, result->C(),
                           result->d(), result->a(), result->b(), q_star,
                           ignored_collision_pairs);

  // Now test epsilon_max being feasible.
  options.scale_max = 0.02;
  options.scale_min = 0.01;
  result = tester.cspace_free_polytope().BinarySearch(ignored_collision_pairs,
                                                      C, d, s_center, options);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->num_iter(), 0);
  EXPECT_TRUE(CompareMatrices(result->C(), C));
  EXPECT_TRUE(CompareMatrices(
      result->d(),
      options.scale_max * d + (1 - options.scale_max) * C * s_center, 1E-10));
  EXPECT_EQ(result->a().size(),
            tester.cspace_free_polytope().separating_planes().size() -
                ignored_collision_pairs.size());
  EXPECT_EQ(result->a().size(),
            tester.cspace_free_polytope().separating_planes().size() -
                ignored_collision_pairs.size());
  CheckSeparationBySamples(tester, *diagram_, s_samples, result->C(),
                           result->d(), result->a(), result->b(), q_star,
                           ignored_collision_pairs);

  // Now check infeasible epsilon_min
  options.scale_min = 100;
  options.scale_max = 200;
  result = tester.cspace_free_polytope().BinarySearch(ignored_collision_pairs,
                                                      C, d, s_center, options);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result->num_iter(), 0);
}
}  // namespace optimization
}  // namespace geometry
}  // namespace drake

int main(int argc, char** argv) {
  // Ensure that we have the MOSEK license for the entire duration of this test,
  // so that we do not have to release and re-acquire the license for every
  // test.
  auto mosek_license = drake::solvers::MosekSolver::AcquireLicense();
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
