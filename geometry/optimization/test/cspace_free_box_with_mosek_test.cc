#include "drake/common/fmt_eigen.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/symbolic_test_util.h"
#include "drake/geometry/optimization/cspace_free_box.h"
#include "drake/geometry/optimization/test/c_iris_test_utilities.h"
#include "drake/solvers/mosek_solver.h"

namespace drake {
namespace geometry {
namespace optimization {
namespace {
// @param q_samples Each row of s_samples is a sampled configuration q.
// @param a Maps the plane index to the separating plane parameter `a` in {x|
// aᵀx+b=0}
// @param b Maps the plane index to the separating plane parameter `b` in {x|
// aᵀx+b=0}
// @param geometry_pair
void CheckSeparationBySamples(
    const CspaceFreeBoxTester& tester, const systems::Diagram<double>& diagram,
    const Eigen::Ref<const Eigen::MatrixXd>& q_samples,
    const Vector3<symbolic::Polynomial>& a, const symbolic::Polynomial& b,
    const Eigen::VectorXd& q_star,
    const SortedPair<geometry::GeometryId>& geometry_pair) {
  auto diagram_context = diagram.CreateDefaultContext();
  const auto& plant = tester.cspace_free_box().rational_forward_kin().plant();
  auto& plant_context =
      plant.GetMyMutableContextFromRoot(diagram_context.get());

  for (int i = 0; i < q_samples.rows(); ++i) {
    const Eigen::Vector3d s_val =
        tester.cspace_free_box().rational_forward_kin().ComputeSValue(
            q_samples.row(i).transpose(), q_star);
    symbolic::Environment env;
    env.insert(tester.cspace_free_box().rational_forward_kin().s(), s_val);
    plant.SetPositions(&plant_context, q_samples.row(i).transpose());
    const int plane_index =
        tester.cspace_free_box().map_geometries_to_separating_planes().at(
            geometry_pair);
    const auto& plane =
        tester.cspace_free_box().separating_planes()[plane_index];
    Eigen::Vector3d a_val;
    for (int j = 0; j < 3; ++j) {
      a_val(j) = a(j).Evaluate(env);
    }
    const double b_val = b.Evaluate(env);
    EXPECT_GE(DistanceToHalfspace(*plane.positive_side_geometry, a_val, b_val,
                                  plane.expressed_body, PlaneSide::kPositive,
                                  plant, plant_context),
              0);
    EXPECT_GE(DistanceToHalfspace(*plane.negative_side_geometry, a_val, b_val,
                                  plane.expressed_body, PlaneSide::kNegative,
                                  plant, plant_context),
              0);
  }
}

void CheckSosLagrangians(
    const std::vector<CspaceFreeBox::SeparatingPlaneLagrangians>&
        lagrangians_vec) {
  for (const auto& lagrangians : lagrangians_vec) {
    for (int i = 0; i < lagrangians.s_box_lower().rows(); ++i) {
      EXPECT_TRUE(IsPolynomialSos(lagrangians.s_box_lower()(i), 0.0));
      EXPECT_TRUE(IsPolynomialSos(lagrangians.s_box_upper()(i), 0.0));
    }
  }
}

TEST_F(CIrisToyRobotTest, ConstructPlaneSearchProgram) {
  CspaceFreeBoxTester tester(plant_, scene_graph_,
                             SeparatingPlaneOrder::kAffine);
  const Eigen::VectorXd q_position_lower = plant_->GetPositionLowerLimits();
  const Eigen::VectorXd q_position_upper = plant_->GetPositionUpperLimits();
  const Eigen::VectorXd q_box_lower =
      0.48 * q_position_lower + 0.52 * q_position_upper;
  const Eigen::VectorXd q_box_upper =
      0.47 * q_position_lower + 0.53 * q_position_upper;
  Eigen::VectorXd s_box_lower;
  Eigen::VectorXd s_box_upper;
  Eigen::VectorXd q_star;
  tester.ComputeSBox(q_box_lower, q_box_upper, &s_box_lower, &s_box_upper,
                     &q_star);
  CspaceFreeBoxTester::PolynomialsToCertify polynomials_to_certify;
  CspaceFreePolytopeBase::IgnoredCollisionPairs ignored_collision_pairs{};
  tester.GeneratePolynomialsToCertify(s_box_lower, s_box_upper, q_star,
                                      ignored_collision_pairs,
                                      &polynomials_to_certify);
  auto get_lagrangian_result =
      [](const solvers::MathematicalProgramResult& prog_result,
         const std::vector<CspaceFreeBox::SeparatingPlaneLagrangians>&
             lagrangians_vec)
      -> std::vector<CspaceFreeBox::SeparatingPlaneLagrangians> {
    std::vector<CspaceFreeBox::SeparatingPlaneLagrangians> lagrangians_result;
    for (const auto& lagrangian : lagrangians_vec) {
      lagrangians_result.push_back(lagrangian.GetSolution(prog_result));
    }
    return lagrangians_result;
  };

  auto check_certificate = [&get_lagrangian_result, &tester, this, &q_star](
                               const CspaceFreeBox::
                                   SeparationCertificateProgram&
                                       separation_certificate_program,
                               const solvers::MathematicalProgramResult& result,
                               const Eigen::MatrixXd& q_samples) {
    const auto positive_side_rational_lagrangians_result =
        get_lagrangian_result(result, separation_certificate_program.certificate
                                          .positive_side_rational_lagrangians);
    const auto negative_side_rational_lagrangians_result =
        get_lagrangian_result(result, separation_certificate_program.certificate
                                          .negative_side_rational_lagrangians);
    CheckSosLagrangians(positive_side_rational_lagrangians_result);
    CheckSosLagrangians(negative_side_rational_lagrangians_result);

    // Now check if the separation plane actually separates the geometry.
    const auto& plane =
        tester.cspace_free_box()
            .separating_planes()[separation_certificate_program.plane_index];
    Vector3<symbolic::Polynomial> a_result;
    for (int i = 0; i < 3; ++i) {
      a_result(i) = result.GetSolution(plane.a(i));
    }
    const symbolic::Polynomial b_result = result.GetSolution(plane.b);
    const SortedPair<geometry::GeometryId> geometry_pair(
        plane.positive_side_geometry->id(), plane.negative_side_geometry->id());

    CheckSeparationBySamples(tester, *(this->diagram_), q_samples, a_result,
                             b_result, q_star, geometry_pair);
  };

  {
    // Test the program for polytopic geometries.
    const int plane_index1 =
        tester.cspace_free_box().map_geometries_to_separating_planes().at(
            SortedPair<geometry::GeometryId>(world_box_, body3_box_));

    CspaceFreeBox::SeparationCertificateProgram separation_certificate_program =
        tester.ConstructPlaneSearchProgram(
            polynomials_to_certify.data.plane_geometries[plane_index1],
            polynomials_to_certify.data.s_minus_s_box_lower,
            polynomials_to_certify.data.s_box_upper_minus_s);
    // Only s(2) should be the indeterminate in the program since that is the
    // only s on the kinematics chain.
    EXPECT_EQ(separation_certificate_program.prog->indeterminates().rows(), 1);
    EXPECT_EQ(separation_certificate_program.prog->indeterminate(0),
              tester.cspace_free_box().rational_forward_kin().s()(2));
    // The Lagrangian polynomials for s(i) - s_lower(i) or s_upper(i) - s(i)
    // should be zero when i = 0 or 1.
    for (const auto& lagrangians : separation_certificate_program.certificate
                                       .positive_side_rational_lagrangians) {
      for (int i : {0, 1}) {
        EXPECT_PRED2(symbolic::test::PolyEqual, lagrangians.s_box_lower()(i),
                     symbolic::Polynomial());
        EXPECT_PRED2(symbolic::test::PolyEqual, lagrangians.s_box_upper()(i),
                     symbolic::Polynomial());
      }
    }
    for (const auto& lagrangians : separation_certificate_program.certificate
                                       .negative_side_rational_lagrangians) {
      for (int i : {0, 1}) {
        EXPECT_PRED2(symbolic::test::PolyEqual, lagrangians.s_box_lower()(i),
                     symbolic::Polynomial());
        EXPECT_PRED2(symbolic::test::PolyEqual, lagrangians.s_box_upper()(i),
                     symbolic::Polynomial());
      }
    }

    solvers::MosekSolver mosek_solver;
    const auto result =
        mosek_solver.Solve(*separation_certificate_program.prog);
    EXPECT_TRUE(result.is_success());

    Eigen::MatrixXd q_samples(100, 3);
    q_samples.setZero();
    q_samples.col(2) = Eigen::VectorXd::LinSpaced(
        q_samples.rows(), q_box_lower(2), q_box_upper(2));

    check_certificate(separation_certificate_program, result, q_samples);
  }

  {
    // Test the program for non-polytopic geometries.
    const int plane_index2 =
        tester.cspace_free_box().map_geometries_to_separating_planes().at(
            SortedPair<geometry::GeometryId>(world_cylinder_, body2_sphere_));

    CspaceFreeBox::SeparationCertificateProgram separation_certificate_program =
        tester.ConstructPlaneSearchProgram(
            polynomials_to_certify.data.plane_geometries[plane_index2],
            polynomials_to_certify.data.s_minus_s_box_lower,
            polynomials_to_certify.data.s_box_upper_minus_s);
    // s(0) s(1) and y should be the indeterminate in the program since that is
    // the only s on the kinematics chain.
    EXPECT_EQ(separation_certificate_program.prog->indeterminates().rows(), 5);
    for (int i = 0;
         i < separation_certificate_program.prog->indeterminates().size();
         ++i) {
      EXPECT_NE(
          separation_certificate_program.prog->indeterminate(i).get_id(),
          tester.cspace_free_box().rational_forward_kin().s()(2).get_id());
    }
    // The Lagrangian polynomials for s(i) - s_lower(i) or s_upper(i) - s(i)
    // should be zero when i = 2
    for (const auto& lagrangians : separation_certificate_program.certificate
                                       .positive_side_rational_lagrangians) {
      EXPECT_PRED2(symbolic::test::PolyEqual, lagrangians.s_box_lower()(2),
                   symbolic::Polynomial());
      EXPECT_PRED2(symbolic::test::PolyEqual, lagrangians.s_box_upper()(2),
                   symbolic::Polynomial());
    }
    for (const auto& lagrangians : separation_certificate_program.certificate
                                       .negative_side_rational_lagrangians) {
      EXPECT_PRED2(symbolic::test::PolyEqual, lagrangians.s_box_lower()(2),
                   symbolic::Polynomial());
      EXPECT_PRED2(symbolic::test::PolyEqual, lagrangians.s_box_upper()(2),
                   symbolic::Polynomial());
    }

    solvers::MosekSolver mosek_solver;
    const auto result =
        mosek_solver.Solve(*separation_certificate_program.prog);
    EXPECT_TRUE(result.is_success());

    Eigen::MatrixXd q_samples(100, 3);
    q_samples.setZero();
    q_samples.col(0) = Eigen::VectorXd::LinSpaced(
        q_samples.rows(), q_box_lower(0), q_box_upper(0));
    q_samples.col(1) = Eigen::VectorXd::LinSpaced(
        q_samples.rows(), q_box_lower(1), q_box_upper(1));

    check_certificate(separation_certificate_program, result, q_samples);
  }
}
}  // namespace
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
