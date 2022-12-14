#include "drake/geometry/optimization/dev/cspace_free_polytope.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/symbolic_test_util.h"
#include "drake/geometry/optimization/dev/collision_geometry.h"
#include "drake/geometry/optimization/dev/test/c_iris_test_utilities.h"
#include "drake/multibody/rational/rational_forward_kinematics.h"
#include "drake/multibody/rational/rational_forward_kinematics_internal.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace geometry {
namespace optimization {
const double kInf = std::numeric_limits<double>::infinity();

// This is a friend class of CspaceFreePolytope, we use it to expose the private
// functions in CspaceFreePolytope for unit testing.
class CspaceFreePolytopeTester {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CspaceFreePolytopeTester)

  CspaceFreePolytopeTester(const multibody::MultibodyPlant<double>* plant,
                           const geometry::SceneGraph<double>* scene_graph,
                           SeparatingPlaneOrder plane_order)
      : cspace_free_polytope_{
            new CspaceFreePolytope(plant, scene_graph, plane_order)} {}

  const CspaceFreePolytope& cspace_free_polytope() const {
    return *cspace_free_polytope_;
  }

  void FindRedundantInequalities(
      const Eigen::MatrixXd& C, const Eigen::VectorXd& d,
      const Eigen::VectorXd& s_lower, const Eigen::VectorXd& s_upper,
      double tighten, std::unordered_set<int>* C_redundant_indices,
      std::unordered_set<int>* s_lower_redundant_indices,
      std::unordered_set<int>* s_upper_redundant_indices) const {
    cspace_free_polytope_->FindRedundantInequalities(
        C, d, s_lower, s_upper, tighten, C_redundant_indices,
        s_lower_redundant_indices, s_upper_redundant_indices);
  }

  void CalcSBoundsPolynomial(
      const Eigen::Ref<const Eigen::VectorXd>& q_star,
      VectorX<symbolic::Polynomial>* s_minus_s_lower,
      VectorX<symbolic::Polynomial>* s_upper_minus_s) const {
    cspace_free_polytope_->CalcSBoundsPolynomial(q_star, s_minus_s_lower,
                                                 s_upper_minus_s);
  }

  template <typename T>
  [[nodiscard]] VectorX<symbolic::Polynomial> CalcDminusCs(
      const Eigen::Ref<const MatrixX<T>>& C,
      const Eigen::Ref<const VectorX<T>>& d) const {
    return cspace_free_polytope_->CalcDminusCs<T>(C, d);
  }

  [[nodiscard]] CspaceFreePolytope::SeparationCertificate
  ConstructPlaneSearchProgram(
      const PlaneSeparatesGeometries& plane_geometries,
      const VectorX<symbolic::Polynomial>& d_minus_Cs,
      const VectorX<symbolic::Polynomial>& s_minus_s_lower,
      const VectorX<symbolic::Polynomial>& s_upper_minus_s,
      const std::unordered_set<int>& C_redundant_indices,
      const std::unordered_set<int>& s_lower_redundant_indices,
      const std::unordered_set<int>& s_upper_redundant_indices,
      std::unordered_map<SortedPair<multibody::BodyIndex>,
                         VectorX<symbolic::Monomial>>*
          map_body_to_monomial_basis) const {
    return cspace_free_polytope_->ConstructPlaneSearchProgram(
        plane_geometries, d_minus_Cs, s_minus_s_lower, s_upper_minus_s,
        C_redundant_indices, s_lower_redundant_indices,
        s_upper_redundant_indices, map_body_to_monomial_basis);
  }

  [[nodiscard]] CspaceFreePolytope::UnitLengthLagrangians
  AddUnitLengthConstraint(
      solvers::MathematicalProgram* prog,
      const VectorX<symbolic::Polynomial>& unit_length_vec,
      const VectorX<symbolic::Polynomial>& d_minus_Cs,
      const VectorX<symbolic::Polynomial>& s_minus_s_lower,
      const VectorX<symbolic::Polynomial>& s_upper_minus_s,
      const std::unordered_set<int>& C_redundant_indices,
      const std::unordered_set<int>& s_lower_redundant_indices,
      const std::unordered_set<int>& s_upper_redundant_indices) const {
    return cspace_free_polytope_->AddUnitLengthConstraint(
        prog, unit_length_vec, d_minus_Cs, s_minus_s_lower, s_upper_minus_s,
        C_redundant_indices, s_lower_redundant_indices,
        s_upper_redundant_indices);
  }

 private:
  std::unique_ptr<CspaceFreePolytope> cspace_free_polytope_;
};

// Project s_sample to the polytope {s | C*s<=d, s_lower<=s<=s_upper}.
[[nodiscard]] Eigen::VectorXd ProjectToPolytope(
    const Eigen::VectorXd& s_sample, const Eigen::Ref<const Eigen::MatrixXd>& C,
    const Eigen::Ref<const Eigen::VectorXd>& d,
    const Eigen::Ref<const Eigen::VectorXd>& s_lower,
    const Eigen::Ref<const Eigen::VectorXd>& s_upper) {
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

// Evaluate the polynomial at a batch of samples, check if all evaluated results
// are positive.
// @param x_samples Each column is a sample of indeterminates.
void CheckPositivePolynomialBySamples(
    const symbolic::Polynomial& poly,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& indeterminates,
    const Eigen::Ref<const Eigen::MatrixXd>& x_samples) {
  EXPECT_TRUE(
      (poly.EvaluateIndeterminates(indeterminates, x_samples).array() >= 0)
          .all());
}

TEST_F(CIrisToyRobotTest, GetCollisionGeometries) {
  const auto link_geometries = GetCollisionGeometries(*plant_, *scene_graph_);
  // Each link has some geometries.
  EXPECT_EQ(link_geometries.size(), plant_->num_bodies());

  auto check_link_geometries =
      [&link_geometries](const multibody::BodyIndex body,
                         const std::unordered_set<geometry::GeometryId>&
                             geometry_ids_expected) {
        auto it = link_geometries.find(body);
        std::unordered_set<geometry::GeometryId> geometry_ids;
        for (const auto& geometry : it->second) {
          EXPECT_EQ(geometry->body_index(), body);
          geometry_ids.emplace(geometry->id());
        }
        EXPECT_EQ(geometry_ids.size(), geometry_ids_expected.size());
        for (const auto id : geometry_ids) {
          EXPECT_GT(geometry_ids_expected.count(id), 0);
        }
      };

  check_link_geometries(plant_->world_body().index(),
                        {world_box_, world_sphere_});
  check_link_geometries(body_indices_[0], {body0_box_, body0_sphere_});
  check_link_geometries(body_indices_[1], {body1_convex_, body1_capsule_});
  check_link_geometries(body_indices_[2], {body2_sphere_, body2_capsule_});
  check_link_geometries(body_indices_[3], {body3_box_, body3_sphere_});
}

TEST_F(CIrisToyRobotTest, CspaceFreePolytopeConstructor) {
  // Test CspaceFreePolytope constructor.
  CspaceFreePolytope dut(plant_, scene_graph_, SeparatingPlaneOrder::kAffine);
  int num_planes_expected = 0;

  const auto link_geometries = GetCollisionGeometries(*plant_, *scene_graph_);
  // Count the expected number of planes by hand.
  num_planes_expected +=
      link_geometries.at(plant_->world_body().index()).size() *
      // Don't include world_body to body0 as there is only a weld joint between
      // them.
      (link_geometries.at(body_indices_[1]).size() +
       link_geometries.at(body_indices_[2]).size() +
       link_geometries.at(body_indices_[3]).size());
  num_planes_expected += link_geometries.at(body_indices_[0]).size() *
                         link_geometries.at(body_indices_[2]).size();
  num_planes_expected += link_geometries.at(body_indices_[1]).size() *
                         link_geometries.at(body_indices_[3]).size();
  num_planes_expected += link_geometries.at(body_indices_[2]).size() *
                         link_geometries.at(body_indices_[3]).size();
  EXPECT_EQ(dut.separating_planes().size(), num_planes_expected);

  const symbolic::Variables s_set{dut.rational_forward_kin().s()};

  for (const auto& [geometry_pair, plane_index] :
       dut.map_geometries_to_separating_planes()) {
    const auto& plane = dut.separating_planes()[plane_index];
    if (plane.positive_side_geometry->id() <
        plane.negative_side_geometry->id()) {
      EXPECT_EQ(geometry_pair.first(), plane.positive_side_geometry->id());
      EXPECT_EQ(geometry_pair.second(), plane.negative_side_geometry->id());
    } else {
      EXPECT_EQ(geometry_pair.first(), plane.negative_side_geometry->id());
      EXPECT_EQ(geometry_pair.second(), plane.positive_side_geometry->id());
    }
    // Check the expressed body.
    EXPECT_EQ(plane.expressed_body,
              multibody::internal::FindBodyInTheMiddleOfChain(
                  *plant_, plane.positive_side_geometry->body_index(),
                  plane.negative_side_geometry->body_index()));
    for (int i = 0; i < 3; ++i) {
      EXPECT_EQ(plane.a(i).TotalDegree(), 1);
      EXPECT_EQ(plane.a(i).indeterminates(), s_set);
    }
    EXPECT_EQ(plane.b.TotalDegree(), 1);
    EXPECT_EQ(plane.b.indeterminates(), s_set);
  }
}

TEST_F(CIrisToyRobotTest, CspaceFreePolytopeGenerateRationals) {
  CspaceFreePolytope dut(plant_, scene_graph_, SeparatingPlaneOrder::kAffine);
  Eigen::Vector3d q_star(0, 0, 0);
  CspaceFreePolytope::FilteredCollsionPairs filtered_collision_pairs = {};
  std::optional<symbolic::Variable> separating_margin{std::nullopt};
  auto ret = dut.GenerateRationals(q_star, filtered_collision_pairs,
                                   separating_margin);
  EXPECT_EQ(ret.size(), dut.separating_planes().size());
  for (const auto& plane_geometries : ret) {
    const auto& plane = dut.separating_planes()[plane_geometries.plane_index];
    if (plane.positive_side_geometry->type() == GeometryType::kCylinder ||
        plane.negative_side_geometry->type() == GeometryType::kCylinder) {
      throw std::runtime_error(
          "Cylinder has not been implemented yet for C-IRIS.");
    } else if (plane.positive_side_geometry->type() == GeometryType::kSphere ||
               plane.positive_side_geometry->type() == GeometryType::kCapsule ||
               plane.negative_side_geometry->type() == GeometryType::kSphere ||
               plane.negative_side_geometry->type() == GeometryType::kCapsule) {
      // If one of the geometry is sphere or capsule, and neither geometry is
      // cylinder, then the unit length vector is just a.
      EXPECT_EQ(plane_geometries.unit_length_vectors.size(), 1);
      for (int i = 0; i < 3; ++i) {
        EXPECT_EQ(plane_geometries.unit_length_vectors[0][i], plane.a(i));
      }
    } else if (plane.positive_side_geometry->type() ==
                   GeometryType::kPolytope &&
               plane.negative_side_geometry->type() ==
                   GeometryType::kPolytope) {
      EXPECT_TRUE(plane_geometries.unit_length_vectors.empty());
    }
    EXPECT_EQ(plane_geometries.positive_side_rationals.size(),
              plane.positive_side_geometry->num_rationals_per_side());
    EXPECT_EQ(plane_geometries.negative_side_rationals.size(),
              plane.negative_side_geometry->num_rationals_per_side());
  }

  // Pass a non-empty filtered_collision_pairs with a separating margin.
  filtered_collision_pairs.emplace(
      SortedPair<geometry::GeometryId>(world_box_, body3_sphere_));
  separating_margin.emplace(symbolic::Variable("delta"));
  ret = dut.GenerateRationals(q_star, filtered_collision_pairs,
                              separating_margin);
  EXPECT_EQ(ret.size(), dut.separating_planes().size() - 1);
  for (const auto& plane_geometries : ret) {
    const auto& plane = dut.separating_planes()[plane_geometries.plane_index];
    if (plane.positive_side_geometry->type() != GeometryType::kCylinder &&
        plane.negative_side_geometry->type() != GeometryType::kCylinder) {
      // The unit length vector is always a.
      EXPECT_EQ(plane_geometries.unit_length_vectors.size(), 1);
      EXPECT_EQ(plane_geometries.unit_length_vectors[0].rows(), 3);
      for (int i = 0; i < 3; ++i) {
        EXPECT_EQ(plane_geometries.unit_length_vectors[0](i), plane.a(i));
      }
    }
  }
}

TEST_F(CIrisToyRobotTest, FindRedundantInequalities) {
  // Test CspaceFreePolytope::FindRedundantInequalities.
  CspaceFreePolytopeTester tester(plant_, scene_graph_,
                                  SeparatingPlaneOrder::kAffine);
  Eigen::Matrix3d C;
  // clang-format off
  C << 1, 1, 0,
       0, 1, 1,
       1, 0, 1;
  // clang-format on
  Eigen::Vector3d d(2, 0.5, 0.1);
  Eigen::Vector3d s_lower(0, 0, 0);
  Eigen::Vector3d s_upper(0.5, 0.45, 1);
  std::unordered_set<int> C_redundant_indices;
  std::unordered_set<int> s_lower_redundant_indices;
  std::unordered_set<int> s_upper_redundant_indices;
  tester.FindRedundantInequalities(
      C, d, s_lower, s_upper, 0., &C_redundant_indices,
      &s_lower_redundant_indices, &s_upper_redundant_indices);
  EXPECT_EQ(C_redundant_indices, std::unordered_set<int>({0}));
  EXPECT_TRUE(s_lower_redundant_indices.empty());
  EXPECT_EQ(s_upper_redundant_indices, std::unordered_set<int>({0, 2}));
}

TEST_F(CIrisToyRobotTest, CalcDminusCs) {
  Eigen::Matrix<symbolic::Variable, 2, 3> C;
  Vector2<symbolic::Variable> d;
  for (int i = 0; i < 2; ++i) {
    d(i) = symbolic::Variable("d" + std::to_string(i));
    for (int j = 0; j < 3; ++j) {
      C(i, j) = symbolic::Variable(fmt::format("C{}{}", i, j));
    }
  }
  CspaceFreePolytopeTester tester(plant_, scene_graph_,
                                  SeparatingPlaneOrder::kAffine);
  const auto& s = tester.cspace_free_polytope().rational_forward_kin().s();

  const auto ret = tester.CalcDminusCs<symbolic::Variable>(C, d);
  EXPECT_EQ(ret.rows(), 2);
  for (int i = 0; i < 2; ++i) {
    EXPECT_PRED2(symbolic::test::PolyEqual, ret(i),
                 symbolic::Polynomial(
                     d(i) - C.row(i).dot(s.cast<symbolic::Expression>()),
                     symbolic::Variables(s)));
  }
}

TEST_F(CIrisToyRobotTest, CalcSBoundsPolynomial) {
  CspaceFreePolytopeTester tester(plant_, scene_graph_,
                                  SeparatingPlaneOrder::kAffine);
  VectorX<symbolic::Polynomial> s_minus_s_lower;
  VectorX<symbolic::Polynomial> s_upper_minus_s;
  const Eigen::Vector3d q_star(0, 0, 0);
  tester.CalcSBoundsPolynomial(q_star, &s_minus_s_lower, &s_upper_minus_s);
  EXPECT_EQ(s_minus_s_lower.rows(), 3);
  EXPECT_EQ(s_upper_minus_s.rows(), 3);
  const Eigen::Vector3d s_lower =
      tester.cspace_free_polytope().rational_forward_kin().ComputeSValue(

          plant_->GetPositionLowerLimits(), q_star);
  const Eigen::Vector3d s_upper =
      tester.cspace_free_polytope().rational_forward_kin().ComputeSValue(
          plant_->GetPositionUpperLimits(), q_star);
  const auto& s = tester.cspace_free_polytope().rational_forward_kin().s();
  for (int i = 0; i < 3; ++i) {
    EXPECT_PRED2(symbolic::test::PolyEqual, s_minus_s_lower(i),
                 symbolic::Polynomial((s(i) - s_lower(i))));
    EXPECT_PRED2(symbolic::test::PolyEqual, s_upper_minus_s(i),
                 symbolic::Polynomial((s_upper(i) - s(i))));
  }
}

void SetupPolytope(const CspaceFreePolytopeTester& tester,
                   const Eigen::Ref<const Eigen::MatrixXd>& C,
                   const Eigen::Ref<const Eigen::VectorXd>& d,
                   const Eigen::Ref<const Eigen::VectorXd>& q_star,
                   VectorX<symbolic::Polynomial>* d_minus_Cs,
                   VectorX<symbolic::Polynomial>* s_minus_s_lower,
                   VectorX<symbolic::Polynomial>* s_upper_minus_s,
                   std::unordered_set<int>* C_redundant_indices,
                   std::unordered_set<int>* s_lower_redundant_indices,
                   std::unordered_set<int>* s_upper_redundant_indices) {
  *d_minus_Cs = tester.CalcDminusCs<double>(C, d);
  tester.CalcSBoundsPolynomial(q_star, s_minus_s_lower, s_upper_minus_s);
  const auto& plant =
      tester.cspace_free_polytope().rational_forward_kin().plant();
  const Eigen::VectorXd s_lower =
      tester.cspace_free_polytope().rational_forward_kin().ComputeSValue(
          plant.GetPositionLowerLimits(), q_star);
  const Eigen::VectorXd s_upper =
      tester.cspace_free_polytope().rational_forward_kin().ComputeSValue(
          plant.GetPositionUpperLimits(), q_star);
  tester.FindRedundantInequalities(
      C, d, s_lower, s_upper, 0., C_redundant_indices,
      s_lower_redundant_indices, s_upper_redundant_indices);
}

TEST_F(CIrisToyRobotTest, ConstructPlaneSearchProgram1) {
  // Test ConstructPlaneSearchProgram with no unit-length-vector constraint.
  CspaceFreePolytopeTester tester(plant_, scene_graph_,
                                  SeparatingPlaneOrder::kAffine);
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
  d << 0.1, 0.2, 0.3, 0.2, 0.2, 0.2, 0.1, 0.1, 0.2;

  VectorX<symbolic::Polynomial> d_minus_Cs;
  VectorX<symbolic::Polynomial> s_minus_s_lower;
  VectorX<symbolic::Polynomial> s_upper_minus_s;
  const Eigen::Vector3d q_star(0, 0, 0);
  std::unordered_set<int> C_redundant_indices;
  std::unordered_set<int> s_lower_redundant_indices;
  std::unordered_set<int> s_upper_redundant_indices;
  SetupPolytope(tester, C, d, q_star, &d_minus_Cs, &s_minus_s_lower,
                &s_upper_minus_s, &C_redundant_indices,
                &s_lower_redundant_indices, &s_upper_redundant_indices);

  std::unordered_map<SortedPair<multibody::BodyIndex>,
                     VectorX<symbolic::Monomial>>
      map_body_to_monomial_basis;
  const auto plane_geometries_vec =
      tester.cspace_free_polytope().GenerateRationals(q_star, {}, std::nullopt);
  // Consider the plane between world_box_ and body3_box_.
  // Notice that this chain only has one DOF, hence one of the rationals is
  // actually a constant.
  int plane_geometries_index = -1;
  for (int i = 0; i < static_cast<int>(plane_geometries_vec.size()); ++i) {
    const auto& plane =
        tester.cspace_free_polytope()
            .separating_planes()[plane_geometries_vec[i].plane_index];
    if (SortedPair<geometry::GeometryId>(plane.positive_side_geometry->id(),
                                         plane.negative_side_geometry->id()) ==
        SortedPair<geometry::GeometryId>(world_box_, body3_box_)) {
      plane_geometries_index = i;
      break;
    }
  }
  const auto& plane_geometries = plane_geometries_vec[plane_geometries_index];
  auto ret = tester.ConstructPlaneSearchProgram(
      plane_geometries, d_minus_Cs, s_minus_s_lower, s_upper_minus_s,
      C_redundant_indices, s_lower_redundant_indices, s_upper_redundant_indices,
      &map_body_to_monomial_basis);

  EXPECT_EQ(map_body_to_monomial_basis.size(), 2);

  solvers::MosekSolver solver;
  if (solver.available()) {
    solvers::SolverOptions solver_options;
    auto result = solver.Solve(*(ret.prog), std::nullopt, solver_options);
    ASSERT_TRUE(result.is_success());

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

    const auto& s = tester.cspace_free_polytope().rational_forward_kin().s();

    auto check_lagrangians =
        [&s_samples, &s](const VectorX<symbolic::Polynomial>& lagrangians,
                         const std::unordered_set<int> redundant_indices) {
          for (int i = 0; i < lagrangians.rows(); ++i) {
            if (redundant_indices.count(i) == 0) {
              CheckPositivePolynomialBySamples(lagrangians(i), s,
                                               s_samples.transpose());
            } else {
              EXPECT_PRED2(symbolic::test::PolyEqual, lagrangians(i),
                           symbolic::Polynomial());
            }
          }
        };

    auto check_rational_positive_in_polytope =
        [&result, &d_minus_Cs, &s_minus_s_lower, &s_upper_minus_s,
         check_lagrangians, &C_redundant_indices, &s_lower_redundant_indices,
         &s_upper_redundant_indices, &s, &s_samples](
            const std::vector<symbolic::RationalFunction>& rationals,
            const std::vector<CspaceFreePolytope::SeparatingPlaneLagrangians>&
                plane_side_lagrangians) {
          for (int i = 0; i < static_cast<int>(rationals.size()); ++i) {
            const auto& search_plane_lagrangians = plane_side_lagrangians[i];
            const auto search_plane_lagrangians_result =
                search_plane_lagrangians.GetSolution(result);
            check_lagrangians(search_plane_lagrangians_result.polytope,
                              C_redundant_indices);
            check_lagrangians(search_plane_lagrangians_result.s_lower,
                              s_lower_redundant_indices);
            check_lagrangians(search_plane_lagrangians_result.s_upper,
                              s_upper_redundant_indices);
            const symbolic::Polynomial poly =
                result.GetSolution(rationals[i].numerator()) -
                search_plane_lagrangians_result.polytope.dot(d_minus_Cs) -
                search_plane_lagrangians_result.s_lower.dot(s_minus_s_lower) -
                search_plane_lagrangians_result.s_upper.dot(s_upper_minus_s);
            CheckPositivePolynomialBySamples(poly, s, s_samples.transpose());
          }
        };
    check_rational_positive_in_polytope(
        plane_geometries.positive_side_rationals,
        ret.positive_side_lagrangians);
    check_rational_positive_in_polytope(
        plane_geometries.negative_side_rationals,
        ret.negative_side_lagrangians);
  }
}

TEST_F(CIrisToyRobotTest, ConstructPlaneSearchProgram2) {
  // Test ConstructPlaneSearchProgram with unit-length-vector constraint.
  CspaceFreePolytopeTester tester(plant_, scene_graph_,
                                  SeparatingPlaneOrder::kAffine);
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
  d << 0.1, 0.2, 0.3, 0.2, 0.2, 0.2, 0.1, 0.1, 0.2;

  VectorX<symbolic::Polynomial> d_minus_Cs;
  VectorX<symbolic::Polynomial> s_minus_s_lower;
  VectorX<symbolic::Polynomial> s_upper_minus_s;
  const Eigen::Vector3d q_star(0, 0, 0);
  std::unordered_set<int> C_redundant_indices;
  std::unordered_set<int> s_lower_redundant_indices;
  std::unordered_set<int> s_upper_redundant_indices;
  SetupPolytope(tester, C, d, q_star, &d_minus_Cs, &s_minus_s_lower,
                &s_upper_minus_s, &C_redundant_indices,
                &s_lower_redundant_indices, &s_upper_redundant_indices);

  std::unordered_map<SortedPair<multibody::BodyIndex>,
                     VectorX<symbolic::Monomial>>
      map_body_to_monomial_basis;
  const auto plane_geometries_vec =
      tester.cspace_free_polytope().GenerateRationals(q_star, {}, std::nullopt);
  // Consider the plane between world_sphere_ and body2_capsule_.
  int plane_geometries_index = -1;
  for (int i = 0; i < static_cast<int>(plane_geometries_vec.size()); ++i) {
    const auto& plane =
        tester.cspace_free_polytope()
            .separating_planes()[plane_geometries_vec[i].plane_index];
    if (SortedPair<geometry::GeometryId>(plane.positive_side_geometry->id(),
                                         plane.negative_side_geometry->id()) ==
        SortedPair<geometry::GeometryId>(world_sphere_, body2_capsule_)) {
      plane_geometries_index = i;
      break;
    }
  }
  const auto& plane_geometries = plane_geometries_vec[plane_geometries_index];
  auto ret = tester.ConstructPlaneSearchProgram(
      plane_geometries, d_minus_Cs, s_minus_s_lower, s_upper_minus_s,
      C_redundant_indices, s_lower_redundant_indices, s_upper_redundant_indices,
      &map_body_to_monomial_basis);
  EXPECT_EQ(map_body_to_monomial_basis.size(), 2);
  EXPECT_EQ(ret.unit_length_lagrangians.size(),
            plane_geometries.unit_length_vectors.size());

  solvers::MosekSolver solver;
  if (solver.available()) {
    const auto result = solver.Solve(*(ret.prog));
    ASSERT_TRUE(result.is_success());
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
    const Eigen::Vector3d s_lower =
        tester.cspace_free_polytope().rational_forward_kin().ComputeSValue(
            plant_->GetPositionLowerLimits(), q_star);
    const Eigen::Vector3d s_upper =
        tester.cspace_free_polytope().rational_forward_kin().ComputeSValue(
            plant_->GetPositionUpperLimits(), q_star);
    for (int i = 0; i < s_samples.rows(); ++i) {
      const Eigen::Vector3d s_val = ProjectToPolytope(
          s_samples.row(i).transpose(), C, d, s_lower, s_upper);
      // Check if the unit_length_vector really has norm <= 1.
      for (const auto& unit_length_vec : plane_geometries.unit_length_vectors) {
        Vector3<symbolic::Polynomial> unit_length_vec_result;
        for (int j = 0; j < 3; ++j) {
          unit_length_vec_result(j) = result.GetSolution(unit_length_vec(j));
        }
        Eigen::Vector3d unit_length_vec_val;
        symbolic::Environment env;
        env.insert(tester.cspace_free_polytope().rational_forward_kin().s(),
                   s_val);
        for (int j = 0; j < 3; ++j) {
          unit_length_vec_val(j) = unit_length_vec_result(j).Evaluate(env);
        }
        EXPECT_LE(unit_length_vec_val.norm(), 1);
      }
    }
  }
}

TEST_F(CIrisToyRobotTest, AddUnitLengthConstraint) {
  // Test CspaceFreePolytope::AddUnitLengthConstraint
  CspaceFreePolytopeTester tester(plant_, scene_graph_,
                                  SeparatingPlaneOrder::kAffine);
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
  d << 0.1, 0.2, 0.3, 0.2, 0.2, 0.2, 0.1, 0.1, 0.2;

  VectorX<symbolic::Polynomial> d_minus_Cs;
  VectorX<symbolic::Polynomial> s_minus_s_lower;
  VectorX<symbolic::Polynomial> s_upper_minus_s;
  const Eigen::Vector3d q_star(0, 0, 0);
  std::unordered_set<int> C_redundant_indices;
  std::unordered_set<int> s_lower_redundant_indices;
  std::unordered_set<int> s_upper_redundant_indices;
  SetupPolytope(tester, C, d, q_star, &d_minus_Cs, &s_minus_s_lower,
                &s_upper_minus_s, &C_redundant_indices,
                &s_lower_redundant_indices, &s_upper_redundant_indices);
  solvers::MathematicalProgram prog;
  prog.AddIndeterminates(
      tester.cspace_free_polytope().rational_forward_kin().s());

  const auto plane_geometries_vec =
      tester.cspace_free_polytope().GenerateRationals(q_star, {}, std::nullopt);
  // Consider the separating plane between world_sphere_ and body2_capsule_.
  int plane_geometries_index = -1;
  for (int i = 0; i < static_cast<int>(plane_geometries_vec.size()); ++i) {
    const auto& plane =
        tester.cspace_free_polytope()
            .separating_planes()[plane_geometries_vec[i].plane_index];
    if (SortedPair<geometry::GeometryId>(plane.positive_side_geometry->id(),
                                         plane.negative_side_geometry->id()) ==
        SortedPair<geometry::GeometryId>(world_sphere_, body2_capsule_)) {
      plane_geometries_index = i;
      prog.AddDecisionVariables(plane.decision_variables);
      break;
    }
  }
  const auto& plane_geometries = plane_geometries_vec[plane_geometries_index];

  prog.AddIndeterminates(tester.cspace_free_polytope().y_slack());
  const auto ret = tester.AddUnitLengthConstraint(
      &prog, plane_geometries.unit_length_vectors[0], d_minus_Cs,
      s_minus_s_lower, s_upper_minus_s, C_redundant_indices,
      s_lower_redundant_indices, s_upper_redundant_indices);
  EXPECT_EQ(ret.polytope.rows(), C.rows());
  const int s_size =
      tester.cspace_free_polytope().rational_forward_kin().s().rows();
  EXPECT_EQ(ret.s_lower.rows(), s_size);
  EXPECT_EQ(ret.s_upper.rows(), s_size);
  const symbolic::Variables y_set{tester.cspace_free_polytope().y_slack()};
  // Make sure that the Lagrangians are quadratic polynomials of y, if they are
  // not redundant.
  for (int i = 0; i < ret.polytope.rows(); ++i) {
    if (C_redundant_indices.count(i) == 0) {
      EXPECT_EQ(ret.polytope(i).indeterminates(), y_set);
      EXPECT_EQ(ret.polytope(i).TotalDegree(), 2);
    } else {
      EXPECT_PRED2(symbolic::test::PolyEqual, ret.polytope(i),
                   symbolic::Polynomial());
    }
  }
  for (int i = 0; i < s_size; ++i) {
    if (s_lower_redundant_indices.count(i) == 0) {
      EXPECT_EQ(ret.s_lower(i).indeterminates(), y_set);
      EXPECT_EQ(ret.s_lower(i).TotalDegree(), 2);
    } else {
      EXPECT_PRED2(symbolic::test::PolyEqual, ret.s_lower(i),
                   symbolic::Polynomial());
    }
    if (s_upper_redundant_indices.count(i) == 0) {
      EXPECT_EQ(ret.s_upper(i).indeterminates(), y_set);
      EXPECT_EQ(ret.s_upper(i).TotalDegree(), 2);
    } else {
      EXPECT_PRED2(symbolic::test::PolyEqual, ret.s_upper(i),
                   symbolic::Polynomial());
    }
  }
  const symbolic::Variables s_set(
      tester.cspace_free_polytope().rational_forward_kin().s());
  EXPECT_EQ(ret.y_square.indeterminates(), s_set);
  EXPECT_EQ(ret.y_square.TotalDegree(), 1);

  solvers::MosekSolver solver;
  if (solver.available()) {
    const auto result = solver.Solve(prog);
    ASSERT_TRUE(result.is_success());
    Vector3<symbolic::Polynomial> unit_length_vec;
    for (int i = 0; i < 3; ++i) {
      unit_length_vec(i) =
          result.GetSolution(plane_geometries.unit_length_vectors[0](i));
    }
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
    const Eigen::Vector3d s_lower =
        tester.cspace_free_polytope().rational_forward_kin().ComputeSValue(
            plant_->GetPositionLowerLimits(), q_star);
    const Eigen::Vector3d s_upper =
        tester.cspace_free_polytope().rational_forward_kin().ComputeSValue(
            plant_->GetPositionUpperLimits(), q_star);
    for (int i = 0; i < s_samples.rows(); ++i) {
      const Eigen::Vector3d s_val = ProjectToPolytope(
          s_samples.row(i).transpose(), C, d, s_lower, s_upper);
      Eigen::Vector3d unit_length_vec_val;
      symbolic::Environment env;
      env.insert(tester.cspace_free_polytope().rational_forward_kin().s(),
                 s_val);
      for (int j = 0; j < 3; ++j) {
        unit_length_vec_val(j) = unit_length_vec(j).Evaluate(env);
      }
      EXPECT_LE(unit_length_vec_val.norm(), 1);
    }
  }
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
