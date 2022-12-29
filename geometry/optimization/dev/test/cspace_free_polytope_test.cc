#include "drake/geometry/optimization/dev/cspace_free_polytope.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/symbolic_test_util.h"
#include "drake/geometry/collision_filter_declaration.h"
#include "drake/geometry/geometry_ids.h"
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
                           SeparatingPlaneOrder plane_order,
                           const Eigen::Ref<const Eigen::VectorXd>& q_star)
      : cspace_free_polytope_{
            new CspaceFreePolytope(plant, scene_graph, plane_order, q_star)} {}

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

  const Eigen::VectorXd& s_lower() const {
    return cspace_free_polytope_->s_lower_;
  }

  const Eigen::VectorXd& s_upper() const {
    return cspace_free_polytope_->s_upper_;
  }

  const VectorX<symbolic::Polynomial>& s_minus_s_lower() const {
    return cspace_free_polytope_->s_minus_s_lower_;
  }

  const VectorX<symbolic::Polynomial>& s_upper_minus_s() const {
    return cspace_free_polytope_->s_upper_minus_s_;
  }

  const std::vector<PlaneSeparatesGeometries>& plane_geometries_w_margin()
      const {
    return cspace_free_polytope_->plane_geometries_w_margin_;
  }

  const std::vector<PlaneSeparatesGeometries>& plane_geometries_wo_margin()
      const {
    return cspace_free_polytope_->plane_geometries_wo_margin_;
  }

  template <typename T>
  [[nodiscard]] VectorX<symbolic::Polynomial> CalcDminusCs(
      const Eigen::Ref<const MatrixX<T>>& C,
      const Eigen::Ref<const VectorX<T>>& d) const {
    return cspace_free_polytope_->CalcDminusCs<T>(C, d);
  }

  [[nodiscard]] const std::unordered_map<SortedPair<multibody::BodyIndex>,
                                         VectorX<symbolic::Monomial>>&
  map_body_to_monomial_basis() const {
    return cspace_free_polytope_->map_body_to_monomial_basis_;
  }

  [[nodiscard]] CspaceFreePolytope::SeparationCertificate
  ConstructPlaneSearchProgram(
      const PlaneSeparatesGeometries& plane_geometries,
      const VectorX<symbolic::Polynomial>& d_minus_Cs,
      const std::unordered_set<int>& C_redundant_indices,
      const std::unordered_set<int>& s_lower_redundant_indices,
      const std::unordered_set<int>& s_upper_redundant_indices) const {
    return cspace_free_polytope_->ConstructPlaneSearchProgram(
        plane_geometries, d_minus_Cs, C_redundant_indices,
        s_lower_redundant_indices, s_upper_redundant_indices);
  }

  [[nodiscard]] CspaceFreePolytope::UnitLengthLagrangians
  AddUnitLengthConstraint(
      solvers::MathematicalProgram* prog,
      const VectorX<symbolic::Polynomial>& unit_length_vec,
      const VectorX<symbolic::Polynomial>& d_minus_Cs,
      const std::unordered_set<int>& C_redundant_indices,
      const std::unordered_set<int>& s_lower_redundant_indices,
      const std::unordered_set<int>& s_upper_redundant_indices,
      const std::optional<VectorX<symbolic::Polynomial>>& polytope_lagrangians)
      const {
    return cspace_free_polytope_->AddUnitLengthConstraint(
        prog, unit_length_vec, d_minus_Cs, C_redundant_indices,
        s_lower_redundant_indices, s_upper_redundant_indices,
        polytope_lagrangians);
  }

  [[nodiscard]] std::vector<
      std::optional<CspaceFreePolytope::SeparationCertificateResult>>
  FindSeparationCertificateGivenPolytope(
      const CspaceFreePolytope::IgnoredCollisionPairs& ignored_collision_pairs,
      const Eigen::Ref<const Eigen::MatrixXd>& C,
      const Eigen::Ref<const Eigen::VectorXd>& d, bool search_separating_margin,
      const CspaceFreePolytope::FindSeparationCertificateGivenPolytopeOptions&
          options) const {
    return cspace_free_polytope_->FindSeparationCertificateGivenPolytope(
        ignored_collision_pairs, C, d, search_separating_margin, options);
  }

  [[nodiscard]] std::vector<PlaneSeparatesGeometries> GenerateRationals(
      bool search_separating_margin) const {
    return cspace_free_polytope_->GenerateRationals(search_separating_margin);
  }

  [[nodiscard]] int GetGramVarSizeForPolytopeSearchProgram(
      const CspaceFreePolytope::IgnoredCollisionPairs& ignored_collision_pairs)
      const {
    return cspace_free_polytope_->GetGramVarSizeForPolytopeSearchProgram(
        ignored_collision_pairs);
  }

  [[nodiscard]] std::unique_ptr<solvers::MathematicalProgram>
  InitializePolytopeSearchProgram(
      const CspaceFreePolytope::IgnoredCollisionPairs& ignored_collision_pairs,
      const MatrixX<symbolic::Variable>& C,
      const VectorX<symbolic::Variable>& d,
      const VectorX<symbolic::Polynomial> d_minus_Cs,
      const std::vector<
          std::optional<CspaceFreePolytope::SeparationCertificateResult>>&
          certificates_vec,
      int gram_total_size) const {
    return cspace_free_polytope_->InitializePolytopeSearchProgram(
        ignored_collision_pairs, C, d, d_minus_Cs, certificates_vec,
        gram_total_size);
  }

  void AddEllipsoidContainmentConstraint(
      solvers::MathematicalProgram* prog, const Eigen::MatrixXd& Q,
      const Eigen::VectorXd& s0, const MatrixX<symbolic::Variable>& C,
      const VectorX<symbolic::Variable>& d,
      const VectorX<symbolic::Variable>& ellipsoid_margins) const {
    cspace_free_polytope_->AddEllipsoidContainmentConstraint(prog, Q, s0, C, d,
                                                             ellipsoid_margins);
  }

  void AddCspacePolytopeContainment(solvers::MathematicalProgram* prog,
                                    const MatrixX<symbolic::Variable>& C,
                                    const VectorX<symbolic::Variable>& d,
                                    const Eigen::MatrixXd& s_inner_pts) const {
    cspace_free_polytope_->AddCspacePolytopeContainment(prog, C, d,
                                                        s_inner_pts);
  }
  [[nodiscard]] std::optional<
      CspaceFreePolytope::FindPolytopeGivenLagrangianResult>
  FindPolytopeGivenLagrangian(
      const CspaceFreePolytope::IgnoredCollisionPairs& ignored_collision_pairs,
      const MatrixX<symbolic::Variable>& C,
      const VectorX<symbolic::Variable>& d,
      const VectorX<symbolic::Polynomial>& d_minus_Cs,
      const std::vector<
          std::optional<CspaceFreePolytope::SeparationCertificateResult>>&
          certificates_vec,
      const Eigen::MatrixXd& Q, const Eigen::VectorXd& s0,
      const VectorX<symbolic::Variable>& ellipsoid_margins, int gram_total_size,
      const CspaceFreePolytope::FindPolytopeGivenLagrangianOptions& options)
      const {
    return cspace_free_polytope_->FindPolytopeGivenLagrangian(
        ignored_collision_pairs, C, d, d_minus_Cs, certificates_vec, Q, s0,
        ellipsoid_margins, gram_total_size, options);
  }

  HPolyhedron GetPolyhedronWithJointLimits(const Eigen::MatrixXd& C,
                                           const Eigen::VectorXd& d) const {
    return cspace_free_polytope_->GetPolyhedronWithJointLimits(C, d);
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
  const Eigen::Vector3d q_star(0, 0, 0);
  CspaceFreePolytopeTester tester(plant_, scene_graph_,
                                  SeparatingPlaneOrder::kAffine, q_star);
  const CspaceFreePolytope& dut = tester.cspace_free_polytope();
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
    // check plane
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

    // ensure the correct number of PlaneSeparatesGeometries objects are
    // constructed. The content of these objects are tested in
    // CspaceFreePolytopeGenerateRationals.
    bool associated_to_PlaneSeparatesGeometries_with_margin = false;
    bool associated_to_PlaneSeparatesGeometries_without_margin = false;
    for (const auto& plane_separates_geometries :
         tester.plane_geometries_w_margin()) {
      if (plane_separates_geometries.plane_index == plane_index) {
        associated_to_PlaneSeparatesGeometries_with_margin = true;
        break;
      }
    }
    EXPECT_TRUE(associated_to_PlaneSeparatesGeometries_with_margin);
    for (const auto& plane_separates_geometries :
         tester.plane_geometries_wo_margin()) {
      if (plane_separates_geometries.plane_index == plane_index) {
        associated_to_PlaneSeparatesGeometries_without_margin = true;
        break;
      }
    }
    EXPECT_TRUE(associated_to_PlaneSeparatesGeometries_without_margin);
  }
}

TEST_F(CIrisToyRobotTest, CspaceFreePolytopeGenerateRationals) {
  const Eigen::Vector3d q_star(0, 0, 0);
  CspaceFreePolytopeTester tester(plant_, scene_graph_,
                                  SeparatingPlaneOrder::kAffine, q_star);
  auto ret = tester.GenerateRationals(false /* search_separating_margin */);
  EXPECT_EQ(ret.size(),
            tester.cspace_free_polytope().separating_planes().size());
  for (const auto& plane_geometries : ret) {
    const auto& plane = tester.cspace_free_polytope()
                            .separating_planes()[plane_geometries.plane_index];
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
    EXPECT_FALSE(plane_geometries.separating_margin.has_value());
  }

  ret = tester.GenerateRationals(true /* search_separating_margin */);
  EXPECT_EQ(ret.size(),
            tester.cspace_free_polytope().separating_planes().size());
  for (const auto& plane_geometries : ret) {
    const auto& plane = tester.cspace_free_polytope()
                            .separating_planes()[plane_geometries.plane_index];
    if (plane.positive_side_geometry->type() != GeometryType::kCylinder &&
        plane.negative_side_geometry->type() != GeometryType::kCylinder) {
      // The unit length vector is always a.
      EXPECT_EQ(plane_geometries.unit_length_vectors.size(), 1);
      EXPECT_EQ(plane_geometries.unit_length_vectors[0].rows(), 3);
      for (int i = 0; i < 3; ++i) {
        EXPECT_EQ(plane_geometries.unit_length_vectors[0](i), plane.a(i));
      }
    }
    EXPECT_TRUE(plane_geometries.separating_margin.has_value());
  }
}

TEST_F(CIrisToyRobotTest, FindRedundantInequalities) {
  // Test CspaceFreePolytope::FindRedundantInequalities.
  const Eigen::Vector3d q_star(0, 0, 0);
  CspaceFreePolytopeTester tester(plant_, scene_graph_,
                                  SeparatingPlaneOrder::kAffine, q_star);
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
  const Eigen::Vector3d q_star(0, 0, 0);
  CspaceFreePolytopeTester tester(plant_, scene_graph_,
                                  SeparatingPlaneOrder::kAffine, q_star);
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
  const Eigen::Vector3d q_star(0, 0, 0);
  CspaceFreePolytopeTester tester(plant_, scene_graph_,
                                  SeparatingPlaneOrder::kAffine, q_star);
  VectorX<symbolic::Polynomial> s_minus_s_lower;
  VectorX<symbolic::Polynomial> s_upper_minus_s;
  EXPECT_EQ(tester.s_minus_s_lower().rows(), 3);
  EXPECT_EQ(tester.s_upper_minus_s().rows(), 3);
  const Eigen::Vector3d s_lower =
      tester.cspace_free_polytope().rational_forward_kin().ComputeSValue(

          plant_->GetPositionLowerLimits(), q_star);
  const Eigen::Vector3d s_upper =
      tester.cspace_free_polytope().rational_forward_kin().ComputeSValue(
          plant_->GetPositionUpperLimits(), q_star);
  const auto& s = tester.cspace_free_polytope().rational_forward_kin().s();
  for (int i = 0; i < 3; ++i) {
    EXPECT_PRED2(symbolic::test::PolyEqual, tester.s_minus_s_lower()(i),
                 symbolic::Polynomial((s(i) - s_lower(i))));
    EXPECT_PRED2(symbolic::test::PolyEqual, tester.s_upper_minus_s()(i),
                 symbolic::Polynomial((s_upper(i) - s(i))));
  }
}

TEST_F(CIrisToyRobotTest, CalcMonomialBasis) {
  const Eigen::Vector3d q_star(0, 0, 0);
  CspaceFreePolytopeTester tester(plant_, scene_graph_,
                                  SeparatingPlaneOrder::kAffine, q_star);
  const auto map_body_to_monomial_basis = tester.map_body_to_monomial_basis();
  // Make sure map_body_to_monomial_basis contains all pairs of bodies.
  for (const auto& plane : tester.cspace_free_polytope().separating_planes()) {
    for (const auto collision_geometry :
         {plane.positive_side_geometry, plane.negative_side_geometry}) {
      auto it =
          map_body_to_monomial_basis.find(SortedPair<multibody::BodyIndex>(
              plane.expressed_body, collision_geometry->body_index()));
      EXPECT_NE(it, map_body_to_monomial_basis.end());
      // Make sure the degree for each variable in the monomial is at most 1.
      for (int i = 0; i < it->second.rows(); ++i) {
        for (const auto& [var, degree] : it->second(i).get_powers()) {
          EXPECT_LE(degree, 1);
        }
      }
    }
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

TEST_F(CIrisToyRobotTest, ConstructPlaneSearchProgram1) {
  // Test ConstructPlaneSearchProgram with no unit-length-vector constraint.
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
  d << 0.1, 0.2, 0.3, 0.2, 0.2, 0.2, 0.1, 0.1, 0.2;

  VectorX<symbolic::Polynomial> d_minus_Cs;
  std::unordered_set<int> C_redundant_indices;
  std::unordered_set<int> s_lower_redundant_indices;
  std::unordered_set<int> s_upper_redundant_indices;
  SetupPolytope(tester, C, d, &d_minus_Cs, &C_redundant_indices,
                &s_lower_redundant_indices, &s_upper_redundant_indices);

  const auto plane_geometries_vec = tester.GenerateRationals(false);
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
      plane_geometries, d_minus_Cs, C_redundant_indices,
      s_lower_redundant_indices, s_upper_redundant_indices);

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

    const VectorX<symbolic::Polynomial>& s_minus_s_lower =
        tester.s_minus_s_lower();
    const VectorX<symbolic::Polynomial>& s_upper_minus_s =
        tester.s_upper_minus_s();
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
  d << 0.1, 0.2, 0.3, 0.2, 0.2, 0.2, 0.1, 0.1, 0.2;

  VectorX<symbolic::Polynomial> d_minus_Cs;
  std::unordered_set<int> C_redundant_indices;
  std::unordered_set<int> s_lower_redundant_indices;
  std::unordered_set<int> s_upper_redundant_indices;
  SetupPolytope(tester, C, d, &d_minus_Cs, &C_redundant_indices,
                &s_lower_redundant_indices, &s_upper_redundant_indices);

  const auto plane_geometries_vec = tester.GenerateRationals(false);
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
      plane_geometries, d_minus_Cs, C_redundant_indices,
      s_lower_redundant_indices, s_upper_redundant_indices);
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
    for (int i = 0; i < s_samples.rows(); ++i) {
      const Eigen::Vector3d s_val =
          ProjectToPolytope(s_samples.row(i).transpose(), C, d,
                            tester.s_lower(), tester.s_upper());
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
  d << 0.1, 0.2, 0.3, 0.2, 0.2, 0.2, 0.1, 0.1, 0.2;

  VectorX<symbolic::Polynomial> d_minus_Cs;
  std::unordered_set<int> C_redundant_indices;
  std::unordered_set<int> s_lower_redundant_indices;
  std::unordered_set<int> s_upper_redundant_indices;
  SetupPolytope(tester, C, d, &d_minus_Cs, &C_redundant_indices,
                &s_lower_redundant_indices, &s_upper_redundant_indices);
  solvers::MathematicalProgram prog;
  prog.AddIndeterminates(
      tester.cspace_free_polytope().rational_forward_kin().s());

  const auto plane_geometries_vec = tester.GenerateRationals(false);
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
      C_redundant_indices, s_lower_redundant_indices, s_upper_redundant_indices,
      std::nullopt /* polytope lagrangians */);
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

TEST_F(CIrisToyRobotTest, FindSeparationCertificateGivenPolytopeSuccess) {
  // Test CspaceFreePolytope::FindSeparationCertificateGivenPolytope for a
  // collision-free C-space polytope.
  auto diagram_context = diagram_->CreateDefaultContext();
  auto& plant_context =
      plant_->GetMyMutableContextFromRoot(diagram_context.get());
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
  Eigen::Matrix3Xd s_vals(3, s_samples.rows());
  Eigen::Matrix3Xd q_vals(3, s_samples.rows());
  for (int i = 0; i < s_samples.rows(); ++i) {
    s_vals.col(i) = ProjectToPolytope(s_samples.row(i).transpose(), C, d,
                                      tester.s_lower(), tester.s_upper());
    q_vals.col(i) =
        tester.cspace_free_polytope().rational_forward_kin().ComputeQValue(
            s_vals.col(i), q_star);
  }

  const CspaceFreePolytope::IgnoredCollisionPairs ignored_collision_pairs{
      SortedPair<geometry::GeometryId>(world_box_, body2_sphere_)};
  CspaceFreePolytope::FindSeparationCertificateGivenPolytopeOptions options;
  options.verbose = true;
  solvers::MosekSolver solver;
  options.solver_id = solver.id();
  if (solver.available()) {
    for (int num_threads : {-1, 1, 5}) {
      options.num_threads = num_threads;

      const auto certificates_result =
          tester.FindSeparationCertificateGivenPolytope(
              ignored_collision_pairs, C, d, true /* search_separating_plane */,
              options);
      EXPECT_EQ(certificates_result.size(),
                tester.cspace_free_polytope().separating_planes().size() -
                    ignored_collision_pairs.size());
      for (const auto& certificate : certificates_result) {
        EXPECT_TRUE(certificate.has_value());
        const auto& plane = tester.cspace_free_polytope()
                                .separating_planes()[certificate->plane_index];
        EXPECT_EQ(
            ignored_collision_pairs.count(SortedPair<geometry::GeometryId>(
                plane.positive_side_geometry->id(),
                plane.negative_side_geometry->id())),
            0);
        EXPECT_TRUE(certificate->separating_margin.has_value());
        EXPECT_GT(certificate->separating_margin.value(), 0);

        // Make sure that the unit length constraint is satisfied.
        for (int j = 0; j < s_vals.cols(); ++j) {
          symbolic::Environment env;
          env.insert(tester.cspace_free_polytope().rational_forward_kin().s(),
                     s_vals.col(j));
          Eigen::Vector3d a_val;
          for (int k = 0; k < 3; ++k) {
            a_val(k) = certificate->a(k).Evaluate(env);
          }
          EXPECT_LE(a_val.norm(), 1);
          const double b_val = certificate->b.Evaluate(env);

          plant_->SetPositions(&plant_context, q_vals.col(j));
          EXPECT_GE(
              DistanceToHalfspace(*(plane.positive_side_geometry), a_val, b_val,
                                  plane.expressed_body, PlaneSide::kPositive,
                                  *plant_, plant_context),
              certificate->separating_margin.value());
          EXPECT_GE(
              DistanceToHalfspace(*(plane.negative_side_geometry), a_val, b_val,
                                  plane.expressed_body, PlaneSide::kNegative,
                                  *plant_, plant_context),
              certificate->separating_margin.value());
        }
      }
    }
    std::unordered_map<SortedPair<geometry::GeometryId>,
                       CspaceFreePolytope::SeparationCertificateResult>
        certificates_map;
    bool is_success =
        tester.cspace_free_polytope().FindSeparationCertificateGivenPolytope(
            C, d, ignored_collision_pairs, true /* search_separating_margin*/,
            options, &certificates_map);
    EXPECT_EQ(certificates_map.size(),
              tester.cspace_free_polytope().separating_planes().size() -
                  ignored_collision_pairs.size());
    EXPECT_TRUE(is_success);

    // Test search_separating_margin = false.
    certificates_map.clear();
    is_success =
        tester.cspace_free_polytope().FindSeparationCertificateGivenPolytope(
            C, d, ignored_collision_pairs, false /* search_separating_margin */,
            options, &certificates_map);
    EXPECT_EQ(certificates_map.size(),
              tester.cspace_free_polytope().separating_planes().size() -
                  ignored_collision_pairs.size());
    EXPECT_TRUE(is_success);
    // Check the separating plane does separate the geometries.
    for (const auto& [geometry_pair, certificate] : certificates_map) {
      const auto& plane = tester.cspace_free_polytope()
                              .separating_planes()[certificate.plane_index];
      for (int j = 0; j < s_vals.cols(); ++j) {
        symbolic::Environment env;
        env.insert(tester.cspace_free_polytope().rational_forward_kin().s(),
                   s_vals.col(j));
        Eigen::Vector3d a_val;
        for (int k = 0; k < 3; ++k) {
          a_val(k) = certificate.a(k).Evaluate(env);
        }
        const double b_val = certificate.b.Evaluate(env);

        plant_->SetPositions(&plant_context, q_vals.col(j));
        EXPECT_GE(
            DistanceToHalfspace(*(plane.positive_side_geometry), a_val, b_val,
                                plane.expressed_body, PlaneSide::kPositive,
                                *plant_, plant_context),
            0);
        EXPECT_GE(
            DistanceToHalfspace(*(plane.negative_side_geometry), a_val, b_val,
                                plane.expressed_body, PlaneSide::kNegative,
                                *plant_, plant_context),
            0);
      }
    }
  }
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
  d << 0.1, 0.3, 0.4, 0.2;

  const CspaceFreePolytope::IgnoredCollisionPairs ignored_collision_pairs{
      SortedPair<geometry::GeometryId>(world_box_, body2_sphere_)};
  CspaceFreePolytope::FindSeparationCertificateGivenPolytopeOptions options;
  options.verbose = true;
  solvers::MosekSolver solver;
  options.solver_id = solver.id();
  if (solver.available()) {
    for (int num_threads : {-1, 1, 5}) {
      options.num_threads = num_threads;

      const auto certificates_result =
          tester.FindSeparationCertificateGivenPolytope(
              ignored_collision_pairs, C, d,
              true /* search_separating_margin */, options);
      EXPECT_EQ(certificates_result.size(),
                tester.cspace_free_polytope().separating_planes().size() -
                    ignored_collision_pairs.size());
      EXPECT_TRUE(
          std::any_of(certificates_result.begin(), certificates_result.end(),
                      [](const std::optional<
                          CspaceFreePolytope::SeparationCertificateResult>&
                             certificate_result) {
                        return !certificate_result.has_value();
                      }));
    }
    // Test with terminate_at_failure=false. This will run the sos for every
    // separating plane.
    options.terminate_at_failure = false;
    const auto certificates_result =
        tester.FindSeparationCertificateGivenPolytope(
            ignored_collision_pairs, C, d, true /* search_separating_margin */,
            options);
    EXPECT_EQ(certificates_result.size(),
              tester.cspace_free_polytope().separating_planes().size() -
                  ignored_collision_pairs.size());
    std::unordered_map<SortedPair<geometry::GeometryId>,
                       CspaceFreePolytope::SeparationCertificateResult>
        certificates_map;
    const bool is_success =
        tester.cspace_free_polytope().FindSeparationCertificateGivenPolytope(
            C, d, ignored_collision_pairs, true /* search_separating_margin */,
            options, &certificates_map);
    EXPECT_FALSE(is_success);

    std::unordered_set<SortedPair<geometry::GeometryId>> inseparable_geometries;
    inseparable_geometries.emplace(body0_box_, body2_capsule_);
    inseparable_geometries.emplace(body0_box_, body2_sphere_);
    inseparable_geometries.emplace(body0_sphere_, body2_capsule_);
    inseparable_geometries.emplace(body0_sphere_, body2_sphere_);
    inseparable_geometries.emplace(body1_convex_, body3_sphere_);
    inseparable_geometries.emplace(body2_sphere_, body3_sphere_);
    for (const auto& certificate : certificates_result) {
      if (certificate.has_value()) {
        const auto& plane = tester.cspace_free_polytope()
                                .separating_planes()[certificate->plane_index];
        const SortedPair<geometry::GeometryId> geometry_pair(
            plane.positive_side_geometry->id(),
            plane.negative_side_geometry->id());
        EXPECT_EQ(inseparable_geometries.count(geometry_pair), 0);
      }
    }
    for (const auto& [geometry_pair, certificate] : certificates_map) {
      EXPECT_EQ(inseparable_geometries.count(geometry_pair), 0);
    }
  }
}

TEST_F(CIrisToyRobotTest, AddEllipsoidContainmentConstraint) {
  const Eigen::Vector3d q_star(0, 0, 0);
  CspaceFreePolytopeTester tester(plant_, scene_graph_,
                                  SeparatingPlaneOrder::kAffine, q_star);

  solvers::MathematicalProgram prog;
  auto C = prog.NewContinuousVariables(8, 3);
  auto d = prog.NewContinuousVariables(8);
  auto ellipsoid_margins = prog.NewContinuousVariables(8);

  Eigen::Matrix3d Q;
  // Use arbitrary Q and s0
  // clang-format off
  Q << 1, 2, -1,
       0, 1, 2,
       2, -1, 3;
  // clang-format on
  const Eigen::Vector3d s0 = 0.4 * tester.s_lower() + tester.s_upper() * 0.6;
  tester.AddEllipsoidContainmentConstraint(&prog, Q, s0, C, d,
                                           ellipsoid_margins);
  prog.AddBoundingBoxConstraint(0, kInf, ellipsoid_margins);
  const auto result = solvers::Solve(prog);
  ASSERT_TRUE(result.is_success());
  const auto C_sol = result.GetSolution(C);
  const auto d_sol = result.GetSolution(d);
  const auto margin_sol = result.GetSolution(ellipsoid_margins);
  for (int i = 0; i < C_sol.rows(); ++i) {
    EXPECT_LE(C_sol.row(i).norm(), 1);
    EXPECT_LE((C_sol.row(i) * Q).norm(),
              d_sol(i) - C_sol.row(i).dot(s0) - margin_sol(i));
  }
}

TEST_F(CIrisToyRobotTest, AddCspacePolytopeContainment) {
  const Eigen::Vector3d q_star(0, 0, 0);
  CspaceFreePolytopeTester tester(plant_, scene_graph_,
                                  SeparatingPlaneOrder::kAffine, q_star);

  Eigen::Matrix<double, 3, 4> s_inner_pts;
  // clang-format off
  s_inner_pts << 1, 2, 0, 3,
                 0, -1, 2, 3,
                 -1, 2, 1, 3;
  // clang-format on
  for (int i = 0; i < s_inner_pts.cols(); ++i) {
    s_inner_pts.col(i) = s_inner_pts.col(i)
                             .cwiseMin(tester.s_upper())
                             .cwiseMax(tester.s_lower());
  }
  solvers::MathematicalProgram prog;
  auto C = prog.NewContinuousVariables<5, 3>();
  auto d = prog.NewContinuousVariables<5>();
  tester.AddCspacePolytopeContainment(&prog, C, d, s_inner_pts);
  EXPECT_EQ(prog.linear_constraints().size(), 1);
  const VectorX<symbolic::Expression> constraint_val =
      prog.linear_constraints()[0].evaluator()->get_sparse_A() *
      prog.linear_constraints()[0].variables();
  EXPECT_TRUE(CompareMatrices(
      prog.linear_constraints()[0].evaluator()->lower_bound(),
      Eigen::VectorXd::Constant(
          prog.linear_constraints()[0].evaluator()->num_constraints(), -kInf)));
  EXPECT_TRUE(CompareMatrices(
      prog.linear_constraints()[0].evaluator()->upper_bound(),
      Eigen::VectorXd::Constant(
          prog.linear_constraints()[0].evaluator()->num_constraints(), 0)));
  for (int i = 0; i < C.rows(); ++i) {
    for (int j = 0; j < s_inner_pts.cols(); ++j) {
      EXPECT_PRED2(symbolic::test::ExprEqual,
                   constraint_val(i * s_inner_pts.cols() + j),
                   C.row(i).dot(s_inner_pts.col(j)) - d(i));
    }
  }
}

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
      if (ignored_collision_pairs.count(SortedPair<geometry::GeometryId>(
              plane.positive_side_geometry->id(),
              plane.negative_side_geometry->id())) == 0) {
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
      SortedPair<geometry::GeometryId>(world_box_, body2_sphere_)};

  CspaceFreePolytope::FindSeparationCertificateGivenPolytopeOptions options;
  options.verbose = true;
  solvers::MosekSolver solver;
  options.solver_id = solver.id();
  options.backoff_scale = 0.01;
  if (solver.available()) {
    options.num_threads = -1;

    const auto certificates_result =
        tester.FindSeparationCertificateGivenPolytope(
            ignored_collision_pairs, C, d, true /* search_separating_margin */,
            options);
    const int gram_total_size =
        tester.GetGramVarSizeForPolytopeSearchProgram(ignored_collision_pairs);
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

    const HPolyhedron cspace_h_polyhedron =
        tester.GetPolyhedronWithJointLimits(C, d);
    const auto ellipsoid =
        cspace_h_polyhedron.MaximumVolumeInscribedEllipsoid();
    const Eigen::MatrixXd ellipsoid_Q = ellipsoid.A().inverse();
    const auto polytope_result = tester.FindPolytopeGivenLagrangian(
        ignored_collision_pairs, C_var, d_var, d_minus_Cs, certificates_result,
        ellipsoid_Q, ellipsoid.center(), ellipsoid_margins, gram_total_size,
        polytope_options);
    ASSERT_TRUE(polytope_result.has_value());
    // Now check if the C-space polytope {s | C*s<=d, s_lower<=s<=s_upper} is
    // collision free.
    // First sample many s values, and project these values into the polytope {s
    // | C*s<=d, s_lower<=s<=s_upper}, then check if the plane separates the
    // geometries at the corresponding configurations.
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
    CheckSeparationBySamples(tester, *diagram_, s_samples, polytope_result->C,
                             polytope_result->d, polytope_result->a,
                             polytope_result->b, q_star,
                             ignored_collision_pairs);

    // Check the margin between the inscribed ellipsoid and the polytope faces.
    EXPECT_TRUE((polytope_result->ellipsoid_margins.array() >= 0).all());
    for (int i = 0; i < polytope_result->C.rows(); ++i) {
      EXPECT_LE(polytope_result->C.row(i).norm(), 1);
      EXPECT_GE((polytope_result->d(i) -
                 polytope_result->C.row(i).dot(ellipsoid.center()) -
                 (polytope_result->C.row(i) * ellipsoid_Q).norm()) /
                    polytope_result->C.row(i).norm(),
                polytope_result->ellipsoid_margins(i));
    }
    // Check that the inequality constraint |Qcᵢ|₂ ≤ dᵢ − δᵢ − cᵢᵀs₀ is active
    // at the optimal solution.
    EXPECT_TRUE(CompareMatrices(
        (ellipsoid_Q * polytope_result->C.transpose()).colwise().norm(),
        (polytope_result->d - polytope_result->ellipsoid_margins -
         polytope_result->C * ellipsoid.center())
            .transpose(),
        1E-5));
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

  solvers::MosekSolver solver;
  if (solver.available()) {
    auto bilinear_alternation_result =
        tester.cspace_free_polytope().SearchWithBilinearAlternation(
            ignored_collision_pairs, C, d, true /* search_margin */,
            bilinear_alternation_options);
    ASSERT_TRUE(bilinear_alternation_result.has_value());
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
    CheckSeparationBySamples(
        tester, *diagram_, s_samples, bilinear_alternation_result->C,
        bilinear_alternation_result->d, bilinear_alternation_result->a,
        bilinear_alternation_result->b, q_star, ignored_collision_pairs);
  }
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
  options.scale_max = 3;
  solvers::MosekSolver solver;
  if (solver.available()) {
    const Eigen::Vector3d s_center(0.01, 0, 0.01);
    auto result = tester.cspace_free_polytope().BinarySearch(
        ignored_collision_pairs, C, d, s_center, options);
    ASSERT_TRUE(result.has_value());
    EXPECT_GT(result->num_iter, 0);
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
    CheckSeparationBySamples(tester, *diagram_, s_samples, result->C, result->d,
                             result->a, result->b, q_star,
                             ignored_collision_pairs);

    // Now test epsilon_max being feasible.
    options.scale_max = 0.02;
    options.scale_min = 0.01;
    result = tester.cspace_free_polytope().BinarySearch(
        ignored_collision_pairs, C, d, s_center, options);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->num_iter, 0);
    EXPECT_TRUE(CompareMatrices(result->C, C));
    EXPECT_TRUE(CompareMatrices(
        result->d,
        options.scale_max * d + (1 - options.scale_max) * C * s_center, 1E-10));
    CheckSeparationBySamples(tester, *diagram_, s_samples, result->C, result->d,
                             result->a, result->b, q_star,
                             ignored_collision_pairs);

    // Now check infeasible epsilon_min
    options.scale_min = 10;
    options.scale_max = 20;
    result = tester.cspace_free_polytope().BinarySearch(
        ignored_collision_pairs, C, d, s_center, options);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result->num_iter, 0);
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
