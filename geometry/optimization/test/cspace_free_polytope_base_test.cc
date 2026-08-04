#include "drake/geometry/optimization/cspace_free_polytope_base.h"

#include <unordered_map>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/symbolic_test_util.h"
#include "drake/geometry/optimization/cspace_free_internal.h"
#include "drake/geometry/optimization/test/c_iris_test_utilities.h"
#include "drake/multibody/rational/rational_forward_kinematics_internal.h"

namespace drake {
namespace geometry {
namespace optimization {
class CspaceFreePolytopeDummy : public CspaceFreePolytopeBase {
 public:
  using CspaceFreePolytopeBase::SForPlane;

  CspaceFreePolytopeDummy(const multibody::MultibodyPlant<double>* plant,
                          const geometry::SceneGraph<double>* scene_graph,
                          SeparatingPlaneOrder plane_order,
                          SForPlane s_for_plane_enum,
                          const CspaceFreePolytopeBase::Options& options =
                              CspaceFreePolytopeBase::Options{})
      : CspaceFreePolytopeBase(plant, scene_graph, plane_order,
                               s_for_plane_enum, options) {}
};

class CspaceFreePolytopeBaseTester {
 public:
  CspaceFreePolytopeBaseTester(
      const multibody::MultibodyPlant<double>* plant,
      const geometry::SceneGraph<double>* scene_graph,
      SeparatingPlaneOrder plane_order,
      CspaceFreePolytopeDummy::SForPlane s_for_plane_enum,
      const CspaceFreePolytopeBase::Options& options =
          CspaceFreePolytopeBase::Options())
      : cspace_free_polytope_base_{plant, scene_graph, plane_order,
                                   s_for_plane_enum, options} {}

  const CspaceFreePolytopeBase& cspace_free_polytope_base() const {
    return cspace_free_polytope_base_;
  }

  const std::unordered_map<SortedPair<multibody::BodyIndex>, std::vector<int>>&
  map_body_pair_to_s_on_chain() const {
    return cspace_free_polytope_base_.map_body_pair_to_s_on_chain();
  }

  template <typename T>
  void CalcSBoundsPolynomial(
      const VectorX<T>& s_box_lower, const VectorX<T>& s_box_upper,
      VectorX<symbolic::Polynomial>* s_minus_s_lower,
      VectorX<symbolic::Polynomial>* s_upper_minus_s) const {
    return cspace_free_polytope_base_.CalcSBoundsPolynomial<T>(
        s_box_lower, s_box_upper, s_minus_s_lower, s_upper_minus_s);
  }

 private:
  CspaceFreePolytopeDummy cspace_free_polytope_base_;
};

TEST_F(CIrisToyRobotTest, CspaceFreePolytopeBaseConstructor1) {
  // Test with SForPlane::kAll.
  CspaceFreePolytopeBaseTester tester(plant_, scene_graph_,
                                      SeparatingPlaneOrder::kAffine,
                                      CspaceFreePolytopeDummy::SForPlane::kAll);

  const CspaceFreePolytopeBase& dut = tester.cspace_free_polytope_base();

  const auto link_geometries =
      internal::GetCollisionGeometries(*plant_, *scene_graph_);
  int num_planes_expected = 0;
  // Count the expected number of planes by hand.
  num_planes_expected +=
      link_geometries.at(plant_->world_body().index()).size() *
      // Don't include world_body to body0 as there is only a weld joint
      // between them.
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
  }

  // Check map_body_pair_to_s_on_chain.
  const std::unordered_map<SortedPair<multibody::BodyIndex>, std::vector<int>>&
      map_body_pair_to_s_on_chain = tester.map_body_pair_to_s_on_chain();
  EXPECT_TRUE(map_body_pair_to_s_on_chain
                  .at({body_indices_[0], plant_->world_body().index()})
                  .empty());
  EXPECT_THAT(map_body_pair_to_s_on_chain.at(
                  {plant_->world_body().index(), body_indices_[1]}),
              testing::ElementsAre(0));
  EXPECT_THAT(map_body_pair_to_s_on_chain.at(
                  {plant_->world_body().index(), body_indices_[2]}),
              testing::ElementsAre(0, 1));
  EXPECT_THAT(map_body_pair_to_s_on_chain.at(
                  {plant_->world_body().index(), body_indices_[3]}),
              testing::ElementsAre(2));
  EXPECT_THAT(
      map_body_pair_to_s_on_chain.at({body_indices_[0], body_indices_[1]}),
      testing::ElementsAre(0));
  EXPECT_THAT(
      map_body_pair_to_s_on_chain.at({body_indices_[0], body_indices_[2]}),
      testing::ElementsAre(0, 1));
  EXPECT_THAT(
      map_body_pair_to_s_on_chain.at({body_indices_[0], body_indices_[3]}),
      testing::ElementsAre(2));
  EXPECT_THAT(
      map_body_pair_to_s_on_chain.at({body_indices_[1], body_indices_[2]}),
      testing::ElementsAre(1));
  EXPECT_THAT(
      map_body_pair_to_s_on_chain.at({body_indices_[1], body_indices_[3]}),
      testing::ElementsAre(0, 2));
  EXPECT_THAT(
      map_body_pair_to_s_on_chain.at({body_indices_[2], body_indices_[3]}),
      testing::ElementsAre(1, 0, 2));
}

TEST_F(CIrisToyRobotTest, CspaceFreePolytopeBaseConstructor2) {
  // Test with SForPlane::kOnChain.
  CspaceFreePolytopeBaseTester tester(
      plant_, scene_graph_, SeparatingPlaneOrder::kAffine,
      CspaceFreePolytopeDummy::SForPlane::kOnChain);

  const CspaceFreePolytopeBase& dut = tester.cspace_free_polytope_base();

  auto check_plane_s = [&dut](geometry::GeometryId geometry1,
                              geometry::GeometryId geometry2,
                              const symbolic::Variables& s_expected) {
    const int plane_index = dut.map_geometries_to_separating_planes().at(
        SortedPair<geometry::GeometryId>(geometry1, geometry2));
    for (int i = 0; i < 3; ++i) {
      const auto& s_in_a =
          dut.separating_planes()[plane_index].a(i).indeterminates();
      EXPECT_EQ(s_in_a.size(), s_expected.size());
      for (const auto& s : s_in_a) {
        EXPECT_TRUE(s_expected.include(s));
      }
    }
    const auto& s_in_b =
        dut.separating_planes()[plane_index].b.indeterminates();
    EXPECT_EQ(s_in_b.size(), s_expected.size());
    for (const auto& s : s_in_b) {
      EXPECT_TRUE(s_expected.include(s));
    }
  };

  const VectorX<symbolic::Variable>& s = dut.rational_forward_kin().s();

  check_plane_s(world_box_, body1_convex_, symbolic::Variables({s(0)}));
  check_plane_s(world_box_, body2_sphere_, symbolic::Variables({s(0), s(1)}));
  check_plane_s(world_box_, body3_box_, symbolic::Variables({s(2)}));
  check_plane_s(body1_capsule_, body3_box_, symbolic::Variables({s(0), s(2)}));
  check_plane_s(body2_sphere_, body3_box_,
                symbolic::Variables({s(0), s(1), s(2)}));
}

TEST_F(CIrisToyRobotTest, CalcSBoundsPolynomial) {
  const Eigen::Vector3d q_star(0, 0, 0);
  CspaceFreePolytopeBaseTester tester(
      plant_, scene_graph_, SeparatingPlaneOrder::kAffine,
      CspaceFreePolytopeDummy::SForPlane::kOnChain);
  VectorX<symbolic::Polynomial> s_minus_s_lower;
  VectorX<symbolic::Polynomial> s_upper_minus_s;
  // Test with double.
  Eigen::Vector3d s_lower(-1, -2, -3);
  Eigen::Vector3d s_upper(0, -1, 2);
  tester.CalcSBoundsPolynomial<double>(s_lower, s_upper, &s_minus_s_lower,
                                       &s_upper_minus_s);
  const auto& s = tester.cspace_free_polytope_base().rational_forward_kin().s();
  for (int i = 0; i < 3; ++i) {
    EXPECT_PRED2(symbolic::test::PolyEqual, s_minus_s_lower(i),
                 symbolic::Polynomial((s(i) - s_lower(i))));
    EXPECT_PRED2(symbolic::test::PolyEqual, s_upper_minus_s(i),
                 symbolic::Polynomial((s_upper(i) - s(i))));
  }
  // Test with s_lower and s_upper being symbolic
  const Vector3<symbolic::Variable> s_lower_sym =
      symbolic::MakeVectorContinuousVariable(3, "s_lower");
  const Vector3<symbolic::Variable> s_upper_sym =
      symbolic::MakeVectorContinuousVariable(3, "s_upper");
  tester.CalcSBoundsPolynomial<symbolic::Variable>(
      s_lower_sym, s_upper_sym, &s_minus_s_lower, &s_upper_minus_s);
  EXPECT_EQ(s_minus_s_lower.rows(), 3);
  EXPECT_EQ(s_upper_minus_s.rows(), 3);
  for (int i = 0; i < 3; ++i) {
    EXPECT_TRUE(
        s_minus_s_lower(i).decision_variables().include(s_lower_sym(i)));
    EXPECT_TRUE(
        s_upper_minus_s(i).decision_variables().include(s_upper_sym(i)));
    EXPECT_PRED2(symbolic::test::PolyEqual, s_minus_s_lower(i),
                 symbolic::Polynomial(s(i) - s_lower_sym(i),
                                      symbolic::Variables({s(i)})));
    EXPECT_PRED2(symbolic::test::PolyEqual, s_upper_minus_s(i),
                 symbolic::Polynomial(s_upper_sym(i) - s(i),
                                      symbolic::Variables({s(i)})));
  }
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
