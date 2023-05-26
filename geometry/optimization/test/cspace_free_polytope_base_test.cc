#include "drake/geometry/optimization/cspace_free_polytope_base.h"

#include <gtest/gtest.h>

#include "drake/geometry/optimization/cspace_free_internal.h"
#include "drake/geometry/optimization/test/c_iris_test_utilities.h"
#include "drake/multibody/rational/rational_forward_kinematics_internal.h"

namespace drake {
namespace geometry {
namespace optimization {
class CspaceFreePolytopeDummy : public CspaceFreePolytopeBase {
 public:
  CspaceFreePolytopeDummy(const multibody::MultibodyPlant<double>* plant,
                          const geometry::SceneGraph<double>* scene_graph,
                          SeparatingPlaneOrder plane_order,
                          const CspaceFreePolytopeBase::Options& options =
                              CspaceFreePolytopeBase::Options{})
      : CspaceFreePolytopeBase(plant, scene_graph, plane_order, options) {}
};

class CspaceFreePolytopeBaseTester {
 public:
  CspaceFreePolytopeBaseTester(const multibody::MultibodyPlant<double>* plant,
                               const geometry::SceneGraph<double>* scene_graph,
                               SeparatingPlaneOrder plane_order,
                               const CspaceFreePolytopeBase::Options& options =
                                   CspaceFreePolytopeBase::Options())
      : cspace_free_polytope_base_{plant, scene_graph, plane_order, options} {}

  const CspaceFreePolytopeBase& cspace_free_polytope_base() const {
    return cspace_free_polytope_base_;
  }

 private:
  CspaceFreePolytopeDummy cspace_free_polytope_base_;
};

TEST_F(CIrisToyRobotTest, CspaceFreePolytopeBaseConstructor) {
  CspaceFreePolytopeBaseTester tester(plant_, scene_graph_,
                                      SeparatingPlaneOrder::kAffine);

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
}
}  // namespace optimization
}  // namespace geometry
}  // namespace drake
