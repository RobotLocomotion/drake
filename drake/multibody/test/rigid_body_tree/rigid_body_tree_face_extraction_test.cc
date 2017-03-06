#include <memory>

#include "drake/common/drake_path.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/joints/floating_base_types.h"

#include "gtest/gtest.h"

using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::Matrix3Xd;

namespace drake {
namespace systems {
namespace plants {
namespace test {
namespace rigid_body_tree {
namespace {


// This unit test verifies that the complete stack of 
// requesting vertices and faces from the consituent components
// of an RBT works.
GTEST_TEST(RBTFaceExtractionTests, ExtractVertsAndFaces) {
  RigidBodyTree<double> tree;
  parsers::sdf::AddModelInstancesFromSdfFileToWorld(
        GetDrakePath() +
            "/multibody/test/rigid_body_tree/small_sphere_on_large_box.sdf",
        multibody::joints::kQuaternion, &tree);

  // Here, we iterate over the RigidBody elements of the tree to find
  // the collision element indices that compose the collision model,
  // retrieve the collision elements, and check the existence of geometry
  // on each one. The box geometry should have 12 tris as faces that we
  // can access.
  bool found_box = false;
  bool found_sphere = false;
  for (auto iter=tree.bodies.begin(); iter!=tree.bodies.end(); iter++){
    auto collision_elems = (*iter)->get_collision_element_ids();
    for (auto collision_elem = collision_elems.begin();
              collision_elem != collision_elems.end();
              collision_elem++){
      auto element = tree.FindCollisionElement(*collision_elem);
      EXPECT_TRUE(element->hasGeometry());
      // It is extremely important that this geometry object is 
      // a reference or pointer so that whatever subclass the
      // element geometry is, doesn't get clobbered.
      const DrakeShapes::Geometry & geometry = element->getGeometry();
      if (geometry.getShape() == DrakeShapes::BOX){
        found_box = true;
        EXPECT_TRUE(geometry.hasFaces());
        DrakeShapes::TrianglesVector faces;
        geometry.getFaces(faces);
        EXPECT_EQ(faces.size(), 12);
      } else if (geometry.getShape() == DrakeShapes::SPHERE){
        found_sphere = true;
        EXPECT_FALSE(geometry.hasFaces());
      }
    }
  }
  EXPECT_TRUE(found_box);
  EXPECT_TRUE(found_sphere);
}

}  // namespace
}  // namespace rigid_body_tree
}  // namespace test
}  // namespace plants
}  // namespace systems
}  // namespace drake
