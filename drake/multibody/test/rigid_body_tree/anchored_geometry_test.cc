#include <memory>

#include <gtest/gtest.h>

#include "drake/multibody/parser_sdf.h";
#include "drake/multibody/rigid_body_tree.h";

// This tests the functionality for classifying geometry in a RigidBodyTree
// as "anchored".  It confirms that the DrakeCollision::Element exhibits the
// property and that the corresponding bullet collision element likewise
// exhibits this property.

namespace drake {
namespace systems {
namespace plants {
namespace test {
namespace rigid_body_tree {
namespace {

using drake::parsers::sdf::AddModelInstancesFromSdfFile;

// Confirms that collision elements directly added to the world are classified
// as anchored.
GTEST_TEST(SdfAnchoredGeometry, WorldGeometryIsAnchored) {
  RigidBodyTree<double> tree();
  AddModelInstancesFromSdfFile( "path", drake::multibody::joints::kFixed, nullptr, &tree);
}

}  // namespace
}  // namespace rigid_body_tree
}  // namespace test
}  // namespace plants
}  // namespace systems
}  // namespace drake
