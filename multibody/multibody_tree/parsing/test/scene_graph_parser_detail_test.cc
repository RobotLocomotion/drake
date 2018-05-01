#include "drake/multibody/multibody_tree/parsing/scene_graph_parser_detail.h"

#include <memory>

#include <gtest/gtest.h>

//#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/scene_graph.h"

namespace drake {

using Eigen::Vector3d;
using geometry::Cylinder;
using geometry::HalfSpace;
using geometry::SceneGraph;
using geometry::Shape;
using geometry::Sphere;
using multibody::parsing::detail::MakeShapeFromSdfGeometry;
using std::make_unique;
using std::unique_ptr;
using systems::Context;
using systems::LeafSystem;

namespace multibody {
namespace multibody_plant {
namespace {

// Helper to create an sdf::Geometry object from its SDF specification given
// as a string.
unique_ptr<sdf::Geometry> MakeSdfGeometryFromString(
    const std::string geometry_spec) {
  const std::string sdf_str =
      "<?xml version='1.0'?>"
      "<sdf version='1.6'>"
      "  <model name='my_model'>"
      "    <link name='link'>"
      "      <visual name='link_visual'>"
      "        <geometry>"
          + geometry_spec +
      "        </geometry>"
      "      </visual>"
      "    </link>"
      "  </model>"
      "</sdf>";
  sdf::SDFPtr sdf_parsed(new sdf::SDF());
  sdf::init(sdf_parsed);
  sdf::readString(sdf_str, sdf_parsed);
  sdf::ElementPtr geometry_element =
      sdf_parsed->Root()->GetElement("model")->
          GetElement("link")->GetElement("visual")->GetElement("geometry");
  auto sdf_geometry = make_unique<sdf::Geometry>();
  sdf_geometry->Load(geometry_element);
  return sdf_geometry;
}

// Verify MakeShapeFromSdfGeometry can make a cylinder from an sdf::Geometry.
GTEST_TEST(SceneGraphParserDetal, MakeCylinderFromSdfGeometry) {
  unique_ptr<sdf::Geometry> sdf_geometry = MakeSdfGeometryFromString(
      "<cylinder>"
      "  <radius>0.5</radius>"
      "  <length>1.2</length>"
      "</cylinder>");
  unique_ptr<Shape> shape = MakeShapeFromSdfGeometry(*sdf_geometry);
  const Cylinder& cylinder = dynamic_cast<const Cylinder&>(*shape);
  EXPECT_EQ(cylinder.get_radius(), 0.5);
  EXPECT_EQ(cylinder.get_length(), 1.2);
}

// Verify MakeShapeFromSdfGeometry can make a sphere from an sdf::Geometry.
GTEST_TEST(SceneGraphParserDetal, MakeSphereFromSdfGeometry) {
  unique_ptr<sdf::Geometry> sdf_geometry = MakeSdfGeometryFromString(
      "<sphere>"
      "  <radius>0.5</radius>"
      "</sphere>");
  unique_ptr<Shape> shape = MakeShapeFromSdfGeometry(*sdf_geometry);
  const Sphere& sphere = dynamic_cast<const Sphere&>(*shape);
  EXPECT_EQ(sphere.get_radius(), 0.5);
}

// Verify MakeShapeFromSdfGeometry can make a half space from an sdf::Geometry.
GTEST_TEST(SceneGraphParserDetal, MakeHalfSpaceFromSdfGeometry) {
  unique_ptr<sdf::Geometry> sdf_geometry = MakeSdfGeometryFromString(
      "<plane>"
      "  <normal>1.0 0.0 0.0</normal>"
      "  <size>1.0 1.0 1.0</size>"
      "</plane>");
  // MakeShapeFromSdfGeometry() ignores <normal> and <size> to create the
  // HalfSpace. Therefore we only verify it created the right object.
  unique_ptr<Shape> shape = MakeShapeFromSdfGeometry(*sdf_geometry);
  EXPECT_TRUE(dynamic_cast<const HalfSpace*>(shape.get()) != nullptr);
}

}  // namespace
}  // namespace multibody_plant
}  // namespace multibody
}  // namespace drake

