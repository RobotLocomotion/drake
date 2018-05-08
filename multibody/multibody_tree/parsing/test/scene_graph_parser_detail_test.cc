#include "drake/multibody/multibody_tree/parsing/scene_graph_parser_detail.h"

#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace multibody {
namespace multibody_plant {
namespace {

using Eigen::Isometry3d;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using geometry::Cylinder;
using geometry::GeometryInstance;
using geometry::HalfSpace;
using geometry::SceneGraph;
using geometry::Shape;
using geometry::Sphere;
using math::RollPitchYaw;
using math::RotationMatrix;
using multibody::parsing::detail::MakeGeometryInstanceFromSdfVisual;
using multibody::parsing::detail::MakeShapeFromSdfGeometry;
using std::make_unique;
using std::unique_ptr;
using systems::Context;
using systems::LeafSystem;

// Helper to create an sdf::Geometry object from its SDF specification given
// as a string. Example of what the string should contain:
//   <cylinder>
//     <radius>0.5</radius>
//     <length>1.2</length>
//   </cylinder>
// and similarly for other SDF geometries.
unique_ptr<sdf::Geometry> MakeSdfGeometryFromString(
    const std::string& geometry_spec) {
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

// Helper to create an sdf::Visual object from its SDF specification given
// as a string. Example of what the string should contain:
//       <visual name = 'some_link_visual'>
//         <pose>1.0 2.0 3.0 3.14 6.28 1.57</pose>
//         <geometry>
//           <cylinder>
//             <radius>0.5</radius>
//             <length>1.2</length>
//           </cylinder>
//         </geometry>
//       </visual>
unique_ptr<sdf::Visual> MakeSdfVisualFromString(
    const std::string& visual_spec) {
  const std::string sdf_str =
      "<?xml version='1.0'?>"
      "<sdf version='1.6'>"
      "  <model name='my_model'>"
      "    <link name='link'>"
      + visual_spec +
      "    </link>"
      "  </model>"
      "</sdf>";
  sdf::SDFPtr sdf_parsed(new sdf::SDF());
  sdf::init(sdf_parsed);
  sdf::readString(sdf_str, sdf_parsed);
  sdf::ElementPtr visual_element =
      sdf_parsed->Root()->GetElement("model")->
          GetElement("link")->GetElement("visual");
  auto sdf_visual = make_unique<sdf::Visual>();
  sdf_visual->Load(visual_element);
  return sdf_visual;
}

// Verify MakeShapeFromSdfGeometry returns nullptr when we specify an <empty>
// sdf::Geometry.
GTEST_TEST(SceneGraphParserDetail, MakeEmptyFromSdfGeometry) {
  unique_ptr<sdf::Geometry> sdf_geometry =
      MakeSdfGeometryFromString("<empty/>");
  unique_ptr<Shape> shape = MakeShapeFromSdfGeometry(*sdf_geometry);
  EXPECT_EQ(shape, nullptr);
}

// Verify MakeShapeFromSdfGeometry can make a cylinder from an sdf::Geometry.
GTEST_TEST(SceneGraphParserDetail, MakeCylinderFromSdfGeometry) {
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
GTEST_TEST(SceneGraphParserDetail, MakeSphereFromSdfGeometry) {
  unique_ptr<sdf::Geometry> sdf_geometry = MakeSdfGeometryFromString(
      "<sphere>"
      "  <radius>0.5</radius>"
      "</sphere>");
  unique_ptr<Shape> shape = MakeShapeFromSdfGeometry(*sdf_geometry);
  const Sphere& sphere = dynamic_cast<const Sphere&>(*shape);
  EXPECT_EQ(sphere.get_radius(), 0.5);
}

// Verify MakeShapeFromSdfGeometry can make a half space from an sdf::Geometry.
GTEST_TEST(SceneGraphParserDetail, MakeHalfSpaceFromSdfGeometry) {
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

// Verify MakeGeometryInstanceFromSdfVisual can make a GeometryInstance from an
// sdf::Visual.
// Since we test MakeShapeFromSdfGeometry separately, there is no need to unit
// test every combination of a <visual> with a different <geometry>.
GTEST_TEST(SceneGraphParserDetail, MakeGeometryInstanceFromSdfVisual) {
  unique_ptr<sdf::Visual> sdf_visual = MakeSdfVisualFromString(
      "<visual name = 'some_link_visual'>"
      "  <pose>1.0 2.0 3.0 3.14 6.28 1.57</pose>"
      "  <geometry>"
      "    <cylinder>"
      "      <radius>0.5</radius>"
      "      <length>1.2</length>"
      "    </cylinder>"
      "  </geometry>"
      "</visual>");
  unique_ptr<GeometryInstance> geometry_instance =
      MakeGeometryInstanceFromSdfVisual(*sdf_visual);

  const Isometry3d& X_LC = geometry_instance->pose();

  // Thes are the expected values as specified by the string above.
  const Vector3d expected_rpy(3.14, 6.28, 1.57);
  const Matrix3d R_LC_expected =
      RotationMatrix<double>(RollPitchYaw<double>(expected_rpy)).matrix();
  const Vector3d p_LCo_expected(1.0, 2.0, 3.0);

  // Verify results to precision given by kTolerance.
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(X_LC.linear(), R_LC_expected,
                              kTolerance, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(X_LC.translation(), p_LCo_expected,
                              kTolerance, MatrixCompareType::relative));
}

// Verify MakeGeometryInstanceFromSdfVisual can make a GeometryInstance from an
// sdf::Visual with a <plane> geometry.
// We test this case separately since, while geometry::HalfSpace is defined in a
// canonical frame C whose pose needs to be specified at a GeometryInstance
// level, the SDF specification does not define this pose at the <geometry>
// level but at the <visual> (or <collision>) level.
GTEST_TEST(SceneGraphParserDetail, MakeHalfSpaceGeometryInstanceFromSdfVisual) {
  unique_ptr<sdf::Visual> sdf_visual = MakeSdfVisualFromString(
      "<visual name = 'some_link_visual'>"
      "  <pose>0.0 0.0 0.0 0.0 0.0 0.0</pose>"
      "  <geometry>"
      "    <plane>"
      "      <normal>1.0 2.0 3.0</normal>"
      "    </plane>"
      "  </geometry>"
      "</visual>");
  unique_ptr<GeometryInstance> geometry_instance =
      MakeGeometryInstanceFromSdfVisual(*sdf_visual);

  // Verify we do have a plane geometry.
  const HalfSpace* shape =
      dynamic_cast<const HalfSpace*>(&geometry_instance->shape());
  ASSERT_TRUE(shape != nullptr);

  // The expected coordinates of the normal vector in the link frame L.
  const Vector3d normal_L_expected = Vector3d(1.0, 2.0, 3.0).normalized();

  // The expected orientation of the canonical frame C (in which the plane's
  // normal aligns with Cz) in the link frame L.
  const Matrix3d R_LC_expected =
      HalfSpace::MakePose(normal_L_expected, Vector3d::Zero()).linear();

  // Retrieve the GeometryInstance pose as parsed from the sdf::Visual.
  const Isometry3d& X_LC = geometry_instance->pose();
  const Matrix3d& R_LC = X_LC.linear();
  const Vector3d normal_L = R_LC.col(2);

  // Verify results to precision given by kTolerance.
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(X_LC.linear(), R_LC_expected,
                              kTolerance, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(normal_L, normal_L_expected,
                              kTolerance, MatrixCompareType::relative));
}

// Verify MakeSdfVisualFromString() returns nullptr when the visual specifies
// an <empty/> geometry.
GTEST_TEST(SceneGraphParserDetail, MakeEmptyGeometryInstanceFromSdfVisual) {
  unique_ptr<sdf::Visual> sdf_visual = MakeSdfVisualFromString(
      "<visual name = 'some_link_visual'>"
      "  <pose>0.0 0.0 0.0 0.0 0.0 0.0</pose>"
      "  <geometry>"
      "    <empty/>"
      "  </geometry>"
      "</visual>");
  unique_ptr<GeometryInstance> geometry_instance =
      MakeGeometryInstanceFromSdfVisual(*sdf_visual);
  EXPECT_EQ(geometry_instance, nullptr);
}

}  // namespace
}  // namespace multibody_plant
}  // namespace multibody
}  // namespace drake

