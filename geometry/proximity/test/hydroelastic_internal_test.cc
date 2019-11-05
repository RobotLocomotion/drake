#include "drake/geometry/proximity/hydroelastic_internal.h"

#include <functional>
#include <limits>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity_properties.h"

namespace drake {
namespace geometry {
namespace internal {
namespace hydroelastic {
namespace {

using Eigen::Vector3d;
using std::function;

// Tests the simple public API of the hydroelastic::Geometries: adding
// geometries and querying the data stored.
GTEST_TEST(Hydroelastic, GeometriesPopulationAndQuery) {
  Geometries geometries;

  // Ids that haven't been added report as undefined.
  GeometryId rigid_id = GeometryId::get_new_id();
  ProximityProperties rigid_properties;
  rigid_properties.AddProperty(kMaterialGroup, kElastic,
                               std::numeric_limits<double>::infinity());
  AddRigidHydroelasticProperties(1.0, &rigid_properties);

  GeometryId soft_id = GeometryId::get_new_id();
  ProximityProperties soft_properties;
  soft_properties.AddProperty(kMaterialGroup, kElastic, 1e8);
  AddSoftHydroelasticProperties(1.0, &soft_properties);

  GeometryId bad_id = GeometryId::get_new_id();
  EXPECT_EQ(geometries.hydroelastic_type(rigid_id),
            HydroelasticType::kUndefined);
  EXPECT_EQ(geometries.hydroelastic_type(soft_id),
            HydroelasticType::kUndefined);
  EXPECT_EQ(geometries.hydroelastic_type(bad_id), HydroelasticType::kUndefined);

  // Once added, they report the appropriate type.
  geometries.MaybeAddGeometry(Sphere(0.5), soft_id, soft_properties);
  EXPECT_EQ(geometries.hydroelastic_type(soft_id),
            HydroelasticType::kSoft);
  geometries.MaybeAddGeometry(Sphere(0.5), rigid_id, rigid_properties);
  EXPECT_EQ(geometries.hydroelastic_type(rigid_id), HydroelasticType::kRigid);

  // Ids that report the correct type, successfully access the appropriate
  // representation.
  DRAKE_EXPECT_NO_THROW(geometries.soft_geometry(soft_id));
  DRAKE_EXPECT_NO_THROW(geometries.rigid_geometry(rigid_id));
}

GTEST_TEST(Hydroelastic, RemoveGeometry) {
  Geometries geometries;

  // Add a rigid geometry.
  const GeometryId rigid_id = GeometryId::get_new_id();
  ProximityProperties rigid_properties;
  rigid_properties.AddProperty(kMaterialGroup, kElastic,
                               std::numeric_limits<double>::infinity());
  AddRigidHydroelasticProperties(1.0, &rigid_properties);
  geometries.MaybeAddGeometry(Sphere(0.5), rigid_id, rigid_properties);
  ASSERT_EQ(geometries.hydroelastic_type(rigid_id), HydroelasticType::kRigid);

  // Add a soft geometry.
  const GeometryId soft_id = GeometryId::get_new_id();
  ProximityProperties soft_properties;
  soft_properties.AddProperty(kMaterialGroup, kElastic, 1e8);
  AddSoftHydroelasticProperties(1.0, &soft_properties);
  geometries.MaybeAddGeometry(Sphere(0.5), soft_id, soft_properties);
  ASSERT_EQ(geometries.hydroelastic_type(soft_id), HydroelasticType::kSoft);

  // Case 1: Remove a geometry that has no representation is a no-op.
  const GeometryId bad_id = GeometryId::get_new_id();
  ASSERT_EQ(geometries.hydroelastic_type(bad_id), HydroelasticType::kUndefined);
  DRAKE_EXPECT_NO_THROW(geometries.RemoveGeometry(bad_id));
  ASSERT_EQ(geometries.hydroelastic_type(bad_id), HydroelasticType::kUndefined);
  ASSERT_EQ(geometries.hydroelastic_type(rigid_id), HydroelasticType::kRigid);
  ASSERT_EQ(geometries.hydroelastic_type(soft_id), HydroelasticType::kSoft);

  // Case 2: Removing a rigid geometry has no effect on anything else. We copy
  // the geometries to make sure that there is always something else to remain
  // untouched.
  {
    Geometries copy{geometries};
    DRAKE_EXPECT_NO_THROW(copy.RemoveGeometry(rigid_id));
    ASSERT_EQ(copy.hydroelastic_type(rigid_id), HydroelasticType::kUndefined);
    ASSERT_EQ(copy.hydroelastic_type(soft_id), HydroelasticType::kSoft);
  }

  // Case 3: Removing a soft geometry has no effect on anything else. We copy
  // the geometries to make sure that there is always something else to remain
  // untouched.
  {
    Geometries copy{geometries};
    DRAKE_EXPECT_NO_THROW(copy.RemoveGeometry(soft_id));
    ASSERT_EQ(copy.hydroelastic_type(soft_id), HydroelasticType::kUndefined);
    ASSERT_EQ(copy.hydroelastic_type(rigid_id), HydroelasticType::kRigid);
  }
}

// Tests the classification of proximity properties.
GTEST_TEST(Hydroelastic, Classify) {
  // Case: props with no declared "hydroelastic" group is, by definition, an
  // unclassified hydroelastic type.
  EXPECT_EQ(Classify(ProximityProperties()), HydroelasticType::kUndefined);

  // Case: Presence of elastic modulus is insufficient to trigger hydroelastic.
  {
    ProximityProperties props;
    props.AddProperty(kMaterialGroup, kElastic, 1e8);
    EXPECT_EQ(Classify(props), HydroelasticType::kUndefined);
  }

  // The *presence* of the kHydroGroup will trigger the attempt to classify a
  // hydroelastic representation. This uses a property that has no bearing on
  // that classification to be that trigger.
  ProximityProperties trigger_hydro;
  trigger_hydro.AddProperty(kHydroGroup, "dummy", 1);

  // Case: missing elastic modulus is a classification error.
  {
    DRAKE_EXPECT_THROWS_MESSAGE(Classify(trigger_hydro), std::logic_error,
        "Properties .+ missing .+ property; compliance cannot be determined");
  }

  // Case: Infinite "elastic_modulus" should report as rigid.
  {
    ProximityProperties props(trigger_hydro);
    props.AddProperty(kMaterialGroup, kElastic,
                      std::numeric_limits<double>::infinity());
    EXPECT_EQ(Classify(props), HydroelasticType::kRigid);
  }

  // Case: Really, really huge, but not infinite "elastic modulus" should report
  // as soft.
  {
    ProximityProperties props(trigger_hydro);
    props.AddProperty(kMaterialGroup, kElastic,
                      std::numeric_limits<double>::max());
    EXPECT_EQ(Classify(props), HydroelasticType::kSoft);
  }

  // Case: Microscopically small elastic modulus should report as soft.
  {
    ProximityProperties props(trigger_hydro);
    props.AddProperty(kMaterialGroup, kElastic, 1e-14);
    EXPECT_EQ(Classify(props), HydroelasticType::kSoft);
  }

  // Case: Zero elastic modulus should throw an error.
  {
    ProximityProperties props(trigger_hydro);
    props.AddProperty(kMaterialGroup, kElastic, 0.0);
    DRAKE_EXPECT_THROWS_MESSAGE(
        Classify(props), std::logic_error,
        fmt::format(".+bad value for .+'{}'.*", kElastic));
  }

  // Case: Negative elastic modulus should throw an error.
  {
    ProximityProperties props(trigger_hydro);
    props.AddProperty(kMaterialGroup, kElastic, -1.0);
    DRAKE_EXPECT_THROWS_MESSAGE(
        Classify(props), std::logic_error,
        fmt::format(".+bad value for .+'{}'.*", kElastic));
  }
}

class HydroelasticRigidGeometryTest : public ::testing::Test {
 protected:
  /** Creates a simple set of properties for generating rigid geometry. */
  ProximityProperties rigid_properties(double edge_length = 0.1) const {
    ProximityProperties properties;
    AddRigidHydroelasticProperties(edge_length, &properties);
    return properties;
  }
};

// TODO(SeanCurtis-TRI): As new shape specifications are added, they are
//  implicitly unsupported and should be added here (and in
//  UnsupportedSofthapes). I'm particularly thinking of capsule and ellipsoid.
// Smoke test for shapes that are *known* to be unsupported as rigid objects.
// NOTE: This will spew warnings to the log.
TEST_F(HydroelasticRigidGeometryTest, UnsupportedRigidShapes) {
  ProximityProperties props = rigid_properties();

  EXPECT_EQ(MakeRigidRepresentation(Cylinder(1, 1), props), std::nullopt);

  EXPECT_EQ(MakeRigidRepresentation(Capsule(1, 1), props), std::nullopt);

  EXPECT_EQ(MakeRigidRepresentation(Ellipsoid(1, 2, 3), props), std::nullopt);

  EXPECT_EQ(MakeRigidRepresentation(HalfSpace(), props), std::nullopt);

  // Note: the file name doesn't have to be valid for this (and the Mesh) test.
  const std::string obj = "drake/geometry/proximity/test/no_such_files.obj";
  EXPECT_EQ(MakeRigidRepresentation(Convex(obj, 1.0), props), std::nullopt);

  EXPECT_EQ(MakeRigidRepresentation(Mesh(obj, 1.0), props), std::nullopt);
}

// Confirm support for a rigid Sphere. Tests that a hydroelastic representation
// is made, and samples the representation to look for evidence of it being the
// *right* representation.
TEST_F(HydroelasticRigidGeometryTest, Sphere) {
  const double radius = 0.5;
  Sphere sphere_spec(radius);

  ProximityProperties props = rigid_properties(0.5);
  std::optional<RigidGeometry> sphere =
      MakeRigidRepresentation(sphere_spec, props);
  EXPECT_NE(sphere, std::nullopt);

  const SurfaceMesh<double>& mesh = sphere->mesh();
  for (SurfaceVertexIndex v(0); v < mesh.num_vertices(); ++v) {
    ASSERT_NEAR(mesh.vertex(v).r_MV().norm(), radius, 1e-15);
  }
}

// Confirm support for a rigid Box. Tests that a hydroelastic representation
// is made, and samples the representation to look for evidence of it being the
// *right* representation.
TEST_F(HydroelasticRigidGeometryTest, Box) {
  const double edge_len = 0.5;
  // Pick a characteristic length *larger* than the box dimensions so that
  // I only get 8 vertices - one at each corner. This is merely evidence that
  // the box method is called -- we rely on tests of that functionality to
  // create more elaborate meshes with smaller edge lengths.
  ProximityProperties props = rigid_properties(1.5 * edge_len);

  std::optional<RigidGeometry> box =
      MakeRigidRepresentation(Box(edge_len, edge_len, edge_len), props);
  EXPECT_NE(box, std::nullopt);

  const SurfaceMesh<double>& mesh = box->mesh();
  EXPECT_EQ(mesh.num_vertices(), 8);
  // Because it is a cube centered at the origin, the distance from the origin
  // to each vertex should be sqrt(3) * edge_len / 2.
  const double expecte_dist = std::sqrt(3) * edge_len / 2;
  for (SurfaceVertexIndex v(0); v < mesh.num_vertices(); ++v) {
    ASSERT_NEAR(mesh.vertex(v).r_MV().norm(), expecte_dist, 1e-15);
  }
}

// Template magic to instantiate a particular kind of shape at compile time.
template <typename ShapeType>
ShapeType make_default_shape() {
  throw std::logic_error(
      "Running Hydroelastic*GeometryErrorTests for an unsupported shape "
      "type");
}

template <>
Sphere make_default_shape<Sphere>() {
  return Sphere(0.5);
}

template <>
Box make_default_shape<Box>() {
  return Box(0.5, 1.25, 3.5);
}

// Boilerplate for testing error conditions relating to properties. Its purpose
// is to test that the `Make*Representation` (either "Rigid" or "Soft")
// family of functions correctly validate all required properties. A property
// value can be wrong for one of three reasons:
//
//   - Missing property
//   - Wrong type for property
//   - Invalid value (maybe)
//
// This is sufficiently generic to test both kinds of geometry (rigid and soft)
// based on any shape, for a property of any type. It confirms that properly
// formatted errors are emitted in all cases.
//
// Not all properties validate invalid values. It might simply be treated as a
// black box. The third error condition is only handled if an example "bad"
// value is provided (see `bad_value` below).
//
// @param shape_spec     The shape from which we attempt to create a hydro-
//                       elastic representation.
// @param group_name     The name of the property group (in which the property
//                       lives).
// @param property_name  The name of the property (in the `group_name` group)
//                       to be tested.
// @param compliance     A string representing the compliance being requested
//                       ("rigid" or "soft").
// @param maker          The function that processes the shape and properties.
//                       Note: this is declared to return void; the
//                       Make*Representation() methods will need to be wrapped.
// @param bad_value      If provided, sets the property to this value to test
//                       validation against bad values.
// @param props          The baseline properties to start the test from --
//                       properties are generally tested in some sequence.
//                       Each test may add a property to a copy of this input
//                       to test a particular aspect.
// @tparam ShapeType  The derived class from geometry::Shape (e.g., Sphere, Box,
//                    etc.)
// @tparam ValueTYpe  The type of value under test.
template <typename ShapeType, typename ValueType>
void TestPropertyErrors(
    const ShapeType& shape_spec, const char* group_name,
    const char* property_name, const char* compliance,
    function<void(const ShapeType&, const ProximityProperties&)> maker,
    std::optional<ValueType> bad_value, const ProximityProperties& props) {
  ShapeName shape_name(shape_spec);

  // Error case: missing property value.
  {
    DRAKE_EXPECT_THROWS_MESSAGE(
        maker(shape_spec, props), std::logic_error,
        fmt::format("Cannot create {} {}.+'{}'\\) property", compliance,
                    shape_name, property_name));
  }

  // Error case: property value is wrong type.
  {
    ProximityProperties wrong_value(props);
    wrong_value.AddProperty(group_name, property_name, "10");
    // This error message comes from GeometryProperties::GetProperty().
    DRAKE_EXPECT_THROWS_MESSAGE(
        maker(shape_spec, wrong_value), std::logic_error,
        fmt::format(".*The property '{}' .+ exists, but is of a different "
                    "type.+string'",
                    property_name));
  }

  // Error case: property value is not positive.
  if (bad_value.has_value()) {
    ProximityProperties negative_value(props);
    negative_value.AddProperty(group_name, property_name, *bad_value);
    DRAKE_EXPECT_THROWS_MESSAGE(
        maker(shape_spec, negative_value), std::logic_error,
        fmt::format("Cannot create {} {}.+'{}'.+ positive", compliance,
                    shape_name, property_name));
  }
}

// TODO(SeanCurtis-TRI): Add Cylinder, Mesh, Capsule, Ellipsoid and Convex as
//  they become supported by either rigid or soft geometries.

// Test suite for testing the common failure conditions for generating rigid
// geometry. Specifically, they just need to be tessellated into a triangle mesh
// and, therefore, only depend on the (hydroelastic, characteristic_length)
// value. This actively excludes half spaces because they don't get
// tessellated (see the `RigidErrorShapeTypes` declaration below.) It should
// include every *other* supported rigid shape type.
template <typename ShapeType>
class HydroelasticRigidGeometryErrorTests : public ::testing::Test {};

TYPED_TEST_SUITE_P(HydroelasticRigidGeometryErrorTests);

TYPED_TEST_P(HydroelasticRigidGeometryErrorTests, BadCharacteristicLength) {
  using ShapeType = TypeParam;
  ShapeType shape_spec = make_default_shape<ShapeType>();

  TestPropertyErrors<ShapeType, double>(
      shape_spec, kHydroGroup, kRezHint, "rigid",
      [](const ShapeType& s, const ProximityProperties& p) {
        MakeRigidRepresentation(s, p);
      },
      -0.2, {});
}

REGISTER_TYPED_TEST_SUITE_P(HydroelasticRigidGeometryErrorTests,
                           BadCharacteristicLength);
typedef ::testing::Types<Sphere, Box> RigidErrorShapeTypes;
INSTANTIATE_TYPED_TEST_SUITE_P(My, HydroelasticRigidGeometryErrorTests,
                              RigidErrorShapeTypes);

class HydroelasticSoftGeometryTest : public ::testing::Test {
 protected:
  /** Creates a simple set of properties for generating soft geometry. */
  ProximityProperties soft_properties(double edge_length = 0.1) const {
    ProximityProperties soft_properties;
    soft_properties.AddProperty(kMaterialGroup, kElastic, 1e8);
    AddSoftHydroelasticProperties(edge_length, &soft_properties);
    return soft_properties;
  }
};

// TODO(SeanCurtis-TRI): As new shape specifications are added, they are
//  implicitly unsupported and should be added here (and in
//  UnsupportedRigidShapes). I'm particularly thinking of capsule and ellipsoid.
// Smoke test for shapes that are *known* to be unsupported as soft objects.
// NOTE: This will spew warnings to the log.
TEST_F(HydroelasticSoftGeometryTest, UnsupportedSoftShapes) {
  ProximityProperties props = soft_properties();

  EXPECT_EQ(MakeSoftRepresentation(Box(1, 1, 1), props), std::nullopt);

  EXPECT_EQ(MakeSoftRepresentation(Cylinder(1, 1), props), std::nullopt);

  EXPECT_EQ(MakeSoftRepresentation(Capsule(1, 1), props), std::nullopt);

  EXPECT_EQ(MakeSoftRepresentation(Ellipsoid(1, 2, 3), props), std::nullopt);

  // Note: the file name doesn't have to be valid for this (and the Mesh) test.
  const std::string obj = "drake/geometry/proximity/test/no_such_files.obj";
  EXPECT_EQ(MakeSoftRepresentation(Convex(obj, 1.0), props), std::nullopt);

  EXPECT_EQ(MakeSoftRepresentation(Mesh(obj, 1.0), props), std::nullopt);
}

// Test construction of a soft sphere. Confirms that the edge length has
// an effect (i.e., that shrinking the edge_length by at least half causes
// a change in mesh resolution. It relies on unit tests for the unit sphere
// generation to confirm that it's the *right* number of tetrahedron. This
// merely confirms that the characteristic_length is being fed in.
TEST_F(HydroelasticSoftGeometryTest, Sphere) {
  const double kRadius = 0.5;
  Sphere sphere_spec(kRadius);

  // Confirm that characteristic length is being fed in properly -- i.e.,
  // if characteristic length cuts in half, It should have more tetrahedra.
  ProximityProperties properties1 = soft_properties(kRadius);
  ProximityProperties properties2 = soft_properties(kRadius / 2);
  std::optional<SoftGeometry> sphere1 =
      MakeSoftRepresentation(sphere_spec, properties1);
  std::optional<SoftGeometry> sphere2 =
      MakeSoftRepresentation(sphere_spec, properties2);
  EXPECT_LT(sphere1->mesh().num_elements(), sphere2->mesh().num_elements());

  // Confirm that all vertices lie inside the sphere and that at least one lies
  // on the boundary.
  double max_distance = -1.0;
  for (const auto& soft_geometry : {*sphere1, *sphere2}) {
    const VolumeMesh<double>& mesh = soft_geometry.mesh();
    for (VolumeVertexIndex v(0); v < mesh.num_vertices(); ++v) {
      const double dist = mesh.vertex(v).r_MV().norm();
      max_distance = std::max(max_distance, dist);
      ASSERT_LE(dist, kRadius);
    }
  }

  ASSERT_NEAR(max_distance, kRadius, 1e-15);

  // Confirm pressure field is as specified in the properties.
  const double E =
      properties1.GetPropertyOrDefault(kMaterialGroup, kElastic, 1e8);
  // We assume that the sphere's pressure is defined as E * (1 - r/R).
  auto pressure = [E, kRadius](const Vector3d& r_MV) {
    return E * (1.0 - r_MV.norm() / kRadius);
  };
  const double kEps = std::numeric_limits<double>::epsilon();
  const VolumeMesh<double>& mesh = sphere1->mesh();
  for (VolumeVertexIndex v(0); v < mesh.num_vertices(); ++v) {
    const VolumeVertex<double>& vertex = mesh.vertex(v);
    // Zero on outside, 1 on inside.
    const double expected_p = pressure(vertex.r_MV());
    EXPECT_NEAR(sphere1->pressure_field().EvaluateAtVertex(v), expected_p,
                kEps);
  }
}

// Test suite for testing the common failure conditions for generating soft
// geometry. Specifically, they need to be tessellated into a tet mesh
// and define a pressure field. This actively excludes half spaces because they
// are treated specially (they don't get tessellated).
// (See the `RigidErrorShapeTypes` declaration below.) It should include every
// *other* supported rigid shape type.
template <typename ShapeType>
class HydroelasticSoftGeometryErrorTests : public ::testing::Test {};

TYPED_TEST_SUITE_P(HydroelasticSoftGeometryErrorTests);

TYPED_TEST_P(HydroelasticSoftGeometryErrorTests, BadCharacteristicLength) {
  using ShapeType = TypeParam;
  ShapeType shape_spec = make_default_shape<ShapeType>();
  TestPropertyErrors<ShapeType, double>(
      shape_spec, kHydroGroup, kRezHint, "soft",
      [](const ShapeType& s, const ProximityProperties& p) {
        MakeSoftRepresentation(s, p);
      },
      -0.2, {});
}

TYPED_TEST_P(HydroelasticSoftGeometryErrorTests, BadElasticModulus) {
  using ShapeType = TypeParam;
  ShapeType shape_spec = make_default_shape<ShapeType>();

  ProximityProperties soft_properties;
  // Add the resolution hint so that creation of the hydroelastic representation
  // can choke on elastic modulus value.
  soft_properties.AddProperty(kHydroGroup, kRezHint, 10.0);
  TestPropertyErrors<ShapeType, double>(
      shape_spec, kMaterialGroup, kElastic, "soft",
      [](const ShapeType& s, const ProximityProperties& p) {
        MakeSoftRepresentation(s, p);
      },
      -0.2, soft_properties);
}

REGISTER_TYPED_TEST_SUITE_P(HydroelasticSoftGeometryErrorTests,
                           BadCharacteristicLength, BadElasticModulus);
typedef ::testing::Types<Sphere> SoftErrorShapeTypes;
INSTANTIATE_TYPED_TEST_SUITE_P(My, HydroelasticSoftGeometryErrorTests,
                              SoftErrorShapeTypes);

}  // namespace
}  // namespace hydroelastic
}  // namespace internal
}  // namespace geometry
}  // namespace drake
