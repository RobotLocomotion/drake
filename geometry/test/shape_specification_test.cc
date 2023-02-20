#include "drake/geometry/shape_specification.h"

#include <filesystem>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/fmt_eigen.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/test_utilities/is_dynamic_castable.h"
#include "drake/common/unused.h"

namespace drake {
namespace geometry {
namespace {

using math::RigidTransformd;
using std::unique_ptr;

// Confirm correct interactions with the Reifier.

class ReifierTest : public ShapeReifier, public ::testing::Test {
 public:
  void ImplementGeometry(const Box&, void* data) override {
    received_user_data_ = data;
    box_made_ = true;
  }
  void ImplementGeometry(const Capsule&, void* data) override {
    received_user_data_ = data;
    capsule_made_ = true;
  }
  void ImplementGeometry(const Convex&, void* data) override {
    received_user_data_ = data;
    convex_made_ = true;
  }
  void ImplementGeometry(const Cylinder&, void* data) override {
    received_user_data_ = data;
    cylinder_made_ = true;
  }
  void ImplementGeometry(const Ellipsoid&, void* data) override {
    received_user_data_ = data;
    ellipsoid_made_ = true;
  }
  void ImplementGeometry(const HalfSpace&, void* data) override {
    received_user_data_ = data;
    half_space_made_ = true;
  }
  void ImplementGeometry(const Mesh&, void* data) override {
    received_user_data_ = data;
    mesh_made_ = true;
  }
  void ImplementGeometry(const MeshcatCone&, void* data) override {
    received_user_data_ = data;
    meshcat_cone_made_ = true;
  }
  void ImplementGeometry(const Sphere&, void* data) override {
    received_user_data_ = data;
    sphere_made_ = true;
  }
  void Reset() {
    box_made_ = false;
    capsule_made_ = false;
    convex_made_ = false;
    cylinder_made_ = false;
    ellipsoid_made_ = false;
    half_space_made_ = false;
    mesh_made_ = false;
    meshcat_cone_made_ = false;
    sphere_made_ = false;
    received_user_data_ = nullptr;
  }

 protected:
  bool box_made_{false};
  bool capsule_made_{false};
  bool convex_made_{false};
  bool cylinder_made_{false};
  bool ellipsoid_made_{false};
  bool half_space_made_{false};
  bool mesh_made_{false};
  bool meshcat_cone_made_{false};
  bool sphere_made_{false};
  void* received_user_data_{nullptr};
};

// Confirms that user data comes through intact.
TEST_F(ReifierTest, UserData) {
  ASSERT_EQ(received_user_data_, nullptr);
  Sphere s(1.0);
  std::pair<int, double> user_data{3, 5.0};
  s.Reify(this, &user_data);
  ASSERT_EQ(received_user_data_, &user_data);
}

// This confirms that the shapes invoke the correct Reify method.
TEST_F(ReifierTest, ReificationDifferentiation) {
  const Box box{1, 2, 3};
  box.Reify(this);
  EXPECT_FALSE(sphere_made_);
  EXPECT_FALSE(half_space_made_);
  EXPECT_FALSE(cylinder_made_);
  EXPECT_TRUE(box_made_);
  EXPECT_FALSE(capsule_made_);
  EXPECT_FALSE(convex_made_);
  EXPECT_FALSE(ellipsoid_made_);
  EXPECT_FALSE(mesh_made_);
  EXPECT_FALSE(meshcat_cone_made_);

  Reset();

  const Capsule capsule{2, 1};
  capsule.Reify(this);
  EXPECT_FALSE(sphere_made_);
  EXPECT_FALSE(half_space_made_);
  EXPECT_FALSE(cylinder_made_);
  EXPECT_FALSE(box_made_);
  EXPECT_TRUE(capsule_made_);
  EXPECT_FALSE(convex_made_);
  EXPECT_FALSE(ellipsoid_made_);
  EXPECT_FALSE(mesh_made_);
  EXPECT_FALSE(meshcat_cone_made_);

  Reset();

  const Convex convex{"fictitious_name.obj", 1.0};
  convex.Reify(this);
  EXPECT_FALSE(sphere_made_);
  EXPECT_FALSE(half_space_made_);
  EXPECT_FALSE(cylinder_made_);
  EXPECT_FALSE(box_made_);
  EXPECT_FALSE(capsule_made_);
  EXPECT_TRUE(convex_made_);
  EXPECT_FALSE(ellipsoid_made_);
  EXPECT_FALSE(mesh_made_);
  EXPECT_FALSE(meshcat_cone_made_);

  Reset();

  const Cylinder cylinder{1, 2};
  cylinder.Reify(this);
  EXPECT_FALSE(sphere_made_);
  EXPECT_FALSE(half_space_made_);
  EXPECT_TRUE(cylinder_made_);
  EXPECT_FALSE(box_made_);
  EXPECT_FALSE(capsule_made_);
  EXPECT_FALSE(convex_made_);
  EXPECT_FALSE(ellipsoid_made_);
  EXPECT_FALSE(mesh_made_);
  EXPECT_FALSE(meshcat_cone_made_);

  Reset();

  const Ellipsoid ellipsoid{1, 2, 3};
  ellipsoid.Reify(this);
  EXPECT_FALSE(sphere_made_);
  EXPECT_FALSE(half_space_made_);
  EXPECT_FALSE(cylinder_made_);
  EXPECT_FALSE(box_made_);
  EXPECT_FALSE(capsule_made_);
  EXPECT_FALSE(convex_made_);
  EXPECT_TRUE(ellipsoid_made_);
  EXPECT_FALSE(mesh_made_);
  EXPECT_FALSE(meshcat_cone_made_);

  Reset();

  const HalfSpace hs{};
  hs.Reify(this);
  EXPECT_FALSE(sphere_made_);
  EXPECT_TRUE(half_space_made_);
  EXPECT_FALSE(cylinder_made_);
  EXPECT_FALSE(box_made_);
  EXPECT_FALSE(capsule_made_);
  EXPECT_FALSE(convex_made_);
  EXPECT_FALSE(ellipsoid_made_);
  EXPECT_FALSE(mesh_made_);
  EXPECT_FALSE(meshcat_cone_made_);

  Reset();

  const Mesh mesh{"fictitious_mesh_name.obj", 1.4};
  mesh.Reify(this);
  EXPECT_FALSE(sphere_made_);
  EXPECT_FALSE(half_space_made_);
  EXPECT_FALSE(cylinder_made_);
  EXPECT_FALSE(box_made_);
  EXPECT_FALSE(capsule_made_);
  EXPECT_FALSE(convex_made_);
  EXPECT_FALSE(ellipsoid_made_);
  EXPECT_TRUE(mesh_made_);
  EXPECT_FALSE(meshcat_cone_made_);

  Reset();

  const MeshcatCone cone{1.2, 3.4, 5.6};
  cone.Reify(this);
  EXPECT_FALSE(sphere_made_);
  EXPECT_FALSE(half_space_made_);
  EXPECT_FALSE(cylinder_made_);
  EXPECT_FALSE(box_made_);
  EXPECT_FALSE(capsule_made_);
  EXPECT_FALSE(convex_made_);
  EXPECT_FALSE(ellipsoid_made_);
  EXPECT_FALSE(mesh_made_);
  EXPECT_TRUE(meshcat_cone_made_);

  Reset();

  const Sphere s(1.0);
  s.Reify(this);
  EXPECT_TRUE(sphere_made_);
  EXPECT_FALSE(half_space_made_);
  EXPECT_FALSE(cylinder_made_);
  EXPECT_FALSE(box_made_);
  EXPECT_FALSE(capsule_made_);
  EXPECT_FALSE(convex_made_);
  EXPECT_FALSE(ellipsoid_made_);
  EXPECT_FALSE(mesh_made_);
  EXPECT_FALSE(meshcat_cone_made_);
}

// Confirms that the ReifiableShape properly clones the right types.
TEST_F(ReifierTest, CloningShapes) {
  Sphere s{1.0};
  ASSERT_TRUE(is_dynamic_castable<Sphere>(s.Clone().get()));
  HalfSpace h;
  ASSERT_TRUE(is_dynamic_castable<HalfSpace>(h.Clone().get()));
  Cylinder c(0.5, 2.0);
  ASSERT_TRUE(is_dynamic_castable<Cylinder>(c.Clone().get()));

  // Confirms clone independence. The idea that a clone can outlive its
  // source and there aren't any unintentional bindings between the two.
  unique_ptr<Shape> cloned_shape{};
  {
    Sphere local_sphere{1.23};
    cloned_shape = local_sphere.Clone();
    Sphere* raw_sphere = static_cast<Sphere*>(cloned_shape.get());
    // Confirm it's an appropriate copy.
    ASSERT_EQ(raw_sphere->radius(), local_sphere.radius());
  }
  // Now confirm it's still alive. I should be able to reify it into a sphere.
  cloned_shape->Reify(this);
  ASSERT_FALSE(box_made_);
  ASSERT_FALSE(capsule_made_);
  EXPECT_FALSE(convex_made_);
  ASSERT_FALSE(cylinder_made_);
  ASSERT_FALSE(half_space_made_);
  ASSERT_FALSE(ellipsoid_made_);
  EXPECT_FALSE(mesh_made_);
  ASSERT_TRUE(sphere_made_);
}

// Given the pose of a plane and its expected translation and z-axis, confirms
// that the pose conforms to expectations. Also confirms that the rotational
// component is orthonormal.
::testing::AssertionResult ValidatePlanePose(
    const RigidTransformd& pose, const Vector3<double>& expected_z,
    const Vector3<double>& expected_translation, double tolerance = 1e-14) {
  using std::abs;

  // Test expected z-axis value.
  const Vector3<double>& z_axis = pose.rotation().col(2);
  if (!CompareMatrices(z_axis, expected_z, tolerance,
                       MatrixCompareType::absolute)) {
    const std::string message = fmt::format(
        "pose =\n{}\nExpected z-axis {} does not match pose's z-axis {}",
        fmt_eigen(pose.GetAsMatrix34()), fmt_eigen(expected_z.transpose()),
        fmt_eigen(z_axis.transpose()));
    return ::testing::AssertionFailure() << message;
  }

  // Test expected translation.
  if (!CompareMatrices(pose.translation(), expected_translation, tolerance,
                       MatrixCompareType::absolute)) {
    const std::string message = fmt::format(
        "pose =\n{}\nExpected translation {} does not match pose's "
        "translation {}",
        fmt_eigen(pose.GetAsMatrix34()),
        fmt_eigen(expected_translation.transpose()),
        fmt_eigen(pose.translation().transpose()));
    return ::testing::AssertionFailure() << message;
  }

  // Test unit-length rotation.
  char axis_labels[] = {'x', 'y', 'z'};
  for (int i = 0; i < 3; ++i) {
    if (abs(pose.rotation().col(i).norm() - 1) > tolerance) {
      const std::string message =
          fmt::format("pose =\n{}\ndoes not have unit length {}-axis {}",
                      fmt_eigen(pose.GetAsMatrix34()), axis_labels[i],
                      fmt_eigen(pose.rotation().col(i).transpose()));
      return ::testing::AssertionFailure() << message;
    }
  }

  // Test orthogonality.
  for (int i = 0; i < 2; ++i) {
    for (int j = i + 1; j < 3; ++j) {
      double dot_product = pose.rotation().col(i).dot(pose.rotation().col(j));
      if (abs(dot_product) > tolerance) {
        const std::string message = fmt::format(
            "For pose =\n{}\nThe {}-axis and {}-axis are not orthogonal",
            fmt_eigen(pose.GetAsMatrix34()), axis_labels[i], axis_labels[j]);
        return ::testing::AssertionFailure() << message;
      }
    }
  }
  const std::string message = fmt::format(
      "pose =\n{}\nhas expected z-axis = {}\nand expected translation = {}",
      fmt_eigen(pose.GetAsMatrix34()), fmt_eigen(expected_z.transpose()),
      fmt_eigen(expected_translation.transpose()));
  return ::testing::AssertionSuccess() << message;
}

// Confirms that the pose computed by HalfSpace::X_FC() is consistent with
// the normal and point provided.
GTEST_TEST(HalfSpaceTest, MakePose) {
  Vector3<double> n;
  Vector3<double> p;

  // All of this assumes that, in the canonical space, the half space's outward-
  // pointing normal points in the +z axis direction.

  // Case: n(0, 0, 1), p(0, 0, 0) --> <0, 0, 1> z-axis
  {
    n << 0, 0, 1;
    p << 0, 0, 0;
    RigidTransformd pose = HalfSpace::MakePose(n, p);
    EXPECT_TRUE(ValidatePlanePose(pose, n, p));
  }

  // Case: n(0, 0, 1), p(0, 0, 1) --> <0, 0, 1> z-axis, <0, 0, 1> translation.
  {
    n << 0, 0, 1;
    p << 0, 0, 1;
    RigidTransformd pose = HalfSpace::MakePose(n, p);
    EXPECT_TRUE(ValidatePlanePose(pose, n, p));
  }

  // Case: n(0, 0, 1), p(1, 0, 0) --> <0, 0, 1> z-axis, <0, 0, 0> translation.
  // Confirms that the pose's translation is the minimum translation.
  {
    n << 0, 0, 1;
    p << 1, 0, 0;
    RigidTransformd pose = HalfSpace::MakePose(n, p);
    EXPECT_TRUE(ValidatePlanePose(pose, n, Vector3<double>::Zero()));
  }

  // Case: n(0, 0, 1), p(1, 1, 1) --> identity rotation, <0, 0, 1> translation.
  // Confirms that the pose's translation is the minimum translation.
  {
    n << 0, 0, 1;
    p << 1, 1, 1;
    RigidTransformd pose = HalfSpace::MakePose(n, p);
    p << 0, 0, 1;
    EXPECT_TRUE(ValidatePlanePose(pose, n, p));
  }

  // Case: n(1, 1, 1), p(0, 0, 0) --> rotation z-axis = <1, 1, 1> / √3,
  //       <0, 0, 0> translation. Rotation matrix orthonormal.
  {
    n << 1, 1, 1;
    p << 0, 0, 0;
    RigidTransformd pose = HalfSpace::MakePose(n, p);
    EXPECT_TRUE(ValidatePlanePose(pose, n.normalized(), p));
  }

  // Case: n(1, 1, 1), p(1, 1, 1) --> rotation z-axis = <1, 1, 1> / √3,
  //       <1, 1, 1> translation. Rotation matrix orthonormal.
  {
    n << 1, 1, 1;
    p << 1, 1, 1;
    RigidTransformd pose = HalfSpace::MakePose(n, p);
    EXPECT_TRUE(ValidatePlanePose(pose, n.normalized(), p));
  }

  // Case: n(1, 1, 1), p(1, 0, 0) --> rotation z-axis = <1, 1, 1> / √3,
  //       <1/3, 1/3, 1/3> translation. Rotation matrix orthonormal.
  // Confirms that the pose's translation is the minimum translation.
  {
    n << 1, 1, 1;
    p << 1, 0, 0;
    RigidTransformd pose = HalfSpace::MakePose(n, p);
    p = n / 3.0;
    EXPECT_TRUE(ValidatePlanePose(pose, n.normalized(), p));
  }

  // Case: Normal magnitude too small should produce an exception.
  {
    n << 1, 1, 1;
    n *= 1e-11;
    p << 0, 0, 0;
    EXPECT_THROW(HalfSpace::MakePose(n, p), std::logic_error);
  }
}

// Confirms the Box::MakeCube correctness.
GTEST_TEST(BoxTest, Cube) {
  Box cube = Box::MakeCube(1.0);
  EXPECT_EQ(cube.width(), 1.0);
  EXPECT_EQ(cube.depth(), 1.0);
  EXPECT_EQ(cube.height(), 1.0);
  EXPECT_TRUE(CompareMatrices(cube.size(), Eigen::Vector3d::Constant(1.0)));
}

// Simple test that exercises all constructors and confirms the construction
// parameters are reflected in the getters.
GTEST_TEST(ShapeTest, Constructors) {
  const std::string kFilename = "/fictitious_name.obj";

  const Box box{1, 2, 3};
  EXPECT_EQ(box.width(), 1);
  EXPECT_EQ(box.depth(), 2);
  EXPECT_EQ(box.height(), 3);
  const Box box2(Vector3<double>{2, 3, 4});
  EXPECT_EQ(box2.width(), 2);
  EXPECT_EQ(box2.depth(), 3);
  EXPECT_EQ(box2.height(), 4);

  const Capsule capsule{2, 1};
  EXPECT_EQ(capsule.radius(), 2);
  EXPECT_EQ(capsule.length(), 1);
  const Capsule capsule2(Vector2<double>{4, 5});
  EXPECT_EQ(capsule2.radius(), 4);
  EXPECT_EQ(capsule2.length(), 5);

  const Convex convex{kFilename, 1.5};
  EXPECT_EQ(convex.filename(), kFilename);
  EXPECT_EQ(convex.scale(), 1.5);

  const Cylinder cylinder{1, 2};
  EXPECT_EQ(cylinder.radius(), 1);
  EXPECT_EQ(cylinder.length(), 2);
  const Cylinder cylinder2(Vector2<double>{2, 3});
  EXPECT_EQ(cylinder2.radius(), 2);
  EXPECT_EQ(cylinder2.length(), 3);

  const Ellipsoid ellipsoid{1, 2, 3};
  EXPECT_EQ(ellipsoid.a(), 1);
  EXPECT_EQ(ellipsoid.b(), 2);
  EXPECT_EQ(ellipsoid.c(), 3);
  const Ellipsoid ellipsoid2(Vector3<double>{2, 3, 4});
  EXPECT_EQ(ellipsoid2.a(), 2);
  EXPECT_EQ(ellipsoid2.b(), 3);
  EXPECT_EQ(ellipsoid2.c(), 4);

  // Note: there are no accessors on the half space; simply confirm it
  // constructs without throwing.
  const HalfSpace hs{};
  unused(hs);

  const Mesh mesh{kFilename, 1.4};
  EXPECT_EQ(mesh.filename(), kFilename);
  EXPECT_EQ(mesh.scale(), 1.4);

  const MeshcatCone cone{1.2, 3.4, 5.6};
  EXPECT_EQ(cone.height(), 1.2);
  EXPECT_EQ(cone.a(), 3.4);
  EXPECT_EQ(cone.b(), 5.6);
  const MeshcatCone cone2(Vector3<double>{6, 7, 8});
  EXPECT_EQ(cone2.height(), 6);
  EXPECT_EQ(cone2.a(), 7);
  EXPECT_EQ(cone2.b(), 8);

  const Sphere s(1.0);
  EXPECT_EQ(s.radius(), 1.0);
}

// Confirms that shape parameters are validated. For the vector-based
// constructors, we only provide a single invocation, relying on the idea that
// it forwards construction to the validating constructor with individual
// parameters.
GTEST_TEST(ShapeTest, NumericalValidation) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      Box(2, 0, 2), "Box width, depth, and height should all be > 0.+");
  DRAKE_EXPECT_THROWS_MESSAGE(
      Box(3, 1, -1), "Box width, depth, and height should all be > 0.+");
  DRAKE_EXPECT_THROWS_MESSAGE(
      Box(Vector3<double>{3, 1, -1}),
      "Box width, depth, and height should all be > 0.+");
  DRAKE_EXPECT_THROWS_MESSAGE(Box::MakeCube(0),
                              "Box width, depth, and height should "
                              "all be > 0.+");

  DRAKE_EXPECT_THROWS_MESSAGE(Capsule(0, 1),
                              "Capsule radius and length should both be > 0.+");
  DRAKE_EXPECT_THROWS_MESSAGE(Capsule(0.5, -1),
                              "Capsule radius and length should both be > 0.+");
  DRAKE_EXPECT_THROWS_MESSAGE(Capsule(Vector2<double>{0.5, -1}),
                              "Capsule radius and length should both be > 0.+");

  DRAKE_EXPECT_THROWS_MESSAGE(Convex("bar", 0),
                              "Convex .scale. cannot be < 1e-8.");
  DRAKE_EXPECT_NO_THROW(Convex("foo", -1));  // Special case for negative scale.

  DRAKE_EXPECT_THROWS_MESSAGE(
      Cylinder(0, 1), "Cylinder radius and length should both be > 0.+");
  DRAKE_EXPECT_THROWS_MESSAGE(
      Cylinder(0.5, -1), "Cylinder radius and length should both be > 0.+");
  DRAKE_EXPECT_THROWS_MESSAGE(
      Cylinder(Vector2<double>{0.5, -1}),
      "Cylinder radius and length should both be > 0.+");

  DRAKE_EXPECT_THROWS_MESSAGE(Ellipsoid(0, 1, 1),
                              "Ellipsoid lengths of principal semi-axes a, b, "
                              "and c should all be > 0.+");
  DRAKE_EXPECT_THROWS_MESSAGE(Ellipsoid(1, 0, 1),
                              "Ellipsoid lengths of principal semi-axes a, b, "
                              "and c should all be > 0.+");
  DRAKE_EXPECT_THROWS_MESSAGE(Ellipsoid(1, 1, 0),
                              "Ellipsoid lengths of principal semi-axes a, b, "
                              "and c should all be > 0.+");
  DRAKE_EXPECT_THROWS_MESSAGE(Ellipsoid(Vector3<double>{1, 1, 0}),
                              "Ellipsoid lengths of principal semi-axes a, b, "
                              "and c should all be > 0.+");

  DRAKE_EXPECT_THROWS_MESSAGE(Mesh("foo", 1e-9),
                              "Mesh .scale. cannot be < 1e-8.");
  DRAKE_EXPECT_NO_THROW(Mesh("foo", -1));  // Special case for negative scale.

  DRAKE_EXPECT_THROWS_MESSAGE(MeshcatCone(0, 1, 1),
                              "MeshcatCone parameters .+ should all be > 0.*");
  DRAKE_EXPECT_THROWS_MESSAGE(MeshcatCone(1, 0, 1),
                              "MeshcatCone parameters .+ should all be > 0.*");
  DRAKE_EXPECT_THROWS_MESSAGE(MeshcatCone(1, 1, 0),
                              "MeshcatCone parameters .+ should all be > 0.*");
  DRAKE_EXPECT_THROWS_MESSAGE(MeshcatCone(Vector3<double>{1, 1, 0}),
                              "MeshcatCone parameters .+ should all be > 0.*");

  DRAKE_EXPECT_THROWS_MESSAGE(Sphere(-0.5),
                              "Sphere radius should be >= 0.+");
  DRAKE_EXPECT_NO_THROW(Sphere(0));  // Special case for 0 radius.
}

class DefaultReifierTest : public ShapeReifier, public ::testing::Test {};

// Tests default implementation of virtual functions for each shape.
TEST_F(DefaultReifierTest, UnsupportedGeometry) {
  DRAKE_EXPECT_THROWS_MESSAGE(this->ImplementGeometry(Box(1, 1, 1), nullptr),
                              "This class (.+) does not support Box.");
  DRAKE_EXPECT_THROWS_MESSAGE(this->ImplementGeometry(Capsule(1, 2), nullptr),
                              "This class (.+) does not support Capsule.");
  DRAKE_EXPECT_THROWS_MESSAGE(this->ImplementGeometry(Convex("a", 1), nullptr),
                              "This class (.+) does not support Convex.");
  DRAKE_EXPECT_THROWS_MESSAGE(this->ImplementGeometry(Cylinder(1, 2), nullptr),
                              "This class (.+) does not support Cylinder.");
  DRAKE_EXPECT_THROWS_MESSAGE(this->ImplementGeometry(Ellipsoid(1, 1, 1),
                              nullptr),
                              "This class (.+) does not support Ellipsoid.");
  DRAKE_EXPECT_THROWS_MESSAGE(this->ImplementGeometry(HalfSpace(), nullptr),
                              "This class (.+) does not support HalfSpace.");
  DRAKE_EXPECT_THROWS_MESSAGE(this->ImplementGeometry(Mesh("foo", 1), nullptr),
                              "This class (.+) does not support Mesh.");
  DRAKE_EXPECT_THROWS_MESSAGE(this->ImplementGeometry(Sphere(0.5), nullptr),
                              "This class (.+) does not support Sphere.");
}

GTEST_TEST(ShapeName, SimpleReification) {
  ShapeName name;

  EXPECT_EQ(name.name(), "");

  Box(1, 2, 3).Reify(&name);
  EXPECT_EQ(name.name(), "Box");

  Capsule(1, 2).Reify(&name);
  EXPECT_EQ(name.name(), "Capsule");

  Convex("filepath", 1.0).Reify(&name);
  EXPECT_EQ(name.name(), "Convex");

  Cylinder(1, 2).Reify(&name);
  EXPECT_EQ(name.name(), "Cylinder");

  Ellipsoid(1, 2, 3).Reify(&name);
  EXPECT_EQ(name.name(), "Ellipsoid");

  HalfSpace().Reify(&name);
  EXPECT_EQ(name.name(), "HalfSpace");

  Mesh("filepath", 1.0).Reify(&name);
  EXPECT_EQ(name.name(), "Mesh");

  MeshcatCone(1.0, 0.25, 0.5).Reify(&name);
  EXPECT_EQ(name.name(), "MeshcatCone");

  Sphere(0.5).Reify(&name);
  EXPECT_EQ(name.name(), "Sphere");
}

GTEST_TEST(ShapeName, ReifyOnConstruction) {
  EXPECT_EQ(ShapeName(Box(1, 2, 3)).name(), "Box");
  EXPECT_EQ(ShapeName(Capsule(1, 2)).name(), "Capsule");
  EXPECT_EQ(ShapeName(Convex("filepath", 1.0)).name(), "Convex");
  EXPECT_EQ(ShapeName(Cylinder(1, 2)).name(), "Cylinder");
  EXPECT_EQ(ShapeName(Ellipsoid(1, 2, 3)).name(), "Ellipsoid");
  EXPECT_EQ(ShapeName(HalfSpace()).name(), "HalfSpace");
  EXPECT_EQ(ShapeName(Mesh("filepath", 1.0)).name(), "Mesh");
  EXPECT_EQ(ShapeName(Sphere(0.5)).name(), "Sphere");
}

GTEST_TEST(ShapeName, Streaming) {
  ShapeName name(Sphere(0.5));
  std::stringstream ss;
  ss << name;
  EXPECT_EQ(name.name(), "Sphere");
  EXPECT_EQ(ss.str(), name.name());
}

GTEST_TEST(ShapeTest, Volume) {
  EXPECT_NEAR(CalcVolume(Box(1, 2, 3)), 6.0, 1e-14);
  EXPECT_NEAR(CalcVolume(Capsule(1.23, 2.4)),
              CalcVolume(Cylinder(1.23, 2.4)) + CalcVolume(Sphere(1.23)),
              1e-14);
  EXPECT_NEAR(CalcVolume(Cylinder(1.3, 2.1)), M_PI * 1.3 * 1.3 * 2.1, 1e-14);
  EXPECT_NEAR(CalcVolume(Ellipsoid(1, 2, 3)), 8.0 * M_PI, 1e-14);
  EXPECT_EQ(CalcVolume(HalfSpace()), std::numeric_limits<double>::infinity());
  EXPECT_NEAR(CalcVolume(MeshcatCone(4, 2, 3)), 8.0 * M_PI, 1e-14);
  EXPECT_NEAR(CalcVolume(Sphere(3)), 36.0 * M_PI, 1e-13);

  const std::string cube_obj =
      FindResourceOrThrow("drake/geometry/test/quad_cube.obj");
  EXPECT_NEAR(CalcVolume(Convex(cube_obj, 1.0)), 8.0, 1e-14);
  EXPECT_NEAR(CalcVolume(Mesh(cube_obj, 1.0)), 8.0, 1e-14);

  DRAKE_EXPECT_THROWS_MESSAGE(CalcVolume(Convex("fakename.obj", 1.0)),
                              "Cannot open file.*");
  DRAKE_EXPECT_THROWS_MESSAGE(CalcVolume(Mesh("fakename.obj", 1.0)),
                              "Cannot open file.*");

  // We only support obj but should eventually support vtk.
  const std::string non_obj = "only_extension_matters.not_obj";
  DRAKE_EXPECT_THROWS_MESSAGE(CalcVolume(Convex(non_obj, 1.0)),
                              ".*only supports .obj files.*");
  DRAKE_EXPECT_THROWS_MESSAGE(CalcVolume(Mesh(non_obj, 1.0)),
                              ".*only supports .obj files.*");
}

GTEST_TEST(ShapeTest, Pathname) {
  const Mesh abspath_mesh("/absolute_path.obj");
  EXPECT_TRUE(std::filesystem::path(abspath_mesh.filename()).is_absolute());
  EXPECT_EQ(abspath_mesh.filename(), "/absolute_path.obj");

  const Convex abspath_convex("/absolute_path.obj");
  EXPECT_TRUE(std::filesystem::path(abspath_convex.filename()).is_absolute());
  EXPECT_EQ(abspath_convex.filename(), "/absolute_path.obj");

  const Mesh relpath_mesh("relative_path.obj");
  EXPECT_TRUE(std::filesystem::path(relpath_mesh.filename()).is_absolute());
  EXPECT_EQ(relpath_mesh.filename(),
            std::filesystem::current_path() / "relative_path.obj");

  const Convex relpath_convex("relative_path.obj");
  EXPECT_TRUE(std::filesystem::path(relpath_convex.filename()).is_absolute());
  EXPECT_EQ(relpath_convex.filename(),
            std::filesystem::current_path() / "relative_path.obj");
}

}  // namespace
}  // namespace geometry
}  // namespace drake
