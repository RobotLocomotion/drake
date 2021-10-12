#include "drake/geometry/shape_specification.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/test_utilities/is_dynamic_castable.h"

namespace drake {
namespace geometry {
namespace {

using math::RigidTransformd;
using std::unique_ptr;

// Confirm correct interactions with the Reifier.

class ReifierTest : public ShapeReifier, public ::testing::Test {
 public:
  void ImplementGeometry(const Sphere&, void* data) override {
    received_user_data_ = data;
    sphere_made_ = true;
  }
  void ImplementGeometry(const Cylinder&, void* data) override {
    received_user_data_ = data;
    cylinder_made_ = true;
  }
  void ImplementGeometry(const HalfSpace&, void* data) override {
    received_user_data_ = data;
    half_space_made_ = true;
  }
  void ImplementGeometry(const Box&, void* data) override {
    received_user_data_ = data;
    box_made_ = true;
  }
  void ImplementGeometry(const Capsule&, void* data) override {
    received_user_data_ = data;
    capsule_made_ = true;
  }
  void ImplementGeometry(const Ellipsoid&, void* data) override {
    received_user_data_ = data;
    ellipsoid_made_ = true;
  }
  void ImplementGeometry(const Mesh&, void* data) override {
    received_user_data_ = data;
    mesh_made_ = true;
  }
  void ImplementGeometry(const Convex&, void* data) override {
    received_user_data_ = data;
    convex_made_ = true;
  }
  void ImplementGeometry(const MeshcatCone&, void* data) override {
    received_user_data_ = data;
    meshcat_cone_made_ = true;
  }
  void Reset() {
    box_made_ = false;
    capsule_made_ = false;
    ellipsoid_made_ = false;
    sphere_made_ = false;
    half_space_made_ = false;
    cylinder_made_ = false;
    convex_made_ = false;
    mesh_made_ = false;
    meshcat_cone_made_ = false;
    received_user_data_ = nullptr;
  }

 protected:
  bool box_made_{false};
  bool capsule_made_{false};
  bool ellipsoid_made_{false};
  bool sphere_made_{false};
  bool cylinder_made_{false};
  bool half_space_made_{false};
  bool convex_made_{false};
  bool mesh_made_{false};
  bool meshcat_cone_made_{false};
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
  Sphere s(1.0);
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
  EXPECT_EQ(s.radius(), 1.0);

  Reset();

  HalfSpace hs{};
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

  Cylinder cylinder{1, 2};
  cylinder.Reify(this);
  EXPECT_FALSE(sphere_made_);
  EXPECT_FALSE(half_space_made_);
  EXPECT_TRUE(cylinder_made_);
  EXPECT_FALSE(box_made_);
  EXPECT_FALSE(capsule_made_);
  EXPECT_FALSE(convex_made_);
  EXPECT_FALSE(ellipsoid_made_);
  EXPECT_FALSE(mesh_made_);
  EXPECT_EQ(cylinder.radius(), 1);
  EXPECT_EQ(cylinder.length(), 2);
  EXPECT_FALSE(meshcat_cone_made_);

  Reset();

  Box box{1, 2, 3};
  box.Reify(this);
  EXPECT_FALSE(sphere_made_);
  EXPECT_FALSE(half_space_made_);
  EXPECT_FALSE(cylinder_made_);
  EXPECT_TRUE(box_made_);
  EXPECT_FALSE(capsule_made_);
  EXPECT_FALSE(convex_made_);
  EXPECT_FALSE(ellipsoid_made_);
  EXPECT_FALSE(mesh_made_);
  EXPECT_EQ(box.width(), 1);
  EXPECT_EQ(box.depth(), 2);
  EXPECT_EQ(box.height(), 3);
  EXPECT_FALSE(meshcat_cone_made_);

  Reset();

  Capsule capsule{2, 1};
  capsule.Reify(this);
  EXPECT_FALSE(sphere_made_);
  EXPECT_FALSE(half_space_made_);
  EXPECT_FALSE(cylinder_made_);
  EXPECT_FALSE(box_made_);
  EXPECT_TRUE(capsule_made_);
  EXPECT_FALSE(convex_made_);
  EXPECT_FALSE(ellipsoid_made_);
  EXPECT_FALSE(mesh_made_);
  EXPECT_EQ(capsule.radius(), 2);
  EXPECT_EQ(capsule.length(), 1);
  EXPECT_FALSE(meshcat_cone_made_);

  Reset();

  Convex convex{"fictitious_name.obj", 1.0};
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

  Ellipsoid ellipsoid{1, 2, 3};
  ellipsoid.Reify(this);
  EXPECT_FALSE(sphere_made_);
  EXPECT_FALSE(half_space_made_);
  EXPECT_FALSE(cylinder_made_);
  EXPECT_FALSE(box_made_);
  EXPECT_FALSE(capsule_made_);
  EXPECT_FALSE(convex_made_);
  EXPECT_TRUE(ellipsoid_made_);
  EXPECT_FALSE(mesh_made_);
  EXPECT_EQ(ellipsoid.a(), 1);
  EXPECT_EQ(ellipsoid.b(), 2);
  EXPECT_EQ(ellipsoid.c(), 3);
  EXPECT_FALSE(meshcat_cone_made_);

  Reset();

  Mesh mesh{"fictitious_mesh_name.obj", 1.4};
  mesh.Reify(this);
  EXPECT_FALSE(sphere_made_);
  EXPECT_FALSE(half_space_made_);
  EXPECT_FALSE(cylinder_made_);
  EXPECT_FALSE(box_made_);
  EXPECT_FALSE(capsule_made_);
  EXPECT_FALSE(convex_made_);
  EXPECT_FALSE(ellipsoid_made_);
  EXPECT_TRUE(mesh_made_);
  EXPECT_EQ(mesh.filename(), std::string("fictitious_mesh_name.obj"));
  EXPECT_EQ(mesh.scale(), 1.4);
  EXPECT_FALSE(meshcat_cone_made_);

  Reset();

  MeshcatCone cone{1.2, 3.4, 5.6};
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
  EXPECT_EQ(cone.height(), 1.2);
  EXPECT_EQ(cone.a(), 3.4);
  EXPECT_EQ(cone.b(), 5.6);
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
  ASSERT_TRUE(sphere_made_);
  ASSERT_FALSE(half_space_made_);
  ASSERT_FALSE(cylinder_made_);
  ASSERT_FALSE(box_made_);
  ASSERT_FALSE(capsule_made_);
  EXPECT_FALSE(convex_made_);
  ASSERT_FALSE(ellipsoid_made_);
  EXPECT_FALSE(mesh_made_);
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
    return ::testing::AssertionFailure()
           << "pose =\n"
           << pose.GetAsMatrix34() << "\nExpected z-axis "
           << expected_z.transpose() << " does not match pose's z-axis "
           << z_axis.transpose();
  }

  // Test expected translation.
  if (!CompareMatrices(pose.translation(), expected_translation, tolerance,
                       MatrixCompareType::absolute)) {
    return ::testing::AssertionFailure()
           << "pose =\n"
           << pose.GetAsMatrix34() << "\nExpected translation "
           << expected_translation.transpose()
           << " does not match pose's translation "
           << pose.translation().transpose();
  }

  // Test unit-length rotation.
  char axis_labels[] = {'x', 'y', 'z'};
  for (int i = 0; i < 3; ++i) {
    if (abs(pose.rotation().col(i).norm() - 1) > tolerance) {
      return ::testing::AssertionFailure()
             << "pose =\n"
             << pose.GetAsMatrix34() << "\ndoes not have unit length "
             << axis_labels[i] << "-axis "
             << pose.rotation().col(i).transpose();
    }
  }

  // Test orthogonality.
  for (int i = 0; i < 2; ++i) {
    for (int j = i + 1; j < 3; ++j) {
      double dot_product = pose.rotation().col(i).dot(pose.rotation().col(j));
      if (abs(dot_product) > tolerance) {
        return ::testing::AssertionFailure()
               << "For pose =\n"
               << pose.GetAsMatrix34() << "\nThe " << axis_labels[i]
               << "-axis and " << axis_labels[j] << "-axis are not orthogonal";
      }
    }
  }
  return ::testing::AssertionSuccess()
         << "pose =\n"
         << pose.GetAsMatrix34()
         << "\nhas expected z-axis = " << expected_z.transpose()
         << "\nand expected translation = " << expected_translation.transpose();
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

// Confirms that shape parameters are validated.
GTEST_TEST(ShapeTest, NumericalValidation) {
  DRAKE_EXPECT_THROWS_MESSAGE(Sphere(-0.5), std::logic_error,
                              "Sphere radius should be >= 0.+");
  DRAKE_EXPECT_NO_THROW(Sphere(0));  // Special case for 0 radius.

  DRAKE_EXPECT_THROWS_MESSAGE(
      Cylinder(0, 1), std::logic_error, "Cylinder radius and length should "
      "both be > 0.+");
  DRAKE_EXPECT_THROWS_MESSAGE(
      Cylinder(0.5, -1), std::logic_error, "Cylinder radius and length should "
      "both be > 0.+");

  DRAKE_EXPECT_THROWS_MESSAGE(
      Box(2, 0, 2), std::logic_error, "Box width, depth, and height should "
      "all be > 0.+");
  DRAKE_EXPECT_THROWS_MESSAGE(
      Box(3, 1, -1), std::logic_error, "Box width, depth, and height should "
      "all be > 0.+");
  DRAKE_EXPECT_THROWS_MESSAGE(Box::MakeCube(0), std::logic_error,
                              "Box width, depth, and height should "
                              "all be > 0.+");

  DRAKE_EXPECT_THROWS_MESSAGE(Capsule(0, 1), std::logic_error,
                              "Capsule radius and length should both be > 0.+");
  DRAKE_EXPECT_THROWS_MESSAGE(Capsule(0.5, -1), std::logic_error,
                              "Capsule radius and length should both be > 0.+");

  DRAKE_EXPECT_THROWS_MESSAGE(Ellipsoid(0, 1, 1), std::logic_error,
                              "Ellipsoid lengths of principal semi-axes a, b, "
                              "and c should all be > 0.+");
  DRAKE_EXPECT_THROWS_MESSAGE(Ellipsoid(1, 0, 1), std::logic_error,
                              "Ellipsoid lengths of principal semi-axes a, b, "
                              "and c should all be > 0.+");
  DRAKE_EXPECT_THROWS_MESSAGE(Ellipsoid(1, 1, 0), std::logic_error,
                              "Ellipsoid lengths of principal semi-axes a, b, "
                              "and c should all be > 0.+");

  DRAKE_EXPECT_THROWS_MESSAGE(Mesh("foo", 1e-9), std::logic_error,
                              "Mesh .scale. cannot be < 1e-8.");
  DRAKE_EXPECT_NO_THROW(Mesh("foo", -1));  // Special case for negative scale.

  DRAKE_EXPECT_THROWS_MESSAGE(Convex("bar", 0), std::logic_error,
                              "Convex .scale. cannot be < 1e-8.");
  DRAKE_EXPECT_NO_THROW(Convex("foo", -1));  // Special case for negative scale.
}

class DefaultReifierTest : public ShapeReifier, public ::testing::Test {};

// Tests default implementation of virtual functions for each shape.
TEST_F(DefaultReifierTest, UnsupportedGeometry) {
  DRAKE_EXPECT_THROWS_MESSAGE(this->ImplementGeometry(Sphere(0.5), nullptr),
                              std::runtime_error,
                              "This class (.+) does not support Sphere.");
  DRAKE_EXPECT_THROWS_MESSAGE(this->ImplementGeometry(Cylinder(1, 2), nullptr),
                              std::runtime_error,
                              "This class (.+) does not support Cylinder.");
  DRAKE_EXPECT_THROWS_MESSAGE(this->ImplementGeometry(HalfSpace(), nullptr),
                              std::runtime_error,
                              "This class (.+) does not support HalfSpace.");
  DRAKE_EXPECT_THROWS_MESSAGE(this->ImplementGeometry(Box(1, 1, 1), nullptr),
                              std::runtime_error,
                              "This class (.+) does not support Box.");
  DRAKE_EXPECT_THROWS_MESSAGE(this->ImplementGeometry(Capsule(1, 2), nullptr),
                              std::runtime_error,
                              "This class (.+) does not support Capsule.");
  DRAKE_EXPECT_THROWS_MESSAGE(this->ImplementGeometry(Ellipsoid(1, 1, 1),
                              nullptr), std::runtime_error,
                              "This class (.+) does not support Ellipsoid.");
  DRAKE_EXPECT_THROWS_MESSAGE(this->ImplementGeometry(Mesh("foo", 1), nullptr),
                              std::runtime_error,
                              "This class (.+) does not support Mesh.");
  DRAKE_EXPECT_THROWS_MESSAGE(this->ImplementGeometry(Convex("a", 1), nullptr),
                              std::runtime_error,
                              "This class (.+) does not support Convex.");
}

GTEST_TEST(ShapeName, SimpleReification) {
  ShapeName name;

  EXPECT_EQ(name.name(), "");

  Sphere(0.5).Reify(&name);
  EXPECT_EQ(name.name(), "Sphere");

  Cylinder(1, 2).Reify(&name);
  EXPECT_EQ(name.name(), "Cylinder");

  Box(1, 2, 3).Reify(&name);
  EXPECT_EQ(name.name(), "Box");

  Capsule(1, 2).Reify(&name);
  EXPECT_EQ(name.name(), "Capsule");

  Ellipsoid(1, 2, 3).Reify(&name);
  EXPECT_EQ(name.name(), "Ellipsoid");

  HalfSpace().Reify(&name);
  EXPECT_EQ(name.name(), "HalfSpace");

  Mesh("filepath", 1.0).Reify(&name);
  EXPECT_EQ(name.name(), "Mesh");

  Convex("filepath", 1.0).Reify(&name);
  EXPECT_EQ(name.name(), "Convex");
}

GTEST_TEST(ShapeName, ReifyOnConstruction) {
  EXPECT_EQ(ShapeName(Sphere(0.5)).name(), "Sphere");
  EXPECT_EQ(ShapeName(Cylinder(1, 2)).name(), "Cylinder");
  EXPECT_EQ(ShapeName(Capsule(1, 2)).name(), "Capsule");
  EXPECT_EQ(ShapeName(Ellipsoid(1, 2, 3)).name(), "Ellipsoid");
  EXPECT_EQ(ShapeName(Box(1, 2, 3)).name(), "Box");
  EXPECT_EQ(ShapeName(HalfSpace()).name(), "HalfSpace");
  EXPECT_EQ(ShapeName(Mesh("filepath", 1.0)).name(), "Mesh");
  EXPECT_EQ(ShapeName(Convex("filepath", 1.0)).name(), "Convex");
}

GTEST_TEST(ShapeName, Streaming) {
  ShapeName name(Sphere(0.5));
  std::stringstream ss;
  ss << name;
  EXPECT_EQ(name.name(), "Sphere");
  EXPECT_EQ(ss.str(), name.name());
}

}  // namespace
}  // namespace geometry
}  // namespace drake
