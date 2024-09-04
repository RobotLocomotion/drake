#include "drake/geometry/shape_specification.h"

#include <filesystem>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/fmt_eigen.h"
#include "drake/common/overloaded.h"
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

GTEST_TEST(VisitTest, ReturnTypeVoid) {
  const Box box(1.0, 2.0, 3.0);
  box.Visit(overloaded{
      [&](const Box& arg) {
        EXPECT_EQ(&arg, &box);
      },
      [](const auto&) {
        GTEST_FAIL();
      },
  });

  const Sphere sphere(1.0);
  sphere.Visit(overloaded{
      [&](const Sphere& arg) {
        EXPECT_EQ(&arg, &sphere);
      },
      [](const auto&) {
        GTEST_FAIL();
      },
  });
}

GTEST_TEST(VisitTest, ReturnTypeConversion) {
  const Box box(1.0, 2.0, 3.0);
  const Sphere sphere(1.0);

  auto get_size = overloaded{
      [](const Box& arg) {
        return arg.size();
      },
      [](const Sphere& arg) {
        return Vector1d(arg.radius());
      },
      [](const auto&) -> Eigen::VectorXd {
        DRAKE_UNREACHABLE();
      },
  };

  Eigen::VectorXd dims;
  dims = box.Visit<Eigen::VectorXd>(get_size);
  dims = sphere.Visit<Eigen::VectorXd>(get_size);
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
  EXPECT_EQ(convex.source().description(), kFilename);
  EXPECT_EQ(convex.extension(), ".obj");
  EXPECT_EQ(convex.scale(), 1.5);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  EXPECT_EQ(convex.filename(), kFilename);
#pragma GCC diagnostic pop

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
  EXPECT_EQ(mesh.source().description(), kFilename);
  EXPECT_EQ(mesh.extension(), ".obj");
  EXPECT_EQ(mesh.scale(), 1.4);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  EXPECT_EQ(mesh.filename(), kFilename);
#pragma GCC diagnostic pop

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
                              "Convex .scale. cannot be < 1e-8.*");
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
                              "Mesh .scale. cannot be < 1e-8.*");
  DRAKE_EXPECT_NO_THROW(Mesh("foo", -1));  // Special case for negative scale.

  DRAKE_EXPECT_THROWS_MESSAGE(MeshcatCone(0, 1, 1),
                              "MeshcatCone parameters .+ should all be > 0.*");
  DRAKE_EXPECT_THROWS_MESSAGE(MeshcatCone(1, 0, 1),
                              "MeshcatCone parameters .+ should all be > 0.*");
  DRAKE_EXPECT_THROWS_MESSAGE(MeshcatCone(1, 1, 0),
                              "MeshcatCone parameters .+ should all be > 0.*");
  DRAKE_EXPECT_THROWS_MESSAGE(MeshcatCone(Vector3<double>{1, 1, 0}),
                              "MeshcatCone parameters .+ should all be > 0.*");

  DRAKE_EXPECT_THROWS_MESSAGE(Sphere(-0.5), "Sphere radius should be >= 0.+");
  DRAKE_EXPECT_NO_THROW(Sphere(0));  // Special case for 0 radius.
}

// Confirms that Convex and Mesh can report their convex hull.
GTEST_TEST(ShapeTest, ConvexHull) {
  const std::string cube_path =
      FindResourceOrThrow("drake/geometry/test/quad_cube.obj");

  auto expect_convex_hull = [](const auto& mesh_like) {
    SCOPED_TRACE(
        fmt::format("Testing convex hull for {}.", mesh_like.type_name()));
    using MeshType = decltype(mesh_like);
    // First call should work for a valid mesh file name.
    const PolygonSurfaceMesh<double>& hull = mesh_like.GetConvexHull();
    // Subsequent calls return references to the same value.
    EXPECT_EQ(&hull, &mesh_like.GetConvexHull());
    const MeshType mesh2(mesh_like);
    // Copies of the mesh share the same hull.
    EXPECT_EQ(&mesh2.GetConvexHull(), &hull);
  };
  expect_convex_hull(Mesh(cube_path));
  expect_convex_hull(Convex(cube_path));
}

GTEST_TEST(ShapeTest, ConvexFromMemory) {
  // This will get normalized to ".obj".
  const std::string mesh_name = "a_mesh.OBJ";
  const std::string obj_contents = R"""(
    v 0 0 0
    v 1 0 0
    v 1 1 0
    v 0 1 0
    v 0 0 1
    v 1 0 1
    v 1 1 1
    v 0 1 1
    # intentionally omit faces.
  )""";
  InMemoryMesh mesh_data(
      MemoryFile(obj_contents, ".OBJ", mesh_name),
      {{"fake.txt", MemoryFile("content", ".txt", "fake.txt")}});
  const Convex convex(std::move(mesh_data), 2.0);
  EXPECT_EQ(convex.extension(), ".obj");
  const MeshSource& source = convex.source();
  ASSERT_TRUE(source.is_in_memory());
  EXPECT_EQ(source.in_memory().mesh_file().filename_hint(), mesh_name);
  EXPECT_NE(source.in_memory().supporting_file("fake.txt"), nullptr);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  EXPECT_THROW(convex.filename(), std::exception);
#pragma GCC diagnostic pop

  // Also confirm that we can compute the convex hull from the in-memory
  // representation. We don't test all file formats; we trust that visual
  // inspection of the code under test shows that it doesn't depend on file
  // format.
  const PolygonSurfaceMesh<double>& hull = convex.GetConvexHull();
  EXPECT_EQ(hull.num_vertices(), 8);
  EXPECT_EQ(hull.num_elements(), 6);
}

GTEST_TEST(ShapeTest, MeshFromMemory) {
  // This will get normalized to ".obj".
  const std::string mesh_name = "a_mesh.OBJ";
  const std::string obj_contents = R"""(
    v 0 0 0
    v 1 0 0
    v 1 1 0
    v 0 1 0
    v 0 0 1
    v 1 0 1
    v 1 1 1
    v 0 1 1
    f 1 2 3 4
    f 5 6 7 8
  )""";
  InMemoryMesh mesh_data(
      MemoryFile(obj_contents, ".OBJ", mesh_name),
      {{"fake.txt", MemoryFile("content", ".txt", "fake.txt")}});
  const Mesh mesh(std::move(mesh_data), 2.0);
  EXPECT_EQ(mesh.extension(), ".obj");
  const MeshSource& source = mesh.source();
  ASSERT_TRUE(source.is_in_memory());
  EXPECT_EQ(source.in_memory().mesh_file().filename_hint(), mesh_name);
  EXPECT_NE(source.in_memory().supporting_file("fake.txt"), nullptr);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  EXPECT_THROW(mesh.filename(), std::exception);
#pragma GCC diagnostic pop

  // Also confirm that we can compute the convex hull from the in-memory
  // representation. We don't test all file formats; we trust that visual
  // inspection of the code under test shows that it doesn't depend on file
  // format.
  const PolygonSurfaceMesh<double>& hull = mesh.GetConvexHull();
  EXPECT_EQ(hull.num_vertices(), 8);
  EXPECT_EQ(hull.num_elements(), 6);
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
  DRAKE_EXPECT_THROWS_MESSAGE(
      this->ImplementGeometry(Ellipsoid(1, 1, 1), nullptr),
      "This class (.+) does not support Ellipsoid.");
  DRAKE_EXPECT_THROWS_MESSAGE(this->ImplementGeometry(HalfSpace(), nullptr),
                              "This class (.+) does not support HalfSpace.");
  DRAKE_EXPECT_THROWS_MESSAGE(this->ImplementGeometry(Mesh("foo", 1), nullptr),
                              "This class (.+) does not support Mesh.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      this->ImplementGeometry(MeshcatCone(1, 1, 1), nullptr),
      "This class (.+) does not support MeshcatCone.");
  DRAKE_EXPECT_THROWS_MESSAGE(this->ImplementGeometry(Sphere(0.5), nullptr),
                              "This class (.+) does not support Sphere.");
}

// Test confirms that the default unsupported functionality can be replaced.
// We'll simply replace it with a no-op. Contrast this with DefaultReifierTest.
class OverrideDefaultGeometryTest : public ShapeReifier,
                                    public ::testing::Test {
 public:
  using ShapeReifier::ThrowUnsupportedGeometry;
  void DefaultImplementGeometry(const Shape&) final {}
};

// Tests the ability to override default implementation.
TEST_F(OverrideDefaultGeometryTest, UnsupportedGeometry) {
  // Confirm that throwing method is still throwing. If the subsequent calls to
  // ImplementGeometry() fail to throw, it is because they do not invoke the
  // throwing method.
  DRAKE_EXPECT_THROWS_MESSAGE(this->ThrowUnsupportedGeometry("Foo"),
                              "This class (.+) does not support Foo.");

  // Confirm the default behavior no longer throws.
  EXPECT_NO_THROW(this->ImplementGeometry(Box(1, 1, 1), nullptr));
  EXPECT_NO_THROW(this->ImplementGeometry(Capsule(1, 2), nullptr));
  EXPECT_NO_THROW(this->ImplementGeometry(Convex("a", 1), nullptr));
  EXPECT_NO_THROW(this->ImplementGeometry(Cylinder(1, 2), nullptr));
  EXPECT_NO_THROW(this->ImplementGeometry(Ellipsoid(1, 1, 1), nullptr));
  EXPECT_NO_THROW(this->ImplementGeometry(HalfSpace(), nullptr));
  EXPECT_NO_THROW(this->ImplementGeometry(Mesh("foo", 1), nullptr));
  EXPECT_NO_THROW(this->ImplementGeometry(MeshcatCone(1, 1, 1), nullptr));
  EXPECT_NO_THROW(this->ImplementGeometry(Sphere(0.5), nullptr));
}

GTEST_TEST(ShapeTest, TypeNameAndToString) {
  // In-memory meshes we'll use on Convex and Mesh.
  const InMemoryMesh in_memory1(MemoryFile("a", ".a", "A"));
  const InMemoryMesh in_memory2(
      MemoryFile("a", ".a", "A"),
      {{"bb", MemoryFile("b", ".b", "B")}, {"cc", "path/to/c"}});
  // The Mesh and Convex in-memory mesh to_string() results should match except
  // for the class name.
  static constexpr const char* mem1_fmt =
      "{}(mesh_data=InMemoryMesh(mesh_file=MemoryFile(contents='a', "
      "extension='.a', filename_hint='A')), scale=1.5)";
  static constexpr const char* mem2_fmt =
      "{}(mesh_data=InMemoryMesh(mesh_file=MemoryFile(contents='a', "
      "extension='.a', filename_hint='A'), supporting_files={{{{'bb', "
      "FileSource(file=MemoryFile(contents='b', extension='.b', "
      "filename_hint='B'))}}, {{'cc', FileSource(path='path/to/c')}}}}), "
      "scale=1.5)";

  const Box box(1.5, 2.5, 3.5);
  const Capsule capsule(1.25, 2.5);
  const Convex convex("/some/file", 1.5);
  const Convex mem_convex1(in_memory1, 1.5);
  const Convex mem_convex2(in_memory2, 1.5);
  const Cylinder cylinder(1.25, 2.5);
  const Ellipsoid ellipsoid(1.25, 2.5, 0.5);
  const HalfSpace half_space;
  const Mesh mesh("/some/file", 1.5);
  const Mesh mem_mesh1(in_memory1, 1.5);
  const Mesh mem_mesh2(in_memory2, 1.5);
  const MeshcatCone cone(1.5, 0.25, 0.5);
  const Sphere sphere(1.25);

  EXPECT_EQ(box.type_name(), "Box");
  EXPECT_EQ(capsule.type_name(), "Capsule");
  EXPECT_EQ(convex.type_name(), "Convex");
  EXPECT_EQ(cylinder.type_name(), "Cylinder");
  EXPECT_EQ(ellipsoid.type_name(), "Ellipsoid");
  EXPECT_EQ(half_space.type_name(), "HalfSpace");
  EXPECT_EQ(mesh.type_name(), "Mesh");
  EXPECT_EQ(mem_mesh1.type_name(), "Mesh");
  EXPECT_EQ(mem_mesh2.type_name(), "Mesh");
  EXPECT_EQ(cone.type_name(), "MeshcatCone");
  EXPECT_EQ(sphere.type_name(), "Sphere");

  EXPECT_EQ(box.to_string(), "Box(width=1.5, depth=2.5, height=3.5)");
  EXPECT_EQ(capsule.to_string(), "Capsule(radius=1.25, length=2.5)");
  EXPECT_EQ(convex.to_string(), "Convex(filename='/some/file', scale=1.5)");
  EXPECT_EQ(mem_convex1.to_string(), fmt::format(mem1_fmt, "Convex"));
  EXPECT_EQ(mem_convex2.to_string(), fmt::format(mem2_fmt, "Convex"));
  EXPECT_EQ(cylinder.to_string(), "Cylinder(radius=1.25, length=2.5)");
  EXPECT_EQ(ellipsoid.to_string(), "Ellipsoid(a=1.25, b=2.5, c=0.5)");
  EXPECT_EQ(half_space.to_string(), "HalfSpace()");
  EXPECT_EQ(mesh.to_string(), "Mesh(filename='/some/file', scale=1.5)");
  EXPECT_EQ(mem_mesh1.to_string(), fmt::format(mem1_fmt, "Mesh"));
  EXPECT_EQ(mem_mesh2.to_string(), fmt::format(mem2_fmt, "Mesh"));
  EXPECT_EQ(cone.to_string(), "MeshcatCone(height=1.5, a=0.25, b=0.5)");
  EXPECT_EQ(sphere.to_string(), "Sphere(radius=1.25)");

  EXPECT_EQ(fmt::to_string(box), "Box(width=1.5, depth=2.5, height=3.5)");
  EXPECT_EQ(fmt::to_string(capsule), "Capsule(radius=1.25, length=2.5)");
  EXPECT_EQ(fmt::to_string(convex), "Convex(filename='/some/file', scale=1.5)");
  EXPECT_EQ(fmt::to_string(mem_convex1), mem_convex1.to_string());
  EXPECT_EQ(fmt::to_string(mem_convex2), mem_convex2.to_string());
  EXPECT_EQ(fmt::to_string(cylinder), "Cylinder(radius=1.25, length=2.5)");
  EXPECT_EQ(fmt::to_string(ellipsoid), "Ellipsoid(a=1.25, b=2.5, c=0.5)");
  EXPECT_EQ(fmt::to_string(half_space), "HalfSpace()");
  EXPECT_EQ(fmt::to_string(mesh), "Mesh(filename='/some/file', scale=1.5)");
  EXPECT_EQ(fmt::to_string(mem_mesh1), mem_mesh1.to_string());
  EXPECT_EQ(fmt::to_string(mem_mesh2), mem_mesh2.to_string());
  EXPECT_EQ(fmt::to_string(cone), "MeshcatCone(height=1.5, a=0.25, b=0.5)");
  EXPECT_EQ(fmt::to_string(sphere), "Sphere(radius=1.25)");

  const Shape& base = box;
  EXPECT_EQ(base.type_name(), "Box");
  EXPECT_EQ(base.to_string(), "Box(width=1.5, depth=2.5, height=3.5)");
  EXPECT_EQ(fmt::to_string(base), "Box(width=1.5, depth=2.5, height=3.5)");
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

  // The convex hull of the cube with a hole, should have the same volume as
  // a cube without the hole.
  const std::string holey_cube_obj =
      FindResourceOrThrow("drake/geometry/test/cube_with_hole.obj");
  EXPECT_NEAR(CalcVolume(Convex(holey_cube_obj, 1.0)), 8.0, 1e-14);
  const std::string cube_obj =
      FindResourceOrThrow("drake/geometry/test/quad_cube.obj");
  EXPECT_NEAR(CalcVolume(Mesh(cube_obj, 1.0)), 8.0, 1e-14);

  // Error thrown in ReadObjFile() (read_obj.cc). Note: this passes the tinyobj
  // error along -- those terminate in new lines and has to be handled in the
  // matching expression explicitly.
  DRAKE_EXPECT_THROWS_MESSAGE(CalcVolume(Convex("fakename.obj")),
                              ".*cannot read the file[^]*");
  // Error thrown in ReadObjToTriangleSurfaceMesh() (obj_to_surface_mesh.cc).
  DRAKE_EXPECT_THROWS_MESSAGE(CalcVolume(Mesh("fakename.obj")),
                              ".*cannot read the file[^]*");

  const std::string non_obj = "only_extension_matters.not_obj";
  DRAKE_EXPECT_THROWS_MESSAGE(CalcVolume(Convex(non_obj)),
                              ".*only applies to .obj, .vtk.*");
  // We only support obj but should eventually support vtk.
  DRAKE_EXPECT_THROWS_MESSAGE(CalcVolume(Mesh(non_obj)),
                              ".*only supports .obj files.*");
}

GTEST_TEST(ShapeTest, Pathname) {
  const Mesh abspath_mesh("/absolute_path.obj");
  EXPECT_TRUE(abspath_mesh.source().path().is_absolute());
  EXPECT_EQ(abspath_mesh.source().path(), "/absolute_path.obj");

  const Convex abspath_convex("/absolute_path.obj");
  EXPECT_TRUE(abspath_convex.source().path().is_absolute());
  EXPECT_EQ(abspath_convex.source().path(), "/absolute_path.obj");

  const Mesh relpath_mesh("relative_path.obj");
  EXPECT_TRUE(relpath_mesh.source().path().is_absolute());
  EXPECT_EQ(relpath_mesh.source().path(),
            std::filesystem::current_path() / "relative_path.obj");

  const Convex relpath_convex("relative_path.obj");
  EXPECT_TRUE(relpath_convex.source().path().is_absolute());
  EXPECT_EQ(relpath_convex.source().path(),
            std::filesystem::current_path() / "relative_path.obj");
}

GTEST_TEST(ShapeTest, MeshExtensions) {
  // Test for case.
  EXPECT_EQ(Mesh("a/b.obj").extension(), ".obj");
  EXPECT_EQ(Mesh("a/b.oBj").extension(), ".obj");
  EXPECT_EQ(Mesh("a/b.ObJ").extension(), ".obj");
  EXPECT_EQ(Mesh("a/b.weird.ObJ").extension(), ".obj");
  // Arbitrary extensions.
  EXPECT_EQ(Mesh("a/b.extension").extension(), ".extension");

  // Now repeat for Convex.

  EXPECT_EQ(Convex("a/b.obj").extension(), ".obj");
  EXPECT_EQ(Convex("a/b.oBj").extension(), ".obj");
  EXPECT_EQ(Convex("a/b.ObJ").extension(), ".obj");
  EXPECT_EQ(Convex("a/b.weird.ObJ").extension(), ".obj");
  // Arbitrary extensions.
  EXPECT_EQ(Convex("a/b.extension").extension(), ".extension");
}

GTEST_TEST(ShapeTest, MoveConstructor) {
  // Create an original mesh.
  Mesh orig("foo.obj");
  const std::string orig_filename = orig.source().description();
  EXPECT_EQ(orig.extension(), ".obj");

  // Move it into a different mesh.
  Mesh next(std::move(orig));
  EXPECT_EQ(next.source().description(), orig_filename);
  EXPECT_EQ(next.extension(), ".obj");

  // The moved-from mesh is in a valid but indeterminate state.
  EXPECT_EQ(orig.source().description().empty(), orig.extension().empty());
}

}  // namespace
}  // namespace geometry
}  // namespace drake
