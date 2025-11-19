#include "drake/geometry/shape_specification.h"

#include <filesystem>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

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

namespace fs = std::filesystem;

using Eigen::Vector2d;
using Eigen::Vector3d;
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
    DRAKE_EXPECT_THROWS_MESSAGE(HalfSpace::MakePose(n, p), ".*norm >=.*");
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
// parameters are reflected in the getters. Mesh and Convex are included in
// their own tests as their family of constructors is so large.
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

  // For Convex, see ConvexConstructor, below.

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

  // For Mesh, see MeshConstructor, below.

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

using ScaleVariant = std::variant<std::monostate, double, Vector3<double>>;

// This function can exercise all Convex constructors (the power set of vertex
// sources and scale types -- even omitting scale entirely).
template <typename... NonScaleArgs>
Convex MakeConvex(const ScaleVariant& scale, NonScaleArgs... args) {
  if (std::holds_alternative<std::monostate>(scale)) {
    return Convex(args...);
  } else if (std::holds_alternative<double>(scale)) {
    return Convex(args..., std::get<double>(scale));
  } else {
    return Convex(args..., std::get<Vector3<double>>(scale));
  }
}

GTEST_TEST(ShapeTest, ConvexConstructor) {
  const double kScale = 1.5;
  const Vector3<double> kScale3(2, 3, 4);

  const std::string kFileName = "/fictitious_name.obj";

  using Source = std::variant<std::filesystem::path, InMemoryMesh, MeshSource,
                              Eigen::Matrix3X<double>>;

  // Note: We're setting up each vertex source so that its "description" is
  // always kFileName. This simplifies the test.
  std::vector<Source> sources{
      kFileName, InMemoryMesh{MemoryFile("v 0 0 0", ".obj", kFileName)},
      MeshSource(kFileName), Eigen::Matrix<double, 3, 1>::Zero()};

  for (const auto& convex_source : sources) {
    for (const auto& scale :
         {ScaleVariant{}, ScaleVariant{kScale}, ScaleVariant{kScale3}}) {
      const auto convex = std::visit<Convex>(
          overloaded{
              [&scale](const std::filesystem::path& path) {
                Convex c = MakeConvex(scale, path);
                return c;
              },
              [&scale](const InMemoryMesh& memory_mesh) {
                Convex c = MakeConvex(scale, memory_mesh);
                return c;
              },
              [&scale](const MeshSource& source) {
                return MakeConvex(scale, source);
              },
              [&scale, &kFileName](const Eigen::Matrix3X<double>& points) {
                // Source of points is, again, kFileName.
                return MakeConvex(scale, points, kFileName);
              }},
          convex_source);
      EXPECT_EQ(convex.source().description(), kFileName);
      EXPECT_EQ(convex.extension(), ".obj");
      std::visit(overloaded{[&convex](std::monostate) {
                              EXPECT_EQ(convex.scale(), 1.0);
                            },
                            [&convex](double s) {
                              EXPECT_EQ(convex.scale(), s);
                            },
                            [&convex](const Vector3<double>& s3) {
                              DRAKE_EXPECT_THROWS_MESSAGE(
                                  convex.scale(),
                                  ".*uniform scaling.*\\[2 3 4\\].*");
                              EXPECT_TRUE(CompareMatrices(convex.scale3(), s3));
                            }},
                 scale);
    }
  }
}

// This function can exercise all Mesh constructors (the power set of mesh
// sources and scale types -- even omitting scale entirely).
template <typename SourceType>
Mesh MakeMesh(const SourceType& source, const ScaleVariant& scale) {
  if (std::holds_alternative<std::monostate>(scale)) {
    return Mesh(source);
  } else if (std::holds_alternative<double>(scale)) {
    return Mesh(source, std::get<double>(scale));
  } else {
    return Mesh(source, std::get<Vector3<double>>(scale));
  }
}

GTEST_TEST(ShapeTest, MeshConstructor) {
  const double kScale = 1.5;
  const Vector3<double> kScale3(2, 3, 4);

  const std::string kFileName = "/fictitious_name.obj";

  using Source = std::variant<std::filesystem::path, InMemoryMesh, MeshSource>;

  // Note: We're setting up each vertex source so that its "description" is
  // always kFileName. This simplifies the test.
  std::vector<Source> sources{
      kFileName, InMemoryMesh{MemoryFile("v 0 0 0", ".obj", kFileName)},
      MeshSource(kFileName)};

  for (const auto& mesh_source : sources) {
    for (const auto& scale :
         {ScaleVariant{}, ScaleVariant{kScale}, ScaleVariant{kScale3}}) {
      const auto mesh = std::visit<Mesh>(
          overloaded{[&scale](const std::filesystem::path& path) {
                       Mesh m = MakeMesh(path, scale);
                       return m;
                     },
                     [&scale](const InMemoryMesh& memory_mesh) {
                       Mesh m = MakeMesh(memory_mesh, scale);
                       return m;
                     },
                     [&scale](const MeshSource& source) {
                       return MakeMesh(source, scale);
                     }},
          mesh_source);
      EXPECT_EQ(mesh.source().description(), kFileName);
      EXPECT_EQ(mesh.extension(), ".obj");
      std::visit(overloaded{[&mesh](std::monostate) {
                              EXPECT_EQ(mesh.scale(), 1.0);
                            },
                            [&mesh](double s) {
                              EXPECT_EQ(mesh.scale(), s);
                            },
                            [&mesh](const Vector3<double>& s3) {
                              DRAKE_EXPECT_THROWS_MESSAGE(
                                  mesh.scale(),
                                  ".*uniform scaling.*\\[2 3 4\\].*");
                              EXPECT_TRUE(CompareMatrices(mesh.scale3(), s3));
                            }},
                 scale);
    }
  }
}

// Confirms the scale factors are tested in Mesh and Convex constructors.
GTEST_TEST(ShapeTest, MeshAndConvexValidateScale) {
  const std::string kFilename = "/fictitious_name.obj";

  using V3 = Vector3<double>;
  using S = std::variant<double, Vector3<double>>;

  // We're using the single scalar to test a range of bad values and using the
  // non-uniform scalar to test each axis independently.
  for (const auto& bad_scale : {S(0.0), S(1e-9), S(-1e-9), S(V3(0, 1, 1)),
                                S(V3(1, 0, 1)), S(V3(1, 1, 0))}) {
    std::visit(overloaded{[&f = kFilename](auto&& scale) {
                 DRAKE_EXPECT_THROWS_MESSAGE(Mesh(f, scale), ".*|scale|.*");
                 DRAKE_EXPECT_THROWS_MESSAGE(Convex(f, scale), ".*|scale|.*");
               }},
               bad_scale);
  }
}

// Confirms that shape parameters are validated. For the vector-based
// constructors, we only provide a single invocation, relying on the idea that
// it forwards construction to the validating constructor with individual
// parameters. We test just enough of the error message to confirm that the
// erroneous value is part of the message. We don't worry about the *rest* of
// the message, relying on DRAKE_THROW_UNLESS to do the right thing.
//
// We haven't tested NaN and infinity for every parameter. We test against *one*
// parameter as a regression and rely on code inspection to confirm that all
// parameters are apparently treated the same.
GTEST_TEST(ShapeTest, NumericalValidation) {
  constexpr double kInf = std::numeric_limits<double>::infinity();
  constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

  DRAKE_EXPECT_THROWS_MESSAGE(Box(2, -1.5, 2), ".*depth =.*");
  DRAKE_EXPECT_THROWS_MESSAGE(Box(3, 1, -1.5), ".*height =.*");
  DRAKE_EXPECT_THROWS_MESSAGE(Box(3, 1, kNan), ".*height =.*");
  DRAKE_EXPECT_THROWS_MESSAGE(Box(3, 1, kInf), ".*height =.*");
  DRAKE_EXPECT_THROWS_MESSAGE(Box(Vector3d{-2.5, 1, 3}), ".*width =.*");
  DRAKE_EXPECT_THROWS_MESSAGE(Box::MakeCube(-1.5), ".*width =.*");

  DRAKE_EXPECT_THROWS_MESSAGE(Capsule(-0.5, 1), ".*radius =.*");
  DRAKE_EXPECT_THROWS_MESSAGE(Capsule(0.5, -1.25), ".*length =.*");
  DRAKE_EXPECT_THROWS_MESSAGE(Capsule(0.5, kNan), ".*length =.*");
  DRAKE_EXPECT_THROWS_MESSAGE(Capsule(0.5, kInf), ".*length =.*");
  DRAKE_EXPECT_THROWS_MESSAGE(Capsule(Vector2d{0.5, -1.75}), ".*length =.*");

  // The first invocation is testing the message contents, the remaining tests
  // merely confirms the other conditions also throw.
  DRAKE_EXPECT_THROWS_MESSAGE(Convex("bar", 0),
                              "Convex .scale. cannot be < 1e-8.*");
  EXPECT_THROW(Convex("bar", kInf), std::exception);
  EXPECT_THROW(Convex("bar", kNan), std::exception);
  EXPECT_THROW(Convex(InMemoryMesh{MemoryFile("a", ".a", "a")}, 0),
               std::exception);
  // Special case for negative scale.
  DRAKE_EXPECT_NO_THROW(Convex("foo", -1));
  DRAKE_EXPECT_NO_THROW(Convex(InMemoryMesh{MemoryFile("a", ".a", "a")}, -1));

  DRAKE_EXPECT_THROWS_MESSAGE(Cylinder(-0.25, 1), ".*radius =.*");
  DRAKE_EXPECT_THROWS_MESSAGE(Cylinder(0.5, -1.5), ".*length =.*");
  DRAKE_EXPECT_THROWS_MESSAGE(Cylinder(0.5, kNan), ".*length =.*");
  DRAKE_EXPECT_THROWS_MESSAGE(Cylinder(0.5, kInf), ".*length =.*");
  DRAKE_EXPECT_THROWS_MESSAGE(Cylinder(Vector2d{0.5, -1.5}), ".*length =.*");

  DRAKE_EXPECT_THROWS_MESSAGE(Ellipsoid(0, 1, 1), ".*a =.*");
  DRAKE_EXPECT_THROWS_MESSAGE(Ellipsoid(1, 0, 1), ".*b =.*");
  DRAKE_EXPECT_THROWS_MESSAGE(Ellipsoid(1, 1, 0), ".*c =.*");
  DRAKE_EXPECT_THROWS_MESSAGE(Ellipsoid(1, 1, kNan), ".*c =.*");
  DRAKE_EXPECT_THROWS_MESSAGE(Ellipsoid(1, 1, kInf), ".*c =.*");
  DRAKE_EXPECT_THROWS_MESSAGE(Ellipsoid(Vector3d{1, 1, 0}), ".*c =.*");

  // The first invocation is testing the message contents, the remaining tests
  // merely confirms the other conditions also throw.
  DRAKE_EXPECT_THROWS_MESSAGE(Mesh("foo", 1e-9),
                              "Mesh .scale. cannot be < 1e-8.*");
  EXPECT_THROW(Mesh("foo", kInf), std::exception);
  EXPECT_THROW(Mesh("foo", kNan), std::exception);
  EXPECT_THROW(Mesh(InMemoryMesh{MemoryFile("a", ".a", "a")}, 0),
               std::exception);
  // Special case for negative scale.
  DRAKE_EXPECT_NO_THROW(Mesh("foo", -1));
  DRAKE_EXPECT_NO_THROW(Mesh(InMemoryMesh{MemoryFile("a", ".a", "a")}, -1));

  DRAKE_EXPECT_THROWS_MESSAGE(MeshcatCone(0, 1, 1), ".*height =.*");
  DRAKE_EXPECT_THROWS_MESSAGE(MeshcatCone(1, 0, 1), ".*a =.*");
  DRAKE_EXPECT_THROWS_MESSAGE(MeshcatCone(1, 1, 0), ".*b =.*");
  DRAKE_EXPECT_THROWS_MESSAGE(MeshcatCone(1, 1, kNan), ".*b =.*");
  DRAKE_EXPECT_THROWS_MESSAGE(MeshcatCone(1, 1, kInf), ".*b =.*");
  DRAKE_EXPECT_THROWS_MESSAGE(MeshcatCone(Vector3d{1, 1, 0}), ".*b =.*");

  DRAKE_EXPECT_THROWS_MESSAGE(Sphere(-0.5), ".*radius =.*");
  DRAKE_EXPECT_THROWS_MESSAGE(Sphere(kNan), ".*radius =.*");
  DRAKE_EXPECT_THROWS_MESSAGE(Sphere(kInf), ".*radius =.*");
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

// Confirmation that Mesh and Convex can successfully produce convex hulls
// from memory. (Construction has otherwise already been tested.)
GTEST_TEST(ShapeTest, ConvexHullsFromMemory) {
  // We'll confirm computation doesn't throw and produces a mesh with expected
  // topology. We don't test all file formats; we trust that visual
  // inspection of the code under test shows that it doesn't depend on file
  // format.
  auto confirm_hull = [](const auto&& shape, std::string_view description) {
    SCOPED_TRACE(description);
    const PolygonSurfaceMesh<double>& hull = shape.GetConvexHull();
    EXPECT_EQ(hull.num_vertices(), 8);
    EXPECT_EQ(hull.num_elements(), 6);
  };

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
  const MeshSource source(
      InMemoryMesh{MemoryFile(obj_contents, ".OBJ", mesh_name)});

  confirm_hull(Convex(source.in_memory()), "Convex from InMemoryMesh");
  confirm_hull(Convex(source), "Convex from MeshSource");
  confirm_hull(Mesh(source.in_memory()), "Mesh from InMemoryMesh");
  confirm_hull(Mesh(source), "Mesh from MeshSource");
}

GTEST_TEST(ShapeTest, ConvexFromVertices) {
  // Make sure the convex hull is automatically taken.
  Eigen::Matrix<double, 3, 5> points;
  points.col(0) << 0, 0, 0;
  points.col(1) << 1, 0, 0;
  points.col(2) << 0.25, 0.25, 0.25;
  points.col(3) << 0, 1, 0;
  points.col(4) << 0, 0, 1;

  const std::string mesh_name = "a_convex";
  // Note: passing scale (of various flavors) has already been tested in the
  // ConvexConstruction test.
  const Convex convex(points, mesh_name);

  const MeshSource& source = convex.source();
  EXPECT_EQ(source.in_memory().mesh_file.filename_hint(), mesh_name);

  // Confirm that the contents of the in-memory mesh file are as expected. Note:
  // We prefix a "\n" to match leading "\n" in the nicely readable raw string.
  EXPECT_EQ("\n" + source.in_memory().mesh_file.contents(),
            R"""(
v 0 0 0
v 1 0 0
v 0.25 0.25 0.25
v 0 1 0
v 0 0 1
)""");
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
  // In-memory mesh we'll use on Convex and Mesh.
  const InMemoryMesh in_memory{
      MemoryFile("a", ".a", "A"),
      {{"bb", MemoryFile("b", ".b", "B")}, {"cc", fs::path("path/to/c")}}};
  // Mesh and Convex both defer to InMemoryMesh to format the member, so we
  // simply need to prove that it calls the right method based on the type of
  // the source (we don't have to worry about *how* InMemoryMesh is written as
  // a string). We do need to make sure they have the [2, 3, 4] scale factor
  // as hard-coded here.
  static constexpr const char* mem_fmt = "{}(mesh_data={}, scale=[2 3 4])";

  const Box box(1.5, 2.5, 3.5);
  const Capsule capsule(1.25, 2.5);
  const Convex convex("/some/file", 1.5);
  const Convex mem_convex(in_memory, Vector3<double>(2, 3, 4));
  const Cylinder cylinder(1.25, 2.5);
  const Ellipsoid ellipsoid(1.25, 2.5, 0.5);
  const HalfSpace half_space;
  const Mesh mesh("/some/file", 1.5);
  const Mesh mem_mesh(in_memory, Vector3<double>(2, 3, 4));
  const MeshcatCone cone(1.5, 0.25, 0.5);
  const Sphere sphere(1.25);

  EXPECT_EQ(box.type_name(), "Box");
  EXPECT_EQ(capsule.type_name(), "Capsule");
  EXPECT_EQ(convex.type_name(), "Convex");
  EXPECT_EQ(mem_convex.type_name(), "Convex");
  EXPECT_EQ(cylinder.type_name(), "Cylinder");
  EXPECT_EQ(ellipsoid.type_name(), "Ellipsoid");
  EXPECT_EQ(half_space.type_name(), "HalfSpace");
  EXPECT_EQ(mesh.type_name(), "Mesh");
  EXPECT_EQ(mem_mesh.type_name(), "Mesh");
  EXPECT_EQ(cone.type_name(), "MeshcatCone");
  EXPECT_EQ(sphere.type_name(), "Sphere");

  EXPECT_EQ(box.to_string(), "Box(width=1.5, depth=2.5, height=3.5)");
  EXPECT_EQ(capsule.to_string(), "Capsule(radius=1.25, length=2.5)");
  EXPECT_EQ(convex.to_string(),
            "Convex(filename='/some/file', scale=[1.5 1.5 1.5])");
  EXPECT_EQ(mem_convex.to_string(),
            fmt::format(mem_fmt, "Convex", in_memory.to_string()));
  EXPECT_EQ(cylinder.to_string(), "Cylinder(radius=1.25, length=2.5)");
  EXPECT_EQ(ellipsoid.to_string(), "Ellipsoid(a=1.25, b=2.5, c=0.5)");
  EXPECT_EQ(half_space.to_string(), "HalfSpace()");
  EXPECT_EQ(mesh.to_string(),
            "Mesh(filename='/some/file', scale=[1.5 1.5 1.5])");
  EXPECT_EQ(mem_mesh.to_string(),
            fmt::format(mem_fmt, "Mesh", in_memory.to_string()));
  EXPECT_EQ(cone.to_string(), "MeshcatCone(height=1.5, a=0.25, b=0.5)");
  EXPECT_EQ(sphere.to_string(), "Sphere(radius=1.25)");

  EXPECT_EQ(fmt::to_string(box), "Box(width=1.5, depth=2.5, height=3.5)");
  EXPECT_EQ(fmt::to_string(capsule), "Capsule(radius=1.25, length=2.5)");
  EXPECT_EQ(fmt::to_string(convex),
            "Convex(filename='/some/file', scale=[1.5 1.5 1.5])");
  EXPECT_EQ(fmt::to_string(mem_convex), mem_convex.to_string());
  EXPECT_EQ(fmt::to_string(cylinder), "Cylinder(radius=1.25, length=2.5)");
  EXPECT_EQ(fmt::to_string(ellipsoid), "Ellipsoid(a=1.25, b=2.5, c=0.5)");
  EXPECT_EQ(fmt::to_string(half_space), "HalfSpace()");
  EXPECT_EQ(fmt::to_string(mesh),
            "Mesh(filename='/some/file', scale=[1.5 1.5 1.5])");
  EXPECT_EQ(fmt::to_string(mem_mesh), mem_mesh.to_string());
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
            fs::current_path() / "relative_path.obj");

  const Convex relpath_convex("relative_path.obj");
  EXPECT_TRUE(relpath_convex.source().path().is_absolute());
  EXPECT_EQ(relpath_convex.source().path(),
            fs::current_path() / "relative_path.obj");
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

  // The moved-from mesh has its source revert to an empty in-memory mesh.
  ASSERT_TRUE(orig.source().is_in_memory());
  EXPECT_TRUE(orig.source().in_memory().mesh_file.contents().empty());
}

}  // namespace
}  // namespace geometry
}  // namespace drake
