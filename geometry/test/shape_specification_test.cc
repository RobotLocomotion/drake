#include "drake/geometry/shape_specification.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/is_dynamic_castable.h"

namespace drake {
namespace geometry {
namespace {

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
  void ImplementGeometry(const Mesh&, void*) override {
    // TODO(SeanCurtis-TRI): Provide body when meshes are meaningfully
    // supported.
    EXPECT_TRUE(false) << "Don't test Meshes until they are fully supported";
  }
  void Reset() {
    sphere_made_ = false;
    half_space_made_ = false;
    cylinder_made_ = false;
    received_user_data_ = nullptr;
  }

 protected:
  bool box_made_{false};
  bool sphere_made_{false};
  bool cylinder_made_{false};
  bool half_space_made_{false};
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

  Reset();

  HalfSpace hs{};
  hs.Reify(this);
  EXPECT_FALSE(sphere_made_);
  EXPECT_TRUE(half_space_made_);
  EXPECT_FALSE(cylinder_made_);
  EXPECT_FALSE(box_made_);

  Reset();

  Cylinder cylinder{1, 2};
  cylinder.Reify(this);
  EXPECT_FALSE(sphere_made_);
  EXPECT_FALSE(half_space_made_);
  EXPECT_TRUE(cylinder_made_);
  EXPECT_FALSE(box_made_);

  Reset();

  Box box{1, 2, 3};
  box.Reify(this);
  EXPECT_FALSE(sphere_made_);
  EXPECT_FALSE(half_space_made_);
  EXPECT_FALSE(cylinder_made_);
  EXPECT_TRUE(box_made_);
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
    ASSERT_EQ(raw_sphere->get_radius(), local_sphere.get_radius());
  }
  // Now confirm it's still alive. I should be able to reify it into a sphere.
  cloned_shape->Reify(this);
  ASSERT_TRUE(sphere_made_);
  ASSERT_FALSE(half_space_made_);
  ASSERT_FALSE(cylinder_made_);
  ASSERT_FALSE(box_made_);
}


// Given the pose of a plane and its expected translation and z-axis, confirms
// that the pose conforms to expectations. Also confirms that the rotational
// component is orthonormal.
::testing::AssertionResult ValidatePlanePose(
    const Eigen::Isometry3d& pose, const Vector3<double>& expected_z,
    const Vector3<double>& expected_translation, double tolerance = 1e-14) {
  using std::abs;

  // Test expected z-axis value.
  const Vector3<double>& z_axis = pose.linear().col(2);
  if (!CompareMatrices(z_axis, expected_z, tolerance,
                       MatrixCompareType::absolute)) {
    return ::testing::AssertionFailure()
        << "pose =\n"
        << pose.matrix() << "\nExpected z-axis " << expected_z.transpose()
        << " does not match pose's z-axis " << z_axis.transpose();
  }

  // Test expected translation.
  if (!CompareMatrices(pose.translation(), expected_translation, tolerance,
                       MatrixCompareType::absolute)) {
    return ::testing::AssertionFailure()
        << "pose =\n"
        << pose.matrix() << "\nExpected translation "
        << expected_translation.transpose()
        << " does not match pose's translation "
        << pose.translation().transpose();
  }

  // Test unit-length rotation.
  char axis_labels[] = {'x', 'y', 'z'};
  for (int i = 0; i < 3; ++i) {
    if (abs(pose.linear().col(i).norm() - 1) > tolerance) {
      return ::testing::AssertionFailure()
          << "pose =\n"
          << pose.matrix() << "\ndoes not have unit length " << axis_labels[i]
          << "-axis " << pose.linear().col(i).transpose();
    }
  }

  // Test orthogonality.
  for (int i = 0; i < 2; ++i) {
    for (int j = i + 1; j < 3; ++j) {
      double dot_product = pose.linear().col(i).dot(pose.linear().col(j));
      if (abs(dot_product) > tolerance) {
        return ::testing::AssertionFailure()
            << "For pose =\n"
            << pose.matrix() << "\nThe " << axis_labels[i] << "-axis and "
            << axis_labels[j] << "-axis are not orthogonal";
      }
    }
  }
  return ::testing::AssertionSuccess()
      << "pose =\n"
      << pose.matrix() << "\nhas expected z-axis = " << expected_z.transpose()
      << "\nand expected translation = " << expected_translation.transpose();
}

// Confirms that the pose computed by HalfSpace::MakePose is consistent with
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
    Isometry3<double> pose = HalfSpace::MakePose(n, p);
    EXPECT_TRUE(ValidatePlanePose(pose, n, p));
  }

  // Case: n(0, 0, 1), p(0, 0, 1) --> <0, 0, 1> z-axis, <0, 0, 1> translation.
  {
    n << 0, 0, 1;
    p << 0, 0, 1;
    Isometry3<double> pose = HalfSpace::MakePose(n, p);
    EXPECT_TRUE(ValidatePlanePose(pose, n, p));
  }

  // Case: n(0, 0, 1), p(1, 0, 0) --> <0, 0, 1> z-axis, <0, 0, 0> translation.
  // Confirms that the pose's translation is the minimum translation.
  {
    n << 0, 0, 1;
    p << 1, 0, 0;
    Isometry3<double> pose = HalfSpace::MakePose(n, p);
    EXPECT_TRUE(ValidatePlanePose(pose, n, Vector3<double>::Zero()));
  }

  // Case: n(0, 0, 1), p(1, 1, 1) --> identity rotation, <0, 0, 1> translation.
  // Confirms that the pose's translation is the minimum translation.
  {
    n << 0, 0, 1;
    p << 1, 1, 1;
    Isometry3<double> pose = HalfSpace::MakePose(n, p);
    p << 0, 0, 1;
    EXPECT_TRUE(ValidatePlanePose(pose, n, p));
  }

  // Case: n(1, 1, 1), p(0, 0, 0) --> rotation z-axis = <1, 1, 1> / √3,
  //       <0, 0, 0> translation. Rotation matrix orthonormal.
  {
    n << 1, 1, 1;
    p << 0, 0, 0;
    Isometry3<double> pose = HalfSpace::MakePose(n, p);
    EXPECT_TRUE(ValidatePlanePose(pose, n.normalized(), p));
  }

  // Case: n(1, 1, 1), p(1, 1, 1) --> rotation z-axis = <1, 1, 1> / √3,
  //       <1, 1, 1> translation. Rotation matrix orthonormal.
  {
    n << 1, 1, 1;
    p << 1, 1, 1;
    Isometry3<double> pose = HalfSpace::MakePose(n, p);
    EXPECT_TRUE(ValidatePlanePose(pose, n.normalized(), p));
  }

  // Case: n(1, 1, 1), p(1, 0, 0) --> rotation z-axis = <1, 1, 1> / √3,
  //       <1/3, 1/3, 1/3> translation. Rotation matrix orthonormal.
  // Confirms that the pose's translation is the minimum translation.
  {
    n << 1, 1, 1;
    p << 1, 0, 0;
    Isometry3<double> pose = HalfSpace::MakePose(n, p);
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

}  // namespace
}  // namespace geometry
}  // namespace drake
