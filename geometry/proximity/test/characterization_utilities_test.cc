#include "drake/geometry/proximity/test/characterization_utilities.h"

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

using Eigen::Vector3d;
using math::RigidTransformd;
using std::pair;
using std::vector;

namespace {

/* These unit tests test the mathematical underpinnings of the
 CharacterizeResultTest. Specifically, it tests that the pieces of code which
 contribute to *configuring* a test are valid.

 It also tests the code execution by creating a *fake* callback whose results
 are controlled by the test to make sure that the test is detecting error that
 is *known* to exist.  */

/* The MakeFclShapeTest tests confirm that Drake shape specifications turn into
 the expected fcl geometries. */

GTEST_TEST(MakeFclShapeTest, Capsule) {
  const Capsule capsule(0.25, 0.75);
  auto fcl_geometry = MakeFclShape(capsule).object();
  const auto& fcl_capsule = dynamic_cast<fcl::Capsuled&>(*fcl_geometry);
  EXPECT_EQ(fcl_capsule.radius, capsule.radius());
  EXPECT_EQ(fcl_capsule.lz, capsule.length());
}

GTEST_TEST(MakeFclShapeTest, Ellipsoid) {
  const Ellipsoid ellipsoid(0.25, 0.75, 0.6);
  auto fcl_geometry = MakeFclShape(ellipsoid).object();
  const auto& fcl_ellipsoid = dynamic_cast<fcl::Ellipsoidd&>(*fcl_geometry);
  EXPECT_EQ(fcl_ellipsoid.radii[0], ellipsoid.a());
  EXPECT_EQ(fcl_ellipsoid.radii[1], ellipsoid.b());
  EXPECT_EQ(fcl_ellipsoid.radii[2], ellipsoid.c());
}

GTEST_TEST(MakeFclShapeTest, Sphere) {
  const Sphere sphere(0.7);
  auto fcl_geometry = MakeFclShape(sphere).object();
  const auto& fcl_sphere = dynamic_cast<fcl::Sphered&>(*fcl_geometry);
  EXPECT_EQ(fcl_sphere.radius, sphere.radius());
}

/* The SampleShapeSurfaceTest tests confirm the two important properties of
 the sample:
   - The point on the plane lies on the surface of the shape.
   - The entire shape lies "under" the plane (i.e. for every point, the signed
     distance to the plane is <= 0).

 For each shape, we determine the *second* item by identifying a/the most
 extreme point of the shape in the direction of the plane normal. As long as its
 distance to the plane is <= 0, then all other points must likewise satisfy the
 requirement.  */
// TODO(SeanCurtis-TRI) When we write our own implementation of GJK/EPA the
//  logic used here to compute the extreme point should be refactored as the
//  per-shape support function (except for Convex).

GTEST_TEST(SampleShapeSurfaceTest, Capsule) {
  const double kEps = std::numeric_limits<double>::epsilon();

  const Capsule capsule(0.5, 0.75);

  /* We validate for a requested penetration that is too deep.  */
  EXPECT_THROW(ShapeConfigurations<double>(capsule, -capsule.radius() * 1.01),
               std::exception);
  /* Otherwise, any distance value larger is fine. */
  for (const auto& tangent_plane :
       ShapeConfigurations<double>(capsule, 0.0).configs()) {
    const Vector3d p_GP = tangent_plane.point;
    const Vector3d n_G = tangent_plane.normal;
    /* A capsule is the Minkowski sum of a sphere and a line segment. So, the
     extreme point E will always be a point on a sphere with the capsule radius
     whose center is located on the line segment. Specifically, it's the point
     of the sphere in the plane normal direction. So, the only challenge is
     figuring out where the sphere center is: the point on the *line* segment
     that is most in the plane normal direction.

     The capsule is defined with its axis aligned with Gz. If n_G.z() is
     positive, the segment's extreme point (and sphere center C) lies at
     (0, 0, l/2), if  negative, (0, 0, -l/2). Otherwise, all points on the line
     are equally close to the plane. */
    const double half_length = capsule.length() / 2;
    const Vector3d p_GC(0, 0, half_length * (n_G.z() > 0 ? 1 : -1));
    const Vector3d p_GE = p_GC + n_G * capsule.radius();
    /* Now confirm the extreme point lies "under" the plane. */
    ASSERT_LE((p_GE - p_GP).dot(n_G), kEps);

    /* Point p_GP should be radius distance from the line segment. We find the
     nearest point to P on the line segment as (0, 0, z) and determine the
     distance is equal to the capsule radius. */
    const double z = std::clamp(p_GP.z(), -half_length, half_length);
    ASSERT_NEAR((p_GP - Vector3d(0, 0, z)).norm(), capsule.radius(), kEps);
  }
}

GTEST_TEST(SampleShapeSurfaceTest, Ellipsoid) {
  /* We need a bit more slack in determining the calculations of the ellipsoid.
   Empirically, one bit seemed sufficient, we'll take two to buy some
   cross-platform resiliency. */
  const double kEps = 4 * std::numeric_limits<double>::epsilon();

  const Ellipsoid ellipsoid(0.5, 0.75, 0.325);

  /* We validate for a requested penetration that is too deep. And we're
   exploiting the knowledge that we constructed the ellipsoid with c as the
   minimum coefficient. */
  EXPECT_THROW(ShapeConfigurations<double>(ellipsoid, -ellipsoid.c() * 1.01),
               std::exception);

  /* These quantities are for the calculation documented below. */
  const Vector3d axes(ellipsoid.a(), ellipsoid.b(), ellipsoid.c());
  const Vector3d axes_squared = axes.cwiseProduct(axes);

  /* Otherwise, any distance value larger is fine. */
  for (const auto& tangent_plane :
       ShapeConfigurations<double>(ellipsoid, 0.0).configs()) {
    const Vector3d p_GP = tangent_plane.point;
    const Vector3d n_G = tangent_plane.normal;
    /* The extreme point on the ellipsoid in the n_G direction can be found by
     (1) applying an affine transformation to transform the ellipsoid and
     normal such that the ellipsoid becomes a unit sphere, (2) find the point on
     the sphere S whose normal is in the same direction as the transformed
     normal, and (3) transform the S to its corresponding point on the ellipsoid
     E.

     Note: The affine transformation from ellipsoid to sphere is as follows:
       - for *point* P on ellipsoid: <Px/a, Py/b, Pz/c>
       - for *normal* n on ellipsoid: <a⋅nx, b⋅ny, c⋅nz>
     We reverse them to map the other way.

       n_G = <nx, ny, nz>
       n_S = <a⋅nx, b⋅ny, c⋅nz>       (1) Affine transform the *normal*.
       S = n_S / |n_S|                (2) We get the normal by normalizing the
                                          direction to the point on the sphere.
       E = <a⋅Sx, b⋅Sy, c⋅Sz>         (3) Affine transform of *point*.

     With some algebra, it looks like this:

       E = <a²⋅nx, b²⋅ny, c²⋅nz> / sqrt(a²⋅nx² + b²⋅ny² + c²⋅nz²) */
    const Vector3d E_scaled = n_G.cwiseProduct(axes_squared);
    const Vector3d p_GE = E_scaled / std::sqrt(n_G.dot(E_scaled));
    /* Validate E lies on the ellipsoid: x²/a² + y²/b² + z²/c² = 1. */
    ASSERT_NEAR(p_GE.cwiseProduct(p_GE).cwiseQuotient(axes_squared).sum(), 1.0,
                kEps);
    /* Now confirm the extreme point lies "under" the plane. */
    ASSERT_LE((p_GE - p_GP).dot(n_G), kEps);

    /* Validate P lies on the ellipsoid: x²/a² + y²/b² + z²/c² = 1. */
    ASSERT_NEAR(p_GP.cwiseProduct(p_GP).cwiseQuotient(axes_squared).sum(), 1.0,
                kEps);
  }
}

GTEST_TEST(SampleShapeSurfaceTest, Sphere) {
  const double kEps = std::numeric_limits<double>::epsilon();

  const Sphere sphere(0.75);

  /* We validate for a requested penetration that is too deep.  */
  EXPECT_THROW(ShapeConfigurations<double>(sphere, -sphere.radius() * 1.01),
               std::exception);
  /* Otherwise, any distance value larger is fine. */
  for (const auto& tangent_plane :
       ShapeConfigurations<double>(sphere, 0.0).configs()) {
    const Vector3d p_GP = tangent_plane.point;
    const Vector3d n_G = tangent_plane.normal;
    /* The extreme point is simply a point radius distance from the origin in
     the n_G direction. */
    const Vector3d p_GE = n_G * sphere.radius();
    /* Now confirm the extreme point lies "under" the plane. */
    ASSERT_LE((p_GE - p_GP).dot(n_G), kEps);

    /* The point P must be radius distance from the origin.  */
    EXPECT_NEAR(p_GP.norm(), sphere.radius(), kEps);
  }
}

/* This confirms that AlignPlanes successfully aligns the planes; the pose given
 makes the points coincident and the normals anti-parallel. */
GTEST_TEST(AlignPlanes, Correctness) {
  const double kEps = std::numeric_limits<double>::epsilon();

  /* We pick an arbitrary, ugly point and normal. */
  const Vector3d p_WP(0.123, -0.456, 0.789);
  const Vector3d m_W = Vector3d(1, 2, -3).normalized();

  /* We'll create a sequence of tangent planes and confirm that they get
   properly aligned. */
  vector<pair<Vector3d, Vector3d>> test_planes{
      {Vector3d{0, 0, 1}, Vector3d{0, -1, 0}},
      {Vector3d{0.125, 0.25, 0.5}, Vector3d{-1, 2, 0}.normalized()},
      {Vector3d{-0.3, -0.7, 0.9}, Vector3d{3, -1, -2}.normalized()}};
  for (const auto& [p_WQ, n_W] : test_planes) {
    const RigidTransformd X_AB = AlignPlanes<double>(p_WP, m_W, p_WQ, n_W);
    EXPECT_TRUE(CompareMatrices(X_AB * p_WQ, p_WP, kEps));
    EXPECT_TRUE(CompareMatrices(X_AB.rotation() * n_W, -m_W, kEps));
  }
}

/* Dummy callback, callback data, and traits for the CharacterizeResultTest.  */

/* The callback and its data is designed to facilitate testing the
 RunCharacterization functions. That function invokes the callback multiple
 times (at least 4 times per configuration), with a fixed *expectation* on what
 the returned value will be. For each run, it measures the error and remembers
 the *worst* error. It then confirms the worst error is on scale with the
 expected level of precision.

 So, for a given "true answer" t and expected error e, we need a callback that
 will produce values in the range [t - e, t + e]. To make the test interesting
 we further need:

   - We have to have at least one result with the expected error.
   - We don't want that result first or last (to make sure the algorithm isn't
     accidentally saving the first or last values).
   - It needs to support an aribtrary number of invocations (although we'll
     require at least two).

 So, fortunately, we don't have to guess. In the tests we run, we know exactly
 how many configurations we're running, so we can announce the total number of
 expected invocations. Therefore, we'll initialize a test by declaring the
 number of configurations and the expected error, the result of which, we'll
 cache a sequence of values that satisfy the properties above.

 That means each test must invoke SetValueSequence() prior to invoking
 RunCharacterization.  */
template <typename T>
class DummyCallbackData {
 public:
  explicit DummyCallbackData(vector<T>* results) : results_(results) {
    DRAKE_DEMAND(results != nullptr);
  }

  static void SetValueSequence(int invocation_count, double truth,
                               double precision) {
    DRAKE_DEMAND(invocation_count >= 2);
    invocations = 0;
    sequence.clear();
    sequence.reserve(invocation_count);
    /* We'll create a kind of damped sinusoidal signal. The first result will
     be truth, the second will be the truth + precision, and all other answers
     will have decaying error.

     t+p  │      o
          │
          │                  o
       t  ├─o───────────────────────
          │
          │            o
      t-p │

           1    2     3     4  ...
      */
    sequence.push_back(truth);
    /* We need this to be <= truth + precision, so we'll knock down the
     precision ever so slightly to ensure it. */
    sequence.push_back(truth + precision * 0.9999);
    const double decay = precision / invocation_count;
    for (int i = 2; i < invocation_count; ++i) {
      /* Linear decay to *almost* truth. */
      precision -= decay;
      /* Sign-alternating signal. */
      sequence.push_back(truth + (i % 2 == 0 ? -1 : 1) * precision);
    }
  }

  static T NextValue() { return sequence.at(invocations++); }

  static size_t get_invocations() { return invocations; }

  vector<T>& results() { return *results_; }
  const vector<T>& results() const { return *results_; }

 private:
  static size_t invocations;
  static vector<T> sequence;
  vector<T>* results_;
};

template <typename T>
size_t DummyCallbackData<T>::invocations = 0;

template <typename T>
vector<T> DummyCallbackData<T>::sequence;

template <typename T>
bool DummyCallback(fcl::CollisionObjectd*, fcl::CollisionObjectd*, void* data) {
  auto& dummy_data = *static_cast<DummyCallbackData<T>*>(data);
  dummy_data.results().push_back(DummyCallbackData<T>::NextValue());
  return false;
}

/* Make sure the sequence satisfies the properties given above. We want to make
 sure it supports the declared invocation count and returns values that are all
 within the precision (with the second value having exactly the expected
 error). */
GTEST_TEST(DummyCallbackTest, ConfirmErrorSequence) {
  /* Combination of various counts, truth values, and precisions. */
  for (int count : {4, 8, 12}) {
    for (double truth : {-0.5, 0.75}) {
      for (double precision : {1e-3, 1e-10, 1e-15}) {
        DummyCallbackData<double>::SetValueSequence(count, truth, precision);
        vector<double> results;
        DummyCallbackData<double> data(&results);
        for (int i = 0; i < count; ++i) {
          DummyCallback<double>(nullptr, nullptr, &data);
        }
        /* The second value has error equal to the declared precision. */
        ASSERT_NEAR(results[1], truth, precision);
        /* All *other* reported values have strictly less error. */
        for (int i = 0; i < count; ++i) {
          if (i == 1) continue;
          ASSERT_LT(std::abs(results[i] - truth), precision);
        }
      }
    }
  }
}

template <typename T>
class DummyImplementation : public DistanceCallback<T> {
 public:
  bool Invoke(
      fcl::CollisionObjectd* obj_A, fcl::CollisionObjectd* obj_B,
      const CollisionFilterLegacy*,
      const std::unordered_map<GeometryId, math::RigidTransform<T>>*) override {
    DummyCallbackData<T> data(&results_);
    return DummyCallback<T>(obj_A, obj_B, &data);
  }

  void ClearResults() override { results_.clear(); }

  int GetNumResults() const override {
    return static_cast<int>(results_.size());
  }

  T GetFirstSignedDistance() const override { return results_[0]; }

 private:
  vector<T> results_;
};

/* CharacterizeResultTest::RunCharacterization runs a number of configurations
 through a callback. Its primary task is to execute the callback and evaluate
 the results, confirming the observed, worst-case error and comparing that with
 expectations. This must be correct for us to trust it to report errors on
 production callbacks.

 In this case, we use the DummyCallback (documented above) with *known*
 semantics. Given the number of known invocations, a truth value, and a
 predicted error, we confirm that the test detects that error.

 We need to know the number of expected invocations. For each unique
 configuration, RunCharacterization invokes the callback multiple times:

   - Creates N poses mapping the configuration to another, arbitrary frame.
     N is the number of poses reported by X_WAs().
   - Once as geometry pair (A, B) and once for (B, A) (in case there's an
     asymmetry in the algorithm that produces bad answers).
   - We override MakeConfigurations so that there's only a single configuration
     between the two shapes.

  Therefore, for one configuration there will be 2N calls to callback. */
class RunCharacterizationCustomTest : public CharacterizeResultTest<double> {
 public:
  RunCharacterizationCustomTest()
      : CharacterizeResultTest<double>(
            std::make_unique<DummyImplementation<double>>()) {}

  vector<double> TestDistances() const final { return {kTruth}; }

  vector<Configuration<double>> MakeConfigurations(
      const Shape&, const Shape&, const vector<double>&) const final {
    return {Configuration<double>{{}, kTruth, "Dummy"}};
  }

  static constexpr double kTruth{1.0};
};

/* Create a single configuration and confirm the expected results.  */
TEST_F(RunCharacterizationCustomTest, CustomConfigurations) {
  const int call_count = 2 * static_cast<int>(X_WAs().size());
  for (const double target_precision : {1e-2, 1e-7, 1e-14}) {
    DummyCallbackData<double>::SetValueSequence(call_count, this->kTruth,
                                                target_precision);
    const QueryInstance query{kSphere, kSphere, target_precision};
    /* If this test passes, it detected the expected precision. */
    RunCharacterization(query);
    /* If this test passes, it did all the required work. */
    EXPECT_EQ(DummyCallbackData<double>::get_invocations(), call_count);
  }
}

/* In this case, we confirm that if MakeConfigurations is *not* overridden, that
 we'll get the expected collection (at least by count) of configurations
 automatically generated. Contrast that with RunCharacterizationCustomTest. */
class RunCharacterizationTest : public CharacterizeResultTest<double> {
 public:
  RunCharacterizationTest()
      : CharacterizeResultTest<double>(
            std::make_unique<DummyImplementation<double>>()) {}

  vector<double> TestDistances() const final {
    return {this->sphere().radius() * 0.25};
  }
};

/* Confirm that when we don't explicitly enumerate the configurations, but
 rely on the test to generate its own, that we get the results we expect. */
TEST_F(RunCharacterizationTest, DefaultConfigurations) {
  const Sphere sphere = this->sphere();
  const int sphere_config_count =
      static_cast<int>(ShapeConfigurations<double>(sphere, 0).configs().size());
  const int pose_count = static_cast<int>(X_WAs().size());
  const int call_count =
      2 * sphere_config_count * sphere_config_count * pose_count;

  const double truth = TestDistances()[0];
  const double max_error = truth * 1e-5;
  DummyCallbackData<double>::SetValueSequence(call_count, truth, max_error);
  const QueryInstance query(kSphere, kSphere, max_error);
  /* If this test passes, it detected the expected max_error. */
  RunCharacterization(query);
  /* If this test passes, it did all the required work. */
  EXPECT_EQ(DummyCallbackData<double>::get_invocations(), call_count);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
