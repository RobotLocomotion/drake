#pragma once

/* @file A collection of utilities to facilitate proximity query testing. The
 functionality specifically targets proximity queries that are signed distance
 (or equivalent to signed distance). It allows for compact representation of
 test scenarios with outcome expectations and provides the common code to
 exercise the various proximity queries. It is the expectation that these
 utilities will be exercised by each set of *callback* unit tests, providing
 correctness coverage and characterization of the query's "quality".  */

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <fcl/fcl.h>
#include <gtest/gtest.h>

#include "drake/common/default_scalars.h"
#include "drake/common/unused.h"
#include "drake/geometry/proximity/collision_filter.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

/* The base class defining a self-contained wrapper for distance-related
 callbacks. To test a single callback, this class should be derived and
 implemented with the details of the particular callback and its callback data
 hidden as implementation details.  */
template <typename T>
class DistanceCallback {
 public:
  virtual ~DistanceCallback() = default;

  /* Invokes the underlying callback for the given data. This invocation will
   append any reports to the persistent set of stored results. The caller is
   responsible for calling ClearResults() if it wants the results of this
   invocation to be distinct from other invocations. */
  virtual bool Invoke(
      fcl::CollisionObjectd*, fcl::CollisionObjectd*,
      const CollisionFilter*,
      const std::unordered_map<GeometryId, math::RigidTransform<T>>*) = 0;

  /* Forces all results to be cleared. */
  virtual void ClearResults() = 0;

  /* Reports the number of results generated by the last call to Invoke(). */
  virtual int GetNumResults() const = 0;

  /* Reports the distance contained in the first result.
   @pre GetNumResults() > 0. */
  virtual T GetFirstSignedDistance() const = 0;
};

/* Encodes the possible outcomes of performing a proximity query. For a given
 set of query parameters/scalar attempting to perform the query should either
 work or not in some enumerable fashion. */
enum Outcome { kSupported, kThrows, kIgnores };

/* The types of geometries that can be considered for characterization tests.
 Note: the geometries enumerated here includes a Point (which is not a
 drake::Shape). */
enum GeometryType {
  kBox,
  kCapsule,
  kConvex,
  kCylinder,
  kEllipsoid,
  kHalfSpace,
  kMesh,
  kPoint,
  kSphere
};
std::ostream& operator<<(std::ostream& out, GeometryType s);

/* This represents a single cell of the table -- an instance of invoking a
 proximity query. It explicitly calls out the two shapes and the expected
 outcome, and relies on the context to define the scalar. */
struct QueryInstance {
  /* Constructs a query instance that should successfully evaluate, producing a
   result with the given error. */
  QueryInstance(GeometryType shape1, GeometryType shape2, double error);

  /* Constructs a query instance that doesn't successfully evaluate -- the test
   may throw or silently do nothing as indicated by the declared `outcome`.
   @pre outcome != kSupported */
  QueryInstance(GeometryType shape1, GeometryType shape2, Outcome outcome);

  const GeometryType shape1{};
  const GeometryType shape2{};
  const double error{};
  const Outcome outcome{};
};

std::ostream& operator<<(std::ostream& out, const QueryInstance& c);

/* Function to convert QueryInstance values into meaningful value test names
 for gtest.  */
std::string QueryInstanceName(
    const testing::TestParamInfo<QueryInstance>& info);

/* Defines a test configuration for two shapes: the pose between the two
 shapes, the expected signed_distance, and a description to aid in assessing
 test failure. */
template <typename T>
struct Configuration {
  math::RigidTransform<T> X_AB;
  double signed_distance{};
  std::string description;
};

/* The challenge of testing signed distance (and related) queries, is to put
 two arbitrary shapes into a non-trivial relative pose which, nevertheless,
 should produce an unambiguous distance value. All of the following code
 supports that effort. See specific notes below.  */

/* Defines a tangent plane on the surface of a shape (in whatever frame the
 shape itself is expressed in). The given `point` lies on both the plane and
 the surface of the shape. The normal should point *out* of the shape such that
 the shape is "under" the plane (i.e., the signed distance to the plane is <= 0
 for all points in the shape). The normal direction may not be unique (e.g., for
 a box's vertex, the associated normal is any direction in the corresponding
 octant).

 The tangent plane contains one further piece of information. Given the tangent
 plane defined by point P and normal n, we define a set of points
 Q = P - δ⋅n (for δ ≥ 0). We report a `max_depth` value -- the largest value of
 δ such that we can guarantee that P is the closest point on the surface of the
 geometry to all of the Qs for δ ∈ [0, δₘₐₓ]. This will inform us as to which
 samples can be combined into a valid configuration. To be a valid configuration
 for penetration, at least *one* of the samples needs to have a `max_depth`
 value that is greater than or equal to the targeted depth.

 We use the TangentPlane to pose geometries; see AlignTangentPlanes for
 details.  */
template <typename T>
struct ShapeTangentPlane {
  Vector3<T> point;
  Vector3<T> normal;
  double max_depth{};
  std::string description;
};

/* Creates a set of "interesting" samples (each a tangent plane) on the given
 geometry.

 The samples we return must be able to support an expected signed distance
 query result. In some cases, the sign and magnitude of the targeted distance
 can change what the samples are. See the various implementations for details.
 */
template <typename T>
class ShapeConfigurations : public ShapeReifier {
 public:
  /* Constructs the configurations for the given shape and requested distance.
   */
  ShapeConfigurations(const Shape& shape, double distance);

  /* @name Implementation of ShapeReifier interface

   Where meaningful, each of these implementations will test the geometry
   against *negative* signed distance to confirm the shape is large enough to be
   able to manifest penetration depth of the requested magnitude. If the test
   fails, the method throws.  */
  //@{

  using ShapeReifier::ImplementGeometry;

  /* Sample a face near a vertex, a vertex, and on an edge. */
  void ImplementGeometry(const Box& box, void*) final;

  /* The capsule samples do *not* depend on `signed_distance`. One is located on
   one of the spherical caps, the other on the barrel.  */
  void ImplementGeometry(const Capsule& capsule, void*) final;

  /* For the purpose of *this* test, the Convex is actually just a box. Its
   sampling is that of the Box's sampling (see above).  */
  void ImplementGeometry(const Convex& convex, void*) final;

  /* Sample the cylinder on one cap face (near its edge), one sample on a
   circular edge, and one on the barrel. The sample on the top face is
   guaranteed to be positioned so that it is closest to all points up to the
   requested penetration depth (assuming the cylinder is large enough).  */
  void ImplementGeometry(const Cylinder& cylinder, void*) final;

  /* The sampling of the ellipsoid does *not* depend on `signed_distance`.
   The samples are simply two arbitrary points on the surface of the ellipsoid
   in different octants.  */
  void ImplementGeometry(const Ellipsoid& ellipsoid, void*) final;

  /* The sampling of the half space does *not* depend on `signed_distance`.
   The sample is at a single point near the origin. We keep it near the origin
   so that precision problems don't get exacerbated by distance-to-origin
   issues, but not *at* the origin so we don't get trivial zeros.  */
  void ImplementGeometry(const HalfSpace& half_space, void*) final;

  /* Simply throws; we'll defer mesh computations until they are supported
   directly (instead of being represented by a Convex.  */
  void ImplementGeometry(const Mesh& mesh, void*) final;

  /* The samples are at two arbitrary points in different octants of the sphere.
   */
  void ImplementGeometry(const Sphere& sphere, void*) final;
  //@}

  /* We're returning a *copy* because we're using this in a manner in which the
   owning reifier may not stay alive. */
  std::vector<ShapeTangentPlane<T>> configs() const { return configs_; }

 private:
  double distance_{};
  std::vector<ShapeTangentPlane<T>> configs_;
};

/* @name Fcl geometry from drake shape specifications. */
class MakeFclShape : public ShapeReifier {
 public:
  explicit MakeFclShape(const Shape& shape);

  /* Implementation of ShapeReifier interface  */
  using ShapeReifier::ImplementGeometry;
  void ImplementGeometry(const Box& box, void*) final;
  void ImplementGeometry(const Capsule& capsule, void*) final;
  void ImplementGeometry(const Convex& convex, void*) final;
  void ImplementGeometry(const Cylinder& cylinder, void*) final;
  void ImplementGeometry(const Ellipsoid& ellipsoid, void*) final;
  void ImplementGeometry(const HalfSpace& half_space, void*) final;
  void ImplementGeometry(const Mesh& mesh, void*) final;
  void ImplementGeometry(const Sphere& sphere, void*) final;

  std::shared_ptr<fcl::CollisionGeometry<double>> object() const {
    return object_;
  }

 private:
  std::shared_ptr<fcl::CollisionGeometry<double>> object_{};
};

/* Reports if the Mesh shape is represented as a Convex shape under the hood. */
::testing::AssertionResult MeshIsConvex();

/* Creates a transform to align two planes. The planes are defined by a
 (point, normal) pair -- the point lies on the plane and the normal is
 perpendicular to the plane. In this case, we have planes (P, m⃗) and (Q, n⃗). We
 produce a transform X = [R t] such that:
   X ⋅ Q = P
   R ⋅ n⃗ = -m⃗
In other words, we transform the second plane (Q, n⃗) so Q and P are coincident
and m⃗ and n⃗ are anti-parallel. The notation is frameless, because we're
not really relating two frames so much as creating a transform operator
(although we *do* assume that all quantities are measured and expressed in a
common frame).

@tparam_nonsymbolic_scalar */
template <typename T>
math::RigidTransform<T> AlignPlanes(const Vector3<T>& P, const Vector3<T>& m,
                                    const Vector3<T>& Q, const Vector3<T>& n);

/* This test provides a common framework for testing signed-distance based
 proximity queries. Specifically, it provides *direct* support in populating
 the scalar support/precision table. It indirectly provides the basis for
 showing general mathematical correctness of the query's underlying math.

 This test supports the documented support table in a very specific way. We want
 these characterization tests to detect regression in underlying code, but also
 to detect improvements in the code under test so that the table can be
 updated. When a test fails, it reports whether the expected precision (as
 declared in the specific test) is too high or too low and direct the developer
 to resolve it and update the table in query_object.h as appropriate.

 The tests are formulated to collide two shapes in various configurations. None
 of the configurations are *trivial* -- we want to avoid the case where the
 algorithm can "luck" in to a high precision solution. Instead, we want to test
 it for some vague "typical" case. So, we have no identity poses and non-zero
 translations. The measures of the geometry are values that are not perfectly
 represented in binary (e.g., prefer 1/3 over 1/2). */
template <typename T>
class CharacterizeResultTest : public ::testing::Test {
 public:
  /* Constructs an instance of the test for the given callback implementation.
   */
  explicit CharacterizeResultTest(std::unique_ptr<DistanceCallback<T>> callback)
      : callback_(std::move(callback)) {}

  /* Runs a single instance of the callback for the given collision objects.
   Confirms the callback result against the expected outcome. If query.outcome
   is kSupported, it confirms the successful execution of the query reports a
   single result. Otherwise, confirms that the callback throws with a common
   "unsupported operation" type exception message. */
  void RunCallback(
      const QueryInstance& query, fcl::CollisionObjectd* obj_A,
      fcl::CollisionObjectd* obj_B,
      const CollisionFilter* collision_filter,
      const std::unordered_map<GeometryId, math::RigidTransform<T>>* X_WGs)
      const;

  /* Computes the *absolute* error of the result produced by the last invocation
   of the callback against the given `expected_distance`. If there are no
   results reported, no error is returned.
   @pre expected_distance != 0. */
  std::optional<double> ComputeErrorMaybe(double expected_distance) const;

  // TODO(SeanCurtis-TRI) I don't need a bunch of configurations if my
  //  expectation is that it will fail. So, I should use that information to
  //  reduce the work I do.
  /* Creates a set of configurations for the two shapes based on the surface
   sample points defined for each shape type (see SampleShapeSurface below). */
  virtual std::vector<Configuration<T>> MakeConfigurations(
      const Shape& shape_A, const Shape& shape_B,
      const std::vector<double>& signed_distances) const;

  /* Runs the test to characterize the query error between geometries of
   two shape types: Shape1 and Shape2. The caller provides one or more
   configuration specifications (including relative poses of the two shapes
   X_AB and the expected signed distance that pose would produce).

   This method transforms the two shapes into the world frame via an arbitrary
   transform X_WA (see corresponding static method). This guarantees that
   we're testing the query under non-trivial transforms. The query is repeated
   twice: once as (A, B) and again as (B, A) for each configuration.

   The expected outcome of the query is contained in `query.outcome`. It
   declares whether the query expects to compute (`kSuported`). If it is
   supported, error is computed between the returned signed distance and the
   expected signed distance. Across *all* queries, the worst error is stored.
   The worst error can't be too much worse or too much *better* than the
   expected value (`query.error`) to pass. If the expectation is
   that the query cannot be computed, we confirm an exception based on a
   standard "unsupported" message used throughout the callbacks.

   This test does most of the work to run the test (accumulating and testing
   worst error), but it uses `RunCallback` to execute the callback and assert
   query expectations and `ComputeErrorMaybe` to compute reportable error.

   The test passes if the worst error lies in the interval:

                |    error    |
                |-------------|     e = query.error
                |             |
               e/4            e

   This gives us a reasonably tight bound on the error.

   If the expected maximum error is "close" to epsilon, we test it against the
   *open* interval:

                    error    |
                -------------|     e = query.error
                             |
                             e

   If the expected error is already functionally epsilon, than having the worst
   error being smaller than epsilon/4 is no bad thing.

   N.B. This testing strategy is *atypical*. Rather than just asserting the
   answer is no worse than an expected level of precision, we also want to
   confirm that it is no better (the lower bound on the interval). The purpose
   of this test is to detect if the quality of the query improves so that we can
   update the table in query_object.h. Therefore, we want to detect if the
   answers get "too good" and the table in the documentation needs updating.

   It is the responsibility of the caller to define poses X_AB that truly tax
   the underlying code. There are configuration where even badly written code
   can get the correct answer to within machine epsilon. We want to report the
   *worst case* error and the tests should be articulated as such.

   @param query        The query instance (which contains the expected error).
   @param shape_A      The first shape in the pair.
   @param shape_B      The second shape in the pair.
   @param configs      A collection of test configurations against which we
                       evaluate the callback. */
  void RunCharacterization(const QueryInstance& query, const Shape& shape_A,
                           const Shape& shape_B,
                           const std::vector<Configuration<T>>& configs,
                           bool is_symmetric = true);

  /* Evaluates the query instance. */
  void RunCharacterization(const QueryInstance& query,
                           bool is_symmetric = true);

  /* Each subclass should define the distances over which it should be
   evaluated. */
  virtual std::vector<double> TestDistances() const = 0;

  /* Generates a geometry id for the given collision object, encodes the id
   into the object, and returns the id.  */
  GeometryId EncodeData(fcl::CollisionObjectd* obj);

  /* Returns a collection of transforms to map an experiment expressed in a
   shape A's frame A, to the world frame. The transforms are arbitrary but
   constructed such that the transforms are non-trivial. This is to make sure we
   don't exhibit high precision due to an overly simplified scenario. */
  static std::vector<math::RigidTransform<T>> X_WAs();

  /* @name A consistent set of shapes to use across all tests

   Each method can produce one of two instances of the shape (for doing, e.g.,
   Box-Box tests with two different proportions).

   As documented in query_object.h, at least one measure of each geometry should
   be approximately 20 cm. The other measures (based on shape parameterization)
   have been arbitrarily selected to avoid symmetry and perfectly represented
   numerical values. */
  //@{

  /* As documented in query_object.h, the 2mm distance/depth for which the
   queries are characterized. */
  static constexpr double kDistance{2e-3};

  static std::unique_ptr<Shape> MakeShape(GeometryType shape, bool use_alt);

  static Box box(bool alt = false);

  static Capsule capsule(bool alt = false);

  static Convex convex(bool alt = false);

  static Cylinder cylinder(bool alt = false);

  static Ellipsoid ellipsoid(bool alt = false);

  static HalfSpace half_space(bool alt = false);

  static Mesh mesh(bool alt = false);

  static Sphere sphere(bool alt = false);

  //}

 protected:
  CollisionFilter collision_filter_;
  std::unique_ptr<DistanceCallback<T>> callback_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
  class ::drake::geometry::internal::CharacterizeResultTest)
DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
  class ::drake::geometry::internal::ShapeConfigurations)
