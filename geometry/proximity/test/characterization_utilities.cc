#include "drake/geometry/proximity/test/characterization_utilities.h"

#include <algorithm>
#include <fstream>
#include <limits>

#include <fmt/format.h>

#include "drake/common/nice_type_name.h"
#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity/proximity_utilities.h"
#include "drake/geometry/proximity_engine.h"
#include "drake/geometry/utilities.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace geometry {
namespace internal {

using Eigen::Vector3d;
using math::RigidTransform;
using std::vector;

std::ostream& operator<<(std::ostream& out, GeometryType s) {
  switch (s) {
    case kBox:
      out << "Box";
      break;
    case kCapsule:
      out << "Capsule";
      break;
    case kConvex:
      out << "Convex";
      break;
    case kCylinder:
      out << "Cylinder";
      break;
    case kEllipsoid:
      out << "Ellipsoid";
      break;
    case kHalfSpace:
      out << "HalfSpace";
      break;
    case kMesh:
      out << "Mesh";
      break;
    case kPoint:
      out << "Point";
      break;
    case kSphere:
      out << "Sphere";
      break;
  }
  return out;
}

QueryInstance::QueryInstance(GeometryType shape1_in, GeometryType shape2_in,
                             double error_in)
    : shape1(shape1_in),
      shape2(shape2_in),
      error(error_in),
      outcome(kSupported) {
  DRAKE_DEMAND(error >= 0);
}

QueryInstance::QueryInstance(GeometryType shape1_in, GeometryType shape2_in,
                             Outcome outcome_in)
    : shape1(shape1_in), shape2(shape2_in), error(-1), outcome(outcome_in) {
  DRAKE_DEMAND(outcome != kSupported);
}

std::ostream& operator<<(std::ostream& out, const QueryInstance& c) {
  out << c.shape1 << " vs " << c.shape2;
  switch (c.outcome) {
    case kSupported:
      out << " with expected error: " << c.error;
      break;
    case kThrows:
      out << " should throw";
      break;
    case kIgnores:
      out << " should be ignored";
      break;
  }
  return out;
}

std::string QueryInstanceName(
    const testing::TestParamInfo<QueryInstance>& info) {
  return fmt::format("{}Vs{}", info.param.shape1, info.param.shape2);
}

template <typename T>
ShapeConfigurations<T>::ShapeConfigurations(const Shape& shape, double distance)
    : ShapeReifier(), distance_(distance) {
  shape.Reify(this);
}

template <typename T>
void ShapeConfigurations<T>::ImplementGeometry(const Box& box, void*) {
  const Vector3d half_size = box.size() * 0.5;
  // By default, we'll position the face point near the vertex, but removed
  // based on the box's minimum measurement.
  double edge_distance = half_size.minCoeff() * 0.25;
  if (distance_ < 0) {
    if (-distance_ > half_size.minCoeff()) {
      throw std::runtime_error(
          fmt::format("The box (with dimensions {}) isn't large enough for "
                      "penetration depth of {}",
                      box.size(), -distance_));
    }
    // If we actually need penetration, we need to make sure that the distance
    // to the edges is *larger* than the requested depth (plus small padding).
    edge_distance = std::max(edge_distance, -distance_) * 1.01;
  }

  // Note: neither the point on the corner nor on the edge are the closest point
  // for *any* point on the inside of the box. The point on the face is closest
  // up to the recorded edge_distance.
  configs_ = vector<ShapeTangentPlane<T>>{
      {half_size.cwiseProduct(Vector3d{1, -1, 1}),
       Vector3d{0.5, -2, 1}.normalized(), 0, "+x/-y/+z vertex"},
      {half_size.cwiseProduct(Vector3d{1, -1, 0.75}),
       Vector3d{2, -2, 0}.normalized(), 0, "+x/-y edge parallel to z axis"},
      {-half_size + Vector3d{edge_distance, edge_distance, 0},
       Vector3d{0, 0, -1}, edge_distance, "-z face near the -x/-y vertex"}};
}

template <typename T>
void ShapeConfigurations<T>::ImplementGeometry(const Capsule& capsule, void*) {
  if (distance_ < 0) {
    const double depth = -distance_;
    if (depth > capsule.radius()) {
      throw std::runtime_error(
          fmt::format("The capsule (with radius {}) isn't large enough for "
                      "penetration depth of {}",
                      capsule.radius(), depth));
    }
  }
  /* We'll arbitrarily return one point on a spherical cap, and one on the
   barrel. */
  const double barrel_theta = 6 * M_PI / 7;
  using std::cos;
  using std::sin;
  const Vector3<T> n_barrel_C{cos(barrel_theta), sin(barrel_theta), 0};
  const Vector3<T> p_CB{n_barrel_C[0] * capsule.radius(),
                        n_barrel_C[1] * capsule.radius(),
                        capsule.length() * 0.3};

  const Vector3<T> n_sphere_C = Vector3<T>{2, 0.3, 1.1}.normalized();
  const Vector3<T> p_CS =
      Vector3<T>{0, 0, capsule.length() / 2} + n_sphere_C * capsule.radius();
  configs_ = vector<ShapeTangentPlane<T>>{
      {p_CB, n_barrel_C, capsule.radius(), "Capsule barrel"},
      {p_CS, n_sphere_C, capsule.radius(), "Capsule +z cap"}};
}

template <typename T>
void ShapeConfigurations<T>::ImplementGeometry(const Convex&, void*) {
  const Box box = CharacterizeResultTest<double>::box();
  ImplementGeometry(box, nullptr);
}

template <typename T>
void ShapeConfigurations<T>::ImplementGeometry(const Cylinder& cylinder,
                                               void*) {
  double distance_to_edge = cylinder.radius() * 0.1;
  if (distance_ < 0) {
    const double depth = -distance_;
    if (depth > cylinder.radius() || depth > cylinder.length() / 2) {
      throw std::runtime_error(
          fmt::format("The cylinder (with radius = {} and length = {}) isn't "
                      "large enough for penetration depth of {}",
                      cylinder.radius(), cylinder.length(), depth));
    }
    distance_to_edge = 1.01 * depth;
  }

  // Point on +z cylinder cap, with +z normal.
  // Arbitrary direction on the cylinder cap's plane.
  const Vector3<T> dir = Vector3<T>{1.3, -0.25, 0}.normalized();
  const T dist = cylinder.radius() - distance_to_edge;
  const Vector3<T> Cz{0, 0, 1};
  const Vector3<T> p_CP = dir * dist + Cz * (cylinder.length() / 2);
  configs_.push_back({p_CP, Cz, distance_to_edge, "cylinder's +z cap"});

  // Point on edge, with normal pointing outward.
  const Vector3<T> p_CE =
      Vector3<T>{1, 1.5, 0}.normalized() * cylinder.radius() +
      Vector3<T>{0, 0, cylinder.length() / 2};
  configs_.push_back(
      {p_CE, Vector3<T>{1, 1.5, 1}.normalized(), 0, "cylinder's +z edge"});

  // Point on barrel -- must be far enough away from the edge to support
  // any penetration depth.
  const Vector3<T> n_A = Vector3<T>{-1.5, 3, 0}.normalized();
  const Vector3<T> p_AB =
      n_A * cylinder.radius() +
      Vector3<T>{0, 0, -cylinder.length() / 2 + distance_to_edge};
  configs_.push_back(
      {p_AB, n_A, cylinder.radius(), "the bottom half of cylinder's barrel"});
}

template <typename T>
void ShapeConfigurations<T>::ImplementGeometry(const Ellipsoid& ellipsoid,
                                               void*) {
  const double min_axis =
      std::min({ellipsoid.a(), ellipsoid.b(), ellipsoid.c()});
  if (distance_ < 0) {
    const double depth = -distance_;
    if (depth > min_axis) {
      throw std::runtime_error(
          fmt::format("The ellipsoid (with radii = {}, {}, and {}) isn't "
                      "large enough for penetration depth of {}",
                      ellipsoid.a(), ellipsoid.b(), ellipsoid.c(), depth));
    }
  }

  auto uv_sample = [&ellipsoid](double u, double v) {
    // Compute a point on the surface of the ellipsoid and the normal at
    // that point. For the point, we use the parametric equation of the
    // ellipsoid (on two periodic parameters):
    //    E = [a⋅cos(u)⋅sin(v), b⋅sin(u)⋅sin(v), c⋅cos(v)].
    // For the normal, we normalize the gradient of
    //    f = x² / a² + y² / b² + z² / c²
    //    ∇f = <2x / a², 2y / b², 2z / c²>
    //    n = ∇f / |∇f|
    const double a = ellipsoid.a();
    const double b = ellipsoid.b();
    const double c = ellipsoid.c();
    const double x = a * std::cos(u) * std::sin(v);
    const double y = b * std::sin(u) * std::sin(v);
    const double z = c * std::cos(v);
    // Because we're normalizing the gradient, we simply omit the redundant
    // scale factor of 2.
    return std::make_pair(
        Vector3d{x, y, z},
        Vector3d{x / a / a, y / b / b, z / c / c}.normalized());
  };

  const auto& [p1, n1] = uv_sample(M_PI / 3, M_PI / 7);
  const auto& [p2, n2] = uv_sample(13 * M_PI / 7, 5 * M_PI / 6);
  configs_ = vector<ShapeTangentPlane<T>>{
      {p1, n1, min_axis, "ellipsoid's -x/+y/+z octant"},
      {p2, n2, min_axis, "ellipsoid's +x/-y/+z octant"}};
}

template <typename T>
void ShapeConfigurations<T>::ImplementGeometry(const HalfSpace&, void*) {
  configs_ = vector<ShapeTangentPlane<T>>{
      {Vector3<T>{0.3, 0.7, 0}, Vector3<T>{0, 0, 1},
       std::numeric_limits<double>::infinity(), "near half space's origin"}};
}

template <typename T>
void ShapeConfigurations<T>::ImplementGeometry(const Mesh&, void*) {
  throw std::logic_error(
      "We're assuming that Mesh is Convex; implement this when Mesh is "
      "represented as its own thing");
}

template <typename T>
void ShapeConfigurations<T>::ImplementGeometry(const Sphere& sphere, void*) {
  const double r = sphere.radius();
  if (distance_ < 0) {
    const double depth = -distance_;
    if (depth > r) {
      throw std::runtime_error(
          fmt::format("The sphere (with radius = {}) isn't large enough for "
                      "penetration depth of {}",
                      r, depth));
    }
  }
  /* A couple of random points in arbitrary directions. */
  const Vector3<T> n1 = Vector3<T>{-1, 1, 1.5}.normalized();
  const Vector3<T> p1 = n1 * r;
  const Vector3<T> n2 = Vector3<T>{1, -2, 0.3}.normalized();
  const Vector3<T> p2 = n2 * r;
  configs_ =
      vector<ShapeTangentPlane<T>>{{p1, n1, r, "sphere's -x/+y/+z octant"},
                                   {p2, n2, r, "sphere's +x/-y/+z octant"}};
}

MakeFclShape::MakeFclShape(const Shape& shape) : ShapeReifier() {
  shape.Reify(this);
}

void MakeFclShape::ImplementGeometry(const Box& box, void*) {
  object_ = std::make_shared<fcl::Boxd>(box.size());
}

void MakeFclShape::ImplementGeometry(const Capsule& capsule, void*) {
  object_ = std::make_shared<fcl::Capsuled>(capsule.radius(), capsule.length());
}

void MakeFclShape::ImplementGeometry(const Convex&, void*) {
  /* Note: we're ignoring the contents of the convex declaration. Instead,
    we're outputting a mesh representing a box with known dimensions.

                   3 ──────────────────┐ 7
                   ╱│                 ╱│
                  ╱ │                ╱ │             z    y
                 ╱  │             5 ╱  │              │  ╱
              1 ┌──────────────────┐   │              │ ╱
                │   │              │   │              │╱
                │   │              │   │              └───── x
                │  2│──────────────│───│ 6
                │  ╱               │  ╱
                │ ╱                │ ╱
                │╱                 │╱
                └──────────────────┘
                0                  4
  */
  const Box box = CharacterizeResultTest<double>::box();
  const Vector3d half_size = box.size() / 2;
  auto vertices = std::make_shared<vector<Vector3d>>();
  for (double x : {-1, 1}) {
    for (double y : {-1, 1}) {
      for (double z : {-1, 1}) {
        vertices->push_back(half_size.cwiseProduct(Vector3d{x, y, z}));
      }
    }
  }
  // clang-format off
    auto faces = std::make_shared<vector<int>>(vector<int>{
      4, 0, 4, 5, 1,   // -y face
      4, 5, 7, 3, 1,   // +z face
      4, 3, 7, 6, 2,   // +y face
      4, 0, 2, 6, 4,   // -z face
      4, 4, 6, 7, 5,   // +x face
      4, 1, 3, 2, 0    // -x face.
    });
  // clang-format on

  object_ = std::make_shared<fcl::Convexd>(vertices, 6, faces);
}

void MakeFclShape::ImplementGeometry(const Cylinder& cylinder, void*) {
  object_ =
      std::make_shared<fcl::Cylinderd>(cylinder.radius(), cylinder.length());
}

void MakeFclShape::ImplementGeometry(const Ellipsoid& ellipsoid, void*) {
  object_ = std::make_shared<fcl::Ellipsoidd>(ellipsoid.a(), ellipsoid.b(),
                                              ellipsoid.c());
}

void MakeFclShape::ImplementGeometry(const HalfSpace&, void*) {
  object_ = std::make_shared<fcl::Halfspaced>(Vector3d{0, 0, 1}, 0);
}

void MakeFclShape::ImplementGeometry(const Mesh&, void*) {
  throw std::logic_error(
      "We're assuming that Mesh is Convex; implement this when Mesh is "
      "represented as its own thing");
}

void MakeFclShape::ImplementGeometry(const Sphere& sphere, void*) {
  object_ = std::make_shared<fcl::Sphered>(sphere.radius());
}

/* Allow a sneak peek into ProximityEngine's inner workings so we can detect
 when a Mesh is no longer represented under the hood as a Convex.  */
class ProximityEngineTester {
 public:
  template <typename T>
  static bool IsFclConvexType(const ProximityEngine<T>& engine, GeometryId id) {
    return engine.IsFclConvexType(id);
  }
};


::testing::AssertionResult MeshIsConvex() {
  // Create a small obj in a temp directory.
  const std::string obj_path = temp_directory() + "/tri.obj";
  {
    std::ofstream out(obj_path);
    if (out.bad()) {
      throw std::runtime_error(
          fmt::format("Unable to write temporary obj file to: {}", obj_path));
    }
    out << "# Simple triangle for testing purposes\n"
        << "v 0 0 0\n"
        << "v 1 0 0\n"
        << "v 0 1 0\n"
        << "f 1 2 3\n";
    out.close();
  }

  // Add the mesh into a proximity engine and confirm it is represented by a
  // fcl::Convex.
  ProximityEngine<double> engine;
  const GeometryId id = GeometryId::get_new_id();
  engine.AddDynamicGeometry(Convex(obj_path, 1.0), {}, id);
  if (ProximityEngineTester::IsFclConvexType(engine, id)) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure()
         << "A Mesh shape is no longer represented by an fcl::Convex in "
         << "ProximityEngine. We need to explicitly characterize Mesh-Shape "
         << "queries for all Shape types.";
}

template <typename T>
RigidTransform<T> AlignPlanes(const Vector3<T>& P, const Vector3<T>& m,
                              const Vector3<T>& Q, const Vector3<T>& n) {
  /* For notation, we'll assume |n| = |m| = 1 (in the code we'll enforce it).

  We find rotation R such that -m = R⋅n.
  The angle between -m and n is simply acos(n.dot(-m)).
  The vector around which to rotate n is given by their cross product. */
  const Vector3<T> neg_mhat = -m.normalized();
  const Vector3<T> nhat = n.normalized();
  const T cos_theta = nhat.dot(neg_mhat);

  /* Default to identity if n and m are already aligned. */
  math::RotationMatrix<T> R;
  constexpr double kAlmostOne = 1 - std::numeric_limits<double>::epsilon();
  if (cos_theta < kAlmostOne) {
    /* They aren't already anti-parallel. */
    if (cos_theta < -kAlmostOne) {
      /* We need a normal perpendicular to nhat. Extract it from a valid
       basis. */
      const math::RotationMatrix<T> basis =
          math::RotationMatrix<T>::MakeFromOneVector(nhat, 2);
      const Vector3<T> rhat = basis.col(0);
      R = math::RotationMatrix<T>(AngleAxis<T>{M_PI, rhat});
    } else {
      const Vector3<T> rhat = nhat.cross(neg_mhat).normalized();
      using std::acos;
      R = math::RotationMatrix<T>(AngleAxis<T>{acos(cos_theta), rhat});
      if ((R * nhat).dot(neg_mhat) < kAlmostOne) {
        /* Detect if we rotated in the wrong direction. By construction this
         shouldn't happen. The direction of rhat will always make use of the
         *positive* angle that we computed with acos.  */
        DRAKE_UNREACHABLE();
      }
    }
  }

  const Vector3<T> p_QP_A = P - R * Q;
  return RigidTransform<T>{R, p_QP_A};
}

template <typename T>
void CharacterizeResultTest<T>::RunCallback(
    const QueryInstance& query, fcl::CollisionObjectd* obj_A,
    fcl::CollisionObjectd* obj_B, const CollisionFilter* collision_filter,
    const std::unordered_map<GeometryId, RigidTransform<T>>* X_WGs)
    const {
  callback_->ClearResults();
  ASSERT_EQ(callback_->GetNumResults(), 0);
  switch (query.outcome) {
    case kSupported:
    case kIgnores:
      ASSERT_FALSE(callback_->Invoke(obj_A, obj_B, collision_filter, X_WGs));
      ASSERT_EQ(callback_->GetNumResults(),
                query.outcome == kSupported ? 1 : 0);
      break;
    case kThrows:
      DRAKE_ASSERT_THROWS_MESSAGE(
          callback_->Invoke(obj_A, obj_B, collision_filter, X_WGs),
          ".+ queries between shapes .+ and .+ are not supported.+");
      break;
  }
}

template <typename T>
std::optional<double> CharacterizeResultTest<T>::ComputeErrorMaybe(
    double expected_distance) const {
  if (callback_->GetNumResults() > 0) {
    const double distance =
        ExtractDoubleOrThrow(callback_->GetFirstSignedDistance());
    DRAKE_DEMAND(std::abs(expected_distance) >
                 std::numeric_limits<double>::epsilon());
    return std::abs(distance - expected_distance);
  }
  return std::nullopt;
}

template <typename T>
vector<Configuration<T>> CharacterizeResultTest<T>::MakeConfigurations(
    const Shape& shape_A, const Shape& shape_B,
    const vector<double>& signed_distances) const {
  vector<Configuration<T>> configs;
  for (const double signed_distance : signed_distances) {
    /* In order to pose the geometries, we start with X_WA = X_WB = I. That
     means all quantities measured and expressed in frames A and B, are
     likewise measured and expressed in the world frame. The notation below
     reflects this. */
    const vector<ShapeTangentPlane<T>> samples_A =
        ShapeConfigurations<T>(shape_A, signed_distance).configs();
    const vector<ShapeTangentPlane<T>> samples_B =
        ShapeConfigurations<T>(shape_B, signed_distance).configs();
    for (const auto& sample_A : samples_A) {
      const Vector3<T>& p_WA = sample_A.point;
      const Vector3<T>& a_norm_W = sample_A.normal;
      /* The witness point. When penetrating, signed_distance is negative and
       we offset from the surface *into* the shape (via the negatively scaled
       normal). When separating, we move in the positive direction.  */
      const Vector3<T> p_WC = p_WA + a_norm_W * signed_distance;
      for (const auto& sample_B : samples_B) {
        /* We can only combine these two samples if at least *one* of them
         can accommodate the requested penetration depth. Note that the sample
         record the value of the maximum *depth* (negative of signed distance).
         */
        using std::max;
        if (max(sample_A.max_depth, sample_B.max_depth) < -signed_distance) {
          continue;
        }
        const Vector3<T>& p_WB = sample_B.point;
        const Vector3<T>& b_norm_W = sample_B.normal;
        const std::string relates =
            signed_distance < 0
                ? " penetrates into "
                : (signed_distance > 0 ? " separated from " : " touching ");
        configs.push_back(
            {AlignPlanes(p_WC, a_norm_W, p_WB, b_norm_W), signed_distance,
             sample_B.description + relates + sample_A.description});
      }
    }
  }
  return configs;
}

template <typename T>
void CharacterizeResultTest<T>::RunCharacterization(
    const QueryInstance& query, const Shape& shape_A, const Shape& shape_B,
    const vector<Configuration<T>>& configs, bool is_symmetric) {
  fcl::CollisionObjectd object_A = MakeFclShape(shape_A).object();
  const GeometryId id_A = EncodeData(&object_A);

  fcl::CollisionObjectd object_B = MakeFclShape(shape_B).object();
  const GeometryId id_B = EncodeData(&object_B);

  const auto& X_WAs = this->X_WAs();
  std::optional<double> worst_error;
  std::optional<Configuration<T>> worst_config;

  auto evaluate_callback =
      [this, &query, &worst_error, &worst_config](
          char first, fcl::CollisionObjectd* obj_A, char second,
          fcl::CollisionObjectd* obj_B, auto test_config,
          const std::unordered_map<GeometryId, RigidTransform<T>>&
              world_poses) {
        SCOPED_TRACE(fmt::format(
            "\n{}-{} (obj {}-obj {}) query:"
            "\n  {}"
            "\n  Expected signed distance: {}"
            "\n  X_AB"
            "\n{}\n",
            GetGeometryName(*obj_A), GetGeometryName(*obj_B), first, second,
            test_config.description, test_config.signed_distance,
            test_config.X_AB.GetAsMatrix34()));
        RunCallback(query, obj_A, obj_B, &collision_filter_, &world_poses);
        const std::optional<double> error =
            ComputeErrorMaybe(test_config.signed_distance);
        if (error.has_value() &&
            (!worst_error.has_value() || *error > *worst_error)) {
          worst_error = error;
          worst_config = test_config;
        }
      };

  for (const auto& X_WA : X_WAs) {
    for (const auto& config : configs) {
      const std::unordered_map<GeometryId, RigidTransform<T>> X_WGs{
          {{id_A, X_WA}, {id_B, X_WA * config.X_AB}}};
      object_A.setTransform(convert_to_double(X_WGs.at(id_A)).GetAsIsometry3());
      object_B.setTransform(convert_to_double(X_WGs.at(id_B)).GetAsIsometry3());

      evaluate_callback('A', &object_A, 'B', &object_B, config, X_WGs);
      if (is_symmetric) {
        evaluate_callback('B', &object_B, 'A', &object_A, config, X_WGs);
      }
    }
  }

  /* Last reality check. If the query was expected to be supported, we better
   report *some* value as an error, even if it's zero. */
  ASSERT_EQ(query.outcome == kSupported, worst_error.has_value());
  /* The only way worst_error has no value, is that we expected *every*
   configuration to fail. Then no value is the correct outcome. */
  if (worst_error.has_value()) {
    /* Our definition of "close" to epsilon: within 2 bits. Still a tight
     bound but gives a modicum of breathing room. */
    constexpr double cutoff = 4 * std::numeric_limits<double>::epsilon();
    if (query.error > cutoff) {
      EXPECT_GT(*worst_error, query.error / 4)
          << "Expected error is too big!"
          << "\n  " << worst_config->description
          << "\n    Expected error: " << query.error
          << "\n    Observed error: " << (*worst_error)
          << "\n    For distance: " << worst_config->signed_distance;
    }
    EXPECT_LE(*worst_error, query.error)
        << "Expected error is too small!"
        << "\n  " << worst_config->description
        << "\n    Expected error: " << query.error
        << "\n    Observed error: " << (*worst_error)
        << "\n    For distance: " << worst_config->signed_distance;
  }
}

template <typename T>
void CharacterizeResultTest<T>::RunCharacterization(const QueryInstance& query,
                                                    bool is_symmetric) {
  auto shape1 = MakeShape(query.shape1, false /* use_alt */);
  auto shape2 = MakeShape(query.shape2, query.shape1 == query.shape2);
  RunCharacterization(query, *shape1, *shape2,
                      MakeConfigurations(*shape1, *shape2, TestDistances()),
                      is_symmetric);
}

template <typename T>
GeometryId CharacterizeResultTest<T>::EncodeData(fcl::CollisionObjectd* obj) {
  const GeometryId id = GeometryId::get_new_id();
  const EncodedData data(id, true);
  data.write_to(obj);
  collision_filter_.AddGeometry(data.id());
  return id;
}

template <typename T>
vector<RigidTransform<T>> CharacterizeResultTest<T>::X_WAs() {
  return vector<RigidTransform<T>>{
      RigidTransform<T>{math::RotationMatrix<T>(AngleAxis<T>(
                            5 * M_PI / 9, Vector3<T>{-1, -1, 1}.normalized())),
                        Vector3<T>{-0.2, 0.15, -0.3}},
      RigidTransform<T>{math::RotationMatrix<T>(AngleAxis<T>(
                            22 * M_PI / 9, Vector3<T>{-1, 1, -1}.normalized())),
                        Vector3<T>{-10.0, 20.2, -12.3}}};
}

template <typename T>
std::unique_ptr<Shape> CharacterizeResultTest<T>::MakeShape(GeometryType shape,
                                                            bool use_alt) {
  switch (shape) {
    case kBox:
      return std::make_unique<Box>(box(use_alt));
    case kCapsule:
      return std::make_unique<Capsule>(capsule(use_alt));
    case kConvex:
      return std::make_unique<Convex>(convex(use_alt));
    case kCylinder:
      return std::make_unique<Cylinder>(cylinder(use_alt));
    case kEllipsoid:
      return std::make_unique<Ellipsoid>(ellipsoid(use_alt));
    case kHalfSpace:
      return std::make_unique<HalfSpace>(half_space(use_alt));
    case kMesh:
      return std::make_unique<Mesh>(mesh(use_alt));
    case kPoint:
      return std::make_unique<Sphere>(0.0);
    case kSphere:
      return std::make_unique<Sphere>(sphere(use_alt));
  }
  DRAKE_UNREACHABLE();
}

template <typename T>
Box CharacterizeResultTest<T>::box(bool alt) {
  if (alt) {
    return Box{kDistance * 100, kDistance * 38, kDistance * 73};
  } else {
    return Box{kDistance * 37, kDistance * 100, kDistance * 43};
  }
}

template <typename T>
Capsule CharacterizeResultTest<T>::capsule(bool alt) {
  if (alt) {
    return Capsule{kDistance * 19, kDistance * 100};
  } else {
    return Capsule{kDistance * 100, kDistance * 51};
  }
}

template <typename T>
Convex CharacterizeResultTest<T>::convex(bool) {
  return Convex("ignored for this test", 1.0);
}

template <typename T>
Cylinder CharacterizeResultTest<T>::cylinder(bool alt) {
  if (alt) {
    return Cylinder{kDistance * 19, kDistance * 100};
  } else {
    return Cylinder{kDistance * 100, kDistance * 71};
  }
}

template <typename T>
Ellipsoid CharacterizeResultTest<T>::ellipsoid(bool alt) {
  if (alt) {
    return Ellipsoid{kDistance * 35, kDistance * 100, kDistance * 68};
  } else {
    return Ellipsoid{kDistance * 47, kDistance * 26, kDistance * 100};
  }
}

template <typename T>
HalfSpace CharacterizeResultTest<T>::half_space(bool) {
  return HalfSpace();
}

template <typename T>
Mesh CharacterizeResultTest<T>::mesh(bool) {
  throw std::logic_error(
      "Mesh will be supported when it is no longer represented as a Convex");
}

template <typename T>
Sphere CharacterizeResultTest<T>::sphere(bool) {
  return Sphere(kDistance * 100);
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS((
    &AlignPlanes<T>
))

}  // namespace internal
}  // namespace geometry
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
  class ::drake::geometry::internal::CharacterizeResultTest)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
  class ::drake::geometry::internal::ShapeConfigurations)
