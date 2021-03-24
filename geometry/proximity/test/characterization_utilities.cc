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
#include "drake/math/orthonormal_basis.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace geometry {
namespace internal {

using Eigen::Vector3d;
using math::RigidTransform;
using std::vector;

std::ostream& operator<<(std::ostream& out, GeometryType s) {
  switch (s) {
    case kCapsule:
      out << "Capsule";
      break;
    case kEllipsoid:
      out << "Ellipsoid";
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

void MakeFclShape::ImplementGeometry(const Capsule& capsule, void*) {
  object_ = std::make_shared<fcl::Capsuled>(capsule.radius(), capsule.length());
}

void MakeFclShape::ImplementGeometry(const Ellipsoid& ellipsoid, void*) {
  object_ = std::make_shared<fcl::Ellipsoidd>(ellipsoid.a(), ellipsoid.b(),
                                              ellipsoid.c());
}

void MakeFclShape::ImplementGeometry(const Sphere& sphere, void*) {
  object_ = std::make_shared<fcl::Sphered>(sphere.radius());
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
      const Matrix3<T> basis = math::ComputeBasisFromAxis(2, nhat);
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

template RigidTransform<double> AlignPlanes<double>(const Vector3<double>&,
                                                    const Vector3<double>&,
                                                    const Vector3<double>&,
                                                    const Vector3<double>&);
template RigidTransform<AutoDiffXd> AlignPlanes<AutoDiffXd>(
    const Vector3<AutoDiffXd>&, const Vector3<AutoDiffXd>&,
    const Vector3<AutoDiffXd>&, const Vector3<AutoDiffXd>&);

template <typename T>
void CharacterizeResultTest<T>::RunCallback(
    const QueryInstance& query, fcl::CollisionObjectd* obj_A,
    fcl::CollisionObjectd* obj_B, const CollisionFilterLegacy* collision_filter,
    const std::unordered_map<GeometryId, RigidTransform<T>>* X_WGs)
    const {
  callback_->ClearResults();
  ASSERT_EQ(callback_->GetNumResults(), 0);
  switch (query.outcome) {
    case kSupported:
      ASSERT_FALSE(callback_->Invoke(obj_A, obj_B, collision_filter, X_WGs));
      ASSERT_EQ(callback_->GetNumResults(), 1) << "No results reported!";
      break;
    case kThrows:
      DRAKE_ASSERT_THROWS_MESSAGE(
          callback_->Invoke(obj_A, obj_B, collision_filter, X_WGs),
          std::exception,
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
    const vector<Configuration<T>>& configs) {
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
      evaluate_callback('B', &object_B, 'A', &object_A, config, X_WGs);
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
      EXPECT_GT(*worst_error, query.error / 2)
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
void CharacterizeResultTest<T>::RunCharacterization(
    const QueryInstance& query) {
  auto shape1 = MakeShape(query.shape1, false /* use_alt */);
  auto shape2 = MakeShape(query.shape2, query.shape1 == query.shape2);
  RunCharacterization(query, *shape1, *shape2,
                      MakeConfigurations(*shape1, *shape2, TestDistances()));
}

template <typename T>
GeometryId CharacterizeResultTest<T>::EncodeData(fcl::CollisionObjectd* obj) {
  const GeometryId id = GeometryId::get_new_id();
  const EncodedData data(id, true);
  data.write_to(obj);
  collision_filter_.AddGeometry(data.encoding());
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
    case kCapsule:
      return std::make_unique<Capsule>(capsule(use_alt));
    case kEllipsoid:
      return std::make_unique<Ellipsoid>(ellipsoid(use_alt));
    case kSphere:
      return std::make_unique<Sphere>(sphere(use_alt));
  }
  DRAKE_UNREACHABLE();
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
Ellipsoid CharacterizeResultTest<T>::ellipsoid(bool alt) {
  if (alt) {
    return Ellipsoid{kDistance * 35, kDistance * 100, kDistance * 68};
  } else {
    return Ellipsoid{kDistance * 47, kDistance * 26, kDistance * 100};
  }
}

template <typename T>
Sphere CharacterizeResultTest<T>::sphere(bool) {
  return Sphere(kDistance * 100);
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
  class ::drake::geometry::internal::CharacterizeResultTest)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
  class ::drake::geometry::internal::ShapeConfigurations)
