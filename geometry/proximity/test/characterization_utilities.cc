#include "drake/geometry/proximity/test/characterization_utilities.h"

#include <algorithm>
#include <fstream>
#include <limits>

#include <fmt/format.h>

#include "drake/common/nice_type_name.h"
#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity_engine.h"
#include "drake/geometry/utilities.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace geometry {
namespace internal {

using Eigen::Vector3d;
using math::RigidTransform;
using std::vector;

std::string to_string(const fcl::NODE_TYPE& node) {
  switch (node) {
    case fcl::BV_UNKNOWN:
    case fcl::BV_AABB:
    case fcl::BV_OBB:
    case fcl::BV_RSS:
    case fcl::BV_kIOS:
    case fcl::BV_OBBRSS:
    case fcl::BV_KDOP16:
    case fcl::BV_KDOP18:
    case fcl::BV_KDOP24:
    case fcl::GEOM_CONE:
    case fcl::GEOM_PLANE:
    case fcl::GEOM_TRIANGLE:
    case fcl::GEOM_OCTREE:
    case fcl::NODE_COUNT:
      return "unsupported type";
    case fcl::GEOM_BOX:
      return "Box";
    case fcl::GEOM_SPHERE:
      return "Sphere";
    case fcl::GEOM_ELLIPSOID:
      return "Ellipsoid";
    case fcl::GEOM_CAPSULE:
      return "Capsule";
    case fcl::GEOM_CYLINDER:
      return "Cylinder";
    case fcl::GEOM_CONVEX:
      return "Convex";
    case fcl::GEOM_HALFSPACE:
      return "HalfSpace";
  }
  DRAKE_UNREACHABLE();
}

MakeFclShape::MakeFclShape(const Shape& shape) : ShapeReifier() {
  shape.Reify(this);
}

void MakeFclShape::ImplementGeometry(const Sphere& sphere, void*) {
  object_ = std::make_shared<fcl::Sphered>(sphere.radius());
}

template <typename T>
void CharacterizeResultTest<T>::RunCallback(
    const Expectation& expectation, fcl::CollisionObjectd* obj_A,
    fcl::CollisionObjectd* obj_B, const CollisionFilterLegacy* collision_filter,
    const std::unordered_map<GeometryId, RigidTransform<T>>* X_WGs)
    const {
  callback_->ClearResults();
  ASSERT_EQ(callback_->GetNumResults(), 0);
  if (expectation.can_compute) {
    ASSERT_FALSE(callback_->Invoke(obj_A, obj_B, collision_filter, X_WGs));
    ASSERT_EQ(callback_->GetNumResults(), 1) << "No results reported!";
  } else {
    DRAKE_ASSERT_THROWS_MESSAGE(
        callback_->Invoke(obj_A, obj_B, collision_filter, X_WGs),
        std::exception, expectation.error_message);
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
void CharacterizeResultTest<T>::RunCharacterization(
    const Expectation& expectation, const Shape& shape_A, const Shape& shape_B,
    const vector<Configuration<T>>& configs) {
  fcl::CollisionObjectd object_A = MakeFclShape(shape_A).object();
  const GeometryId id_A = EncodeData(&object_A);

  fcl::CollisionObjectd object_B = MakeFclShape(shape_B).object();
  const GeometryId id_B = EncodeData(&object_B);

  const auto& X_WAs = this->X_WAs();
  std::optional<double> worst_error;
  std::optional<Configuration<T>> worst_config;

  auto evaluate_callback =
      [this, &expectation, &worst_error, &worst_config](
          char first, fcl::CollisionObjectd* obj_A, char second,
          fcl::CollisionObjectd* obj_B, auto test_config,
          const std::unordered_map<GeometryId, RigidTransform<T>>&
              world_poses) {
        SCOPED_TRACE(fmt::format(
            "\n{}-{} (obj {}-obj {}) query:"
            "\n  Expected signed distance: {}"
            "\n  X_AB"
            "\n{}\n",
            to_string(obj_A->collisionGeometry()->getNodeType()),
            to_string(obj_B->collisionGeometry()->getNodeType()), first, second,
            test_config.signed_distance, test_config.X_AB.GetAsMatrix34()));
        RunCallback(expectation, obj_A, obj_B, &collision_filter_,
                    &world_poses);
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

  /* Last reality check. If the expectation was for computability, we better
   report *some* value as an error, even if it's zero. */
  ASSERT_EQ(expectation.can_compute, worst_error.has_value());
  /* The only way worst_error has no value, is that we expected *every*
   configuration to fail. Then no value is the correct outcome. */
  if (worst_error.has_value()) {
    /* Our definition of "close" to epsilon: within 2 bits. Still a tight
     bound but gives a modicum of breathing room. */
    constexpr double cutoff = 4 * std::numeric_limits<double>::epsilon();
    if (expectation.max_error > cutoff) {
      EXPECT_GT(*worst_error, expectation.max_error / 2)
          << "Expected error is too big!"
          << "\n    Expected error: " << expectation.max_error
          << "\n    Observed error: " << (*worst_error)
          << "\n    For distance: " << worst_config->signed_distance;
    }
    EXPECT_LE(*worst_error, expectation.max_error)
        << "Expected error is too small!"
        << "\n    Expected error: " << expectation.max_error
        << "\n    Observed error: " << (*worst_error)
        << "\n    For distance: " << worst_config->signed_distance;
  }
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

}  // namespace internal
}  // namespace geometry
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
  class ::drake::geometry::internal::CharacterizeResultTest)
