#pragma once

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

#include <fcl/fcl.h>
#include <fmt/format.h>

#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {

// TODO(SeanCurtis-TRI): Given the dependencies on fcl for this file, the name
//  should reflect it so that it doesn't get included in files that will
//  eventually get included in the public API.

// TODO(SeanCurtis-TRI): Snake case this name.
/** Calculates an absolute tolerance value conditioned to a problem's
 characteristic, positive-valued `size`. The tolerance is sufficient to account
 for round-off error which arises due to transformations.
 @pre size > 0.  */
constexpr double DistanceToPointRelativeTolerance(double size) {
  return 1e-14 * std::max(1.0, size);
}

/** Class for coordinating the collision objects stored in the proximity engine
 with the geometries stored in SceneGraph. The two are related in that the
 collision object stores the GeometryId of the corresponding SceneGraph
 geometry as well as a bit to mark whether it is anchored or dynamic.

 It stores this data by compactly "encoding" them. The encoding packs a geometry
 id value with a bit indicating if the id refers to dynamic or anchored geometry
 (proximity engine segregates them). The highest-order bit indicates dynamic (1)
 or anchored (0). The remaining lower bits store the id. The data is stored in a
 pointer-sized integer. This integer is, in turn, stored directly into
 fcl::CollisionObject's void* user data member.  */
class EncodedData {
 public:
  using ValueType = decltype(GeometryId::get_new_id().get_value());

  /** Constructs encoded data directly from the id and the known
   anchored/dynamic characterization.  */
  EncodedData(GeometryId id, bool is_dynamic)
      : data_(static_cast<ValueType>(id.get_value())) {
    // Make sure we haven't used so many ids that we're *using* the highest-
    // order bit -- i.e., it must be strictly positive.
    DRAKE_DEMAND(data_ > 0);
    if (is_dynamic) set_dynamic();
    // NOTE: data is encoded as anchored by default. So, an action only needs to
    // be taken in the dynamic case.
  }

  /** Constructs encoded data by extracting it from the given collision object.
   */
  explicit EncodedData(const fcl::CollisionObject<double>& fcl_object)
      : data_(reinterpret_cast<ValueType>(fcl_object.getUserData())) {}

  /** Constructs encoded data for the given id identified as dynamic.  */
  static EncodedData encode_dynamic(GeometryId id) {
    return {id, true};
  }

  /** Constructs encoded data for the given id identified as anchored.  */
  static EncodedData encode_anchored(GeometryId id) {
    return {id, false};
  }

  /** Sets the encoded data to be dynamic.  */
  void set_dynamic() { data_ |= kIsDynamicMask; }

  /** Sets the encoded data to be anchored.  */
  void set_anchored() { data_ &= ~kIsDynamicMask; }

  /** Writes the encoded data into the collision object's user data.  */
  void write_to(fcl::CollisionObject<double>* object) const {
    object->setUserData(reinterpret_cast<void*>(data_));
  }

  /** Reports true if the encoded data is marked as dynamic. False if anchored.
   */
  bool is_dynamic() const { return (data_ & kIsDynamicMask) != 0; }

  /** Reports the stored id.  */
  GeometryId id() const {
    return static_cast<GeometryId>(data_ & ~kIsDynamicMask);
  }

  /** Reports the encoded data.  */
  ValueType encoding() const { return data_; }

 private:
  // For this encoding to work we have the following requirements:
  //  1. ValueType must be at least as large as a pointer.
  //  2. ValueType must be at least as large as GeometryId. (At least as large
  //     still implies that with enough GeometryIds allocated, the high order
  //     bit could become ambiguous -- however, that would require one id for
  //     each atom in the universe).
  // These static asserts guarantee these conditions.

  // This is redundant of the declaration of ValueType; it serves as an
  // independent witness to the requirement in case the declaration changes.
  static_assert(sizeof(ValueType) >= sizeof(GeometryId),
                "The encoded data type must be at least as large as the "
                "identifier to use in EncodedData");

  static_assert(sizeof(ValueType) >= sizeof(void*),
                "The encoded data type must be at least as large as a pointer "
                "type");

  // Regardless of how large ValueType is, ultimately, we need to be able to
  // pack the encoding into a void*. So, we set the bit mask as the highest
  // order bit of something pointer sized.
  static const ValueType kIsDynamicMask = ValueType{1}
      << (sizeof(void*) * 8 - 1);

  // The encoded data - id and mobility type masked together.
  ValueType data_{};
};

/** Returns the name of the geometry associated with the given collision
 `object`.  */
std::string GetGeometryName(const fcl::CollisionObjectd& object);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
