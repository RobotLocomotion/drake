#pragma once

// Exclude internal classes from doxygen.
#if !defined(DRAKE_DOXYGEN_CXX)

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

#include <fcl/fcl.h>
#include <fmt/format.h>

#include "drake/geometry/geometry_ids.h"

namespace drake {
namespace geometry {
namespace internal {

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
  using ValueType = int64_t;

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
  void write_to(fcl::CollisionObject<double>* object) {
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
  // Note: This sets a mask to be 1000...0 based on the size of a pointer.
  // C++ guarantees that ValueType can hold a pointer and cast to a pointer,
  // but it doesn't guarantee it's the *same size* as a pointer. So, we set
  // the mask value based on pointer size. This is important because we're
  // storing it in the void* of the collision object.
  static const ValueType kIsDynamicMask = ValueType{1}
      << (sizeof(void*) * 8 - 1);

  static_assert(sizeof(ValueType) == sizeof(GeometryId),
                 "The encoded data must be the same size as the identifier "
                 "to use in EncodedData");

  // The encoded data - id and mobility type.
  // We're using an unsigned value here because:
  //   - Bitmasking games are typically more intuitive with unsigned values
  //     (think of bit shifting as an example here).
  //   - This unsigned integer doesn't bleed out into the API at all.
  ValueType data_{};
};

/** Returns the name of the geometry associated with the given collision
 `object`.  */
std::string GetGeometryName(const fcl::CollisionObjectd& object);

}  // namespace internal
}  // namespace geometry
}  // namespace drake

#endif  // !defined(DRAKE_DOXYGEN_CXX)
