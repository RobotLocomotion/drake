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
#include "drake/geometry/geometry_index.h"

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
 collision object stores the GeometryIndex of the corresponding SceneGraph
 geometry as well as a bit to mark whether it is anchored or dynamic.

 It stores this data by compactly "encoding" them. The encoding packs a geometry
 index value with a bit indicating if the index refers to dynamic or anchored
 geometry (proximity engine segregates them). The highest-order bit indicates
 dynamic (1) or anchored (0). The remaining lower bits store the index. The data
 is stored in a pointer-sized integer. This integer is, in turn, stored directly
 into fcl::CollisionObject's void* user data member.  */
class EncodedData {
 public:
  /** Constructs encoded data directly from the index and the known
   anchored/dynamic characterization.  */
  EncodedData(GeometryIndex index, bool is_dynamic)
      : data_(static_cast<uintptr_t>(index)) {
    if (is_dynamic) set_dynamic();
    // NOTE: data is encoded as anchored by default. So, an action only needs to
    // be taken in the dynamic case.
  }

  /** Constructs encoded data by extracting it from the given collision object.
   */
  explicit EncodedData(const fcl::CollisionObject<double>& fcl_object)
      : data_(reinterpret_cast<uintptr_t>(fcl_object.getUserData())) {}

  /** Constucts encoded data for the given index identified as dynamic.  */
  static EncodedData encode_dynamic(GeometryIndex index) {
    return EncodedData(index, true);
  }

  /** Constucts encoded data for the given index identified as anchored.  */
  static EncodedData encode_anchored(GeometryIndex index) {
    return EncodedData(index, false);
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

  /** Reports the stored index.  */
  GeometryIndex index() const {
    return static_cast<GeometryIndex>(data_ & ~kIsDynamicMask);
  }

  /** Given a map from GeometryIndex to ids of the SceneGraph geometries,
   reports the geometry id for the encoded data.  */
  GeometryId id(const std::vector<GeometryId>& geometry_map) const {
    return geometry_map[index()];
  }

  /** Reports the encoded data.  */
  uintptr_t encoding() const { return data_; }

 private:
  // Note: This sets a mask to be 1000...0 based on the size of a pointer.
  // C++ guarantees that uintptr_t can hold a pointer and cast to a pointer,
  // but it doesn't guarantee it's the *same size* as a pointer. So, we set
  // the mask value based on pointer size. This is important because we're
  // storing it in the void* of the collision object.
  static const uintptr_t kIsDynamicMask = uintptr_t{1}
      << (sizeof(void*) * 8 - 1);

  // The encoded data - index and mobility type.
  // We're using an unsigned value here because:
  //   - Bitmasking games are typically more intuitive with unsigned values
  //     (think of bit shifting as an example here).
  //   - This unsigned integer doesn't bleed out into the API at all.
  uintptr_t data_{};
};

/** Returns the name of the geometry associated with the given collision
 `object`.  */
std::string GetGeometryName(const fcl::CollisionObjectd& object);

}  // namespace internal
}  // namespace geometry
}  // namespace drake

#endif  // !defined(DRAKE_DOXYGEN_CXX)
