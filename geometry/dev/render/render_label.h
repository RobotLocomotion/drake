#pragma once

#include <atomic>
#include <cstdint>
#include <iostream>
#include <limits>
#include <string>

#include "drake/common/hash.h"
#include "drake/common/never_destroyed.h"

namespace drake {

// Forward declaration to support friend declaration.
namespace systems {
namespace sensors {

template <typename IdType>
class ColorPalette;

}  // namespace sensors
}  // namespace systems

namespace geometry {
namespace dev {
namespace render {

/**
 Class representing object "labels" for rendering.

 Geometry in the scene can be grouped into classifications. Each classification
 is associated with a label value. For example, all links in a robot arm could
 be classified as "robot", the static environment (tables, etc.) could be
 classified as "environment", and manipulands would likewise get one or more
 classes. Each of these would be associated with a label _value_.

 These classes are used in the renderer in generating a "label image"
 (@see RenderEngine::RenderLabelImage() for details).

 This %RenderLabel class is responsible for guaranteeing *globally* unique
 labels such that two geometry sources cannot accidentally assign the same label
 value to different classes.

 __Usage__

 Geometry sources should invoke RenderLabel::new_label() to generate a
 globally unique label. The _semantics_ of that label is defined by the
 geometry source. That label should be stored and then applied to all of the
 visual geometries which belong to that semantic class. The label image will
 contain integer values which correspond to the set of __known__ labels. Those
 values can be compared with the source's persisted labels to determine the
 class of object that rendered to the corresponding pixel.

 Multiple geometry sources can share a label to have a common semantic
 interpretation. The label would be generated and then shared. The rendered
 image can include labels defined by _all_ geometry sources. So, any particular
 geometry source may encounter label values it did not generate. The
 system must either ignore them, or have been given knowledge of those labels
 from some other source.

 The set of known labels always includes two universal labels: "empty" and
 "terrain". The empty label is applied to pixels in which *no* geometry is
 drawn. The "terrain" label is a generic catch all; it can be used to detect
 that something rasterized to the pixel, but the geometry's nature is
 unimportant. The name comes from driving in which the uncharacterized
 background would be "terrain". The label values can be acquired by calling
 RenderLabel::empty_label() and RenderLabel::terrain_label(),
 respectively.

 @note The renderer only supports approximately 1500 unique labels. Every call
 to new_label() consumes one of those unique labels. Be careful not to
 generate and then throw them away; they can't be recovered.

 @cond
 NOTE: In many ways, this is very similar to the TypeSafeIndex class. There are
 several key differences:

   - added static members,
   - special values (empty and terrain),
   - arbitrarily limited domain (kLabelCount).

 // TODO(SeanCurtis-TRI):
 //  1. Change the "default" labels to:
 //      - "empty" (no object at that pixel)
 //      - "ignored" (the former "terrain")
 //      - "unclassified" (the default value which indicates that no label was
 //        explicitly assigned -- indcating possible user error).
 @endcond
 */
class RenderLabel {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RenderLabel)

  /** Default constructor; the result is the empty label value. */
  RenderLabel() : value_(kEmptyValue) {}

  /** Reports if the label is the empty label. */
  bool is_empty() const { return value_ == kEmptyValue; }

  /** Reports if the label is the terrain label. */
  bool is_terrain() const { return value_ == kTerrainValue; }

  /** Compares this label with the `other` label. Reports true if they have the
   same value. */
  bool operator==(const RenderLabel& other) const {
    return value_ == other.value_;
  }

  /** Compares this label with the `other` label. Reports true if they have
   different values. */
  bool operator!=(const RenderLabel& other) const {
    return value_ != other.value_;
  }

  /** Allows the labels to be compared to imply a total ordering -- facilitates
   use in data structures which require ordering (e.g., std::set).  */
  bool operator<(const RenderLabel& other) const {
    return value_ < other.value_;
  }

  /** Generates a new identifier for this id type. This new identifier will be
   different from all previous identifiers created. This method does _not_
   make any guarantees about the values of ids from successive invocations.
   This method is guaranteed to be thread safe.
   @throws std::runtime_error if the number of calls to new_label() exceeds the
   number of ids supported by Type.
   */
  static RenderLabel new_label() {
    // Note that id 0 is empty, 1 is terrain.
    static never_destroyed<std::atomic<int16_t>> next_label(0);
    int16_t value = next_label.access()++;
    if (value >= kLabelCount) {
      throw std::runtime_error(
          "Calls to new_label() have exhausted the available labels");
    }
    return RenderLabel(value);
  }

  /** Returns a label representing the "empty", or no object, label. */
  static RenderLabel empty_label() {
    return RenderLabel{kEmptyValue};
  }

  /** Returns a label representing the "terrain" label. */
  static RenderLabel terrain_label() {
    return RenderLabel{kTerrainValue};
  }

  /** Implements the @ref hash_append concept. */
  template <class HashAlgorithm>
  friend void hash_append(HashAlgorithm& hasher,
                          const RenderLabel& item) noexcept {
    using drake::hash_append;
    hash_append(hasher, item.value_);
  }

  /** Implicit conversion to int16_t. */
  operator int16_t() const { return value_; }

  friend std::ostream& operator<<(std::ostream& out, const RenderLabel& label);

  /** Enables use of labels with to_string. It requires ADL to work. So,
   it should be invoked as: `to_string(label);` and should be preceded by
   `using std::to_string`.*/
  friend std::string to_string(const RenderLabel& label);

 private:
  // TODO(SeanCurtis-TRI): Modify how the color palette allocates colors to
  // remove this artificial limit.
  // The maximum number of labels available--dictated, currently, by the
  // implementation of ColorPalette.
  static constexpr int16_t kLabelCount = 256 * 6 - 1;
  static constexpr int16_t kEmptyValue = kLabelCount + 1;
  static constexpr int16_t kTerrainValue = kEmptyValue + 1;

  static_assert(kLabelCount <= INT16_MAX,
                "The maximum label value must be small enough to fit in an "
                    "2-byte integer");

  // Temporary support for the color palette to do pre-emptive color allocation.
  template <typename T>
  friend class systems::sensors::ColorPalette;

  // Instantiates an identifier from the underlying representation type.
  explicit RenderLabel(int val) : value_(val) {}

  // The underlying value.
  int16_t value_;
};


}  // namespace render
}  // namespace dev
}  // namespace geometry
}  // namespace drake

namespace std {

/** Enables use of the label to serve as a key in STL containers.
 @relates RenderLabel
 */
template <>
struct hash<drake::geometry::dev::render::RenderLabel>
    : public drake::DefaultHash {};

}  // namespace std
