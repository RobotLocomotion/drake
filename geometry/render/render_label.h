#pragma once

#include <cstdint>
#include <iostream>
#include <limits>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/common/hash.h"
#include "drake/systems/sensors/pixel_types.h"

namespace drake {
namespace geometry {
namespace render {

/**
 Class representing object "labels" for rendering.

 In a "label image" (see RenderEngine::RenderLabelImage() for details) each
 pixel value indicates the classification of the object that rendered into that
 pixel. The %RenderLabel class provides that value and one label is associated
 with each rendered geometry.

 The labels could be unique for each geometry, or multiple geometries could all
 share the same label (becoming indistinguishable in the label image).
 Ultimately, it is the user's responsibility to assign labels in a manner that
 is meaningful for their application.

 @anchor reserved_render_label
 <h3>Reserved labels</h3>

 There are several %RenderLabels that are reserved. They have specific meaning
 in the context of the rendering ecosystem and are globally available to all
 applications. They are:

 - `empty`: a pixel with the `empty` %RenderLabel value indicates that _no_
   geometry rendered to that pixel. Implemented as RenderLabel::kEmpty.
 - `do not render`: any geometry assigned the `do not render` tag will _not_ be
   rendered into a label image. This is a clear declaration that a geometry
   should be omitted. Useful for marking, e.g., glass windows so that the
   visible geometry behind the glass is what is included in the label image.
   Implemented as RenderLabel::kDoNotRender.
 - `don't care`: the `don't care` label is intended as a convenient dumping
   ground. This would be for geometry that _should_ render into the label image,
   but whose class is irrelevant (e.g., the walls of a room a robot is working
   in or the background terrain in driving simulation). Implemented as
   RenderLabel::kDontCare.
 - `unspecified`: a default-constructed %RenderLabel has an `unspecified` value.
   Implemented as RenderLabel::kUnspecified.

 Generally, there is no good reason to assign `empty` or `unspecified` labels
 to a geometry. A RenderEngine implementation is entitled to throw an exception
 if you attempt to do so.

 <h2>Usage</h2>

 An application can simply instantiate %RenderLabel with an arbitrary value.
 This allows the application to define a particular mapping from render label
 class to a preferred %RenderLabel value. For a label image to be _meaningful_,
 every pixel value should admit an unambiguous interpretation. The application
 bears _full_ responsibility in making sure that a single value is not
 inadvertently associated with multiple render classes. Finally, a %RenderLabel
 cannot be explicitly constructed with a reserved value -- those can only be
 accessed through the static methods provided.

 @note The %RenderLabel class is based on a 16-bit integer. This makes the label
 image more compact but means there are only, approximately, 32,000 unique
 %RenderLabel values.  */
class RenderLabel {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RenderLabel)

  // TODO(SeanCurtis-TRI): Change this to an unsigned int by defining the
  // kLabel16U pixel type (and accompanying ImageTraits). Change the @note in
  // the class documentation to match.
  using ValueType = systems::sensors::ImageTraits<
      systems::sensors::PixelType::kLabel16I>::ChannelType;

  /** Constructs a label with the reserved `unspecified` value.  */
  RenderLabel() = default;

  /** Constructs a label with the given `value`.
   @throws std::logic_error if a) is negative, or b) the `value` is one of the
                               reserved values.  */
  explicit RenderLabel(int value) : RenderLabel(value, true) {}

  /** Compares this label with the `other` label. Reports true if they have the
   same value.  */
  bool operator==(const RenderLabel& other) const {
    return value_ == other.value_;
  }

  /** Compares this label with the `other` label. Reports true if they have
   different values.  */
  bool operator!=(const RenderLabel& other) const {
    return value_ != other.value_;
  }

  /** Allows the labels to be compared to imply a total ordering -- facilitates
   use in data structures which require ordering (e.g., std::set). The ordering
   has no particular meaning for applications.  */
  bool operator<(const RenderLabel& other) const {
    return value_ < other.value_;
  }

  /** Implements the @ref hash_append concept. */
  template <class HashAlgorithm>
  friend void hash_append(
      HashAlgorithm& hasher, const RenderLabel& item) noexcept {
    using drake::hash_append;
    hash_append(hasher, item.value_);
  }

  /** @name  The reserved render labels

   See class documentation on
   @ref reserved_render_label "reserved labels" for details.  */
  //@{

  /** See @ref reserved_render_label "Reserved labels"  */
  static const RenderLabel kEmpty;

  /** See @ref reserved_render_label "Reserved labels"  */
  static const RenderLabel kDoNotRender;

  /** See @ref reserved_render_label "Reserved labels"  */
  static const RenderLabel kDontCare;

  /** See @ref reserved_render_label "Reserved labels"  */
  static const RenderLabel kUnspecified;

  /** The largest value that a %RenderLabel can be instantiated on. */
  static const ValueType kMaxUnreserved;

  //@}

  /** Reports if the label is a reserved label.  */
  bool is_reserved() const { return value_ > kMaxUnreserved; }

  /** Implicit conversion to its underlying integer representation.  */
  operator ValueType() const { return value_; }

  /** Enables use of labels with the streaming operator.  */
  friend std::ostream& operator<<(std::ostream& out, const RenderLabel& label);

  /** Converts the RenderLabel value to a string representation.  */
  friend std::string to_string(const RenderLabel& label);

 private:
  // RenderEngine needs access to encode labels as raster colors and to convert
  // rasterized colors back into labels.
  friend class RenderEngine;

  // Private constructor precludes general construction except by the approved
  // factories (see above).
  RenderLabel(int value, bool needs_testing)
      : value_(static_cast<ValueType>(value)) {
    if (value < 0 || (needs_testing && value > kMaxUnreserved)) {
      throw std::logic_error(
          "Invalid construction of RenderLabel with invalid value: " +
          std::to_string(value));
    }
  }

  static constexpr ValueType kMaxValue = std::numeric_limits<ValueType>::max();

  // The underlying value; this implicitly defines the unspecified value to be
  // the maximum value.
  ValueType value_{kMaxValue};
};

}  // namespace render
}  // namespace geometry
}  // namespace drake

namespace std {

/** Enables use of the label to serve as a key in STL containers.
 @relates RenderLabel  */
template <>
struct hash<drake::geometry::render::RenderLabel>
  : public drake::DefaultHash {};

}  // namespace std
