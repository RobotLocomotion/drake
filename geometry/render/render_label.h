#pragma once

#include <atomic>
#include <cstdint>
#include <iostream>
#include <limits>
#include <string>

#include "drake/common/hash.h"
#include "drake/common/never_destroyed.h"

namespace drake {
namespace geometry {
namespace render {

// Forward declaration in support of friend declaration.
class RenderEngine;

namespace internal {

// Forward declaration in support of friend declaration.
class RenderLabelManager;

}  // namespace internal

/**
 Class representing object "labels" for rendering.

 In a "label image" (@see RenderEngine::RenderLabelImage() for details) each
 pixel value indicates the classification of the object that rendered into that
 pixel. The %RenderLabel class provides that value and one label is associated
 with each rendered geometry.

 The labels could be unique for each geometry, or multiple geometries could all
 share the same label (becoming indistinguishable in the label image).
 Ultimately, it is the user's responsibility to assign labels in a manner that
 is meaningful for their application.

 <h3>Reserved labels</h3>

 There are several %RenderLabels that are reserved. They have specific meaning
 in the context of the rendering ecosystem and are globally available to all
 applications. They are:

 - `empty`: a pixel with the `empty` %RenderLabel value indicates that _no_
   geometry rendered to that pixel. Assigning this label to geometry is _highly_
   inadvisable and will typically lead to undesirable results.
 - `do_not_render`: any geometry assigned the `do_not_render` tag will _not_ be
   rendered into a label image. This is a clear declaration that a geometry
   should be omitted. Useful for marking, e.g., glass windows so that the
   visible geometry behind the glass is what is included in the label image.
 - `dont_care`: the `dont_care` label is intended as a convenient dumping
   ground. This would be for geometry that _should_ render into the label image,
   but whose class is irrelevant (e.g., the walls of a room a robot is working
   in or the background terrain in driving simulation).
 - `unspecified`: in the absence of an explicitly assigned render label, a
   geometry receives the `unspecified` label. RenderEngine implementations will
   throw an exception if they are given a geometry with an `unspecified`
   RenderLabel, because it is considered to be a likely a defect for an
   application not to explicitly assign a label to every rendered geometry.

 <h2>Usage</h2>

 For a label image to be _meaningful_, every pixel value should admit an
 unambiguous interpretation. To do that, %RenderLabels need to be coordinated
 to avoid accidental overloading. An application can achieve this in one of two
 ways: the application can rely on SceneGraph to allocate and coordinate
 %RenderLabel values across multiple geometry sources or the application can
 manage its own RenderLabel values. Mixing the two strategies is highly
 inadvisable; the responsibility for guaranteeing unique %RenderLabel values
 does not survive sharing well.

 <h3>Allocation of %RenderLabel values from SceneGraph</h3>

 An application defines a render label _class_ defined by a unique string (e.g.,
 "car", "robot", "table", etc.) in conjunction with the id of a geometry source
 -- if two sources were both to declare a "robot" label class, they would still
 be distinct classes.

 The application associates a %RenderLabel with its render label class by
 invoking SceneGraph::GetRenderLabel(). The application can store a mapping
 between that %RenderLabel and its declared interpretation, or exploit the
 fact that SceneGraph maintains that mapping for all %RenderLabel values it
 allocates and simply query the registered name of the label via
 SceneGraphInspector::GetRenderLabelName().

 <h3>Manual %RenderLabel allocation</h3>

 An application can simply instantiate %RenderLabel with an arbitrary value.
 This allows the application to define a particular mapping from render label
 class to a preferred %RenderLabel value. The applications bears _full_
 responsibility in making sure that a single value is not inadvertently
 associated with multiple render classes. Finally, a %RenderLabel cannot be
 explicitly constructed with a reserved value -- those can only be accessed
 through the static methods provided.  */
class RenderLabel {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RenderLabel)

  using ValueType = int16_t;

  /** Constructs a label with the reserved `unspecified` value.  */
  RenderLabel() : RenderLabel(kUnspecified) {}

  /** Constructs a label with the given `value`.
   @throws std::logic_error if a) is negative, or b) the `value` is one of the
                               reserved values.  */
  explicit RenderLabel(ValueType value) : RenderLabel(value, true) {}

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
   has no particular semantic meaning for applications.  */
  bool operator<(const RenderLabel& other) const {
    return value_ < other.value_;
  }

  /** @name  The reserved render labels  */
  //@{

  static const RenderLabel kEmpty;
  static const RenderLabel kDoNotRender;
  static const RenderLabel kDontCare;
  static const RenderLabel kUnspecified;

  /** The largest value that a %RenderLabel can be instantiated on. */
  static const ValueType kMaxUnreserved;

  //@}

  /** Reports if the label is a reserved label.  */
  bool is_reserved() const { return value_ > kMaxUnreserved; }

  /** Implements the @ref hash_append concept.  */
  template <class HashAlgorithm>
  friend void hash_append(HashAlgorithm& hasher,
                          const RenderLabel& item) noexcept {
    using drake::hash_append;
    hash_append(hasher, item.value_);
  }

  /** Implicit conversion to its underlying integer representation.  */
  operator ValueType() const { return value_; }

  /** Enables use of labels with the streaming operator.  */
  friend std::ostream& operator<<(std::ostream& out, const RenderLabel& label);

  /** Converts the RenderLabel value to a string representation.  */
  friend std::string to_string(const RenderLabel& label);

 private:
  // Returns the largest valid *unreserved* value a RenderLabel can be
  // constructed with.
  static ValueType maximum_unreserved() { return kMaxUnreserved; }

  // Private constructor precludes general construction except by the approved
  // factories (see above).
  RenderLabel(ValueType value, bool needs_testing) : value_(value) {
    if (value < 0 || (needs_testing && value > kMaxUnreserved)) {
      throw std::logic_error(
          "Invalid construction of RenderLabel with invalid value: " +
          std::to_string(value));
    }
  }

  // The underlying value.
  ValueType value_;
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
