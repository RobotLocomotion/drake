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

 %RenderLabel values are managed by a SceneGraph instance; the SceneGraph is
 responsible for defining and allocating them. They cannot be generally
 constructed.

 __Usage__

 Every geometry added to a RenderEngine will render into a label image. How that
 geometry appears depends on the value of its associated %RenderLabel. The
 application is responsible for creating a mapping between assigned %RenderLabel
 values and semantic interpretation. This can be done in several ways:

 1. Allocate a new render label by name via SceneGraph::GetRenderLabel().
    SceneGraph will provide a %RenderLabel with an arbitrary, unique value. The
    application can store a mapping between that %RenderLabel and its declared
    interpretation, or use SceneGraph to query the registered name of the label
    via SceneGraphInspector::GetRenderLabelName().
 2. Assign one of the _reserved_ %RenderLabel values:
      - empty: a pixel with the "empty" %RenderLabel value indicates that _no_
        geometry rendered to that pixel. It's worth noting that this doesn't
        make a geometry "invisible" to a label rendering. In other words, if
        one object marked with "empty" occludes another object, that second
        object's label is _not_ rendered, but the pixel gets the "empty" value.
        Therefore, it is highly inadvisable to mark any geometry with the
        "empty" render label.
      - ignored: the "ignored" label is a convenient dumping ground. Any
        geometry that doesn't need to be explicitly categorized can simply be
        assigned the ignored label.
      - unclassified: in the absence of an explicitly assigned render label,
        a geometry receives the unclassified label. It is considered a defect
        for an application not to explicitly assign a label to every rendered
        geometry; this tag simply aids in detecting if such has occurred. This
        marks the distinction between an application _actively_ declaring it
        doesn't care about a geometry (via the "ignored" tag) and an oversight
        that may lead to a geometry not receiving a tag.  */
class RenderLabel {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RenderLabel)

  /** (Internal) The default constructor is provided strictly for compatibility
   with STL structures. It will only create an unclassified render label.  */
  RenderLabel() : RenderLabel(kUnclassified) {}

  using ValueType = int16_t;

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

  /** Returns a label representing the "empty", or no object, label.  */
  static RenderLabel empty_label() {
    return RenderLabel{kEmpty};
  }

  /** Reports if the label is the empty label.  */
  bool is_empty() const { return value_ == kEmpty; }

  /** Returns a label representing the "ignored" label.  */
  static RenderLabel ignored_label() {
    return RenderLabel{kIgnored};
  }

  /** Reports if the label is the ignored label.  */
  bool is_ignored() const { return value_ == kIgnored; }

  /** Returns a label representing the "unclassified" label.  */
  static RenderLabel unclassified_label() {
    return RenderLabel{kUnclassified};
  }

  /** Reports if the label is the unclassified label.  */
  bool is_unclassified() const { return value_ == kUnclassified; }

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
  // Classes that can serve as RenderLabel factories.
  friend class internal::RenderLabelManager;
  friend class RenderEngine;
  friend class RenderLabelTester;

  // Returns the largest valid *unreserved* value a RenderLabel can be
  // constructed with.
  static ValueType maximum_unreserved() { return kMaxUnreserved; }

  // Private constructor precludes general construction except by the approved
  // factories (see above).
  explicit RenderLabel(ValueType value) : value_(value) {
    DRAKE_DEMAND(value >= 0);
  }

  // Reserved label values.
  static constexpr ValueType kUnclassified =
      std::numeric_limits<ValueType>::max();
  static constexpr ValueType kIgnored = kUnclassified - 1;
  static constexpr ValueType kEmpty = kIgnored - 1;
  static constexpr ValueType kMaxUnreserved = kEmpty - 1;

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
