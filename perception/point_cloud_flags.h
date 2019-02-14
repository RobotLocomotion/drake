#pragma once

#include <iostream>
#include <stdexcept>
#include <string>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace perception {

/// Point cloud flags.
namespace pc_flags {

typedef int BaseFieldT;
/// Indicates the data the point cloud stores.
enum BaseField : int {
  kNone = 0,
  /// Inherit other fields. May imply an intersection of all
  /// compatible descriptors.
  kInherit = 1 << 0,
  /// XYZ point in Cartesian space.
  kXYZs = 1 << 1,
  /// Normals.
  kNormals = 1 << 2,
  /// RGB colors.
  kRGBs = 1 << 3,
};

namespace internal {

// N.B. Ensure this is the largest bit.
constexpr BaseField kMaxBitInUse = kRGBs;

}  // namespace internal

/// Describes an descriptor field with a name and the descriptor's size.
///
/// @note This is defined as follows to enable an open set of descriptors, but
/// ensure that these descriptor types are appropriately matched.
/// As `PointCloud` evolves and more algorithms are mapped into Drake,
/// promoting an descriptor field to a proper field should be considered if (a)
/// it is used frequently enough AND (b) if it is often used in conjunction
/// with other fields.
class DescriptorType final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DescriptorType)

  constexpr DescriptorType(int size, const char* name)
      : size_(size),
        name_(name) {}

  inline int size() const { return size_; }
  inline std::string name() const { return name_; }
  inline bool operator==(const DescriptorType& other) const {
    return size_ == other.size_ && name() == other.name();
  }
  inline bool operator!=(const DescriptorType& other) const {
    return !(*this == other);
  }

 private:
  int size_{};
  const char* name_{nullptr};
};

/// No descriptor.
constexpr DescriptorType kDescriptorNone(0, "kDescriptorNone");
/// Curvature.
constexpr DescriptorType kDescriptorCurvature(1, "kDescriptorCurvature");
/// Point-feature-histogram.
constexpr DescriptorType kDescriptorFPFH(33, "kDescriptorFPFH");

/**
 * Allows combination of `BaseField` and `DescriptorType` for a `PointCloud`.
 * You may combine multiple `BaseField`s, but you may have only zero or one
 * `DescriptorType`.
 *
 * This provides the mechanism to use basic bit-mask operators (| &) to
 * combine / intersect fields for convenience.
 */
class Fields {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Fields)

  /// @throws std::runtime_error if `base_fields` is not composed of valid
  /// `BaseField`s.
  Fields(BaseFieldT base_fields, DescriptorType descriptor_type)
      : base_fields_(base_fields),
        descriptor_type_(descriptor_type) {}

  /// @throws std::runtime_error if `base_fields` is not composed of valid
  /// `BaseField`s.
  Fields(BaseFieldT base_fields)  // NOLINT(runtime/explicit)
      : base_fields_(base_fields) {
    if (base_fields < 0 || base_fields >= (internal::kMaxBitInUse << 1))
      throw std::runtime_error("Invalid BaseField specified.");
  }

  // NOLINTNEXTLINE(runtime/explicit): This conversion is desirable.
  Fields(const DescriptorType& descriptor_type)
      : descriptor_type_(descriptor_type) {}

  /// Returns the contained base fields.
  BaseFieldT base_fields() const { return base_fields_; }

  /// Returns whether there are any base fields contained by this set of fields.
  bool has_base_fields() const {
    return base_fields_ != kNone;
  }

  /// Returns the contained descriptor type.
  const DescriptorType& descriptor_type() const { return descriptor_type_; }

  /// Returns whether there is a descriptor contained by this set of fields.
  bool has_descriptor() const {
    return descriptor_type_ != kDescriptorNone;
  }

  /// Provides in-place union.
  /// @throws std::runtime_error if multiple non-None `DescriptorType`s are
  /// specified.
  Fields& operator|=(const Fields& rhs) {
    base_fields_ = base_fields_ | rhs.base_fields_;
    if (has_descriptor())
      throw std::runtime_error(
          "Cannot have multiple Descriptor flags. "
          "Can only add flags iff (!rhs.has_descriptor()).");
    descriptor_type_ = rhs.descriptor_type_;
    return *this;
  }

  /// Provides union.
  /// @see operator|= for preconditions.
  Fields operator|(const Fields& rhs) const {
    return Fields(*this) |= rhs;
  }

  /// Provides in-place intersection.
  Fields& operator&=(const Fields& rhs) {
    base_fields_ &= rhs.base_fields_;
    if (descriptor_type_ != rhs.descriptor_type_) {
      descriptor_type_ = kDescriptorNone;
    }
    return *this;
  }

  /// Provides intersection.
  Fields operator&(const Fields& rhs) const {
    return Fields(*this) &= rhs;
  }

  /// Returns whether both value types (BaseField + DescriptorType) are none.
  bool empty() const {
    return !has_base_fields() && !has_descriptor();
  }

  /// Returns whether this set of fields contains (is a superset of) `rhs`.
  bool contains(const Fields& rhs) const {
    return (*this & rhs) == rhs;
  }

  bool operator==(const Fields& rhs) const {
    return (base_fields_ == rhs.base_fields_
            && descriptor_type_ == rhs.descriptor_type_);
  }

  bool operator!=(const Fields& rhs) const {
    return !(*this == rhs);
  }

  /// Provides human-readable output.
  friend std::ostream& operator<<(std::ostream& os, const Fields& rhs);

 private:
  // TODO(eric.cousineau): Use `optional` to avoid the need for `none` objects?
  BaseFieldT base_fields_{kNone};
  DescriptorType descriptor_type_{kDescriptorNone};
};

// Do not use implicit conversion because it becomes ambiguous.
/// Makes operator| compatible for `BaseField` + `DescriptorType`.
/// @see Fields::operator|= for preconditions.
inline Fields operator|(const BaseFieldT& lhs, const DescriptorType& rhs) {
  return Fields(lhs) | Fields(rhs);
}

/// Makes operator| compatible for `DescriptorType` + `Fields`
/// (`DescriptorType` or `BaseFields`).
/// @see Fields::operator|= for preconditions.
inline Fields operator|(const DescriptorType& lhs, const Fields& rhs) {
  return Fields(lhs) | rhs;
}

// TODO(eric.cousineau): Add compatible operator& if the need arises.

}  // namespace pc_flags

}  // namespace perception
}  // namespace drake
