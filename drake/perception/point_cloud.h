#pragma once

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/eigen_types.h"

namespace drake {
namespace perception {

/// Point cloud flags.
namespace pc_flags {

/// Indicates the data the point cloud stores.
enum Field : int {
  /// Inherit other fields. May imply an intersection of all
  /// compatible extras.
  kInherit = 0,
  /// XYZ point in Cartesian space.
  kXYZs = 1 << 0,
  /// Extras, whose type (and structure) is specified by `ExtraType`.
  kExtras = 1 << 3,
};
typedef int Fields;

/// Describes an extra field with a name and the extra's size.
///
/// @note This is defined as follows to enable an open set of extras, but
/// ensure that these extra types are appropriately matched.
/// As `PointCloud` evolves and more algorithms are mapped into Drake,
/// promoting an extra field to a proper field should be considered if (a) it
/// is used frequently enough AND (b) if it is often used in conjunction
/// with other fields.
class ExtraType final {
 public:
  ExtraType(int size, const std::string& name)
      : size_(size),
        name_(name) {}

  ExtraType(const ExtraType& other)
      : ExtraType(other.size(), other.name()) {}

  // Strictly enforce immutable semantics.
  ExtraType(const ExtraType&&) = delete;
  ExtraType& operator=(const ExtraType&) = delete;
  ExtraType& operator=(ExtraType&&) = delete;

  inline int size() const { return size_; }
  inline const std::string& name() const { return name_; }
  inline bool operator==(const ExtraType& other) const {
    return size_ == other.size_ && name_ == other.name_;
  }
  inline bool operator!=(const ExtraType& other) const {
    return !(*this == other);
  }

 private:
  const int size_{};
  const std::string name_;
};

/// No extra.
const ExtraType kExtraNone(0, "None");
/// Inherited extra.
const ExtraType kExtraInherit(-1, "Inherit");
/// Curvature.
const ExtraType kExtraCurvature(1, "Curvature");
/// Point-feature-histogram.
const ExtraType kExtraFPFH(33, "FPFH");

/// Returns a human-friendly representation of a field and extra set.
std::string ToString(Fields fields, const ExtraType &extra_type);

}  // namespace pc_flags


/// Implements a point cloud (with contiguous storage), whose main goal is to
/// offer a convenient, synchronized interface to commonly used fields and
/// data types applicable for basic 3D perception.
///
/// This is a mix between the philosophy of PCL (templated interface to
/// provide a compile-time open set, run-time closed set) and VTK (non-templated
/// interface to provide a very free form run-time open set).
/// You may construct one PointCloud which will contain different sets of
/// data, but you cannot change the contained data types after construction.
/// However, you can mutate the data contained within the structure and resize
/// the structure.
///
/// Definitions:
///  - point - An entry in a point cloud (not exclusively an XYZ point).
///  - feature - Geometric information of a point.
///  - descriptor - Non-geometric information of a point.
///  - field - A feature or descriptor described by the point cloud.
///  - extra - Runtime-definable information (feature or descriptor) for a
///  point.
///
/// This point cloud class provides:
///  - xyz - Cartesian XYZ coordinates (float[3]).
///  - extra - An open-set of fields (PFH, SHOT, etc) (float[X]).
///
/// @note "contiguous" here means contiguous in memory. This was chosen to
/// avoid complications with PCL, where "dense" implies that the point cloud
/// corresponds to a depth image, and is indexed accordingly (a grid with
/// column-major storage).
///
/// @note The accessors / mutators for the point fields of this class returns
/// references to the original Eigen matrices. This implies that they are
/// invalidated whenever memory is reallocated for the values. Given this,
/// minimize the lifetime of these references to be as short as possible.
/// Additionally, algorithms wanting fast access to values should avoid the
/// single point accessors (e.g. `xyz(i)`, `color(i)`).
///
/// @note The distinction between "feature" and "descriptor" were taken from
/// the survey paper:
///   Pomerleau, François, Francis Colas, and Roland Siegwart. "A review of
///   point cloud registration algorithms for mobile robotics." Foundations and
///   Trends® in Robotics 4.1 (2015): 1-104.
class PointCloud final {
 public:
  /// Geometric scalar type.
  typedef float T;

  /// Extra scalar type.
  typedef T E;

  /// Represents an invalid or uninitialized value.
  static constexpr T kDefaultValue = std::numeric_limits<T>::quiet_NaN();
  static inline bool IsInvalidValue(T value) { return std::isnan(value); }

  /// Constructs a point cloud of a given `new_size`, with the prescribed
  /// `fields`. If `kExtras` is one of the fields, then
  /// `extra` should included and should not be `kNone`.
  /// @param new_size
  ///   Size of the point cloud after construction.
  /// @param fields
  ///   Fields that the point cloud contains.
  /// @param extra
  ///   Extra field types. @see ExtraType
  PointCloud(int new_size,
             pc_flags::Fields fields = pc_flags::kXYZs,
             const pc_flags::ExtraType& extra_type = pc_flags::kExtraNone);

  /// Copies another point cloud's fields and data.
  PointCloud(const PointCloud& other)
      : PointCloud(other, pc_flags::kInherit) {}

  /// Copies another point cloud's fields and data.
  /// @param copy_fields
  ///   Fields to copy. If this is `kInherit`, then `other`s fields will be
  ///   copied.
  /// @param extra_type
  ///   Extra type to copy. Must
  // Do not define a default argument for `copy_fields` so that this is
  // not ambiguous w.r.t. the copy constructor.
  PointCloud(const PointCloud& other,
             pc_flags::Fields copy_fields,
             const pc_flags::ExtraType& extra_type = pc_flags::kExtraInherit);

  PointCloud& operator=(const PointCloud& other);

  ~PointCloud();

  // TODO(eric.cousineau): Consider locking the point cloud or COW to permit
  // shallow copies.

  /// Returns the fields provided by this point cloud.
  pc_flags::Fields fields() const { return fields_; }

  /// Returns the number of points in this point cloud.
  int size() const { return size_; }

  /// Conservative resize; will maintain existing data, and initialize new
  /// data to their invalid values.
  /// @param new_size
  ///    The new size of the value. If less than the present `size()`, then
  ///    the values will be truncated. If greater than the present `size()`,
  ///    then the new values will be uninitalized if `skip_initialize` is not
  ///    true.
  /// @param skip_initialize
  ///    Do not default-initialize new values.
  void resize(int new_size, bool skip_initialize = false);


  /// @name Features (Geometric)
  /// @{

  /// Returns if this cloud provides XYZ points.
  bool has_xyzs() const;

  /// Returns access to XYZ points.
  /// This method aborts if this cloud does not provide XYZ points.
  Eigen::Ref<const Matrix3X<T>> xyzs() const;

  /// Returns mutable access to XYZ points.
  /// This method aborts if this cloud does not provide XYZ points.
  Eigen::Ref<Matrix3X<T>> mutable_xyzs();

  /// Returns access to a XYZ point.
  /// This method aborts if this cloud does not provide XYZ points.
  Vector3<T> xyz(int i) const { return xyzs().col(i); }

  /// Returns mutable access to a XYZ point.
  /// This method aborts if this cloud does not provide XYZ points.
  Eigen::Ref<Vector3<T>> mutable_xyz(int i) {
    return mutable_xyzs().col(i);
  }

  ///@}

  /// @name Extras (Features and/or Descriptors)
  /// @{

  /// Returns if this point cloud provides extra points.
  bool has_extras() const;

  /// Returns if the point cloud provides a specific extra.
  bool has_extras(const pc_flags::ExtraType& extra_type) const;

  /// Returns the extra type.
  const pc_flags::ExtraType& extra_type() const { return extra_type_; }

  /// Returns access to extra points.
  /// This method aborts if this point cloud does not provide extra points.
  Eigen::Ref<const MatrixX<E>> extras() const;

  /// Returns mutable access to extra points.
  /// This method aborts if this point cloud does not provide extra points.
  Eigen::Ref<MatrixX<E>> mutable_extras();

  /// Returns access to a extra point.
  /// This method aborts if this cloud does not provide extra points.
  VectorX<T> extra(int i) const { return extras().col(i); }

  /// Returns mutable access to a extra point.
  /// This method aborts if this cloud does not provide extra points.
  Eigen::Ref<VectorX<T>> mutable_extra(int i) {
    return mutable_extras().col(i);
  }

  /// @}

  /// @name Container Manipulation
  /// @{

  /// Copy all points from another point cloud.
  /// @param other
  ///    Other point cloud.
  /// @param fields_in
  ///    Fields to copy. If this is `kInherit`, then both clouds must have the
  ///    exact same fields. Otherwise, both clouds must support the fields
  ///    indicated by `c`.
  /// @param allow_resize
  ///    Permit resizing to the other cloud's size.
  void SetFrom(
      const PointCloud& other,
      pc_flags::Fields fields_in = pc_flags::kInherit,
      bool allow_resize = true);

  // TODO(eric.cousineau): Add indexed version.

  /// Adds `add_size` default-initialized points.
  /// @param new_size
  ///    Number of points to add.
  /// @param skip_initialization
  ///    Do not require that the new values be initialized.
  void AddPoints(int add_size, bool skip_initialization = false);

  /// @}

  /// @name Fields
  /// @{

  /// Returns if a point cloud has a given set of fields.
  /// @pre If `kExtra` is not present in `c`, then `extra_type` must be
  /// `kExtraNone`. Otherwise, `extra_type` must be a valid extra.
  bool HasFields(
      pc_flags::Fields fields_in,
      const pc_flags::ExtraType& extra_type_in = pc_flags::kExtraNone) const;

  /// Requires a given set of fields.
  /// @see HasFields for preconditions.
  /// @throws std::runtime_error if this point cloud does not have these
  /// fields.
  void RequireFields(
      pc_flags::Fields fields_in,
      const pc_flags::ExtraType& extra_type_in = pc_flags::kExtraNone) const;

  /// Returns if a point cloud has exactly a given set of fields.
  /// @see HasFields for preconditions.
  bool HasExactFields(
      pc_flags::Fields fields_in,
      const pc_flags::ExtraType& extra_type_in = pc_flags::kExtraNone) const;

  /// Requires the exact given set of fields.
  /// @see HasFields for preconditions.
  /// @throws std::runtime_error if this point cloud does not have exactly
  /// these fields.
  void RequireExactFields(
      pc_flags::Fields field_set,
      const pc_flags::ExtraType& extra_type = pc_flags::kExtraNone) const;

  /// @}

 private:
  void SetDefault(int start, int num);

  // Provides PIMPL encapsulation of storage mechanism.
  class Storage;

  // Represents the size of the point cloud.
  int size_;
  // Represents which fields are enabled for this point cloud.
  const pc_flags::Fields fields_{};
  // Represents extra type stored.
  // @note If `has_extras()` is false, this should be `kExtraNone`.
  const pc_flags::ExtraType extra_type_{pc_flags::kExtraNone};
  // Owns storage used for the point cloud.
  std::unique_ptr<Storage> storage_;
};

// TODO(eric.cousineau): Consider a way of reinterpret_cast<>ing the array
// data to permit more semantic access to members, PCL-style
// (e.g. point.x, point.r, etc: the reverse of PointCloud<>::getMatrixXfMap()).
// Need to ensure alignments are commensurate. Will only work with
// homogeneous data (possibly with heterogeneous data, if strides can be
// used).

}  // namespace perception
}  // namespace drake
