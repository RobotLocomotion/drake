#include "drake/perception/point_cloud.h"

#include <utility>

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "drake/common/drake_assert.h"

using Eigen::Map;
using Eigen::NoChange;

namespace drake {
namespace perception {

namespace {

// Convenience aliases.
typedef PointCloud::T T;
typedef PointCloud::C C;
typedef PointCloud::D D;

}  // namespace

/*
 * Provides encapsulated storage for a `PointCloud`.
 *
 * This storage is not responsible for initializing default values.
 */
class PointCloud::Storage {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Storage)

  Storage(int new_size, pc_flags::Fields fields)
      : fields_(fields) {
    // Ensure that we incorporate the size of the descriptors.
    descriptors_.resize(fields_.descriptor_type().size(), 0);
    // Resize as normal.
    resize(new_size);
  }

  // Returns size of the storage.
  int size() const { return size_; }

  // Resize to parent cloud's size.
  void resize(int new_size) {
    size_ = new_size;
    if (fields_.contains(pc_flags::kXYZs))
      xyzs_.conservativeResize(NoChange, new_size);
    if (fields_.contains(pc_flags::kNormals))
      normals_.conservativeResize(NoChange, new_size);
    if (fields_.contains(pc_flags::kRGBs))
      rgbs_.conservativeResize(NoChange, new_size);
    if (fields_.has_descriptor())
      descriptors_.conservativeResize(NoChange, new_size);
    CheckInvariants();
  }

  Eigen::Ref<Matrix3X<T>> xyzs() { return xyzs_; }
  Eigen::Ref<Matrix3X<T>> normals() { return normals_; }
  Eigen::Ref<Matrix3X<C>> rgbs() { return rgbs_; }
  Eigen::Ref<MatrixX<T>> descriptors() { return descriptors_; }

 private:
  void CheckInvariants() const {
    if (fields_.contains(pc_flags::kXYZs)) {
      const int xyz_size = xyzs_.cols();
      DRAKE_DEMAND(xyz_size == size());
    }
    if (fields_.contains(pc_flags::kNormals)) {
      const int normals_size = normals_.cols();
      DRAKE_DEMAND(normals_size == size());
    }
    if (fields_.contains(pc_flags::kRGBs)) {
      const int rgbs_size = rgbs_.cols();
      DRAKE_DEMAND(rgbs_size == size());
    }
    if (fields_.has_descriptor()) {
      const int descriptor_size = descriptors_.cols();
      DRAKE_DEMAND(descriptor_size == size());
    }
  }

  const pc_flags::Fields fields_;
  int size_{};
  Matrix3X<T> xyzs_;
  Matrix3X<T> normals_;
  Matrix3X<C> rgbs_;
  MatrixX<T> descriptors_;
};

namespace {

pc_flags::Fields ResolveFields(
    const PointCloud& other, pc_flags::Fields fields) {
  if (fields == pc_flags::kInherit) {
    return other.fields();
  } else {
    return fields;
  }
}

// Resolves the fields from a pair of point clouds and desired fields.
// Implements the resolution rules in `SetFrom`.
// @pre Valid point clouds `a` and `b`.
// @returns Fields that both point clouds have.
pc_flags::Fields ResolvePairFields(
    const PointCloud& a,
    const PointCloud& b,
    pc_flags::Fields fields) {
  if (fields == pc_flags::kInherit) {
    // If we do not permit a subset, expect the exact same fields.
    a.RequireExactFields(b.fields());
    return a.fields();
  } else {
    a.RequireFields(fields);
    b.RequireFields(fields);
    return fields;
  }
}

}  // namespace

PointCloud::PointCloud(
    int new_size, pc_flags::Fields fields, bool skip_initialize)
    : size_(new_size),
      fields_(fields) {
  if (fields_ == pc_flags::kNone)
    throw std::runtime_error("Cannot construct a PointCloud without fields");
  if (fields_.contains(pc_flags::kInherit))
    throw std::runtime_error("Cannot construct a PointCloud with kInherit");
  storage_.reset(new Storage(size_, fields_));
  if (!skip_initialize) {
    SetDefault(0, size_);
  }
}

PointCloud::PointCloud(const PointCloud& other,
                       pc_flags::Fields copy_fields)
    : PointCloud(other.size(), ResolveFields(other, copy_fields)) {
  SetFrom(other);
}

PointCloud::PointCloud(PointCloud&& other)
    : PointCloud(0, other.fields(), true) {
  // This has zero size. Directly swap storages.
  storage_.swap(other.storage_);
  std::swap(size_, other.size_);
  DRAKE_DEMAND(storage_->size() == size());
}

PointCloud& PointCloud::operator=(const PointCloud& other) {
  SetFrom(other);
  return *this;
}

PointCloud& PointCloud::operator=(PointCloud&& other) {
  // We may only take rvalue references if the fields match exactly.
  RequireExactFields(other.fields());
  // Swap storages.
  size_ = other.size_;
  storage_.swap(other.storage_);
  DRAKE_DEMAND(storage_->size() == size());
  // Empty out the other cloud, but let it remain being a valid point cloud
  // (with non-null storage).
  other.resize(0, false);
  return *this;
}

// Define destructor here to use complete definition of `Storage`.
PointCloud::~PointCloud() {}

void PointCloud::resize(int new_size, bool skip_initialization) {
  DRAKE_DEMAND(new_size >= 0);
  int old_size = size();
  size_ = new_size;
  storage_->resize(new_size);
  DRAKE_DEMAND(storage_->size() == new_size);
  if (new_size > old_size && !skip_initialization) {
    int size_diff = new_size - old_size;
    SetDefault(old_size, size_diff);
  }
}

void PointCloud::SetDefault(int start, int num) {
  auto set = [=](auto ref, auto value) {
    ref.middleCols(start, num).setConstant(value);
  };
  if (has_xyzs()) {
    set(mutable_xyzs(), kDefaultValue);
  }
  if (has_normals()) {
    set(mutable_normals(), kDefaultValue);
  }
  if (has_rgbs()) {
    set(mutable_rgbs(), kDefaultColor);
  }
  if (has_descriptors()) {
    set(mutable_descriptors(), kDefaultValue);
  }
}

void PointCloud::SetFrom(const PointCloud& other,
                         pc_flags::Fields fields_in,
                         bool allow_resize) {
  int old_size = size();
  int new_size = other.size();
  if (allow_resize) {
    resize(new_size);
  } else if (new_size != old_size) {
    throw std::runtime_error(
        fmt::format("SetFrom: {} != {}", new_size, old_size));
  }
  pc_flags::Fields fields_resolved =
      ResolvePairFields(*this, other, fields_in);
  if (fields_resolved.contains(pc_flags::kXYZs)) {
    mutable_xyzs() = other.xyzs();
  }
  if (fields_resolved.contains(pc_flags::kNormals)) {
    mutable_normals() = other.normals();
  }
  if (fields_resolved.contains(pc_flags::kRGBs)) {
    mutable_rgbs() = other.rgbs();
  }
  if (fields_resolved.has_descriptor()) {
    mutable_descriptors() = other.descriptors();
  }
}

void PointCloud::Expand(
    int add_size,
    bool skip_initialization) {
  DRAKE_DEMAND(add_size >= 0);
  const int new_size = size() + add_size;
  resize(new_size, skip_initialization);
}

bool PointCloud::has_xyzs() const {
  return fields_.contains(pc_flags::kXYZs);
}
Eigen::Ref<const Matrix3X<T>> PointCloud::xyzs() const {
  DRAKE_DEMAND(has_xyzs());
  return storage_->xyzs();
}
Eigen::Ref<Matrix3X<T>> PointCloud::mutable_xyzs() {
  DRAKE_DEMAND(has_xyzs());
  return storage_->xyzs();
}

bool PointCloud::has_normals() const {
  return fields_.contains(pc_flags::kNormals);
}
Eigen::Ref<const Matrix3X<T>> PointCloud::normals() const {
  DRAKE_DEMAND(has_normals());
  return storage_->normals();
}
Eigen::Ref<Matrix3X<T>> PointCloud::mutable_normals() {
  DRAKE_DEMAND(has_normals());
  return storage_->normals();
}

bool PointCloud::has_rgbs() const {
  return fields_.contains(pc_flags::kRGBs);
}
Eigen::Ref<const Matrix3X<C>> PointCloud::rgbs() const {
  DRAKE_DEMAND(has_rgbs());
  return storage_->rgbs();
}
Eigen::Ref<Matrix3X<C>> PointCloud::mutable_rgbs() {
  DRAKE_DEMAND(has_rgbs());
  return storage_->rgbs();
}

bool PointCloud::has_descriptors() const {
  return fields_.has_descriptor();
}
bool PointCloud::has_descriptors(
    const pc_flags::DescriptorType& descriptor_type) const {
  return fields_.contains(descriptor_type);
}
Eigen::Ref<const MatrixX<D>> PointCloud::descriptors() const {
  DRAKE_DEMAND(has_descriptors());
  return storage_->descriptors();
}
Eigen::Ref<MatrixX<D>> PointCloud::mutable_descriptors() {
  DRAKE_DEMAND(has_descriptors());
  return storage_->descriptors();
}

bool PointCloud::HasFields(
    pc_flags::Fields fields_in) const {
  DRAKE_DEMAND(!fields_in.contains(pc_flags::kInherit));
  return fields_.contains(fields_in);
}

void PointCloud::RequireFields(
    pc_flags::Fields fields_in) const {
  if (!HasFields(fields_in)) {
    throw std::runtime_error(
        fmt::format("PointCloud does not have expected fields.\n"
                    "Expected {}, got {}",
                    fields_in, fields()));
  }
}

bool PointCloud::HasExactFields(
    pc_flags::Fields fields_in) const {
  return fields() == fields_in;
}

void PointCloud::RequireExactFields(
    pc_flags::Fields fields_in) const {
  if (!HasExactFields(fields_in)) {
    throw std::runtime_error(
        fmt::format("PointCloud does not have the exact expected fields."
                    "\nExpected {}, got {}",
                    fields_in, fields()));
  }
}

}  // namespace perception
}  // namespace drake
