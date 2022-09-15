#include "drake/perception/point_cloud.h"

#include <algorithm>
#include <functional>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include <drake_vendor/nanoflann.hpp>
#include <fmt/format.h>
#include <fmt/ostream.h>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
#include "drake/common/unused.h"

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

  // Update fields, allocating (but not initializing) new fields when needed.
  void UpdateFields(pc_flags::Fields f) {
    xyzs_.conservativeResize(NoChange, f.contains(pc_flags::kXYZs) ? size_ : 0);
    normals_.conservativeResize(NoChange,
                                f.contains(pc_flags::kNormals) ? size_ : 0);
    rgbs_.conservativeResize(NoChange, f.contains(pc_flags::kRGBs) ? size_ : 0);
    descriptors_.conservativeResize(NoChange, f.has_descriptor() ? size_ : 0);
    fields_ = f;
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

  pc_flags::Fields fields_;
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

PointCloud PointCloud::Crop(const Eigen::Ref<const Vector3<T>>& lower_xyz,
                            const Eigen::Ref<const Vector3<T>>& upper_xyz) {
  DRAKE_DEMAND((lower_xyz.array() <= upper_xyz.array()).all());
  if (!has_xyzs()) {
    throw std::runtime_error("PointCloud must have xyzs in order to Crop");
  }
  PointCloud crop(size_, fields(), true);
  int index = 0;
  for (int i = 0; i < size_; ++i) {
    if (((xyzs().col(i).array() >= lower_xyz.array()) &&
         (xyzs().col(i).array() <= upper_xyz.array()))
            .all()) {
      crop.mutable_xyzs().col(index) = xyzs().col(i);
      if (has_normals()) {
        crop.mutable_normals().col(index) = normals().col(i);
      }
      if (has_rgbs()) {
        crop.mutable_rgbs().col(index) = rgbs().col(i);
      }
      if (has_descriptors()) {
        crop.mutable_descriptors().col(index) = descriptors().col(i);
      }
      ++index;
    }
  }
  crop.resize(index);
  return crop;
}

PointCloud Concatenate(const std::vector<PointCloud>& clouds) {
  const int num_clouds = clouds.size();
  DRAKE_DEMAND(num_clouds >= 1);
  int count = clouds[0].size();
  for (int i = 1; i < num_clouds; ++i) {
    DRAKE_THROW_UNLESS(clouds[i].fields() == clouds[0].fields());
    count += clouds[i].size();
  }
  PointCloud new_cloud(count, clouds[0].fields(), true);
  int index = 0;
  for (int i = 0; i < num_clouds; ++i) {
    const int s = clouds[i].size();
    if (new_cloud.has_normals()) {
      new_cloud.mutable_xyzs().middleCols(index, s) = clouds[i].xyzs();
    }
    if (new_cloud.has_normals()) {
      new_cloud.mutable_normals().middleCols(index, s) = clouds[i].normals();
    }
    if (new_cloud.has_rgbs()) {
      new_cloud.mutable_rgbs().middleCols(index, s) = clouds[i].rgbs();
    }
    if (new_cloud.has_descriptors()) {
      new_cloud.mutable_descriptors().middleCols(index, s) =
          clouds[i].descriptors();
    }
    index += s;
  }
  return new_cloud;
}

namespace {

// Hash function for Eigen::Vector3i
// implemented as in boost::hash_combine.
struct Vector3iHash {
  std::size_t operator()(const Eigen::Vector3i& index) const {
    size_t hash = 0;
    const auto add_to_hash = [&hash] (int value) {
      hash ^= std::hash<int>{}(value) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
    };
    add_to_hash(index.x());
    add_to_hash(index.y());
    add_to_hash(index.z());
    return hash;
  }
};

}  // namespace

PointCloud PointCloud::VoxelizedDownSample(double voxel_size) const {
  // This is a simple, narrow, no-frills implementation of the
  // voxel_down_sample algorithm in Open3d and/or the down-sampling by a
  // VoxelGrid filter in PCL.
  DRAKE_THROW_UNLESS(has_xyzs());
  DRAKE_THROW_UNLESS(voxel_size > 0);
  Eigen::Vector3f lower_xyz =
      Eigen::Vector3f::Constant(std::numeric_limits<float>::infinity());
  for (int i = 0; i < size_; ++i) {
    if (xyz(i).array().isFinite().all()) {
      lower_xyz[0] = std::min(xyz(i)[0], lower_xyz[0]);
      lower_xyz[1] = std::min(xyz(i)[1], lower_xyz[1]);
      lower_xyz[2] = std::min(xyz(i)[2], lower_xyz[2]);
    }
  }

  // Create a map from voxel coordinate to a set of points.
  absl::flat_hash_map<Eigen::Vector3i, std::vector<int>, Vector3iHash>
      voxel_map;
  for (int i = 0; i < size_; ++i) {
    if (xyz(i).array().isFinite().all()) {
      voxel_map[((xyz(i) - lower_xyz) / voxel_size).cast<int>()].emplace_back(
          i);
    }
  }
  PointCloud down_sampled(voxel_map.size(), fields());

  // Iterate through the map populating the elements of the down_sampled cloud.
  // TODO(russt): Consider using OpenMP. Sample code from calderpg-tri provided
  // during review of #17885.
  int index_in_down_sampled = 0;
  for (const auto& [coordinates, indices_in_this] : voxel_map) {
    unused(coordinates);
    // Use doubles instead of floats for accumulators to avoid round-off errors.
    Eigen::Vector3d xyz{Eigen::Vector3d::Zero()};
    Eigen::Vector3d normal{Eigen::Vector3d::Zero()};
    Eigen::Vector3d rgb{Eigen::Vector3d::Zero()};
    Eigen::VectorXd descriptor{
        Eigen::VectorXd::Zero(has_descriptors() ? descriptors().rows() : 0)};
    int num_normals{0};
    int num_descriptors{0};

    for (int index_in_this : indices_in_this) {
      xyz += xyzs().col(index_in_this).cast<double>();
      if (has_normals() &&
          normals().col(index_in_this).array().isFinite().all()) {
        normal += normals().col(index_in_this).cast<double>();
        ++num_normals;
      }
      if (has_rgbs()) {
        rgb += rgbs().col(index_in_this).cast<double>();
      }
      if (has_descriptors() &&
          descriptors().col(index_in_this).array().isFinite().all()) {
        descriptor += descriptors().col(index_in_this).cast<double>();
        ++num_descriptors;
      }
    }
    down_sampled.mutable_xyzs().col(index_in_down_sampled) =
        (xyz / indices_in_this.size()).cast<T>();
    if (has_normals()) {
      down_sampled.mutable_normals().col(index_in_down_sampled) =
          (normal / num_normals).cast<T>();
    }
    if (has_rgbs()) {
      down_sampled.mutable_rgbs().col(index_in_down_sampled) =
          (rgb / indices_in_this.size()).cast<C>();
    }
    if (has_descriptors()) {
      down_sampled.mutable_descriptors().col(index_in_down_sampled) =
          (descriptor / num_descriptors).cast<D>();
    }
    ++index_in_down_sampled;
  }

  return down_sampled;
}

void PointCloud::EstimateNormals(double radius, int max_nearest_neighbors) {
  DRAKE_DEMAND(radius > 0);
  DRAKE_DEMAND(max_nearest_neighbors > 0);
  DRAKE_THROW_UNLESS(size() >= 3);
  DRAKE_THROW_UNLESS(has_xyzs());

  if (!has_normals()) {
    fields_ |= pc_flags::kNormals;
    storage_->UpdateFields(fields_);
  }

  const Eigen::MatrixX3f data = xyzs().transpose();
  nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixX3f, 3> kd_tree(3, data);

  // Iterate through all points and compute their normals.
  // TODO(russt): Make an OpenMP implementation of this.
  VectorX<int64_t> indices(max_nearest_neighbors);
  Eigen::VectorXf distances(max_nearest_neighbors);
  Eigen::Matrix3d covariance;
  Eigen::Matrix<double, 9, 1> cumulants;

  for (int i = 0; i < size_; ++i) {
    // With nanoflann, I can either search for the max_nearest_neighbors and
    // take the ones closer than radius, or search for all points closer than
    // radius and keep the max_nearest_neighbors.
    const int num_neighbors = kd_tree.index->knnSearch(
        xyz(i).data(), max_nearest_neighbors, indices.data(), distances.data());

    if (num_neighbors < 2) {
      mutable_normal(i) = Eigen::Vector3f{0, 0, 1};
      continue;
    }

    // Compute the covariance matrix.
    {
      cumulants.setZero();
      int count = 0;
      for (int j = 0; j < num_neighbors; ++j) {
        if (distances[j] <= radius) {
          ++count;
          const Eigen::Vector3f& point = xyz(indices[j]);
          cumulants(0) += point(0);
          cumulants(1) += point(1);
          cumulants(2) += point(2);
          cumulants(3) += point(0) * point(0);
          cumulants(4) += point(0) * point(1);
          cumulants(5) += point(0) * point(2);
          cumulants(6) += point(1) * point(1);
          cumulants(7) += point(1) * point(2);
          cumulants(8) += point(2) * point(2);
        }
      }
      if (count < 2) {
        mutable_normal(i) = Eigen::Vector3f{0, 0, 1};
        continue;
      }
      cumulants /= count;
      covariance(0, 0) = cumulants(3) - cumulants(0) * cumulants(0);
      covariance(1, 1) = cumulants(6) - cumulants(1) * cumulants(1);
      covariance(2, 2) = cumulants(8) - cumulants(2) * cumulants(2);
      covariance(0, 1) = cumulants(4) - cumulants(0) * cumulants(1);
      covariance(1, 0) = covariance(0, 1);
      covariance(0, 2) = cumulants(5) - cumulants(0) * cumulants(2);
      covariance(2, 0) = covariance(0, 2);
      covariance(1, 2) = cumulants(7) - cumulants(1) * cumulants(2);
      covariance(2, 1) = covariance(1, 2);
    }

    // TODO(russt): Open3d implements a "FastEigen3x3" for an optimized version
    // of this. We probably should, too.
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver;
    solver.compute(covariance, Eigen::ComputeEigenvectors);
    mutable_normal(i) = solver.eigenvectors().col(0).cast<float>();
  }
}

}  // namespace perception
}  // namespace drake
