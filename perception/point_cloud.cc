#include "drake/perception/point_cloud.h"

#include <algorithm>
#include <atomic>
#include <functional>
#include <unordered_map>
#include <utility>
#include <vector>

#include <common_robotics_utilities/dynamic_spatial_hashed_voxel_grid.hpp>
#include <common_robotics_utilities/voxel_grid.hpp>
#include <fmt/format.h>
#include <nanoflann.hpp>

#include "drake/common/drake_assert.h"
#include "drake/common/unused.h"

using common_robotics_utilities::voxel_grid::DSHVGSetType;
using common_robotics_utilities::voxel_grid::DynamicSpatialHashedVoxelGrid;
using common_robotics_utilities::voxel_grid::GridIndex;
using common_robotics_utilities::voxel_grid::GridSizes;
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
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Storage);

  Storage(int new_size, pc_flags::Fields fields) : fields_(fields) {
    // Ensure that we incorporate the size of the descriptors.
    descriptors_.resize(fields_.descriptor_type().size(), 0);
    // Resize as normal.
    resize(new_size);
  }

  // Returns a reference to the fields provided by this storage. Note that the
  // outer class PointCloud::fields() returns a copy, but for Storage::fields()
  // we need to return a reference for performance and because we need to return
  // a nested reference for the descriptor_type().
  const pc_flags::Fields& fields() const { return fields_; }

  // Returns size of the storage.
  int size() const { return size_; }

  // A repaired implementation of Eigen::Matrix::conservativeResize that is safe
  // to call when the new size is zero. (Eigen 3.4.0 calls 'realloc(size=0)' in
  // that case, which is undefined.)
  //
  // Once Drake's minimum supported Eigen version is circa 2023 or newer, we can
  // go back to calling conservativeResize even with a zero size.
  template <typename EigenMatrix>
  static void ConservativeResizeCols(EigenMatrix* m, int cols) {
    if (cols == 0) {
      m->resize(m->rows(), cols);
    } else {
      m->conservativeResize(NoChange, cols);
    }
  }

  // Resize to parent cloud's size.
  void resize(int new_size) {
    size_ = new_size;
    if (fields_.contains(pc_flags::kXYZs))
      ConservativeResizeCols(&xyzs_, new_size);
    if (fields_.contains(pc_flags::kNormals))
      ConservativeResizeCols(&normals_, new_size);
    if (fields_.contains(pc_flags::kRGBs))
      ConservativeResizeCols(&rgbs_, new_size);
    if (fields_.has_descriptor())
      ConservativeResizeCols(&descriptors_, new_size);
    CheckInvariants();
  }

  // Update fields, allocating (but not initializing) new fields when needed.
  void UpdateFields(pc_flags::Fields f) {
    xyzs_.resize(Eigen::NoChange, f.contains(pc_flags::kXYZs) ? size_ : 0);
    normals_.resize(Eigen::NoChange,
                    f.contains(pc_flags::kNormals) ? size_ : 0);
    rgbs_.resize(Eigen::NoChange, f.contains(pc_flags::kRGBs) ? size_ : 0);
    descriptors_.resize(f.descriptor_type().size(),
                        f.has_descriptor() ? size_ : 0);
    fields_ = f;
    CheckInvariants();
  }

  Eigen::Ref<Matrix3X<T>> xyzs() { return xyzs_; }
  Eigen::Ref<Matrix3X<T>> normals() { return normals_; }
  Eigen::Ref<Matrix3X<C>> rgbs() { return rgbs_; }
  Eigen::Ref<MatrixX<T>> descriptors() { return descriptors_; }

 private:
  void CheckInvariants() const {
    const int xyz_size = xyzs_.cols();
    if (fields_.contains(pc_flags::kXYZs)) {
      DRAKE_DEMAND(xyz_size == size());
    } else {
      DRAKE_DEMAND(xyz_size == 0);
    }
    const int normals_size = normals_.cols();
    if (fields_.contains(pc_flags::kNormals)) {
      DRAKE_DEMAND(normals_size == size());
    } else {
      DRAKE_DEMAND(normals_size == 0);
    }
    const int rgbs_size = rgbs_.cols();
    if (fields_.contains(pc_flags::kRGBs)) {
      DRAKE_DEMAND(rgbs_size == size());
    } else {
      DRAKE_DEMAND(rgbs_size == 0);
    }
    const int descriptor_cols = descriptors_.cols();
    const int descriptor_rows = descriptors_.rows();
    if (fields_.has_descriptor()) {
      DRAKE_DEMAND(descriptor_cols == size());
      DRAKE_DEMAND(descriptor_rows == fields_.descriptor_type().size());
    } else {
      DRAKE_DEMAND(descriptor_cols == 0);
      DRAKE_DEMAND(descriptor_rows == 0);
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

pc_flags::Fields ResolveFields(const PointCloud& other,
                               pc_flags::Fields fields) {
  if (fields == pc_flags::kInherit) {
    return other.fields();
  } else {
    return fields;
  }
}

// Finds the added fields from `new_fields` w.r.t. `old_fields`.
pc_flags::Fields FindAddedFields(pc_flags::Fields old_fields,
                                 pc_flags::Fields new_fields) {
  pc_flags::Fields added_fields(pc_flags::kNone);
  for (const auto field :
       {pc_flags::kXYZs, pc_flags::kNormals, pc_flags::kRGBs}) {
    if (!old_fields.contains(field) && new_fields.contains(field))
      added_fields |= field;
  }

  if (new_fields.has_descriptor() &&
      old_fields.descriptor_type() != new_fields.descriptor_type()) {
    added_fields |= new_fields.descriptor_type();
  }
  return added_fields;
}

}  // namespace

PointCloud::PointCloud(int new_size, pc_flags::Fields fields,
                       bool skip_initialize) {
  if (fields == pc_flags::kNone)
    throw std::runtime_error("Cannot construct a PointCloud without fields");
  if (fields.contains(pc_flags::kInherit))
    throw std::runtime_error("Cannot construct a PointCloud with kInherit");
  storage_.reset(new Storage(new_size, fields));
  if (!skip_initialize) {
    SetDefault(0, new_size);
  }
}

PointCloud::PointCloud(const PointCloud& other, pc_flags::Fields copy_fields)
    : PointCloud(other.size(), ResolveFields(other, copy_fields)) {
  SetFrom(other);
}

PointCloud::PointCloud(PointCloud&& other)
    : PointCloud(0, other.storage_->fields(), true) {
  // This has zero size. Directly swap storages.
  storage_.swap(other.storage_);
}

PointCloud& PointCloud::operator=(const PointCloud& other) {
  SetFrom(other);
  return *this;
}

PointCloud& PointCloud::operator=(PointCloud&& other) {
  // Swap storages.
  storage_.swap(other.storage_);
  // Empty out the other cloud, but let it remain being a valid point cloud
  // (with non-null storage).
  other.resize(0, false);
  return *this;
}

// Define destructor here to use complete definition of `Storage`.
PointCloud::~PointCloud() {}

pc_flags::Fields PointCloud::fields() const {
  return storage_->fields();
}

int PointCloud::size() const {
  return storage_->size();
}

void PointCloud::resize(int new_size, bool skip_initialization) {
  DRAKE_DEMAND(new_size >= 0);
  const int old_size = size();
  if (old_size == new_size) {
    return;
  }
  storage_->resize(new_size);
  DRAKE_DEMAND(storage_->size() == new_size);
  if (new_size > old_size && !skip_initialization) {
    const int size_diff = new_size - old_size;
    SetDefault(old_size, size_diff);
  }
}

void PointCloud::SetFields(pc_flags::Fields new_fields, bool skip_initialize) {
  const pc_flags::Fields old_fields = storage_->fields();
  if (old_fields == new_fields) {
    return;
  }
  storage_->UpdateFields(new_fields);

  if (!skip_initialize) {
    // Default-initialize containers for newly added fields.
    const pc_flags::Fields added_fields =
        FindAddedFields(old_fields, new_fields);
    if (added_fields.contains(pc_flags::kXYZs))
      mutable_xyzs().setConstant(kDefaultValue);
    if (added_fields.contains(pc_flags::kNormals))
      mutable_normals().setConstant(kDefaultValue);
    if (added_fields.contains(pc_flags::kRGBs))
      mutable_rgbs().setConstant(kDefaultColor);
    if (added_fields.has_descriptor())
      mutable_descriptors().setConstant(kDefaultValue);
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

void PointCloud::SetFrom(const PointCloud& other, pc_flags::Fields fields_in,
                         bool allow_resize) {
  // Update the size of this point cloud if necessary.
  int old_size = size();
  int new_size = other.size();
  if (allow_resize) {
    resize(new_size);
  } else if (new_size != old_size) {
    throw std::runtime_error(
        fmt::format("SetFrom: {} != {}", new_size, old_size));
  }

  // Update or check the fields of the point cloud(s) if necessary.
  pc_flags::Fields fields_to_copy = fields_in;
  if (fields_in == pc_flags::kInherit) {
    fields_to_copy = other.storage_->fields();
    SetFields(other.storage_->fields(), true);
  } else {
    this->RequireFields(fields_to_copy);
    other.RequireFields(fields_to_copy);
  }

  // Populate data from `other` to this point cloud.
  if (fields_to_copy.contains(pc_flags::kXYZs)) {
    mutable_xyzs() = other.xyzs();
  }
  if (fields_to_copy.contains(pc_flags::kNormals)) {
    mutable_normals() = other.normals();
  }
  if (fields_to_copy.contains(pc_flags::kRGBs)) {
    mutable_rgbs() = other.rgbs();
  }
  if (fields_to_copy.has_descriptor()) {
    mutable_descriptors() = other.descriptors();
  }
}

void PointCloud::Expand(int add_size, bool skip_initialization) {
  DRAKE_DEMAND(add_size >= 0);
  const int new_size = size() + add_size;
  resize(new_size, skip_initialization);
}

bool PointCloud::has_xyzs() const {
  return storage_->fields().contains(pc_flags::kXYZs);
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
  return storage_->fields().contains(pc_flags::kNormals);
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
  return storage_->fields().contains(pc_flags::kRGBs);
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
  return storage_->fields().has_descriptor();
}
bool PointCloud::has_descriptors(
    const pc_flags::DescriptorType& descriptor_type) const {
  return storage_->fields().contains(descriptor_type);
}
const pc_flags::DescriptorType& PointCloud::descriptor_type() const {
  return storage_->fields().descriptor_type();
}
Eigen::Ref<const MatrixX<D>> PointCloud::descriptors() const {
  DRAKE_DEMAND(has_descriptors());
  return storage_->descriptors();
}
Eigen::Ref<MatrixX<D>> PointCloud::mutable_descriptors() {
  DRAKE_DEMAND(has_descriptors());
  return storage_->descriptors();
}

bool PointCloud::HasFields(pc_flags::Fields fields_in) const {
  DRAKE_DEMAND(!fields_in.contains(pc_flags::kInherit));
  return storage_->fields().contains(fields_in);
}

void PointCloud::RequireFields(pc_flags::Fields fields_in) const {
  if (!HasFields(fields_in)) {
    throw std::runtime_error(
        fmt::format("PointCloud does not have expected fields.\n"
                    "Expected {}, got {}",
                    fields_in, storage_->fields()));
  }
}

bool PointCloud::HasExactFields(pc_flags::Fields fields_in) const {
  return storage_->fields() == fields_in;
}

void PointCloud::RequireExactFields(pc_flags::Fields fields_in) const {
  if (!HasExactFields(fields_in)) {
    throw std::runtime_error(
        fmt::format("PointCloud does not have the exact expected fields."
                    "\nExpected {}, got {}",
                    fields_in, storage_->fields()));
  }
}

PointCloud PointCloud::Crop(const Eigen::Ref<const Vector3<T>>& lower_xyz,
                            const Eigen::Ref<const Vector3<T>>& upper_xyz) {
  DRAKE_DEMAND((lower_xyz.array() <= upper_xyz.array()).all());
  if (!has_xyzs()) {
    throw std::runtime_error("PointCloud must have xyzs in order to Crop");
  }
  PointCloud crop(size(), storage_->fields(), true);
  int index = 0;
  for (int i = 0; i < size(); ++i) {
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

void PointCloud::FlipNormalsTowardPoint(
    const Eigen::Ref<const Vector3<T>>& p_CP) {
  DRAKE_THROW_UNLESS(has_xyzs());
  DRAKE_THROW_UNLESS(has_normals());

  for (int i = 0; i < size(); ++i) {
    // Note: p_CP - xyz could be arbitrarily close to zero; but this behavior
    // is still reasonable.
    if ((p_CP - xyz(i)).dot(normal(i)) < 0.0) {
      mutable_normal(i) *= T(-1.0);
    }
  }
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
    if (new_cloud.has_xyzs()) {
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

PointCloud PointCloud::VoxelizedDownSample(
    const double voxel_size, const Parallelism parallelize) const {
  DRAKE_THROW_UNLESS(has_xyzs());
  DRAKE_THROW_UNLESS(voxel_size > 0);

  // Create a dynamic-spatial-hashed voxel grid (DSHVG) to bin points. While a
  // DSHVG usually has each dynamic "chunk" contain multiple voxels, by setting
  // the chunk size to (voxel_size, 1, 1, 1) each chunk contains a single voxel
  // and the whole DSHVG behaves as a sparse voxel grid.
  const GridSizes chunk_sizes(voxel_size, INT64_C(1), INT64_C(1), INT64_C(1));
  const std::vector<int> default_chunk_value;
  // By providing an initial estimated number of chunks, we reduce reallocation
  // and rehashing in the DSHVG.
  const size_t num_expected_chunks = static_cast<size_t>(size() / 16);
  DynamicSpatialHashedVoxelGrid<std::vector<int>> dynamic_voxel_grid(
      chunk_sizes, default_chunk_value, num_expected_chunks);

  const auto& my_xyzs = storage_->xyzs();

  // Add points into the voxel grid.
  for (int i = 0; i < size(); ++i) {
    if (my_xyzs.col(i).array().isFinite().all()) {
      auto chunk_query = dynamic_voxel_grid.GetLocationMutable3d(
          my_xyzs.col(i).cast<double>());
      if (chunk_query) {
        // If the containing chunk has already been allocated, add the current
        // point index directly.
        chunk_query.Value().emplace_back(i);
      } else {
        // If the containing chunk hasn't already been allocated, create a new
        // chunk containing the current point index.
        dynamic_voxel_grid.SetLocation3d(my_xyzs.col(i).cast<double>(),
                                         DSHVGSetType::SET_CHUNK, {i});
      }
    }
  }

  // Initialize downsampled cloud.
  PointCloud down_sampled(
      dynamic_voxel_grid.GetImmutableInternalChunks().size(),
      storage_->fields());

  const bool this_has_normals = has_normals();
  const bool this_has_rgbs = has_rgbs();
  const bool this_has_descriptors = has_descriptors();

  Storage& storage = *storage_;
  Storage& down_sampled_storage = *down_sampled.storage_;
  // Helper lambda to process a single voxel cell.
  const auto process_voxel = [&storage, &down_sampled_storage, this_has_normals,
                              this_has_rgbs, this_has_descriptors](
                                 int index_in_down_sampled,
                                 const std::vector<int>& indices_in_this) {
    // Use doubles instead of floats for accumulators to avoid round-off errors.
    Eigen::Vector3d xyz{Eigen::Vector3d::Zero()};
    Eigen::Vector3d normal{Eigen::Vector3d::Zero()};
    Eigen::Vector3d rgb{Eigen::Vector3d::Zero()};
    Eigen::VectorXd descriptor{Eigen::VectorXd::Zero(
        this_has_descriptors ? storage.descriptors().rows() : 0)};
    int num_normals{0};
    int num_descriptors{0};

    for (int index_in_this : indices_in_this) {
      xyz += storage.xyzs().col(index_in_this).cast<double>();
      if (this_has_normals &&
          storage.normals().col(index_in_this).array().isFinite().all()) {
        normal += storage.normals().col(index_in_this).cast<double>();
        ++num_normals;
      }
      if (this_has_rgbs) {
        rgb += storage.rgbs().col(index_in_this).cast<double>();
      }
      if (this_has_descriptors &&
          storage.descriptors().col(index_in_this).array().isFinite().all()) {
        descriptor += storage.descriptors().col(index_in_this).cast<double>();
        ++num_descriptors;
      }
    }
    down_sampled_storage.xyzs().col(index_in_down_sampled) =
        (xyz / indices_in_this.size()).cast<T>();
    if (this_has_normals) {
      down_sampled_storage.normals().col(index_in_down_sampled) =
          (normal / num_normals).normalized().cast<T>();
    }
    if (this_has_rgbs) {
      down_sampled_storage.rgbs().col(index_in_down_sampled) =
          (rgb / indices_in_this.size()).cast<C>();
    }
    if (this_has_descriptors) {
      down_sampled_storage.descriptors().col(index_in_down_sampled) =
          (descriptor / num_descriptors).cast<D>();
    }
  };

  // Since the parallel form imposes additional overhead, only use it when
  // we are supposed to parallelize.
  [[maybe_unused]] const int num_threads = parallelize.num_threads();
#if defined(_OPENMP)
  const bool operate_in_parallel = num_threads > 1;
#else
  constexpr bool operate_in_parallel = false;
#endif

  // Since we specify chunks contain a single voxel, a chunk's lone voxel can be
  // retrieved with index (0, 0, 0).
  const GridIndex kSingleVoxel(0, 0, 0);

  // Populate the elements of the down_sampled cloud.
  if (operate_in_parallel) {
    // Flatten voxel cells to allow parallel processing.
    std::vector<const std::vector<int>*> voxel_indices;
    voxel_indices.reserve(
        dynamic_voxel_grid.GetImmutableInternalChunks().size());
    for (const auto& [chunk_region, chunk] :
         dynamic_voxel_grid.GetImmutableInternalChunks()) {
      unused(chunk_region);
      const std::vector<int>& indices_in_this =
          chunk.GetIndexImmutable(kSingleVoxel).Value();
      voxel_indices.emplace_back(&indices_in_this);
    }

    // Process voxel cells in parallel.
#if defined(_OPENMP)
#pragma omp parallel for num_threads(num_threads)
#endif
    for (int index_in_down_sampled = 0;
         index_in_down_sampled < static_cast<int>(voxel_indices.size());
         ++index_in_down_sampled) {
      process_voxel(index_in_down_sampled,
                    *voxel_indices[index_in_down_sampled]);
    }
  } else {
    int index_in_down_sampled = 0;
    for (const auto& [chunk_region, chunk] :
         dynamic_voxel_grid.GetImmutableInternalChunks()) {
      unused(chunk_region);
      const std::vector<int>& indices_in_this =
          chunk.GetIndexImmutable(kSingleVoxel).Value();
      process_voxel(index_in_down_sampled, indices_in_this);
      ++index_in_down_sampled;
    }
  }
  return down_sampled;
}

bool PointCloud::EstimateNormals(
    const double radius, const int num_closest,
    [[maybe_unused]] const Parallelism parallelize) {
  DRAKE_DEMAND(radius > 0);
  DRAKE_DEMAND(num_closest >= 3);
  DRAKE_THROW_UNLESS(has_xyzs());
  constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
  const double squared_radius = radius * radius;

  if (!has_normals()) {
    storage_->UpdateFields(storage_->fields() | pc_flags::kNormals);
  }

  const Eigen::MatrixX3f data = xyzs().transpose();
  nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixX3f, 3,
                                      nanoflann::metric_L2_Simple>
      kd_tree(3, data);

  // Iterate through all points and compute their normals.
  std::atomic<bool> all_points_have_at_least_three_neighbors(true);

#if defined(_OPENMP)
#pragma omp parallel for num_threads(parallelize.num_threads())
#endif
  for (int i = 0; i < size(); ++i) {
    VectorX<Eigen::Index> indices(num_closest);
    Eigen::VectorXf distances(num_closest);

    // nanoflann allows two types of queries:
    // 1. search for the num_closest points, and then keep those within radius
    // 2. search for points within radius, and then keep the num_closest
    // for dense clouds where the number of points within radius would be high,
    // approach (1) is considerably faster.
    const int num_neighbors = kd_tree.index_->knnSearch(
        xyz(i).data(), num_closest, indices.data(), distances.data());

    if (num_neighbors < 3) {
      all_points_have_at_least_three_neighbors = false;
    }

    if (num_neighbors < 2) {
      mutable_normal(i) = Eigen::Vector3f::Constant(kNaN);
      continue;
    }

    // Compute the covariance matrix.
    int count = 0;
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();

    for (int j = 0; j < num_neighbors; ++j) {
      if (distances[j] <= squared_radius) {
        ++count;
        mean += xyz(indices[j]).cast<double>();
      }
    }

    if (count < 3) {
      all_points_have_at_least_three_neighbors = false;
    }

    if (count < 2) {
      mutable_normal(i) = Eigen::Vector3f::Constant(kNaN);
      continue;
    }

    mean /= count;

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();

    for (int j = 0; j < num_neighbors; ++j) {
      if (distances[j] <= squared_radius) {
        const Eigen::VectorXd x_minus_mean =
            xyz(indices[j]).cast<double>() - mean;
        covariance += x_minus_mean * x_minus_mean.transpose();
      }
    }

    // TODO(russt): Open3d implements a "FastEigen3x3" for an optimized version
    // of this. We probably should, too.
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver;
    solver.computeDirect(covariance, Eigen::ComputeEigenvectors);
    mutable_normal(i) = solver.eigenvectors().col(0).cast<float>();
  }
  return all_points_have_at_least_three_neighbors.load();
}

}  // namespace perception
}  // namespace drake
