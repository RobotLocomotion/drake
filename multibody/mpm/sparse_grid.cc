#include "sparse_grid.h"

#include <utility>

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {

template <typename T, int log2_max_grid_size>
SparseGrid<T, log2_max_grid_size>::SparseGrid(double dx) : dx_(dx) {
  DRAKE_DEMAND(dx > 0);
}

template <typename T, int log2_max_grid_size>
std::unique_ptr<SparseGrid<T, log2_max_grid_size>>
SparseGrid<T, log2_max_grid_size>::Clone() const {
  auto result = std::make_unique<SparseGrid<T, log2_max_grid_size>>(dx_);
  result->spgrid_.SetFrom(this->spgrid_);
  return result;
}

template <typename T, int log2_max_grid_size>
void SparseGrid<T, log2_max_grid_size>::Allocate(
    const std::vector<Vector3<T>>& q_WPs) {
  particle_sorter_.Sort(spgrid_, dx_, q_WPs);
  spgrid_.Allocate(particle_sorter_.GetActiveBlockOffsets());
}

template <typename T, int log2_max_grid_size>
Pad<Vector3<T>> SparseGrid<T, log2_max_grid_size>::GetPadNodes(
    const Vector3<T>& q_WP) const {
  Pad<Vector3<T>> result;
  const Vector3<int> base_node = ComputeBaseNode<T>(q_WP / static_cast<T>(dx_));
  for (int i = -1; i <= 1; ++i) {
    for (int j = -1; j <= 1; ++j) {
      for (int k = -1; k <= 1; ++k) {
        const Vector3<int> offset(i, j, k);
        result[i + 1][j + 1][k + 1] =
            static_cast<T>(dx_) * (base_node + offset).cast<T>();
      }
    }
  }
  return result;
}

template <typename T, int log2_max_grid_size>
void SparseGrid<T, log2_max_grid_size>::SetGridData(
    const std::function<GridData<T>(const Vector3<int>&)>& callback) {
  spgrid_.IterateGridWithOffset([&](uint64_t offset, GridData<T>* node_data) {
    const Vector3<int> coordinate = spgrid_.OffsetToCoordinate(offset);
    *node_data = callback(coordinate);
  });
}

template <typename T, int log2_max_grid_size>
std::vector<std::pair<Vector3<int>, GridData<T>>>
SparseGrid<T, log2_max_grid_size>::GetGridData() const {
  std::vector<std::pair<Vector3<int>, GridData<T>>> result;
  spgrid_.IterateConstGridWithOffset(
      [&](uint64_t offset, const GridData<T>& node_data) {
        if (!node_data.is_inactive()) {
          const Vector3<int> coordinate = spgrid_.OffsetToCoordinate(offset);
          result.emplace_back(coordinate, node_data);
        }
      });
  return result;
}

template <typename T, int log2_max_grid_size>
MassAndMomentum<T>
SparseGrid<T, log2_max_grid_size>::ComputeTotalMassAndMomentum() const {
  MassAndMomentum<T> result;
  spgrid_.IterateConstGridWithOffset(
      [&](uint64_t offset, const GridData<T>& node_data) {
        if (node_data.m > 0.0) {
          const Vector3<T>& xi =
              spgrid_.OffsetToCoordinate(offset).template cast<T>() *
              static_cast<T>(dx_);
          result.mass += node_data.m;
          result.linear_momentum += node_data.m * node_data.v;
          result.angular_momentum += node_data.m * xi.cross(node_data.v);
        }
      });
  return result;
}

}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake

template class drake::multibody::mpm::internal::SparseGrid<float, 10>;
template class drake::multibody::mpm::internal::SparseGrid<double, 10>;
template class drake::multibody::mpm::internal::SparseGrid<float, 5>;
template class drake::multibody::mpm::internal::SparseGrid<double, 5>;
