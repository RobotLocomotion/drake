#include "drake/multibody/mpm/sparse_grid.h"

#include <utility>

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {

template <typename T>
SparseGrid<T>::SparseGrid(double dx) : dx_(dx) {
  DRAKE_DEMAND(dx > 0);
}

template <typename T>
std::unique_ptr<SparseGrid<T>> SparseGrid<T>::Clone() const {
  auto result = std::make_unique<SparseGrid<T>>(dx_);
  result->spgrid_.SetFrom(this->spgrid_);
  return result;
}

template <typename T>
void SparseGrid<T>::Allocate(const std::vector<Vector3<T>>& q_WPs) {
  particle_sorter_.Sort(spgrid_, dx_, q_WPs);
  spgrid_.Allocate(particle_sorter_.GetActiveBlockOffsets());
}

template <typename T>
Pad<Vector3<T>> SparseGrid<T>::GetPadNodes(const Vector3<T>& q_WP) const {
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

template <typename T>
void SparseGrid<T>::SetGridData(
    const std::function<GridData<T>(const Vector3<int>&)>& callback) {
  spgrid_.IterateGridWithOffset([&](uint64_t offset, GridData<T>* node_data) {
    const Vector3<int> coordinate = spgrid_.OffsetToCoordinate(offset);
    *node_data = callback(coordinate);
  });
}

template <typename T>
std::vector<std::pair<Vector3<int>, GridData<T>>> SparseGrid<T>::GetGridData()
    const {
  std::vector<std::pair<Vector3<int>, GridData<T>>> result;
  spgrid_.IterateGridWithOffset(
      [&](uint64_t offset, const GridData<T>& node_data) {
        if (node_data.m > 0.0) {
          const Vector3<int> coordinate = spgrid_.OffsetToCoordinate(offset);
          result.emplace_back(coordinate, node_data);
        }
      });
  return result;
}

template <typename T>
MassAndMomentum<T> SparseGrid<T>::ComputeTotalMassAndMomentum() const {
  MassAndMomentum<T> result;
  spgrid_.IterateGridWithOffset(
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

template <typename T>
contact_solvers::internal::VertexPartialPermutation
SparseGrid<T>::SetNodeIndices() {
  std::vector<int> participating_nodes;
  int node_index = 0;
  int participating_node_index = 0;
  spgrid_.IterateGrid([&](GridData<T>* node_data) {
    if (node_data->m > 0.0) {
      if (node_data->index_or_flag.is_flag()) {
        participating_nodes.push_back(participating_node_index++);
      } else {
        participating_nodes.push_back(-1);
      }
      node_data->index_or_flag.set_index(node_index++);
    }
  });
  return contact_solvers::internal::VertexPartialPermutation(
      std::move(participating_nodes));
}

}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake

template class drake::multibody::mpm::internal::SparseGrid<float>;
template class drake::multibody::mpm::internal::SparseGrid<double>;
