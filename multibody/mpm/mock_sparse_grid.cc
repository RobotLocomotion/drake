#include "drake/multibody/mpm/mock_sparse_grid.h"

#include "drake/math/autodiff.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {

template <typename T>
std::unique_ptr<MockSparseGrid<T>> MockSparseGrid<T>::Clone() const {
  auto clone = std::make_unique<MockSparseGrid<T>>(dx_);
  clone->spgrid_.SetFrom(this->spgrid_);
  clone->grid_data_ = this->grid_data_;
  return clone;
}

template <typename T>
void MockSparseGrid<T>::Allocate(const std::vector<Vector3<T>>& q_WPs) {
  if constexpr (std::is_same_v<T, double>) {
    particle_sorter_.Sort(spgrid_, dx_, q_WPs);
  } else {
    std::vector<Vector3<double>> q_WPs_double;
    q_WPs_double.reserve(q_WPs.size());
    for (const auto& q_WP : q_WPs) {
      q_WPs_double.push_back(math::DiscardGradient(q_WP));
    }
    particle_sorter_.Sort(spgrid_, dx_, q_WPs_double);
  }
  spgrid_.Allocate(particle_sorter_.GetActiveBlockOffsets());
  grid_data_.clear();
  spgrid_.IterateGridWithOffset([&](uint64_t offset, GridData<double>*) {
    const Vector3<int> coordinate = spgrid_.OffsetToCoordinate(offset);
    grid_data_[coordinate] = GridData<T>();
  });
}

template <typename T>
Pad<Vector3<double>> MockSparseGrid<T>::GetPadNodes(
    const Vector3<T>& q_WP) const {
  const auto& q_WP_double = math::DiscardGradient(q_WP);
  Pad<Vector3<double>> result;
  const Vector3<int> base_node = ComputeBaseNode<double>(q_WP_double / dx_);
  for (int i = -1; i <= 1; ++i) {
    for (int j = -1; j <= 1; ++j) {
      for (int k = -1; k <= 1; ++k) {
        const Vector3<int> offset(i, j, k);
        result[i + 1][j + 1][k + 1] = dx_ * (base_node + offset).cast<double>();
      }
    }
  }
  return result;
}

template <typename T>
Pad<GridData<T>> MockSparseGrid<T>::GetPadData(
    uint64_t center_node_offset) const {
  /* Unfortunately we can't call SpGrid::GetPadData here because the data isn't
   actually stored there. */
  const Vector3<int> center_node_coordinate =
      spgrid_.OffsetToCoordinate(center_node_offset);
  Pad<GridData<T>> pad_data;
  for (int i = -1; i <= 1; ++i) {
    const int x = center_node_coordinate.x() + i;
    for (int j = -1; j <= 1; ++j) {
      const int y = center_node_coordinate.y() + j;
      for (int k = -1; k <= 1; ++k) {
        const int z = center_node_coordinate.z() + k;
        const Vector3<int> node_coordinate(x, y, z);
        const auto it = grid_data_.find(node_coordinate);
        if (it != grid_data_.end()) {
          pad_data[i + 1][j + 1][k + 1] = it->second;
        }
      }
    }
  }
  return pad_data;
}

template <typename T>
void MockSparseGrid<T>::SetPadData(uint64_t center_node_offset,
                                   const Pad<GridData<T>>& pad_data) {
  const Vector3<int> center_node_coordinate =
      spgrid_.OffsetToCoordinate(center_node_offset);
  for (int i = -1; i <= 1; ++i) {
    const int x = center_node_coordinate.x() + i;
    for (int j = -1; j <= 1; ++j) {
      const int y = center_node_coordinate.y() + j;
      for (int k = -1; k <= 1; ++k) {
        const int z = center_node_coordinate.z() + k;
        const Vector3<int> node_coordinate(x, y, z);
        grid_data_[node_coordinate] = pad_data[i + 1][j + 1][k + 1];
      }
    }
  }
}

template <typename T>
void MockSparseGrid<T>::SetGridData(
    const std::function<GridData<T>(const Vector3<int>&)>& callback) {
  for (auto& [node, data] : grid_data_) {
    data = callback(node);
  }
}

template <typename T>
std::vector<std::pair<Vector3<int>, GridData<T>>>
MockSparseGrid<T>::GetGridData() const {
  std::vector<std::pair<Vector3<int>, GridData<T>>> result;
  for (const auto& [node, data] : grid_data_) {
    if (data.m > 0.0) result.emplace_back(node, data);
  }
  return result;
}

template <typename T>
MassAndMomentum<T> MockSparseGrid<T>::ComputeTotalMassAndMomentum() const {
  MassAndMomentum<T> result;
  for (const auto& [node, data] : grid_data_) {
    if (data.m > 0.0) {
      const Vector3<double> xi = dx_ * node.template cast<double>();
      result.mass += data.m;
      result.linear_momentum += data.m * data.v;
      result.angular_momentum += data.m * xi.cross(data.v);
    }
  }
  return result;
}

template <typename T>
void MockSparseGrid<T>::IterateGrid(
    const std::function<void(GridData<T>*)>& func) {
  for (auto& [_, data] : grid_data_) {
    func(&data);
  }
}

template <typename T>
void MockSparseGrid<T>::IterateGrid(
    const std::function<void(const GridData<T>&)>& func) const {
  for (const auto& [_, data] : grid_data_) {
    func(data);
  }
}

template <typename T>
contact_solvers::internal::VertexPartialPermutation
MockSparseGrid<T>::SetNodeIndices() {
  std::vector<int> participating_nodes;
  int node_index = 0;
  int participating_node_index = 0;
  auto index_grid = [&](GridData<T>* data) {
    if (data->m > 0.0) {
      if (data->index_or_flag.is_flag()) {
        participating_nodes.push_back(participating_node_index++);
      } else {
        participating_nodes.push_back(-1);
      }
      data->index_or_flag.set_index(node_index++);
    }
  };
  IterateGrid(index_grid);
  return contact_solvers::internal::VertexPartialPermutation(
      std::move(participating_nodes));
}

}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake

template class drake::multibody::mpm::internal::MockSparseGrid<double>;
template class drake::multibody::mpm::internal::MockSparseGrid<
    drake::AutoDiffXd>;
