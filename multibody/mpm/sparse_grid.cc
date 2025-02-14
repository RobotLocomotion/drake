#include "sparse_grid.h"

#include <utility>

#include "drake/common/ssize.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {

template <typename T>
SparseGrid<T>::SparseGrid(double dx) : dx_(dx) {
  DRAKE_DEMAND(dx > 0);
}

template <typename T>
void SparseGrid<T>::Allocate(const ParticleSorter& particles) {
  spgrid_.Allocate(particles.GetActiveBlockOffsets());
}

template <typename T>
Pad<Vector3<T>> SparseGrid<T>::GetPadNodes(const Vector3<T>& q_WP) const {
  Pad<Vector3<T>> result;
  const Vector3<int> base_node = ComputeBaseNode<T>(q_WP / static_cast<T>(dx_));
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        const Vector3<int> offset(i - 1, j - 1, k - 1);
        result[i][j][k] = static_cast<T>(dx_) * (base_node + offset).cast<T>();
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
  spgrid_.IterateConstGridWithOffset(
      [&](uint64_t offset, const GridData<T>& node_data) {
        if (!node_data.is_inactive()) {
          const Vector3<int> coordinate = spgrid_.OffsetToCoordinate(offset);
          result.emplace_back(std::make_pair(coordinate, node_data));
        }
      });
  return result;
}

template <typename T>
MassAndMomentum<T> SparseGrid<T>::ComputeTotalMassAndMomentum() const {
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

template class drake::multibody::mpm::internal::SparseGrid<float>;
template class drake::multibody::mpm::internal::SparseGrid<double>;
