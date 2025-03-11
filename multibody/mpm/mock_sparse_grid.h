#pragma once

#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/multibody/mpm/grid_data.h"
#include "drake/multibody/mpm/mass_and_momentum.h"
#include "drake/multibody/mpm/particle_data.h"
#include "drake/multibody/mpm/particle_sorter.h"
#include "drake/multibody/mpm/spgrid.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {

/* A testing-only implementation that mirrors the SparseGrid interface.

 This class mimics the set of functions provided by SparseGrid and is intended
 solely for unit testing. It internally uses an `std::map<Vector3d,
 GridState<T>>` to manage grid data and supports both `double` and `AutoDiffXd`
 types, facilitating derivative tests involving the grid.

 Although the grid data is not stored using SpGrid, this class retains SpGrid
 functionality to convert `uint64_t` offsets to `Vector3<int>` coordinates.

 @tparam double or AutoDiffXd. */
template <typename T>
class MockSparseGrid {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockSparseGrid);

  using Scalar = T;
  using NodeScalarType = double;
  using NodeType = Vector3<NodeScalarType>;
  using PadNodeType = Pad<NodeType>;
  using PadDataType = Pad<GridData<T>>;

  explicit MockSparseGrid(double dx) : dx_(dx) {}

  std::unique_ptr<MockSparseGrid<T>> Clone() const;

  void Allocate(const std::vector<Vector3<T>>& q_WPs);

  double dx() const { return dx_; }

  Pad<Vector3<double>> GetPadNodes(const Vector3<T>& q_WP) const;

  Pad<GridData<T>> GetPadData(uint64_t center_node_offset) const;

  void SetPadData(uint64_t center_node_offset,
                  const Pad<GridData<T>>& pad_data);

  void SetGridData(
      const std::function<GridData<T>(const Vector3<int>&)>& callback);

  std::vector<std::pair<Vector3<int>, GridData<T>>> GetGridData() const;

  MassAndMomentum<T> ComputeTotalMassAndMomentum() const;

  const SpGrid<GridData<double>>& spgrid() const { return spgrid_; }

  int num_blocks() const { return spgrid_.num_blocks(); }

  void ApplyGridToParticleKernel(
      ParticleData<T>* particle_data,
      const std::function<void(int, const Pad<Vector3<double>>&,
                               const Pad<GridData<T>>&, ParticleData<T>*)>&
          kernel) const {
    particle_sorter_.Iterate(this, particle_data, kernel);
  }

  void ApplyParticleToGridKernel(
      const ParticleData<T>& particle_data,
      const std::function<void(int, const Pad<Vector3<double>>&,
                               const ParticleData<T>&, Pad<GridData<T>>*)>&
          kernel) {
    particle_sorter_.Iterate(this, &particle_data, kernel);
  }

 private:
  double dx_{};
  SpGrid<GridData<double>>
      spgrid_;  // used only for offset conversion and sorting particles, but
                // not for storing the grid data.
  ParticleSorter particle_sorter_;

  /* Lexicographical comparison for Vector3. */
  struct Vector3Comparator {
    bool operator()(const Vector3<int>& lhs, const Vector3<int>& rhs) const {
      if (lhs.x() != rhs.x()) return lhs.x() < rhs.x();
      if (lhs.y() != rhs.y()) return lhs.y() < rhs.y();
      return lhs.z() < rhs.z();
    }
  };
  // Actual grid data.
  std::map<Vector3<int>, GridData<T>, Vector3Comparator> grid_data_;
};

}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
