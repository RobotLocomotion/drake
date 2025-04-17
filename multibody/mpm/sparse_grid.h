#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/multibody/contact_solvers/sap/partial_permutation.h"
#include "drake/multibody/mpm/grid_data.h"
#include "drake/multibody/mpm/mass_and_momentum.h"
#include "drake/multibody/mpm/particle_data.h"
#include "drake/multibody/mpm/particle_sorter.h"
#include "drake/multibody/mpm/spgrid.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {

/* Note: MockSparseGrid mirrors the public APIs in SparseGrid to enable
 derivative testing. Changes in this class's public API should be
 reflected in MocSparseGrid. */

/* SparseGrid is a 3D grid that is sparsely populated with GridData (see
 multibody::mpm::internal::GridData) implemented with a Sparse Paged Grid
 (SpGrid) data structure (see multibody::mpm::internal::SpGrid). The SparseGrid
 class is used to allocate, set, and access grid data in an MPM simulation.
 Usually, there is only one SparseGrid in an entire MPM simulation, and all
 particles interact with this grid. The MPM algorithm should use this class to
 perform operations like particle-to-grid (P2G) and grid-to-particle (G2P)
 transfers, instead of directly accessing underlying SpGrid.

 Here we recall a few terminologies (defined elsewhere) and define a few new
 ones that are used in the documentation below.

 Recall that a grid node is the "base node" of a particle if it's the closest
 grid node to the particle. Also recall that a "pad" in a grid is a 3x3x3
 subgrid. We define that a grid node is "in the support" of a particle if it
 belongs to a pad whose center node is the base node of the particle. A grid
 node is said to be "supported" if it is in the support of at least one
 particle.

 Recall that a "block" in an SpGrid is a rectangular subgrid that occupies a
 continuous chunk of memory of size 4kB (a "page"). The SparseGrid class
 allocates memory in units of these blocks; it allocates for a block if it might
 contain a supported grid node. See `Allocate` for more details.

 @tparam T float or double. */
template <typename T>
class SparseGrid {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SparseGrid);

  using Scalar = T;
  using NodeScalarType = T;
  using NodeType = Vector3<T>;
  using PadNodeType = Pad<NodeType>;
  using PadDataType = Pad<GridData<T>>;

  /* Constructs a SparseGrid with grid spacing `dx` in meters.
   @pre dx > 0. */
  explicit SparseGrid(double dx);

  /* Creates a deep copy of `this` sparse grid. */
  std::unique_ptr<SparseGrid<T>> Clone() const;

  /* Allocates memory for all blocks that may contain at least one supported
   grid node. All allocated grid data are default constructed.
   @note With T == float, GridData is 32 bytes. With T == double, GridData is 64
         bytes. Since each block is 4kB, each allocated block contains 4*4*8 =
         128 grid nodes (float) or 4*4*4 = 64 grid nodes (double).
   @note Since we only allocate with granularity of blocks, to ensure that all
         the supported grid nodes are allocated, we allocate the block that
         contains the particle and the one ring of blocks around it.
   @param[in] q_WPs  The world frame positions of all MPM particles. */
  void Allocate(const std::vector<Vector3<T>>& q_WPs);

  /* Grid spacing in meters. */
  double dx() const { return dx_; }

  /* Given the position of a particle in the world frame, returns the world
   frame positions of the grid nodes in its support. */
  Pad<Vector3<T>> GetPadNodes(const Vector3<T>& q_WP) const;

  /* Given the SpGrid offset of a grid node, returns the grid data in the pad
   with the given node at the center.
   @pre All nodes in the requested pad are allocatetd. */
  Pad<GridData<T>> GetPadData(uint64_t center_node_offset) const {
    return spgrid_.GetPadData(center_node_offset);
  }

  /* Given the offset (see multibody::mpm::internal::SpGrid) of a grid node,
   writes the grid data in the pad with the given node at the center.
   @pre All nodes in the requested pad are allocated. */
  void SetPadData(uint64_t center_node_offset,
                  const Pad<GridData<T>>& pad_data) {
    spgrid_.SetPadData(center_node_offset, pad_data);
  }

  /* Sets the grid state with the given callback function that maps the world
   space coordinate of the grid node to the data at that node.
   @note The callback is only applied to the allocated grid nodes.
   @note Testing only. */
  void SetGridData(
      const std::function<GridData<T>(const Vector3<int>&)>& callback);

  /* Returns grid data from all grid nodes with positive mass as a pair of world
   space coordinate and grid data of the node.
   @note Testing only. */
  std::vector<std::pair<Vector3<int>, GridData<T>>> GetGridData() const;

  /* Computes the mass, linear momentum, and angular momentum (about world
   origin) on the grid. */
  MassAndMomentum<T> ComputeTotalMassAndMomentum() const;

  /* Returns the SpGrid underlying this SparseGrid. */
  const SpGrid<GridData<T>>& spgrid() const { return spgrid_; }

  /* Returns the number of blocks allocated in the SpGrid. */
  int num_blocks() const { return spgrid_.num_blocks(); }

  /* Iterates over all particles and the grid nodes supported by them, applying
   the given kernel.

   This method traverses every particle in the provided ParticleData instance.
   For each particle, it:
     - Retrieves the pad of grid nodes and grid data that the particle supports.
     - Invokes the provided kernel function with:
       - The index of the current particle.
       - A const reference to the positions of the pad of grid nodes supported
         by the particle.
       - A const reference to the pad of grid data.
       - A pointer to the particle data to be modified by the kernel.
   Note that this kernel should only read/modify the particle data with the
   given particle index, and it should not assume an ordering of how the
   particles are iterated.

   @param[in,out] particle_data  Pointer to the particle data to iterate over.
                                 As input, this provides the particle position
                                 to locate the relevant grid pad as well as the
                                 current particle state to be used in the
                                 kernel. As output, this provides the particle
                                 data to be modified by the kernel. This data
                                 may be modified by the kernel.
   @param[in] kernel             The grid-to-particle kernel to apply.
   @pre The grid's Allocate() method must have been called with the positions
   contained in the given particle_data. */
  void ApplyGridToParticleKernel(
      ParticleData<T>* particle_data,
      const std::function<void(int, const Pad<Vector3<T>>&,
                               const Pad<GridData<T>>&, ParticleData<T>*)>&
          kernel) const {
    particle_sorter_.Iterate(this, particle_data, kernel);
  }

  /* Iterates over all particles and the grid nodes supported by them, applying
   the given kernel.

   This method traverses every particle in the provided ParticleData instance.
   For each particle, it:
     - Retrieves the pad of grid nodes and grid data that the particle supports.
     - Invokes the provided kernel function with:
       - The index of the current particle.
       - A const reference to the positions of the pad of grid nodes supported
         by the particle.
       - A const reference to the pad of grid data.
       - A const reference to the particle data.
   Note that this kernel should only read the particle data with the given
   particle index, and it should not assume an ordering of how the particles are
   iterated.

   @param[in] particle_data  Pointer to the particle data to iterate over. It
                             provides the particle position to locate the
                             relevant grid pad as well as the current particle
                             state to be used in the kernel.
   @param[in] kernel         The grid-to-particle kernel to apply.
   @pre The grid's Allocate() method must have been called with the positions
   contained in the given particle_data. */
  void IterateParticleAndGrid(
      const ParticleData<T>& particle_data,
      const std::function<void(int, const Pad<Vector3<T>>&,
                               const Pad<GridData<T>>&,
                               const ParticleData<T>&)>& kernel) const {
    particle_sorter_.Iterate(this, &particle_data, kernel);
  }

  /* Iterates over all particles and the grid nodes supported by them,
   applying the given kernel, and writes back the updated grid data.

   This method is similar to ApplyGridToParticleKernel(), but it is intended for
   cases where the particle data is read-only and the grid data is mutable.

   This method traverses every particle in the provided ParticleData instance.
   For each particle, it:
    - Retrieves the pad of grid nodes and grid data that the particle supports.
    - Invokes the provided kernel function with:
      - The index of the current particle.
      - A const reference to the positions of the pad of grid nodes supported by
        the particle.
      - A const reference to the particle data.
      - A pointer to the pad of grid data to be modified.
   Note that this kernel should only read the particle data with the given
   particle index, and it should not assume an ordering of how the particles are
   iterated.

   The updated grid data on each pad is written back automatically to the grid
   once all particles supporting the pad have been processed.

   @param[in] particle_data  A const reference to the particle data to iterate
                             over.
   @param[in] kernel         The particle-to-grid kernel to apply.
   @post The grid data corresponding to each particle's support is updated with
   any modifications performed by the kernel.
   @pre The grid's Allocate() method must have been called with the
   positions contained in the given particle_data. */
  void ApplyParticleToGridKernel(
      const ParticleData<T>& particle_data,
      const std::function<void(int, const Pad<Vector3<T>>&,
                               const ParticleData<T>&, Pad<GridData<T>>*)>&
          kernel) {
    particle_sorter_.Iterate(this, &particle_data, kernel);
  }

  /* Iterates over all grid nodes in the grid and applies the given function
   `func` to each grid node. */
  void IterateGrid(const std::function<void(GridData<T>*)>& func) {
    spgrid_.IterateGrid(func);
  }

  /* Iterates over all grid nodes in the grid and applies the given function
   `func` to each grid node. */
  void IterateGrid(const std::function<void(const GridData<T>&)>& func) const {
    spgrid_.IterateGrid(func);
  }

  /* Sets the indices of all active nodes in the grid and returns a
   VertexPartialPermutation that maps the grid indices/dofs to the permuted grid
   indices/dofs. With the permuted grid indices, grid nodes/dofs participating
   in constraints come before nodes those don't participate in any constraint.
   Nodes are marked as "participating" during P2G in Transfer::ParticleToGrid().
   A node is marked as participating whenever it is in the support of a particle
   participating in a constraint. Refer to Transfer::ParticleToGrid() for
   details. */
  contact_solvers::internal::VertexPartialPermutation SetNodeIndices();

 private:
  /* Grid spacing (in meters). */
  double dx_{};
  SpGrid<GridData<T>> spgrid_;
  ParticleSorter particle_sorter_;
};

}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
