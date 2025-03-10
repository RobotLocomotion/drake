#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/multibody/mpm/grid_data.h"
#include "drake/multibody/mpm/mass_and_momentum.h"
#include "drake/multibody/mpm/particle_sorter.h"
#include "drake/multibody/mpm/spgrid.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {

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

  /* Returns grid data from all non-inactive grid nodes (see
   GridData::is_inactive) as a pair of world space coordinate and grid data of
   the node.
   @note Testing only. */
  std::vector<std::pair<Vector3<int>, GridData<T>>> GetGridData() const;

  /* Computes the mass, linear momentum, and angular momentum (about world
   origin) on the grid. */
  MassAndMomentum<T> ComputeTotalMassAndMomentum() const;

  /* Returns the SpGrid underlying this SparseGrid. */
  const SpGrid<GridData<T>>& spgrid() const { return spgrid_; }

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
