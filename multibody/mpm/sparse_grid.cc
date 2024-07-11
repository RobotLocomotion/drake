#include "sparse_grid.h"

#include <iostream>

#include "parallel_stable_sort/openmp/parallel_stable_sort.h"
#include <omp.h>
#include <tbb/parallel_sort.h>

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {

template <typename T>
SparseGrid<T>::SparseGrid(T dx)
    : dx_(dx),
      allocator_(std::make_unique<Allocator>(kMaxGridSize, kMaxGridSize,
                                             kMaxGridSize)),
      blocks_(std::make_unique<PageMap>(*allocator_)),
      padded_blocks_(std::make_unique<PageMap>(*allocator_)) {
  DRAKE_DEMAND(dx > 0);
  /* Compute the block offset strides. */
  const int num_nodes_in_block_x = 1 << Mask::block_xbits;
  const int num_nodes_in_block_y = 1 << Mask::block_ybits;
  const int num_nodes_in_block_z = 1 << Mask::block_zbits;
  for (int i = -1; i <= 1; ++i) {
    for (int j = -1; j <= 1; ++j) {
      for (int k = -1; k <= 1; ++k) {
        block_offset_strides_[i + 1][j + 1][k + 1] = Mask::Linear_Offset(
            i * num_nodes_in_block_x, j * num_nodes_in_block_y,
            k * num_nodes_in_block_z);
      }
    }
  }
  /* Compute the neighbor offset strides. */
  for (int i = -1; i <= 1; ++i) {
    for (int j = -1; j <= 1; ++j) {
      for (int k = -1; k <= 1; ++k) {
        neighbor_offset_strides_[i + 1][j + 1][k + 1] =
            Mask::Linear_Offset(i, j, k);
      }
    }
  }

  /* Maintain the invariance that the last entry in sentinel_particles_ is the
   number of particles. */
  sentinel_particles_.push_back(0);
}

template <typename T>
void SparseGrid<T>::Allocate(ParticleData<T>* particles) {
  SortParticleIndices(particles);
  blocks_->Clear();
  padded_blocks_->Clear();

  /* Touch all pages that contain particles. */
  for (int i = 0; i < ssize(sentinel_particles_) - 1; ++i) {
    blocks_->Set_Page(particles_[sentinel_particles_[i]].base_node_offset);
  }
  blocks_->Update_Block_Offsets();
  auto [block_offsets, num_blocks] = blocks_->Get_Blocks();
  /* Touch all neighbor pages of each page in `blocks_`. */
  for (int b = 0; b < static_cast<int>(num_blocks); ++b) {
    const uint64_t current_offset = block_offsets[b];
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        for (int k = 0; k < 3; ++k) {
          const uint64_t neighbor_offset =
              Mask::Packed_Add(current_offset, block_offset_strides_[i][j][k]);
          padded_blocks_->Set_Page(neighbor_offset);
        }
      }
    }
  }
  padded_blocks_->Update_Block_Offsets();
  std::tie(block_offsets, num_blocks) = padded_blocks_->Get_Blocks();
  const uint64_t page_size = 1 << kLog2Page;
  const uint64_t data_size = sizeof(GridData<T>);
  /* Note that Array is a wrapper around pointer to data memory and can be
   cheaply copied. */
  Array data = allocator_->Get_Array();
  /* Zero out the data in each block. */
  for (int b = 0; b < static_cast<int>(num_blocks); ++b) {
    const uint64_t offset = block_offsets[b];
    for (uint64_t i = 0; i < page_size; i += data_size) {
      data(offset + i).set_zero();
    }
  }
}

template <typename T>
void SparseGrid<T>::SetGridState(
    const std::function<GridData<T>(const Vector3<T>&)>& callback) {
  auto [block_offsets, num_blocks] = padded_blocks_->Get_Blocks();
  const uint64_t page_size = 1 << kLog2Page;
  const uint64_t data_size = sizeof(GridData<T>);
  Array data = allocator_->Get_Array();
  for (int b = 0; b < static_cast<int>(num_blocks); ++b) {
    const uint64_t offset = block_offsets[b];
    for (uint64_t i = 0; i < page_size; i += data_size) {
      const Vector3<int> coordinate = OffsetToCoordinate(offset + i);
      const Vector3<T> xi_W = dx_ * coordinate.cast<T>();
      data(offset + i) = callback(xi_W);
    }
  }
}

template <typename T>
NeighborArray<Vector3<T>> SparseGrid<T>::GetNeighborNodes(
    const Vector3<T>& x) const {
  NeighborArray<Vector3<T>> result;
  const Vector3<int> base_node = mpm::internal::base_node<T>(x / dx_);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        const Vector3<int> offset(i - 1, j - 1, k - 1);
        result[i][j][k] = dx_ * (base_node + offset).cast<T>();
      }
    }
  }
  return result;
}

template <typename T>
NeighborArray<GridData<T>> SparseGrid<T>::GetNeighborData(
    uint64_t base_node_offset) const {
  NeighborArray<GridData<T>> result;
  ConstArray data = allocator_->Get_Const_Array();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        const uint64_t offset = Mask::Packed_Add(
            base_node_offset, neighbor_offset_strides_[i][j][k]);
        result[i][j][k] = data(offset);
      }
    }
  }
  return result;
}

template <typename T>
void SparseGrid<T>::SetNeighborData(uint64_t base_node_offset,
                                    const NeighborArray<GridData<T>>& data) {
  Array grid_data = allocator_->Get_Array();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        const uint64_t offset = Mask::Packed_Add(
            base_node_offset, neighbor_offset_strides_[i][j][k]);
        grid_data(offset) = data[i][j][k];
      }
    }
  }
}

template <typename T>
Vector3<int> SparseGrid<T>::OffsetToCoordinate(uint64_t offset) const {
  const std::array<int, 3> reference_space_coordinate =
      Mask::LinearToCoord(offset);
  const std::array<int, 3> reference_space_origin =
      Mask::LinearToCoord(origin_offset_);
  return Vector3<int>(
      reference_space_coordinate[0] - reference_space_origin[0],
      reference_space_coordinate[1] - reference_space_origin[1],
      reference_space_coordinate[2] - reference_space_origin[2]);
}

/* Sort particles by base node offsets so that particles that are close to
 each other in physical space are close to each other in memory. */
template <typename T>
void SparseGrid<T>::Sort(std::vector<ParticleIndex>* particles) {
  pss::parallel_stable_sort(particles->begin(), particles->end(),
                            [](const ParticleIndex& a, const ParticleIndex& b) {
                              if (a.base_node_offset == b.base_node_offset) {
                                return a.index < b.index;
                              }
                              return a.base_node_offset < b.base_node_offset;
                            });
}

template <typename T>
void SparseGrid<T>::SortParticleIndices(ParticleData<T>* data) {
  const std::vector<Vector3<T>>& particle_positions = data->x;
  const int num_particles = particle_positions.size();
  particles_.resize(num_particles);
#pragma omp parallel for
  for (int p = 0; p < num_particles; ++p) {
    const Vector3<int> base_node =
        mpm::internal::base_node<T>(particle_positions[p] / dx_);
    // TODO(xuchenhan-tri): We are computing base nodes twice. We should cut it
    // down to once.
    data->bspline[p] = BSplineWeights<T>(particle_positions[p], dx_);
    particles_[p].base_node_offset =
        CoordinateToOffset(base_node[0], base_node[1], base_node[2]);
    particles_[p].index = p;
  }
  Sort(&particles_);

  /* We use sentinel_particles_ to indicate particles that belong to separate
   pages. */
  sentinel_particles_.clear();
  for (int b = 0; b < 8; ++b) {
    colored_pages_[b].clear();
  }
  uint64_t last_page = 0;
  int block = 0;
  for (int p = 0; p < num_particles; ++p) {
    /* The bits in the offset is ordered as follows:

      page bits | block bits | data bits

     block bits and data bits add up to kLog2Page bits.
     We right shift to get the page bits. */
    const uint64_t page = particles_[p].base_node_offset >> kLog2Page;
    if (p == 0 || last_page != page) {
      last_page = page;
      sentinel_particles_.push_back(p);
      int color = get_color(page);
      colored_pages_[color].push_back(block++);
    }
  }
  sentinel_particles_.push_back(num_particles);
}

}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake

template class drake::multibody::mpm::internal::SparseGrid<float>;
template class drake::multibody::mpm::internal::SparseGrid<double>;