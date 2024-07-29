#include "sparse_grid.h"

#include <utility>

#include "ips2ra/ips2ra.hpp"

#if defined(_OPENMP)
#include <omp.h>
#endif

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {

template <typename T>
SparseGrid<T>::SparseGrid(T dx, Parallelism parallelism)
    : dx_(dx),
      allocator_(std::make_unique<Allocator>(kMaxGridSize, kMaxGridSize,
                                             kMaxGridSize)),
      blocks_(std::make_unique<PageMap>(*allocator_)),
      padded_blocks_(std::make_unique<PageMap>(*allocator_)),
      parallelism_(parallelism) {
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
  /* Compute the cell offset strides. */
  for (int i = -1; i <= 1; ++i) {
    for (int j = -1; j <= 1; ++j) {
      for (int k = -1; k <= 1; ++k) {
        cell_offset_strides_[i + 1][j + 1][k + 1] =
            Mask::Linear_Offset(i, j, k);
      }
    }
  }

  /* Maintain the invariance that the last entry in sentinel_particles_ is the
   number of particles. */
  sentinel_particles_.push_back(0);
}

template <typename T>
void SparseGrid<T>::Allocate(const std::vector<Vector3<T>>& q_WPs) {
  SortParticleIndices(q_WPs);
  blocks_->Clear();
  padded_blocks_->Clear();

  /* Touch all blocks that contain particles. */
  for (int i = 0; i < ssize(sentinel_particles_) - 1; ++i) {
    blocks_->Set_Page(base_node_offsets_[sentinel_particles_[i]]);
  }
  blocks_->Update_Block_Offsets();
  auto [block_offsets, num_blocks] = blocks_->Get_Blocks();
  /* Touch all neighboring blocks of each block in `blocks_` to ensure all grid
   nodes that might be affected by a particle has memory allocated. */
  for (int b = 0; b < static_cast<int>(num_blocks); ++b) {
    const uint64_t current_offset = block_offsets[b];
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        for (int k = 0; k < 3; ++k) {
          const uint64_t neighbor_block_offset =
              Mask::Packed_Add(current_offset, block_offset_strides_[i][j][k]);
          padded_blocks_->Set_Page(neighbor_block_offset);
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
Pad<Vector3<T>> SparseGrid<T>::GetPadNodes(const Vector3<T>& q_WP) const {
  Pad<Vector3<T>> result;
  const Vector3<int> base_node = ComputeBaseNode<T>(q_WP / dx_);
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
Pad<GridData<T>> SparseGrid<T>::GetPadData(uint64_t center_node_offset) const {
  Pad<GridData<T>> result;
  ConstArray data = allocator_->Get_Const_Array();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        const uint64_t offset =
            Mask::Packed_Add(center_node_offset, cell_offset_strides_[i][j][k]);
        result[i][j][k] = data(offset);
      }
    }
  }
  return result;
}

template <typename T>
void SparseGrid<T>::SetPadData(uint64_t center_node_offset,
                               const Pad<GridData<T>>& pad_data) {
  Array grid_data = allocator_->Get_Array();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        const uint64_t offset =
            Mask::Packed_Add(center_node_offset, cell_offset_strides_[i][j][k]);
        grid_data(offset) = pad_data[i][j][k];
      }
    }
  }
}

template <typename T>
void SparseGrid<T>::ExplicitVelocityUpdate(const Vector3<T>& dv) {
  const uint64_t page_size = 1 << kLog2Page;
  const uint64_t data_size = sizeof(GridData<T>);
  auto [block_offsets, num_blocks] = padded_blocks_->Get_Blocks();
  Array grid_data = allocator_->Get_Array();
  for (int b = 0; b < static_cast<int>(num_blocks); ++b) {
    const uint64_t block_offset = block_offsets[b];
    for (uint64_t i = 0; i < page_size; i += data_size) {
      const uint64_t node_offset = block_offset + i;
      GridData<T>& node_data = grid_data(node_offset);
      if (node_data.m > 0.0) {
        node_data.v /= node_data.m;
        node_data.v += dv;
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

template <typename T>
void SparseGrid<T>::SetGridData(
    const std::function<GridData<T>(const Vector3<int>&)>& callback) {
  auto [block_offsets, num_blocks] = padded_blocks_->Get_Blocks();
  const uint64_t page_size = 1 << kLog2Page;
  const uint64_t data_size = sizeof(GridData<T>);
  Array data = allocator_->Get_Array();
  for (int b = 0; b < static_cast<int>(num_blocks); ++b) {
    const uint64_t offset = block_offsets[b];
    for (uint64_t i = 0; i < page_size; i += data_size) {
      const Vector3<int> coordinate = OffsetToCoordinate(offset + i);
      data(offset + i) = callback(coordinate);
    }
  }
}

template <typename T>
std::vector<std::pair<Vector3<int>, GridData<T>>> SparseGrid<T>::GetGridData()
    const {
  const uint64_t page_size = 1 << kLog2Page;
  const uint64_t data_size = sizeof(GridData<T>);
  std::vector<std::pair<Vector3<int>, GridData<T>>> result;
  auto [block_offsets, num_blocks] = padded_blocks_->Get_Blocks();
  ConstArray grid_data = allocator_->Get_Const_Array();
  for (int b = 0; b < static_cast<int>(num_blocks); ++b) {
    const uint64_t block_offset = block_offsets[b];
    for (uint64_t i = 0; i < page_size; i += data_size) {
      const uint64_t node_offset = block_offset + i;
      const GridData<T>& node_data = grid_data(node_offset);
      if (node_data.m > 0.0) {
        const Vector3<int> node_coordinate = OffsetToCoordinate(node_offset);
        result.emplace_back(std::make_pair(node_coordinate, node_data));
      }
    }
  }
  return result;
}

template <typename T>
MassAndMomentum<T> SparseGrid<T>::ComputeTotalMassAndMomentum() const {
  MassAndMomentum<T> result;
  const std::vector<std::pair<Vector3<int>, GridData<T>>> grid_data =
      GetGridData();
  for (int i = 0; i < ssize(grid_data); ++i) {
    const T& mi = grid_data[i].second.m;
    const Vector3<T>& vi = grid_data[i].second.v;
    const Vector3<T>& xi = grid_data[i].first.template cast<T>() * dx_;
    result.mass += grid_data[i].second.m;
    result.linear_momentum += mi * vi;
    result.angular_momentum += mi * xi.cross(vi);
  }
  return result;
}

template <typename T>
void SparseGrid<T>::SortParticleIndices(const std::vector<Vector3<T>>& q_WPs) {
  const int num_particles = q_WPs.size();
  data_indices_.resize(num_particles);
  base_node_offsets_.resize(num_particles);
  particle_sorters_.resize(num_particles);

  /* We sort particles first based on their base node offsets, and if those are
   the same, we sort by their data indices. To do that, we notice that the base
   node offset of the particle looks like

       page bits | block bits | data bits

   with all the data bits being equal to zero. Also, the left most bits of the
   page bits are zero because at most 2^(3*kLog2MaxGridSize) number of grid
   nodes and that takes up 3*kLog2MaxGridSize bits. The page bits and block bits
   have 64 - data bits in total, so the left most 64 - data bits - 3 *
   kLog2MaxGridSize bits are zero. So we left shift the base node offset by that
   amount and now we get the lowest 64 - 3 * kLog2MaxGridSize bits (which we
   name `kIndexBits`) to be zero. With kLog2MaxGridSize == 10, we have 44 bits
   to work with, more than enough to store the particle indices. We then sort
   the resulting 64 bit unsigned integers which is enough to achieve the sorting
   objective. */
  constexpr int kIndexBits = 64 - 3 * kLog2MaxGridSize;
  constexpr int kZeroPageBits = 64 - kDataBits - 3 * kLog2MaxGridSize;
  [[maybe_unused]] const int num_threads = parallelism_.num_threads();

#if defined(_OPENMP)
#pragma omp parallel for num_threads(num_threads)
#endif
  for (int p = 0; p < num_particles; ++p) {
    const Vector3<int> base_node = ComputeBaseNode<T>(q_WPs[p] / dx_);
    base_node_offsets_[p] =
        CoordinateToOffset(base_node[0], base_node[1], base_node[2]);
    data_indices_[p] = p;
    /* Confirm the data bits of the base node offset are all zero. */
    DRAKE_ASSERT((base_node_offsets_[p] & ((uint64_t(1) << kDataBits) - 1)) ==
                 0);
    /* Confirm the left most bits in the page bits are unused. */
    DRAKE_ASSERT((base_node_offsets_[p] &
                  ~((uint64_t(1) << (64 - kZeroPageBits)) - 1)) == 0);
    particle_sorters_[p] =
        (base_node_offsets_[p] << kZeroPageBits) + data_indices_[p];
  }

#if defined(_OPENMP)
  ips2ra::parallel::sort(particle_sorters_.begin(), particle_sorters_.end(),
                         ips2ra::Config<>::identity{}, num_threads);
#else
  ips2ra::sort(particle_sorters_.begin(), particle_sorters_.end());
#endif

  /* Peel off the data indices and the base node offsets from particle_sorters_.
   Meanwhile, reorder the data indices and the base node offsets based on the
   sorting results. */
#if defined(_OPENMP)
#pragma omp parallel for num_threads(num_threads)
#endif
  for (int p = 0; p < ssize(particle_sorters_); ++p) {
    data_indices_[p] = particle_sorters_[p] & ((uint64_t(1) << kIndexBits) - 1);
    base_node_offsets_[p] = (particle_sorters_[p] >> kIndexBits) << kDataBits;
  }

  /* Record the sentinel particles and the coloring of the blocks. */
  sentinel_particles_.clear();
  for (int b = 0; b < 8; ++b) {
    colored_blocks_[b].clear();
  }
  uint64_t previous_page{};
  int block = 0;
  for (int p = 0; p < num_particles; ++p) {
    /* The bits in the offset is ordered as follows:

      page bits | block bits | data bits

     block bits and data bits add up to kLog2Page bits.
     We right shift to get the page bits. */
    const uint64_t page = base_node_offsets_[p] >> kLog2Page;
    if (p == 0 || previous_page != page) {
      previous_page = page;
      sentinel_particles_.push_back(p);
      const int color = get_color(page);
      colored_blocks_[color].push_back(block++);
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
