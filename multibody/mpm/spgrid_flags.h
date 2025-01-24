#pragma once

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {

/* A subset of SPGrid flags and configs we care about. */
struct SpGridFlags {
  int log2_page{12};            // 4KB page size.
  int log2_max_grid_size{10};   // Largest grid size is 1024 x 1024 x 1024.
  int data_bits{6};             // Number of bits to represent GridData.
  int num_nodes_in_block_x{4};  // Number of nodes in a block in x direction.
  int num_nodes_in_block_y{4};  // Number of nodes in a block in y direction.
  int num_nodes_in_block_z{4};  // Number of nodes in a block in z direction.
};

}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
