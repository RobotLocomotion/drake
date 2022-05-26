#include "drake/multibody/fem/mpm-dev/MPMTransfer.h"

namespace drake {
namespace multibody {
namespace mpm {

// TODO(yiminlin.tri): We assume particles all lies in the grid
// Also may be a good idea to remove this routine's dependency on the grid,
// this need future refactoring
void MPMTransfer::SortParticles(const Grid& grid, Particles* particles) {
    Vector3<int> batch_idx_3D;
    int num_particles = particles->get_num_particles();
    // A temporary array storing the particle index permutation after sorting
    std::vector<size_t> sorted_indices(num_particles);
    // A temporary array storing the batch index correspond to each particle
    std::vector<int> batch_indices(num_particles);
    batch_sizes_.resize(grid.get_num_gridpt());  // Initialize batch_size to be
                                                 // 0 for every batch

    // Preallocate the indices of batches
    for (int p = 0; p < num_particles; ++p) {
        batch_idx_3D = CalcBatchIndex(particles->get_position(p), grid.get_h());
        batch_indices[p] = grid.Reduce3DIndex(batch_idx_3D(0), batch_idx_3D(1),
                                              batch_idx_3D(2));
        ++batch_sizes_[batch_indices[p]];
    }

    std::iota(sorted_indices.begin(), sorted_indices.end(), 0);
    std::sort(sorted_indices.begin(), sorted_indices.end(),
       [&grid, &batch_indices](size_t i1, size_t i2) {
                                return batch_indices[i1] < batch_indices[i2];});

    // Reorder the particles
    particles->Reorder(sorted_indices);
}

Vector3<int> MPMTransfer::CalcBatchIndex(const Vector3<double>& xp, double h)
                                                                        const {
    return Vector3<int>(std::round(xp(0)/h), std::round(xp(1)/h),
                        std::round(xp(2)/h));
}

}  // namespace mpm
}  // namespace multibody
}  // namespace drake
