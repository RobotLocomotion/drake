#include "drake/multibody/fem/mpm-dev/MPMTransfer.h"

namespace drake {
namespace multibody {
namespace mpm {

void MPMTransfer::SetUpTransfer(const Grid& grid, Particles* particles) {
    SortParticles(grid, particles);
    UpdateBasisAndGradientParticles(grid, *particles);
}

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

void MPMTransfer::UpdateBasisAndGradientParticles(const Grid& grid,
                                                  const Particles& particles) {
    int num_gridpt, num_particles, p_start, p_end, count;
    double h;
    num_gridpt = grid.get_num_gridpt();
    Vector3<int> num_gridpt_1D = grid.get_num_gridpt_1D();
    Vector3<int> bottom_corner = grid.get_bottom_corner();
    std::vector<BSpline> bases(num_gridpt);
    num_gridpt = grid.get_num_gridpt();
    num_particles = particles.get_num_particles();
    h = grid.get_h();

    bases_val_particles_.reserve(num_particles);
    bases_grad_particles_.reserve(num_particles);

    // Define the bases vector, whose ith entry is the BSpline basis correspond
    // to ith BSpline basis.
    count = 0;
    for (int k = bottom_corner(2); k < bottom_corner(2) + num_gridpt_1D(2);
                                                                        ++k) {
    for (int j = bottom_corner(1); j < bottom_corner(1) + num_gridpt_1D(1);
                                                                        ++j) {
    for (int i = bottom_corner(0); i < bottom_corner(0) + num_gridpt_1D(0);
                                                                        ++i) {
        bases[count++] = BSpline(h, grid.get_position(i, j, k));
    }
    }
    }

    // For each batch of particles
    p_start = 0;
    for (const auto& [batch_index_flat, batch_index_3d] : grid.get_indices()) {
        p_end = p_start + batch_sizes_[batch_index_flat];
        // For each particle in the batch (Assume particles are sorted with
        // respect to the batch index), update basis evaluations
        for (int p = p_start; p < p_end; ++p) {
            const Vector3<double>& xp = particles.get_position(p);
            EvalBasisOnBatch(p, xp, grid, batch_index_3d, bases);
        }
        p_start = p_end;
    }
}

void MPMTransfer::EvalBasisOnBatch(int p, const Vector3<double>& xp,
                                   const Grid& grid,
                                   const Vector3<int>& batch_index_3d,
                                   const std::vector<BSpline>& bases) {
    int bi = batch_index_3d[0];
    int bj = batch_index_3d[1];
    int bk = batch_index_3d[2];
    int idx_local;
    for (int k = bk - 1; k <= bk + 1; ++k) {
    for (int j = bj - 1; j <= bj + 1; ++j) {
    for (int i = bi - 1; i <= bi + 1; ++i) {
    if (grid.in_index_range(i, j, k)) {
        idx_local = (i-bi+1) + 3*(j-bj+1) + 9*(k-bk+1);
        // For each particle in the batch (Assume particles are sorted with
        // respect to the batch index), update basis evaluations
        std::tie(bases_val_particles_[p][idx_local],
                 bases_grad_particles_[p][idx_local]) =
        bases[grid.Reduce3DIndex(i, j, k)].EvalBasisAndGradient(xp);
    } else {
        throw std::logic_error("Particles out of bound");
    }
    }
    }
    }
}

Vector3<int> MPMTransfer::CalcBatchIndex(const Vector3<double>& xp, double h)
                                                                        const {
    return Vector3<int>(std::round(xp(0)/h), std::round(xp(1)/h),
                        std::round(xp(2)/h));
}

}  // namespace mpm
}  // namespace multibody
}  // namespace drake
