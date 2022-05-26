#include "drake/multibody/fem/mpm-dev/Grid.h"

namespace drake {
namespace multibody {
namespace mpm {

Grid::Grid(const Vector3<int>& num_gridpt_1D, double h,
           const Vector3<int>& bottom_corner):
           num_gridpt_(num_gridpt_1D(0)*num_gridpt_1D(1)*num_gridpt_1D(2)),
           num_gridpt_1D_(num_gridpt_1D), h_(h),
           bottom_corner_(bottom_corner) {
    int idx;
    DRAKE_ASSERT(num_gridpt_1D_(0) >= 0);
    DRAKE_ASSERT(num_gridpt_1D_(1) >= 0);
    DRAKE_ASSERT(num_gridpt_1D_(2) >= 0);
    DRAKE_ASSERT(h_ > 0.0);

    indices_ = std::vector<std::pair<int, Vector3<int>>>(num_gridpt_);
    positions_ = std::vector<Vector3<double>>(num_gridpt_);
    velocities_ = std::vector<Vector3<double>>(num_gridpt_);
    masses_ = std::vector<double>(num_gridpt_);
    forces_ = std::vector<Vector3<double>>(num_gridpt_);

    // Initialize the positions of grid points
    for (int k = bottom_corner_(2);
             k < bottom_corner_(2) + num_gridpt_1D_(2); ++k) {
    for (int j = bottom_corner_(1);
             j < bottom_corner_(1) + num_gridpt_1D_(1); ++j) {
    for (int i = bottom_corner_(0);
             i < bottom_corner_(0) + num_gridpt_1D_(0); ++i) {
        idx = Reduce3DIndex(i, j, k);
        indices_[idx] = std::pair<int, Vector3<int>>(idx, {i, j, k});
        positions_[idx] = Vector3<double>{h_*i, h_*j, h_*k};
    }
    }
    }
}

int Grid::get_num_gridpt() const {
    return num_gridpt_;
}

const Vector3<int>& Grid::get_num_gridpt_1D() const {
    return num_gridpt_1D_;
}

double Grid::get_h() const {
    return h_;
}

const Vector3<int>& Grid::get_bottom_corner() const {
    return bottom_corner_;
}

const Vector3<double>& Grid::get_position(int i, int j, int k) const {
    DRAKE_ASSERT(in_index_range(i, j, k));
    return positions_[Reduce3DIndex(i, j, k)];
}

const Vector3<double>& Grid::get_velocity(int i, int j, int k) const {
    DRAKE_ASSERT(in_index_range(i, j, k));
    return velocities_[Reduce3DIndex(i, j, k)];
}

double Grid::get_mass(int i, int j, int k) const {
    DRAKE_ASSERT(in_index_range(i, j, k));
    return masses_[Reduce3DIndex(i, j, k)];
}

const Vector3<double>& Grid::get_force(int i, int j, int k) const {
    DRAKE_ASSERT(in_index_range(i, j, k));
    return forces_[Reduce3DIndex(i, j, k)];
}

void Grid::set_position(int i, int j, int k, const Vector3<double>& position) {
    DRAKE_ASSERT(in_index_range(i, j, k));
    positions_[Reduce3DIndex(i, j, k)] = position;
}

void Grid::set_velocity(int i, int j, int k, const Vector3<double>& velocity) {
    DRAKE_ASSERT(in_index_range(i, j, k));
    velocities_[Reduce3DIndex(i, j, k)] = velocity;
}

void Grid::set_mass(int i, int j, int k, double mass) {
    DRAKE_ASSERT(in_index_range(i, j, k));
    masses_[Reduce3DIndex(i, j, k)] = mass;
}

void Grid::set_force(int i, int j, int k, const Vector3<double>& force) {
    DRAKE_ASSERT(in_index_range(i, j, k));
    forces_[Reduce3DIndex(i, j, k)] = force;
}

int Grid::Reduce3DIndex(int i, int j, int k) const {
    DRAKE_ASSERT(in_index_range(i, j, k));
    return (k-bottom_corner_(2))*(num_gridpt_1D_(0)*num_gridpt_1D_(1))
         + (j-bottom_corner_(1))*num_gridpt_1D_(0)
         + (i-bottom_corner_(0));
}

Vector3<int> Grid::Expand1DIndex(int idx) const {
    return Vector3<int>(
            bottom_corner_(0) + idx % num_gridpt_1D_(0),
            bottom_corner_(1) + (idx / num_gridpt_1D_(0)) % num_gridpt_1D_(1),
            bottom_corner_(2) + idx / (num_gridpt_1D_(0)*num_gridpt_1D_(1)));
}

const std::vector<std::pair<int, Vector3<int>>>& Grid::get_indices() const {
    return indices_;
}

bool Grid::in_index_range(int i, int j, int k) const {
    return ((i < bottom_corner_(0) + num_gridpt_1D_(0)) &&
            (j < bottom_corner_(1) + num_gridpt_1D_(1)) &&
            (k < bottom_corner_(2) + num_gridpt_1D_(2)) &&
            (i >= bottom_corner_(0)) &&
            (j >= bottom_corner_(1)) &&
            (k >= bottom_corner_(2)));
}

}  // namespace mpm
}  // namespace multibody
}  // namespace drake
