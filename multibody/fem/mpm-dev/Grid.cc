#include "drake/multibody/fem/mpm-dev/Grid.h"

namespace drake {
namespace multibody {
namespace mpm {

constexpr double kEps = 4.0 * std::numeric_limits<double>::epsilon();

Grid::Grid(): num_gridpt_(0), num_gridpt_1D_(0, 0, 0), h_(1.0) {}

Grid::Grid(const Vector3<int>& num_gridpt_1D, double h,
           const Vector3<double>& bottom_corner_pos):
           num_gridpt_(num_gridpt_1D(0)*num_gridpt_1D(1)*num_gridpt_1D(2)),
           num_gridpt_1D_(num_gridpt_1D), h_(h),
           bottom_corner_position_(bottom_corner_pos) {
    int idx;
    DRAKE_ASSERT(num_gridpt_1D_(0) >= 0);
    DRAKE_ASSERT(num_gridpt_1D_(1) >= 0);
    DRAKE_ASSERT(num_gridpt_1D_(2) >= 0);
    DRAKE_ASSERT(h_ > 0.0);

    positions_ = std::vector<Vector3<double>>(num_gridpt_);
    velocities_ = std::vector<Vector3<double>>(num_gridpt_);
    masses_ = std::vector<double>(num_gridpt_);
    forces_ = std::vector<Vector3<double>>(num_gridpt_);

    // Initialize the positions of grid points
    for (int k = 0; k < num_gridpt_1D_(2); ++k) {
    for (int j = 0; j < num_gridpt_1D_(1); ++j) {
    for (int i = 0; i < num_gridpt_1D_(0); ++i) {
        idx = reduce3DIndex(i, j, k);
        positions_[idx] = bottom_corner_position_
                        + Vector3<double>{i*h_, j*h_, k*h_};
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

const Vector3<double>& Grid::get_bottom_corner_position() const {
    return bottom_corner_position_;
}

const Vector3<double>& Grid::get_position(int i, int j, int k) const {
    check_3D_index(i, j, k);
    return positions_[reduce3DIndex(i, j, k)];
}

const Vector3<double>& Grid::get_velocity(int i, int j, int k) const {
    check_3D_index(i, j, k);
    return velocities_[reduce3DIndex(i, j, k)];
}

double Grid::get_mass(int i, int j, int k) const {
    check_3D_index(i, j, k);
    return masses_[reduce3DIndex(i, j, k)];
}

const Vector3<double>& Grid::get_force(int i, int j, int k) const {
    check_3D_index(i, j, k);
    return forces_[reduce3DIndex(i, j, k)];
}

void Grid::set_position(int i, int j, int k, const Vector3<double>& position) {
    check_3D_index(i, j, k);
    positions_[reduce3DIndex(i, j, k)] = position;
}

void Grid::set_velocity(int i, int j, int k, const Vector3<double>& velocity) {
    check_3D_index(i, j, k);
    velocities_[reduce3DIndex(i, j, k)] = velocity;
}

void Grid::set_mass(int i, int j, int k, double mass) {
    check_3D_index(i, j, k);
    masses_[reduce3DIndex(i, j, k)] = mass;
}

void Grid::set_force(int i, int j, int k, const Vector3<double>& force) {
    check_3D_index(i, j, k);
    forces_[reduce3DIndex(i, j, k)] = force;
}

int Grid::reduce3DIndex(int i, int j, int k) const {
    check_3D_index(i, j, k);
    return k*(num_gridpt_1D_(0)*num_gridpt_1D_(1))
         + j*num_gridpt_1D_(0) + i;
}

void Grid::check_3D_index(int i, int j, int k) const {
    DRAKE_ASSERT(i < num_gridpt_1D_(0));
    DRAKE_ASSERT(j < num_gridpt_1D_(1));
    DRAKE_ASSERT(k < num_gridpt_1D_(2));
    DRAKE_ASSERT(i >= 0);
    DRAKE_ASSERT(j >= 0);
    DRAKE_ASSERT(k >= 0);
}

}  // namespace mpm
}  // namespace multibody
}  // namespace drake
