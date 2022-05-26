#pragma once

#include <limits>
#include <vector>

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace mpm {

// A grid class holding vectors of grid points' state. The grid can be
// visually represented as:
//
//                 h
//               o - o - o - o - o - o
//               |   |   |   |   |   |
//           o - o - o - o - o - o - o
//           |   |   |   |   |   |   |
//       o - o - o - o - o - o - o - o
//       |   |   |   |   |   |   |   |
//       o - o - o - o - o - o - o - o
//       |   |   |   |   |   |   |
//       o - o - o - o - o - o - o                  z   y
//       |   |   |   |   |   |                      | /
//       o - o - o - o - o - o                       -- x
//  bottom_corner
//
// The grid can be uniquely represented as a bottom corner and the number of
// grid points in x, y and z directions.
// Since we assume an uniform grid, the distance between neighboring grid points
// are all h. The bottom_corner is the grid node with the smallest (x, y, z)
// values. We assume the bottom corner aligns with the "index space" (i, j, k)
// , which is an 3D integer space (Z \times Z \times Z). The actual physical
// coordinate of the index space is defined through the mapping (ih, jh, kh).
// Positions of grid points will be stored as physical space coordinates.
// The vector num_gridpt_1D stores the number of grid points along x, y, and z
// directions. Figure above is an example with num_gridpt_1D = (6, 3, 4).
// We stores all the data holding states as 1D vectors, with linear indexing
// following the lexiographical order in x, y, z directions
class Grid {
 public:
    Grid() = default;
    Grid(const Vector3<int>& num_gridpt_1D, double h,
         const Vector3<int>& bottom_corner);

    int get_num_gridpt() const;
    const Vector3<int>& get_num_gridpt_1D() const;
    double get_h() const;
    const Vector3<int>& get_bottom_corner() const;

    // For below (i, j, k), we expect users pass in the coordinate in the
    // index space as documented above.
    const Vector3<double>& get_position(int i, int j, int k) const;
    const Vector3<double>& get_velocity(int i, int j, int k) const;
    double get_mass(int i, int j, int k) const;
    const Vector3<double>& get_force(int i, int j, int k) const;

    void set_position(int i, int j, int k, const Vector3<double>& position);
    void set_velocity(int i, int j, int k, const Vector3<double>& velocity);
    void set_mass(int i, int j, int k, double mass);
    void set_force(int i, int j, int k, const Vector3<double>& force);

    // Reduce an 3D (i, j, k) index in the index space to a corresponding
    // linear lexiographical ordered index
    int Reduce3DIndex(int i, int j, int k) const;

 private:
    // DEBUG mode: check the passed in (i, j, k) lies within the index range of
    // this Grid
    void check_3D_index(int i, int j, int k) const;

    int num_gridpt_;
    Vector3<int> num_gridpt_1D_;              // Number of grid points on the
                                              // grid along x, y, z directions
    double h_;
    Vector3<int> bottom_corner_{};

    std::vector<Vector3<double>> positions_{};
    std::vector<Vector3<double>> velocities_{};
    std::vector<double> masses_{};
    std::vector<Vector3<double>> forces_{};
};  // class Grid

}  // namespace mpm
}  // namespace multibody
}  // namespace drake
