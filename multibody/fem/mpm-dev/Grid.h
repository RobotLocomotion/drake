#pragma once

#include <limits>
#include <vector>

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace mpm {

// A particles class holding vectors of particles' state. The grid can be
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
// , where num_gridpt_x/y/z = 6, 3, 4 respectively.
// Since we assume an uniform grid, the distance between neighboring gird points
// are all h. The bottom_corner is the grid node with the smallest (x, y, z)
// values. We adopt the lexicographical ordering the the grid nodes, where
// an index (i, j, k) denotes the indices in (x, y, z) directions, ordered by
// the values incrementally. Users will access the states on grid nodes by
// the (i, j, k) indexing. User can only change the states hold by each grid
// values after constructing the Grid object. (Cannot modify the actual
// geoemtry)
class Grid {
 public:
    Grid();
    Grid(const Vector3<int>& num_gridpt_1D, double h,
         const Vector3<double>& bottom_corner_pos);

    int get_num_gridpt() const;
    const Vector3<int>& get_num_gridpt_1D() const;
    double get_h() const;
    const Vector3<double>& get_bottom_corner_position() const;

    const Vector3<double>& get_position(int i, int j, int k) const;
    const Vector3<double>& get_velocity(int i, int j, int k) const;
    double get_mass(int i, int j, int k) const;
    const Vector3<double>& get_force(int i, int j, int k) const;

    void set_position(int i, int j, int k, const Vector3<double>& position);
    void set_velocity(int i, int j, int k, const Vector3<double>& velocity);
    void set_mass(int i, int j, int k, double mass);
    void set_force(int i, int j, int k, const Vector3<double>& force);

 private:
    int reduce3DIndex(int i, int j, int k) const;
    void check_3D_index(int i, int j, int k) const;

    int num_gridpt_;
    Vector3<int> num_gridpt_1D_;
    double h_;
    Vector3<double> bottom_corner_position_{};

    std::vector<Vector3<double>> positions_{};
    std::vector<Vector3<double>> velocities_{};
    std::vector<double> masses_{};
    std::vector<Vector3<double>> forces_{};
};  // class Grid

}  // namespace mpm
}  // namespace multibody
}  // namespace drake
