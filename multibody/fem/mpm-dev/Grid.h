#pragma once

#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/mpm-dev/BSpline.h"

namespace drake {
namespace multibody {
namespace mpm {

// A particles class holding vectors of particles' state. The grid can be
// visually represented as:
//
//                 h             top_corner
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
// values, and the top_corner is the grid node with the largest (x, y, z)
// values. We adopt the lexicographical ordering the the grid nodes, where
// an index (i, j, k) denotes the indices in (x, y, z) directions, ordered by
// the values incrementally. Users will access the states on grid nodes by
// the (i, j, k) indexing. User can only change the states hold by each grid
// values after constructing the Grid object. (Cannot modify the actual
// geoemtry)
class Grid {
 public:
    Grid();
    Grid(int num_gridpt_x, int num_gridpt_y, int num_gridpt_z, double h,
         const Vector3<double>& bottom_corner,
         const Vector3<double>& top_corner);

    int get_num_gridpt() const;
    int get_num_gridpt_x() const;
    int get_num_gridpt_y() const;
    int get_num_gridpt_z() const;
    double get_h() const;
    const Vector3<double>& get_bottom_corner() const;
    const Vector3<double>& get_top_corner() const;

    const Vector3<double>& get_position(int i, int j, int k) const;
    const Vector3<double>& get_velocity(int i, int j, int k) const;
    double get_mass(int i, int j, int k) const;
    const Vector3<double>& get_force(int i, int j, int k) const;

    void set_position(int i, int j, int k, const Vector3<double>& position);
    void set_velocity(int i, int j, int k, const Vector3<double>& velocity);
    void set_mass(int i, int j, int k, double mass);
    void set_force(int i, int j, int k, const Vector3<double>& force);

 private:
    int reduce_3D_index(int i, int j, int k);

    int num_gridpt_;
    int num_gridpt_x_;
    int num_gridpt_y_;
    int num_gridpt_z_;
    double h_;
    Vector3<double> bottom_corner_{};
    Vector3<double> top_corner_{};
    std::vector<BSpline> bases_{};

    std::vector<Vector3<double>> positions_{};
    std::vector<Vector3<double>> velocities_{};
    std::vector<double> masses_{};
    std::vector<Vector3<double>> forces_{};
};  // class Particles

}  // namespace mpm
}  // namespace multibody
}  // namespace drake
