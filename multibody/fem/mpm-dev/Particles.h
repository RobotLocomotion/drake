#pragma once

#include <vector>

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace mpm {

// A particles class holding vectors of particles' state
class Particles {
 public:
    Particles();
    explicit Particles(int num_particles);

    int get_num_particles();
    const Vector3<double>& get_position_at(int index) const;
    const Vector3<double>& get_velocity_at(int index) const;
    const double& get_mass_at(int index) const;
    const double& get_volume_at(int index) const;
    const Matrix3<double>& get_deformation_gradient_at(int index) const;
    const Matrix3<double>& get_stress_at(int index) const;

    void set_position_at(int index, const Vector3<double>& position);
    void set_velocity_at(int index, const Vector3<double>& velocity);
    void set_mass_at(int index, double mass);
    void set_volume_at(int index, double volume);
    void set_deformation_gradient_at(int index,
                                 const Matrix3<double>& deformation_gradient);
    void set_stress_at(int index, const Matrix3<double>& stress);

    void add_particle(const Vector3<double>& position,
                      const Vector3<double>& velocity,
                      double mass, double volume,
                      const Matrix3<double>& deformation_gradient,
                      const Matrix3<double>& stress);

 private:
    int num_particles_;
    std::vector<Vector3<double>> positions_{};
    std::vector<Vector3<double>> velocities_{};
    std::vector<double> masses_{};
    std::vector<double> volumes_{};
    std::vector<Matrix3<double>> deformation_gradients_{};
    std::vector<Matrix3<double>> stresses_{};
};  // class Particles

}  // namespace mpm
}  // namespace multibody
}  // namespace drake
