#include "drake/multibody/fem/mpm-dev/Particles.h"

namespace drake {
namespace multibody {
namespace mpm {

Particles::Particles(): num_particles_(0) {}

Particles::Particles(int num_particles) {
    DRAKE_ASSERT(num_particles > 0);
    num_particles_ = num_particles;
    positions_ = std::vector<Vector3<double>>(num_particles);
    velocities_ = std::vector<Vector3<double>>(num_particles);
    masses_ = std::vector<double>(num_particles);
    volumes_ = std::vector<double>(num_particles);
    deformation_gradients_ = std::vector<Matrix3<double>>(num_particles);
    stresses_ = std::vector<Matrix3<double>>(num_particles);
}

int Particles::get_num_particles() {
    return num_particles_;
}

const Vector3<double>& Particles::get_position(int index) const {
    return positions_[index];
}

const Vector3<double>& Particles::get_velocity(int index) const {
    return velocities_[index];
}

const double& Particles::get_mass(int index) const {
    return masses_[index];
}

const double& Particles::get_volume(int index) const {
    return volumes_[index];
}

const Matrix3<double>& Particles::get_deformation_gradient(int index) const {
    return deformation_gradients_[index];
}

const Matrix3<double>& Particles::get_stress(int index) const {
    return stresses_[index];
}

void Particles::set_position(int index, const Vector3<double>& position) {
    positions_[index] = position;
}

void Particles::set_velocity(int index, const Vector3<double>& velocity) {
    velocities_[index] = velocity;
}

void Particles::set_mass(int index, double mass) {
    DRAKE_DEMAND(mass > 0.0);
    masses_[index] = mass;
}

void Particles::set_volume(int index, double volume) {
    DRAKE_DEMAND(volume > 0.0);
    volumes_[index] = volume;
}

void Particles::set_deformation_gradient(int index,
                        const Matrix3<double>& deformation_gradient) {
    deformation_gradients_[index] = deformation_gradient;
}

void Particles::set_stress(int index, const Matrix3<double>& stress) {
    stresses_[index] = stress;
}

void Particles::add_particle(const Vector3<double>& position,
                             const Vector3<double>& velocity,
                             double mass, double volume,
                             const Matrix3<double>& deformation_gradient,
                             const Matrix3<double>& stress) {
    positions_.emplace_back(position);
    velocities_.emplace_back(velocity);
    masses_.emplace_back(mass);
    volumes_.emplace_back(volume);
    deformation_gradients_.emplace_back(deformation_gradient);
    stresses_.emplace_back(stress);
    num_particles_++;
}

}  // namespace mpm
}  // namespace multibody
}  // namespace drake
