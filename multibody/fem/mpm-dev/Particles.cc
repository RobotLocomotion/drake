#include "drake/multibody/fem/mpm-dev/Particles.h"

namespace drake {
namespace multibody {
namespace mpm {

Particles::Particles(): num_particles_(0) {}

Particles::Particles(int num_particles): num_particles_(num_particles),
                                         positions_(num_particles),
                                         velocities_(num_particles),
                                         masses_(num_particles),
                                         reference_volumes_(num_particles),
                                         deformation_gradients_(num_particles),
                                         kirchhoff_stresses_(num_particles) {
    DRAKE_ASSERT(num_particles >= 0);
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

const double& Particles::get_reference_volume(int index) const {
    return reference_volumes_[index];
}

const Matrix3<double>& Particles::get_deformation_gradient(int index) const {
    return deformation_gradients_[index];
}

const Matrix3<double>& Particles::get_kirchhoff_stress(int index) const {
    return kirchhoff_stresses_[index];
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

void Particles::set_reference_volume(int index, double reference_volume) {
    DRAKE_DEMAND(reference_volume > 0.0);
    reference_volumes_[index] = reference_volume;
}

void Particles::set_deformation_gradient(int index,
                        const Matrix3<double>& deformation_gradient) {
    deformation_gradients_[index] = deformation_gradient;
}

void Particles::set_kirchhoff_stress(int index,
                                     const Matrix3<double>& kirchhoff_stress) {
    kirchhoff_stresses_[index] = kirchhoff_stress;
}

void Particles::AddParticle(const Vector3<double>& position,
                            const Vector3<double>& velocity,
                            double mass, double reference_volume,
                            const Matrix3<double>& deformation_gradient,
                            const Matrix3<double>& kirchhoff_stress) {
    positions_.emplace_back(position);
    velocities_.emplace_back(velocity);
    masses_.emplace_back(mass);
    reference_volumes_.emplace_back(reference_volume);
    deformation_gradients_.emplace_back(deformation_gradient);
    kirchhoff_stresses_.emplace_back(kirchhoff_stress);
    num_particles_++;
}

}  // namespace mpm
}  // namespace multibody
}  // namespace drake
