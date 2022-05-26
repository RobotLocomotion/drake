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

int Particles::get_num_particles() const {
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

const std::vector<Vector3<double>>& Particles::get_positions() const {
    return positions_;
}

const std::vector<Vector3<double>>& Particles::get_velocities() const {
    return velocities_;
}

const std::vector<double>& Particles::get_masses() const {
    return masses_;
}

const std::vector<double>& Particles::get_reference_volumes() const {
    return reference_volumes_;
}

const std::vector<Matrix3<double>>& Particles::get_deformation_gradients()
                                                                        const {
    return deformation_gradients_;
}
const std::vector<Matrix3<double>>& Particles::get_kirchhoff_stresses() const {
    return kirchhoff_stresses_;
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

void Particles::set_positions(const std::vector<Vector3<double>>& positions) {
    positions_ = positions;
}

void Particles::set_velocities(const std::vector<Vector3<double>>& velocities) {
    velocities_ = velocities;
}

void Particles::set_masses(const std::vector<double>& masses) {
    masses_ = masses;
}

void Particles::set_reference_volumes(const std::vector<double>&
                                        reference_volumes) {
    reference_volumes_ = reference_volumes;
}

void Particles::set_deformation_gradients(const std::vector<Matrix3<double>>&
                                deformation_gradients) {
    deformation_gradients_ = deformation_gradients;
}

void Particles::set_kirchhoff_stresses(const std::vector<Matrix3<double>>&
                            kirchhoff_stresses) {
    kirchhoff_stresses_ = kirchhoff_stresses;
}

void Particles::Reorder(const std::vector<size_t>& new_order) {
    DRAKE_DEMAND(static_cast<int>(new_order.size()) == num_particles_);
    int p_new;
    std::vector<Vector3<double>> positions_sorted(num_particles_);
    std::vector<Vector3<double>> velocities_sorted(num_particles_);
    std::vector<double> masses_sorted(num_particles_);
    std::vector<double> reference_volumes_sorted(num_particles_);
    std::vector<Matrix3<double>> deformation_gradients_sorted(num_particles_);
    std::vector<Matrix3<double>> kirchhoff_stresses_sorted(num_particles_);
    for (int p = 0; p < num_particles_; ++p) {
        p_new = new_order[p];
        positions_sorted[p]             = positions_[p_new];
        velocities_sorted[p]            = velocities_[p_new];
        masses_sorted[p]                = masses_[p_new];
        reference_volumes_sorted[p]     = reference_volumes_[p_new];
        deformation_gradients_sorted[p] = deformation_gradients_[p_new];
        kirchhoff_stresses_sorted[p]    = kirchhoff_stresses_[p_new];
    }
    positions_.swap(positions_sorted);
    velocities_.swap(velocities_sorted);
    masses_.swap(masses_sorted);
    reference_volumes_.swap(reference_volumes_sorted);
    deformation_gradients_.swap(deformation_gradients_sorted);
    kirchhoff_stresses_.swap(kirchhoff_stresses_sorted);
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
