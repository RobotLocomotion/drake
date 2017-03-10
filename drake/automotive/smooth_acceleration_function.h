#pragma once

namespace drake {
namespace automotive {

/// Computes and returns an acceleration command that results in a smooth
/// acceleration profile.
///
///  Instantiated templates for the following ScalarTypes are provided:
/// - double
/// - drake::AutoDiffXd
/// - drake::symbolic::Expression
///
/// They are already available to link against in the containing library.
///
/// @ingroup automotive_systems
template <typename T>
void calc_smooth_acceleration(const T& desired_acceleration,
    const T& current_velocity, const T& max_velocity,
    const T& velocity_limit_kp, T* result);

}  // namespace automotive
}  // namespace drake
