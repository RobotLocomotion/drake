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
T calc_smooth_acceleration(T desired_acceleration, T current_velocity,
    T max_velocity, T velocity_limit_kp);

}  // namespace automotive
}  // namespace drake
