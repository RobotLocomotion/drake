#pragma once

namespace drake {
namespace automotive {

/// Computes and returns an acceleration command that results in a smooth
/// acceleration profile. It is smooth in the sense that it looks pleasant and
/// realistic, though it may not reflect the actual acceleration behavior of
/// real vehicles.
///
/// For a given acceleration plan, all of the input parameters typically remain
/// constant except for @p current_velocity.
///
/// Instantiated templates for the following ScalarTypes are provided:
/// - double
/// - drake::AutoDiffXd
/// - drake::symbolic::Expression
///
/// They are already available to link against in the containing library.
///
/// @ingroup automotive_systems
template <typename T>
T calc_smooth_acceleration(const T& desired_acceleration, const T& max_velocity,
    const T& velocity_limit_kp, const T& current_velocity);

}  // namespace automotive
}  // namespace drake
