#pragma once

namespace drake {
namespace multibody {
namespace joints {

/// Indicates how unattached links are to be mobilized in the world (fixed or
/// floating) after parsing.
enum FloatingBaseType {
    /// A fixed body will be added to all unattached links. An error will be
    // triggered if no unattached links are found.
    kFixed = 0,
    /// A floating roll-pitch-yaw joint will be added to all unattached links.
    /// An error will be triggered if no unattached links are found.
    kRollPitchYaw = 1,
    /// A floating quaternion joint will be added to all unattached links. An
    /// error will be triggered if no unattached links are found.
    kQuaternion = 2,
    /// A floating quaternion joint will be added to all unattached links. No
    /// errror will be raised if no unattachced links are found.
    /// @note Intended for compatibility with MultibodyPlant SDF parsing.
    kImplicitQuaternion = 3,
};

}  // namespace joints
}  // namespace multibody
}  // namespace drake
