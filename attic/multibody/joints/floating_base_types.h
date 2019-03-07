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
    /// (Experimental) Intended for compatibility with MultibodyPlant SDF
    /// parsing: A floating quaternion joint will be added to all unattached
    /// links; however, it does not trigger an error if no unattachced links
    /// are found.
    /// @warning This is an experimental feature whose functionality and name
    /// may change before it becomes stable. Please proceed with caution.
    kExperimentalMultibodyPlantStyle = 3,
};

}  // namespace joints
}  // namespace multibody
}  // namespace drake
