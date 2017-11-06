#pragma once

#include <map>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/rotational_inertia.h"

namespace drake {
namespace multibody {
namespace multibody_tree {
namespace parsing {

/// A representation of a `<link>` entry in an SDF file.
class SDFLink {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SDFLink);
  /// Creates a new link object specification with the given `link_name`.
  /// Per SDF specification, `link_name` must be unique within the scope of the
  /// link's model.
  explicit SDFLink(const std::string& link_name) : name_(link_name) {}

  /// Returns the name of `this` link.
  const std::string& name() const { return name_; }

  /// Returns the mass for `this` link.
  double mass() const { return mass_; }

  /// Returns the rotational inertia `I_Icm` of this link about its center of
  /// mass and expressed in the "inertial" frame `Icm` defined within the
  /// `<inertial>` element of an SDF file. The inertial frame `Icm` is defined
  /// by elements `<frame>` and `<pose>` within the `<inertial>` element.
  /// Refer to the documentation for the SDF specification for default values
  /// and for different ways to specify the inertial frame.
  const RotationalInertia<double>& rotational_inertia() const {
    return I_Icm_;
  }

 private:
  // Default values for non-required parameters for a link in an SDF files.
  // Most of these are guesses since the SDF specification does not specify
  // them.
  // TODO(amcastro-tri): File an issue and find an owner to resolve these
  // default values and make them part of the standard.
  // Ideally probably we'd like to have an sdf::kDefaultMass so this code does
  // not go out of sync with OSRC' SDF parser.
  // Reference issue number here at PR time.

  // Mass default value.
  static constexpr double kDefaultMass = 0.0;

  // Default rotational inertia.
  static constexpr RotationalInertia<double> kDefaultRotationalInertia{
      RotationalInertia(0.0, 0.0, 0.0)};

  // Name of the root frame of this cache.
  std::string name_;

  // The mass of this link.
  double mass_{kDefaultMass};

  // Rotational inertia of this link about its center of mass.
  // Icm denotes the <inertial> frame which per SDF specification must be at the
  // link's center of mass. The "inertial" frame Icm may or not be aligned with
  // the principal axes of the body.
  // Therefore, I_Icm is the inertia of this link, about its center of mass,
  // expressed in the "inertial" frame Icm.
  RotationalInertia<double> I_Icm_{kDefaultRotationalInertia};

  // Pose of this link's frame L in the model frame D.
  // Note: frame L is not necessarily at the link's com nor necessarily aligned
  // with the link's principal axes.
  Isometry3<double> X_DL_;
};

}  // namespace parsing
}  // namespace multibody_tree
}  // namespace multibody
}  // namespace drake
