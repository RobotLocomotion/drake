#pragma once

#include <map>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace parsing {

/// A representation of a `<link>` element in an SDF file.
/// For details on the specification of links, including conventions and default
/// values, please refer to the documentation for the
/// <a href="http://sdformat.org/spec?ver=1.6&elem=link">
/// &lt;link&gt; element</a>.
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

  /// Sets the mass for `this` link to `mass_value`.
  void set_mass(double mass_value) { mass_ = mass_value; }

  /// Returns the rotational inertia `I_Icm` of `this` link about its center of
  /// mass and expressed in the "inertial" frame `Icm` defined within the
  /// `<inertial>` element of an SDF file. The inertial frame `Icm` is defined
  /// by elements `<frame>` and `<pose>` within the `<inertial>` element.
  /// Refere to the documentation for the
  /// <a href="http://sdformat.org/spec?ver=1.6&elem=link#link_inertial">
  /// &lt;inertial&gt; element</a> for details on how the inertial frame is
  /// defined.
  const Matrix3<double>& get_inertia_matrix() const {
    return I_Icm_;
  }

  /// Sets the inertia matrix for `this` link. The input matrix must represent
  /// a valid inertia matrix about the link's COM, which coincides with the
  /// inertial frame origin `Io`, and must be expressed in the inertial frame I.
  void set_inertia_matrix(const Matrix3<double>& I_Io_I) {
    I_Icm_ = I_Io_I;
  }

  /// Returns the pose `X_DL` of `this` link's frame L in the parent model
  /// frame D.
  const Isometry3<double>& get_pose_in_model() const {
    return X_DL_;
  }

  /// Sets the pose `X_DL` of `this` link's frame L in the parent model frame D.
  void set_pose_in_model(const Isometry3<double>& X_DL) {
    X_DL_ = X_DL;
  }

  /// Gets the pose of the inertial (`<inertial>`) frame I in the link frame L.
  const Isometry3<double>& get_inertial_frame_pose() const {
    return X_LI_;
  }

  /// Sets the pose of the inertial (`<inertial>`) frame I in the link frame L.
  void set_inertial_frame_pose(const Isometry3<double>& X_LI) {
    X_LI_ = X_LI;
  }

 private:
  // Name of the root frame of this cache.
  std::string name_;

  // The mass of this link.
  // TODO(amcastro-tri): default value must be provided by sdformat library.
  double mass_{0.0};

  // The pose of the <inertial> frame I in a measured-in frame L.
  // TODO(amcastro-tri): default value must be provided by sdformat library.
  Isometry3<double> X_LI_{Isometry3<double>::Identity()};

  // Rotational inertia of this link about its center of mass.
  // Icm denotes the <inertial> frame which per SDF specification must be at the
  // link's center of mass. The "inertial" frame Icm may or not be aligned with
  // the principal axes of the body.
  // Therefore, I_Icm is the inertia of this link, about its center of mass,
  // expressed in the "inertial" frame Icm.
  // TODO(amcastro-tri): default value must be provided by sdformat library.
  Matrix3<double> I_Icm_{Matrix3<double>::Zero()};

  // Pose of this link's frame L in the model frame D.
  // Note: frame L is not necessarily at the link's com nor necessarily aligned
  // with the link's principal axes.
  Isometry3<double> X_DL_;
};

}  // namespace parsing
}  // namespace multibody
}  // namespace drake
