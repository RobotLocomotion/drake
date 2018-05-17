#include "drake/multibody/multibody_tree/parsing/scene_graph_parser_detail.h"

#include <memory>
#include <string>
#include <utility>

#include <sdf/sdf.hh>

#include "drake/geometry/geometry_instance.h"
#include "drake/multibody/multibody_tree/multibody_plant/coulomb_friction.h"
#include "drake/multibody/multibody_tree/parsing/sdf_parser_common.h"

namespace drake {
namespace multibody {
namespace parsing {
namespace detail {

using Eigen::Isometry3d;
using Eigen::Vector3d;
using geometry::GeometryInstance;
using multibody_plant::CoulombFriction;
using std::make_unique;

namespace {

// Helper to return the child element of `element` named `child_name`.
// Returns nullptr if not present.
sdf::ElementPtr GetElementPointerOrNullPtr(
    sdf::ElementPtr element, const std::string& child_name) {
  // First verify <child_name> is present (otherwise GetElement() has the
  // side effect of adding new elements if not present!!).
  if (element->HasElement(child_name)) {
    return element->GetElement(child_name);
  }
  return nullptr;
}

// Helper to return the child element of `element` named `child_name`.
// Throws std::logic_error if not found.
sdf::ElementPtr GetElementPointerOrThrow(
    sdf::ElementPtr element, const std::string &child_name) {
  // First verify <child_name> is present (otherwise GetElement() has the
  // side effect of adding new elements if not present!!).
  if (!element->HasElement(child_name)) {
    throw std::logic_error(
        "Element <" + child_name + "> not found nested within element <" +
            element->GetName() + ">.");
  }
  return element->GetElement(child_name);;
}

// Helper to return the value of a child of `element` named `child_name`.
// An std::logic_error is thrown if `child_name` does not exist or if no value
// was provided by the user that is, if `<child_name></child_name>` is empty.
template <typename T>
T GetValueOrThrow(sdf::ElementPtr element, const std::string& child_name) {
  if (!element->HasElement(child_name)) {
    throw std::logic_error(
        "Element <" + child_name + "> is required within element "
            "<" + element->GetName() + ">.");
  }
  std::pair<T, bool> value_pair = element->Get<T>(child_name, T());
  if (value_pair.second == false) {
    throw std::logic_error(
        "No value provide for <" + child_name + "> within element "
            "<" + element->GetName() + ">.");
  }
  return value_pair.first;
}

}  // namespace

std::unique_ptr<geometry::Shape> MakeShapeFromSdfGeometry(
    const sdf::Geometry& sdf_geometry) {
  switch (sdf_geometry.Type()) {
    case sdf::GeometryType::EMPTY: {
      return std::unique_ptr<geometry::Shape>(nullptr);
    }
    case sdf::GeometryType::BOX: {
      const sdf::Box& shape = *sdf_geometry.BoxShape();
      const Vector3d box_size = ToVector3(shape.Size());
      return make_unique<geometry::Box>(box_size(0), box_size(1), box_size(2));
    }
    case sdf::GeometryType::CYLINDER: {
      // TODO(amcastro-tri): Verify with @nkoenig that sdf::Cylinder's axis
      // point in the positive z direction as Drake's cylinders do.
      const sdf::Cylinder& shape = *sdf_geometry.CylinderShape();
      return make_unique<geometry::Cylinder>(shape.Radius(), shape.Length());
    }
    case sdf::GeometryType::PLANE: {
      // While sdf::Plane contains the normal of the plane, geometry::HalfSpace
      // only encodes a half space with normal along the z-axis direction of a
      // canonical frame C. Therefore the normal information is used during
      // the parsing of a GeometryInstance, which does contain the pose of the
      // half space in the parent link frame.
      return make_unique<geometry::HalfSpace>();
    }
    case sdf::GeometryType::SPHERE: {
      const sdf::Sphere& shape = *sdf_geometry.SphereShape();
      return make_unique<geometry::Sphere>(shape.Radius());
    }
    // TODO(amcastro-tri): When sdformat supports it add the MESH case.
    default: {
      throw std::logic_error("Geometry type not supported.");
    }
  }
}

std::unique_ptr<GeometryInstance> MakeGeometryInstanceFromSdfVisual(
    const sdf::Visual& sdf_visual) {
  // Retrieve the pose of the visual frame G in the parent link L in which
  // geometry gets defined.
  const Isometry3d X_LG = ToIsometry3(sdf_visual.Pose());
  const sdf::Geometry& sdf_geometry = *sdf_visual.Geom();

  // Nothing left to do, return nullptr signaling this is an empty visual.
  if (sdf_geometry.Type() == sdf::GeometryType::EMPTY) {
    return std::unique_ptr<GeometryInstance>(nullptr);
  }

  // GeometryInstance defines its shapes in a "canonical frame" C. For instance:
  // - A half-space's normal is directed along the Cz axis,
  // - A cylinder's length is parallel to the Cz axis,
  // - etc.

  // X_LC defines the pose of the canonical frame in the link frame L.
  Isometry3d X_LC = X_LG;  // In most cases C coincides with the SDF G frame.

  // For a half-space, C and G are not the same since  SDF allows to specify
  // the normal of the plane in the G frame.
  if (sdf_geometry.Type() == sdf::GeometryType::PLANE) {
    const sdf::Plane& shape = *sdf_geometry.PlaneShape();
    // TODO(amcastro-tri): we assume the normal is in the frame of the visual
    // geometry G. Verify this with @nkoenig.
    const Vector3d normal_G = ToVector3(shape.Normal());
    // sdf::Plane also has sdf::Plane::Size(), but we ignore it since in Drake
    // planes are entire half-spaces.

    // The normal expressed in the frame G defines the pose of the half space
    // in its canonical frame C in which the normal aligns with the z-axis
    // direction.
    const Isometry3d X_GC =
        geometry::HalfSpace::MakePose(normal_G, Vector3d::Zero());

    // Correct X_LC to include the pose X_GC
    X_LC = X_LG * X_GC;
  }

  // TODO(amcastro-tri): Extract <material> once sdf::Visual supports it.

  return make_unique<GeometryInstance>(
      X_LC, MakeShapeFromSdfGeometry(sdf_geometry));
}

Isometry3d MakeGeometryPoseFromSdfCollision(
    const sdf::Collision& sdf_collision) {
  // Retrieve the pose of the collision frame G in the parent link L in which
  // geometry gets defined.
  const Isometry3d X_LG = ToIsometry3(sdf_collision.Pose());

  // GeometryInstance defines its shapes in a "canonical frame" C. For instance:
  // - A half-space's normal is directed along the Cz axis,
  // - A cylinder's length is parallel to the Cz axis,
  // - etc.

  // X_LC defines the pose of the canonical frame in the link frame L.
  Isometry3d X_LC = X_LG;  // In most cases C coincides with the SDF G frame.

  // For a half-space, C and G are not the same since  SDF allows to specify
  // the normal of the plane in the G frame.
  const sdf::Geometry& sdf_geometry = *sdf_collision.Geom();
  if (sdf_geometry.Type() == sdf::GeometryType::PLANE) {
    const sdf::Plane& shape = *sdf_geometry.PlaneShape();
    const Vector3d normal_G = ToVector3(shape.Normal());
    // sdf::Plane also has sdf::Plane::Size(), but we ignore it since in Drake
    // planes are entire half-spaces.

    // The normal expressed in the frame G defines the pose of the half space
    // in its canonical frame C in which the normal aligns with the z-axis
    // direction.
    const Isometry3d X_GC =
        geometry::HalfSpace::MakePose(normal_G, Vector3d::Zero());

    // Correct X_LC to include the pose X_GC
    X_LC = X_LG * X_GC;
  }
  return X_LC;
}

CoulombFriction<double> MakeCoulombFrictionFromSdfCollision(
    const sdf::Collision& sdf_collision) {

  const sdf::ElementPtr collision_element = sdf_collision.Element();
  // Element pointers can only be nullptr if Load() was not called on the sdf::
  // object. Only a bug could cause this.
  DRAKE_DEMAND(collision_element != nullptr);

  const sdf::ElementPtr friction_element =
      GetElementPointerOrNullPtr(collision_element, "drake_friction");

  // If friction_element is not found, the default is that of a frictionless
  // surface (i.e. zero friction coefficients).
  if (!friction_element) return CoulombFriction<double>();

  // Once <drake_friction> is (optionally) specified, <static_friction> and
  // <dynamic_friction> are required.
  const double static_friction = GetValueOrThrow<double>(
      friction_element, "static_friction");
  const double dynamic_friction = GetValueOrThrow<double>(
      friction_element, "dynamic_friction");

  try {
    return CoulombFriction<double>(static_friction, dynamic_friction);
  } catch (std::logic_error& e) {
    throw std::logic_error("From <collision> with name '" +
        sdf_collision.Name() + "': " + e.what());
  }
}

CoulombFriction<double> MakeCoulombFrictionFromSdfCollisionOde(
    const sdf::Collision& sdf_collision) {

  const sdf::ElementPtr collision_element = sdf_collision.Element();
  // Element pointers can only be nullptr if Load() was not called on the sdf::
  // object. Only a bug could cause this.
  DRAKE_DEMAND(collision_element != nullptr);

  const sdf::ElementPtr surface_element =
      GetElementPointerOrNullPtr(collision_element, "surface");

  // If the surface is not found, the default is that of a frictionless
  // surface (i.e. zero friction coefficients).
  if (!surface_element) return CoulombFriction<double>();

  // Once <surface> is found, <friction> and <ode> are required.
  const sdf::ElementPtr friction_element =
      GetElementPointerOrThrow(surface_element, "friction");
  const sdf::ElementPtr ode_element =
      GetElementPointerOrThrow(friction_element, "ode");


  // Once <ode> is found, <mu> (for static) and <mu2> (for dynamic) are
  // required.
  const double static_friction = GetValueOrThrow<double>(ode_element, "mu");
  const double dynamic_friction = GetValueOrThrow<double>(ode_element, "mu2");

  try {
    return CoulombFriction<double>(static_friction, dynamic_friction);
  } catch (std::logic_error& e) {
    throw std::logic_error("From <collision> with name '" +
        sdf_collision.Name() + "': " + e.what());
  }
}

}  // namespace detail

}  // namespace parsing
}  // namespace multibody
}  // namespace drake
