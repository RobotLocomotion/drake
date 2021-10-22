#include "drake/multibody/fixed_fem/dev/collision_objects.h"

#include "drake/common/nice_type_name.h"
#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/geometry/proximity/make_capsule_mesh.h"
#include "drake/geometry/proximity/make_cylinder_mesh.h"
#include "drake/geometry/proximity/make_ellipsoid_mesh.h"
#include "drake/geometry/proximity/make_sphere_mesh.h"
#include "drake/geometry/proximity/obj_to_surface_mesh.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
using geometry::Box;
using geometry::Capsule;
using geometry::Convex;
using geometry::Cylinder;
using geometry::Ellipsoid;
using geometry::HalfSpace;
using geometry::Mesh;
using geometry::ReadObjToTriangleSurfaceMesh;
using geometry::Sphere;
using geometry::TriangleSurfaceMesh;
using geometry::internal::MakeBoxSurfaceMesh;
using geometry::internal::MakeCapsuleSurfaceMesh;
using geometry::internal::MakeCylinderSurfaceMesh;
using geometry::internal::MakeEllipsoidSurfaceMesh;
using geometry::internal::MakeSphereSurfaceMesh;

template <typename ShapeType>
TriangleSurfaceMesh<double> MakeRigidSurfaceMesh(const ShapeType& shape,
                                                 double) {
  throw std::logic_error(fmt::format(
      "Trying to make a rigid surface mesh for an unsupported type shape. "
      "The types supported are: Sphere, Box, Cylinder, Capsule, Ellipsoid, "
      "Mesh and Convex. The shape provided is {}.",
      NiceTypeName::Get(shape)));
  DRAKE_UNREACHABLE();
}

TriangleSurfaceMesh<double> MakeRigidSurfaceMesh(const Sphere& sphere,
                                         double resolution_hint) {
  return MakeSphereSurfaceMesh<double>(sphere, resolution_hint);
}

TriangleSurfaceMesh<double> MakeRigidSurfaceMesh(const Box& box, double) {
  // Use the coarsest mesh for the box. The safety factor 1.1 guarantees the
  // resolution-hint argument is larger than the box size, so the mesh
  // will have only 8 vertices and 12 triangles.
  return MakeBoxSurfaceMesh<double>(box, 1.1 * box.size().maxCoeff());
}

TriangleSurfaceMesh<double> MakeRigidSurfaceMesh(const Cylinder& cylinder,
                                         double resolution_hint) {
  return MakeCylinderSurfaceMesh<double>(cylinder, resolution_hint);
}

TriangleSurfaceMesh<double> MakeRigidSurfaceMesh(const Capsule& capsule,
                                         double resolution_hint) {
  return MakeCapsuleSurfaceMesh<double>(capsule, resolution_hint);
}

TriangleSurfaceMesh<double> MakeRigidSurfaceMesh(const Ellipsoid& ellipsoid,
                                         double resolution_hint) {
  return MakeEllipsoidSurfaceMesh<double>(ellipsoid, resolution_hint);
}

TriangleSurfaceMesh<double> MakeRigidSurfaceMesh(const Mesh& mesh_spec,
                                                 double) {
  return ReadObjToTriangleSurfaceMesh(mesh_spec.filename(), mesh_spec.scale());
}

TriangleSurfaceMesh<double> MakeRigidSurfaceMesh(const Convex& convex_spec,
                                                 double) {
  return ReadObjToTriangleSurfaceMesh(convex_spec.filename(),
                                      convex_spec.scale());
}

template <typename T>
void CollisionObjects<T>::ImplementGeometry(const Sphere& sphere,
                                            void* user_data) {
  MakeRigidRepresentation(sphere, *static_cast<ReifyData*>(user_data));
}

template <typename T>
void CollisionObjects<T>::ImplementGeometry(const Cylinder& cylinder,
                                            void* user_data) {
  MakeRigidRepresentation(cylinder, *static_cast<ReifyData*>(user_data));
}

template <typename T>
void CollisionObjects<T>::ImplementGeometry(const HalfSpace& half_space,
                                            void* user_data) {
  MakeRigidRepresentation(half_space, *static_cast<ReifyData*>(user_data));
}

template <typename T>
void CollisionObjects<T>::ImplementGeometry(const Box& box, void* user_data) {
  MakeRigidRepresentation(box, *static_cast<ReifyData*>(user_data));
}

template <typename T>
void CollisionObjects<T>::ImplementGeometry(const Capsule& capsule,
                                            void* user_data) {
  MakeRigidRepresentation(capsule, *static_cast<ReifyData*>(user_data));
}

template <typename T>
void CollisionObjects<T>::ImplementGeometry(const Ellipsoid& ellipsoid,
                                            void* user_data) {
  MakeRigidRepresentation(ellipsoid, *static_cast<ReifyData*>(user_data));
}

template <typename T>
void CollisionObjects<T>::ImplementGeometry(const Mesh& mesh, void* user_data) {
  MakeRigidRepresentation(mesh, *static_cast<ReifyData*>(user_data));
}

template <typename T>
void CollisionObjects<T>::ImplementGeometry(const Convex& convex,
                                            void* user_data) {
  MakeRigidRepresentation(convex, *static_cast<ReifyData*>(user_data));
}

template <typename T>
template <typename ShapeType>
void CollisionObjects<T>::MakeRigidRepresentation(const ShapeType& shape,
                                                  const ReifyData& data) {
  // This ("fem/dev", "resolution_hint") property is a stopgap. We don't want
  // to use hydro's resolution hint as that would needlessly entangle this
  // functionality with that. This property name isn't the *final* name, but,
  // while in dev, it gives us some power to articulate resolution.
  // TODO(SeanCurtis-TRI) Define a final name for this property and document it
  // appropriately.
  rigid_representations_[data.id] = {
      std::make_unique<TriangleSurfaceMesh<double>>(MakeRigidSurfaceMesh(
          shape, data.properties.GetPropertyOrDefault(
                     "fem/dev", "resolution_hint", kDefaultResolutionHint))),
      data.properties};
  geometry_ids_.emplace_back(data.id);
}
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::fem::internal::CollisionObjects);
