#pragma once

#include "drake/geometry/query_results/contact_surface.h"
#include "drake/geometry/query_results/surface_mesh.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/math/spatial_force.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {

template <class T>
class MultibodyPlant;

namespace hydroelastic_contact {

template <class T>
class HydroelasticTraction {
 public:
  HydroelasticTraction(MultibodyPlant<T>* plant) : plant_(plant) {}

  Vector3<T> CalcTractionAtPoint(
      const systems::Context<T>& context,
      const geometry::ContactSurface<T>& surface,
      geometry::SurfaceFaceIndex face_index,
      const typename geometry::SurfaceMesh<T>::Barycentric&
          r_barycentric_M, const Vector3<T>& r_W,
      double dissipation, double mu_coulomb) const;
  void ShiftTractionToSpatialForcesAtBodyOrigins(
      const systems::Context<T>& context,
      const geometry::ContactSurface<T>& surface,
      const Vector3<T>& r_W,
      const Vector3<T>& traction_W,
      multibody::SpatialForce<T>* f_Mo_W,
      multibody::SpatialForce<T>* f_No_W) const;

 private:
  const math::RigidTransform<T>& GetTransformFromGeometry(
      const systems::Context<T>& context,
      geometry::GeometryId geometry_id) const;
  Vector3<T> CalcContactPoint(
      const systems::Context<T>& context,
      const geometry::ContactSurface<T>& surface,
      geometry::SurfaceFaceIndex face_index,
      const typename geometry::SurfaceMesh<T>::Barycentric& r_barycentric_M)
      const;

  MultibodyPlant<T>* plant_{nullptr};
};

}  // namespace hydroelastic_contact
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::hydroelastic_contact::HydroelasticTraction)
