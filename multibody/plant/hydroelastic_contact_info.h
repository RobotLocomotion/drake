#pragma once

#include <memory>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/multibody/math/spatial_algebra.h"

namespace drake {
namespace multibody {

/**
 A class containing information regarding contact and contact response between
 two geometries attached to a pair of bodies. This class provides the output
 from the Hydroelastic contact model and includes:

    - The shared contact surface between the two geometries, which includes
      the virtual pressures acting at every point on the contact surface.
    - The spatial force from the integrated tractions that is applied at the
      centroid of the contact surface.

 The two geometries, denoted M and N (and obtainable via
 `contact_surface().id_M()` and `contact_surface().id_N()`) are attached to
 bodies A and B, respectively.

 When T = Expression, the class is specialized to not contain any member data,
 because ContactSurface doesn't support Expression.

 @tparam_default_scalar
 */
template <typename T>
class HydroelasticContactInfo {
 public:
  /** Constructs this structure using the given contact surface and spatial
   force.

   The geometry::ContactSurface defines contact between two geometries M and N
   (via contact_surface().id_M() and contact_surface().id_N(), respectively).
   %HydroelasticContactInfo further associates geometries M and N with the
   bodies to which they are rigidly fixed, A and B, respectively. It is the
   responsibility of the caller of this constructor to ensure that the
   parameters satisfy the documented invariants, see below. Similarly, the
   spatial force `F_Ac_W` must be provided as indicated by the monogram notation
   in use, that is, it is the spatial force on body A, at the contact surface's
   centroid C, and expressed in the world frame.

   @param[in] contact_surface Contact surface between two geometries M and N,
     see geometry::ContactSurface::id_M() and geometry::ContactSurface::id_N().
     This must point to immutable data (i.e., data that will never change),
     and must not be nullptr.
   @param[in] F_Ac_W Spatial force applied on body A, at contact surface
     centroid C, and expressed in the world frame W. The position `p_WC` of C in
     the world frame W can be obtained with
     `ContactSurface::centroid()`. */
  HydroelasticContactInfo(
      std::shared_ptr<const geometry::ContactSurface<T>> contact_surface,
      const SpatialForce<T>& F_Ac_W)
      : contact_surface_(std::move(contact_surface)), F_Ac_W_(F_Ac_W) {
    DRAKE_THROW_UNLESS(contact_surface_ != nullptr);
  }

  /** (Advanced) Constructs by aliasing the given contact_surface, without any
   expensive reference counting. It is the responsibility of the caller to
   ensure that the given contact_surface outlives this object. Copying or
   moving this object will result in a deep copy of the surface. */
  HydroelasticContactInfo(const geometry::ContactSurface<T>* contact_surface,
                          const SpatialForce<T>& F_Ac_W)
      // Delegate to the prior constructor, by using a shared_ptr with no
      // managed object.
      : HydroelasticContactInfo(
            std::shared_ptr<const geometry::ContactSurface<T>>(
                /* managed object = */ std::shared_ptr<const void>{},
                /* stored pointer = */ contact_surface),
            F_Ac_W) {}

  /** @name Implements CopyConstructible, CopyAssignable */
  /** @{ */
  HydroelasticContactInfo(const HydroelasticContactInfo&);
  HydroelasticContactInfo& operator=(const HydroelasticContactInfo&);
  // The move operators would offer no real performance advantage over copying,
  // so we don't provide either of the move overloads.
  /** @} */

  ~HydroelasticContactInfo();

  /** Returns a reference to the ContactSurface data structure. Note that
   the mesh and gradient vector fields are expressed in the world frame. */
  const geometry::ContactSurface<T>& contact_surface() const {
    return *contact_surface_;
  }

  /** Gets the spatial force applied on body A, at the centroid point C of the
   surface mesh M, and expressed in the world frame W. The position `p_WC` of
   the centroid point C in the world frame W can be obtained with
   `contact_surface().centroid()`. */
  const SpatialForce<T>& F_Ac_W() const { return F_Ac_W_; }

 private:
  // Note that the mesh of the contact surface is defined in the world frame.
  // This is never nullptr.
  std::shared_ptr<const geometry::ContactSurface<T>> contact_surface_;

  // The spatial force applied at the centroid (Point C) of the surface mesh.
  SpatialForce<T> F_Ac_W_;
};

#ifndef __MKDOC_PY__
/** Full specialization of HydroelasticContactInfo for T = Expression, with
 no member data. */
template <>
class HydroelasticContactInfo<symbolic::Expression> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(HydroelasticContactInfo);
  HydroelasticContactInfo() = default;
};
#endif

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::HydroelasticContactInfo);
