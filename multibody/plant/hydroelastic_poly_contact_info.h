#pragma once

#include <memory>
#include <utility>
#include <variant>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/query_results/polygonal_contact_surface.h"
#include "drake/multibody/math/spatial_algebra.h"
#ifdef DRAKE_POLY_QUADRATURE_DATA
#include "drake/multibody/plant/hydroelastic_quadrature_point_data.h"
#endif

namespace drake {
namespace multibody {

/**
 A class containing information regarding contact and contact response between
 two geometries attached to a pair of bodies. This class provides the output
 from the Hydroelastic contact model and includes:

    - The shared polygonal contact surface between the two geometries, which
      includes
      the virtual pressures acting at every point on the contact surface.
    - The tractions acting at the quadrature points on the contact surface.
    - The slip speeds at the quadrature points on the contact surface.
    - The spatial force from the integrated tractions that is applied at the
      centroid of the contact surface.

 The two geometries, denoted M and N (and obtainable via
 `contact_surface().id_M()` and `contact_surface().id_N()`) are attached to
 bodies A and B, respectively.

 @tparam_default_scalar
 */
template <typename T>
class HydroelasticPolyContactInfo {
 public:
  /** @name                 Construction
   The constructors below adhere to a number of invariants. The
   geometry::PolygonalontactSurface defines contact between two geometries M and
   N (via contact_surface().id_M() and contact_surface().id_N(), respectively).
   %HydroelasticPolyContactInfo further associates geometries M and N with the
   bodies to which they are rigidly fixed, A and B, respectively. It is the
   responsibility of the caller of these constructors to ensure that the
   parameters satisfy the documented invariants, see below. Similarly, the
   spatial force `F_Ac_W` must be provided as indicated by the monogram notation
   in use, that is, it is the spatial force on body A, at the contact surface's
   centroid C, and expressed in the world frame. Quadrature points data must be
   provided in accordance to the conventions and monogram notation documented in
   HydroelasticQuadraturePointData. */
  // @{

  /**
   @anchor hydro_contact_info_non_owning_ctor
   Constructs this structure using the given contact surface, traction field,
   and slip field. This constructor does not own the PolygonalContactSurface; it
   points to a PolygonalContactSurface that another owns, see contact_surface().

   @param[in] contact_surface Contact surface between two geometries M and N,
     see geometry::PolygonalContactSurface::id_M() and
     geometry::PolygonalContactSurface::id_N().
   @param[in] F_Ac_W Spatial force applied on body A, at contact surface
     centroid C, and expressed in the world frame W. The position `p_WC` of C in
     the world frame W can be obtained with
     `PolygonalContactSurface::mesh_W().centroid()`.
   @param[in] quadrature_point_data Hydroelastic field data at each quadrature
     point. Data must be provided in accordance to the convention that geometry
     M and N are attached to bodies A and B, respectively. Refer to
     HydroelasticQuadraturePointData for further details.
   */
  HydroelasticPolyContactInfo(
      const geometry::PolygonalContactSurface<T>* contact_surface,
      const SpatialForce<T>& F_Ac_W
#ifdef DRAKE_POLY_QUADRATURE_DATA
      ,
      std::vector<HydroelasticQuadraturePointData<T>>&& quadrature_point_data
#endif
      )
      : contact_surface_(contact_surface),
        F_Ac_W_(F_Ac_W)
#ifdef DRAKE_POLY_QUADRATURE_DATA
        ,
        quadrature_point_data_(std::move(quadrature_point_data))
#endif
  {
    DRAKE_DEMAND(contact_surface != nullptr);
  }

  /** This constructor takes ownership of `contact_surface` via a
   std::unique_ptr, instead of aliasing a pre-existing contact surface. In all
   other respects, it is identical to the @ref
   hydro_contact_info_non_owning_ctor "other overload" that takes
   `contact_surface` by raw pointer.  */
  HydroelasticPolyContactInfo(
      std::unique_ptr<geometry::PolygonalContactSurface<T>> contact_surface,
      const SpatialForce<T>& F_Ac_W
#ifdef DRAKE_POLY_QUADRATURE_DATA
      ,
      std::vector<HydroelasticQuadraturePointData<T>>&& quadrature_point_data
#endif
      )
      : contact_surface_(std::move(contact_surface)),
        F_Ac_W_(F_Ac_W)
#ifdef DRAKE_POLY_QUADRATURE_DATA
        ,
        quadrature_point_data_(std::move(quadrature_point_data))
#endif
  {
    DRAKE_DEMAND(
        std::get<std::unique_ptr<geometry::PolygonalContactSurface<T>>>(
            contact_surface_) != nullptr);
  }
  // @}

  /// @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
  /// MoveAssignable.
  //@{

  /** Clones this data structure, making deep copies of all underlying data.
   @note The new object will contain a cloned PolygonalContactSurface even if
         the original was constructed using a raw pointer referencing an
         existing PolygonalContactSurface.
   */
  HydroelasticPolyContactInfo(const HydroelasticPolyContactInfo& info) {
    *this = info;
  }

  /** Clones this object in the same manner as the copy constructor.
   @see HydroelasticPolyContactInfo(const HydroelasticPolyContactInfo&)
   */
  HydroelasticPolyContactInfo& operator=(
      const HydroelasticPolyContactInfo& info) {
    contact_surface_ = std::make_unique<geometry::PolygonalContactSurface<T>>(
        info.contact_surface());
    F_Ac_W_ = info.F_Ac_W_;
#ifdef DRAKE_POLY_QUADRATURE_DATA
    quadrature_point_data_ = info.quadrature_point_data_;
#endif
    return *this;
  }

  HydroelasticPolyContactInfo(HydroelasticPolyContactInfo&&) = default;
  HydroelasticPolyContactInfo& operator=(HydroelasticPolyContactInfo&&) =
      default;

  //@}

  /* Returns a reference to the PolygonalContactSurface data structure. Note
   that the mesh and gradient vector fields are expressed in the world frame. */
  const geometry::PolygonalContactSurface<T>& contact_surface() const {
    if (std::holds_alternative<const geometry::PolygonalContactSurface<T>*>(
            contact_surface_)) {
      return *std::get<const geometry::PolygonalContactSurface<T>*>(
          contact_surface_);
    } else {
      return *std::get<std::unique_ptr<geometry::PolygonalContactSurface<T>>>(
          contact_surface_);
    }
  }
#ifdef DRAKE_POLY_QUADRATURE_DATA
  /// Gets the intermediate data, including tractions, computed by the
  /// quadrature process.
  const std::vector<HydroelasticQuadraturePointData<T>>& quadrature_point_data()
      const {
    return quadrature_point_data_;
  }
#endif
  /// Gets the spatial force applied on body A, at the centroid point C of the
  /// surface mesh M, and expressed in the world frame W. The position `p_WC` of
  /// the centroid point C in the world frame W can be obtained with
  /// `contact_surface().mesh_W().centroid()`.
  const SpatialForce<T>& F_Ac_W() const { return F_Ac_W_; }

 private:
  // TODO(SeanCurtis-TRI): I suspect this should be cleaned up -- I don't think
  //  this class *ever* owns the underlying contact surface.
  // Note that the mesh of the contact surface is defined in the world frame.
  std::variant<const geometry::PolygonalContactSurface<T>*,
               std::unique_ptr<geometry::PolygonalContactSurface<T>>>
      contact_surface_;

  // The spatial force applied at the centroid (Point C) of the surface mesh.
  SpatialForce<T> F_Ac_W_;
#ifdef DRAKE_POLY_QUADRATURE_DATA
  // The traction and slip velocity evaluated at each quadrature point.
  std::vector<HydroelasticQuadraturePointData<T>>
      quadrature_point_data_;
#endif
};

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::HydroelasticPolyContactInfo)
