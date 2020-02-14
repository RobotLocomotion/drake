#pragma once

#include <memory>
#include <utility>
#include <variant>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/multibody/math/spatial_force.h"
#include "drake/multibody/plant/hydroelastic_quadrature_point_data.h"

namespace drake {
namespace multibody {

/**
 A class containing information regarding contact and contact response between
 two geometries attached to a pair of bodies. This class provides the output
 from the Hydroelastic contact model and includes:

    - The shared contact surface between the two geometries, which includes
      the virtual pressures acting at every point on the contact surface.
    - The tractions acting at the quadrature points on the contact surface.
    - The slip speeds at the quadrature points on the contact surface.
    - The spatial force from the integrated tractions that is applied at the
      centroid of the contact surface.

 The two geometries, denoted M and N (and obtainable via
 `contact_surface().id_M()` and `contact_surface().id_N()`) are attached to
 bodies A and B, respectively.

 @tparam T Must be one of drake's default scalar types.
 */
template <typename T>
class HydroelasticContactInfo {
 public:
  /**
   Constructs this structure using the given contact surface, traction field,
   and slip field. This constructor does not own the ContactSurface; it points
   to a ContactSurface that another owns.
   @see contact_surface()
   @see traction_A_W()
   @see vslip_AB_W()
   */
  HydroelasticContactInfo(
      const geometry::ContactSurface<T>* contact_surface,
      const SpatialForce<T>& F_Ac_W,
      std::vector<HydroelasticQuadraturePointData<T>>&& quadrature_point_data)
      : contact_surface_(contact_surface),
        F_Ac_W_(F_Ac_W),
        quadrature_point_data_(std::move(quadrature_point_data)) {
    DRAKE_DEMAND(contact_surface);
  }

  /**
   Constructs this structure using the given contact surface, traction field,
   and slip field.  This constructor takes ownership of the ContactSurface.
   @see contact_surface()
   */
  HydroelasticContactInfo(
      std::unique_ptr<geometry::ContactSurface<T>> contact_surface,
      const SpatialForce<T>& F_Ac_W,
      std::vector<HydroelasticQuadraturePointData<T>>&& quadrature_point_data)
      : contact_surface_(std::move(contact_surface)),
        F_Ac_W_(F_Ac_W),
        quadrature_point_data_(std::move(quadrature_point_data)) {
    DRAKE_DEMAND(std::get<std::unique_ptr<geometry::ContactSurface<T>>>(
                     contact_surface_)
                     .get());
  }

  /// @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
  /// MoveAssignable.
  //@{

  /** Clones this data structure, making deep copies of all underlying data.
   @note The new object will contain a cloned ContactSurface even if the
         original was constructed using a raw pointer referencing an existing
         ContactSurface.
   */
  HydroelasticContactInfo(const HydroelasticContactInfo& info) {
    *this = info;
  }

  /** Clones this object in the same manner as the copy constructor.
   @see HydroelasticContactInfo(const HydroelasticContactInfo&)
   */
  HydroelasticContactInfo& operator=(const HydroelasticContactInfo& info) {
    contact_surface_ =
        std::make_unique<geometry::ContactSurface<T>>(info.contact_surface());
    F_Ac_W_ = info.F_Ac_W_;
    quadrature_point_data_ = info.quadrature_point_data_;
    return *this;
  }

  HydroelasticContactInfo(HydroelasticContactInfo&&) = default;
  HydroelasticContactInfo& operator=(HydroelasticContactInfo&&) = default;

  //@}

  /// Returns a reference to the ContactSurface data structure. Note that
  /// the mesh and gradient vector fields are expressed in the world frame.
  const geometry::ContactSurface<T>& contact_surface() const {
    if (std::holds_alternative<const geometry::ContactSurface<T>*>(
            contact_surface_)) {
      return *std::get<const geometry::ContactSurface<T>*>(contact_surface_);
    } else {
      return *std::get<std::unique_ptr<geometry::ContactSurface<T>>>
                  (contact_surface_);
    }
  }

  /// Gets the intermediate data, including tractions, computed by the
  /// quadrature process.
  const std::vector<HydroelasticQuadraturePointData<T>>& quadrature_point_data()
      const {
    return quadrature_point_data_;
  }

  /// Gets the spatial force applied at the centroid (Point C) of the surface
  /// mesh (C can be obtained through `contact_surface().mesh_W().`).
  const SpatialForce<T>& F_Ac_W() const { return F_Ac_W_; }

 private:
  // Note that the mesh of the contact surface is defined in the world frame.
  std::variant<const geometry::ContactSurface<T>*,
                 std::unique_ptr<geometry::ContactSurface<T>>> contact_surface_;

  // The spatial force applied at the centroid (Point C) of the surface mesh.
  SpatialForce<T> F_Ac_W_;

  // The traction and slip velocity evaluated at each quadrature point.
  std::vector<HydroelasticQuadraturePointData<T>>
      quadrature_point_data_;
};

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::HydroelasticContactInfo)
