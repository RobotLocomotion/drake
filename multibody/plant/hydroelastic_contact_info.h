#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_variant.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/query_results/contact_surface.h"

namespace drake {
namespace multibody {

/**
 A class containing information regarding contact and contact response between
 two geometries attached to a pair of bodies. This class provides the output
 from the Hydroelastic contact model and includes:

    - The shared contact surface between the two geometries, which includes
      the virtual pressure acting at every point on the contact surface.
    - The traction acting at every point on the contact surface.
    - The slip speed at every point on the contact surface.

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
      std::unique_ptr<geometry::SurfaceMeshField<Vector3<T>, T>> traction_A_W,
      std::unique_ptr<geometry::SurfaceMeshField<Vector3<T>, T>> vslip_AB_W) :
      contact_surface_(contact_surface),
      traction_A_W_(std::move(traction_A_W)),
      vslip_AB_W_(std::move(vslip_AB_W)) {
    DRAKE_DEMAND(contact_surface);
    DRAKE_DEMAND(traction_A_W_.get());
    DRAKE_DEMAND(vslip_AB_W_.get());
  }

  /**
   Constructs this structure using the given contact surface, traction field,
   and slip field.  This constructor takes ownership of the ContactSurface.
   @see contact_surface()
   @see traction_A_W()
   @see vslip_AB_W()
   */
  HydroelasticContactInfo(
      std::unique_ptr<geometry::ContactSurface<T>> contact_surface,
      std::unique_ptr<geometry::SurfaceMeshField<Vector3<T>, T>> traction_A_W,
      std::unique_ptr<geometry::SurfaceMeshField<Vector3<T>, T>> vslip_AB_W) :
      contact_surface_(std::move(contact_surface)),
      traction_A_W_(std::move(traction_A_W)),
      vslip_AB_W_(std::move(vslip_AB_W)) {
    DRAKE_DEMAND(drake::get<std::unique_ptr<geometry::ContactSurface<T>>>(
                     contact_surface_).get());
    DRAKE_DEMAND(traction_A_W_.get());
    DRAKE_DEMAND(vslip_AB_W_.get());
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
    const geometry::SurfaceMesh<T>& mesh = contact_surface().mesh_W();
    traction_A_W_ = info.traction_A_W_->CloneAndSetMesh(&mesh);
    vslip_AB_W_ = info.vslip_AB_W_->CloneAndSetMesh(&mesh);
    return *this;
  }

  HydroelasticContactInfo(HydroelasticContactInfo&&);
  HydroelasticContactInfo& operator=(HydroelasticContactInfo&&) = default;

  //@}

  /// Returns a reference to the ContactSurface data structure. Note that
  /// the mesh and gradient vector fields are expressed in the world frame.
  const geometry::ContactSurface<T>& contact_surface() const {
    if (drake::holds_alternative<const geometry::ContactSurface<T>*>(
            contact_surface_)) {
      return *drake::get<const geometry::ContactSurface<T>*>(contact_surface_);
    } else {
      return *drake::get<std::unique_ptr<geometry::ContactSurface<T>>>
                  (contact_surface_);
    }
  }

  /// Returns the field giving the traction acting on Body A, expressed in the
  /// world frame. At each point Q on the contact surface, `traction_A_W` gives
  /// the traction `traction_Aq_W`, where `Aq` is a frame attached to Body A and
  /// shifted to Q.
  const geometry::SurfaceMeshField<Vector3<T>, T>& traction_A_W() const {
    return *traction_A_W_;
  }

  /// Returns the field giving the slip velocity of Body B relative to Body A,
  /// expressed in the world frame. At each point Q on the contact surface,
  /// `vslip_AB_W` gives the slip velocity `vslip_AqBq_W`, which is the
  /// "tangential" velocity of Frame Bq (located at Q and attached Frame B)
  /// relative to the tangential velocity of Frame Aq (also located at Q and
  /// attached to Frame B). The tangential velocity at Q corresponds to the
  /// components of velocity orthogonal to the normal to the contact surface at
  /// Q.
  const geometry::SurfaceMeshField<Vector3<T>, T>& vslip_AB_W() const {
    return *vslip_AB_W_;
  }

 private:
  // Note that the mesh of the contact surface is defined in the world frame.
  drake::variant<const geometry::ContactSurface<T>*,
                 std::unique_ptr<geometry::ContactSurface<T>>> contact_surface_;
  std::unique_ptr<geometry::SurfaceMeshField<Vector3<T>, T>> traction_A_W_;
  std::unique_ptr<geometry::SurfaceMeshField<Vector3<T>, T>> vslip_AB_W_;
};

// Workaround for https://gcc.gnu.org/bugzilla/show_bug.cgi?id=57728 which
// should be moved back into the class definition once we no longer need to
// support GCC versions prior to 6.3.
template <typename T>
HydroelasticContactInfo<T>::HydroelasticContactInfo(
    HydroelasticContactInfo&&) = default;

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::HydroelasticContactInfo)
