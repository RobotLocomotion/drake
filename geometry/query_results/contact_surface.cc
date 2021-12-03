#include "drake/geometry/query_results/contact_surface.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace geometry {

using std::make_unique;
using std::move;
using std::unique_ptr;
using std::vector;

template <typename T>
const Vector3<T>& ContactSurface<T>::EvaluateGradE_M_W(int index) const {
  if (grad_eM_W_ == nullptr) {
    throw std::runtime_error(
        "ContactSurface::EvaluateGradE_M_W() invalid; no gradient values "
        "stored. Mesh M may be rigid, or the constituent gradients weren't "
        "requested.");
  }
  return (*grad_eM_W_)[index];
}

template <typename T>
const Vector3<T>& ContactSurface<T>::EvaluateGradE_N_W(int index) const {
  if (grad_eN_W_ == nullptr) {
    throw std::runtime_error(
        "ContactSurface::EvaluateGradE_N_W() invalid; no gradient values "
        "stored. Mesh N may be rigid, or the constituent gradients weren't "
        "requested.");
  }
  return (*grad_eN_W_)[index];
}

template <typename T>
bool ContactSurface<T>::Equal(const ContactSurface<T>& surface) const {
  // Confirm we have the same representation. Technically, mesh and field
  // representations are linked, but we'll test both to be safe.
  if (this->mesh_W_.index() != surface.mesh_W_.index()) return false;
  if (this->e_MN_.index() != surface.e_MN_.index()) return false;

  if (is_triangle()) {
    if (!this->tri_mesh_W().Equal(surface.tri_mesh_W())) return false;
    if (!this->tri_e_MN().Equal(surface.tri_e_MN())) return false;
  } else {
    if (!this->poly_mesh_W().Equal(surface.poly_mesh_W())) return false;
    if (!this->poly_e_MN().Equal(surface.poly_e_MN())) return false;
  }

  // TODO(SeanCurtis-TRI) This isn't testing the following quantities:
  //  1. Geometry ids
  //  2. Gradients.

  // All checks passed.
  return true;
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ContactSurface);

}  // namespace geometry
}  // namespace drake
