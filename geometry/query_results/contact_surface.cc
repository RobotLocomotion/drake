#include "drake/geometry/query_results/contact_surface.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace geometry {

using std::make_unique;
using std::unique_ptr;
using std::vector;

template <typename T>
ContactSurface<T>::ContactSurface(
    GeometryId id_M, GeometryId id_N, MeshVariant mesh_W, FieldVariant e_MN,
    std::unique_ptr<std::vector<Vector3<T>>> grad_eM_W,
    std::unique_ptr<std::vector<Vector3<T>>> grad_eN_W, int)
    : id_M_(id_M),
      id_N_(id_N),
      mesh_W_(std::move(mesh_W)),
      e_MN_(std::move(e_MN)),
      grad_eM_W_(std::move(grad_eM_W)),
      grad_eN_W_(std::move(grad_eN_W)) {
  // If defined the gradient values must map 1-to-1 onto elements.
  if (is_triangle()) {
    DRAKE_THROW_UNLESS(grad_eM_W_ == nullptr ||
                       static_cast<int>(grad_eM_W_->size()) ==
                           tri_mesh_W().num_elements());
    DRAKE_THROW_UNLESS(grad_eN_W_ == nullptr ||
                       static_cast<int>(grad_eN_W_->size()) ==
                           tri_mesh_W().num_elements());
  } else {
    DRAKE_THROW_UNLESS(grad_eM_W_ == nullptr ||
                       static_cast<int>(grad_eM_W_->size()) ==
                           poly_mesh_W().num_elements());
    DRAKE_THROW_UNLESS(grad_eN_W_ == nullptr ||
                       static_cast<int>(grad_eN_W_->size()) ==
                           poly_mesh_W().num_elements());
  }
  if constexpr (scalar_predicate<T>::is_bool) {
    if (id_N_ < id_M_) SwapMAndN();
  }
}

template <typename T>
void ContactSurface<T>::SwapMAndN() {
  std::swap(id_M_, id_N_);

  // TODO(SeanCurtis-TRI): Determine if this work is necessary. It is neither
  // documented nor tested that the face winding is guaranteed to be one way
  // or the other. Alternatively, this should be documented and tested.
  std::visit(
      [](auto&& mesh) {
        mesh->ReverseFaceWinding();
      },
      mesh_W_);

  // Note: the scalar field does not depend on the order of M and N.
  std::swap(grad_eM_W_, grad_eN_W_);
}

template <typename T>
ContactSurface<T>::ContactSurface(const ContactSurface& surface) {
  *this = surface;
}

template <typename T>
ContactSurface<T>& ContactSurface<T>::operator=(const ContactSurface& surface) {
  if (&surface == this) return *this;

  id_M_ = surface.id_M_;
  id_N_ = surface.id_N_;
  if (surface.is_triangle()) {
    mesh_W_ = std::make_unique<TriangleSurfaceMesh<T>>(surface.tri_mesh_W());
    // We can't simply copy the mesh fields; the copies must contain pointers
    // to the new mesh. So, we use CloneAndSetMesh() instead.
    e_MN_ = surface.tri_e_MN().CloneAndSetMesh(&tri_mesh_W());
  } else {
    mesh_W_ = std::make_unique<PolygonSurfaceMesh<T>>(surface.poly_mesh_W());
    // We can't simply copy the mesh fields; the copies must contain pointers
    // to the new mesh. So, we use CloneAndSetMesh() instead.
    e_MN_ = surface.poly_e_MN().CloneAndSetMesh(&poly_mesh_W());
  }

  if (surface.grad_eM_W_) {
    grad_eM_W_ = std::make_unique<std::vector<Vector3<T>>>(*surface.grad_eM_W_);
  }
  if (surface.grad_eN_W_) {
    grad_eN_W_ = std::make_unique<std::vector<Vector3<T>>>(*surface.grad_eN_W_);
  }

  return *this;
}

template <typename T>
ContactSurface<T>::~ContactSurface() {}

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
