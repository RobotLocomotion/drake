#include "drake/geometry/proximity/mesh_field_linear.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace geometry {

template <class T, class MeshType>
MeshFieldLinear<T, MeshType>::MeshFieldLinear(std::string name,
                                              std::vector<T>&& values,
                                              const MeshType* mesh,
                                              bool calculate_gradient)
    : mesh_(mesh), name_(std::move(name)), values_(std::move(values)) {
  DRAKE_DEMAND(mesh_ != nullptr);
  DRAKE_DEMAND(static_cast<int>(values_.size()) == this->mesh().num_vertices());
  if (calculate_gradient) {
    CalcGradientField();
    CalcValueAtMeshOriginForAllElements();
    DRAKE_DEMAND(mesh->num_elements() == static_cast<int>(gradients_.size()));
    DRAKE_DEMAND(mesh->num_elements() ==
                 static_cast<int>(values_at_Mo_.size()));
  }
}

template <class T, class MeshType>
std::unique_ptr<MeshFieldLinear<T, MeshType>>
MeshFieldLinear<T, MeshType>::CloneAndSetMesh(const MeshType* new_mesh) const {
  DRAKE_DEMAND(new_mesh != nullptr);
  DRAKE_DEMAND(new_mesh->num_vertices() == mesh_->num_vertices());
  // TODO(DamrongGuoy): Check that the `new_mesh` is equivalent to the
  //  current `mesh_M_`.
  std::unique_ptr<MeshFieldLinear> new_mesh_field = CloneWithNullMesh();
  new_mesh_field->mesh_ = new_mesh;
  return new_mesh_field;
}

template <class T, class MeshType>
bool MeshFieldLinear<T, MeshType>::Equal(
    const MeshFieldLinear<T, MeshType>& field) const {
  if (!this->mesh().Equal(field.mesh())) return false;

  // Check field value at each vertex.
  for (typename MeshType::VertexIndex i(0); i < this->mesh().num_vertices();
       ++i) {
    if (values_.at(i) != field.values_.at(i)) return false;
  }
  if (gradients_ != field.gradients_) return false;
  if (values_at_Mo_ != field.values_at_Mo_) return false;
  // All checks passed.
  return true;
}

template <class T, class MeshType>
void MeshFieldLinear<T, MeshType>::CalcGradientField() {
  gradients_.clear();
  gradients_.reserve(this->mesh().num_elements());
  for (typename MeshType::ElementIndex e(0); e < this->mesh().num_elements();
       ++e) {
    gradients_.push_back(CalcGradientVector(e));
  }
}

template <class T, class MeshType>
Vector3<T> MeshFieldLinear<T, MeshType>::CalcGradientVector(
    typename MeshType::ElementIndex e) const {
  std::array<T, MeshType::kVertexPerElement> u;
  for (int i = 0; i < MeshType::kVertexPerElement; ++i) {
    u[i] = values_[this->mesh().element(e).vertex(i)];
  }
  return this->mesh().CalcGradientVectorOfLinearField(u, e);
}

template <class T, class MeshType>
void MeshFieldLinear<T, MeshType>::CalcValueAtMeshOriginForAllElements() {
  values_at_Mo_.clear();
  values_at_Mo_.reserve(this->mesh().num_elements());
  for (typename MeshType::ElementIndex e(0); e < this->mesh().num_elements();
       ++e) {
    values_at_Mo_.push_back(CalcValueAtMeshOrigin(e));
  }
}

template <class T, class MeshType>
T MeshFieldLinear<T, MeshType>::CalcValueAtMeshOrigin(
    typename MeshType::ElementIndex e) const {
  DRAKE_DEMAND(e < gradients_.size());
  const typename MeshType::VertexIndex v0 = this->mesh().element(e).vertex(0);
  const Vector3<T>& p_MV0 = this->mesh().vertex(v0).r_MV();
  // f(V₀) = ∇fᵉ⋅p_MV₀ + fᵉ(Mo)
  // fᵉ(Mo) = f(V₀) - ∇fᵉ⋅p_MV₀
  return values_[v0] - gradients_[e].dot(p_MV0);
}

template class MeshFieldLinear<double, SurfaceMesh<double>>;
template class MeshFieldLinear<AutoDiffXd, SurfaceMesh<AutoDiffXd>>;
template class MeshFieldLinear<double, VolumeMesh<double>>;
template class MeshFieldLinear<AutoDiffXd, VolumeMesh<AutoDiffXd>>;

}  // namespace geometry
}  // namespace drake

