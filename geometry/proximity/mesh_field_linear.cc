#include "drake/geometry/proximity/mesh_field_linear.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace geometry {

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

