#include "drake/geometry/proximity/mesh_field_linear.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace geometry {

template <class FieldValue, class MeshType>
void MeshFieldLinear<FieldValue, MeshType>::CalcGradientField() {
  gradients_.clear();
  gradients_.reserve(this->mesh().num_elements());
  for (typename MeshType::ElementIndex e(0); e < this->mesh().num_elements();
       ++e) {
    gradients_.push_back(CalcGradientVector(e));
  }
}

template <class FieldValue, class MeshType>
Vector3<FieldValue> MeshFieldLinear<FieldValue, MeshType>::CalcGradientVector(
    typename MeshType::ElementIndex e) const {
  std::array<FieldValue, MeshType::kVertexPerElement> u;
  for (int i = 0; i < MeshType::kVertexPerElement; ++i) {
    u[i] = values_[this->mesh().element(e).vertex(i)];
  }
  return this->mesh().CalcGradientVectorOfLinearField(u, e);
}

template class MeshFieldLinear<double, SurfaceMesh<double>>;
template class MeshFieldLinear<AutoDiffXd, SurfaceMesh<AutoDiffXd>>;
template class MeshFieldLinear<double, VolumeMesh<double>>;
template class MeshFieldLinear<AutoDiffXd, VolumeMesh<AutoDiffXd>>;

}  // namespace geometry
}  // namespace drake

