#include "drake/multibody/hydroelastic_contact/triangle_quadrature.h"

// Default template instantiations below.
namespace drake {
namespace multibody {
namespace hydroelastic_contact {

// First template argument is return type, second is scalar type.
template class TriangleQuadrature<double, double>;
template class TriangleQuadrature<VectorX<double>, double>;
template class TriangleQuadrature<SpatialForce<double>, double>;

}  // namespace hydroelastic_contact
}  // namespace multibody
}  // namespace drake
