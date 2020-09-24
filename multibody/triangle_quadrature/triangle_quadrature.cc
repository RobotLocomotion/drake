#include "drake/multibody/triangle_quadrature/triangle_quadrature.h"

#include "drake/multibody/math/spatial_algebra.h"

// Default template instantiations below.
namespace drake {
namespace multibody {

// First template argument is return type, second is scalar type.
template class TriangleQuadrature<double, double>;
template class TriangleQuadrature<VectorX<double>, double>;
template class TriangleQuadrature<SpatialForce<double>, double>;

}  // namespace multibody
}  // namespace drake
