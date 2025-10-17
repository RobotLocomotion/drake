#include "drake/multibody/fem/tet_subdivision_quadrature.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

template <int subdivisions>
std::pair<typename TetSubdivisionQuadrature<subdivisions>::LocationsType,
          typename TetSubdivisionQuadrature<subdivisions>::WeightsType>
TetSubdivisionQuadrature<subdivisions>::MakePointsAndWeights() {
  /* Recursively find centroids of sub-tets in standard reference domain.
   We'll accumulate them in a std::vector first, then copy into std::array. */

  /* The vertices of the *reference* tetrahedron. */
  const Tetrahedron root{{Vector3<double>(0, 0, 0), Vector3<double>(1, 0, 0),
                          Vector3<double>(0, 1, 0), Vector3<double>(0, 0, 1)}};

  std::vector<Vector3<double>> centroids;
  centroids.reserve(num_quadrature_points);
  SubdivideAndCollectCentroids(root, subdivisions, &centroids);

  /* The final arrays for Quadrature base class constructor. */
  LocationsType points_array;
  WeightsType weights_array;

  /* Each sub-tet has volume = (1/6) / (4^N). Each centroid is assigned that
   weight for integration in the reference tetrahedron. */
  const double w = (1.0 / 6.0) / static_cast<double>(num_quadrature_points);

  for (int i = 0; i < num_quadrature_points; ++i) {
    points_array[i] = centroids[i];
    weights_array[i] = w;
  }

  return std::make_pair(std::move(points_array), std::move(weights_array));
}

template <int subdivisions>
void TetSubdivisionQuadrature<subdivisions>::SubdivideAndCollectCentroids(
    const Tetrahedron& tet, int level, std::vector<Vector3<double>>* result) {
  if (level <= 0) {
    /* No further subdivision. Store the centroid of `tet`. */
    result->push_back(ComputeCentroid(tet));
    return;
  }

  /* Otherwise, subdivide this tetrahedron into 4 smaller ones by connecting
   the centroid G to each face: */
  const Vector3<double> G = ComputeCentroid(tet);

  const Tetrahedron t0{{G, tet.v[0], tet.v[1], tet.v[2]}};
  const Tetrahedron t1{{G, tet.v[0], tet.v[1], tet.v[3]}};
  const Tetrahedron t2{{G, tet.v[0], tet.v[2], tet.v[3]}};
  const Tetrahedron t3{{G, tet.v[1], tet.v[2], tet.v[3]}};

  /* Recurse. */
  SubdivideAndCollectCentroids(t0, level - 1, result);
  SubdivideAndCollectCentroids(t1, level - 1, result);
  SubdivideAndCollectCentroids(t2, level - 1, result);
  SubdivideAndCollectCentroids(t3, level - 1, result);
}

template <int subdivisions>
Vector3<double> TetSubdivisionQuadrature<subdivisions>::ComputeCentroid(
    const Tetrahedron& tet) {
  return (tet.v[0] + tet.v[1] + tet.v[2] + tet.v[3]) / 4.0;
}

template class TetSubdivisionQuadrature<0>;
template class TetSubdivisionQuadrature<1>;
template class TetSubdivisionQuadrature<2>;
template class TetSubdivisionQuadrature<3>;
template class TetSubdivisionQuadrature<4>;

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
