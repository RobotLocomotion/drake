#pragma once

#include <array>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/quadrature.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* Returns 4^n. */
constexpr int pow4(int n) {
  return (n <= 0) ? 1 : 4 * pow4(n - 1);
}

/* A derived class implementing centroid-based subdivision quadrature for a
 tetrahedron in its standard reference domain. The standard tetrahedron is
 defined by vertices:

    v0 = (0,0,0),
    v1 = (1,0,0),
    v2 = (0,1,0),
    v3 = (0,0,1),

 which has volume = 1/6.

 We subdivide N times by recursively connecting the centroid of each tetrahedron
 to its 4 vertices, yielding 4^N sub-tetrahedra total. The centroid of each
 sub-tetrahedron is used as a quadrature point, each with weight = (1/6) / 4^N.

 This approach is *not* a high-order accurate rule. but is convenient for
 sampling more quadrature points in coarse tetrahedral elements when that's
 desired.
 @tparam subdivisions The number of subdivisions N. */
template <int subdivisions>
class TetSubdivisionQuadrature final
    : public Quadrature</* natural dimension */ 3,
                        /* num quadrature points */ pow4(subdivisions)> {
 public:
  static constexpr int natural_dimension = 3;
  static constexpr int num_quadrature_points = pow4(subdivisions);
  using Base = Quadrature<natural_dimension, num_quadrature_points>;
  using LocationsType =
      typename Base::LocationsType;  // array<Vector3<double>, 4^N>
  using WeightsType = typename Base::WeightsType;  // array<double, 4^N>
  using VectorD = typename Base::VectorD;          // Vector3<double>

  /* Constructor builds the data (points + weights) and then forwards it to the
   base class constructor. */
  TetSubdivisionQuadrature() : Base(MakePointsAndWeights()) {}

 private:
  /* A small struct to hold 4 vertices of a tetrahedron in R^3 for the
   subdivision recursion. */
  struct Tetrahedron {
    Vector3<double> v[4];
  };

  /* Returns (points[], weights[]) in a single std::pair, which the base class
   constructor expects.  The number of points is 4^N.  We store them in
   a std::array<Vector3<double>, 4^N> and std::array<double, 4^N>. */
  static std::pair<LocationsType, WeightsType> MakePointsAndWeights() {
    /* Recursively find centroids of sub-tets in standard reference domain.
     We'll accumulate them in a std::vector first, then copy into std::array. */

    /* The vertices of the *reference* tetrahedron. */
    const Tetrahedron root{{Vector3<double>(0, 0, 0), Vector3<double>(1, 0, 0),
                            Vector3<double>(0, 1, 0),
                            Vector3<double>(0, 0, 1)}};

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

  /* Recursively subdivides a tetrahedron `tet` `level` times, collecting the
   centroid of each final sub-tet in the std::vector `result`. */
  static void SubdivideAndCollectCentroids(
      const Tetrahedron& tet, int level, std::vector<Vector3<double>>* result) {
    if (level <= 0) {
      /* No further subdivision. Store the centroid of `tet`. */
      result->push_back(ComputeCentroid(tet));
      return;
    }

    /* Otherwise, subdivide this tetrahedron into 4 smaller ones by connecting
     he centroid G to each face: */
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

  /* Computes the centroid of a tetrahedron with corners v0,v1,v2,v3. */
  static Vector3<double> ComputeCentroid(const Tetrahedron& tet) {
    return (tet.v[0] + tet.v[1] + tet.v[2] + tet.v[3]) / 4.0;
  }
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
