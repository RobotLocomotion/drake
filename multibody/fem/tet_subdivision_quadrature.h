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

/* Returns 4ᴺ */
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
 to its 4 vertices, yielding 4ᴺ sub-tetrahedra total. The centroid of each
 sub-tetrahedron is used as a quadrature point, each with weight = (1/6) / 4ᴺ.

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
      typename Base::LocationsType;                // array<Vector3<double>, 4ᴺ>
  using WeightsType = typename Base::WeightsType;  // array<double, 4ᴺ>
  using VectorD = typename Base::VectorD;          // Vector3<double>

  static_assert(0 <= subdivisions && subdivisions <= 4,
                "Only 0, 1, 2, 3, and 4 subdivisions are supported.");

  /* Constructor builds the data (points + weights) and then forwards it to the
   base class constructor. */
  TetSubdivisionQuadrature() : Base(MakePointsAndWeights()) {}

 private:
  /* A small struct to hold 4 vertices of a tetrahedron in R³ for the
   subdivision recursion. */
  struct Tetrahedron {
    Vector3<double> v[4];
  };

  /* Returns (points[], weights[]) in a single std::pair, which the base class
   constructor expects.  The number of points is 4ᴺ. We store them in
   a std::array<Vector3<double>, 4ᴺ> and std::array<double, 4ᴺ>. */
  static std::pair<LocationsType, WeightsType> MakePointsAndWeights();

  /* Recursively subdivides a tetrahedron `tet` `level` times, collecting the
   centroid of each final sub-tet in the std::vector `result`. */
  static void SubdivideAndCollectCentroids(
      const Tetrahedron& tet, int level, std::vector<Vector3<double>>* result);

  /* Computes the centroid of a tetrahedron with corners v0,v1,v2,v3. */
  static Vector3<double> ComputeCentroid(const Tetrahedron& tet);
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
