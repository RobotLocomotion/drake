#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"

namespace drake {
namespace geometry {

/** The data for reporting the distance between two geometries, A and B.
 @tparam T The underlying scalar type. Must be a valid Eigen scalar. */
template <typename T>
struct NearestPair {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(NearestPair)

  NearestPair() {}

  /** Constructor
   @param a       The id of the first geometry (A).
   @param b       The id of the second geometry (B).
   @param p_A     The point on geometry A's surface nearest B, in A's frame.
   @param p_B     The point on geometry B's surface nearest A, in B's frame.
   @param dist    The distance between p_A and p_B. */
  NearestPair(GeometryId a, GeometryId b, const Vector3<T>& p_A,
              const Vector3<T>& p_B, T dist) : id_A(a), id_B(b),
                                               p_ACa(p_A), p_BCb(p_B),
                                               distance(dist) {}

  /** The id of the first geometry in the pair. */
  GeometryId id_A;
  /** The id of the second geometry in the pair. */
  GeometryId id_B;
  /** The point on geometry A's surface nearest B, in A's frame. */
  Vector3<T> p_ACa;
  /** The point on geometry B's surface nearest A, in B's frame. */
  Vector3<T> p_BCb;
  /** The distance between p_A_A and p_B_B (measured in a common frame). */
  T distance{};
};

}  // namespace geometry
}  // namespace drake
