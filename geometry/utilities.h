#pragma once

#include <iterator>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "drake/common/autodiff.h"
#include "drake/common/autodiff_overloads.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

/* Canonicalizes the given geometry *candidate* name. A canonicalized name may
 still not be valid (as it may duplicate a previously used name). See
 @ref canonicalized_geometry_names "documentation in GeometryInstance" for
 details. */
std::string CanonicalizeStringName(const std::string& name);

// A const range iterator through the keys of an unordered map.
template <typename K, typename V>
class MapKeyRange {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MapKeyRange)

  class ConstIterator {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ConstIterator)

    /* In order to be able to instantiate an std::vector from a MapKeyRange,
     e.g.,

        std::vector<K> values(range.begin(), range.end());

     we need to define the following member types to enable
     std::iterator_traits. Curiously, it isn't required for doing the same for
     e.g., std::set. */
    using difference_type = int;
    using value_type = K;
    using pointer = const K*;
    using reference = const K&;
    using iterator_category = std::input_iterator_tag;

    const K& operator*() const { return itr_->first; }
    const ConstIterator& operator++() {
      ++itr_;
      return *this;
    }
    bool operator!=(const ConstIterator& other) { return itr_ != other.itr_; }

   private:
    explicit ConstIterator(
        typename std::unordered_map<K, V>::const_iterator itr)
        : itr_(itr) {}

   private:
    typename std::unordered_map<K, V>::const_iterator itr_;
    friend class MapKeyRange;
  };

  explicit MapKeyRange(const std::unordered_map<K, V>* map) : map_(map) {
    DRAKE_DEMAND(map != nullptr);
  }
  ConstIterator begin() const { return ConstIterator(map_->cbegin()); }
  ConstIterator end() const { return ConstIterator(map_->cend()); }

 private:
  const std::unordered_map<K, V>* map_;
};

/* @name Isometry scalar conversion

 Some of SceneGraph's inner-workings are _not_ templated on scalar type and
 always require double values. These functions work in an ADL-compatible
 manner to allow SceneGraph to mindlessly convert Quantity<T> to
 Quantity<double> efficiently. There is *particular* emphasis on making the
 Quantity<double> -> Quantity<double> as cheap as possible.  */
//@{

inline const Vector3<double>& convert_to_double(const Vector3<double>& vec) {
  return vec;
}

template <class VectorType>
Vector3<double> convert_to_double(
    const Vector3<Eigen::AutoDiffScalar<VectorType>>& vec) {
  Vector3<double> result;
  for (int r = 0; r < 3; ++r) {
    result(r) = ExtractDoubleOrThrow(vec(r));
  }
  return result;
}

// Don't needlessly copy transforms that are already scalar-valued.
inline const math::RigidTransformd& convert_to_double(
    const math::RigidTransformd& X_AB) {
  return X_AB;
}

template <class VectorType>
math::RigidTransformd convert_to_double(
    const math::RigidTransform<Eigen::AutoDiffScalar<VectorType>>& X_AB) {
  Matrix3<double> R_converted;
  Vector3<double> p_converted;
  for (int r = 0; r < 3; ++r) {
    p_converted(r) = ExtractDoubleOrThrow(X_AB.translation()(r));
    for (int c = 0; c < 3; ++c) {
      R_converted(r, c) = ExtractDoubleOrThrow(X_AB.rotation().matrix()(r, c));
    }
  }
  return math::RigidTransformd(math::RotationMatrixd(R_converted), p_converted);
}

//@}
//
/** Reads the OBJ file with the given `filename` into a collection of data
 * suitable for instantiating an fcl::Convex shape. It includes the vertex
 * positions, face encodings (see TinyObjToFclFaces), and number of faces.
 * @param filename The name of the obj file.
 * @param scale Scale to coordinates.
 * @param triangulate Whether triangulate polygon face in .obj or not. Refer to
 * LoadObj function in
 * https://github.com/tinyobjloader/tinyobjloader/blob/master/tiny_obj_loader.h
 * for more details.
 * @return (vertices, faces, num_faces) vertices[i] is the i'th vertex in the
 * mesh. faces is interpreted as
 *
 * faces = { n0, v0_0,v0_1,...,v0_n0-1,
 *           n1, v1_0,v1_1,...,v1_n1-1,
 *           n2, v2_0,v2_1,...,v2_n2-1,
 *           ...}
 *
 * where n_i is the number of vertices of face_i, vi_j is the index in
 * `vertices` for a vertex on face i. Note that the size of faces is larger than
 * num_faces.
 *
 */
std::tuple<std::shared_ptr<std::vector<Eigen::Vector3d>>,
           std::shared_ptr<std::vector<int>>, int>
ReadObjForConvex(const std::string& filename, double scale, bool triangulate);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
