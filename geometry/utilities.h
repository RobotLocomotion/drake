#pragma once

#include <string>
#include <unordered_map>

#include "drake/common/autodiff.h"
#include "drake/common/autodiff_overloads.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace geometry {
namespace internal {

/** Canonicalizes the given geometry *candidate* name. A canonicalized name may
 still not be valid (as it may duplicate a previously used name). See
 @ref canonicalized_geometry_names "documentation in GeometryInstance" for
 details. */
std::string CanonicalizeStringName(const std::string& name);

/// A const range iterator through the keys of an unordered map.
template <typename K, typename V>
class MapKeyRange {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MapKeyRange)

  class ConstIterator {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ConstIterator)

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
    DRAKE_DEMAND(map);
  }
  ConstIterator begin() const { return ConstIterator(map_->cbegin()); }
  ConstIterator end() const { return ConstIterator(map_->cend()); }

 private:
  const std::unordered_map<K, V>* map_;
};

/** @name Isometry scalar conversion

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

// TODO(SeanCurtis-TRI): Get rid of these when I finally swap for
// RigidTransforms.

inline const Isometry3<double>& convert_to_double(
    const Isometry3<double>& transform) {
  return transform;
}

template <class VectorType>
Isometry3<double> convert_to_double(
    const Isometry3<Eigen::AutoDiffScalar<VectorType>>& transform) {
  Isometry3<double> result;
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      result.matrix()(r, c) = ExtractDoubleOrThrow(transform.matrix()(r, c));
    }
  }
  return result;
}

//@}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
