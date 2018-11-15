#pragma once

#include <string>
#include <unordered_map>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace geometry {

// TODO(SeanCurtis-TRI): Get rid of the "detail" namespace.
namespace detail {

/** Canonicalizes the given geometry *candidate* name. A canonicalized name may
 still not be valid (as it may duplicate a previously used name). See
 @ref canonicalized_geometry_names "documentation in GeometryInstance" for
 details. */
std::string CanonicalizeStringName(const std::string& name);

}  // namespace detail

namespace internal {

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

}  // namespace internal

}  // namespace geometry
}  // namespace drake
