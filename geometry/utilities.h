#pragma once

#include <string>

namespace drake {
namespace geometry {
namespace detail {

/** Canonicalizes the given geometry *candidate* name. A canonicalized name may
 still not be valid (as it may duplicate a previously used name). See
 @ref canonicalized_geometry_names "documentation in GeometryInstance" for
 details. */
std::string CanonicalizeStringName(const std::string& name);

}  // namespace detail
}  // namespace geometry
}  // namespace drake
