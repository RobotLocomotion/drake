#pragma once

/** @file
 Provides a set of functions to facilitate visualization operations based on
 geometry world state. */

namespace drake {
namespace geometry {

template <typename T> class GeometrySystem;

/** Dispatches an LCM load message based on the registered geometry. It should
 be invoked _after_ registration is complete, but before context allocation.
 @param system      The system whose geometry will be sent in an LCM message.
 @throws std::logic_error if the system has already had its context allocated.
 */
void DispatchLoadMessage(const GeometrySystem<double>& system);

}  // namespace geometry
}  // namespace drake
