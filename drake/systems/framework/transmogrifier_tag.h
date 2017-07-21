#pragma once

#include "drake/common/drake_compat.h"

namespace drake {
namespace systems {

/// A tag type (dummy type) that stands in as the first constructor argument
/// for the "transmogrification copy constructor".  For example:
///
/// @code
/// template <typename T>
/// class MySystem : public drake::systems::LeafSystem<T> {
///  public:
///   /// User constructor.
///   MySystem(... stuff ...) {
///     this->template SetConcreteSubclass<MySystem>();
///   }
///
///   /// Transmogrification constructor.
///   MySystem(const drake::systems::TransmogrifierTag&,
///            const MySystem<double>& other)
///       : MySystem(other.get_stuff()) {}
/// };
/// @endcode
///
/// The presence of this type as the first argument to a constructor marks the
/// "transmogrification copy constructor" -- a constructor where a System with
/// a different scalar typed is passed in as an const reference argument and
/// the constructor should initialize its own System to match `other` by
/// delegating to a conventional constructor.
struct TransmogrifierTag {};

}  // namespace systems
}  // namespace drake
