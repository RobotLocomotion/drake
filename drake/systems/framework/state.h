#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/systems/framework/vector_interface.h"

namespace drake {
namespace systems {

/// The State is a container for all the data comprising the complete state of
/// a particular System at a particular moment. Any field in the State may be
/// empty if it is not applicable to the System in question. A System may not
/// maintain state in any place other than the State object.
///
/// @tparam T A mathematical type compatible with Eigen's Scalar.
template <typename T>
struct State {
  // TODO(david-german-tri): Add state.
};

}  // namespace systems
}  // namesapce drake
