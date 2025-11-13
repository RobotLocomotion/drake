#include "drake/systems/framework/framework_common.h"

namespace drake {
namespace systems {
#ifndef DRAKE_DOXYGEN_CXX
// This file defines the first virtual funciton of the class
// to emit the vtable only in this translation unit.
namespace internal {

SystemMessageInterface::~SystemMessageInterface() = default;

ContextMessageInterface::~ContextMessageInterface() = default;

SystemParentServiceInterface::~SystemParentServiceInterface() = default;

}  // namespace internal
#endif
}  // namespace systems
}  // namespace drake
