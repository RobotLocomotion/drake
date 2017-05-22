#pragma once

#include "drake/common/drake_assert.h"

namespace drake {
namespace systems {

constexpr int kAutoSize = -1;

/// All system ports are either vectors of Eigen scalars, or black-box
/// AbstractValues which may contain any type.
typedef enum {
  kVectorValued = 0,
  kAbstractValued = 1,
} PortDataType;

}  // namespace systems
}  // namespace drake
