#pragma once

/// @file
/// When using Drake's symbolic expressions library (e.g., the classes
/// drake::symbolic::Expression or drake::symbolic::Formula), we provide
/// a single include statement to cover all of the required classes:
/// `#include <drake/common/symbolic/expression.h>`.

// Delegate the internal implementation details to our subdirectory.
#include "drake/common/symbolic/expression/all.h"
