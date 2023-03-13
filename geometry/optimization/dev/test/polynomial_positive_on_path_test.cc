#include "drake/geometry/optimization/dev/polynomial_positive_on_path.h"

#include <optional>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/symbolic_test_util.h"
#include "drake/geometry/optimization/dev/test/c_iris_path_test_utilities.h"
#include "drake/geometry/optimization/test/c_iris_test_utilities.h"
#include "drake/solvers/solve.h"

