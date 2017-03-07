// Adapted with permission from code by Evan Drumwright
// (https://github.com/edrumwri).

#include "drake/solvers/moby_lcp_solver.h"

#include <Eigen/LU>
#include <Eigen/SparseCore>
#include <Eigen/SparseLU>

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "drake/common/drake_assert.h"

// TODO(edrumwri): Move the templated code into this file.
