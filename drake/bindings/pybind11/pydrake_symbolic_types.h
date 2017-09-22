#pragma once

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "drake/common/symbolic.h"

PYBIND11_NUMPY_OBJECT_DTYPE(drake::symbolic::Variable);
PYBIND11_NUMPY_OBJECT_DTYPE(drake::symbolic::Expression);
PYBIND11_NUMPY_OBJECT_DTYPE(drake::symbolic::Formula);
