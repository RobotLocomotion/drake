#ifndef DRAKE_DRAKECORE_H
#define DRAKE_DRAKECORE_H

#include "drake/Path.h"
#include "Vector.h"
#include "Function.h"
#include "Gradient.h"

#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

#define PRINT_FUNCTION_NAME std::cout << __PRETTY_FUNCTION__ << std::endl;

#endif  // DRAKE_DRAKECORE_H
