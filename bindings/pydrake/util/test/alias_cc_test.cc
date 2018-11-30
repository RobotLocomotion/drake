// Ensure we can include files from their old path.
#include "drake/bindings/pydrake/util/cpp_param_pybind.h"
#include "drake/bindings/pydrake/util/cpp_template_pybind.h"
#include "drake/bindings/pydrake/util/deprecation_pybind.h"
#include "drake/bindings/pydrake/util/drake_optional_pybind.h"
#include "drake/bindings/pydrake/util/eigen_geometry_pybind.h"
#include "drake/bindings/pydrake/util/eigen_pybind.h"
#include "drake/bindings/pydrake/util/type_pack.h"
#include "drake/bindings/pydrake/util/type_safe_index_pybind.h"
#include "drake/bindings/pydrake/util/wrap_function.h"
#include "drake/bindings/pydrake/util/wrap_pybind.h"

// Brief symbol check.
using drake::pydrake::GetPyParam;

// No-op; compilation test only.
int main() {
  return 0;
}
