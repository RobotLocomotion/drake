#include "drake/bindings/pydrake/common/parsing_py.h"

#include "pybind11/eval.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(parsing, m) {
  PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(m);
  m.doc() = "Common parsing utilities.";

  /* The order of execution matters -- a module may rely on the definition
   of bindings executed prior to it. */
  DefineParsingPackageMap(m);
}
}  // namespace pydrake
}  // namespace drake
