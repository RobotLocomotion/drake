#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/solvers/solvers_py.h"
#include "drake/bindings/pydrake/symbolic_types_pybind.h"
#include "drake/solvers/bilinear_product_util.h"

namespace drake {
namespace pydrake {
namespace internal {
void DefineSolversBilinearProductUtil(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::solvers;
  constexpr auto& doc = pydrake_doc.drake.solvers;

  m.def("ReplaceBilinearTerms", &ReplaceBilinearTerms, py::arg("e"),
      py::arg("x"), py::arg("y"), py::arg("W"), doc.ReplaceBilinearTerms.doc);
}
}  // namespace internal
}  // namespace pydrake
}  // namespace drake
