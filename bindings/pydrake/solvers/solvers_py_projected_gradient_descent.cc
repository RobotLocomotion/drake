#include "drake/bindings/generated_docstrings/solvers.h"
#include "drake/bindings/pydrake/common/ensure_callback_lifetime_pybind.h"
#include "drake/bindings/pydrake/common/serialize_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/solvers/solvers_py.h"
#include "drake/solvers/projected_gradient_descent_solver.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefineSolversProjectedGradientDescent(py::module_ m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::solvers;
  constexpr auto& doc = pydrake_doc_solvers.drake.solvers;

  {
    using Class = ProjectedGradientDescentSolver;
    constexpr auto& cls_doc = doc.ProjectedGradientDescentSolver;
    class_<Class, SolverInterface> pgd_cls(
        m, "ProjectedGradientDescentSolver", cls_doc.doc, py::dynamic_attr());

    pgd_cls  // BR
        .def(py::init<>(), cls_doc.ctor.doc)
        .def(
            "SetCustomGradientFunction",
            [](Class& self, py::callable custom_gradient_function) {
              // The solver borrows the callable; ensure_callback_lifetime
              // keeps it alive in a way that Python's GC can see.
              PyObject* ptr = custom_gradient_function.ptr();
              self.SetCustomGradientFunction(
                  [ptr](const Eigen::VectorXd& in) -> Eigen::VectorXd {
                    py::gil_scoped_acquire guard;
                    py::handle callback = ptr;
                    return py::cast<Eigen::VectorXd>(callback(in));
                  });
            },
            py::arg("custom_gradient_function"),
            internal::ensure_callback_lifetime<1, 2>(),
            cls_doc.SetCustomGradientFunction.doc)
        .def("SetProjectionSolverInterface",
            &Class::SetProjectionSolverInterface,
            py::arg("projection_solver_interface").none(),
            cls_doc.SetProjectionSolverInterface.doc)
        .def_static("ConvergenceTolOptionName",
            &Class::ConvergenceTolOptionName,
            cls_doc.ConvergenceTolOptionName.doc)
        .def_static("MaxIterationsOptionName", &Class::MaxIterationsOptionName,
            cls_doc.MaxIterationsOptionName.doc)
        .def_static("BacktrackingCOptionName", &Class::BacktrackingCOptionName,
            cls_doc.BacktrackingCOptionName.doc)
        .def_static("BacktrackingTauOptionName",
            &Class::BacktrackingTauOptionName,
            cls_doc.BacktrackingTauOptionName.doc)
        .def_static("BacktrackingAlpha0OptionName",
            &Class::BacktrackingAlpha0OptionName,
            cls_doc.BacktrackingAlpha0OptionName.doc)
        .def_static("id", &Class::id, cls_doc.id.doc);
    pgd_cls.attr("kDefaultConvergenceTol") = &Class::kDefaultConvergenceTol;
    pgd_cls.attr("kDefaultMaxIterations") = &Class::kDefaultMaxIterations;
    pgd_cls.attr("kDefaultBacktrackingC") = &Class::kDefaultBacktrackingC;
    pgd_cls.attr("kDefaultBacktrackingTau") = &Class::kDefaultBacktrackingTau;
    pgd_cls.attr("kDefaultBacktrackingAlpha0") =
        &Class::kDefaultBacktrackingAlpha0;
    pgd_cls.attr("kDefaultMaxLineSearchSteps") =
        &Class::kDefaultMaxLineSearchSteps;

    pgd_cls.def(
        "SetCustomProjectionFunction",
        [](Class& self, py::callable python_projection_function) {
          // As above, the solver borrows the callable.
          PyObject* ptr = python_projection_function.ptr();
          self.SetCustomProjectionFunction(
              [ptr](const Eigen::VectorXd& in, Eigen::VectorXd* out) {
                DRAKE_DEMAND(out != nullptr);
                py::gil_scoped_acquire guard;
                py::handle callback = ptr;
                bool result;
                std::tie(result, *out) =
                    py::cast<std::pair<bool, Eigen::VectorXd>>(callback(in));
                return result;
              });
        },
        py::arg("custom_projection_function"),
        internal::ensure_callback_lifetime<1, 2>(),
        R"""(
Specify a custom projection function. Otherwise, this solver will
attempt to solve the L2 projection onto the feasible set of the
MathematicalProgram it's used to solve. The projection function
should return a tuple, whose first entry is a boolean value
indicating success or failure, and whose second value is the result
of the projection. It should take in as an argument the point we are
trying to stay close to.

Example
-------
You can define a custom projection function in Python, such as projecting onto the nonnegative orthant:

.. code-block:: python

    def nonnegative_projection(x):
        # Project onto the set {x | x >= 0}
        return True, np.maximum(x, 0)
)""");
  }
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
