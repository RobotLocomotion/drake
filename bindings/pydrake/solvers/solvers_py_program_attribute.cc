#include "drake/bindings/generated_docstrings/solvers.h"
#include "drake/bindings/pydrake/common/eigen_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/solvers/solvers_py.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace pydrake {
namespace internal {
void DefineProgramAttribute(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::solvers;
  constexpr auto& doc = pydrake_doc_solvers.drake.solvers;
  py::enum_<ProgramAttribute>(m, "ProgramAttribute", doc.ProgramAttribute.doc)
      .value("kGenericCost", ProgramAttribute::kGenericCost,
          doc.ProgramAttribute.kGenericCost.doc)
      .value("kGenericConstraint", ProgramAttribute::kGenericConstraint,
          doc.ProgramAttribute.kGenericConstraint.doc)
      .value("kQuadraticCost", ProgramAttribute::kQuadraticCost,
          doc.ProgramAttribute.kQuadraticCost.doc)
      .value("kQuadraticConstraint", ProgramAttribute::kQuadraticConstraint,
          doc.ProgramAttribute.kQuadraticConstraint.doc)
      .value("kLinearCost", ProgramAttribute::kLinearCost,
          doc.ProgramAttribute.kLinearCost.doc)
      .value("kLinearConstraint", ProgramAttribute::kLinearConstraint,
          doc.ProgramAttribute.kLinearConstraint.doc)
      .value("kLinearEqualityConstraint",
          ProgramAttribute::kLinearEqualityConstraint,
          doc.ProgramAttribute.kLinearEqualityConstraint.doc)
      .value("kLinearComplementarityConstraint",
          ProgramAttribute::kLinearComplementarityConstraint,
          doc.ProgramAttribute.kLinearComplementarityConstraint.doc)
      .value("kLorentzConeConstraint", ProgramAttribute::kLorentzConeConstraint,
          doc.ProgramAttribute.kLorentzConeConstraint.doc)
      .value("kRotatedLorentzConeConstraint",
          ProgramAttribute::kRotatedLorentzConeConstraint,
          doc.ProgramAttribute.kRotatedLorentzConeConstraint.doc)
      .value("kPositiveSemidefiniteConstraint",
          ProgramAttribute::kPositiveSemidefiniteConstraint,
          doc.ProgramAttribute.kPositiveSemidefiniteConstraint.doc)
      .value("kExponentialConeConstraint",
          ProgramAttribute::kExponentialConeConstraint,
          doc.ProgramAttribute.kExponentialConeConstraint.doc)
      .value("kL2NormCost", ProgramAttribute::kL2NormCost,
          doc.ProgramAttribute.kL2NormCost.doc)
      .value("kBinaryVariable", ProgramAttribute::kBinaryVariable,
          doc.ProgramAttribute.kBinaryVariable.doc)
      .value("kCallback", ProgramAttribute::kCallback,
          doc.ProgramAttribute.kCallback.doc);

  py::enum_<ProgramType>(m, "ProgramType", doc.ProgramType.doc)
      .value("kLP", ProgramType::kLP, doc.ProgramType.kLP.doc)
      .value("kQP", ProgramType::kQP, doc.ProgramType.kQP.doc)
      .value("kSOCP", ProgramType::kSOCP, doc.ProgramType.kSOCP.doc)
      .value("kSDP", ProgramType::kSDP, doc.ProgramType.kSDP.doc)
      .value("kGP", ProgramType::kGP, doc.ProgramType.kGP.doc)
      .value("kCGP", ProgramType::kCGP, doc.ProgramType.kCGP.doc)
      .value("kMILP", ProgramType::kMILP, doc.ProgramType.kMILP.doc)
      .value("kMIQP", ProgramType::kMIQP, doc.ProgramType.kMIQP.doc)
      .value("kMISOCP", ProgramType::kMISOCP, doc.ProgramType.kMISOCP.doc)
      .value("kMISDP", ProgramType::kMISDP, doc.ProgramType.kMISDP.doc)
      .value("kQuadraticCostConicConstraint",
          ProgramType::kQuadraticCostConicConstraint,
          doc.ProgramType.kQuadraticCostConicConstraint.doc)
      .value("kNLP", ProgramType::kNLP, doc.ProgramType.kNLP.doc)
      .value("kLCP", ProgramType::kLCP, doc.ProgramType.kLCP.doc)
      .value("kUnknown", ProgramType::kUnknown, doc.ProgramType.kUnknown.doc);
}
}  // namespace internal
}  // namespace pydrake
}  // namespace drake
