#pragma once

#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/symbolic.h"

namespace drake {
namespace symbolic {

/// Visitor class for code generation.
class CodeGenVisitor {
 public:
  using IdToIndexMap =
      std::unordered_map<Variable::Id, std::vector<Variable>::size_type>;

  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CodeGenVisitor)

  /// Constructs an instance of this visitor class from @p id_to_idx_map.
  ///
  /// @note This class keeps a constant reference of @p id_to_idx_map. So @p
  /// id_to_idx_map should outlive this visitor object.
  explicit CodeGenVisitor(const IdToIndexMap& id_to_idx_map);

  /// Generates C expression for the expression @p e.
  std::string CodeGen(const Expression& e) const;

 private:
  std::string VisitVariable(const Expression& e) const;
  std::string VisitConstant(const Expression& e) const;
  std::string VisitAddition(const Expression& e) const;
  std::string VisitMultiplication(const Expression& e) const;
  // Helper method to handle unary cases.
  std::string VisitUnary(const std::string& f, const Expression& e) const;
  // Helper method to handle binary cases.
  std::string VisitBinary(const std::string& f, const Expression& e) const;
  std::string VisitPow(const Expression& e) const;
  std::string VisitDivision(const Expression& e) const;
  std::string VisitAbs(const Expression& e) const;
  std::string VisitLog(const Expression& e) const;
  std::string VisitExp(const Expression& e) const;
  std::string VisitSqrt(const Expression& e) const;
  std::string VisitSin(const Expression& e) const;
  std::string VisitCos(const Expression& e) const;
  std::string VisitTan(const Expression& e) const;
  std::string VisitAsin(const Expression& e) const;
  std::string VisitAcos(const Expression& e) const;
  std::string VisitAtan(const Expression& e) const;
  std::string VisitAtan2(const Expression& e) const;
  std::string VisitSinh(const Expression& e) const;
  std::string VisitCosh(const Expression& e) const;
  std::string VisitTanh(const Expression& e) const;
  std::string VisitMin(const Expression& e) const;
  std::string VisitMax(const Expression& e) const;
  std::string VisitCeil(const Expression& e) const;
  std::string VisitFloor(const Expression& e) const;
  std::string VisitIfThenElse(const Expression& e) const;
  std::string VisitUninterpretedFunction(const Expression& e) const;
  // Makes VisitExpression a friend of this class so that it can use private
  // methods.
  friend std::string VisitExpression<std::string>(const CodeGenVisitor*,
                                                  const Expression&);

  const IdToIndexMap& id_to_idx_map_;
};

/// For a given symbolic expression @p e, generates two C functions, @p
/// function_name and `function_name_in`. The generated `function_name` takes an
/// array of doubles for parameters and returns an evaluation
/// result. `function_name_in` returns the length of @p parameters.
///
/// @param[in] function_name Name of the generated C function.
/// @param[in] parameters    Vector of variables provide the ordering of
///                          symbolic variables.
/// @param[in] e             Symbolic expression to codegen.
///
/// For example, `Codegen("f", {x, y}, 1 + sin(x) + cos(y))` generates the
/// following string.
///
/// <pre>
/// double f(const double* p) {
///     return (1 + sin(p[0]) + cos(p[1]));
/// }
/// int f_in() {
///     return 2;  // size of `{x, y}`.
/// }
/// </pre>
///
/// Note that in this example `x` and `y` are mapped to `p[0]` and `p[1]`
/// respectively because we passed `{x, y}` to `Codegen`.
///
/// Note that generated code does not include any headers while it may use math
/// functions defined in `<math.h>` such as sin, cos, exp, and log. A user of
/// generated code is responsible to include `<math.h>` if needed to compile
/// generated code.
std::string CodeGen(const std::string& function_name,
                    const std::vector<Variable>& parameters,
                    const Expression& e);

/// For a given symbolic dense matrix @p M, generates four C functions,
/// `<function_name>`, `<function_name>_in`, `<function_name>_rows`, and
/// `<function_name>_cols`. The generated `<function_name>` takes two
/// parameters:
///
///  - const double* p : An array of doubles for input parameters.
///  - double* m : An array of doubles to store the evaluation result.
///
/// `<function_name>_in()`, `<function_name>_rows()`, and
/// `<function_name>_cols()` return the number of input parameters, the number
/// of rows in the matrix, and the number of columns in the matrix respectively.
///
/// Please consider the following example:
///
/// @code
/// Eigen::Matrix<symbolic::Expression, 2, 2, Eigen::ColMajor> M;
/// M(0, 0) = 1.0;
/// M(1, 0) = 3 + x + y;
/// M(0, 1) = 4 * y;
/// M(1, 1) = sin(x);
/// CodeGen("f", {x, y}, M);
/// @endcode
///
/// When executed, the last line of the above example generates the following:
///
/// @code
/// void f(const double* p, double* m) {
///     m[0] = 1.000000;
///     m[1] = (3 + p[0] + p[1]);
///     m[2] = (4 * p[1]);
///     m[3] = sin(p[0]);
/// }
/// int f_in() {
///     return 2;
/// }
/// int f_rows() {
///     return 2;
/// }
/// int f_cols() {
///     return 2;
/// }
/// @endcode
///
/// Note that in the example, the matrix `M` is stored in column-major order and
/// the `CodeGen` function respects the storage order in the generated code. If
/// `M` were stored in row-major order, `CodeGen` would return the following:
///
/// void f(const double* p, double* m) {
///     m[0] = 1.000000;
///     m[1] = (4 * p[1]);
///     m[2] = (3 + p[0] + p[1]);
///     m[3] = sin(p[0]);
/// }
///
/// Note that generated code does not include any headers while it may use math
/// functions defined in `<math.h>` such as sin, cos, exp, and log. A user of
/// generated code is responsible to include `<math.h>` if needed to compile
/// generated code.
template <typename Derived>
std::string CodeGen(const std::string& function_name,
                    const std::vector<Variable>& parameters,
                    const Eigen::PlainObjectBase<Derived>& M) {
  std::ostringstream oss;
  // Add header for the main function.
  oss << "void " << function_name << "(const double* p, double* m) {\n";
  // Codegen the matrix.
  // Build a map from Variable::Id to index (in parameters).
  CodeGenVisitor::IdToIndexMap id_to_idx_map;
  for (std::vector<Variable>::size_type i = 0; i < parameters.size(); ++i) {
    id_to_idx_map.emplace(parameters[i].get_id(), i);
  }
  const CodeGenVisitor visitor{id_to_idx_map};
  for (int i = 0; i < M.cols() * M.rows(); ++i) {
    oss << "    "
        << "m[" << i << "] = " << visitor.CodeGen(M.data()[i]) << ";\n";
  }
  // Add footer for the main function.
  oss << "}\n";
  /// Handle `function_name_in`.
  oss << "int " << function_name << "_in() {\n"
      << "    return " << parameters.size() << ";\n"
      << "}\n";
  /// Handle `function_name_rows`.
  oss << "int " << function_name << "_rows() {\n"
      << "    return " << M.rows() << ";\n"
      << "}\n";
  /// Handle `function_name_cols`.
  oss << "int " << function_name << "_cols() {\n"
      << "    return " << M.cols() << ";\n"
      << "}\n";
  return oss.str();
}

}  // namespace symbolic
}  // namespace drake
