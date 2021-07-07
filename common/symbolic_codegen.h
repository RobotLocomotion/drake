#pragma once

#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

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

  /// Constructs an instance of this visitor class using the vector of
  /// variables, @p parameters. This visitor will map a symbolic variable `var`
  /// into `p[n]` where `n` is the index of the variable `var` in the given @p
  /// parameters.
  explicit CodeGenVisitor(const std::vector<Variable>& parameters);

  /// Generates C expression for the expression @p e.
  [[nodiscard]] std::string CodeGen(const Expression& e) const;

 private:
  [[nodiscard]] std::string VisitVariable(const Expression& e) const;
  [[nodiscard]] std::string VisitConstant(const Expression& e) const;
  [[nodiscard]] std::string VisitAddition(const Expression& e) const;
  [[nodiscard]] std::string VisitMultiplication(const Expression& e) const;
  // Helper method to handle unary cases.
  [[nodiscard]] std::string VisitUnary(const std::string& f,
                                       const Expression& e) const;
  // Helper method to handle binary cases.
  [[nodiscard]] std::string VisitBinary(const std::string& f,
                                        const Expression& e) const;
  [[nodiscard]] std::string VisitPow(const Expression& e) const;
  [[nodiscard]] std::string VisitDivision(const Expression& e) const;
  [[nodiscard]] std::string VisitAbs(const Expression& e) const;
  [[nodiscard]] std::string VisitLog(const Expression& e) const;
  [[nodiscard]] std::string VisitExp(const Expression& e) const;
  [[nodiscard]] std::string VisitSqrt(const Expression& e) const;
  [[nodiscard]] std::string VisitSin(const Expression& e) const;
  [[nodiscard]] std::string VisitCos(const Expression& e) const;
  [[nodiscard]] std::string VisitTan(const Expression& e) const;
  [[nodiscard]] std::string VisitAsin(const Expression& e) const;
  [[nodiscard]] std::string VisitAcos(const Expression& e) const;
  [[nodiscard]] std::string VisitAtan(const Expression& e) const;
  [[nodiscard]] std::string VisitAtan2(const Expression& e) const;
  [[nodiscard]] std::string VisitSinh(const Expression& e) const;
  [[nodiscard]] std::string VisitCosh(const Expression& e) const;
  [[nodiscard]] std::string VisitTanh(const Expression& e) const;
  [[nodiscard]] std::string VisitMin(const Expression& e) const;
  [[nodiscard]] std::string VisitMax(const Expression& e) const;
  [[nodiscard]] std::string VisitCeil(const Expression& e) const;
  [[nodiscard]] std::string VisitFloor(const Expression& e) const;
  [[nodiscard]] std::string VisitIfThenElse(const Expression& e) const;
  [[nodiscard]] std::string VisitUninterpretedFunction(
      const Expression& e) const;
  // Makes VisitExpression a friend of this class so that it can use private
  // methods.
  friend std::string VisitExpression<std::string>(const CodeGenVisitor*,
                                                  const Expression&);

  IdToIndexMap id_to_idx_map_;
};

/// @defgroup codegen Code Generation
/// @ingroup technical_notes
/// @{
/// Provides `CodeGen` functions which generate C99 code to evaluate symbolic
/// expressions and matrices.
///
/// @note Generated code does not contain `#include` directives while it may use
/// math functions defined in `<math.h>` such as `sin`, `cos`, `exp`, and `log`.
/// A user of generated code is responsible to include `<math.h>` if needed to
/// compile generated code.

/// For a given symbolic expression @p e, generates two C functions,
/// `<function_name>` and `<function_name>_meta`. The generated
/// `<function_name>` function takes an array of doubles for parameters and
/// returns an evaluation result. `<function_name>_meta` returns a nested struct
/// from which a caller can obtain the following information:
///  - `.p.size`: the size of input parameters.
///
/// @param[in] function_name Name of the generated C function.
/// @param[in] parameters    Vector of variables provide the ordering of
///                          symbolic variables.
/// @param[in] e             Symbolic expression to codegen.
///
/// For example, `Codegen("f", {x, y}, 1 + sin(x) + cos(y))` generates the
/// following string.
///
/// @code
/// double f(const double* p) {
///     return (1 + sin(p[0]) + cos(p[1]));
/// }
/// typedef struct {
///     /* p: input, vector */
///     struct { int size; } p;
/// } f_meta_t;
/// f_meta_t f_meta() { return {{2}}; }
/// @endcode
///
/// Note that in this example `x` and `y` are mapped to `p[0]` and `p[1]`
/// respectively because we passed `{x, y}` to `Codegen`.
std::string CodeGen(const std::string& function_name,
                    const std::vector<Variable>& parameters,
                    const Expression& e);

namespace internal {
// Generates code for the internal representation of a matrix, @p data, using
// @p function_name, @p parameters, and @p size. It outputs the generated code
// to the output stream @p os.
//
// @note This function is used to implement std::string
// drake::symbolic::CodeGen(const std::string &, const std::vector<Variable>&,
// const Eigen::PlainObjectBase<Derived>&).
void CodeGenDenseData(const std::string& function_name,
                      const std::vector<Variable>& parameters,
                      const Expression* data, int size, std::ostream* os);

// Generates code for the meta information and outputs to the output stream @p
// os.
//
// @note This function is used to implement std::string
// drake::symbolic::CodeGen(const std::string &, const std::vector<Variable>&,
// const Eigen::PlainObjectBase<Derived>&).
void CodeGenDenseMeta(const std::string& function_name, int parameter_size,
                      int rows, int cols, std::ostream* os);
}  // namespace internal

/// For a given symbolic dense matrix @p M, generates two C functions,
/// `<function_name>` and `<function_name>_meta`. The generated
/// `<function_name>` takes two parameters:
///
///  - const double* p : An array of doubles for input parameters.
///  - double* m : An array of doubles to store the evaluation result.
///
/// `<function_name>_meta()` returns a nested struct from which a caller can
/// obtain the following information:
///  - `.p.size`: the size of input parameters.
///  - `.m.rows`: the number of rows in the matrix.
///  - `.m.cols`: the number of columns in the matrix.
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
/// When executed, the last line of the above example generates the following
/// code:
///
/// @code
/// void f(const double* p, double* m) {
///   m[0] = 1.000000;
///   m[1] = (3 + p[0] + p[1]);
///   m[2] = (4 * p[1]);
///   m[3] = sin(p[0]);
/// }
/// typedef struct {
///   /* p: input, vector */
///   struct {
///     int size;
///   } p;
///   /* m: output, matrix */
///   struct {
///     int rows;
///     int cols;
///   } m;
/// } f_meta_t;
/// f_meta_t f_meta() { return {{2}, {2, 2}}; }
/// @endcode
///
/// Note that in this example, the matrix `M` is stored in column-major order
/// and the `CodeGen` function respects the storage order in the generated code.
/// If `M` were stored in row-major order, `CodeGen` would return the following:
///
/// @code
/// void f(const double* p, double* m) {
///     m[0] = 1.000000;
///     m[1] = (4 * p[1]);
///     m[2] = (3 + p[0] + p[1]);
///     m[3] = sin(p[0]);
/// }
/// @endcode
template <typename Derived>
std::string CodeGen(const std::string& function_name,
                    const std::vector<Variable>& parameters,
                    const Eigen::PlainObjectBase<Derived>& M) {
  static_assert(std::is_same_v<typename Derived::Scalar, Expression>,
                "CodeGen should take a symbolic matrix.");
  std::ostringstream oss;
  internal::CodeGenDenseData(function_name, parameters, M.data(),
                             M.cols() * M.rows(), &oss);
  internal::CodeGenDenseMeta(function_name, parameters.size(), M.rows(),
                             M.cols(), &oss);
  return oss.str();
}

/// For a given symbolic column-major sparse matrix @p M, generates two C
/// functions, `<function_name>` and `<function_name>_meta`. The generated
/// `<function_name>` is used to construct a sparse matrix of double which
/// stores the evaluation result of the symbolic matrix @p M for a given
/// double-precision floating-point assignment for the symbolic variables in @p
/// M. `<function_name>` takes one input parameter `p` and three output
/// parameters (`outer_indicies`, `inner_indices`, and `values`).
///
///  - const double* p : An array of doubles for input parameters.
///  - int* outer_indices : An array of integer to store the starting positions
///                         of the inner vectors.
///  - int* inner_indices : An array of integer to store the array of inner
///                         indices.
///  - double* values : An array of doubles to store the evaluated non-zero
///  elements.
///
/// The three outputs, (`outer_indices`, `inner_indices`, `values`), represent a
/// sparse matrix in the widely-used Compressed Column Storage (CCS) scheme. For
/// more information about the CCS scheme, please read
/// https://eigen.tuxfamily.org/dox/group__TutorialSparse.html.
///
/// `<function_name>_meta()` returns a nested struct from which a caller can
/// obtain the following information:
///  - `.p.size`: the size of input parameters.
///  - `.m.rows`: the number of rows in the matrix.
///  - `.m.cols`: the number of columns in the matrix.
///  - `.m.non_zeros`: the number of non-zero elements in the matrix.
///  - `.m.outer_indices`: the length of the outer_indices.
///  - `.m.inner_indices`: the length of the inner_indices.
///
/// @throws std::exception if @p M is not compressed.
// TODO(soonho-tri): Support row-major sparse matrices.
///
/// Please consider the following example which generates code for a 3x6
/// sparse matrix.
///
/// @code
/// Eigen::SparseMatrix<Expression, Eigen::ColMajor> m(3, 6);
/// m.insert(0, 0) = x;
/// m.insert(0, 4) = z;
/// m.insert(1, 2) = y;
/// m.insert(2, 3) = y;
/// m.insert(2, 5) = y;
/// m.makeCompressed();
/// // | x  0  0  0  z  0|
/// // | 0  0  y  0  0  0|
/// // | 0  0  0  y  0  y|
/// CodeGen("f", {x, y, z}, m);
/// @endcode
///
/// When executed, the last line of the above example generates the following
/// code:
///
/// @code
/// void f(const double* p,
///        int* outer_indices,
///        int* inner_indices,
///        double* values) {
///     outer_indices[0] = 0;
///     outer_indices[1] = 1;
///     outer_indices[2] = 1;
///     outer_indices[3] = 2;
///     outer_indices[4] = 3;
///     outer_indices[5] = 4;
///     outer_indices[6] = 5;
///
///     inner_indices[0] = 0;
///     inner_indices[1] = 1;
///     inner_indices[2] = 2;
///     inner_indices[3] = 0;
///     inner_indices[4] = 2;
///
///     values[0] = p[0];
///     values[1] = p[1];
///     values[2] = p[1];
///     values[3] = p[2];
///     values[4] = p[1];
/// }
///
/// typedef struct {
///     /* p: input, vector */
///     struct { int size; } p;
///     /* m: output, matrix */
///     struct {
///         int rows;
///         int cols;
///         int non_zeros;
///     } m;
/// } f_meta_t;
/// f_meta_t f_meta() { return {{3}, {3, 6, 5}}; }
/// @endcode
///
/// In the following example, we show how to use the generated function to
/// evaluate the symbolic matrix and construct a sparse matrix of double using
/// `Eigen::Map`.
///
/// @code
/// // set up param, outer_indices, inner_indices, and values.
/// f_meta_t meta = f_meta();
/// const Eigen::Vector3d param{1 /* x */, 2 /* y */, 3 /* z */};
/// std::vector<int> outer_indices(meta.m.cols + 1);
/// std::vector<int> inner_indices(meta.m.non_zeros);
/// std::vector<double> values(meta.m.non_zeros);
///
/// // call f to fill outer_indices, inner_indices, and values.
/// f(param.data(), outer_indices.data(), inner_indices.data(), values.data());
///
/// // use Eigen::Map to turn (outer_indices, inner_indices, values) into a
/// // sparse matrix.
/// Eigen::Map<Eigen::SparseMatrix<double, Eigen::ColMajor>> map_sp(
///     meta.m.rows, meta.m.cols, meta.m.non_zeros, outer_indices.data(),
///     inner_indices.data(), values.data());
/// const Eigen::SparseMatrix<double> m_double{map_sp.eval()};
/// @endcode
std::string CodeGen(
    const std::string& function_name, const std::vector<Variable>& parameters,
    const Eigen::Ref<const Eigen::SparseMatrix<Expression, Eigen::ColMajor>>&
        M);
/// @} End of codegen group.

}  // namespace symbolic
}  // namespace drake
