#include "drake/common/symbolic_codegen.h"

#include <sstream>
#include <stdexcept>

#include <fmt/format.h>

namespace drake {
namespace symbolic {

using std::ostream;
using std::ostringstream;
using std::runtime_error;
using std::string;
using std::to_string;
using std::vector;

CodeGenVisitor::CodeGenVisitor(const vector<Variable>& parameters) {
  for (vector<Variable>::size_type i = 0; i < parameters.size(); ++i) {
    id_to_idx_map_.emplace(parameters[i].get_id(), i);
  }
}

string CodeGenVisitor::CodeGen(const Expression& e) const {
  return VisitExpression<string>(this, e);
}

string CodeGenVisitor::VisitVariable(const Expression& e) const {
  const Variable& v{get_variable(e)};
  const auto it{id_to_idx_map_.find(v.get_id())};
  if (it == id_to_idx_map_.end()) {
    throw runtime_error("Variable index is not found.");
  }
  return "p[" + to_string(it->second) + "]";
}

string CodeGenVisitor::VisitConstant(const Expression& e) const {
  return to_string(get_constant_value(e));
}

string CodeGenVisitor::VisitAddition(const Expression& e) const {
  const double c{get_constant_in_addition(e)};
  const auto& expr_to_coeff_map{get_expr_to_coeff_map_in_addition(e)};
  ostringstream oss;
  oss << "(" << c;
  for (const auto& item : expr_to_coeff_map) {
    const Expression& e_i{item.first};
    const double c_i{item.second};
    oss << " + ";
    if (c_i == 1.0) {
      oss << CodeGen(e_i);
    } else {
      oss << "(" << c_i << " * " << CodeGen(e_i) << ")";
    }
  }
  oss << ")";
  return oss.str();
}

string CodeGenVisitor::VisitMultiplication(const Expression& e) const {
  const double c{get_constant_in_multiplication(e)};
  const auto& base_to_exponent_map{
      get_base_to_exponent_map_in_multiplication(e)};
  ostringstream oss;
  oss << "(" << c;
  for (const auto& item : base_to_exponent_map) {
    const Expression& e_1{item.first};
    const Expression& e_2{item.second};
    oss << " * ";
    if (is_one(e_2)) {
      oss << CodeGen(e_1);
    } else {
      oss << "pow(" << CodeGen(e_1) << ", " << CodeGen(e_2) << ")";
    }
  }
  oss << ")";
  return oss.str();
}

// Helper method to handle unary cases.
string CodeGenVisitor::VisitUnary(const string& f, const Expression& e) const {
  return f + "(" + CodeGen(get_argument(e)) + ")";
}

// Helper method to handle binary cases.
string CodeGenVisitor::VisitBinary(const string& f, const Expression& e) const {
  return f + "(" + CodeGen(get_first_argument(e)) + ", " +
         CodeGen(get_second_argument(e)) + ")";
}

string CodeGenVisitor::VisitPow(const Expression& e) const {
  return VisitBinary("pow", e);
}

string CodeGenVisitor::VisitDivision(const Expression& e) const {
  return "(" + CodeGen(get_first_argument(e)) + " / " +
         CodeGen(get_second_argument(e)) + ")";
}

string CodeGenVisitor::VisitAbs(const Expression& e) const {
  return VisitUnary("fabs", e);
}

string CodeGenVisitor::VisitLog(const Expression& e) const {
  return VisitUnary("log", e);
}

string CodeGenVisitor::VisitExp(const Expression& e) const {
  return VisitUnary("exp", e);
}

string CodeGenVisitor::VisitSqrt(const Expression& e) const {
  return VisitUnary("sqrt", e);
}

string CodeGenVisitor::VisitSin(const Expression& e) const {
  return VisitUnary("sin", e);
}

string CodeGenVisitor::VisitCos(const Expression& e) const {
  return VisitUnary("cos", e);
}

string CodeGenVisitor::VisitTan(const Expression& e) const {
  return VisitUnary("tan", e);
}

string CodeGenVisitor::VisitAsin(const Expression& e) const {
  return VisitUnary("asin", e);
}

string CodeGenVisitor::VisitAcos(const Expression& e) const {
  return VisitUnary("acos", e);
}

string CodeGenVisitor::VisitAtan(const Expression& e) const {
  return VisitUnary("atan", e);
}

string CodeGenVisitor::VisitAtan2(const Expression& e) const {
  return VisitBinary("atan2", e);
}

string CodeGenVisitor::VisitSinh(const Expression& e) const {
  return VisitUnary("sinh", e);
}

string CodeGenVisitor::VisitCosh(const Expression& e) const {
  return VisitUnary("cosh", e);
}

string CodeGenVisitor::VisitTanh(const Expression& e) const {
  return VisitUnary("tanh", e);
}

string CodeGenVisitor::VisitMin(const Expression& e) const {
  return VisitBinary("fmin", e);
}

string CodeGenVisitor::VisitMax(const Expression& e) const {
  return VisitBinary("fmax", e);
}

string CodeGenVisitor::VisitCeil(const Expression& e) const {
  return VisitUnary("ceil", e);
}

string CodeGenVisitor::VisitFloor(const Expression& e) const {
  return VisitUnary("floor", e);
}

string CodeGenVisitor::VisitIfThenElse(const Expression&) const {
  throw runtime_error("Codegen does not support if-then-else expressions.");
}

string CodeGenVisitor::VisitUninterpretedFunction(const Expression&) const {
  throw runtime_error("Codegen does not support uninterpreted functions.");
}

string CodeGen(const string& function_name, const vector<Variable>& parameters,
               const Expression& e) {
  ostringstream oss;
  // Add header for the main function.
  oss << "double " << function_name << "(const double* p) {\n";
  // Codegen the expression.
  oss << "    return " << CodeGenVisitor{parameters}.CodeGen(e) << ";\n";
  // Add footer for the main function.
  oss << "}\n";
  // <function_name>_meta_t type.
  oss << "typedef struct {\n"
         "    /* p: input, vector */\n"
         "    struct { int size; } p;\n"
         "} "
      << function_name << "_meta_t;\n";
  // <function_name>_meta().
  oss << function_name << "_meta_t " << function_name << "_meta() { return {{"
      << parameters.size() << "}}; }\n";
  return oss.str();
}

namespace internal {
void CodeGenDenseData(const string& function_name,
                      const vector<Variable>& parameters,
                      const Expression* const data, const int size,
                      ostream* const os) {
  // Add header for the main function.
  (*os) << "void " << function_name << "(const double* p, double* m) {\n";
  const CodeGenVisitor visitor{parameters};
  for (int i = 0; i < size; ++i) {
    (*os) << "    "
          << "m[" << i << "] = " << visitor.CodeGen(data[i]) << ";\n";
  }
  // Add footer for the main function.
  (*os) << "}\n";
}

void CodeGenDenseMeta(const string& function_name, const int parameter_size,
                      const int rows, const int cols, ostream* const os) {
  // <function_name>_meta_t type.
  (*os) << "typedef struct {\n"
           "    /* p: input, vector */\n"
           "    struct { int size; } p;\n"
           "    /* m: output, matrix */\n"
           "    struct { int rows; int cols; } m;\n"
           "} "
        << function_name << "_meta_t;\n";
  // <function_name>_meta().
  (*os) << function_name << "_meta_t " << function_name << "_meta() { return {{"
        << parameter_size << "}, {" << rows << ", " << cols << "}}; }\n";
}

void CodeGenSparseData(const string& function_name,
                       const vector<Variable>& parameters,
                       const int outer_index_size, const int non_zeros,
                       const int* const outer_index_ptr,
                       const int* const inner_index_ptr,
                       const Expression* const value_ptr, ostream* const os) {
  // Print header.
  (*os) << fmt::format(
      "void {}(const double* p, int* outer_indices, int* "
      "inner_indices, double* values) {{\n",
      function_name);

  for (int i = 0; i < outer_index_size; ++i) {
    (*os) << fmt::format("    outer_indices[{0}] = {1};\n", i,
                         outer_index_ptr[i]);
  }
  for (int i = 0; i < non_zeros; ++i) {
    (*os) << fmt::format("    inner_indices[{0}] = {1};\n", i,
                         inner_index_ptr[i]);
  }
  const CodeGenVisitor visitor{parameters};
  for (int i = 0; i < non_zeros; ++i) {
    (*os) << fmt::format("    values[{0}] = {1};\n", i,
                         visitor.CodeGen(value_ptr[i]));
  }
  // Print footer.
  (*os) << "}\n";
}

void CodeGenSparseMeta(const string& function_name, const int parameter_size,
                       const int rows, const int cols, const int non_zeros,
                       const int outer_indices, const int inner_indices,
                       ostream* const os) {
  // <function_name>_meta_t type.
  (*os) << "typedef struct {\n"
           "    /* p: input, vector */\n"
           "    struct { int size; } p;\n"
           "    /* m: output, matrix */\n"
           "    struct {\n"
           "        int rows;\n"
           "        int cols;\n"
           "        int non_zeros;\n"
           "        int outer_indices;\n"
           "        int inner_indices;\n"
           "    } m;\n"
           "} "
        << function_name << "_meta_t;\n";
  // <function_name>_meta().
  (*os) << fmt::format(
      "{0}_meta_t {1}_meta() {{ return {{{{{2}}}, {{{3}, {4}, {5}, {6}, "
      "{7}}}}}; }}\n",
      function_name, function_name, parameter_size, rows, cols, non_zeros,
      outer_indices, inner_indices);
}

}  // namespace internal

std::string CodeGen(
    const std::string& function_name, const std::vector<Variable>& parameters,
    const Eigen::Ref<const Eigen::SparseMatrix<Expression>>& M) {
  DRAKE_ASSERT(M.isCompressed());
  ostringstream oss;
  internal::CodeGenSparseData(function_name, parameters, M.cols() + 1,
                              M.nonZeros(), M.outerIndexPtr(),
                              M.innerIndexPtr(), M.valuePtr(), &oss);
  internal::CodeGenSparseMeta(function_name, parameters.size(), M.rows(),
                              M.cols(), M.nonZeros(), M.cols() + 1,
                              M.nonZeros(), &oss);
  return oss.str();
}

}  // namespace symbolic
}  // namespace drake
