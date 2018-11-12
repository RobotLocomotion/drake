#pragma once

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

}  // namespace symbolic
}  // namespace drake
