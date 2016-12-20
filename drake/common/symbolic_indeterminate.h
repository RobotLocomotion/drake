#pragma once
#include <ostream>
#include <string>

#include "drake/common/symbolic_variable_cell.h"

namespace drake {
namespace symbolic {

class Indeterminate : public VariableCell {
 public:
  explicit Indeterminate(const std::string& name);

  bool equal_to(const VariableCell& var) const override;
  bool less(const VariableCell& var) const override;
  std::ostream& Display(std::ostream& os) const override;
};

}  // namespace symbolic
}  // namespace drake
