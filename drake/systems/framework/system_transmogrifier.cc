#include "drake/systems/framework/system_transmogrifier.h"

namespace drake {
namespace systems {

using Expression = symbolic::Expression;

template <> std::unique_ptr<System<double>>
SystemTransmogrifier::Convert(const System<double>& other) const {
  if (!c00_) { return nullptr; }
  return c00_(other);
}

template <> std::unique_ptr<System<AutoDiffXd>>
SystemTransmogrifier::Convert(const System<double>& other) const {
  if (!c10_) { return nullptr; }
  return c10_(other);
}

template <> std::unique_ptr<System<Expression>>
SystemTransmogrifier::Convert(const System<double>& other) const {
  if (!c20_) { return nullptr; }
  return c20_(other);
}

template <> std::unique_ptr<System<double>>
SystemTransmogrifier::Convert(const System<AutoDiffXd>& other) const {
  if (!c01_) { return nullptr; }
  return c01_(other);
}

template <> std::unique_ptr<System<AutoDiffXd>>
SystemTransmogrifier::Convert(const System<AutoDiffXd>& other) const {
  if (!c11_) { return nullptr; }
  return c11_(other);
}

template <> std::unique_ptr<System<Expression>>
SystemTransmogrifier::Convert(const System<AutoDiffXd>& other) const {
  if (!c21_) { return nullptr; }
  return c21_(other);
}

template <> std::unique_ptr<System<double>>
SystemTransmogrifier::Convert(const System<Expression>& other) const {
  if (!c02_) { return nullptr; }
  return c02_(other);
}

template <> std::unique_ptr<System<AutoDiffXd>>
SystemTransmogrifier::Convert(const System<Expression>& other) const {
  if (!c12_) { return nullptr; }
  return c12_(other);
}

template <> std::unique_ptr<System<Expression>>
SystemTransmogrifier::Convert(const System<Expression>& other) const {
  if (!c22_) { return nullptr; }
  return c22_(other);
}

}  // namespace systems
}  // namespace drake
