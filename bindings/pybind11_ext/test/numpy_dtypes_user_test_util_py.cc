/// @file
/// Tests NumPy user dtypes, using `pybind11`s C++ testing infrastructure.

#include <memory>
#include <string>

#include <Eigen/Dense>
#include <fmt/format.h>
#include <fmt/ostream.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>

#include "drake/bindings/pybind11_ext/numpy_dtypes_user.h"

using std::string;
using std::to_string;
using std::unique_ptr;

namespace py = pybind11;

namespace {

template <typename T>
using MatrixX = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

/*
Goals:

 * Show API
 * Show operator overloads
 * Exercise memory bugs (allocation, etc.)
 * Show implicit / explicit conversions.

The simplest mechanism is to do super simple symbolics.
*/

// Captures length of a given `Symbol`. Can implicitly convert to and from a
// `Symbol.
class LengthValueImplicit {
 public:
  // NOLINTNEXTLINE(runtime/explicit)
  LengthValueImplicit(int value) : value_(value) {}
  int value() const { return value_; }

  bool operator==(const LengthValueImplicit& other) const {
    return value_ == other.value_;
  }
 private:
  int value_{};
};

// Captures value of a given `Symbol`. Can explicitly convert to and from a
// `Symbol.
class StrValueExplicit {
 public:
  explicit StrValueExplicit(const string& value) : value_(new string(value)) {}
  const string& value() const { return *value_; }
 private:
  std::shared_ptr<string> value_{};
};

// No construction possible from this; purely for testing explicit operator
// overloads.
struct OperandExplicit {};

class Symbol {
 public:
  Symbol() : Symbol("") {}

  Symbol(const Symbol& other) : Symbol(other.value()) {}
  // `operator=` must be overloaded so that we do not copy the underyling
  // `shared_ptr` (when creating an array repeating from the same scalar).
  Symbol& operator=(const Symbol& other) {
    // WARNING: Because NumPy can assign from `memzero` memory, we must handle
    // this case.
    if (!str_) str_.reset(new string());
    *str_ = *other.str_;
    return *this;
  }

  explicit Symbol(string str) : str_(new string(str)) {}

  // Explicit conversion.
  explicit Symbol(const StrValueExplicit& other) : Symbol(other.value()) {}

  // Implicit conversion.
  // NOLINTNEXTLINE(runtime/explicit)
  Symbol(const LengthValueImplicit& other)
      : Symbol(fmt::format("length({})", other.value())) {}
  explicit Symbol(double value) : Symbol(fmt::format("float({})", value)) {}

  // N.B. Due to constraints of `pybind11`s architecture, we must try to handle
  // `str` conversion from an invalid state. See `add_init`.
  // WARNING: If the user encounters `memzero` memory, this case must handled.
  string value() const { return str_ ? *str_ : "<invalid>"; }

  // To be explicit.
  operator int() const { return str_->size(); }
  operator StrValueExplicit() const { return StrValueExplicit(*str_); }
  // To be implicit.
  operator LengthValueImplicit() const { return str_->size(); }

  template <typename... Args>
  static Symbol format(string pattern, const Args&... args) {
    return Symbol(fmt::format(pattern, args...));
  }

  // Closed under the following operators:
  Symbol& operator+=(const Symbol& rhs) { return inplace_binary("+", rhs); }
  Symbol operator+(const Symbol& rhs) const { return binary("+", rhs); }
  Symbol& operator-=(const Symbol& rhs) { return inplace_binary("-", rhs); }
  Symbol operator-(const Symbol& rhs) const { return binary("-", rhs); }
  Symbol& operator*=(const Symbol& rhs) { return inplace_binary("*", rhs); }
  Symbol operator*(const Symbol& rhs) const { return binary("*", rhs); }
  Symbol& operator/=(const Symbol& rhs) { return inplace_binary("/", rhs); }
  Symbol operator/(const Symbol& rhs) const { return binary("/", rhs); }
  Symbol& operator&=(const Symbol& rhs) { return inplace_binary("&", rhs); }
  Symbol operator&(const Symbol& rhs) const { return binary("&", rhs); }
  Symbol& operator|=(const Symbol& rhs) { return inplace_binary("|", rhs); }
  Symbol operator|(const Symbol& rhs) const { return binary("|", rhs); }
  Symbol operator==(const Symbol& rhs) const { return binary("==", rhs); }
  Symbol operator!=(const Symbol& rhs) const { return binary("!=", rhs); }
  Symbol operator<(const Symbol& rhs) const { return binary("<", rhs); }
  Symbol operator<=(const Symbol& rhs) const { return binary("<=", rhs); }
  Symbol operator>(const Symbol& rhs) const { return binary(">", rhs); }
  Symbol operator>=(const Symbol& rhs) const { return binary(">=", rhs); }
  Symbol operator&&(const Symbol& rhs) const { return binary("&&", rhs); }
  Symbol operator||(const Symbol& rhs) const { return binary("||", rhs); }

  // - Not closed.
  Symbol& operator+=(const OperandExplicit&) {
    *str_ = fmt::format("({}) + operand", *this);
    return *this;
  }
  Symbol operator+(const OperandExplicit&) const {
    Symbol lhs(*this);
    lhs += OperandExplicit{};
    return lhs;
  }

 private:
  Symbol binary(const char* op, const Symbol& rhs) const {
    Symbol lhs(*this);
    lhs.inplace_binary(op, rhs);
    return lhs;
  }
  Symbol& inplace_binary(const char* op, const Symbol& rhs) {
    *str_ = fmt::format("({}) {} ({})", value(), op, rhs.value());
    return *this;
  }

  // Data member to ensure that we do not get segfaults when carrying around
  // `shared_ptr`s, and to ensure that the data is memcpy-moveable.
  // N.B. This is not used for Copy-on-Write optimizations.
  std::shared_ptr<string> str_;
};

Symbol operator+(const OperandExplicit&, const Symbol& rhs) {
  return Symbol::format("operand + ({})", rhs);
}

std::ostream& operator<<(std::ostream& os, const Symbol& s) {
  return os << s.value();
}

namespace math {

Symbol abs(const Symbol& s) { return Symbol::format("abs({})", s); }
Symbol cos(const Symbol& s) { return Symbol::format("cos({})", s); }
Symbol sin(const Symbol& s) { return Symbol::format("sin({})", s); }
Symbol pow(const Symbol& a, const Symbol& b) {
  return Symbol::format("({}) ^ ({})", a, b);
}

}  // namespace math

template <typename Class, typename Return>
auto MakeRepr(const string& name, Return (Class::*method)() const) {
  return [name, method](Class* self) {
    return py::str("<{} '{}'>").format(name, (self->*method)());
  };
}

template <typename Class, typename Return>
auto MakeStr(Return (Class::*method)() const) {
  return [method](Class* self) {
    return py::str("{}").format((self->*method)());
  };
}

// Simple container to check referencing of symbols.
class SymbolContainer {
 public:
  SymbolContainer(int rows, int cols) : symbols_(rows, cols) {}
  Eigen::Ref<MatrixX<Symbol>> symbols() { return symbols_; }

 private:
  MatrixX<Symbol> symbols_;
};

}  // namespace

PYBIND11_NUMPY_DTYPE_USER(LengthValueImplicit);
PYBIND11_NUMPY_DTYPE_USER(StrValueExplicit);
PYBIND11_NUMPY_DTYPE_USER(OperandExplicit);
PYBIND11_NUMPY_DTYPE_USER(Symbol);

namespace {

PYBIND11_MODULE(numpy_dtypes_user_test_util, m) {
  // N.B. You must pre-declare all types that must interact using UFuncs, as
  // they must already be registered at that point of defining the UFunc.
  py::dtype_user<LengthValueImplicit> length(m, "LengthValueImplicit");
  py::dtype_user<StrValueExplicit> str(m, "StrValueExplicit");
  py::dtype_user<OperandExplicit> operand(m, "OperandExplicit");
  py::dtype_user<Symbol> sym(m, "Symbol");

  length  // BR
      .def(py::init<int>())
      .def("value", &LengthValueImplicit::value)
      .def("__repr__",
           MakeRepr("LengthValueImplicit", &LengthValueImplicit::value))
      .def("__str__", MakeStr(&LengthValueImplicit::value))
      .def_loop(py::self == py::self);

  str  // BR
      .def(py::init<string>())
      .def("value", &StrValueExplicit::value)
      .def("__repr__", MakeRepr("StrValueExplicit", &StrValueExplicit::value))
      .def("__str__", MakeStr(&StrValueExplicit::value));

  operand  // BR
      .def(py::init())
      .def("__repr__",
           [](const OperandExplicit&) { return "<OperandExplicit>"; })
      .def("__str__",
           [](const OperandExplicit&) { return "<OperandExplicit>"; });

  sym  // BR
      // Nominal definitions.
      .def(py::init())
      .def(py::init<const string&>())
      .def(py::init<double>())
      // N.B. Constructing `StrValueExplicit` only matters for user experience,
      // since it's explicit. However, implicit conversions *must* have an
      // accompanying constructor.
      .def(py::init<const LengthValueImplicit&>())
      .def(py::init<const Symbol&>())
      .def("__repr__", MakeRepr("Symbol", &Symbol::value))
      .def("__str__", MakeStr(&Symbol::value))
      .def("value", &Symbol::value)
      // - Test referencing.
      .def("self_reference",
           [](const Symbol& self) { return &self; },
           py::return_value_policy::reference)
      // Casting.
      // - From
      // WARNING: See above about implicit conversions + constructors.
      .def_loop(py::dtype_method::explicit_conversion<double, Symbol>())
      .def_loop(py::dtype_method::explicit_conversion<
          StrValueExplicit, Symbol>())
      .def_loop(py::dtype_method::implicit_conversion<
          LengthValueImplicit, Symbol>())
      // - To
      .def_loop(py::dtype_method::explicit_conversion<Symbol, int>())
      .def_loop(py::dtype_method::explicit_conversion<
          Symbol, StrValueExplicit>())
      .def_loop(py::dtype_method::implicit_conversion<
          Symbol, LengthValueImplicit>())
      // Operators.
      // N.B. Inplace operators do not have UFuncs in NumPy.
      // - Math.
      .def_loop(py::self + py::self)
      .def(py::self += py::self)
      .def_loop(py::self - py::self)
      .def(py::self -= py::self)
      .def_loop(py::self * py::self)
      .def(py::self *= py::self)
      .def_loop(py::self / py::self)
      .def(py::self /= py::self)
      // - Bitwise.
      .def_loop(py::self & py::self)
      .def(py::self &= py::self)
      .def_loop(py::self | py::self)
      .def(py::self |= py::self)
      // - Logical.
      .def_loop(py::self == py::self)
      .def_loop(py::self != py::self)
      .def_loop(py::self < py::self)
      .def_loop(py::self <= py::self)
      .def_loop(py::self > py::self)
      .def_loop(py::self >= py::self)
      // - Not closed.
      .def_loop(py::self + OperandExplicit{})
      // NOLINTNEXTLINE(whitespace/braces)
      .def_loop(OperandExplicit{} + py::self)
      .def(py::self += OperandExplicit{})
      // .def_loop(py::self && py::self)
      // .def_loop(py::self || py::self)
      // Explicit UFunc.
      .def_loop(py::dtype_method::dot())
      .def_loop("__pow__", &math::pow)
      .def_loop("abs", &math::abs)
      .def_loop("cos", &math::cos)
      .def_loop("sin", &math::sin);

  py::ufunc(m, "custom_binary_ufunc")
      .def_loop<Symbol>([](const Symbol& lhs, const Symbol& rhs) {
        return Symbol::format("custom-symbol({}, {})", lhs, rhs);
      })
      .def_loop<Symbol>([](const Symbol& lhs, const OperandExplicit& rhs) {
        return Symbol::format("custom-operand-rhs({})", lhs);
      })
      .def_loop<Symbol>([](const OperandExplicit& lhs, const Symbol& rhs) {
        return Symbol::format("custom-operand-lhs({})", rhs);
      })
      .def_loop<LengthValueImplicit>(
          [](const LengthValueImplicit& lhs, const LengthValueImplicit& rhs) {
            return LengthValueImplicit(lhs.value() + rhs.value());
          })
      .def_loop<StrValueExplicit>(
          [](const StrValueExplicit& lhs, const StrValueExplicit& rhs) {
            return StrValueExplicit(fmt::format(
                "custom-str({}, {})", lhs.value(), rhs.value()));
          });

  m.def("add_one",
        [](Eigen::Ref<MatrixX<Symbol>> value) {
          value.array() += Symbol(1.);
        });

  py::class_<SymbolContainer>(m, "SymbolContainer")
      .def(py::init<int, int>(), py::arg("rows"), py::arg("cols"))
      .def("symbols", &SymbolContainer::symbols,
           py::return_value_policy::reference_internal);
}

}  // namespace
