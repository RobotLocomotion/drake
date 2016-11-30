#include "drake/common/functional_form.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <ostream>
#include <type_traits>

#include "drake/common/drake_assert.h"

namespace drake {

namespace {
// Return the underlying index of an enum class type.
template <typename T>
constexpr typename std::underlying_type<T>::type IndexOf(T x) {
  return static_cast<typename std::underlying_type<T>::type>(x);
}
}  // namespace

// Hide this from doxygen because it incorrectly documents this
// as public.  It is not user-facing anyway.
#if !defined(DRAKE_DOXYGEN_CXX)
enum class FunctionalForm::Form {
  kZero,
  kConstant,
  kLinear,
  kAffine,
  kPolynomial,
  kDifferentiable,
  kArbitrary,
  kUndefined,
};
#endif  // !defined(DRAKE_DOXYGEN_CXX)

class FunctionalForm::Internal {
 public:
  // Short constant names to make op tables more readable.
  static Form constexpr ZERO = Form::kZero;
  static Form constexpr CONS = Form::kConstant;
  static Form constexpr LIN = Form::kLinear;
  static Form constexpr AFF = Form::kAffine;
  static Form constexpr POLY = Form::kPolynomial;
  static Form constexpr DIFF = Form::kDifferentiable;
  static Form constexpr ARB = Form::kArbitrary;
  static Form constexpr UNDF = Form::kUndefined;

  // Tables recording how ops combine forms.
  static int constexpr kSize = IndexOf(Form::kUndefined) + 1;
  static Form const kAdd[kSize][kSize];
  static Form const kMul[kSize][kSize];
  static Form const kDiv[kSize][kSize];
  static Form const kAbs[kSize];
  static Form const kCos[kSize];
  static Form const kExp[kSize];
  static Form const kLog[kSize];
  static Form const kMax[kSize][kSize];
  static Form const kSin[kSize];
  static Form const kSqrt[kSize];

  // Only some of the forms need to carry variables.
  static bool need_vars(Form f) {
    return f != Form::kZero && f != Form::kConstant;
  }

  static Form FormFromDouble(double d) {
    if (d == 0) {
      return Form::kZero;
    }
    if (std::isnan(d)) {
      return Form::kUndefined;
    }
    return Form::kConstant;
  }
};

FunctionalForm::FunctionalForm() : FunctionalForm(Form::kUndefined, {}) {}

FunctionalForm::FunctionalForm(double d)
    : FunctionalForm(Internal::FormFromDouble(d), {}) {}

FunctionalForm::FunctionalForm(Form f, Variables&& v)
    : vars_(std::move(v)), form_(f) {
  DRAKE_ASSERT(form_ == Form::kUndefined ||
               Internal::need_vars(form_) == !vars_.empty());
}

FunctionalForm FunctionalForm::Zero() {
  return FunctionalForm(Form::kZero, {});
}

FunctionalForm FunctionalForm::Constant() {
  return FunctionalForm(Form::kConstant, {});
}

FunctionalForm FunctionalForm::Linear(Variables v) {
  return FunctionalForm(Form::kLinear, std::move(v));
}

FunctionalForm FunctionalForm::Affine(Variables v) {
  return FunctionalForm(Form::kAffine, std::move(v));
}

FunctionalForm FunctionalForm::Polynomial(Variables v) {
  return FunctionalForm(Form::kPolynomial, std::move(v));
}

FunctionalForm FunctionalForm::Differentiable(Variables v) {
  return FunctionalForm(Form::kDifferentiable, std::move(v));
}

FunctionalForm FunctionalForm::Arbitrary(Variables v) {
  return FunctionalForm(Form::kArbitrary, std::move(v));
}

FunctionalForm FunctionalForm::Undefined(Variables v) {
  return FunctionalForm(Form::kUndefined, std::move(v));
}

bool FunctionalForm::IsZero() const { return form_ == Form::kZero; }

bool FunctionalForm::IsConstant() const { return form_ == Form::kConstant; }

bool FunctionalForm::IsLinear() const { return form_ == Form::kLinear; }

bool FunctionalForm::IsAffine() const { return form_ == Form::kAffine; }

bool FunctionalForm::IsPolynomial() const { return form_ == Form::kPolynomial; }

bool FunctionalForm::IsDifferentiable() const {
  return form_ == Form::kDifferentiable;
}

bool FunctionalForm::IsArbitrary() const { return form_ == Form::kArbitrary; }

bool FunctionalForm::IsUndefined() const { return form_ == Form::kUndefined; }

bool FunctionalForm::Is(FunctionalForm const& r) const {
  return form_ == r.form_ && vars_ == r.vars_;
}

FunctionalForm::Variables FunctionalForm::GetVariables() const { return vars_; }

FunctionalForm operator+(FunctionalForm const& l, FunctionalForm const& r) {
  FunctionalForm::Form f =
      FunctionalForm::Internal::kAdd[IndexOf(l.form_)][IndexOf(r.form_)];
  FunctionalForm::Variables vars;
  if (FunctionalForm::Internal::need_vars(f)) {
    vars = FunctionalForm::Variables::Union(l.vars_, r.vars_);
  }
  return FunctionalForm(f, std::move(vars));
}

FunctionalForm operator-(FunctionalForm const& l, FunctionalForm const& r) {
  FunctionalForm::Form f =
      FunctionalForm::Internal::kAdd[IndexOf(l.form_)][IndexOf(r.form_)];
  FunctionalForm::Variables vars;
  if (FunctionalForm::Internal::need_vars(f)) {
    vars = FunctionalForm::Variables::Union(l.vars_, r.vars_);
  }
  return FunctionalForm(f, std::move(vars));
}

FunctionalForm operator*(FunctionalForm const& l, FunctionalForm const& r) {
  FunctionalForm::Form f =
      FunctionalForm::Internal::kMul[IndexOf(l.form_)][IndexOf(r.form_)];
  FunctionalForm::Variables vars;
  if (FunctionalForm::Internal::need_vars(f)) {
    vars = FunctionalForm::Variables::Union(l.vars_, r.vars_);
  }
  return FunctionalForm(f, std::move(vars));
}

FunctionalForm operator/(FunctionalForm const& l, FunctionalForm const& r) {
  FunctionalForm::Form f =
      FunctionalForm::Internal::kDiv[IndexOf(l.form_)][IndexOf(r.form_)];
  FunctionalForm::Variables vars;
  if (FunctionalForm::Internal::need_vars(f)) {
    vars = FunctionalForm::Variables::Union(l.vars_, r.vars_);
  }
  return FunctionalForm(f, std::move(vars));
}

FunctionalForm abs(FunctionalForm const& x) {
  return FunctionalForm(FunctionalForm::Internal::kAbs[IndexOf(x.form_)],
                        FunctionalForm::Variables(x.vars_));
}

FunctionalForm cos(FunctionalForm const& x) {
  return FunctionalForm(FunctionalForm::Internal::kCos[IndexOf(x.form_)],
                        FunctionalForm::Variables(x.vars_));
}

FunctionalForm exp(FunctionalForm const& x) {
  return FunctionalForm(FunctionalForm::Internal::kExp[IndexOf(x.form_)],
                        FunctionalForm::Variables(x.vars_));
}

FunctionalForm log(FunctionalForm const& x) {
  return FunctionalForm(FunctionalForm::Internal::kLog[IndexOf(x.form_)],
                        FunctionalForm::Variables(x.vars_));
}

FunctionalForm max(FunctionalForm const& l, FunctionalForm const& r) {
  FunctionalForm::Form f =
      FunctionalForm::Internal::kMax[IndexOf(l.form_)][IndexOf(r.form_)];
  FunctionalForm::Variables vars;
  if (FunctionalForm::Internal::need_vars(f)) {
    vars = FunctionalForm::Variables::Union(l.vars_, r.vars_);
  }
  return FunctionalForm(f, std::move(vars));
}

FunctionalForm max(FunctionalForm const& l, double r) {
  return max(l, FunctionalForm(r));
}

FunctionalForm max(double l, FunctionalForm const& r) {
  return max(FunctionalForm(l), r);
}

FunctionalForm min(FunctionalForm const& l, FunctionalForm const& r) {
  FunctionalForm::Form f =
      FunctionalForm::Internal::kMax[IndexOf(l.form_)][IndexOf(r.form_)];
  FunctionalForm::Variables vars;
  if (FunctionalForm::Internal::need_vars(f)) {
    vars = FunctionalForm::Variables::Union(l.vars_, r.vars_);
  }
  return FunctionalForm(f, std::move(vars));
}

FunctionalForm min(FunctionalForm const& l, double r) {
  return min(l, FunctionalForm(r));
}

FunctionalForm min(double l, FunctionalForm const& r) {
  return min(FunctionalForm(l), r);
}

FunctionalForm sin(FunctionalForm const& x) {
  return FunctionalForm(FunctionalForm::Internal::kSin[IndexOf(x.form_)],
                        FunctionalForm::Variables(x.vars_));
}

FunctionalForm sqrt(FunctionalForm const& x) {
  return FunctionalForm(FunctionalForm::Internal::kSqrt[IndexOf(x.form_)],
                        FunctionalForm::Variables(x.vars_));
}

FunctionalForm& operator+=(FunctionalForm& l, FunctionalForm const& r) {
  l = l + r;
  return l;
}

FunctionalForm& operator-=(FunctionalForm& l, FunctionalForm const& r) {
  l = l - r;
  return l;
}

// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
FunctionalForm& operator*=(FunctionalForm& l, FunctionalForm const& r) {
  l = l * r;
  return l;
}

// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
FunctionalForm& operator/=(FunctionalForm& l, FunctionalForm const& r) {
  l = l / r;
  return l;
}

// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
FunctionalForm& operator+=(FunctionalForm& l, double r) {
  l = l + r;
  return l;
}

// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
FunctionalForm& operator-=(FunctionalForm& l, double r) {
  l = l - r;
  return l;
}

// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
FunctionalForm& operator*=(FunctionalForm& l, double r) {
  l = l * r;
  return l;
}

// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
FunctionalForm& operator/=(FunctionalForm& l, double r) {
  l = l / r;
  return l;
}

FunctionalForm operator+(FunctionalForm const& l, double r) {
  return l + FunctionalForm(r);
}
FunctionalForm operator+(double l, FunctionalForm const& r) {
  return FunctionalForm(l) + r;
}
FunctionalForm operator-(FunctionalForm const& l, double r) {
  return l - FunctionalForm(r);
}
FunctionalForm operator-(double l, FunctionalForm const& r) {
  return FunctionalForm(l) - r;
}
FunctionalForm operator*(FunctionalForm const& l, double r) {
  return l * FunctionalForm(r);
}
FunctionalForm operator*(double l, FunctionalForm const& r) {
  return FunctionalForm(l) * r;
}
FunctionalForm operator/(FunctionalForm const& l, double r) {
  return l / FunctionalForm(r);
}
FunctionalForm operator/(double l, FunctionalForm const& r) {
  return FunctionalForm(l) / r;
}

static const char* const kFunctionalFormNames[] = {
    "zero", "cons", "lin", "aff", "poly", "diff", "arb", "undf",
};

std::ostream& operator<<(std::ostream& os, FunctionalForm const& f) {
  os << kFunctionalFormNames[IndexOf(f.form_)];
  if (!f.vars_.empty()) {
    os << "(";
    const char* sep = "";
    for (auto const& v : f.vars_) {
      os << sep << v;
      sep = ",";
    }
    os << ")";
  }
  return os;
}

//---------------------------------------------------------------------------

// Hide this from doxygen because it incorrectly documents this
// as public.  It is not user-facing anyway.
#if !defined(DRAKE_DOXYGEN_CXX)
enum class FunctionalForm::Variable::Tag { kNil, kIndex, kNamed };
#endif  // !defined(DRAKE_DOXYGEN_CXX)

FunctionalForm::Variable::Variable() : tag_(Tag::kNil) {}

FunctionalForm::Variable::Variable(std::size_t index)
    : index_(index), tag_(Tag::kIndex) {}

FunctionalForm::Variable::Variable(std::string name)
    : name_(std::move(name)), tag_(Tag::kNamed) {
  DRAKE_ASSERT(!name_.empty());
}

FunctionalForm::Variable::Variable(Variable const& v) : tag_(v.tag_) {
  switch (tag_) {
    case Tag::kNil:
      break;
    case Tag::kIndex:
      new (&index_) std::size_t(v.index_);
      break;
    case Tag::kNamed:
      new (&name_) std::string(v.name_);
      break;
  }
}

FunctionalForm::Variable::Variable(Variable&& v) noexcept : tag_(v.tag_) {
  switch (tag_) {
    case Tag::kNil:
      break;
    case Tag::kIndex:
      new (&index_) std::size_t(std::move(v.index_));
      break;
    case Tag::kNamed:
      new (&name_) std::string(std::move(v.name_));
      break;
  }
  // Leave the object from which we moved in a default state.
  v.~Variable();
  new (&v) Variable();
}

FunctionalForm::Variable::~Variable() { Destruct(); }

FunctionalForm::Variable& FunctionalForm::Variable::operator=(
    Variable const& v) {
  Variable tmp(v);
  *this = std::move(tmp);
  return *this;
}

FunctionalForm::Variable& FunctionalForm::Variable::operator=(
    Variable&& v) noexcept {
  Destruct();
  new (this) Variable(std::move(v));
  return *this;
}

void FunctionalForm::Variable::Destruct() noexcept {
  switch (tag_) {
    case Tag::kNil:
      break;
    case Tag::kIndex:
      break;
    case Tag::kNamed:
      name_.~basic_string();
      break;
  }
}

bool FunctionalForm::Variable::is_nil() const { return tag_ == Tag::kNil; }

bool FunctionalForm::Variable::is_index() const { return tag_ == Tag::kIndex; }

bool FunctionalForm::Variable::is_named() const { return tag_ == Tag::kNamed; }

static std::size_t constexpr kInvalidIndex = ~static_cast<std::size_t>(0);

std::size_t FunctionalForm::Variable::index() const {
  if (is_index()) {
    return index_;
  } else {
    return kInvalidIndex;
  }
}

std::string const& FunctionalForm::Variable::name() const {
  if (is_named()) {
    return name_;
  } else {
    static std::string const kInvalidName;

    return kInvalidName;
  }
}

std::ostream& operator<<(std::ostream& os, FunctionalForm::Variable const& v) {
  using Tag = FunctionalForm::Variable::Tag;
  switch (v.tag_) {
    case Tag::kNil:
      break;
    case Tag::kIndex:
      os << v.index_;
      break;
    case Tag::kNamed:
      os << v.name_;
      break;
  }
  return os;
}

bool operator==(FunctionalForm::Variable const& l,
                FunctionalForm::Variable const& r) {
  if (l.tag_ == r.tag_) {
    using Tag = FunctionalForm::Variable::Tag;
    switch (l.tag_) {
      case Tag::kNil:
        return true;
      case Tag::kIndex:
        return l.index_ == r.index_;
      case Tag::kNamed:
        return l.name_ == r.name_;
    }
  }
  return false;
}

bool operator!=(FunctionalForm::Variable const& l,
                FunctionalForm::Variable const& r) {
  return !(l == r);
}

bool operator<(FunctionalForm::Variable const& l,
               FunctionalForm::Variable const& r) {
  if (l.tag_ == r.tag_) {
    using Tag = FunctionalForm::Variable::Tag;
    switch (l.tag_) {
      case Tag::kNil:
        return false;
      case Tag::kIndex:
        return l.index_ < r.index_;
      case Tag::kNamed:
        return l.name_ < r.name_;
    }
  }
  return IndexOf(l.tag_) < IndexOf(r.tag_);
}

bool operator<=(FunctionalForm::Variable const& l,
                FunctionalForm::Variable const& r) {
  return !(r < l);
}

bool operator>(FunctionalForm::Variable const& l,
               FunctionalForm::Variable const& r) {
  return (r < l);
}

bool operator>=(FunctionalForm::Variable const& l,
                FunctionalForm::Variable const& r) {
  return !(l < r);
}

//---------------------------------------------------------------------------

namespace {

std::vector<FunctionalForm::Variable> Unique(
    std::vector<FunctionalForm::Variable> vars) {
  std::sort(vars.begin(), vars.end());
  vars.erase(std::unique(vars.begin(), vars.end()), vars.end());
  return vars;
}

}  // namespace

FunctionalForm::Variables::Variables(std::initializer_list<Variable> init)
    : Variables(std::vector<Variable>(std::move(init))) {}

FunctionalForm::Variables::Variables(std::vector<Variable>&& vars)
    : vars_(std::make_shared<std::vector<Variable>>(Unique(std::move(vars)))) {}

FunctionalForm::Variables::const_iterator FunctionalForm::Variables::begin()
    const {
  return vars_ ? vars_->begin() : const_iterator();
}

FunctionalForm::Variables::const_iterator FunctionalForm::Variables::end()
    const {
  return vars_ ? vars_->end() : const_iterator();
}

bool FunctionalForm::Variables::empty() const {
  return vars_ ? vars_->empty() : true;
}

std::size_t FunctionalForm::Variables::size() const {
  return vars_ ? vars_->size() : 0;
}

FunctionalForm::Variables FunctionalForm::Variables::Union(Variables const& l,
                                                           Variables const& r) {
  if (r.empty()) {
    return l;
  }
  if (l.empty()) {
    return r;
  }
  std::vector<FunctionalForm::Variable> vars;
  vars.insert(vars.end(), l.vars_->begin(), l.vars_->end());
  vars.insert(vars.end(), r.vars_->begin(), r.vars_->end());
  return Variables(std::move(vars));
}

bool operator==(FunctionalForm::Variables const& lhs,
                FunctionalForm::Variables const& rhs) {
  if (lhs.vars_ == rhs.vars_) {
    return true;
  }
  if (!rhs.vars_) {
    return lhs.vars_->empty();
  }
  if (!lhs.vars_) {
    return rhs.vars_->empty();
  }
  return *lhs.vars_ == *rhs.vars_;
}

bool operator!=(FunctionalForm::Variables const& lhs,
                FunctionalForm::Variables const& rhs) {
  return !(lhs == rhs);
}

//---------------------------------------------------------------------------

FunctionalForm::Form const FunctionalForm::Internal::kAdd[kSize][kSize] = {
    /* clang-format off */
    /*    \ r:  ZERO  CONS   LIN   AFF  POLY  DIFF   ARB  UNDF */
    /*  l: \    ----  ----  ----  ----  ----  ----  ----  ---- */
    /* ZERO */ {ZERO, CONS,  LIN,  AFF, POLY, DIFF,  ARB, UNDF},
    /* CONS */ {CONS, CONS,  AFF,  AFF, POLY, DIFF,  ARB, UNDF},
    /* LIN  */ { LIN,  AFF,  LIN,  AFF, POLY, DIFF,  ARB, UNDF},
    /* AFF  */ { AFF,  AFF,  AFF,  AFF, POLY, DIFF,  ARB, UNDF},
    /* POLY */ {POLY, POLY, POLY, POLY, POLY, DIFF,  ARB, UNDF},
    /* DIFF */ {DIFF, DIFF, DIFF, DIFF, DIFF, DIFF,  ARB, UNDF},
    /* ARB  */ { ARB,  ARB,  ARB,  ARB,  ARB,  ARB,  ARB, UNDF},
    /* UNDF */ {UNDF, UNDF, UNDF, UNDF, UNDF, UNDF, UNDF, UNDF},
    /* clang-format on */
};

FunctionalForm::Form const FunctionalForm::Internal::kMul[kSize][kSize] = {
    /* clang-format off */
    /*    \ r:  ZERO  CONS   LIN   AFF  POLY  DIFF   ARB  UNDF */
    /*  l: \    ----  ----  ----  ----  ----  ----  ----  ---- */
    /* ZERO */ {ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, UNDF},
    /* CONS */ {ZERO, CONS,  LIN,  AFF, POLY, DIFF,  ARB, UNDF},
    /* LIN  */ {ZERO,  LIN, POLY, POLY, POLY, DIFF,  ARB, UNDF},
    /* AFF  */ {ZERO,  AFF, POLY, POLY, POLY, DIFF,  ARB, UNDF},
    /* POLY */ {ZERO, POLY, POLY, POLY, POLY, DIFF,  ARB, UNDF},
    /* DIFF */ {ZERO, DIFF, DIFF, DIFF, DIFF, DIFF,  ARB, UNDF},
    /* ARB  */ {ZERO,  ARB,  ARB,  ARB,  ARB,  ARB,  ARB, UNDF},
    /* UNDF */ {UNDF, UNDF, UNDF, UNDF, UNDF, UNDF, UNDF, UNDF},
    /* clang-format on */
};

FunctionalForm::Form const FunctionalForm::Internal::kDiv[kSize][kSize] = {
    /* clang-format off */
    /*    \ r:  ZERO  CONS   LIN   AFF  POLY  DIFF   ARB  UNDF */
    /*  l: \    ----  ----  ----  ----  ----  ----  ----  ---- */
    /* ZERO */ {UNDF, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, UNDF},
    /* CONS */ {UNDF, CONS, DIFF, DIFF, DIFF, DIFF,  ARB, UNDF},
    /* LIN  */ {UNDF,  LIN, DIFF, DIFF, DIFF, DIFF,  ARB, UNDF},
    /* AFF  */ {UNDF,  AFF, DIFF, DIFF, DIFF, DIFF,  ARB, UNDF},
    /* POLY */ {UNDF, POLY, DIFF, DIFF, DIFF, DIFF,  ARB, UNDF},
    /* DIFF */ {UNDF, DIFF, DIFF, DIFF, DIFF, DIFF,  ARB, UNDF},
    /* ARB  */ {UNDF,  ARB,  ARB,  ARB,  ARB,  ARB,  ARB, UNDF},
    /* UNDF */ {UNDF, UNDF, UNDF, UNDF, UNDF, UNDF, UNDF, UNDF},
    /* clang-format on */
};

FunctionalForm::Form const FunctionalForm::Internal::kAbs[kSize] = {
    /* clang-format off */
    /* ZERO  CONS   LIN   AFF  POLY  DIFF   ARB  UNDF */
    /* ----  ----  ----  ----  ----  ----  ----  ---- */
       ZERO, CONS, DIFF, DIFF, DIFF, DIFF,  ARB, UNDF,
    /* clang-format on */
};

FunctionalForm::Form const FunctionalForm::Internal::kCos[kSize] = {
    /* clang-format off */
    /* ZERO  CONS   LIN   AFF  POLY  DIFF   ARB  UNDF */
    /* ----  ----  ----  ----  ----  ----  ----  ---- */
       CONS, CONS, DIFF, DIFF, DIFF, DIFF,  ARB, UNDF,
    /* clang-format on */
};

FunctionalForm::Form const FunctionalForm::Internal::kExp[kSize] = {
    /* clang-format off */
    /* ZERO  CONS   LIN   AFF  POLY  DIFF   ARB  UNDF */
    /* ----  ----  ----  ----  ----  ----  ----  ---- */
       CONS, CONS, DIFF, DIFF, DIFF, DIFF,  ARB, UNDF,
    /* clang-format on */
};

FunctionalForm::Form const FunctionalForm::Internal::kLog[kSize] = {
    /* clang-format off */
    /* ZERO  CONS   LIN   AFF  POLY  DIFF   ARB  UNDF */
    /* ----  ----  ----  ----  ----  ----  ----  ---- */
       UNDF, CONS, DIFF, DIFF, DIFF, DIFF,  ARB, UNDF,
    /* clang-format on */
};

FunctionalForm::Form const FunctionalForm::Internal::kMax[kSize][kSize] = {
    /* clang-format off */
    /*    \ r:  ZERO  CONS   LIN   AFF  POLY  DIFF   ARB  UNDF */
    /*  l: \    ----  ----  ----  ----  ----  ----  ----  ---- */
    /* ZERO */ {ZERO, CONS, DIFF, DIFF, DIFF, DIFF,  ARB, UNDF},
    /* CONS */ {CONS, CONS, DIFF, DIFF, DIFF, DIFF,  ARB, UNDF},
    /* LIN  */ {DIFF, DIFF, DIFF, DIFF, DIFF, DIFF,  ARB, UNDF},
    /* AFF  */ {DIFF, DIFF, DIFF, DIFF, DIFF, DIFF,  ARB, UNDF},
    /* POLY */ {DIFF, DIFF, DIFF, DIFF, DIFF, DIFF,  ARB, UNDF},
    /* DIFF */ {DIFF, DIFF, DIFF, DIFF, DIFF, DIFF,  ARB, UNDF},
    /* ARB  */ { ARB,  ARB,  ARB,  ARB,  ARB,  ARB,  ARB, UNDF},
    /* UNDF */ {UNDF, UNDF, UNDF, UNDF, UNDF, UNDF, UNDF, UNDF},
    /* clang-format on */
};

FunctionalForm::Form const FunctionalForm::Internal::kSin[kSize] = {
    /* clang-format off */
    /* ZERO  CONS   LIN   AFF  POLY  DIFF   ARB  UNDF */
    /* ----  ----  ----  ----  ----  ----  ----  ---- */
       ZERO, CONS, DIFF, DIFF, DIFF, DIFF,  ARB, UNDF,
    /* clang-format on */
};

FunctionalForm::Form const FunctionalForm::Internal::kSqrt[kSize] = {
    /* clang-format off */
    /* ZERO  CONS   LIN   AFF  POLY  DIFF   ARB  UNDF */
    /* ----  ----  ----  ----  ----  ----  ----  ---- */
       ZERO, CONS, DIFF, DIFF, DIFF, DIFF,  ARB, UNDF,
    /* clang-format on */
};

}  // namespace drake
