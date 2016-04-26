#include "drake/core/functional_form.h"

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <ostream>
#include <type_traits>

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
  // Only some of the forms need to carry variables.
  static bool need_vars(Form f) {
    return f != Form::kZero && f != Form::kConstant;
  }
};

FunctionalForm::FunctionalForm() : FunctionalForm(Form::kUndefined, {}) {}

FunctionalForm::FunctionalForm(double d)
    : FunctionalForm(d == 0 ? Form::kZero : Form::kConstant, {}) {}

FunctionalForm::FunctionalForm(Form f, Variables&& v)
    : vars_(std::move(v)), form_(f) {
  assert(form_ == Form::kUndefined ||
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

static std::string const kFunctionalFormNames[] = {
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
  assert(!name_.empty());
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

static std::string const kInvalidName;

std::string const& FunctionalForm::Variable::name() const {
  if (is_named()) {
    return name_;
  } else {
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

}  // namespace drake
