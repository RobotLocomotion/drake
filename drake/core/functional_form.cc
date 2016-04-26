#include "functional_form.h"

#include <algorithm>
#include <cassert>
#include <iostream>

namespace drake {

#if !defined(DRAKE_DOXYGEN_CXX)
enum class FunctionalForm::Class {
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
  static bool need_vars(Class c) {
    return c != Class::kZero && c != Class::kConstant;
  }
};

FunctionalForm::FunctionalForm() : FunctionalForm(Class::kUndefined, {}) {}

FunctionalForm::FunctionalForm(double d)
    : FunctionalForm(d == 0 ? Class::kZero : Class::kConstant, {}) {}

FunctionalForm::FunctionalForm(Class c, Variables&& v)
    : vars_(std::move(v)), class_(c) {
  assert(class_ == Class::kUndefined ||
         Internal::need_vars(class_) == !vars_.empty());
}

FunctionalForm FunctionalForm::Zero() {
  return FunctionalForm(Class::kZero, {});
}

FunctionalForm FunctionalForm::Constant() {
  return FunctionalForm(Class::kConstant, {});
}

FunctionalForm FunctionalForm::Linear(Variables v) {
  return FunctionalForm(Class::kLinear, std::move(v));
}

FunctionalForm FunctionalForm::Affine(Variables v) {
  return FunctionalForm(Class::kAffine, std::move(v));
}

FunctionalForm FunctionalForm::Polynomial(Variables v) {
  return FunctionalForm(Class::kPolynomial, std::move(v));
}

FunctionalForm FunctionalForm::Differentiable(Variables v) {
  return FunctionalForm(Class::kDifferentiable, std::move(v));
}

FunctionalForm FunctionalForm::Arbitrary(Variables v) {
  return FunctionalForm(Class::kArbitrary, std::move(v));
}

FunctionalForm FunctionalForm::Undefined(Variables v) {
  return FunctionalForm(Class::kUndefined, std::move(v));
}

bool FunctionalForm::IsZero() const { return class_ == Class::kZero; }

bool FunctionalForm::IsConstant() const { return class_ == Class::kConstant; }

bool FunctionalForm::IsLinear() const { return class_ == Class::kLinear; }

bool FunctionalForm::IsAffine() const { return class_ == Class::kAffine; }

bool FunctionalForm::IsPolynomial() const {
  return class_ == Class::kPolynomial;
}

bool FunctionalForm::IsDifferentiable() const {
  return class_ == Class::kDifferentiable;
}

bool FunctionalForm::IsArbitrary() const { return class_ == Class::kArbitrary; }

bool FunctionalForm::IsUndefined() const { return class_ == Class::kUndefined; }

bool FunctionalForm::Is(FunctionalForm const& r) const {
  return class_ == r.class_ && vars_ == r.vars_;
}

FunctionalForm::Variables FunctionalForm::GetVariables() const { return vars_; }

static std::string const kFunctionalFormClassNames[] = {
    "zero", "cons", "lin", "aff", "poly", "diff", "arb", "undf",
};

std::ostream& operator<<(std::ostream& os, FunctionalForm const& f) {
  os << kFunctionalFormClassNames[int(f.class_)];
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

#if !defined(DRAKE_DOXYGEN_CXX)
enum class FunctionalForm::Variable::Tag { kNil, kIndex, kNamed };
#endif  // !defined(DRAKE_DOXYGEN_CXX)

FunctionalForm::Variable::Variable() : tag_(Tag::kNil) {}

FunctionalForm::Variable::Variable(std::size_t index)
    : index_(index), tag_(Tag::kIndex) {}

FunctionalForm::Variable::Variable(std::string&& name)
    : name_(std::move(name)), tag_(Tag::kNamed) {
  assert(!name_.empty());
}

FunctionalForm::Variable::Variable(std::string const& name)
    : Variable(std::string(name)) {}

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

FunctionalForm::Variable::Variable(Variable&& v) : tag_(v.tag_) {
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

FunctionalForm::Variable& FunctionalForm::Variable::operator=(Variable&& v) {
  Destruct();
  new (this) Variable(std::move(v));
  return *this;
}

void FunctionalForm::Variable::Destruct() {
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
  return int(l.tag_) < int(r.tag_);
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
    std::vector<FunctionalForm::Variable>&& vars) {
  std::sort(vars.begin(), vars.end());
  vars.erase(std::unique(vars.begin(), vars.end()), vars.end());
  return std::move(vars);
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
