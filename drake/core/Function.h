#ifndef DRAKE_CORE_FUNCTION_H_
#define DRAKE_CORE_FUNCTION_H_

#include <string>
#include <vector>
#include <stdexcept>
#include <iostream>
#include <functional>
#include <memory>
#include <Eigen/Dense>
#include "Gradient.h"

namespace Drake {

/** InputOutputRelation
 * \brief Tags which can be used to inform algorithms about underlying structure
 *in a function
 * e.g., linear, affine, polynomial, analytic, differentiable, continuous,
 *measurable, and -- lastly -- arbitrary
 *
 * note: i considered using inheritance to capture the relationship, but passing
 *around types at runtime
 * was more of a pain than simply capturing the inheritance with the helper
 *functions below.
 */
struct InputOutputRelation {
  enum class Form {
    ARBITRARY,
    DIFFERENTIABLE,
    POLYNOMIAL,
    AFFINE,
    LINEAR,
    CONSTANT,
    ZERO
    // todo: add more (rational, ...)
  };
  Form form;
  // todo: add sparsity info

  InputOutputRelation(Form f) : form(f) {}

  static bool isA(const Form& f, const Form& base) {
    if (f == base || base == Form::ARBITRARY) return true;
    if (f == Form::ARBITRARY) return false;
    return isA(derivesFrom(f), base);
  }

  bool isA(Form base) { return isA(form, base); }

  static Form leastCommonAncestor(const Form& f1, const Form& f2) {
    if (f1 == Form::ARBITRARY || f2 == Form::ARBITRARY) return Form::ARBITRARY;
    if (isA(f2, f1)) return f1;
    return leastCommonAncestor(derivesFrom(f1), f2);
  }

  static Form leastCommonAncestor(std::initializer_list<Form> forms) {
    if (forms.size() < 1)
      throw std::runtime_error(
          "leastCommonAncestor requires at least one argument");
    if (forms.size() == 1) return *forms.begin();

    std::initializer_list<Form>::const_iterator iter = forms.begin();
    Form ret = *iter;
    while (++iter != forms.end()) {
      ret = leastCommonAncestor(ret, *iter);
    }
    return ret;
  }

  static InputOutputRelation composeWith(
      const InputOutputRelation& g,
      const InputOutputRelation& f) {  // composition of functions y = g(f(x))
    return InputOutputRelation(leastCommonAncestor(g.form, f.form));
  }

  /*
  static InputOutputRelation linearCombination(const InputOutputRelation& g,
  const InputOutputRelation& f) {
    return InputOutputRelation(leastCommonAncestor({g.form, f.form, LINEAR}));
  }*/
  static InputOutputRelation combine(
      const InputOutputRelation& a,
      const InputOutputRelation& b) {  // vertical concatenation
    return InputOutputRelation(leastCommonAncestor(a.form, b.form));
  }

  static InputOutputRelation combine(
      std::initializer_list<InputOutputRelation> args) {
    if (args.size() < 1)
      throw std::runtime_error("combine requires at least one argument");
    if (args.size() == 1) return *args.begin();

    std::initializer_list<InputOutputRelation>::const_iterator iter =
        args.begin();
    InputOutputRelation ret = *iter;
    while (++iter != args.end()) {
      ret = combine(ret, *iter);
    }
    return ret;
  }

 private:
  static Form derivesFrom(const Form& f) {  // capture the inheritance
                                            // relationships (without resorting
                                            // to using types)
    switch (f) {
      case Form::DIFFERENTIABLE:
        return Form::ARBITRARY;
      case Form::POLYNOMIAL:
        return Form::DIFFERENTIABLE;
      case Form::AFFINE:
        return Form::POLYNOMIAL;
      case Form::LINEAR:
        return Form::AFFINE;
      case Form::CONSTANT:
        return Form::AFFINE;
      case Form::ZERO:
        return Form::LINEAR;  // note: really want multiple inheritance here,
                              // since it's also constant
      default:
        return Form::ARBITRARY;
    }
  }
};

template <typename ScalarType>
using VecIn = Eigen::Ref<Eigen::Matrix<ScalarType, Eigen::Dynamic, 1> const>;
template <typename ScalarType>
using VecOut = Eigen::Matrix<ScalarType, Eigen::Dynamic, 1>;

/** FunctionTraits
 * @brief Define interface to a function of the form y = f(x).
 */
template <typename F>
struct FunctionTraits {
  // TODO: add in/out relation, possibly distinguish differentiable functions
  static size_t numInputs(F const& f) { return f.numInputs(); }
  static size_t numOutputs(F const& f) { return f.numOutputs(); }
  template <typename ScalarType>
  static void eval(F const& f, VecIn<ScalarType> const& x,
                   VecOut<ScalarType>& y) {
    f.eval(x, y);
  }
};

template <typename F>
struct FunctionTraits<std::reference_wrapper<F>> {
  static size_t numInputs(std::reference_wrapper<F> const& f) {
    return f.get().numInputs();
  }
  static size_t numOutputs(std::reference_wrapper<F> const& f) {
    return f.get().numOutputs();
  }
  template <typename ScalarType>
  static void eval(std::reference_wrapper<F> const& f,
                   VecIn<ScalarType> const& x, VecOut<ScalarType>& y) {
    f.get().eval(x, y);
  }
};

template <typename F>
struct FunctionTraits<std::shared_ptr<F>> {
  static size_t numInputs(std::shared_ptr<F> const& f) {
    return f->numInputs();
  }
  static size_t numOutputs(std::shared_ptr<F> const& f) {
    return f->numOutputs();
  }
  template <typename ScalarType>
  static void eval(std::shared_ptr<F> const& f, VecIn<ScalarType> const& x,
                   VecOut<ScalarType>& y) {
    f->eval(x, y);
  }
};

template <typename F>
struct FunctionTraits<std::unique_ptr<F>> {
  static size_t numInputs(std::unique_ptr<F> const& f) {
    return f->numInputs();
  }
  static size_t numOutputs(std::unique_ptr<F> const& f) {
    return f->numOutputs();
  }
  template <typename ScalarType>
  static void eval(std::unique_ptr<F> const& f, VecIn<ScalarType> const& x,
                   VecOut<ScalarType>& y) {
    f->eval(x, y);
  }
};

// idea: use templates to support multi-input, multi-output functions which
// implement, e.g.
// void eval(x1,..., xn,  y1,..., ym), and
// InputOutputRelation getInputOutputRelation(input_index, output_index)
};

#endif  // DRAKE_CORE_FUNCTION_H_
