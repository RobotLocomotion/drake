#pragma once

#include <map>
#include <memory>
#include <typeindex>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/symbolic_expression.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_transmogrifier_traits.h"
#include "drake/systems/framework/system_type_tag.h"
#include "drake/systems/framework/transmogrifier_tag.h"

namespace drake {
namespace systems {

/// Helper class to transmogrify a System<U> into a System<T>, intended for
/// internal use by the System framework, not directly by users.
///
/// Because it is not templated on any a System subclass, this class it can be
/// used by LeafSystem without any direct knowledge of what subtypes being
/// converted.  In other words, it enables a runtime flavor of the CRTP.
class SystemTransmogrifier {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SystemTransmogrifier);

  /// Creates an object that returns nullptr for all Convert() requests.  The
  /// single-argument constructor below is the typical way to create a useful
  /// instance of this type.
  SystemTransmogrifier();

  /// Creates an object that uses S's transmogrification copy constructor,
  /// i.e., S<T>::S(const TransmogrifierTag&, const S<U>&).
  ///
  /// At most, only scalar types drawn from the following list are supported:
  /// - double
  /// - AutoDiffXd
  /// - symbolic::Expression
  ///
  /// By default, all pairs drawn from the list can be used for T and U.
  /// Systems may specialize transmogrifier::Traits to support fewer types,
  /// or call Add<T, U>() on the returned object to support custom types.
  ///
  /// @tparam S is the System type to transmogrify
  template <template <typename> class S>
  explicit SystemTransmogrifier(SystemTypeTag<S>) : SystemTransmogrifier() {
    using Expression = symbolic::Expression;
    AddIfSupported<S, double,     double>();
    AddIfSupported<S, AutoDiffXd, double>();
    AddIfSupported<S, Expression, double>();
    AddIfSupported<S, double,     AutoDiffXd>();
    AddIfSupported<S, AutoDiffXd, AutoDiffXd>();
    AddIfSupported<S, Expression, AutoDiffXd>();
    AddIfSupported<S, double,     Expression>();
    AddIfSupported<S, AutoDiffXd, Expression>();
    AddIfSupported<S, Expression, Expression>();
  }

  /// Destructor.
  ~SystemTransmogrifier();

  /// A std::function used to transmogrify a System<U> into a System<T>.
  template <typename T, typename U>
  using TransmogrifierFunction =
      std::function<std::unique_ptr<System<T>>(const System<U>&)>;

  /// Registers the std::function to be used to transmogrify a System<U> into a
  /// System<T>.  Any given pair of types can be added at most once.
  template <typename T, typename U>
  void Add(const TransmogrifierFunction<T, U>&);

  /// Transmogrifies a System<U> into a System<T>.  This is the API that
  /// LeafSystem uses to provide a default implementation of transmogrify.
  ///
  /// @tparam U is the donor scalar type (to transmogrify from)
  /// @tparam T is the resulting scalar type (to transmogrify into)
  template <typename T, typename U>
  std::unique_ptr<System<T>> Convert(const System<U>& other) const;

 private:
  using Func = std::function<void*(const void*)>;
  void Insert(const std::type_info&, const std::type_info&, const Func&);
  const Func& Find(const std::type_info&, const std::type_info&) const;
  template <template <typename> class S, typename T, typename U>
  void AddIfSupported();

  // This maps from {T, U} pair to the Func that transmogrifies from U into T.
  std::map<std::pair<std::type_index, std::type_index>, Func> funcs_;
};

#if !defined(DRAKE_DOXYGEN_CXX)

template <typename T, typename U>
void SystemTransmogrifier::Add(const TransmogrifierFunction<T, U>& func) {
  // Make sure func contains a target (i.e., is not null-ish).
  DRAKE_ASSERT(static_cast<bool>(func));
  // Copy func into a lambda that ends up copied into our Impl, keyed on the
  // typeid's of T and U..  The lambda casts between T*,U* and void*,void* in
  // order to have a non-templated signature and thus fit into a homogeneously-
  // typed std::map.
  Insert(typeid(T), typeid(U), [func](const void* bare_u) {
    DRAKE_ASSERT(bare_u);
    const System<U>& other = *static_cast<const System<U>*>(bare_u);
    return func(other).release();
  });
}

template <typename T, typename U>
std::unique_ptr<System<T>> SystemTransmogrifier::Convert(
    const System<U>& other) const {
  // Lookup the lambda that Add() stored and call it.  If Find() does not see a
  // match, it will return a Func that yields null, _not_ a null-ish Func.
  const Func& converter = Find(typeid(T), typeid(U));
  System<T>* const result = static_cast<System<T>*>(converter(&other));
  return std::unique_ptr<System<T>>(result);
}

namespace system_transmogrifier_detail {
// When Traits says that transmogrification is supported.
template <template <typename> class S, typename T, typename U>
static std::unique_ptr<System<T>> Make(
    const System<U>& other, std::true_type) {
  const auto& my_other = dynamic_cast<const S<U>&>(other);
  return std::make_unique<S<T>>(TransmogrifierTag{}, my_other);
}
// When Traits says not to transmogrify.
template <template <typename> class S, typename T, typename U>
static std::unique_ptr<System<T>> Make(
    const System<U>&, std::false_type) {
  // AddIfSupported is guaranteed not to call us, but we *will* be compiled,
  // so we have to have some kind of function body.
  DRAKE_ABORT();
}
}  // namespace system_transmogrifier_detail

// Add transmogrifier for S<U> into an S<T>, iff Traits says its supported.
template <template <typename> class S, typename T, typename U>
void SystemTransmogrifier::AddIfSupported() {
  using supported =
      typename transmogrifier::Traits<S>::template supported<T, U>;
  if (supported::value) {
    const TransmogrifierFunction<T, U> func = [](const System<U>& other) {
      // Dispatch to an overload based on whether S<U> ==> S<T> is supported.
      return system_transmogrifier_detail::Make<S, T, U>(other, supported{});
    };
    Add(func);
  }
}

#endif  // DRAKE_DOXYGEN_CXX

}  // namespace systems
}  // namespace drake
