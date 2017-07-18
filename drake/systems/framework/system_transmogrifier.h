#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/symbolic_expression.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace systems {

/// A tag type (dummy type) that stands in as the first constructor argument
/// for the "transmogrification copy constructor".  For example:
///
/// @code
/// template <typename T>
/// class MySystem : public LeafSystem<T> {
///  public:
///   /// User constructor.
///   MySystem(... stuff ...) {
///     this->template SetConcreteSubclass<MySystem>();
///   }
///
///   /// Transmogrification constructor.
///   template <typename U>
///   SystemA(const TransmogrifierTag&, const MySystem<U>& other)
///       : SystemA(other.get_stuff()) {}
/// };
/// @endcode
///
/// The presence of this type as the first argument to a constructor marks the
/// "transmogrification copy constructor" -- a constructor where a System with
/// a different scalar typed is passed in as an const reference argument and
/// the constructor should initialize its own System to match `other` by
/// delegating to a conventional constructor.
struct TransmogrifierTag {};

namespace system_transmogrifier {

/// A templated traits class for whether S<T> can transmogrify into S<U>; the
/// default value is true for all values of S, T, and U.
///
/// Particular System subclasses may specialize this template to indicate
/// whether the framework should support transmogrification for certain
/// combinations of S, T, and U.
///
/// In supported cases, the "transmogrification copy constructor" for those
/// types will be used; in unsupported cases, the "transmogrification copy
/// constructor" will not even be mentioned, so that S need not even compile
/// for certain values of T or U.
///
/// @tparam S is the System type to transmogrify
template <template <typename> class S>
struct Traits {
  /// @tparam U is the donor scalar type (to transmogrify from)
  /// @tparam T is the resulting scalar type (to transmogrify into)
  template <typename T, typename U>
  using supported = std::true_type;
};

/// A concrete traits class providing sugar to disable support for
/// symbolic::Expression.  For example, if MySystem does not support the
/// symbolic expression scalar type, it should specialize Traits as follows:
///
/// @code
/// namespace system_transmogrifier {
/// template <> struct Traits<MySystem> : public NonSymbolicTraits {};
/// }
/// @endcode
struct NonSymbolicTraits {
  template <typename T, typename U>
  using supported = typename std::conditional<
    !std::is_same<T, symbolic::Expression>::value &&
    !std::is_same<U, symbolic::Expression>::value,
    std::true_type, std::false_type>::type;
};

}  // namespace system_transmogrifier

/// Helper class to transmogrify a System<U> into a System<T>, intended for
/// internal use by the System framework, not directly by users.
///
/// Because it is not templated on any a System subclass, this class it can be
/// used by LeafSystem without any direct knowledge of what subtypes being
/// converted.  In other words, it enables a runtime flavor of the CRTP.
///
/// Only scalar types drawn from the following list are supported:
/// - double
/// - AutoDiffXd
/// - symbolic::Expression
///
class SystemTransmogrifier {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SystemTransmogrifier)

  /// A default-constructed object is a no-op object that returns nullptr for
  /// all Convert() requests.  The MakeDefaultTransmogrifier() function is the
  /// typical way to create a useful instance of this type.
  SystemTransmogrifier() = default;

  /// Transmogrifies a System<U> into a System<T>.  This is the API that
  /// LeafSystem uses to provide a default implementation of transmogrify.
  /// The class overview explains the supported values of U and T.
  //
  /// @tparam U is the donor scalar type (to transmogrify from)
  /// @tparam T is the resulting scalar type (to transmogrify into)
  template <typename T, typename U>
  std::unique_ptr<System<T>> Convert(const System<U>&) const {
    // When we don't know a U -> T conversion, return nullptr.  We have member
    // function template specializations below for the supported U x T matrix.
    return nullptr;
  }

 private:
  using Expression = symbolic::Expression;

  // A std::function that transmogrifies System<U> to System<T>.
  template <typename T, typename U>
  using ConvertFunction =
      std::function<std::unique_ptr<System<T>>(const System<U>&)>;

  // Only allow MakeDefaultSystemTransmogrifier to call our Make().
  template <template <typename> class S>
  friend SystemTransmogrifier MakeDefaultSystemTransmogrifier();

  // Factory method that takes a Converter struct template argument that
  // provides a static template method Converter::Convert<T, U>.
  template <typename Converter>
  static SystemTransmogrifier Make() {
    SystemTransmogrifier result;
    // clang-format off
    result.c00_ = &(Converter::template Convert<double,     double>);
    result.c10_ = &(Converter::template Convert<AutoDiffXd, double>);
    result.c20_ = &(Converter::template Convert<Expression, double>);
    result.c01_ = &(Converter::template Convert<double,     AutoDiffXd>);
    result.c11_ = &(Converter::template Convert<AutoDiffXd, AutoDiffXd>);
    result.c21_ = &(Converter::template Convert<Expression, AutoDiffXd>);
    result.c02_ = &(Converter::template Convert<double,     Expression>);
    result.c12_ = &(Converter::template Convert<AutoDiffXd, Expression>);
    result.c22_ = &(Converter::template Convert<Expression, Expression>);
    // clang-format on
    return result;
  }

  // The matrix of conversion functions.  Any or all may be null.
  // clang-format off
  ConvertFunction<double,     double>     c00_;
  ConvertFunction<AutoDiffXd, double>     c10_;
  ConvertFunction<Expression, double>     c20_;
  ConvertFunction<double,     AutoDiffXd> c01_;
  ConvertFunction<AutoDiffXd, AutoDiffXd> c11_;
  ConvertFunction<Expression, AutoDiffXd> c21_;
  ConvertFunction<double,     Expression> c02_;
  ConvertFunction<AutoDiffXd, Expression> c12_;
  ConvertFunction<Expression, Expression> c22_;
  // clang-format on
};

#if !defined(DRAKE_DOXYGEN_CXX)

// Declare the member function specializations that exist in our cc file.
// These are the transmogrifications known to (and used by) the framework.
template <> std::unique_ptr<System<double>>
SystemTransmogrifier::Convert(const System<double>&) const;
template <> std::unique_ptr<System<AutoDiffXd>>
SystemTransmogrifier::Convert(const System<double>&) const;
template <> std::unique_ptr<System<symbolic::Expression>>
SystemTransmogrifier::Convert(const System<double>&) const;
template <> std::unique_ptr<System<double>>
SystemTransmogrifier::Convert(const System<AutoDiffXd>&) const;
template <> std::unique_ptr<System<AutoDiffXd>>
SystemTransmogrifier::Convert(const System<AutoDiffXd>&) const;
template <> std::unique_ptr<System<symbolic::Expression>>
SystemTransmogrifier::Convert(const System<AutoDiffXd>&) const;
template <> std::unique_ptr<System<double>>
SystemTransmogrifier::Convert(const System<symbolic::Expression>&) const;
template <> std::unique_ptr<System<AutoDiffXd>>
SystemTransmogrifier::Convert(const System<symbolic::Expression>&) const;
template <> std::unique_ptr<System<symbolic::Expression>>
SystemTransmogrifier::Convert(const System<symbolic::Expression>&) const;

namespace detail {
// Provide a Converter template argument for SystemTransmogrifier::Make().
template <template <typename> class S>
struct TransmogrifierTagConverter {
  // Disable construction; only use static methods.
  TransmogrifierTagConverter() = delete;

  // Transmogrify an S<U> into an S<T> using the TransmogrifierTag constructor,
  // i.e., S<T>::S(const TransmogrifierTag&, const S<U>&).  Throws an exception
  // if the @p other object is not a System subclass of type S.
  template <typename T, typename U>
  static std::unique_ptr<System<T>> Convert(const System<U>& other) {
    // Dispatch to an overload based on whether S<U> --> S<T> is supported.
    auto supported =
        typename system_transmogrifier::Traits<S>::template supported<T, U>{};
    return TransmogrifierTagConverter::ConvertImpl<T, U>(other, supported);
  }

 private:
  // When Traits says that transmogrification is supported.
  template <typename T, typename U>
  static std::unique_ptr<System<T>> ConvertImpl(
      const System<U>& other, std::true_type) {
    const auto& my_other = dynamic_cast<const S<U>&>(other);
    return std::make_unique<S<T>>(TransmogrifierTag{}, my_other);
  }

  // When Traits says not to transmogrify.
  template <typename T, typename U>
  static std::unique_ptr<System<T>> ConvertImpl(
      const System<U>& other, std::false_type) {
    return nullptr;
  }
};
}  // namespace detail

#endif  // !defined(DRAKE_DOXYGEN_CXX)

/// Factory method that creates a transmogrifier using the transmogrification
/// copy constructor, i.e., S<T>::S(const TransmogrifierTag&, const S<U>&).
///
/// @tparam S is the System type to transmogrify
template <template <typename> class S>
SystemTransmogrifier MakeDefaultSystemTransmogrifier() {
  return SystemTransmogrifier::Make<detail::TransmogrifierTagConverter<S>>();
}

}  // namespace systems
}  // namespace drake
