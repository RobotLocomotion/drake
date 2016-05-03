#ifndef DRAKE_CONVERSIONS_H_H
#define DRAKE_CONVERSIONS_H_H

#include <Eigen/Core>
#include <unsupported/Eigen/AutoDiff>
#include <utility>
#include <type_traits>

namespace Drake {

namespace internal {
template <typename T1, typename T2>
struct PromoteDetail {

};

template <>
struct PromoteDetail<double, double> {
  typedef double type;
};

template <typename Derived>
struct PromoteDetail<double, Eigen::AutoDiffScalar<Derived>> {
  typedef Eigen::AutoDiffScalar<Derived> type;
};

template <typename Derived>
struct PromoteDetail<Eigen::AutoDiffScalar<Derived>, double> {
  typedef Eigen::AutoDiffScalar<Derived> type;
};

template <typename Derived>
struct PromoteDetail<Eigen::AutoDiffScalar<Derived>, Eigen::AutoDiffScalar<Derived>> {
  typedef Eigen::AutoDiffScalar<Derived> type;
};
}

template <typename T1, typename T2>
using Promote = typename internal::PromoteDetail<typename std::remove_cv<typename std::remove_reference<T1>::type>::type, typename std::remove_cv<typename std::remove_reference<T2>::type>::type>::type;

template <bool B, class T = void>
using enable_if_t = typename std::enable_if<B, T>::type;

template <typename To>
struct Convert {
  template <typename From>
  To operator()(const From& from) {
    return To(from);
  }

  To&& operator()(To&& from) {
    return std::forward<To>(from);
  }
};

template <typename ToScalar>
struct ConvertMatrix {
  template <typename T>
  struct To {
    typedef decltype(std::declval<typename std::remove_cv<typename std::remove_reference<T>::type>::type>().template cast<ToScalar>()) type;
  };

  // version for ToScalar != FromScalar
  template <typename T>
  const Drake::enable_if_t<!std::is_same<ToScalar, typename std::remove_reference<T>::type::Scalar>::value, typename To<T>::type> operator()(T &&from) {
    return std::forward<T>(from).template cast<ToScalar>();
  }

  // version for ToScalar == FromScalar
  template <typename T>
  const Drake::enable_if_t<std::is_same<ToScalar, typename std::remove_reference<T>::type::Scalar>::value, T&&> operator()(T &&from) {
    return std::forward<T>(from);
  }
};
}

#endif //DRAKE_CONVERSIONS_H_H
