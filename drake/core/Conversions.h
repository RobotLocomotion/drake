//
// Created by Twan Koolen on 4/27/16.
//

#ifndef DRAKE_CONVERSIONS_H_H
#define DRAKE_CONVERSIONS_H_H

#include <Eigen/Core>
#include <utility>

namespace Drake {
template <typename T1, typename T2>
using Promote = decltype(std::declval<T1>() * std::declval<T2>());

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
  struct To {};

  template <typename Derived>
  struct To<Eigen::MatrixBase<Derived>> {
    typedef Eigen::Matrix<ToScalar, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime, 0, Derived::MaxRowsAtCompileTime, Derived::MaxColsAtCompileTime> type;
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

//  template <typename DerivedFrom>
//  const Drake::enable_if_t<std::is_same<ToScalar, typename DerivedFrom::Scalar>::value, Eigen::Block<DerivedFrom>> operator()(Eigen::Block<DerivedFrom> &&from) {
//    return from;
//  }
};
}

#endif //DRAKE_CONVERSIONS_H_H
