//
// Created by Twan Koolen on 4/27/16.
//

#ifndef DRAKE_CONVERSIONS_H_H
#define DRAKE_CONVERSIONS_H_H

#include <Eigen/Core>

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

  const To& operator()(const To& from) {
    return from;
  }
};

template <typename ToScalar>
struct ConvertMatrix {
  template <typename DerivedFrom>
  using To = Eigen::Matrix<ToScalar, DerivedFrom::RowsAtCompileTime, DerivedFrom::ColsAtCompileTime, 0, DerivedFrom::MaxRowsAtCompileTime, DerivedFrom::MaxColsAtCompileTime>;

  // version for ToScalar != FromScalar
  template <typename DerivedFrom>
  const Drake::enable_if_t<!std::is_same<ToScalar, typename DerivedFrom::Scalar>::value, To<DerivedFrom>> operator()(const Eigen::MatrixBase<DerivedFrom> &from) {
    return from.template cast<ToScalar>();
  }

  // version for ToScalar == FromScalar
  template <typename DerivedFrom>
  const Drake::enable_if_t<std::is_same<ToScalar, typename DerivedFrom::Scalar>::value, const Eigen::MatrixBase<DerivedFrom>&> operator()(const Eigen::MatrixBase<DerivedFrom> &from) {
    return from;
  }
};
}

#endif //DRAKE_CONVERSIONS_H_H
