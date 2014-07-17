#ifndef DRAKEGRADIENTUTIL_H_
#define DRAKEGRADIENTUTIL_H_

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

/*
 * Profile results: looks like return value optimization works; a version that sets a reference
 * instead of returning a value is just as fast and this is cleaner.
 */
template <typename Derived>
typename Derived::PlainObject transposeGrad(const Eigen::MatrixBase<Derived>& dX, int rows_X)
{
  typename Derived::PlainObject dX_transpose;
  int numel = dX.rows();
  int index = 0;
  for (int i = 0; i < numel; i++) {
    dX_transpose.row(i) = dX.row(index);
    index += rows_X;
    if (index >= numel) {
      index = (index % numel) + 1;
    }
  }
  return dX_transpose;
}


#endif /* DRAKEGRADIENTUTIL_H_ */
