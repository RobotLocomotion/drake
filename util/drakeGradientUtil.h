#ifndef DRAKEGRADIENTUTIL_H_
#define DRAKEGRADIENTUTIL_H_

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

template <typename Derived>
void transposeGrad(const Eigen::MatrixBase<Derived>& dX, int rows_X, Eigen::MatrixXd& dX_transpose)
{
//  Eigen::MatrixXd dX_transpose(dX.rows(), dX.cols());
  dX_transpose.resize(dX.rows(), dX.cols());
  int numel = dX.rows();
  int index = 0;
  for (int i = 0; i < numel; i++) {
    dX_transpose.row(i) = dX.row(index);
    index += rows_X;
    if (index >= numel) {
      index = (index % numel) + 1;
    }
  }
}



#endif /* DRAKEGRADIENTUTIL_H_ */
