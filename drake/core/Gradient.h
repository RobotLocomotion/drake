#ifndef DRAKE_GRADIENT_H
#define DRAKE_GRADIENT_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/AutoDiff>

namespace Drake {
  // todo: recursive template to get arbitrary gradient order

  // note: tried using template default values (e.g. Eigen::Dynamic), but they didn't seem to work on my mac clang
  template <int num_vars> using TaylorVard = Eigen::AutoDiffScalar< Eigen::Matrix<double,num_vars,1> >;
  template <int num_vars, int rows> using TaylorVecd = Eigen::Matrix< TaylorVard<num_vars>, rows, 1>;
  template <int num_vars, int rows, int cols> using TaylorMatd = Eigen::Matrix< TaylorVard<num_vars>, rows, cols>;

  typedef TaylorVard<Eigen::Dynamic> TaylorVarXd;
  typedef TaylorVecd<Eigen::Dynamic,Eigen::Dynamic> TaylorVecXd;
  typedef TaylorMatd<Eigen::Dynamic,Eigen::Dynamic,Eigen::Dynamic> TaylorMatXd;

  // initializes the vector with x=val and dx=eye(numel(val))
  template <typename Derived>
  TaylorVecXd initTaylorVecXd(const Eigen::MatrixBase<Derived>& val) {
    TaylorVecXd x(val.rows());
    Eigen::MatrixXd der = Eigen::MatrixXd::Identity(val.rows(),val.rows());
    for (int i=0; i<val.rows(); i++) {
      x(i).value() = val(i);
      x(i).derivatives() = der.col(i);
    }
    return x;
  }
}

#endif //DRAKE_GRADIENT_H
