#ifndef DRAKE_GRADIENT_H
#define DRAKE_GRADIENT_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/AutoDiff>
#include "drake/util/drakeGradientUtil.h" // todo: pull the core tools into this file and zap the old gradient util.

namespace Drake {
  // todo: recursive template to get arbitrary gradient order

  // note: tried using template default values (e.g. Eigen::Dynamic), but they didn't seem to work on my mac clang
  template <int num_vars> using TaylorVard = Eigen::AutoDiffScalar< Eigen::Matrix<double,num_vars,1> >;
  template <int num_vars, int rows> using TaylorVecd = Eigen::Matrix< TaylorVard<num_vars>, rows, 1>;
  template <int num_vars, int rows, int cols> using TaylorMatd = Eigen::Matrix< TaylorVard<num_vars>, rows, cols>;

  typedef TaylorVard<Eigen::Dynamic> TaylorVarXd;
  typedef TaylorVecd<Eigen::Dynamic,Eigen::Dynamic> TaylorVecXd;
  typedef TaylorMatd<Eigen::Dynamic,Eigen::Dynamic,Eigen::Dynamic> TaylorMatXd;

  /** initTaylorVecXd
   * @brief initializes the vector with x=val and dx=eye(numel(val))
   */
  template <typename DerivedA, typename DerivedB>
  void initTaylorVecXd(const Eigen::MatrixBase<DerivedA>& val, Eigen::MatrixBase<DerivedB>& auto_diff_vec) {
    for (int i=0; i<val.size(); i++) {
      auto_diff_vec(i) = Eigen::AutoDiffScalar<typename DerivedB::Scalar::DerType>(val(i),val.size(),i);
    }
  }

  template <typename Derived>
  TaylorVecXd initTaylorVecXd(const Eigen::MatrixBase<Derived>& val) {
    TaylorVecXd auto_diff_vec(val.rows());
    initTaylorVecXd(val, auto_diff_vec);
    return auto_diff_vec;
  }

  /** constantTaylorVecXd
   * @brief initializes the vector with x=val and dx=zeros(numel(val))
   */
  template <typename DerivedA, typename DerivedB>
  void constantTaylorVecXd(const Eigen::MatrixBase<DerivedA>& val, Eigen::MatrixBase<DerivedB>& auto_diff_vec) {
    for (int i=0; i<val.size(); i++) {
      auto_diff_vec(i) = Eigen::AutoDiffScalar<typename DerivedB::Scalar::DerType>(val(i));
    }
  }

  template <typename Derived>
  TaylorVecXd constantTaylorVecXd(const Eigen::MatrixBase<Derived>& val) {
    TaylorVecXd auto_diff_vec(val.rows());
    constantTaylorVecXd(val, auto_diff_vec);
    return auto_diff_vec;
  }


  /** initTaylorVecXd
   * @brief initializes the vector with x=value and dx=gradient
   */
  template<typename Derived, typename DerivedGradient>
  void initTaylorVecXd(const Eigen::MatrixBase<Derived>& val, const Eigen::MatrixBase<DerivedGradient>& gradient, TaylorVecXd& auto_diff_vector)
  {
    auto_diff_vector.resize(gradient.size(),1);
    auto nx = gradient.cols();
    for (Eigen::Index row = 0; row < auto_diff_vector.size(); row++) {
      auto_diff_vector(row).value() = val(row);
      auto_diff_vector(row).derivatives().resize(nx,1);
      auto_diff_vector(row).derivatives() = gradient.row(row).transpose();
    }
  }
}

#endif //DRAKE_GRADIENT_H
