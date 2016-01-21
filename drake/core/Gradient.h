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

  /** initTaylorVecXd
   * @brief initializes the vector with x=val and dx=eye(numel(val))
   */
  template <typename DerivedA, typename DerivedB>
  void initTaylorVecXd(const Eigen::MatrixBase<DerivedA>& val, Eigen::MatrixBase<DerivedB>& auto_diff_vec) {
    Eigen::MatrixXd der = Eigen::MatrixXd::Identity(val.rows(),val.rows());
    for (int i=0; i<val.rows(); i++) {
      auto_diff_vec(i).value() = val(i);
      auto_diff_vec(i).derivatives() = der.col(i);
    }
  }

  template <typename Derived>
  TaylorVecXd initTaylorVecXd(const Eigen::MatrixBase<Derived>& val) {
    TaylorVecXd auto_diff_vec(val.rows());
    initTaylorVecXd(val, auto_diff_vec);
    return auto_diff_vec;
  }


  /** initTaylorVecXd
   * @brief initializes the vector with x=value and dx=gradient
   */
  template<typename Derived, typename DerivedGradient>
  void initTaylorVecXd(const Eigen::MatrixBase<Derived>& val, const Eigen::MatrixBase<DerivedGradient>& gradient, TaylorVecXd& auto_diff_vector)
  {
    typedef typename Eigen::MatrixBase<DerivedGradient>::Index Index;
    auto_diff_vector.resize(gradient.rows(),1);
    auto nx = gradient.cols();
    for (Index row = 0; row < auto_diff_vector.rows(); row++) {
      auto_diff_vector(row).value() = val(row);
      auto_diff_vector(row).derivatives().resize(nx,1);
      auto_diff_vector(row).derivatives() = gradient.row(row).transpose();
    }
  }
}

#endif //DRAKE_GRADIENT_H
