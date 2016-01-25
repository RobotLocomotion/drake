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

  /** \brief The appropriate AutoDiffScalar gradient type given the value type and the number of derivatives at compile time
   */
  template<typename Derived, int Nq>
  using AutoDiffMatrixType = Eigen::Matrix<TaylorVard<Nq>, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime, Derived::Options, Derived::MaxRowsAtCompileTime, Derived::MaxColsAtCompileTime>;


  /** \brief initializes an autodiff matrix given a matrix of values and gradient matrix
   * \param[in] val value matrix
   * \param[in] gradient gradient matrix; the derivatives of val(j) are stored in row j of the gradient matrix.
   * \param[out] autodiff_matrix matrix of AutoDiffScalars with the same size as
   */
  template<typename Derived, typename DerivedGradient, typename DerivedAutoDiff>
  void initializeAutoDiffGivenGradientMatrix(const Eigen::MatrixBase<Derived> &val, const Eigen::MatrixBase<DerivedGradient> &gradient, Eigen::MatrixBase<DerivedAutoDiff> &auto_diff_matrix)
  {
    static_assert(Derived::SizeAtCompileTime == DerivedGradient::RowsAtCompileTime, "gradient has wrong number of rows at compile time");
    assert(val.size() == gradient.rows() && "gradient has wrong number of rows at runtime");
    typedef AutoDiffMatrixType<Derived, DerivedGradient::ColsAtCompileTime> ExpectedAutoDiffType;
    static_assert(ExpectedAutoDiffType::RowsAtCompileTime == DerivedAutoDiff::RowsAtCompileTime, "auto diff matrix has wrong number of rows at compile time");
    static_assert(ExpectedAutoDiffType::ColsAtCompileTime == DerivedAutoDiff::ColsAtCompileTime, "auto diff matrix has wrong number of columns at compile time");
    static_assert(std::is_same<typename DerivedAutoDiff::Scalar, typename ExpectedAutoDiffType::Scalar>::value, "wrong auto diff scalar type");

    typedef typename Eigen::MatrixBase<DerivedGradient>::Index Index;
    auto_diff_matrix.resize(val.rows(), val.cols());
    auto num_derivs = gradient.cols();
    for (Index row = 0; row < auto_diff_matrix.size(); row++) {
      auto_diff_matrix(row).value() = val(row);
      auto_diff_matrix(row).derivatives().resize(num_derivs, 1);
      auto_diff_matrix(row).derivatives() = gradient.row(row).transpose();
    }
  }

  /** \brief initialize a single autodiff matrix given the corresponding value matrix.
   * @param[in] mat 'regular' matrix of values
   * @param[out] ret AutoDiff matrix
   * @param[in] num_derivatives the size of the derivatives vector @default the size of mat
   * @param[in] deriv_num_start starting index into derivative vector (i.e. element deriv_num_start in derivative vector corresponds to mat(0, 0)). @default 0
   */
  template<typename Derived, typename DerivedAutoDiff>
  void initializeAutoDiff(const Eigen::MatrixBase<Derived> &val, Eigen::MatrixBase<DerivedAutoDiff> &auto_diff_matrix, Eigen::DenseIndex num_derivatives = Eigen::Dynamic, Eigen::DenseIndex deriv_num_start = 0) {
    using ADScalar = typename DerivedAutoDiff::Scalar;
    static_assert(Derived::RowsAtCompileTime == DerivedAutoDiff::RowsAtCompileTime, "auto diff matrix has wrong number of rows at compile time");
    static_assert(Derived::ColsAtCompileTime == DerivedAutoDiff::ColsAtCompileTime, "auto diff matrix has wrong number of columns at compile time");

    if (num_derivatives == Eigen::Dynamic)
      num_derivatives = val.size();

    auto_diff_matrix.resize(val.rows(), val.cols());
    Eigen::DenseIndex deriv_num = deriv_num_start;
    for (Eigen::DenseIndex i = 0; i < val.size(); i++) {
      auto_diff_matrix(i) = ADScalar(val(i), num_derivatives, deriv_num++);
    }
  }

  /** \brief initialize a single autodiff matrix given the corresponding value matrix.
   * @param[in] mat 'regular' matrix of values
   * @param[in] num_derivatives the size of the derivatives vector @default the size of mat
   * @param[in] deriv_num_start starting index into derivative vector (i.e. element deriv_num_start in derivative vector corresponds to mat(0, 0)). @default 0
   * @return AutoDiff matrix
   */
  template<int Nq = Eigen::Dynamic, typename Derived>
  AutoDiffMatrixType<Derived, Nq> initializeAutoDiff(const Eigen::MatrixBase<Derived> &mat, Eigen::DenseIndex num_derivatives = -1, Eigen::DenseIndex deriv_num_start = 0) {
    if (num_derivatives == -1)
      num_derivatives = mat.size();

    AutoDiffMatrixType<Derived, Nq> ret(mat.rows(), mat.cols());
    initializeAutoDiff(mat, ret, num_derivatives, deriv_num_start);
    return ret;
  }

  // TODO begin: move to different location?
  namespace internal {
    /** \brief helper for totalSizeAtCompileTime function (recursive)
     */
    template<typename Head, typename ...Tail>
    struct TotalSizeAtCompileTime {
      static constexpr int eval() {
        return Head::SizeAtCompileTime == Eigen::Dynamic || TotalSizeAtCompileTime<Tail...>::eval() == Eigen::Dynamic ?
               Eigen::Dynamic :
               Head::SizeAtCompileTime + TotalSizeAtCompileTime<Tail...>::eval();
      }
    };

    /** \brief helper for totalSizeAtCompileTime function (base case)
     */
    template<typename Head>
    struct TotalSizeAtCompileTime<Head> {
      static constexpr int eval() {
        return Head::SizeAtCompileTime;
      }
    };
  }

  /** \brief determine the total size at compile time of a number of arguments based on their SizeAtCompileTime static members
   */
  template<typename ...Args>
  constexpr int totalSizeAtCompileTime() {
    return internal::TotalSizeAtCompileTime<Args...>::eval();
  }

  /** \brief determine the total size at runtime of a number of arguments using their size() methods (base case).
   */
  constexpr Eigen::DenseIndex totalSizeAtRunTime() {
    return 0;
  }

  /** \brief determine the total size at runtime of a number of arguments using their size() methods (recursive)
   */
  template<typename Head, typename ...Tail>
  Eigen::DenseIndex totalSizeAtRunTime(const Eigen::MatrixBase<Head> &head, const Tail &... tail) {
    return head.size() + totalSizeAtRunTime(tail...);
  }
  // TODO end: move to different location?

  namespace internal {
    /** \brief helper for initializeAutoDiffArgs function (recursive)
     */
    template<size_t Index>
    struct InitializeAutoDiffArgsHelper {
      template<typename ...ValueTypes, typename ...AutoDiffTypes>
      static void run(const std::tuple<ValueTypes...> &values, std::tuple<AutoDiffTypes...> &auto_diffs, Eigen::DenseIndex num_derivatives, Eigen::DenseIndex deriv_num_start) {
        constexpr size_t tuple_index = sizeof...(AutoDiffTypes) - Index;
        const auto& value = std::get<tuple_index>(values);
        auto& auto_diff = std::get<tuple_index>(auto_diffs);
        auto_diff.resize(value.rows(), value.cols());
        initializeAutoDiff(value, auto_diff, num_derivatives, deriv_num_start);
        InitializeAutoDiffArgsHelper<Index - 1>::run(values, auto_diffs, num_derivatives, deriv_num_start + value.size());
      }
    };

    /** \brief helper for initializeAutoDiffArgs function (base case)
     */
    template<>
    struct InitializeAutoDiffArgsHelper<0> {
      template<typename ...ValueTypes, typename ...AutoDiffTypes>
      static void run(const std::tuple<ValueTypes...> &values, const std::tuple<AutoDiffTypes...> &auto_diffs, Eigen::DenseIndex num_derivatives, Eigen::DenseIndex deriv_num_start) {
        // empty
      }
    };
  }

  template<typename ...Args>
  std::tuple<AutoDiffMatrixType<Args, totalSizeAtCompileTime<Args...>()>...> initializeAutoDiffArgs(const Args &... args) {
    Eigen::DenseIndex dynamic_num_derivs = totalSizeAtRunTime(args...);
    std::tuple<AutoDiffMatrixType<Args, totalSizeAtCompileTime<Args...>()>...> ret(AutoDiffMatrixType<Args, totalSizeAtCompileTime<Args...>()>(args.rows(), args.cols())...);
    auto values = std::forward_as_tuple(args...);
    internal::InitializeAutoDiffArgsHelper<sizeof...(args)>::run(values, ret, dynamic_num_derivs, 0);
    return ret;
  }
}

#endif //DRAKE_GRADIENT_H
