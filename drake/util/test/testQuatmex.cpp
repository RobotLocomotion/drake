#include <tuple>
#include "drake/util/drakeGeometryUtil.h"
#include "drake/util/drakeGradientUtil.h"
#include "mex.h"
using namespace Eigen;
using namespace std;

namespace drake {
  namespace internal {
    template <typename Head, typename ...Tail>
    struct TotalSizeAtCompileTime {
      static constexpr int eval() {
        return Head::SizeAtCompileTime == Eigen::Dynamic || TotalSizeAtCompileTime<Tail...>::eval() == Eigen::Dynamic ?
               Eigen::Dynamic :
               Head::SizeAtCompileTime + TotalSizeAtCompileTime<Tail...>::eval();
      }
    };

    template <typename Head>
    struct TotalSizeAtCompileTime<Head> {
      static constexpr int eval() {
        return Head::SizeAtCompileTime;
      }
    };
  }

  template < typename... T>
  std::tuple<const T &...> ctie(const T &... args) {
    return std::tie(args...);
  }

  template <typename ...Args>
  constexpr int totalSizeAtCompileTime() {
    return internal::TotalSizeAtCompileTime<Args...>::eval();
  }

  DenseIndex totalSizeAtRunTime() {
    return 0;
  }

  template <typename Head, typename ...Tail>
  DenseIndex totalSizeAtRunTime(const Eigen::MatrixBase<Head>& head, const Tail&... tail) {
    return head.size() + totalSizeAtRunTime(tail...);
  }

  template <typename Derived, int Nq>
  using GradientType = Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::Matrix<typename Derived::Scalar, Nq, 1> >, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime >;

  template <int Nq, typename Derived>
  DenseIndex initializeAutoDiff(GradientType<Derived, Nq>& ret, const Eigen::MatrixBase<Derived> &mat, DenseIndex num_derivatives, DenseIndex deriv_num_start) {
    using ADScalar = typename GradientType<Derived, Nq>::Scalar;
    ret.resize(mat.rows(), mat.cols());
    DenseIndex deriv_num = deriv_num_start;
    for (int i = 0; i < mat.size(); i++) {
      ret(i) = ADScalar(mat(i), num_derivatives, deriv_num++);
    }
    deriv_num_start += mat.size();
    return deriv_num_start;
  }


  namespace internal {
    template <int Index>
    struct InitializeAutoDiffTuples {
      template <typename ...AutoDiffTypes, typename ...ValueTypes>
      static DenseIndex eval(tuple<AutoDiffTypes...>& ret, const tuple<ValueTypes...>& mat, DenseIndex num_derivatives, DenseIndex deriv_num_start) {
        deriv_num_start += initializeAutoDiff(std::get<Index>(ret), std::get<Index>(mat), num_derivatives, deriv_num_start);
        return InitializeAutoDiffTuples<Index - 1>::eval(ret, mat, num_derivatives, deriv_num_start);
      }
    };

    template <>
    struct InitializeAutoDiffTuples<-1> {
      template<typename ...AutoDiffTypes, typename ...ValueTypes>
      static DenseIndex eval(const tuple<AutoDiffTypes...> &ret, const tuple<ValueTypes...> &mat, DenseIndex num_derivatives, DenseIndex deriv_num_start) {
        return deriv_num_start;
      }
    };
  }

  template <typename ...Args>
  using InitializeAutoDiffReturnType = std::tuple<GradientType<Args, totalSizeAtCompileTime<Args>()>...>;

  template <typename Head, typename ...Tail>
  InitializeAutoDiffReturnType<Head, Tail...> initializeAutoDiffArgs(const Head& arg0, const Tail&... args) {
    DenseIndex dynamic_num_derivs = drake::totalSizeAtRunTime(args...);
    InitializeAutoDiffReturnType<Head, Tail...> ret;
    auto values = make_tuple(arg0, args...);
    internal::InitializeAutoDiffTuples<sizeof...(args)>::eval(ret, values, dynamic_num_derivs, 0);
    return ret;
  }

  // ideally: from tuple of rvalue references to arguments (constructed using forward_as_tuple) to tuple of autodiff matrices, properly initialized
}



void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {

  if (nrhs != 5) {
    mexErrMsgIdAndTxt("Drake:testQuatmex:BadInputs", "Usage [r,dr,e,ed,quat,dquat,q3,dq3,w,dw] = testQuatmex(q1,q2,axis,u,v)");
  }
  Vector4d q1;
  Vector4d q2;
  memcpy(q1.data(), mxGetPr(prhs[0]), sizeof(double) * 4);
  memcpy(q2.data(), mxGetPr(prhs[1]), sizeof(double) * 4);

  Vector3d axis;
  memcpy(axis.data(), mxGetPr(prhs[2]), sizeof(double) * 3);

  Vector3d u,v;
  memcpy(u.data(),mxGetPr(prhs[3]),sizeof(double)*3);
  memcpy(v.data(),mxGetPr(prhs[4]),sizeof(double)*3);

  auto bla = drake::initializeAutoDiffArgs(q1, q2);

//  {
//    constexpr int static_num_derivs = drake::totalSizeAtCompileTime<decltype(q1), decltype(q2)>();
//    DenseIndex dynamic_num_derivs = drake::totalSizeAtRunTime(q1, q2);
//
//    DenseIndex deriv_num = 0;
//    auto q1_autodiff = drake::initializeAutoDiff<static_num_derivs>(q1, dynamic_num_derivs, deriv_num);
//    auto q2_autodiff = drake::initializeAutoDiff<static_num_derivs>(q2, dynamic_num_derivs, deriv_num);
//
//    auto r_autodiff = quatDiff(q1_autodiff, q2_autodiff);
//    auto r = autoDiffToValueMatrix(r_autodiff);
//    auto dr = autoDiffToGradientMatrix(r_autodiff);
//
//    plhs[0] = mxCreateDoubleMatrix(4, 1, mxREAL);
//    plhs[1] = mxCreateDoubleMatrix(4, 8, mxREAL);
//    memcpy(mxGetPr(plhs[0]), r.data(), sizeof(double) * 4);
//    memcpy(mxGetPr(plhs[1]), dr.data(), sizeof(double) * 4 * 8);
//  }
//
//  {
//    constexpr int static_num_derivs = drake::totalSizeAtCompileTime<decltype(q1), decltype(q2), decltype(axis)>();
//    DenseIndex dynamic_num_derivs = drake::totalSizeAtRunTime(q1, q2, axis);
//    DenseIndex deriv_num = 0;
//    auto q1_autodiff = drake::initializeAutoDiff<static_num_derivs>(q1, dynamic_num_derivs, deriv_num);
//    auto q2_autodiff = drake::initializeAutoDiff<static_num_derivs>(q2, dynamic_num_derivs, deriv_num);
//    auto axis_autodiff = drake::initializeAutoDiff<static_num_derivs>(axis, dynamic_num_derivs, deriv_num);
//    auto e_autodiff = quatDiffAxisInvar(q1_autodiff, q2_autodiff, axis_autodiff);
//    auto e = e_autodiff.value();
//    auto de = e_autodiff.derivatives().transpose().eval();
//
//    plhs[2] = mxCreateDoubleScalar(e);
//    plhs[3] = mxCreateDoubleMatrix(1, 11, mxREAL);
//    memcpy(mxGetPr(plhs[3]), de.data(), sizeof(double) * 11);
//  }
//
//  {
//    constexpr int static_num_derivs = drake::totalSizeAtCompileTime<decltype(q1), decltype(q2)>();
//    DenseIndex dynamic_num_derivs = drake::totalSizeAtRunTime(q1, q2);
//    DenseIndex deriv_num = 0;
//    auto q1_autodiff = drake::initializeAutoDiff<static_num_derivs>(q1, dynamic_num_derivs, deriv_num);
//    auto q2_autodiff = drake::initializeAutoDiff<static_num_derivs>(q2, dynamic_num_derivs, deriv_num);
//
//    auto q3_autodiff = quatProduct(q1_autodiff, q2_autodiff);
//    auto q3 = autoDiffToValueMatrix(q3_autodiff);
//    auto dq3 = autoDiffToGradientMatrix(q3_autodiff);
//
//    plhs[4] = mxCreateDoubleMatrix(4, 1, mxREAL);
//    plhs[5] = mxCreateDoubleMatrix(4, 8, mxREAL);
//    memcpy(mxGetPr(plhs[4]), q3.data(), sizeof(double) * 4);
//    memcpy(mxGetPr(plhs[5]), dq3.data(), sizeof(double) * 4 * 8);
//  }
//
//  {
//    constexpr int static_num_derivs = drake::totalSizeAtCompileTime<decltype(q1), decltype(u)>();
//    DenseIndex dynamic_num_derivs = drake::totalSizeAtRunTime(q1, u);
//    DenseIndex deriv_num = 0;
//    auto q1_autodiff = drake::initializeAutoDiff<static_num_derivs>(q1, dynamic_num_derivs, deriv_num);
//    auto u_autodiff = drake::initializeAutoDiff<static_num_derivs>(u, dynamic_num_derivs, deriv_num);
//
//    auto w_autodiff = quatRotateVec(q1_autodiff, u_autodiff);
//    auto w = autoDiffToValueMatrix(w_autodiff);
//    auto dw = autoDiffToGradientMatrix(w_autodiff);
//
//    plhs[6] = mxCreateDoubleMatrix(3, 1, mxREAL);
//    plhs[7] = mxCreateDoubleMatrix(3, 7, mxREAL);
//    memcpy(mxGetPr(plhs[6]), w.data(), sizeof(double) * 3);
//    memcpy(mxGetPr(plhs[7]), dw.data(), sizeof(double) * 3 * 7);
//  }
}
