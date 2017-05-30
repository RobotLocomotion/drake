// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Mehdi Goli    Codeplay Software Ltd.
// Ralph Potter  Codeplay Software Ltd.
// Luke Iwanski  Codeplay Software Ltd.
// Contact: eigen@codeplay.com
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

// General include header of SYCL target for Tensor Module
#ifndef UNSUPPORTED_EIGEN_CXX11_SRC_TENSOR_TENSORSYCLFUNCTORS_H
#define UNSUPPORTED_EIGEN_CXX11_SRC_TENSOR_TENSORSYCLFUNCTORS_H

namespace Eigen {
namespace TensorSycl {
namespace internal {

/// ReductionFunctor
template < typename HostExpr, typename PlaceHolderExpr, typename FunctorExpr, typename Tuple_of_Acc, typename Dims, typename Op, typename Index> class ReductionFunctor {
 public:
  typedef cl::sycl::accessor<uint8_t, 1, cl::sycl::access::mode::discard_write, cl::sycl::access::target::global_buffer> write_accessor;
  ReductionFunctor(write_accessor output_accessor_, FunctorExpr functors_, Tuple_of_Acc tuple_of_accessors_,Dims dims_, Op functor_, Index range_)
  :output_accessor(output_accessor_), functors(functors_), tuple_of_accessors(tuple_of_accessors_), dims(dims_), functor(functor_), range(range_) {}
  void operator()(cl::sycl::nd_item<1> itemID) {

    typedef typename ConvertToDeviceExpression<const HostExpr>::Type DevExpr;
    auto device_expr = createDeviceExpression<DevExpr, PlaceHolderExpr>(functors, tuple_of_accessors);
    /// reduction cannot be captured automatically through our device conversion recursion. The reason is that reduction has two behaviour
    /// the first behaviour is when it is used as a root to lauch the sub-kernel. The second one is when it is treated as a leafnode to pass the
    /// calculated result to its parent kernel. While the latter is automatically detected through our device expression generator. The former is created here.
    const auto device_self_expr= Eigen::TensorReductionOp<Op, Dims, decltype(device_expr.expr) ,MakeGlobalPointer>(device_expr.expr, dims, functor);
    /// This is the evaluator for device_self_expr. This is exactly similar to the self which has been passed to run function. The difference is
    /// the device_evaluator is detectable and recognisable on the device.
    typedef Eigen::TensorEvaluator<decltype(device_self_expr), Eigen::DefaultDevice> DeviceSelf;
    auto device_self_evaluator = Eigen::TensorEvaluator<decltype(device_self_expr), Eigen::DefaultDevice>(device_self_expr, Eigen::DefaultDevice());
    auto output_accessor_ptr =ConvertToActualTypeSycl(typename DeviceSelf::CoeffReturnType, output_accessor);
    /// const cast added as a naive solution to solve the qualifier drop error
    auto globalid=static_cast<Index>(itemID.get_global_linear_id());
    if (globalid< range) {
      typename DeviceSelf::CoeffReturnType accum = functor.initialize();
      Eigen::internal::GenericDimReducer<DeviceSelf::NumReducedDims-1, DeviceSelf, Op>::reduce(device_self_evaluator, device_self_evaluator.firstInput(static_cast<typename DevExpr::Index>(globalid)),const_cast<Op&>(functor), &accum);
      functor.finalize(accum);
      output_accessor_ptr[globalid]= accum;
    }
  }
 private:
  write_accessor output_accessor;
  FunctorExpr functors;
  Tuple_of_Acc tuple_of_accessors;
  Dims dims;
  Op functor;
  Index range;
};


}
}
}
#endif  // UNSUPPORTED_EIGEN_CXX11_SRC_TENSOR_TENSORSYCLFUNCTORS_H
