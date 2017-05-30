// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Mehdi Goli    Codeplay Software Ltd.
// Ralph Potter  Codeplay Software Ltd.
// Luke Iwanski  Codeplay Software Ltd.
// Contact: <eigen@codeplay.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

/*****************************************************************
 * TensorSyclextractFunctors.h
 *
 * \brief:
 *  Used to extract all the functors allocated to each node of the expression
*tree.
 *
*****************************************************************/

#ifndef UNSUPPORTED_EIGEN_CXX11_SRC_TENSOR_TENSORSYCL_EXTRACT_FUNCTORS_HPP
#define UNSUPPORTED_EIGEN_CXX11_SRC_TENSOR_TENSORSYCL_EXTRACT_FUNCTORS_HPP

namespace Eigen {
namespace TensorSycl {
namespace internal {
/// \struct FunctorExtractor:  This struct is used to extract the functors
/// constructed on
/// the host-side, to pack them and reuse them in reconstruction of the
/// expression on the device.
/// We have to do that as in Eigen the functors are not stateless so we cannot
/// re-instantiate them on the device.
/// We have to pass instantiated functors to the device.
// This struct is used for leafNode (TensorMap) and nodes behaving like leafNode (TensorForcedEval).
template <typename Evaluator> struct FunctorExtractor{
  typedef typename Evaluator::Dimensions Dimensions;
  const Dimensions m_dimensions;
  EIGEN_STRONG_INLINE const Dimensions& dimensions() const { return m_dimensions; }
  FunctorExtractor(const Evaluator& expr)
  : m_dimensions(expr.dimensions()) {}

};

#define SYCLEXTRTENSORMAPFIXEDSIZE(CVQual)\
template <typename Scalar_, typename Dimensions_, int Options_2, typename IndexType, int Options_, template <class> class MakePointer_, typename Dev>\
struct FunctorExtractor< TensorEvaluator <CVQual TensorMap<TensorFixedSize<Scalar_, Dimensions_, Options_2, IndexType>, Options_, MakePointer_> , Dev> >{\
FunctorExtractor(const TensorEvaluator <CVQual TensorMap<TensorFixedSize<Scalar_, Dimensions_, Options_2, IndexType>, Options_, MakePointer_> , Dev>& ){}\
};

SYCLEXTRTENSORMAPFIXEDSIZE(const)
SYCLEXTRTENSORMAPFIXEDSIZE()
#undef SYCLEXTRTENSORMAPFIXEDSIZE

/// specialisation of the \ref FunctorExtractor struct when the node type is
/// TensorCwiseNullaryOp,  TensorCwiseUnaryOp, and  TensorBroadcastingOp
#define SYCLEXTRFUNCUNARY(CVQual)\
template <template <class, class> class UnaryCategory, typename OP, typename RHSExpr, typename Dev>\
struct FunctorExtractor<TensorEvaluator<CVQual UnaryCategory<OP, RHSExpr>, Dev> > {\
  FunctorExtractor<TensorEvaluator<RHSExpr, Dev> > rhsExpr;\
  OP func;\
  FunctorExtractor(const TensorEvaluator<CVQual UnaryCategory<OP, RHSExpr>, Dev>& expr)\
  : rhsExpr(expr.impl()), func(expr.functor()) {}\
};

SYCLEXTRFUNCUNARY(const)
SYCLEXTRFUNCUNARY()
#undef SYCLEXTRFUNCUNARY

/// specialisation of the \ref FunctorExtractor struct when the node type is
/// TensorCwiseBinaryOp
#define SYCLEXTRFUNCBIINARY(CVQual)\
template <template<class, class, class> class BinaryCategory, typename OP, typename LHSExpr, typename RHSExpr, typename Dev>\
struct FunctorExtractor<TensorEvaluator<CVQual BinaryCategory<OP, LHSExpr, RHSExpr>, Dev> > {\
  FunctorExtractor<TensorEvaluator<LHSExpr, Dev> > lhsExpr;\
  FunctorExtractor<TensorEvaluator<RHSExpr, Dev> > rhsExpr;\
  OP func;\
  FunctorExtractor(const TensorEvaluator<CVQual BinaryCategory<OP, LHSExpr, RHSExpr>, Dev>& expr)\
  : lhsExpr(expr.left_impl()),rhsExpr(expr.right_impl()),func(expr.functor()) {}\
};

SYCLEXTRFUNCBIINARY(const)
SYCLEXTRFUNCBIINARY()
#undef SYCLEXTRFUNCBIINARY

/// specialisation of the \ref FunctorExtractor struct when the node type is TensorCwiseTernaryOp
#define SYCLEXTRFUNCTERNARY(CVQual)\
template <template <class, class, class, class> class TernaryCategory, typename OP, typename Arg1Expr, typename Arg2Expr, typename Arg3Expr,typename Dev>\
struct FunctorExtractor<TensorEvaluator<CVQual TernaryCategory<OP, Arg1Expr, Arg2Expr, Arg3Expr>, Dev> > {\
  FunctorExtractor<TensorEvaluator<Arg1Expr, Dev> > arg1Expr;\
  FunctorExtractor<TensorEvaluator<Arg2Expr, Dev> > arg2Expr;\
  FunctorExtractor<TensorEvaluator<Arg3Expr, Dev> > arg3Expr;\
  OP func;\
  FunctorExtractor(const TensorEvaluator<CVQual TernaryCategory<OP, Arg1Expr, Arg2Expr, Arg3Expr>, Dev>& expr)\
  : arg1Expr(expr.arg1Impl()), arg2Expr(expr.arg2Impl()), arg3Expr(expr.arg3Impl()), func(expr.functor()) {}\
};

SYCLEXTRFUNCTERNARY(const)
SYCLEXTRFUNCTERNARY()
#undef SYCLEXTRFUNCTERNARY

/// specialisation of the \ref FunctorExtractor struct when the node type is
/// TensorCwiseSelectOp. This is an specialisation without OP so it has to be separated.
#define SYCLEXTRFUNCSELECTOP(CVQual)\
template <typename IfExpr, typename ThenExpr, typename ElseExpr, typename Dev>\
struct FunctorExtractor< TensorEvaluator<CVQual TensorSelectOp<IfExpr, ThenExpr, ElseExpr>, Dev> > {\
  FunctorExtractor<TensorEvaluator<IfExpr, Dev> > ifExpr;\
  FunctorExtractor<TensorEvaluator<ThenExpr, Dev> > thenExpr;\
  FunctorExtractor<TensorEvaluator<ElseExpr, Dev> > elseExpr;\
  FunctorExtractor(const TensorEvaluator<CVQual TensorSelectOp<IfExpr, ThenExpr, ElseExpr>, Dev>& expr)\
  : ifExpr(expr.cond_impl()), thenExpr(expr.then_impl()), elseExpr(expr.else_impl()) {}\
};

SYCLEXTRFUNCSELECTOP(const)
SYCLEXTRFUNCSELECTOP()
#undef SYCLEXTRFUNCSELECTOP

/// specialisation of the \ref FunctorExtractor struct when the node type is
/// const TensorAssignOp. This is an specialisation without OP so it has to be separated.
#define SYCLEXTRFUNCASSIGNOP(CVQual)\
template <typename LHSExpr, typename RHSExpr, typename Dev>\
struct FunctorExtractor<TensorEvaluator<CVQual TensorAssignOp<LHSExpr, RHSExpr>, Dev> > {\
  FunctorExtractor<TensorEvaluator<LHSExpr, Dev> > lhsExpr;\
  FunctorExtractor<TensorEvaluator<RHSExpr, Dev> > rhsExpr;\
  FunctorExtractor(const TensorEvaluator<CVQual TensorAssignOp<LHSExpr, RHSExpr>, Dev>& expr)\
  : lhsExpr(expr.left_impl()), rhsExpr(expr.right_impl()) {}\
};
SYCLEXTRFUNCASSIGNOP(const)
SYCLEXTRFUNCASSIGNOP()
#undef SYCLEXTRFUNCASSIGNOP

/// specialisation of the \ref FunctorExtractor struct when the node type is
/// TensorEvalToOp, This is an specialisation without OP so it has to be separated.
#define SYCLEXTRFUNCEVALTOOP(CVQual)\
template <typename RHSExpr, typename Dev>\
struct FunctorExtractor<TensorEvaluator<CVQual TensorEvalToOp<RHSExpr>, Dev> > {\
  FunctorExtractor<TensorEvaluator<RHSExpr, Dev> > rhsExpr;\
  FunctorExtractor(const TensorEvaluator<CVQual TensorEvalToOp<RHSExpr>, Dev>& expr)\
  : rhsExpr(expr.impl()) {}\
};

SYCLEXTRFUNCEVALTOOP(const)
SYCLEXTRFUNCEVALTOOP()
#undef SYCLEXTRFUNCEVALTOOP

template<typename Dim, size_t NumOutputDim> struct DimConstr {
template<typename InDim>
  static EIGEN_STRONG_INLINE Dim getDim(InDim dims ) {return dims;}
};

template<typename Dim> struct DimConstr<Dim, 0> {
  template<typename InDim>
    static EIGEN_STRONG_INLINE Dim getDim(InDim dims ) {return Dim(static_cast<Dim>(dims.TotalSize()));}
};

#define SYCLEXTRFUNCREDUCTIONOP(CVQual)\
template<typename Op, typename Dims, typename ArgType, template <class> class MakePointer_, typename Device>\
struct FunctorExtractor<TensorEvaluator<CVQual TensorReductionOp<Op, Dims, ArgType, MakePointer_>, Device>>{\
  typedef TensorEvaluator<CVQual TensorReductionOp<Op, Dims, ArgType, MakePointer_>, Device> Evaluator;\
  typedef typename Eigen::internal::conditional<Evaluator::NumOutputDims==0, DSizes<typename Evaluator::Index, 1>, typename Evaluator::Dimensions >::type Dimensions;\
  const Dimensions m_dimensions;\
  EIGEN_STRONG_INLINE const Dimensions& dimensions() const { return m_dimensions; }\
  FunctorExtractor(const TensorEvaluator<CVQual TensorReductionOp<Op, Dims, ArgType, MakePointer_>, Device>& expr)\
  : m_dimensions(DimConstr<Dimensions, Evaluator::NumOutputDims>::getDim(expr.dimensions())) {}\
};


SYCLEXTRFUNCREDUCTIONOP(const)
SYCLEXTRFUNCREDUCTIONOP()
#undef SYCLEXTRFUNCREDUCTIONOP

/// specialisation of the \ref FunctorExtractor struct when the node type is
/// const TensorSlicingOp. This is an specialisation without OP so it has to be separated.
#define SYCLEXTRFUNCTSLICEOP(CVQual)\
template <typename StartIndices, typename Sizes, typename XprType, typename Dev>\
struct FunctorExtractor<TensorEvaluator<CVQual TensorSlicingOp<StartIndices, Sizes, XprType>, Dev> > {\
  FunctorExtractor<TensorEvaluator<XprType, Dev> > xprExpr;\
  const StartIndices m_offsets;\
  const Sizes m_dimensions;\
  FunctorExtractor(const TensorEvaluator<CVQual  TensorSlicingOp<StartIndices, Sizes, XprType>, Dev>& expr)\
  : xprExpr(expr.impl()), m_offsets(expr.startIndices()), m_dimensions(expr.dimensions()) {}\
  EIGEN_STRONG_INLINE const StartIndices& startIndices() const {return m_offsets;}\
  EIGEN_STRONG_INLINE const Sizes& dimensions() const {return m_dimensions;}\
};

SYCLEXTRFUNCTSLICEOP(const)
SYCLEXTRFUNCTSLICEOP()
#undef SYCLEXTRFUNCTSLICEOP

#define SYCLEXTRFUNCTSLICESTRIDEOP(CVQual)\
template<typename StartIndices, typename StopIndices, typename Strides, typename XprType, typename Dev>\
struct FunctorExtractor<TensorEvaluator<CVQual TensorStridingSlicingOp<StartIndices, StopIndices, Strides, XprType>, Dev> >{\
  FunctorExtractor<TensorEvaluator<XprType, Dev> > xprExpr;\
  const StartIndices m_startIndices;\
  const StopIndices m_stopIndices;\
  const Strides m_strides;\
  FunctorExtractor(const TensorEvaluator<CVQual  TensorStridingSlicingOp<StartIndices, StopIndices,Strides, XprType>, Dev>& expr)\
  : xprExpr(expr.impl()), m_startIndices(expr.exprStartIndices()), m_stopIndices(expr.exprStopIndices()), m_strides(expr.strides()) {}\
  EIGEN_STRONG_INLINE  const StartIndices& startIndices() const { return m_startIndices; }\
  EIGEN_STRONG_INLINE  const StartIndices& stopIndices() const { return m_stopIndices; }\
  EIGEN_STRONG_INLINE  const StartIndices& strides() const { return m_strides; }\
};

SYCLEXTRFUNCTSLICESTRIDEOP(const)
SYCLEXTRFUNCTSLICESTRIDEOP()
#undef SYCLEXTRFUNCTSLICESTRIDEOP

// Had to separate reshapeOP otherwise it will be mistaken by UnaryCategory
#define SYCLRESHAPEANDSHUFFLEOPFUNCEXT(OPEXPR, FUNCCALL, CVQual)\
template<typename Param, typename XprType, typename Dev>\
struct FunctorExtractor<Eigen::TensorEvaluator<CVQual Eigen::OPEXPR<Param, XprType>, Dev> > {\
  FunctorExtractor<Eigen::TensorEvaluator<XprType, Dev> > xprExpr;\
  const Param m_param;\
  EIGEN_STRONG_INLINE const Param& param() const { return m_param; }\
  FunctorExtractor(const Eigen::TensorEvaluator<CVQual Eigen::OPEXPR<Param, XprType>, Dev>& expr)\
  : xprExpr(expr.impl()), m_param(expr.FUNCCALL) {}\
};

SYCLRESHAPEANDSHUFFLEOPFUNCEXT(TensorReshapingOp, dimensions(), const)
SYCLRESHAPEANDSHUFFLEOPFUNCEXT(TensorReshapingOp, dimensions(), )

SYCLRESHAPEANDSHUFFLEOPFUNCEXT(TensorShufflingOp, shufflePermutation(), const)
SYCLRESHAPEANDSHUFFLEOPFUNCEXT(TensorShufflingOp, shufflePermutation(), )
#undef SYCLRESHAPEANDSHUFFLEOPFUNCEXT

// Had to separate reshapeOP otherwise it will be mistaken by UnaryCategory
#define PADDINGOPFUNCEXT(OPEXPR, FUNCCALL, SCALARFUNCCALL, CVQual)\
template<typename Param, typename XprType, typename Dev>\
struct FunctorExtractor<Eigen::TensorEvaluator<CVQual Eigen::OPEXPR<Param, XprType>, Dev> > {\
  FunctorExtractor<Eigen::TensorEvaluator<XprType, Dev> > xprExpr;\
  const Param m_param;\
  typedef typename Eigen::TensorEvaluator<CVQual Eigen::OPEXPR<Param, XprType>, Dev>::Scalar Scalar;\
  const Scalar m_scalar_param;\
  EIGEN_STRONG_INLINE const Param& param() const { return m_param; }\
  EIGEN_STRONG_INLINE const Scalar& scalar_param() const { return m_scalar_param; }\
  FunctorExtractor(const Eigen::TensorEvaluator<CVQual Eigen::OPEXPR<Param, XprType>, Dev>& expr)\
  : xprExpr(expr.impl()), m_param(expr.FUNCCALL), m_scalar_param(expr.SCALARFUNCCALL)  {}\
};

PADDINGOPFUNCEXT(TensorPaddingOp, padding(), padding_value(), const)
PADDINGOPFUNCEXT(TensorPaddingOp, padding(), padding_value(), )
#undef PADDINGOPFUNCEXT

/// template deduction function for FunctorExtractor
template <typename Evaluator>
auto inline extractFunctors(const Evaluator& evaluator)-> FunctorExtractor<Evaluator> {
  return FunctorExtractor<Evaluator>(evaluator);
}
}  // namespace internal
}  // namespace TensorSycl
}  // namespace Eigen

#endif  // UNSUPPORTED_EIGEN_CXX11_SRC_TENSOR_TENSORSYCL_EXTRACT_FUNCTORS_HPP
