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
 * TensorSyclExtractAccessor.h
 *
 * \brief:
 * ExtractAccessor takes Expression placeHolder expression and the tuple of sycl
 * buffers as an input. Using pre-order tree traversal, ExtractAccessor
 * recursively calls itself for its children in the expression tree. The
 * leaf node in the PlaceHolder expression is nothing but a container preserving
 * the order of the actual data in the tuple of sycl buffer. By invoking the
 * extract accessor for the PlaceHolder<N>, an accessor is created for the Nth
 * buffer in the tuple of buffers. This accessor is then added as an Nth
 * element in the tuple of accessors. In this case we preserve the order of data
 * in the expression tree.
 *
 * This is the specialisation of extract accessor method for different operation
 * type in the PlaceHolder expression.
 *
*****************************************************************/

#ifndef UNSUPPORTED_EIGEN_CXX11_SRC_TENSOR_TENSORSYCL_EXTRACT_ACCESSOR_HPP
#define UNSUPPORTED_EIGEN_CXX11_SRC_TENSOR_TENSORSYCL_EXTRACT_ACCESSOR_HPP

namespace Eigen {
namespace TensorSycl {
namespace internal {
/// \struct ExtractAccessor: Extract Accessor Class is used to extract the
/// accessor from a buffer.
/// Depending on the type of the leaf node we can get a read accessor or a
/// read_write accessor
template <typename Evaluator>
struct ExtractAccessor;

struct AccessorConstructor{
  template<typename Arg> static inline auto getTuple(cl::sycl::handler& cgh, const Arg& eval)
  -> decltype(ExtractAccessor<Arg>::getTuple(cgh, eval)) {
  return ExtractAccessor<Arg>::getTuple(cgh, eval);
  }

  template<typename Arg1, typename Arg2> static inline auto getTuple(cl::sycl::handler& cgh, const Arg1& eval1, const Arg2& eval2)
  -> decltype(utility::tuple::append(ExtractAccessor<Arg1>::getTuple(cgh, eval1), ExtractAccessor<Arg2>::getTuple(cgh, eval2))) {
    return utility::tuple::append(ExtractAccessor<Arg1>::getTuple(cgh, eval1), ExtractAccessor<Arg2>::getTuple(cgh, eval2));
  }
  template<typename Arg1, typename Arg2, typename Arg3>	static inline auto getTuple(cl::sycl::handler& cgh, const Arg1& eval1 , const Arg2& eval2 , const Arg3& eval3)
  -> decltype(utility::tuple::append(ExtractAccessor<Arg1>::getTuple(cgh, eval1),utility::tuple::append(ExtractAccessor<Arg2>::getTuple(cgh, eval2), ExtractAccessor<Arg3>::getTuple(cgh, eval3)))) {
    return utility::tuple::append(ExtractAccessor<Arg1>::getTuple(cgh, eval1),utility::tuple::append(ExtractAccessor<Arg2>::getTuple(cgh, eval2), ExtractAccessor<Arg3>::getTuple(cgh, eval3)));
  }
  template< cl::sycl::access::mode AcM, typename Arg> static inline auto getAccessor(cl::sycl::handler& cgh, const Arg& eval)
  -> decltype(utility::tuple::make_tuple( eval.device().template get_sycl_accessor<AcM>(cgh,eval.data()))){
    return utility::tuple::make_tuple(eval.device().template get_sycl_accessor<AcM>(cgh,eval.data()));
  }
};

/// specialisation of the \ref ExtractAccessor struct when the node type is
///  TensorCwiseNullaryOp,  TensorCwiseUnaryOp and  TensorBroadcastingOp
#define SYCLUNARYCATEGORYEXTACC(CVQual)\
template <template<class, class> class UnaryCategory, typename OP, typename RHSExpr, typename Dev>\
struct ExtractAccessor<TensorEvaluator<CVQual UnaryCategory<OP, RHSExpr>, Dev> > {\
  static inline auto getTuple(cl::sycl::handler& cgh, const TensorEvaluator<CVQual UnaryCategory<OP, RHSExpr>, Dev>& eval)\
  -> decltype(AccessorConstructor::getTuple(cgh, eval.impl())){\
    return AccessorConstructor::getTuple(cgh, eval.impl());\
  }\
};

SYCLUNARYCATEGORYEXTACC(const)
SYCLUNARYCATEGORYEXTACC()
#undef SYCLUNARYCATEGORYEXTACC


/// specialisation of the \ref ExtractAccessor struct when the node type is TensorCwiseBinaryOp
#define SYCLBINARYCATEGORYEXTACC(CVQual)\
template <template<class, class, class> class BinaryCategory, typename OP,  typename LHSExpr, typename RHSExpr, typename Dev>\
struct ExtractAccessor<TensorEvaluator<CVQual BinaryCategory<OP, LHSExpr, RHSExpr>, Dev> > {\
  static inline auto getTuple(cl::sycl::handler& cgh, const TensorEvaluator<CVQual BinaryCategory<OP, LHSExpr, RHSExpr>, Dev>& eval)\
  -> decltype(AccessorConstructor::getTuple(cgh, eval.left_impl(), eval.right_impl())){\
    return AccessorConstructor::getTuple(cgh, eval.left_impl(), eval.right_impl());\
  }\
};

SYCLBINARYCATEGORYEXTACC(const)
SYCLBINARYCATEGORYEXTACC()
#undef SYCLBINARYCATEGORYEXTACC

/// specialisation of the \ref ExtractAccessor struct when the node type is
/// const TensorCwiseTernaryOp
#define SYCLTERNARYCATEGORYEXTACC(CVQual)\
template <template<class, class, class, class> class TernaryCategory, typename OP, typename Arg1Expr, typename Arg2Expr, typename Arg3Expr, typename Dev>\
struct ExtractAccessor<TensorEvaluator<CVQual TernaryCategory<OP, Arg1Expr, Arg2Expr, Arg3Expr>, Dev> > {\
  static inline auto getTuple(cl::sycl::handler& cgh, const TensorEvaluator<CVQual TernaryCategory<OP, Arg1Expr, Arg2Expr, Arg3Expr>, Dev>& eval)\
  -> decltype(AccessorConstructor::getTuple(cgh, eval.arg1Impl(), eval.arg2Impl(), eval.arg3Impl())){\
    return AccessorConstructor::getTuple(cgh, eval.arg1Impl(), eval.arg2Impl(), eval.arg3Impl());\
  }\
};

SYCLTERNARYCATEGORYEXTACC(const)
SYCLTERNARYCATEGORYEXTACC()
#undef SYCLTERNARYCATEGORYEXTACC


/// specialisation of the \ref ExtractAccessor struct when the node type is
/// TensorCwiseSelectOp. This is a special case where there is no OP
#define SYCLSELECTOPEXTACC(CVQual)\
template <typename IfExpr, typename ThenExpr, typename ElseExpr, typename Dev>\
struct ExtractAccessor<TensorEvaluator<CVQual TensorSelectOp<IfExpr, ThenExpr, ElseExpr>, Dev> > {\
  static inline auto getTuple(cl::sycl::handler& cgh, const TensorEvaluator<CVQual TensorSelectOp<IfExpr, ThenExpr, ElseExpr>, Dev>& eval)\
  -> decltype(AccessorConstructor::getTuple(cgh, eval.cond_impl(), eval.then_impl(), eval.else_impl())){\
    return AccessorConstructor::getTuple(cgh, eval.cond_impl(), eval.then_impl(), eval.else_impl());\
  }\
};

SYCLSELECTOPEXTACC(const)
SYCLSELECTOPEXTACC()
#undef SYCLSELECTOPEXTACC

/// specialisation of the \ref ExtractAccessor struct when the node type is TensorAssignOp
#define SYCLTENSORASSIGNOPEXTACC(CVQual)\
template <typename LHSExpr, typename RHSExpr, typename Dev>\
struct ExtractAccessor<TensorEvaluator<CVQual TensorAssignOp<LHSExpr, RHSExpr>, Dev> > {\
  static inline auto getTuple(cl::sycl::handler& cgh, const TensorEvaluator<CVQual TensorAssignOp<LHSExpr, RHSExpr>, Dev>& eval)\
  -> decltype(AccessorConstructor::getTuple(cgh, eval.left_impl(), eval.right_impl())){\
    return AccessorConstructor::getTuple(cgh, eval.left_impl(), eval.right_impl());\
 }\
};

 SYCLTENSORASSIGNOPEXTACC(const)
 SYCLTENSORASSIGNOPEXTACC()
 #undef SYCLTENSORASSIGNOPEXTACC

/// specialisation of the \ref ExtractAccessor struct when the node type is const TensorMap
#define TENSORMAPEXPR(CVQual, ACCType)\
template <typename PlainObjectType, int Options_, typename Dev>\
struct ExtractAccessor<TensorEvaluator<CVQual TensorMap<PlainObjectType, Options_>, Dev> > {\
  static inline auto getTuple(cl::sycl::handler& cgh,const TensorEvaluator<CVQual TensorMap<PlainObjectType, Options_>, Dev>& eval)\
  -> decltype(AccessorConstructor::template getAccessor<ACCType>(cgh, eval)){\
    return AccessorConstructor::template getAccessor<ACCType>(cgh, eval);\
  }\
};

TENSORMAPEXPR(const, cl::sycl::access::mode::read)
TENSORMAPEXPR(, cl::sycl::access::mode::read_write)
#undef TENSORMAPEXPR

/// specialisation of the \ref ExtractAccessor struct when the node type is TensorForcedEvalOp
#define SYCLFORCEDEVALEXTACC(CVQual)\
template <typename Expr, typename Dev>\
struct ExtractAccessor<TensorEvaluator<CVQual TensorForcedEvalOp<Expr>, Dev> > {\
  static inline auto getTuple(cl::sycl::handler& cgh, const TensorEvaluator<CVQual TensorForcedEvalOp<Expr>, Dev>& eval)\
  -> decltype(AccessorConstructor::template getAccessor<cl::sycl::access::mode::read>(cgh, eval)){\
    return AccessorConstructor::template getAccessor<cl::sycl::access::mode::read>(cgh, eval);\
  }\
};

SYCLFORCEDEVALEXTACC(const)
SYCLFORCEDEVALEXTACC()
#undef SYCLFORCEDEVALEXTACC


/// specialisation of the \ref ExtractAccessor struct when the node type is TensorEvalToOp
#define SYCLEVALTOEXTACC(CVQual)\
template <typename Expr, typename Dev>\
struct ExtractAccessor<TensorEvaluator<CVQual TensorEvalToOp<Expr>, Dev> > {\
  static inline auto getTuple(cl::sycl::handler& cgh,const TensorEvaluator<CVQual TensorEvalToOp<Expr>, Dev>& eval)\
  -> decltype(utility::tuple::append(AccessorConstructor::template getAccessor<cl::sycl::access::mode::write>(cgh, eval), AccessorConstructor::getTuple(cgh, eval.impl()))){\
    return utility::tuple::append(AccessorConstructor::template getAccessor<cl::sycl::access::mode::write>(cgh, eval), AccessorConstructor::getTuple(cgh, eval.impl()));\
  }\
};

SYCLEVALTOEXTACC(const)
SYCLEVALTOEXTACC()
#undef SYCLEVALTOEXTACC

/// specialisation of the \ref ExtractAccessor struct when the node type is TensorReductionOp
#define SYCLREDUCTIONEXTACC(CVQual)\
template <typename OP, typename Dim, typename Expr, typename Dev>\
struct ExtractAccessor<TensorEvaluator<CVQual TensorReductionOp<OP, Dim, Expr>, Dev> > {\
  static inline auto getTuple(cl::sycl::handler& cgh, const TensorEvaluator<CVQual TensorReductionOp<OP, Dim, Expr>, Dev>& eval)\
  -> decltype(AccessorConstructor::template getAccessor<cl::sycl::access::mode::read>(cgh, eval)){\
    return AccessorConstructor::template getAccessor<cl::sycl::access::mode::read>(cgh, eval);\
  }\
};

SYCLREDUCTIONEXTACC(const)
SYCLREDUCTIONEXTACC()
#undef SYCLREDUCTIONEXTACC

/// specialisation of the \ref ExtractAccessor struct when the node type is
/// const TensorSlicingOp. This is a special case where there is no OP
#define SYCLSLICEOPEXTACC(CVQual)\
template <typename StartIndices, typename Sizes, typename XprType, typename Dev>\
struct ExtractAccessor<TensorEvaluator<CVQual TensorSlicingOp<StartIndices, Sizes, XprType>, Dev> > {\
  static inline auto getTuple(cl::sycl::handler& cgh, const TensorEvaluator<CVQual TensorSlicingOp<StartIndices, Sizes, XprType>, Dev>& eval)\
  -> decltype(AccessorConstructor::getTuple(cgh, eval.impl())){\
    return AccessorConstructor::getTuple(cgh, eval.impl());\
  }\
};

SYCLSLICEOPEXTACC(const)
SYCLSLICEOPEXTACC()
#undef SYCLSLICEOPEXTACC

#define SYCLSLICESTRIDEOPEXTACC(CVQual)\
template<typename StartIndices, typename StopIndices, typename Strides, typename XprType, typename Dev>\
struct ExtractAccessor<TensorEvaluator<CVQual TensorStridingSlicingOp<StartIndices, StopIndices, Strides, XprType>, Dev> >{\
  static inline auto getTuple(cl::sycl::handler& cgh, const TensorEvaluator<CVQual TensorStridingSlicingOp<StartIndices, StopIndices, Strides, XprType>, Dev>& eval)\
  -> decltype(AccessorConstructor::getTuple(cgh, eval.impl())){\
    return AccessorConstructor::getTuple(cgh, eval.impl());\
  }\
};

SYCLSLICESTRIDEOPEXTACC(const)
SYCLSLICESTRIDEOPEXTACC()
#undef SYCLSLICESTRIDEOPEXTACC


#define PADDINGRESHAPEANDSHUFFOPEXTRACC(OPEXPR, CVQual)\
template<typename Param, typename XprType, typename Dev>\
struct ExtractAccessor<TensorEvaluator<CVQual OPEXPR<Param, XprType>, Dev> > {\
  static inline auto getTuple(cl::sycl::handler& cgh, const TensorEvaluator<CVQual OPEXPR<Param, XprType>, Dev>& eval)\
  -> decltype(AccessorConstructor::getTuple(cgh, eval.impl())){\
    return AccessorConstructor::getTuple(cgh, eval.impl());\
  }\
};

// tensor padding
PADDINGRESHAPEANDSHUFFOPEXTRACC(TensorPaddingOp, const)
PADDINGRESHAPEANDSHUFFOPEXTRACC(TensorPaddingOp, )
// tensor reshaping
PADDINGRESHAPEANDSHUFFOPEXTRACC(TensorReshapingOp, const)
PADDINGRESHAPEANDSHUFFOPEXTRACC(TensorReshapingOp, )
/// Tensor shuffling
PADDINGRESHAPEANDSHUFFOPEXTRACC(TensorShufflingOp, const)
PADDINGRESHAPEANDSHUFFOPEXTRACC(TensorShufflingOp, )
#undef PADDINGRESHAPEANDSHUFFOPEXTRACC

/// template deduction for \ref ExtractAccessor
template <typename Evaluator>
auto createTupleOfAccessors(cl::sycl::handler& cgh, const Evaluator& eval)
-> decltype(ExtractAccessor<Evaluator>::getTuple(cgh, eval)) {
  return ExtractAccessor<Evaluator>::getTuple(cgh, eval);
}

} /// namespace TensorSycl
} /// namespace internal
} /// namespace Eigen
#endif  // UNSUPPORTED_EIGEN_CXX11_SRC_TENSOR_TENSORSYCL_EXTRACT_ACCESSOR_HPP
