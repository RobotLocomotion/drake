// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2008-2009 Gael Guennebaud <gael.guennebaud@inria.fr>
//
// Eigen is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
//
// Alternatively, you can redistribute it and/or
// modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation; either version 2 of
// the License, or (at your option) any later version.
//
// Eigen is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License or the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License and a copy of the GNU General Public License along with
// Eigen. If not, see <http://www.gnu.org/licenses/>.

#ifndef EIGEN_SPARSETRANSPOSE_H
#define EIGEN_SPARSETRANSPOSE_H

namespace Eigen { 

template<typename MatrixType> class TransposeImpl<MatrixType,Sparse>
  : public SparseMatrixBase<Transpose<MatrixType> >
{
    typedef typename internal::remove_all<typename MatrixType::Nested>::type _MatrixTypeNested;
  public:

    EIGEN_SPARSE_PUBLIC_INTERFACE(Transpose<MatrixType>)

    class InnerIterator;
    class ReverseInnerIterator;

    inline Index nonZeros() const { return derived().nestedExpression().nonZeros(); }
};

// NOTE: VC10 trigger an ICE if don't put typename TransposeImpl<MatrixType,Sparse>:: in front of Index,
// a typedef typename TransposeImpl<MatrixType,Sparse>::Index Index;
// does not fix the issue.
// An alternative is to define the nested class in the parent class itself.
template<typename MatrixType> class TransposeImpl<MatrixType,Sparse>::InnerIterator
  : public _MatrixTypeNested::InnerIterator
{
    typedef typename _MatrixTypeNested::InnerIterator Base;
  public:

    EIGEN_STRONG_INLINE InnerIterator(const TransposeImpl& trans, typename TransposeImpl<MatrixType,Sparse>::Index outer)
      : Base(trans.derived().nestedExpression(), outer)
    {}
    inline typename TransposeImpl<MatrixType,Sparse>::Index row() const { return Base::col(); }
    inline typename TransposeImpl<MatrixType,Sparse>::Index col() const { return Base::row(); }
};

template<typename MatrixType> class TransposeImpl<MatrixType,Sparse>::ReverseInnerIterator
  : public _MatrixTypeNested::ReverseInnerIterator
{
    typedef typename _MatrixTypeNested::ReverseInnerIterator Base;
  public:

    EIGEN_STRONG_INLINE ReverseInnerIterator(const TransposeImpl& xpr, typename TransposeImpl<MatrixType,Sparse>::Index outer)
      : Base(xpr.derived().nestedExpression(), outer)
    {}
    inline typename TransposeImpl<MatrixType,Sparse>::Index row() const { return Base::col(); }
    inline typename TransposeImpl<MatrixType,Sparse>::Index col() const { return Base::row(); }
};

} // end namespace Eigen

#endif // EIGEN_SPARSETRANSPOSE_H
