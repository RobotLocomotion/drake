// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2008 Gael Guennebaud <gael.guennebaud@inria.fr>
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

#ifndef EIGEN_VISITOR_H
#define EIGEN_VISITOR_H

namespace Eigen { 

namespace internal {

template<typename Visitor, typename Derived, int UnrollCount>
struct visitor_impl
{
  enum {
    col = (UnrollCount-1) / Derived::RowsAtCompileTime,
    row = (UnrollCount-1) % Derived::RowsAtCompileTime
  };

  static inline void run(const Derived &mat, Visitor& visitor)
  {
    visitor_impl<Visitor, Derived, UnrollCount-1>::run(mat, visitor);
    visitor(mat.coeff(row, col), row, col);
  }
};

template<typename Visitor, typename Derived>
struct visitor_impl<Visitor, Derived, 1>
{
  static inline void run(const Derived &mat, Visitor& visitor)
  {
    return visitor.init(mat.coeff(0, 0), 0, 0);
  }
};

template<typename Visitor, typename Derived>
struct visitor_impl<Visitor, Derived, Dynamic>
{
  typedef typename Derived::Index Index;
  static inline void run(const Derived& mat, Visitor& visitor)
  {
    visitor.init(mat.coeff(0,0), 0, 0);
    for(Index i = 1; i < mat.rows(); ++i)
      visitor(mat.coeff(i, 0), i, 0);
    for(Index j = 1; j < mat.cols(); ++j)
      for(Index i = 0; i < mat.rows(); ++i)
        visitor(mat.coeff(i, j), i, j);
  }
};

} // end namespace internal

/** Applies the visitor \a visitor to the whole coefficients of the matrix or vector.
  *
  * The template parameter \a Visitor is the type of the visitor and provides the following interface:
  * \code
  * struct MyVisitor {
  *   // called for the first coefficient
  *   void init(const Scalar& value, Index i, Index j);
  *   // called for all other coefficients
  *   void operator() (const Scalar& value, Index i, Index j);
  * };
  * \endcode
  *
  * \note compared to one or two \em for \em loops, visitors offer automatic
  * unrolling for small fixed size matrix.
  *
  * \sa minCoeff(Index*,Index*), maxCoeff(Index*,Index*), DenseBase::redux()
  */
template<typename Derived>
template<typename Visitor>
void DenseBase<Derived>::visit(Visitor& visitor) const
{
  enum { unroll = SizeAtCompileTime != Dynamic
                   && CoeffReadCost != Dynamic
                   && (SizeAtCompileTime == 1 || internal::functor_traits<Visitor>::Cost != Dynamic)
                   && SizeAtCompileTime * CoeffReadCost + (SizeAtCompileTime-1) * internal::functor_traits<Visitor>::Cost
                      <= EIGEN_UNROLLING_LIMIT };
  return internal::visitor_impl<Visitor, Derived,
      unroll ? int(SizeAtCompileTime) : Dynamic
    >::run(derived(), visitor);
}

namespace internal {

/** \internal
  * \brief Base class to implement min and max visitors
  */
template <typename Derived>
struct coeff_visitor
{
  typedef typename Derived::Index Index;
  typedef typename Derived::Scalar Scalar;
  Index row, col;
  Scalar res;
  inline void init(const Scalar& value, Index i, Index j)
  {
    res = value;
    row = i;
    col = j;
  }
};

/** \internal
  * \brief Visitor computing the min coefficient with its value and coordinates
  *
  * \sa DenseBase::minCoeff(Index*, Index*)
  */
template <typename Derived>
struct min_coeff_visitor : coeff_visitor<Derived>
{
  typedef typename Derived::Index Index;
  typedef typename Derived::Scalar Scalar;
  void operator() (const Scalar& value, Index i, Index j)
  {
    if(value < this->res)
    {
      this->res = value;
      this->row = i;
      this->col = j;
    }
  }
};

template<typename Scalar>
struct functor_traits<min_coeff_visitor<Scalar> > {
  enum {
    Cost = NumTraits<Scalar>::AddCost
  };
};

/** \internal
  * \brief Visitor computing the max coefficient with its value and coordinates
  *
  * \sa DenseBase::maxCoeff(Index*, Index*)
  */
template <typename Derived>
struct max_coeff_visitor : coeff_visitor<Derived>
{
  typedef typename Derived::Index Index;
  typedef typename Derived::Scalar Scalar;
  void operator() (const Scalar& value, Index i, Index j)
  {
    if(value > this->res)
    {
      this->res = value;
      this->row = i;
      this->col = j;
    }
  }
};

template<typename Scalar>
struct functor_traits<max_coeff_visitor<Scalar> > {
  enum {
    Cost = NumTraits<Scalar>::AddCost
  };
};

} // end namespace internal

/** \returns the minimum of all coefficients of *this
  * and puts in *row and *col its location.
  *
  * \sa DenseBase::minCoeff(Index*), DenseBase::maxCoeff(Index*,Index*), DenseBase::visitor(), DenseBase::minCoeff()
  */
template<typename Derived>
template<typename IndexType>
typename internal::traits<Derived>::Scalar
DenseBase<Derived>::minCoeff(IndexType* row, IndexType* col) const
{
  internal::min_coeff_visitor<Derived> minVisitor;
  this->visit(minVisitor);
  *row = minVisitor.row;
  if (col) *col = minVisitor.col;
  return minVisitor.res;
}

/** \returns the minimum of all coefficients of *this
  * and puts in *index its location.
  *
  * \sa DenseBase::minCoeff(IndexType*,IndexType*), DenseBase::maxCoeff(IndexType*,IndexType*), DenseBase::visitor(), DenseBase::minCoeff()
  */
template<typename Derived>
template<typename IndexType>
typename internal::traits<Derived>::Scalar
DenseBase<Derived>::minCoeff(IndexType* index) const
{
  EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived)
  internal::min_coeff_visitor<Derived> minVisitor;
  this->visit(minVisitor);
  *index = (RowsAtCompileTime==1) ? minVisitor.col : minVisitor.row;
  return minVisitor.res;
}

/** \returns the maximum of all coefficients of *this
  * and puts in *row and *col its location.
  *
  * \sa DenseBase::minCoeff(IndexType*,IndexType*), DenseBase::visitor(), DenseBase::maxCoeff()
  */
template<typename Derived>
template<typename IndexType>
typename internal::traits<Derived>::Scalar
DenseBase<Derived>::maxCoeff(IndexType* row, IndexType* col) const
{
  internal::max_coeff_visitor<Derived> maxVisitor;
  this->visit(maxVisitor);
  *row = maxVisitor.row;
  if (col) *col = maxVisitor.col;
  return maxVisitor.res;
}

/** \returns the maximum of all coefficients of *this
  * and puts in *index its location.
  *
  * \sa DenseBase::maxCoeff(IndexType*,IndexType*), DenseBase::minCoeff(IndexType*,IndexType*), DenseBase::visitor(), DenseBase::maxCoeff()
  */
template<typename Derived>
template<typename IndexType>
typename internal::traits<Derived>::Scalar
DenseBase<Derived>::maxCoeff(IndexType* index) const
{
  EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived)
  internal::max_coeff_visitor<Derived> maxVisitor;
  this->visit(maxVisitor);
  *index = (RowsAtCompileTime==1) ? maxVisitor.col : maxVisitor.row;
  return maxVisitor.res;
}

} // end namespace Eigen

#endif // EIGEN_VISITOR_H
