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

#ifndef EIGEN_SPARSEREDUX_H
#define EIGEN_SPARSEREDUX_H

namespace Eigen { 

template<typename Derived>
typename internal::traits<Derived>::Scalar
SparseMatrixBase<Derived>::sum() const
{
  eigen_assert(rows()>0 && cols()>0 && "you are using a non initialized matrix");
  Scalar res(0);
  for (Index j=0; j<outerSize(); ++j)
    for (typename Derived::InnerIterator iter(derived(),j); iter; ++iter)
      res += iter.value();
  return res;
}

template<typename _Scalar, int _Options, typename _Index>
typename internal::traits<SparseMatrix<_Scalar,_Options,_Index> >::Scalar
SparseMatrix<_Scalar,_Options,_Index>::sum() const
{
  eigen_assert(rows()>0 && cols()>0 && "you are using a non initialized matrix");
  return Matrix<Scalar,1,Dynamic>::Map(&m_data.value(0), m_data.size()).sum();
}

template<typename _Scalar, int _Options, typename _Index>
typename internal::traits<SparseVector<_Scalar,_Options, _Index> >::Scalar
SparseVector<_Scalar,_Options,_Index>::sum() const
{
  eigen_assert(rows()>0 && cols()>0 && "you are using a non initialized matrix");
  return Matrix<Scalar,1,Dynamic>::Map(&m_data.value(0), m_data.size()).sum();
}

} // end namespace Eigen

#endif // EIGEN_SPARSEREDUX_H
