// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2011 Gael Guennebaud <gael.guennebaud@inria.fr>
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

#ifndef EIGEN_INCOMPLETE_LU_H
#define EIGEN_INCOMPLETE_LU_H

namespace Eigen { 

template <typename _Scalar>
class IncompleteLU
{
    typedef _Scalar Scalar;
    typedef Matrix<Scalar,Dynamic,1> Vector;
    typedef typename Vector::Index Index;
    typedef SparseMatrix<Scalar,RowMajor> FactorType;

  public:
    typedef Matrix<Scalar,Dynamic,Dynamic> MatrixType;

    IncompleteLU() : m_isInitialized(false) {}

    template<typename MatrixType>
    IncompleteLU(const MatrixType& mat) : m_isInitialized(false)
    {
      compute(mat);
    }

    Index rows() const { return m_lu.rows(); }
    Index cols() const { return m_lu.cols(); }

    template<typename MatrixType>
    IncompleteLU& compute(const MatrixType& mat)
    {
      m_lu = mat;
      int size = mat.cols();
      Vector diag(size);
      for(int i=0; i<size; ++i)
      {
        typename FactorType::InnerIterator k_it(m_lu,i);
        for(; k_it && k_it.index()<i; ++k_it)
        {
          int k = k_it.index();
          k_it.valueRef() /= diag(k);

          typename FactorType::InnerIterator j_it(k_it);
          typename FactorType::InnerIterator kj_it(m_lu, k);
          while(kj_it && kj_it.index()<=k) ++kj_it;
          for(++j_it; j_it; )
          {
            if(kj_it.index()==j_it.index())
            {
              j_it.valueRef() -= k_it.value() * kj_it.value();
              ++j_it;
              ++kj_it;
            }
            else if(kj_it.index()<j_it.index()) ++kj_it;
            else                                ++j_it;
          }
        }
        if(k_it && k_it.index()==i) diag(i) = k_it.value();
        else                        diag(i) = 1;
      }
      m_isInitialized = true;
      return *this;
    }

    template<typename Rhs, typename Dest>
    void _solve(const Rhs& b, Dest& x) const
    {
      x = m_lu.template triangularView<UnitLower>().solve(b);
      x = m_lu.template triangularView<Upper>().solve(x);
    }

    template<typename Rhs> inline const internal::solve_retval<IncompleteLU, Rhs>
    solve(const MatrixBase<Rhs>& b) const
    {
      eigen_assert(m_isInitialized && "IncompleteLU is not initialized.");
      eigen_assert(cols()==b.rows()
                && "IncompleteLU::solve(): invalid number of rows of the right hand side matrix b");
      return internal::solve_retval<IncompleteLU, Rhs>(*this, b.derived());
    }

  protected:
    FactorType m_lu;
    bool m_isInitialized;
};

namespace internal {

template<typename _MatrixType, typename Rhs>
struct solve_retval<IncompleteLU<_MatrixType>, Rhs>
  : solve_retval_base<IncompleteLU<_MatrixType>, Rhs>
{
  typedef IncompleteLU<_MatrixType> Dec;
  EIGEN_MAKE_SOLVE_HELPERS(Dec,Rhs)

  template<typename Dest> void evalTo(Dest& dst) const
  {
    dec()._solve(rhs(),dst);
  }
};

} // end namespace internal

} // end namespace Eigen

#endif // EIGEN_INCOMPLETE_LU_H
