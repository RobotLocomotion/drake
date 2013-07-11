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

#ifndef EIGEN_BASIC_PRECONDITIONERS_H
#define EIGEN_BASIC_PRECONDITIONERS_H

namespace Eigen { 

/** \ingroup IterativeLinearSolvers_Module
  * \brief A preconditioner based on the digonal entries
  *
  * This class allows to approximately solve for A.x = b problems assuming A is a diagonal matrix.
  * In other words, this preconditioner neglects all off diagonal entries and, in Eigen's language, solves for:
  * \code
  * A.diagonal().asDiagonal() . x = b
  * \endcode
  *
  * \tparam _Scalar the type of the scalar.
  *
  * This preconditioner is suitable for both selfadjoint and general problems.
  * The diagonal entries are pre-inverted and stored into a dense vector.
  *
  * \note A variant that has yet to be implemented would attempt to preserve the norm of each column.
  *
  */
template <typename _Scalar>
class DiagonalPreconditioner
{
    typedef _Scalar Scalar;
    typedef Matrix<Scalar,Dynamic,1> Vector;
    typedef typename Vector::Index Index;

  public:
    typedef Matrix<Scalar,Dynamic,Dynamic> MatrixType;

    DiagonalPreconditioner() : m_isInitialized(false) {}

    template<typename MatrixType>
    DiagonalPreconditioner(const MatrixType& mat) : m_invdiag(mat.cols())
    {
      compute(mat);
    }

    Index rows() const { return m_invdiag.size(); }
    Index cols() const { return m_invdiag.size(); }
    
    template<typename MatrixType>
    DiagonalPreconditioner& analyzePattern(const MatrixType& )
    {
      return *this;
    }
    
    template<typename MatrixType>
    DiagonalPreconditioner& factorize(const MatrixType& mat)
    {
      m_invdiag.resize(mat.cols());
      for(int j=0; j<mat.outerSize(); ++j)
      {
        typename MatrixType::InnerIterator it(mat,j);
        while(it && it.index()!=j) ++it;
        if(it && it.index()==j)
          m_invdiag(j) = Scalar(1)/it.value();
        else
          m_invdiag(j) = 0;
      }
      m_isInitialized = true;
      return *this;
    }
    
    template<typename MatrixType>
    DiagonalPreconditioner& compute(const MatrixType& mat)
    {
      return factorize(mat);
    }

    template<typename Rhs, typename Dest>
    void _solve(const Rhs& b, Dest& x) const
    {
      x = m_invdiag.array() * b.array() ;
    }

    template<typename Rhs> inline const internal::solve_retval<DiagonalPreconditioner, Rhs>
    solve(const MatrixBase<Rhs>& b) const
    {
      eigen_assert(m_isInitialized && "DiagonalPreconditioner is not initialized.");
      eigen_assert(m_invdiag.size()==b.rows()
                && "DiagonalPreconditioner::solve(): invalid number of rows of the right hand side matrix b");
      return internal::solve_retval<DiagonalPreconditioner, Rhs>(*this, b.derived());
    }

  protected:
    Vector m_invdiag;
    bool m_isInitialized;
};

namespace internal {

template<typename _MatrixType, typename Rhs>
struct solve_retval<DiagonalPreconditioner<_MatrixType>, Rhs>
  : solve_retval_base<DiagonalPreconditioner<_MatrixType>, Rhs>
{
  typedef DiagonalPreconditioner<_MatrixType> Dec;
  EIGEN_MAKE_SOLVE_HELPERS(Dec,Rhs)

  template<typename Dest> void evalTo(Dest& dst) const
  {
    dec()._solve(rhs(),dst);
  }
};

}

/** \ingroup IterativeLinearSolvers_Module
  * \brief A naive preconditioner which approximates any matrix as the identity matrix
  *
  * \sa class DiagonalPreconditioner
  */
class IdentityPreconditioner
{
  public:

    IdentityPreconditioner() {}

    template<typename MatrixType>
    IdentityPreconditioner(const MatrixType& ) {}
    
    template<typename MatrixType>
    IdentityPreconditioner& analyzePattern(const MatrixType& ) { return *this; }
    
    template<typename MatrixType>
    IdentityPreconditioner& factorize(const MatrixType& ) { return *this; }

    template<typename MatrixType>
    IdentityPreconditioner& compute(const MatrixType& ) { return *this; }
    
    template<typename Rhs>
    inline const Rhs& solve(const Rhs& b) const { return b; }
};

} // end namespace Eigen

#endif // EIGEN_BASIC_PRECONDITIONERS_H
