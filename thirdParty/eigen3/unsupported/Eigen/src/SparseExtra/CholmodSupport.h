// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2008-2010 Gael Guennebaud <gael.guennebaud@inria.fr>
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

#ifndef EIGEN_CHOLMODSUPPORT_H
#define EIGEN_CHOLMODSUPPORT_H

namespace internal {

template<typename Scalar, typename CholmodType>
void cholmod_configure_matrix(CholmodType& mat)
{
  if (internal::is_same<Scalar,float>::value)
  {
    mat.xtype = CHOLMOD_REAL;
    mat.dtype = CHOLMOD_SINGLE;
  }
  else if (internal::is_same<Scalar,double>::value)
  {
    mat.xtype = CHOLMOD_REAL;
    mat.dtype = CHOLMOD_DOUBLE;
  }
  else if (internal::is_same<Scalar,std::complex<float> >::value)
  {
    mat.xtype = CHOLMOD_COMPLEX;
    mat.dtype = CHOLMOD_SINGLE;
  }
  else if (internal::is_same<Scalar,std::complex<double> >::value)
  {
    mat.xtype = CHOLMOD_COMPLEX;
    mat.dtype = CHOLMOD_DOUBLE;
  }
  else
  {
    eigen_assert(false && "Scalar type not supported by CHOLMOD");
  }
}

} // namespace internal

/** Wraps the Eigen sparse matrix \a mat into a Cholmod sparse matrix object.
  * Note that the data are shared.
  */
template<typename _Scalar, int _Options, typename _Index>
cholmod_sparse viewAsCholmod(SparseMatrix<_Scalar,_Options,_Index>& mat)
{
  typedef SparseMatrix<_Scalar,_Options,_Index> MatrixType;
  cholmod_sparse res;
  res.nzmax   = mat.nonZeros();
  res.nrow    = mat.rows();;
  res.ncol    = mat.cols();
  res.p       = mat._outerIndexPtr();
  res.i       = mat._innerIndexPtr();
  res.x       = mat._valuePtr();
  res.sorted  = 1;
  res.packed  = 1;
  res.dtype   = 0;
  res.stype   = -1;
  
  if (internal::is_same<_Index,int>::value)
  {
    res.itype = CHOLMOD_INT;
  }
  else
  {
    eigen_assert(false && "Index type different than int is not supported yet");
  }

  // setup res.xtype
  internal::cholmod_configure_matrix<_Scalar>(res);
  
  res.stype = 0;
  
  return res;
}

template<typename _Scalar, int _Options, typename _Index>
const cholmod_sparse viewAsCholmod(const SparseMatrix<_Scalar,_Options,_Index>& mat)
{
  cholmod_sparse res = viewAsCholmod(mat.const_cast_derived());
  return res;
}

/** Returns a view of the Eigen sparse matrix \a mat as Cholmod sparse matrix.
  * The data are not copied but shared. */
template<typename _Scalar, int _Options, typename _Index, unsigned int UpLo>
cholmod_sparse viewAsCholmod(const SparseSelfAdjointView<SparseMatrix<_Scalar,_Options,_Index>, UpLo>& mat)
{
  cholmod_sparse res = viewAsCholmod(mat.matrix().const_cast_derived());
  
  if(UpLo==Upper) res.stype =  1;
  if(UpLo==Lower) res.stype = -1;

  return res;
}

/** Returns a view of the Eigen \b dense matrix \a mat as Cholmod dense matrix.
  * The data are not copied but shared. */
template<typename Derived>
cholmod_dense viewAsCholmod(MatrixBase<Derived>& mat)
{
  EIGEN_STATIC_ASSERT((internal::traits<Derived>::Flags&RowMajorBit)==0,THIS_METHOD_IS_ONLY_FOR_COLUMN_MAJOR_MATRICES);
  typedef typename Derived::Scalar Scalar;

  cholmod_dense res;
  res.nrow   = mat.rows();
  res.ncol   = mat.cols();
  res.nzmax  = res.nrow * res.ncol;
  res.d      = Derived::IsVectorAtCompileTime ? mat.derived().size() : mat.derived().outerStride();
  res.x      = mat.derived().data();
  res.z      = 0;

  internal::cholmod_configure_matrix<Scalar>(res);

  return res;
}

/** Returns a view of the Cholmod sparse matrix \a cm as an Eigen sparse matrix.
  * The data are not copied but shared. */
template<typename Scalar, int Flags, typename Index>
MappedSparseMatrix<Scalar,Flags,Index> viewAsEigen(cholmod_sparse& cm)
{
  return MappedSparseMatrix<Scalar,Flags,Index>
         (cm.nrow, cm.ncol, reinterpret_cast<Index*>(cm.p)[cm.ncol],
          reinterpret_cast<Index*>(cm.p), reinterpret_cast<Index*>(cm.i),reinterpret_cast<Scalar*>(cm.x) );
}

enum CholmodMode {
  CholmodAuto, CholmodSimplicialLLt, CholmodSupernodalLLt, CholmodLDLt
};

/** \brief A Cholesky factorization and solver based on Cholmod
  *
  * This class allows to solve for A.X = B sparse linear problems via a LL^T or LDL^T Cholesky factorization
  * using the Cholmod library. The sparse matrix A must be selfajoint and positive definite. The vectors or matrices
  * X and B can be either dense or sparse.
  *
  * \tparam _MatrixType the type of the sparse matrix A, it must be a SparseMatrix<>
  * \tparam _UpLo the triangular part that will be used for the computations. It can be Lower
  *               or Upper. Default is Lower.
  *
  */
template<typename _MatrixType, int _UpLo = Lower>
class CholmodDecomposition
{
  public:
    typedef _MatrixType MatrixType;
    enum { UpLo = _UpLo };
    typedef typename MatrixType::Scalar Scalar;
    typedef typename MatrixType::RealScalar RealScalar;
    typedef MatrixType CholMatrixType;
    typedef typename MatrixType::Index Index;

  public:

    CholmodDecomposition()
      : m_cholmodFactor(0), m_info(Success), m_isInitialized(false)
    {
      cholmod_start(&m_cholmod);
      setMode(CholmodLDLt);
    }

    CholmodDecomposition(const MatrixType& matrix)
      : m_cholmodFactor(0), m_info(Success), m_isInitialized(false)
    {
      cholmod_start(&m_cholmod);
      compute(matrix);
    }

    ~CholmodDecomposition()
    {
      if(m_cholmodFactor)
        cholmod_free_factor(&m_cholmodFactor, &m_cholmod);
      cholmod_finish(&m_cholmod);
    }
    
    inline Index cols() const { return m_cholmodFactor->n; }
    inline Index rows() const { return m_cholmodFactor->n; }
    
    void setMode(CholmodMode mode)
    {
      switch(mode)
      {
        case CholmodAuto:
          m_cholmod.final_asis = 1;
          m_cholmod.supernodal = CHOLMOD_AUTO;
          break;
        case CholmodSimplicialLLt:
          m_cholmod.final_asis = 0;
          m_cholmod.supernodal = CHOLMOD_SIMPLICIAL;
          m_cholmod.final_ll = 1;
          break;
        case CholmodSupernodalLLt:
          m_cholmod.final_asis = 1;
          m_cholmod.supernodal = CHOLMOD_SUPERNODAL;
          break;
        case CholmodLDLt:
          m_cholmod.final_asis = 1;
          m_cholmod.supernodal = CHOLMOD_SIMPLICIAL;
          break;
        default:
          break;
      }
    }
    
    /** \brief Reports whether previous computation was successful.
      *
      * \returns \c Success if computation was succesful,
      *          \c NumericalIssue if the matrix.appears to be negative.
      */
    ComputationInfo info() const
    {
      eigen_assert(m_isInitialized && "Decomposition is not initialized.");
      return m_info;
    }

    /** Computes the sparse Cholesky decomposition of \a matrix */
    void compute(const MatrixType& matrix)
    {
      analyzePattern(matrix);
      factorize(matrix);
    }
    
    /** \returns the solution x of \f$ A x = b \f$ using the current decomposition of A.
      *
      * \sa compute()
      */
    template<typename Rhs>
    inline const internal::solve_retval<CholmodDecomposition, Rhs>
    solve(const MatrixBase<Rhs>& b) const
    {
      eigen_assert(m_isInitialized && "LLT is not initialized.");
      eigen_assert(rows()==b.rows()
                && "CholmodDecomposition::solve(): invalid number of rows of the right hand side matrix b");
      return internal::solve_retval<CholmodDecomposition, Rhs>(*this, b.derived());
    }
    
    /** \returns the solution x of \f$ A x = b \f$ using the current decomposition of A.
      *
      * \sa compute()
      */
    template<typename Rhs>
    inline const internal::sparse_solve_retval<CholmodDecomposition, Rhs>
    solve(const SparseMatrixBase<Rhs>& b) const
    {
      eigen_assert(m_isInitialized && "LLT is not initialized.");
      eigen_assert(rows()==b.rows()
                && "CholmodDecomposition::solve(): invalid number of rows of the right hand side matrix b");
      return internal::sparse_solve_retval<CholmodDecomposition, Rhs>(*this, b.derived());
    }
    
    /** Performs a symbolic decomposition on the sparcity of \a matrix.
      *
      * This function is particularly useful when solving for several problems having the same structure.
      * 
      * \sa factorize()
      */
    void analyzePattern(const MatrixType& matrix)
    {
      if(m_cholmodFactor)
      {
        cholmod_free_factor(&m_cholmodFactor, &m_cholmod);
        m_cholmodFactor = 0;
      }
      cholmod_sparse A = viewAsCholmod(matrix.template selfadjointView<UpLo>());
      m_cholmodFactor = cholmod_analyze(&A, &m_cholmod);
      
      this->m_isInitialized = true;
      this->m_info = Success;
      m_analysisIsOk = true;
      m_factorizationIsOk = false;
    }
    
    /** Performs a numeric decomposition of \a matrix
      *
      * The given matrix must has the same sparcity than the matrix on which the symbolic decomposition has been performed.
      *
      * \sa analyzePattern()
      */
    void factorize(const MatrixType& matrix)
    {
      eigen_assert(m_analysisIsOk && "You must first call analyzePattern()");
      cholmod_sparse A = viewAsCholmod(matrix.template selfadjointView<UpLo>());
      cholmod_factorize(&A, m_cholmodFactor, &m_cholmod);
      
      this->m_info = Success;
      m_factorizationIsOk = true;
    }
    
    /** Returns a reference to the Cholmod's configuration structure to get a full control over the performed operations.
     *  See the Cholmod user guide for details. */
    cholmod_common& cholmod() { return m_cholmod; }
    
    #ifndef EIGEN_PARSED_BY_DOXYGEN
    /** \internal */
    template<typename Rhs,typename Dest>
    void _solve(const MatrixBase<Rhs> &b, MatrixBase<Dest> &dest) const
    {
      eigen_assert(m_factorizationIsOk && "The decomposition is not in a valid state for solving, you must first call either compute() or symbolic()/numeric()");
      const Index size = m_cholmodFactor->n;
      eigen_assert(size==b.rows());

      // note: cd stands for Cholmod Dense
      cholmod_dense b_cd = viewAsCholmod(b.const_cast_derived());
      cholmod_dense* x_cd = cholmod_solve(CHOLMOD_A, m_cholmodFactor, &b_cd, &m_cholmod);
      if(!x_cd)
      {
        this->m_info = NumericalIssue;
      }
      // TODO optimize this copy by swapping when possible (be carreful with alignment, etc.)
      dest = Matrix<Scalar,Dest::RowsAtCompileTime,Dest::ColsAtCompileTime>::Map(reinterpret_cast<Scalar*>(x_cd->x),b.rows(),b.cols());
      cholmod_free_dense(&x_cd, &m_cholmod);
    }
    
    /** \internal */
    template<typename RhsScalar, int RhsOptions, typename RhsIndex, typename DestScalar, int DestOptions, typename DestIndex>
    void _solve(const SparseMatrix<RhsScalar,RhsOptions,RhsIndex> &b, SparseMatrix<DestScalar,DestOptions,DestIndex> &dest) const
    {
      eigen_assert(m_factorizationIsOk && "The decomposition is not in a valid state for solving, you must first call either compute() or symbolic()/numeric()");
      const Index size = m_cholmodFactor->n;
      eigen_assert(size==b.rows());

      // note: cs stands for Cholmod Sparse
      cholmod_sparse b_cs = viewAsCholmod(b);
      cholmod_sparse* x_cs = cholmod_spsolve(CHOLMOD_A, m_cholmodFactor, &b_cs, &m_cholmod);
      if(!x_cs)
      {
        this->m_info = NumericalIssue;
      }
      // TODO optimize this copy by swapping when possible (be carreful with alignment, etc.)
      dest = viewAsEigen<DestScalar,DestOptions,DestIndex>(*x_cs);
      cholmod_free_sparse(&x_cs, &m_cholmod);
    }
    #endif // EIGEN_PARSED_BY_DOXYGEN
    
    template<typename Stream>
    void dumpMemory(Stream& s)
    {}

  protected:
    mutable cholmod_common m_cholmod;
    cholmod_factor* m_cholmodFactor;
    mutable ComputationInfo m_info;
    bool m_isInitialized;
    int m_factorizationIsOk;
    int m_analysisIsOk;
};

namespace internal {
  
template<typename _MatrixType, int _UpLo, typename Rhs>
struct solve_retval<CholmodDecomposition<_MatrixType,_UpLo>, Rhs>
  : solve_retval_base<CholmodDecomposition<_MatrixType,_UpLo>, Rhs>
{
  typedef CholmodDecomposition<_MatrixType,_UpLo> Dec;
  EIGEN_MAKE_SOLVE_HELPERS(Dec,Rhs)

  template<typename Dest> void evalTo(Dest& dst) const
  {
    dec()._solve(rhs(),dst);
  }
};

template<typename _MatrixType, int _UpLo, typename Rhs>
struct sparse_solve_retval<CholmodDecomposition<_MatrixType,_UpLo>, Rhs>
  : sparse_solve_retval_base<CholmodDecomposition<_MatrixType,_UpLo>, Rhs>
{
  typedef CholmodDecomposition<_MatrixType,_UpLo> Dec;
  EIGEN_MAKE_SPARSE_SOLVE_HELPERS(Dec,Rhs)

  template<typename Dest> void evalTo(Dest& dst) const
  {
    dec()._solve(rhs(),dst);
  }
};

}

#endif // EIGEN_CHOLMODSUPPORT_H
