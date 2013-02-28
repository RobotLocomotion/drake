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

#ifndef EIGEN_SPARSEVECTOR_H
#define EIGEN_SPARSEVECTOR_H

/** \class SparseVector
  *
  * \brief a sparse vector class
  *
  * \tparam _Scalar the scalar type, i.e. the type of the coefficients
  *
  * See http://www.netlib.org/linalg/html_templates/node91.html for details on the storage scheme.
  *
  * This class can be extended with the help of the plugin mechanism described on the page
  * \ref TopicCustomizingEigen by defining the preprocessor symbol \c EIGEN_SPARSEVECTOR_PLUGIN.
  */

namespace internal {
template<typename _Scalar, int _Options, typename _Index>
struct traits<SparseVector<_Scalar, _Options, _Index> >
{
  typedef _Scalar Scalar;
  typedef _Index Index;
  typedef Sparse StorageKind;
  typedef MatrixXpr XprKind;
  enum {
    IsColVector = _Options & RowMajorBit ? 0 : 1,

    RowsAtCompileTime = IsColVector ? Dynamic : 1,
    ColsAtCompileTime = IsColVector ? 1 : Dynamic,
    MaxRowsAtCompileTime = RowsAtCompileTime,
    MaxColsAtCompileTime = ColsAtCompileTime,
    Flags = _Options | NestByRefBit | LvalueBit,
    CoeffReadCost = NumTraits<Scalar>::ReadCost,
    SupportedAccessPatterns = InnerRandomAccessPattern
  };
};
}

template<typename _Scalar, int _Options, typename _Index>
class SparseVector
  : public SparseMatrixBase<SparseVector<_Scalar, _Options, _Index> >
{
  public:
    EIGEN_SPARSE_PUBLIC_INTERFACE(SparseVector)
    EIGEN_SPARSE_INHERIT_ASSIGNMENT_OPERATOR(SparseVector, +=)
    EIGEN_SPARSE_INHERIT_ASSIGNMENT_OPERATOR(SparseVector, -=)
//     EIGEN_SPARSE_INHERIT_ASSIGNMENT_OPERATOR(SparseVector, =)

  protected:
  public:

    typedef SparseMatrixBase<SparseVector> SparseBase;
    enum { IsColVector = internal::traits<SparseVector>::IsColVector };
    
    enum {
      Options = _Options
    };

    CompressedStorage<Scalar,Index> m_data;
    Index m_size;

    CompressedStorage<Scalar,Index>& _data() { return m_data; }
    CompressedStorage<Scalar,Index>& _data() const { return m_data; }

  public:

    EIGEN_STRONG_INLINE Index rows() const { return IsColVector ? m_size : 1; }
    EIGEN_STRONG_INLINE Index cols() const { return IsColVector ? 1 : m_size; }
    EIGEN_STRONG_INLINE Index innerSize() const { return m_size; }
    EIGEN_STRONG_INLINE Index outerSize() const { return 1; }
    EIGEN_STRONG_INLINE Index innerNonZeros(Index j) const { eigen_assert(j==0); return m_size; }

    EIGEN_STRONG_INLINE const Scalar* _valuePtr() const { return &m_data.value(0); }
    EIGEN_STRONG_INLINE Scalar* _valuePtr() { return &m_data.value(0); }

    EIGEN_STRONG_INLINE const Index* _innerIndexPtr() const { return &m_data.index(0); }
    EIGEN_STRONG_INLINE Index* _innerIndexPtr() { return &m_data.index(0); }

    inline Scalar coeff(Index row, Index col) const
    {
      eigen_assert((IsColVector ? col : row)==0);
      return coeff(IsColVector ? row : col);
    }
    inline Scalar coeff(Index i) const { return m_data.at(i); }

    inline Scalar& coeffRef(Index row, Index col)
    {
      eigen_assert((IsColVector ? col : row)==0);
      return coeff(IsColVector ? row : col);
    }

    /** \returns a reference to the coefficient value at given index \a i
      * This operation involes a log(rho*size) binary search. If the coefficient does not
      * exist yet, then a sorted insertion into a sequential buffer is performed.
      *
      * This insertion might be very costly if the number of nonzeros above \a i is large.
      */
    inline Scalar& coeffRef(Index i)
    {
      return m_data.atWithInsertion(i);
    }

  public:

    class InnerIterator;

    inline void setZero() { m_data.clear(); }

    /** \returns the number of non zero coefficients */
    inline Index nonZeros() const  { return static_cast<Index>(m_data.size()); }

    inline void startVec(Index outer)
    {
      eigen_assert(outer==0);
    }

    inline Scalar& insertBackByOuterInner(Index outer, Index inner)
    {
      eigen_assert(outer==0);
      return insertBack(inner);
    }
    inline Scalar& insertBack(Index i)
    {
      m_data.append(0, i);
      return m_data.value(m_data.size()-1);
    }

    inline Scalar& insert(Index row, Index col)
    {
      Index inner = IsColVector ? row : col;
      Index outer = IsColVector ? col : row;
      eigen_assert(outer==0);
      return insert(inner);
    }
    Scalar& insert(Index i)
    {
      Index startId = 0;
      Index p = m_data.size() - 1;
      // TODO smart realloc
      m_data.resize(p+2,1);

      while ( (p >= startId) && (m_data.index(p) > i) )
      {
        m_data.index(p+1) = m_data.index(p);
        m_data.value(p+1) = m_data.value(p);
        --p;
      }
      m_data.index(p+1) = i;
      m_data.value(p+1) = 0;
      return m_data.value(p+1);
    }

    /**
      */
    inline void reserve(Index reserveSize) { m_data.reserve(reserveSize); }


    inline void finalize() {}

    void prune(Scalar reference, RealScalar epsilon = NumTraits<RealScalar>::dummy_precision())
    {
      m_data.prune(reference,epsilon);
    }

    void resize(Index rows, Index cols)
    {
      eigen_assert(rows==1 || cols==1);
      resize(IsColVector ? rows : cols);
    }

    void resize(Index newSize)
    {
      m_size = newSize;
      m_data.clear();
    }

    void resizeNonZeros(Index size) { m_data.resize(size); }

    inline SparseVector() : m_size(0) { resize(0); }

    inline SparseVector(Index size) : m_size(0) { resize(size); }

    inline SparseVector(Index rows, Index cols) : m_size(0) { resize(rows,cols); }

    template<typename OtherDerived>
    inline SparseVector(const MatrixBase<OtherDerived>& other)
      : m_size(0)
    {
      *this = other.derived();
    }

    template<typename OtherDerived>
    inline SparseVector(const SparseMatrixBase<OtherDerived>& other)
      : m_size(0)
    {
      *this = other.derived();
    }

    inline SparseVector(const SparseVector& other)
      : m_size(0)
    {
      *this = other.derived();
    }

    inline void swap(SparseVector& other)
    {
      std::swap(m_size, other.m_size);
      m_data.swap(other.m_data);
    }

    inline SparseVector& operator=(const SparseVector& other)
    {
      if (other.isRValue())
      {
        swap(other.const_cast_derived());
      }
      else
      {
        resize(other.size());
        m_data = other.m_data;
      }
      return *this;
    }

    template<typename OtherDerived>
    inline SparseVector& operator=(const SparseMatrixBase<OtherDerived>& other)
    {
      if (int(RowsAtCompileTime)!=int(OtherDerived::RowsAtCompileTime))
        return Base::operator=(other.transpose());
      else
        return Base::operator=(other);
    }

    #ifndef EIGEN_PARSED_BY_DOXYGEN
    template<typename Lhs, typename Rhs>
    inline SparseVector& operator=(const SparseSparseProduct<Lhs,Rhs>& product)
    {
      return Base::operator=(product);
    }
    #endif

//       const bool needToTranspose = (Flags & RowMajorBit) != (OtherDerived::Flags & RowMajorBit);
//       if (needToTranspose)
//       {
//         // two passes algorithm:
//         //  1 - compute the number of coeffs per dest inner vector
//         //  2 - do the actual copy/eval
//         // Since each coeff of the rhs has to be evaluated twice, let's evauluate it if needed
//         typedef typename internal::nested<OtherDerived,2>::type OtherCopy;
//         OtherCopy otherCopy(other.derived());
//         typedef typename internal::remove_all<OtherCopy>::type _OtherCopy;
//
//         resize(other.rows(), other.cols());
//         Eigen::Map<VectorXi>(m_outerIndex,outerSize()).setZero();
//         // pass 1
//         // FIXME the above copy could be merged with that pass
//         for (int j=0; j<otherCopy.outerSize(); ++j)
//           for (typename _OtherCopy::InnerIterator it(otherCopy, j); it; ++it)
//             ++m_outerIndex[it.index()];
//
//         // prefix sum
//         int count = 0;
//         VectorXi positions(outerSize());
//         for (int j=0; j<outerSize(); ++j)
//         {
//           int tmp = m_outerIndex[j];
//           m_outerIndex[j] = count;
//           positions[j] = count;
//           count += tmp;
//         }
//         m_outerIndex[outerSize()] = count;
//         // alloc
//         m_data.resize(count);
//         // pass 2
//         for (int j=0; j<otherCopy.outerSize(); ++j)
//           for (typename _OtherCopy::InnerIterator it(otherCopy, j); it; ++it)
//           {
//             int pos = positions[it.index()]++;
//             m_data.index(pos) = j;
//             m_data.value(pos) = it.value();
//           }
//
//         return *this;
//       }
//       else
//       {
//         // there is no special optimization
//         return SparseMatrixBase<SparseMatrix>::operator=(other.derived());
//       }
//     }

    friend std::ostream & operator << (std::ostream & s, const SparseVector& m)
    {
      for (Index i=0; i<m.nonZeros(); ++i)
        s << "(" << m.m_data.value(i) << "," << m.m_data.index(i) << ") ";
      s << std::endl;
      return s;
    }

    // this specialized version does not seems to be faster
//     Scalar dot(const SparseVector& other) const
//     {
//       int i=0, j=0;
//       Scalar res = 0;
//       asm("#begindot");
//       while (i<nonZeros() && j<other.nonZeros())
//       {
//         if (m_data.index(i)==other.m_data.index(j))
//         {
//           res += m_data.value(i) * internal::conj(other.m_data.value(j));
//           ++i; ++j;
//         }
//         else if (m_data.index(i)<other.m_data.index(j))
//           ++i;
//         else
//           ++j;
//       }
//       asm("#enddot");
//       return res;
//     }

    /** Destructor */
    inline ~SparseVector() {}

    /** Overloaded for performance */
    Scalar sum() const;

  public:

    /** \deprecated use setZero() and reserve() */
    EIGEN_DEPRECATED void startFill(Index reserve)
    {
      setZero();
      m_data.reserve(reserve);
    }

    /** \deprecated use insertBack(Index,Index) */
    EIGEN_DEPRECATED Scalar& fill(Index r, Index c)
    {
      eigen_assert(r==0 || c==0);
      return fill(IsColVector ? r : c);
    }

    /** \deprecated use insertBack(Index) */
    EIGEN_DEPRECATED Scalar& fill(Index i)
    {
      m_data.append(0, i);
      return m_data.value(m_data.size()-1);
    }

    /** \deprecated use insert(Index,Index) */
    EIGEN_DEPRECATED Scalar& fillrand(Index r, Index c)
    {
      eigen_assert(r==0 || c==0);
      return fillrand(IsColVector ? r : c);
    }

    /** \deprecated use insert(Index) */
    EIGEN_DEPRECATED Scalar& fillrand(Index i)
    {
      return insert(i);
    }

    /** \deprecated use finalize() */
    EIGEN_DEPRECATED void endFill() {}
    
#   ifdef EIGEN_SPARSEVECTOR_PLUGIN
#     include EIGEN_SPARSEVECTOR_PLUGIN
#   endif
};

template<typename Scalar, int _Options, typename _Index>
class SparseVector<Scalar,_Options,_Index>::InnerIterator
{
  public:
    InnerIterator(const SparseVector& vec, Index outer=0)
      : m_data(vec.m_data), m_id(0), m_end(static_cast<Index>(m_data.size()))
    {
      eigen_assert(outer==0);
    }

    InnerIterator(const CompressedStorage<Scalar,Index>& data)
      : m_data(data), m_id(0), m_end(static_cast<Index>(m_data.size()))
    {}

    template<unsigned int Added, unsigned int Removed>
    InnerIterator(const Flagged<SparseVector,Added,Removed>& vec, Index )
      : m_data(vec._expression().m_data), m_id(0), m_end(m_data.size())
    {}

    inline InnerIterator& operator++() { m_id++; return *this; }

    inline Scalar value() const { return m_data.value(m_id); }
    inline Scalar& valueRef() { return const_cast<Scalar&>(m_data.value(m_id)); }

    inline Index index() const { return m_data.index(m_id); }
    inline Index row() const { return IsColVector ? index() : 0; }
    inline Index col() const { return IsColVector ? 0 : index(); }

    inline operator bool() const { return (m_id < m_end); }

  protected:
    const CompressedStorage<Scalar,Index>& m_data;
    Index m_id;
    const Index m_end;
};

#endif // EIGEN_SPARSEVECTOR_H
