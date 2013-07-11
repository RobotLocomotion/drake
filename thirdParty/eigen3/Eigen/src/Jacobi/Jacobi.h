// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2009 Benoit Jacob <jacob.benoit.1@gmail.com>
// Copyright (C) 2009 Gael Guennebaud <gael.guennebaud@inria.fr>
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

#ifndef EIGEN_JACOBI_H
#define EIGEN_JACOBI_H

namespace Eigen { 

/** \ingroup Jacobi_Module
  * \jacobi_module
  * \class JacobiRotation
  * \brief Rotation given by a cosine-sine pair.
  *
  * This class represents a Jacobi or Givens rotation.
  * This is a 2D rotation in the plane \c J of angle \f$ \theta \f$ defined by
  * its cosine \c c and sine \c s as follow:
  * \f$ J = \left ( \begin{array}{cc} c & \overline s \\ -s  & \overline c \end{array} \right ) \f$
  *
  * You can apply the respective counter-clockwise rotation to a column vector \c v by
  * applying its adjoint on the left: \f$ v = J^* v \f$ that translates to the following Eigen code:
  * \code
  * v.applyOnTheLeft(J.adjoint());
  * \endcode
  *
  * \sa MatrixBase::applyOnTheLeft(), MatrixBase::applyOnTheRight()
  */
template<typename Scalar> class JacobiRotation
{
  public:
    typedef typename NumTraits<Scalar>::Real RealScalar;

    /** Default constructor without any initialization. */
    JacobiRotation() {}

    /** Construct a planar rotation from a cosine-sine pair (\a c, \c s). */
    JacobiRotation(const Scalar& c, const Scalar& s) : m_c(c), m_s(s) {}

    Scalar& c() { return m_c; }
    Scalar c() const { return m_c; }
    Scalar& s() { return m_s; }
    Scalar s() const { return m_s; }

    /** Concatenates two planar rotation */
    JacobiRotation operator*(const JacobiRotation& other)
    {
      return JacobiRotation(m_c * other.m_c - internal::conj(m_s) * other.m_s,
                            internal::conj(m_c * internal::conj(other.m_s) + internal::conj(m_s) * internal::conj(other.m_c)));
    }

    /** Returns the transposed transformation */
    JacobiRotation transpose() const { return JacobiRotation(m_c, -internal::conj(m_s)); }

    /** Returns the adjoint transformation */
    JacobiRotation adjoint() const { return JacobiRotation(internal::conj(m_c), -m_s); }

    template<typename Derived>
    bool makeJacobi(const MatrixBase<Derived>&, typename Derived::Index p, typename Derived::Index q);
    bool makeJacobi(RealScalar x, Scalar y, RealScalar z);

    void makeGivens(const Scalar& p, const Scalar& q, Scalar* z=0);

  protected:
    void makeGivens(const Scalar& p, const Scalar& q, Scalar* z, internal::true_type);
    void makeGivens(const Scalar& p, const Scalar& q, Scalar* z, internal::false_type);

    Scalar m_c, m_s;
};

/** Makes \c *this as a Jacobi rotation \a J such that applying \a J on both the right and left sides of the selfadjoint 2x2 matrix
  * \f$ B = \left ( \begin{array}{cc} x & y \\ \overline y & z \end{array} \right )\f$ yields a diagonal matrix \f$ A = J^* B J \f$
  *
  * \sa MatrixBase::makeJacobi(const MatrixBase<Derived>&, Index, Index), MatrixBase::applyOnTheLeft(), MatrixBase::applyOnTheRight()
  */
template<typename Scalar>
bool JacobiRotation<Scalar>::makeJacobi(RealScalar x, Scalar y, RealScalar z)
{
  typedef typename NumTraits<Scalar>::Real RealScalar;
  if(y == Scalar(0))
  {
    m_c = Scalar(1);
    m_s = Scalar(0);
    return false;
  }
  else
  {
    RealScalar tau = (x-z)/(RealScalar(2)*internal::abs(y));
    RealScalar w = internal::sqrt(internal::abs2(tau) + RealScalar(1));
    RealScalar t;
    if(tau>RealScalar(0))
    {
      t = RealScalar(1) / (tau + w);
    }
    else
    {
      t = RealScalar(1) / (tau - w);
    }
    RealScalar sign_t = t > RealScalar(0) ? RealScalar(1) : RealScalar(-1);
    RealScalar n = RealScalar(1) / internal::sqrt(internal::abs2(t)+RealScalar(1));
    m_s = - sign_t * (internal::conj(y) / internal::abs(y)) * internal::abs(t) * n;
    m_c = n;
    return true;
  }
}

/** Makes \c *this as a Jacobi rotation \c J such that applying \a J on both the right and left sides of the 2x2 selfadjoint matrix
  * \f$ B = \left ( \begin{array}{cc} \text{this}_{pp} & \text{this}_{pq} \\ (\text{this}_{pq})^* & \text{this}_{qq} \end{array} \right )\f$ yields
  * a diagonal matrix \f$ A = J^* B J \f$
  *
  * Example: \include Jacobi_makeJacobi.cpp
  * Output: \verbinclude Jacobi_makeJacobi.out
  *
  * \sa JacobiRotation::makeJacobi(RealScalar, Scalar, RealScalar), MatrixBase::applyOnTheLeft(), MatrixBase::applyOnTheRight()
  */
template<typename Scalar>
template<typename Derived>
inline bool JacobiRotation<Scalar>::makeJacobi(const MatrixBase<Derived>& m, typename Derived::Index p, typename Derived::Index q)
{
  return makeJacobi(internal::real(m.coeff(p,p)), m.coeff(p,q), internal::real(m.coeff(q,q)));
}

/** Makes \c *this as a Givens rotation \c G such that applying \f$ G^* \f$ to the left of the vector
  * \f$ V = \left ( \begin{array}{c} p \\ q \end{array} \right )\f$ yields:
  * \f$ G^* V = \left ( \begin{array}{c} r \\ 0 \end{array} \right )\f$.
  *
  * The value of \a z is returned if \a z is not null (the default is null).
  * Also note that G is built such that the cosine is always real.
  *
  * Example: \include Jacobi_makeGivens.cpp
  * Output: \verbinclude Jacobi_makeGivens.out
  *
  * This function implements the continuous Givens rotation generation algorithm
  * found in Anderson (2000), Discontinuous Plane Rotations and the Symmetric Eigenvalue Problem.
  * LAPACK Working Note 150, University of Tennessee, UT-CS-00-454, December 4, 2000.
  *
  * \sa MatrixBase::applyOnTheLeft(), MatrixBase::applyOnTheRight()
  */
template<typename Scalar>
void JacobiRotation<Scalar>::makeGivens(const Scalar& p, const Scalar& q, Scalar* z)
{
  makeGivens(p, q, z, typename internal::conditional<NumTraits<Scalar>::IsComplex, internal::true_type, internal::false_type>::type());
}


// specialization for complexes
template<typename Scalar>
void JacobiRotation<Scalar>::makeGivens(const Scalar& p, const Scalar& q, Scalar* r, internal::true_type)
{
  if(q==Scalar(0))
  {
    m_c = internal::real(p)<0 ? Scalar(-1) : Scalar(1);
    m_s = 0;
    if(r) *r = m_c * p;
  }
  else if(p==Scalar(0))
  {
    m_c = 0;
    m_s = -q/internal::abs(q);
    if(r) *r = internal::abs(q);
  }
  else
  {
    RealScalar p1 = internal::norm1(p);
    RealScalar q1 = internal::norm1(q);
    if(p1>=q1)
    {
      Scalar ps = p / p1;
      RealScalar p2 = internal::abs2(ps);
      Scalar qs = q / p1;
      RealScalar q2 = internal::abs2(qs);

      RealScalar u = internal::sqrt(RealScalar(1) + q2/p2);
      if(internal::real(p)<RealScalar(0))
        u = -u;

      m_c = Scalar(1)/u;
      m_s = -qs*internal::conj(ps)*(m_c/p2);
      if(r) *r = p * u;
    }
    else
    {
      Scalar ps = p / q1;
      RealScalar p2 = internal::abs2(ps);
      Scalar qs = q / q1;
      RealScalar q2 = internal::abs2(qs);

      RealScalar u = q1 * internal::sqrt(p2 + q2);
      if(internal::real(p)<RealScalar(0))
        u = -u;

      p1 = internal::abs(p);
      ps = p/p1;
      m_c = p1/u;
      m_s = -internal::conj(ps) * (q/u);
      if(r) *r = ps * u;
    }
  }
}

// specialization for reals
template<typename Scalar>
void JacobiRotation<Scalar>::makeGivens(const Scalar& p, const Scalar& q, Scalar* r, internal::false_type)
{

  if(q==Scalar(0))
  {
    m_c = p<Scalar(0) ? Scalar(-1) : Scalar(1);
    m_s = Scalar(0);
    if(r) *r = internal::abs(p);
  }
  else if(p==Scalar(0))
  {
    m_c = Scalar(0);
    m_s = q<Scalar(0) ? Scalar(1) : Scalar(-1);
    if(r) *r = internal::abs(q);
  }
  else if(internal::abs(p) > internal::abs(q))
  {
    Scalar t = q/p;
    Scalar u = internal::sqrt(Scalar(1) + internal::abs2(t));
    if(p<Scalar(0))
      u = -u;
    m_c = Scalar(1)/u;
    m_s = -t * m_c;
    if(r) *r = p * u;
  }
  else
  {
    Scalar t = p/q;
    Scalar u = internal::sqrt(Scalar(1) + internal::abs2(t));
    if(q<Scalar(0))
      u = -u;
    m_s = -Scalar(1)/u;
    m_c = -t * m_s;
    if(r) *r = q * u;
  }

}

/****************************************************************************************
*   Implementation of MatrixBase methods
****************************************************************************************/

/** \jacobi_module
  * Applies the clock wise 2D rotation \a j to the set of 2D vectors of cordinates \a x and \a y:
  * \f$ \left ( \begin{array}{cc} x \\ y \end{array} \right )  =  J \left ( \begin{array}{cc} x \\ y \end{array} \right ) \f$
  *
  * \sa MatrixBase::applyOnTheLeft(), MatrixBase::applyOnTheRight()
  */
namespace internal {
template<typename VectorX, typename VectorY, typename OtherScalar>
void apply_rotation_in_the_plane(VectorX& _x, VectorY& _y, const JacobiRotation<OtherScalar>& j);
}

/** \jacobi_module
  * Applies the rotation in the plane \a j to the rows \a p and \a q of \c *this, i.e., it computes B = J * B,
  * with \f$ B = \left ( \begin{array}{cc} \text{*this.row}(p) \\ \text{*this.row}(q) \end{array} \right ) \f$.
  *
  * \sa class JacobiRotation, MatrixBase::applyOnTheRight(), internal::apply_rotation_in_the_plane()
  */
template<typename Derived>
template<typename OtherScalar>
inline void MatrixBase<Derived>::applyOnTheLeft(Index p, Index q, const JacobiRotation<OtherScalar>& j)
{
  RowXpr x(this->row(p));
  RowXpr y(this->row(q));
  internal::apply_rotation_in_the_plane(x, y, j);
}

/** \ingroup Jacobi_Module
  * Applies the rotation in the plane \a j to the columns \a p and \a q of \c *this, i.e., it computes B = B * J
  * with \f$ B = \left ( \begin{array}{cc} \text{*this.col}(p) & \text{*this.col}(q) \end{array} \right ) \f$.
  *
  * \sa class JacobiRotation, MatrixBase::applyOnTheLeft(), internal::apply_rotation_in_the_plane()
  */
template<typename Derived>
template<typename OtherScalar>
inline void MatrixBase<Derived>::applyOnTheRight(Index p, Index q, const JacobiRotation<OtherScalar>& j)
{
  ColXpr x(this->col(p));
  ColXpr y(this->col(q));
  internal::apply_rotation_in_the_plane(x, y, j.transpose());
}

namespace internal {
template<typename VectorX, typename VectorY, typename OtherScalar>
void /*EIGEN_DONT_INLINE*/ apply_rotation_in_the_plane(VectorX& _x, VectorY& _y, const JacobiRotation<OtherScalar>& j)
{
  typedef typename VectorX::Index Index;
  typedef typename VectorX::Scalar Scalar;
  enum { PacketSize = packet_traits<Scalar>::size };
  typedef typename packet_traits<Scalar>::type Packet;
  eigen_assert(_x.size() == _y.size());
  Index size = _x.size();
  Index incrx = _x.innerStride();
  Index incry = _y.innerStride();

  Scalar* EIGEN_RESTRICT x = &_x.coeffRef(0);
  Scalar* EIGEN_RESTRICT y = &_y.coeffRef(0);

  /*** dynamic-size vectorized paths ***/

  if(VectorX::SizeAtCompileTime == Dynamic &&
    (VectorX::Flags & VectorY::Flags & PacketAccessBit) &&
    ((incrx==1 && incry==1) || PacketSize == 1))
  {
    // both vectors are sequentially stored in memory => vectorization
    enum { Peeling = 2 };

    Index alignedStart = internal::first_aligned(y, size);
    Index alignedEnd = alignedStart + ((size-alignedStart)/PacketSize)*PacketSize;

    const Packet pc = pset1<Packet>(j.c());
    const Packet ps = pset1<Packet>(j.s());
    conj_helper<Packet,Packet,NumTraits<Scalar>::IsComplex,false> pcj;

    for(Index i=0; i<alignedStart; ++i)
    {
      Scalar xi = x[i];
      Scalar yi = y[i];
      x[i] =  j.c() * xi + conj(j.s()) * yi;
      y[i] = -j.s() * xi + conj(j.c()) * yi;
    }

    Scalar* EIGEN_RESTRICT px = x + alignedStart;
    Scalar* EIGEN_RESTRICT py = y + alignedStart;

    if(internal::first_aligned(x, size)==alignedStart)
    {
      for(Index i=alignedStart; i<alignedEnd; i+=PacketSize)
      {
        Packet xi = pload<Packet>(px);
        Packet yi = pload<Packet>(py);
        pstore(px, padd(pmul(pc,xi),pcj.pmul(ps,yi)));
        pstore(py, psub(pcj.pmul(pc,yi),pmul(ps,xi)));
        px += PacketSize;
        py += PacketSize;
      }
    }
    else
    {
      Index peelingEnd = alignedStart + ((size-alignedStart)/(Peeling*PacketSize))*(Peeling*PacketSize);
      for(Index i=alignedStart; i<peelingEnd; i+=Peeling*PacketSize)
      {
        Packet xi   = ploadu<Packet>(px);
        Packet xi1  = ploadu<Packet>(px+PacketSize);
        Packet yi   = pload <Packet>(py);
        Packet yi1  = pload <Packet>(py+PacketSize);
        pstoreu(px, padd(pmul(pc,xi),pcj.pmul(ps,yi)));
        pstoreu(px+PacketSize, padd(pmul(pc,xi1),pcj.pmul(ps,yi1)));
        pstore (py, psub(pcj.pmul(pc,yi),pmul(ps,xi)));
        pstore (py+PacketSize, psub(pcj.pmul(pc,yi1),pmul(ps,xi1)));
        px += Peeling*PacketSize;
        py += Peeling*PacketSize;
      }
      if(alignedEnd!=peelingEnd)
      {
        Packet xi = ploadu<Packet>(x+peelingEnd);
        Packet yi = pload <Packet>(y+peelingEnd);
        pstoreu(x+peelingEnd, padd(pmul(pc,xi),pcj.pmul(ps,yi)));
        pstore (y+peelingEnd, psub(pcj.pmul(pc,yi),pmul(ps,xi)));
      }
    }

    for(Index i=alignedEnd; i<size; ++i)
    {
      Scalar xi = x[i];
      Scalar yi = y[i];
      x[i] =  j.c() * xi + conj(j.s()) * yi;
      y[i] = -j.s() * xi + conj(j.c()) * yi;
    }
  }

  /*** fixed-size vectorized path ***/
  else if(VectorX::SizeAtCompileTime != Dynamic &&
          (VectorX::Flags & VectorY::Flags & PacketAccessBit) &&
          (VectorX::Flags & VectorY::Flags & AlignedBit))
  {
    const Packet pc = pset1<Packet>(j.c());
    const Packet ps = pset1<Packet>(j.s());
    conj_helper<Packet,Packet,NumTraits<Scalar>::IsComplex,false> pcj;
    Scalar* EIGEN_RESTRICT px = x;
    Scalar* EIGEN_RESTRICT py = y;
    for(Index i=0; i<size; i+=PacketSize)
    {
      Packet xi = pload<Packet>(px);
      Packet yi = pload<Packet>(py);
      pstore(px, padd(pmul(pc,xi),pcj.pmul(ps,yi)));
      pstore(py, psub(pcj.pmul(pc,yi),pmul(ps,xi)));
      px += PacketSize;
      py += PacketSize;
    }
  }

  /*** non-vectorized path ***/
  else
  {
    for(Index i=0; i<size; ++i)
    {
      Scalar xi = *x;
      Scalar yi = *y;
      *x =  j.c() * xi + conj(j.s()) * yi;
      *y = -j.s() * xi + conj(j.c()) * yi;
      x += incrx;
      y += incry;
    }
  }
}

} // end namespace internal

} // end namespace Eigen

#endif // EIGEN_JACOBI_H
