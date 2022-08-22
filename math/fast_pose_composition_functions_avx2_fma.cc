#include "drake/math/fast_pose_composition_functions_avx2_fma.h"

#if defined(__AVX2__) && defined(__FMA__)
#include <cstdint>

#include <cpuid.h>
#include <immintrin.h>
#else
#include <iostream>
#endif

/* N.B. Do not include any other drake headers here because this file will be
part of a compilation unit that may have a different opinion about whether SIMD
instructions are enabled than Eigen does in the rest of Drake. */

namespace drake {
namespace math {
namespace internal {

#if defined(__AVX2__) && defined(__FMA__)
namespace {
/* Reinterpret user-friendly class names to raw arrays of double.

We make judicious use below of potentially-dangerous reinterpret_casts to
convert from user-friendly APIs written in terms of RotationMatrix& and
RigidTransform& (whose declarations are necessarily unknown here) to
implementation-friendly double* types. Why this is safe here:
  - our implementations of these classes guarantee a particular memory
    layout on which we can depend,
  - the address of a class is the address of its first member (mandated by
    the standard), and
  - reinterpret_cast of a pointer to another pointer type and back yields the
    same pointer, i.e. the bit pattern does not change.
*/

const double* GetRawMatrixStart(const RotationMatrix<double>& R) {
  return reinterpret_cast<const double*>(&R);
}

double* GetMutableRawMatrixStart(RotationMatrix<double>* R) {
  return reinterpret_cast<double*>(R);
}

const double* GetRawMatrixStart(const RigidTransform<double>& X) {
  return reinterpret_cast<const double*>(&X);
}

double* GetMutableRawMatrixStart(RigidTransform<double>* X) {
  return reinterpret_cast<double*>(X);
}

// Check if AVX2 is supported by the CPU. We can assume that OS support for AVX2
// is available if AVX2 is supported by hardware, and do not need to test if it
// is enabled in software as well.
bool CheckCpuForAvxSupport() {
  return __builtin_cpu_supports("avx2");
}

// Turn d into d d d d.
__m256d four(double d) {
  return _mm256_set1_pd(d);
}

/* Composition of rotation matrices R_AC = R_AB * R_BC.
Each matrix is 9 consecutive doubles in column order.

We want to perform this 3x3 matrix multiply:

     R_AC    R_AB    R_BC

    r u x   a d g   A D G
    s v y = b e h * B E H     All column ordered in memory.
    t w z   c f i   C F I

Strategy: compute column rst in parallel, then uvw, then xyz.
r = aA+dB+gC
s = bA+eB+hC  etc.
t = cA+fB+iC

Load columns from left matrix, duplicate elements from right. Perform 4
operations in parallel but ignore the 4th result. Be careful not to load or
store past the last element.

This requires 45 flops (9 dot products) but we're doing 60 here and throwing
away 15 of them. However, we only issue 9 floating point instructions. */
void ComposeRRAvx(const double* R_AB, const double* R_BC, double* R_AC) {
  constexpr uint64_t yes = uint64_t(1ull << 63);
  constexpr uint64_t no  = uint64_t(0);
  const __m256i mask = _mm256_setr_epi64x(yes, yes, yes, no);

  // Aliases for readability (named after first entries in comment above).
  const double* a = R_AB; const double* A = R_BC; double* r = R_AC;

  const __m256d col0 = _mm256_loadu_pd(a);             // a b c (d)  d unused
  const __m256d col1 = _mm256_loadu_pd(a+3);           // d e f (g)  g unused
  const __m256d col2 = _mm256_maskload_pd(a+6, mask);  // g h i (0)

  const __m256d ABCD = _mm256_loadu_pd(A);             // A B C D
  const __m256d EFGH = _mm256_loadu_pd(A+4);           // E F G H
  const double I = *(A+8);                             // I

  __m256d res;

  // Column rst                                         r   s   t   (-)
  res = _mm256_mul_pd(col0, four(ABCD[0]));         //  aA  bA  cA ( dA)
  res = _mm256_fmadd_pd(col1, four(ABCD[1]), res);  // +dB +eB +fB (+gB)
  res = _mm256_fmadd_pd(col2, four(ABCD[2]), res);  // +gC +hC +iC (+0C)
  _mm256_storeu_pd(r, res);  // r s t (u)  we will overwrite u

  // Column uvw                                         u   v   w   (-)
  res = _mm256_mul_pd(col0, four(ABCD[3]));         //  aD  bD  cD ( dD)
  res = _mm256_fmadd_pd(col1, four(EFGH[0]), res);  // +dE +eE +fE (+gE)
  res = _mm256_fmadd_pd(col2, four(EFGH[1]), res);  // +gF +hF +iF (+0F)
  _mm256_storeu_pd(r+3, res);  // u v w (x)  we will overwrite x

  // Column xyz                                         x   y   z   (-)
  res = _mm256_mul_pd(col0, four(EFGH[2]));         //  aG  bG  cG ( dG)
  res = _mm256_fmadd_pd(col1, four(EFGH[3]), res);  // +dH +eH +fH (+gH)
  res = _mm256_fmadd_pd(col2, four(I), res);        // +gI +hI +iI (+0I)
  _mm256_maskstore_pd(r+6, mask, res);  // x y z

  // The compiler will generate a vzeroupper instruction if needed.
}

/* Composition of rotation matrices R_AC = R_BA⁻¹ * R_BC.
Each matrix is 9 consecutive doubles in column order.

We want to perform this 3x3 matrix multiply:

     R_AC    R_BA⁻¹  R_BC

    r u x   a b c   A D G
    s v y = d e f * B E H     R_BA⁻¹ is row ordered; R_BC col ordered
    t w z   g h i   C F I

This is 45 flops altogether.

Strategy: compute column rst in parallel, then uvw, then xyz.
r = aA+bB+cC
s = dA+eB+fC  etc.
t = gA+hB+iC

Load columns from left matrix, duplicate elements from right. (Tricky here since
the inverse is row ordered -- we pull in the rows and then shuffle things.)
Peform 4 operations in parallel but ignore the 4th result. Be careful not to
load or store past the last element.

We end up doing 3*4 + 6*8 = 60 flops to get 45 useful ones. However, we do that
in only 9 floating point instructions (3 packed multiplies, 6 packed
fused-multiply-adds).

It is OK if the result R_AC overlaps one or both of the inputs. */
void ComposeRinvRAvx(const double* R_BA, const double* R_BC, double* R_AC) {
  constexpr uint64_t yes = uint64_t(1ull << 63);
  constexpr uint64_t no  = uint64_t(0);
  const __m256i mask = _mm256_setr_epi64x(yes, yes, yes, no);

  // Aliases for readability (named after first entries in comment above).
  const double* a = R_BA; const double* A = R_BC; double* r = R_AC;

  __m256d a0, a1, a2;  // Accumulators.

  // Load columns of R_AB (rows of the given R_BA) into registers via a
  // series of loads, blends, and permutes. See above for the meaning of these
  // element names.
  const __m256d abcd = _mm256_loadu_pd(a);                   // a b c d
  const __m256d efgh = _mm256_loadu_pd(a+4);                 // e f g h
  const __m256d ebgh = _mm256_blend_pd(abcd, efgh, 0b1101);  // e b g h
  const __m256d behg = _mm256_permute_pd(ebgh, 0b0101);      // b e h (g) col1

  const __m256d cdgh = _mm256_permute2f128_pd(abcd, efgh,
                                              0b00110001);   // c d g h
  const __m256d adgh = _mm256_blend_pd(abcd, cdgh, 0b1110);  // a d g (h) col0

  const __m256d fghi = _mm256_loadu_pd(a+5);                 // f g h i
  const __m256d ffih = _mm256_permute_pd(fghi, 0b0100);      // f f i h
  const __m256d cfih = _mm256_blend_pd(cdgh, ffih, 0b0110);  // c f i (h) col2

  const __m256d ABCD = _mm256_loadu_pd(A);     // A B C D
  const __m256d EFGH = _mm256_loadu_pd(A+4);   // E F G H
  const double I = *(A+8);                     // I

  a0 = _mm256_mul_pd(adgh, four(ABCD[0]));   // aA dA gA (hA)
  a1 = _mm256_mul_pd(adgh, four(ABCD[3]));   // aD dD gD (hD)
  a2 = _mm256_mul_pd(adgh, four(EFGH[2]));   // aG dG gG (hG)

  a0 = _mm256_fmadd_pd(behg, four(ABCD[1]), a0);  // aA+bB dA+eB gA+hB (hA+gB)
  a1 = _mm256_fmadd_pd(behg, four(EFGH[0]), a1);  // aD+bE dD+eE gD+hE (hD+gE)
  a2 = _mm256_fmadd_pd(behg, four(EFGH[3]), a2);  // aG+bH dG+eH gG+hH (hG+gH)

  a0 = _mm256_fmadd_pd(cfih, four(ABCD[2]), a0);  // aA+bB+cC dA+eB+fC gA+hB+iC
                                                  //                  (hA+cB+hC)
  _mm256_storeu_pd(r, a0);                        // r s t (u) will overwrite u

  a1 = _mm256_fmadd_pd(cfih, four(EFGH[1]), a1);  // aD+bE+cF dD+eE+fF gD+hE+iF
                                                  //                  (hD+cE+hF)
  _mm256_storeu_pd(r+3, a1);                      // u v w (x) will overwrite x

  a2 = _mm256_fmadd_pd(cfih, four(I), a2);        // aG+bH+cI dG+eH+fI gG+hH+iI
                                                  //                  (hG+cH+hI)
  _mm256_maskstore_pd(r+6, mask, a2);             // x y z

  // The compiler will generate a vzeroupper instruction if needed.
}

/* Composition of transforms X_AC = X_AB * X_BC.
The rotation matrix and offset vector occupy 12 consecutive doubles.

For the code below, consider the elements of these 3x4 matrices to be named
as follows:

       X_AC       X_AB      X_BC
    r u x' xx   a d g x   A D G X
    s v y' yy   b e h y   B E H Y    All column ordered in memory.
    t w z' zz   c f i z   C F I Z

(Sorry about the ugly symbols on the left -- I ran out of letters.)

We will handle the rotation matrix and offset vector separately.

We want to perform this 3x3 matrix multiply:

     R_AC    R_AB    R_BC

    r u x'   a d g   A D G
    s v y' = b e h * B E H     All column ordered in memory.
    t w z'   c f i   C F I

and    xx   x   a d g   X
       yy = y + b e h * Y
       zz   z   c f i   Z

This is 63 flops altogether.

Strategy: compute column rst in parallel, then uvw, then xyz.
r = aA+dB+gC          xx = x + aX + dY + gZ
s = bA+eB+hC  etc.    yy = y + bX + eY + hZ
t = cA+fB+iC          zz = z + cX + fY + iZ

Load columns from left matrix, duplicate elements from right.
Peform 4 operations in parallel but ignore the 4th result.
Be careful not to load or store past the last element.

We end up doing 3*4 + 9*8 = 84 flops to get 63 useful ones.
However, we do that in only 12 floating point instructions
(three packed multiplies, nine packed fused-multiply-adds). */
void ComposeXXAvx(const double* X_AB, const double* X_BC, double* X_AC) {
  constexpr uint64_t yes = uint64_t(1ull << 63);
  constexpr uint64_t no  = uint64_t(0);
  const __m256i mask = _mm256_setr_epi64x(yes, yes, yes, no);

  // Aliases for readability (named after first entries in comment above).
  const double* a = X_AB; const double* A = X_BC; double* r = X_AC;

  const __m256d abcd = _mm256_loadu_pd(a);             // a b c (d)  d unused
  const __m256d xyz0 = _mm256_maskload_pd(a+9, mask);  // x y z (0)

  const __m256d ABCD = _mm256_loadu_pd(A);             // A B C D
  const __m256d EFGH = _mm256_loadu_pd(A+4);           // E F G H
  const __m256d IXYZ = _mm256_loadu_pd(A+8);           // I X Y Z

  __m256d a0, a1, a2, a3;  // Accumulators.

  a0 = _mm256_mul_pd(abcd, four(ABCD[0]));          // aA bA cA (dA)
  a1 = _mm256_mul_pd(abcd, four(ABCD[3]));          // aD bD cD (dD)
  a2 = _mm256_mul_pd(abcd, four(EFGH[2]));          // aG bG cG (dG)
  a3 = _mm256_fmadd_pd(abcd, four(IXYZ[1]), xyz0);  // x+aX y+bX z+cX (0+dX)

  const __m256d defg = _mm256_loadu_pd(a+3);      // d e f (g)  g is unused
  a0 = _mm256_fmadd_pd(defg, four(ABCD[1]), a0);  // aA+dB bA+eB cA+fB (dA+gB)
  a1 = _mm256_fmadd_pd(defg, four(EFGH[0]), a1);  // aD+dE bD+eE cD+fE (dD+gE)
  a2 = _mm256_fmadd_pd(defg, four(EFGH[3]), a2);  // aG+dH bG+eH cG+fH (dG+gH)
  a3 = _mm256_fmadd_pd(defg, four(IXYZ[2]), a3);  // x+aX+dY y+bX+eY z+cX+fY
                                                  //                   (0+dX+gY)

  const __m256d ghix = _mm256_loadu_pd(a+6);      // g h i (x)
  a0 = _mm256_fmadd_pd(ghix, four(ABCD[2]), a0);  // aA+dB+gC bA+eB+hC cA+fB+iC
                                                  //                  (dA+gB+xC)
  _mm256_storeu_pd(r, a0);                        // r s t (u) will overwrite u

  a1 = _mm256_fmadd_pd(ghix, four(EFGH[1]), a1);  // aD+dE+gF bD+eE+hF cD+fE+iF
                                                  //                  (dD+gE+0F)
  _mm256_storeu_pd(r+3, a1);                      // u v w (x) will overwrite x

  a2 = _mm256_fmadd_pd(ghix, four(IXYZ[0]), a2);  // aG+dH+gI bG+eH+hI cG+fH+iI
                                                  //                  (dG+gH+0I)
  _mm256_storeu_pd(r+6, a2);                      // x' y' z' (xx)
                                                  //           will overwrite xx

  a3 = _mm256_fmadd_pd(ghix, four(IXYZ[3]), a3);  // x+aX+dY+gZ y+bX+eY+hZ
                                                  //     z+cX+fY+iZ (0+dX+gY+xZ)
  _mm256_maskstore_pd(r+9, mask, a3);             // xx yy zz

  // The compiler will generate a vzeroupper instruction if needed.
}

/* Composition of transforms X_AC = X_BA⁻¹ * X_BC.
The rotation matrix and offset vector occupy 12 consecutive doubles. See
previous method for notation used here.

We want to perform this 3x3 matrix multiply:

     R_AC    R_BA⁻¹  R_BC

    r u x'   a b c   A D G
    s v y' = d e f * B E H     R_BA⁻¹ is row ordered; R_BC col ordered
    t w z'   g h i   C F I

and    xx   a b c     X   x
       yy = d e f * ( Y − y )
       zz   g h i     Z   z

This is 63 flops altogether.

Strategy: compute column rst in parallel, then uvw, then xyz.
Let PQR=XYZ-xyz.
r = aA+bB+cC          xx = aP + bQ + cR
s = dA+eB+fC  etc.    yy = dP + eQ + fR
t = gA+hB+iC          zz = gP + hQ + iR

Load columns from left matrix, duplicate elements from right.
(Tricky here since the inverse is row ordered -- we pull in
the rows and then shuffle things.)
Peform 4 operations in parallel but ignore the 4th result.
Be careful not to load or store past the last element.

We end up doing 5*4 + 8*8 = 84 flops to get 63 useful ones.
However, we do that in only 13 floating point instructions
(4 packed multiplies, 1 packed subtract, 8 packed fused-multiply-adds).
*/
void ComposeXinvXAvx(const double* X_BA, const double* X_BC, double* X_AC) {
  constexpr uint64_t yes = uint64_t(1ull << 63);
  constexpr uint64_t no  = uint64_t(0);
  const __m256i mask = _mm256_setr_epi64x(yes, yes, yes, no);

  // Aliases for readability (named after first entries in comment above).
  const double* a = X_BA; const double* A = X_BC; double* r = X_AC;

  const __m256d abcd = _mm256_loadu_pd(a);                   // a b c d
  const __m256d efgh = _mm256_loadu_pd(a+4);                 // e f g h
  const __m256d ebgh = _mm256_blend_pd(abcd, efgh, 0b1101);  // e b g h
  const __m256d behg = _mm256_permute_pd(ebgh, 0b0101);      // b e h (g) col1

  const __m256d cdgh = _mm256_permute2f128_pd(abcd, efgh,
                                              0b00110001);   // c d g h
  const __m256d adgh = _mm256_blend_pd(abcd, cdgh, 0b1110);  // a d g (h) col0

  const __m256d fghi = _mm256_loadu_pd(a+5);
  const __m256d ffih = _mm256_permute_pd(fghi, 0b0100);      // f f i h
  const __m256d cfih = _mm256_blend_pd(cdgh, ffih, 0b0110);  // c f i (h) col2

  const __m256d IXYZ = _mm256_loadu_pd(A+8);       // I X Y Z
  const __m256d ixyz = _mm256_loadu_pd(a+8);       // i x y z
  const __m256d _PQR = _mm256_sub_pd(IXYZ, ixyz);  // _PQR = (I)XYZ − (i)xyz
  const __m256d ABCD = _mm256_loadu_pd(A);         // A B C D
  const __m256d EFGH = _mm256_loadu_pd(A+4);       // E F G H

  __m256d a0, a1, a2, a3;  // Accumulators.

  a0 = _mm256_mul_pd(adgh, four(ABCD[0]));        // aA dA gA (hA)
  a1 = _mm256_mul_pd(adgh, four(ABCD[3]));        // aD dD gD (hD)
  a2 = _mm256_mul_pd(adgh, four(EFGH[2]));        // aG dG gG (hG)
  a3 = _mm256_mul_pd(adgh, four(_PQR[1]));        // aP dP gP (hP)

  a0 = _mm256_fmadd_pd(behg, four(ABCD[1]), a0);  // aA+bB dA+eB gA+hB (hA+gB)
  a1 = _mm256_fmadd_pd(behg, four(EFGH[0]), a1);  // aD+bE dD+eE gD+hE (hD+gE)
  a2 = _mm256_fmadd_pd(behg, four(EFGH[3]), a2);  // aG+bH dG+eH gG+hH (hG+gH)
  a3 = _mm256_fmadd_pd(behg, four(_PQR[2]), a3);  // aP+bQ dP+eQ gP+hQ (hP+gQ)

  a0 = _mm256_fmadd_pd(cfih, four(ABCD[2]), a0);  // aA+bB+cC dA+eB+fC gA+hB+iC
                                                  //                  (hA+cB+hC)
  _mm256_storeu_pd(r, a0);                        // r s t (u) will overwrite u

  a1 = _mm256_fmadd_pd(cfih, four(EFGH[1]), a1);  // aD+bE+cF dD+eE+fF gD+hE+iF
                                                  //                  (hD+cE+hF)
  _mm256_storeu_pd(r+3, a1);                      // u v w (x) will overwrite x

  a2 = _mm256_fmadd_pd(cfih, four(IXYZ[0]), a2);  // aG+bH+cI dG+eH+fI gG+hH+iI
                                                  //                  (hG+cH+hI)
  _mm256_storeu_pd(r+6, a2);                      // x' y' z' (xx)
                                                  //           will overwrite xx

  a3 = _mm256_fmadd_pd(cfih, four(_PQR[3]), a3);  // aP+bQ+cR dP+eQ+fR gP+hQ+iR
                                                  //                  (hP+cQ+hR)
  _mm256_maskstore_pd(r+9, mask, a3);             // xx yy zz

  // The compiler will generate a vzeroupper instruction if needed.
}

}  // namespace

// See note above as to why these reinterpret_casts are safe.

bool AvxSupported() {
  static const bool avx_supported = CheckCpuForAvxSupport();
  return avx_supported;
}

void ComposeRRAvx(const RotationMatrix<double>& R_AB,
                  const RotationMatrix<double>& R_BC,
                  RotationMatrix<double>* R_AC) {
  ComposeRRAvx(GetRawMatrixStart(R_AB), GetRawMatrixStart(R_BC),
               GetMutableRawMatrixStart(R_AC));
}

void ComposeRinvRAvx(const RotationMatrix<double>& R_BA,
                     const RotationMatrix<double>& R_BC,
                     RotationMatrix<double>* R_AC) {
  ComposeRinvRAvx(GetRawMatrixStart(R_BA), GetRawMatrixStart(R_BC),
                  GetMutableRawMatrixStart(R_AC));
}

void ComposeXXAvx(const RigidTransform<double>& X_AB,
                  const RigidTransform<double>& X_BC,
                  RigidTransform<double>* X_AC) {
  ComposeXXAvx(GetRawMatrixStart(X_AB), GetRawMatrixStart(X_BC),
               GetMutableRawMatrixStart(X_AC));
}

void ComposeXinvXAvx(const RigidTransform<double>& X_BA,
                     const RigidTransform<double>& X_BC,
                     RigidTransform<double>* X_AC) {
  ComposeXinvXAvx(GetRawMatrixStart(X_BA), GetRawMatrixStart(X_BC),
                  GetMutableRawMatrixStart(X_AC));
}
#else
namespace {
void AbortNotEnabledInBuild(const char* func) {
  std::cerr << "abort: " << func << " is not enabled in build" << std::endl;
  std::abort();
}
}  // namespace

bool AvxSupported() { return false; }

void ComposeRRAvx(const RotationMatrix<double>&,
                  const RotationMatrix<double>&,
                  RotationMatrix<double>*) {
  AbortNotEnabledInBuild(__func__);
}

void ComposeRinvRAvx(const RotationMatrix<double>&,
                     const RotationMatrix<double>&,
                     RotationMatrix<double>*) {
  AbortNotEnabledInBuild(__func__);
}

void ComposeXXAvx(const RigidTransform<double>&,
                  const RigidTransform<double>&,
                  RigidTransform<double>*) {
  AbortNotEnabledInBuild(__func__);
}

void ComposeXinvXAvx(const RigidTransform<double>&,
                     const RigidTransform<double>&,
                     RigidTransform<double>*) {
  AbortNotEnabledInBuild(__func__);
}
#endif

}  // namespace internal
}  // namespace math
}  // namespace drake
