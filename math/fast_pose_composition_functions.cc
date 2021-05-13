#include "drake/math/fast_pose_composition_functions.h"

#include <algorithm>

#if defined(__AVX2__) && defined(__FMA__)
#include <cstdint>

#include <immintrin.h>
#endif

namespace drake {
namespace math {

/* The portable C++ versions are always defined. They should be written
to maximize the chance that a dumb compiler can generate fast code.

The AVX functions are optionally defined depending on the compiler flags used
for this compilation unit. If defined, the no-suffix, publicly-visible functions
like ComposeRR() are implemented with the "...Avx" methods, otherwise with the
"...Portable" methods.

The AVX functions are strictly local to this file, but the portable ones are
placed in namespace internal so that they can be unit tested regardless of
whether they are used on this platform to implement the publicly-visible
functions.

Note that, except when explicitly marked "...NoAlias", the methods below must
allow for the output argument's memory to overlap with any of the input
arguments' memory. */

namespace {
/* Dot product of a row of l and a column of m, where both l and m are 3x3s
in column order. */
double row_x_col(const double* l, const double* m) {
  return l[0] * m[0] + l[3] * m[1] + l[6] * m[2];
}

/* @pre R_AC is disjoint in memory from the inputs. */
void ComposeRRNoAlias(const double* R_AB, const double* R_BC, double* R_AC) {
  R_AC[0] = row_x_col(&R_AB[0], &R_BC[0]);
  R_AC[1] = row_x_col(&R_AB[1], &R_BC[0]);
  R_AC[2] = row_x_col(&R_AB[2], &R_BC[0]);
  R_AC[3] = row_x_col(&R_AB[0], &R_BC[3]);
  R_AC[4] = row_x_col(&R_AB[1], &R_BC[3]);
  R_AC[5] = row_x_col(&R_AB[2], &R_BC[3]);
  R_AC[6] = row_x_col(&R_AB[0], &R_BC[6]);
  R_AC[7] = row_x_col(&R_AB[1], &R_BC[6]);
  R_AC[8] = row_x_col(&R_AB[2], &R_BC[6]);
}
}  // namespace

namespace internal {

/* Composition of rotation matrices R_AC = R_AB * R_BC. Each matrix is 9
consecutive doubles in column order. */
void ComposeRRPortable(const double* R_AB, const double* R_BC, double* R_AC) {
  double R_AC_temp[9];  // Protect from overlap with inputs.
  ComposeRRNoAlias(R_AB, R_BC, R_AC_temp);
  std::copy(R_AC_temp, R_AC_temp + 9, R_AC);
}

}  // namespace internal

#if defined(__AVX2__) && defined(__FMA__)

namespace {

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
  constexpr int64_t yes = int64_t(1) << 63;
  constexpr int64_t no  = int64_t(0);
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

}  // namespace

/* Use AVX methods. */
bool IsUsingPortableCompositionMethods() { return false; }

void ComposeRR(const double* R_AB, const double* R_BC, double* R_AC) {
  ComposeRRAvx(R_AB, R_BC, R_AC);
}

#else
/* Use portable methods. */
bool IsUsingPortableCompositionMethods() { return true; }

void ComposeRR(const double* R_AB, const double* R_BC, double* R_AC) {
  internal::ComposeRRPortable(R_AB, R_BC, R_AC);
}
#endif

}  // namespace math
}  // namespace drake
