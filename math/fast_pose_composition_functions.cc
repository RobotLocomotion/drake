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

/* Dot product of a column of l and a column of m, where both l and m are 3x3s
in column order. */
double col_x_col(const double* l, const double* m) {
  return l[0] * m[0] + l[1] * m[1] + l[2] * m[2];
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

/* @pre R_AC is disjoint in memory from the inputs. */
void ComposeRinvRNoAlias(const double* R_BA, const double* R_BC,
                          double* R_AC) {
  R_AC[0] = col_x_col(&R_BA[0], &R_BC[0]);
  R_AC[1] = col_x_col(&R_BA[3], &R_BC[0]);
  R_AC[2] = col_x_col(&R_BA[6], &R_BC[0]);
  R_AC[3] = col_x_col(&R_BA[0], &R_BC[3]);
  R_AC[4] = col_x_col(&R_BA[3], &R_BC[3]);
  R_AC[5] = col_x_col(&R_BA[6], &R_BC[3]);
  R_AC[6] = col_x_col(&R_BA[0], &R_BC[6]);
  R_AC[7] = col_x_col(&R_BA[3], &R_BC[6]);
  R_AC[8] = col_x_col(&R_BA[6], &R_BC[6]);
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

/* Composition of rotation matrices R_AC = R_BA⁻¹ * R_BC. Each matrix is 9
consecutive doubles in column order (the inverse can be viewed as the same
matrix in row order). */
void ComposeRinvRPortable(const double* R_BA, const double* R_BC,
                          double* R_AC) {
  double R_AC_temp[9];  // Protect from overlap with inputs.
  ComposeRinvRNoAlias(R_BA, R_BC, R_AC_temp);
  std::copy(R_AC_temp, R_AC_temp + 9, R_AC);
}

// TODO(sherm1) ComposeXXPortable() and ComposeXinvXPortable().

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

}  // namespace

/* Use AVX methods. */
bool IsUsingPortableCompositionFunctions() { return false; }

void ComposeRR(const double* R_AB, const double* R_BC, double* R_AC) {
  ComposeRRAvx(R_AB, R_BC, R_AC);
}
void ComposeRinvR(const double* R_BA, const double* R_BC, double* R_AC) {
  ComposeRinvRAvx(R_BA, R_BC, R_AC);
}

// TODO(sherm1) ComposeXX() and ComposeXinvX().

#else
/* Use portable functions. */
bool IsUsingPortableCompositionFunctions() { return true; }

void ComposeRR(const double* R_AB, const double* R_BC, double* R_AC) {
  internal::ComposeRRPortable(R_AB, R_BC, R_AC);
}
void ComposeRinvR(const double* R_BA, const double* R_BC, double* R_AC) {
  internal::ComposeRinvRPortable(R_BA, R_BC, R_AC);
}
void ComposeXX(const double* X_AB, const double* X_BC, double* X_AC) {
  internal::ComposeXXPortable(X_AB, X_BC, X_AC);
}
void ComposeXinvX(const double* X_BA, const double* X_BC, double* X_AC) {
  internal::ComposeXinvXPortable(X_BA, X_BC, X_AC);
}
#endif

}  // namespace math
}  // namespace drake
