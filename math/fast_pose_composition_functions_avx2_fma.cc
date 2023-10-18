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
  __builtin_cpu_init();
  return __builtin_cpu_supports("avx2");
}

/* =============================================================================
   Notation conventions used in this file
================================================================================

In comments, we'll use "<....>" to indicate a 256-bit AVX register containing 4
lanes of doubles, so e.g. <abcd> denotes the array of doubles [a, b, c, d]. We
use both lowercase and uppercase letters for naming lanes. In variable names,
we leave out the <> marker so the variable name would be just "abcd".

The lower order lanes are spelled on the left, so for example if YMM0 = <abcd>
then we have XMM0 = <ab>. Note that even though we spell lanes from lowest to
highest, bitwise constants end up being written highest to lowest (as is typical
for numbers) so we have blend(<abcd>, <ABCD>, 0b0101) as <AbCd>.

Sometimes we don't need to give a name to a lane (because we don't care what its
value is). In that case we use the canonical "_" to indicate "don't care", e.g.,
<abc_> indicates the array of doubles [a, b, c, _] where we only care about the
first three lanes. This leads to variables named e.g. "abc_" which look like C++
member fields, but are not.

For details about the various SIMD instructions, see:

 https://www.intel.com/content/www/us/en/docs/intrinsics-guide/index.html

 https://www.agner.org/optimize/instruction_tables.pdf

For inspecting the machine code, see Godbolt and in particular the LLVM-MCA tool
(available under the "Add Tool" menu).

 https://godbolt.org/

============================================================================= */

// Turns d into <dddd>.
__m256d four(double d) {
  return _mm256_set1_pd(d);
}

/* Composition of rotation matrices R_AC = R_AB * R_BC.

Each matrix is 9 consecutive doubles in column-major order.

  R_AB = abcdefghi
  R_BC = ABCDEFGHI
  R_AC = jklmnopqr

We want to perform this 3x3 matrix multiplication:

   R_AC      R_AB      R_BC

  j m p     a d g     A D G
  k n q  =  b e h  @  B E H
  l o r     c f i     C F I

Writing that out longhand, we have:

  j = aA + dB + gC
  k = bA + eB + hC
  l = cA + fB + iC

  m = aD + dE + gF
  n = bD + eE + hF
  o = cD + fE + iF

  p = aG + dH + gI
  q = bG + eH + hI
  r = cG + fH + iI

Strategy: compute jkl in parallel, then mno, then pqr.

  <jkl_> =   <abc_> * <AAA_>
           + <def_> * <BBB_>
           + <ghi_> * <CCC_>

  <mno_> =   <abc_> * <DDD_>
           + <def_> * <EEE_>
           + <ghi_> * <FFF_>

  <pqr_> =   <abc_> * <GGG_>
           + <def_> * <HHH_>
           + <ghi_> * <III_>

We load columns from left matrix and broadcast elements from right, performing
4 operations in parallel but ignoring the 4th result. We must be careful not to
load or store past the last element.

Rotation matrix composition requires 45 flops (27 multiplies and 18 additions)
but we're doing 60 here and throwing away 15 of them. However, we only issue 9
SIMD floating point instructions (3 multiplies and 6 fused-multiply-adds).

It is OK if the result R_AC overlaps one or both of the inputs. */
void ComposeRRAvx(const double* R_AB, const double* R_BC, double* R_AC) {
  constexpr uint64_t yes = uint64_t(1ull << 63);
  constexpr uint64_t no = uint64_t(0);
  const __m256i mask = _mm256_setr_epi64x(yes, yes, yes, no);

  // Load the input.
  const __m256d abc_ = _mm256_loadu_pd(R_AB);      // (d is loaded but unused)
  const __m256d def_ = _mm256_loadu_pd(R_AB + 3);  // (g is loaded but unused)
  const __m256d ghi_ = _mm256_maskload_pd(R_AB + 6, mask);
  const double A = R_BC[0];
  const double B = R_BC[1];
  const double C = R_BC[2];
  const double D = R_BC[3];
  const double E = R_BC[4];
  const double F = R_BC[5];
  const double G = R_BC[6];
  const double H = R_BC[7];
  const double I = R_BC[8];

  // Column jkl:                                    j   k   l   _  =
  __m256d jkl_ = _mm256_mul_pd(abc_, four(A));  //  aA  bA  cA  _
  jkl_ = _mm256_fmadd_pd(def_, four(B), jkl_);  // +dB +eB +fB  _
  jkl_ = _mm256_fmadd_pd(ghi_, four(C), jkl_);  // +gC +hC +iC  _

  // Column mno:                                    m   n   o   _  =
  __m256d mno_ = _mm256_mul_pd(abc_, four(D));  //  aD  bD  cD  _
  mno_ = _mm256_fmadd_pd(def_, four(E), mno_);  // +dE +eE +fE  _
  mno_ = _mm256_fmadd_pd(ghi_, four(F), mno_);  // +gF +hF +iF  _

  // Column pqr:                                    p   q   r   _  =
  __m256d pqr_ = _mm256_mul_pd(abc_, four(G));  //  aG  bG  cG  _
  pqr_ = _mm256_fmadd_pd(def_, four(H), pqr_);  // +dH +eH +fH  _
  pqr_ = _mm256_fmadd_pd(ghi_, four(I), pqr_);  // +gI +hI +iI  _

  // Store the output.
  _mm256_storeu_pd(R_AC, jkl_);      // 4-wide write temporarily overwrites m
  _mm256_storeu_pd(R_AC + 3, mno_);  // 4-wide write temporarily overwrites p
  _mm256_maskstore_pd(R_AC + 6, mask, pqr_);  // 3-wide write to stay in bounds

  // The compiler will generate a vzeroupper instruction if needed.
}

/* Composition of rotation matrices R_AC = R_BA⁻¹ * R_BC.

Each matrix is 9 consecutive doubles in column-major order.

  R_BA = abcdefghi
  R_BC = ABCDEFGHI
  R_AC = jklmnopqr

We want to perform this 3x3 matrix multiplication:

   R_AC      R_BA⁻¹    R_BC

  j m p     a b c     A D G          Note that R_BA⁻¹ (aka R_AB) is in effect
  k n q  =  d e f  @  B E H          stored in row-major order, since our
  l o r     g h i     C F I          argument R_BA was in column-major order.

Writing that out longhand, we have:

  j = aA + bB + cC
  k = dA + eB + fC
  l = gA + hB + iC

  m = aD + bE + cF
  n = dD + eE + fF
  o = gD + hE + iF

  p = aG + bH + cI
  q = dG + eH + fI
  r = gG + hH + iI

Strategy: compute jkl in parallel, then mno, then pqr.

  <jkl_> =   <adg_> * <AAA_>
           + <beh_> * <BBB_>
           + <cfi_> * <CCC_>

  <mno_> =   <adg_> * <DDD_>
           + <beh_> * <EEE_>
           + <cfi_> * <FFF_>

  <pqr_> =   <adg_> * <GGG_>
           + <beh_> * <HHH_>
           + <cfi_> * <III_>

We load columns from left matrix (tricky here since the inverse is row-major --
we'll pull in contiguous words and then shuffle) and broadcast elements from
right, performing 4 operations in parallel but ignoring the 4th result. We must
be careful not to load or store past the last element.

Rotation matrix composition requires 45 flops (27 multiplies and 18 additions)
but we're doing 60 here and throwing away 15 of them. However, we only issue 9
SIMD floating point instructions (3 multiplies and 6 fused-multiply-adds).

It is OK if the result R_AC overlaps one or both of the inputs. */
void ComposeRinvRAvx(const double* R_BA, const double* R_BC, double* R_AC) {
  constexpr uint64_t yes = uint64_t(1ull << 63);
  constexpr uint64_t no = uint64_t(0);
  const __m256i mask = _mm256_setr_epi64x(yes, yes, yes, no);

  // Load the input. Loads columns of R_AB (rows of the given R_BA) into
  // registers via a series of loads, blends, and permutes.
  const __m256d abcd = _mm256_loadu_pd(R_BA);
  const __m256d efgh = _mm256_loadu_pd(R_BA + 4);
  const __m256d eb_h = _mm256_blend_pd(abcd, efgh, 0b1101);
  const __m256d beh_ = _mm256_permute_pd(eb_h, 0b0101);
  const __m256d cdg_ = _mm256_permute2f128_pd(abcd, efgh, 0b00110001);
  const __m256d adg_ = _mm256_blend_pd(abcd, cdg_, 0b1110);
  const __m256d fghi = _mm256_loadu_pd(R_BA + 5);
  const __m256d _fi_ = _mm256_permute_pd(fghi, 0b0100);
  const __m256d cfi_ = _mm256_blend_pd(cdg_, _fi_, 0b0110);
  const double A = R_BC[0];
  const double B = R_BC[1];
  const double C = R_BC[2];
  const double D = R_BC[3];
  const double E = R_BC[4];
  const double F = R_BC[5];
  const double G = R_BC[6];
  const double H = R_BC[7];
  const double I = R_BC[8];

  // Column jkl:                                    j   k   l   _  =
  __m256d jkl_ = _mm256_mul_pd(adg_, four(A));  //  aA  dA  gA  _
  jkl_ = _mm256_fmadd_pd(beh_, four(B), jkl_);  // +bB +eB +hB  _
  jkl_ = _mm256_fmadd_pd(cfi_, four(C), jkl_);  // +cC +fC +iC  _

  // Column mno:                                    m   n   o   _  =
  __m256d mno_ = _mm256_mul_pd(adg_, four(D));  //  aD  dD  gD  _
  mno_ = _mm256_fmadd_pd(beh_, four(E), mno_);  // +bE +eE +hE  _
  mno_ = _mm256_fmadd_pd(cfi_, four(F), mno_);  // +cF +fF +iF  _

  // Column pqr:                                    p   q   r   _  =
  __m256d pqr_ = _mm256_mul_pd(adg_, four(G));  //  aG  dG  gG  _
  pqr_ = _mm256_fmadd_pd(beh_, four(H), pqr_);  // +bH +eH +hH  _
  pqr_ = _mm256_fmadd_pd(cfi_, four(I), pqr_);  // +cI +fI +iI  _

  // Store the output.
  _mm256_storeu_pd(R_AC, jkl_);      // 4-wide write temporarily overwrites m
  _mm256_storeu_pd(R_AC + 3, mno_);  // 4-wide write temporarily overwrites p
  _mm256_maskstore_pd(R_AC + 6, mask, pqr_);  // 3-wide write to stay in bounds

  // The compiler will generate a vzeroupper instruction if needed.
}

/* Composition of transforms X_AC = X_AB * X_BC.

Each matrix is 12 consecutive doubles in column-major order.

  X_AB = abcdefghixyz
  X_BC = ABCDEFGHIXYZ
  X_AC = jklmnopqrstu

We want to perform this 4x4 matrix multiplication:

      X_AC        X_AB        X_BC
    j m p s     a d g x     A D G X
    k n q t  =  b e h y  @  B E H Y
    l o r u     c f i z     C F I Z
    0 0 0 1     0 0 0 1     0 0 0 1

(Note that the "0 0 0 1" is only an illustration of the math; our transform
matrix does not actually store that row.)

Instead of calculating the full 4x4 multiply, we can take advantage of the
implicit constants in the last row to handle the rotation matrix and translation
vector separately.

We'll perform this 3x3 matrix multiplication:

     R_AC      R_AB      R_BC

    j m p     a d g     A D G
    k n q  =  b e h  @  B E H
    l o r     c f i     C F I

and this translation vector multiply-accumulate:

    s     x     a d g     X
    t  =  y  +  b e h  @  Y
    u     z     c f i     Z

Writing that out longhand, we have:

  j = aA + dB + gC
  k = bA + eB + hC
  l = cA + fB + iC

  m = aD + dE + gF
  n = bD + eE + hF
  o = cD + fE + iF

  p = aG + dH + gI
  q = bG + eH + hI
  r = cG + fH + iI

  s = x + aX + dY + gZ
  t = y + bX + eY + hZ
  u = z + cX + fY + iZ

Strategy: compute stu in parallel, then jkl parallel, then mno, then pqr.
The <stu_> vector has the deepest pipeline, so we get it going first.

  <stu_> =   <xyz_>
           + <abc_> * <XXX_>
           + <def_> * <YYY_>
           + <ghi_> * <ZZZ_>

  <jkl_> =   <adg_> * <AAA_>
           + <beh_> * <BBB_>
           + <cfi_> * <CCC_>

  <mno_> =   <adg_> * <DDD_>
           + <beh_> * <EEE_>
           + <cfi_> * <FFF_>

  <pqr_> =   <adg_> * <GGG_>
           + <beh_> * <HHH_>
           + <cfi_> * <III_>

We load columns from left matrix and broadcast elements from right, performing
4 operations in parallel but ignoring the 4th result. We must be careful not to
load or store past the last element.

Rigid transform composition requires 63 flops (36 multiplies and 27 additions)
but we're doing 84 here and throwing away 21 of them. However, we only issue 12
SIMD floating point instructions (3 multiplies and 9 fused-multiply-adds).

It is OK if the result X_AC overlaps one or both of the inputs. */
void ComposeXXAvx(const double* X_AB, const double* X_BC, double* X_AC) {
  constexpr uint64_t yes = uint64_t(1ull << 63);
  constexpr uint64_t no = uint64_t(0);
  const __m256i mask = _mm256_setr_epi64x(yes, yes, yes, no);

  // Load the input.
  const __m256d abc_ = _mm256_loadu_pd(X_AB);      // (d is loaded but unused)
  const __m256d def_ = _mm256_loadu_pd(X_AB + 3);  // (g is loaded but unused)
  const __m256d ghi_ = _mm256_loadu_pd(X_AB + 6);  // (x is loaded but unused)
  const __m256d xyz_ = _mm256_maskload_pd(X_AB + 9, mask);
  const double A = X_BC[0];
  const double B = X_BC[1];
  const double C = X_BC[2];
  const double D = X_BC[3];
  const double E = X_BC[4];
  const double F = X_BC[5];
  const double G = X_BC[6];
  const double H = X_BC[7];
  const double I = X_BC[8];
  const double X = X_BC[9];
  const double Y = X_BC[10];
  const double Z = X_BC[11];

  // Column stu:                                    s   t   u   _  =
  __m256d stu_ = xyz_;                          //  x   y   z   _
  stu_ = _mm256_fmadd_pd(abc_, four(X), stu_);  // +aX +bX +cX  _
  stu_ = _mm256_fmadd_pd(def_, four(Y), stu_);  // +dY +eY +fY  _
  stu_ = _mm256_fmadd_pd(ghi_, four(Z), stu_);  // +gZ +hZ +iZ  _

  // Column jkl:                                    j   k   l   _  =
  __m256d jkl_ = _mm256_mul_pd(abc_, four(A));  //  aA  bA  cA  _
  jkl_ = _mm256_fmadd_pd(def_, four(B), jkl_);  // +dB +eB +fB  _
  jkl_ = _mm256_fmadd_pd(ghi_, four(C), jkl_);  // +gC +hC +iC  _

  // Column mno:                                    m   n   o   _  =
  __m256d mno_ = _mm256_mul_pd(abc_, four(D));  //  aD  bD  cD  _
  mno_ = _mm256_fmadd_pd(def_, four(E), mno_);  // +dE +eE +fE  _
  mno_ = _mm256_fmadd_pd(ghi_, four(F), mno_);  // +gF +hF +iF  _

  // Column pqr:                                    p   q   r   _  =
  __m256d pqr_ = _mm256_mul_pd(abc_, four(G));  //  aG  bG  cG  _
  pqr_ = _mm256_fmadd_pd(def_, four(H), pqr_);  // +dH +eH +fH  _
  pqr_ = _mm256_fmadd_pd(ghi_, four(I), pqr_);  // +gI +hI +iI  _

  // Store the output.
  _mm256_storeu_pd(X_AC, jkl_);      // 4-wide write temporarily overwrites m
  _mm256_storeu_pd(X_AC + 3, mno_);  // 4-wide write temporarily overwrites p
  _mm256_storeu_pd(X_AC + 6, pqr_);  // 4-wide write temporarily overwrites s
  _mm256_maskstore_pd(X_AC + 9, mask, stu_);  // 3-wide write to stay in bounds

  // The compiler will generate a vzeroupper instruction if needed.
}

/* Composition of transforms X_AC = X_BA⁻¹ * X_BC.

Each matrix is 12 consecutive doubles in column-major order.

  X_BA = abcdefghixyz
  X_BC = ABCDEFGHIXYZ
  X_AC = jklmnopqrstu

We want to perform this 4x4 matrix multiplication:

      X_AC        X_BA⁻¹              X_BC
    j m p s     a b c -x(a+b+c)     A D G X
    k n q t  =  d e f -y(d+e+f)  @  B E H Y
    l o r u     g h i -z(g+h+i)     C F I Z
    0 0 0 1     0 0 0 -1            0 0 0 1

(Note that the "0 0 0 1" is only an illustration of the math; our transform
matrix does not actually store that row.)

Instead of calculating the full 4x4 multiply, we can take advantage of the
implicit constants in the last row to handle the rotation matrix and translation
vector separately.

We'll perform this 3x3 matrix multiplication:

   R_AC      R_BA⁻¹    R_BC

  j m p     a b c     A D G          Note that R_BA⁻¹ (aka R_AB) is in effect
  k n q  =  d e f  @  B E H          stored in row-major order, since our
  l o r     g h i     C F I          argument R_BA was in column-major order.

and this translation vector subtraction and matrix multiplication:

  s     a b c       X   x
  t  =  d e f  @  ( Y − y )
  u     g h i       Z   z

Writing that out longhand, we have:

  j = aA + bB + cC
  k = dA + eB + fC
  l = gA + hB + iC

  m = aD + bE + cF
  n = dD + eE + fF
  o = gD + hE + iF

  p = aG + bH + cI
  q = dG + eH + fI
  r = gG + hH + iI

  s = a(X-x) + b(Y-y) + c(Z-z)
  t = d(X-x) + e(Y-y) + f(Z-z)
  u = g(X-x) + h(Y-y) + i(Z-z)

Strategy: compute stu in parallel, then jkl, then mno, then pqr.
The <stu_> vector has the deepest pipeline, so we get it going first.

  <stu_> =   <adg_> * (<XXX_> - <xxx_>)
           + <beh_> * (<YYY_> - <yyy_>)
           + <cfi_> * (<ZZZ_> - <zzz_>)

  <jkl_> =   <adg_> * <AAA_>
           + <beh_> * <BBB_>
           + <cfi_> * <CCC_>

  <mno_> =   <adg_> * <DDD_>
           + <beh_> * <EEE_>
           + <cfi_> * <FFF_>

  <pqr_> =   <adg_> * <GGG_>
           + <beh_> * <HHH_>
           + <cfi_> * <III_>

We load columns from left matrix (tricky here since the inverse is row-major --
we'll pull in contiguous words and then shuffle) and broadcast elements from
right, performing 4 operations in parallel but ignoring the 4th result. We must
be careful not to load or store past the last element.

Rigid transform inverse composition requires 63 flops (36 multiplies, 24
additions, and 3 subtractions) but we're doing 84 here and throwing away 21 of
them. However, we only issue 13 SIMD floating point instructions (4 multiplies,
8 fused-multiply-adds, and 1 subtraction). */
void ComposeXinvXAvx(const double* X_BA, const double* X_BC, double* X_AC) {
  constexpr uint64_t yes = uint64_t(1ull << 63);
  constexpr uint64_t no = uint64_t(0);
  const __m256i mask = _mm256_setr_epi64x(yes, yes, yes, no);

  // Load the input. Loads columns of R_AB (rows of the given R_BA) into
  // registers via a series of loads, blends, and permutes.
  const __m256d abcd = _mm256_loadu_pd(X_BA);
  const __m256d efgh = _mm256_loadu_pd(X_BA + 4);
  const __m256d eb_h = _mm256_blend_pd(abcd, efgh, 0b1101);
  const __m256d beh_ = _mm256_permute_pd(eb_h, 0b0101);
  const __m256d cdg_ = _mm256_permute2f128_pd(abcd, efgh, 0b00110001);
  const __m256d adg_ = _mm256_blend_pd(abcd, cdg_, 0b1110);
  const __m256d fghi = _mm256_loadu_pd(X_BA + 5);
  const __m256d _fi_ = _mm256_permute_pd(fghi, 0b0100);
  const __m256d cfi_ = _mm256_blend_pd(cdg_, _fi_, 0b0110);
  const __m256d _xyz = _mm256_loadu_pd(X_BA + 8);
  const __m256d IXYZ = _mm256_loadu_pd(X_BC + 8);  // (_XYZ is a reserved word.)
  const __m256d subtract_XYZ_xyz = IXYZ - _xyz;
  const double A = X_BC[0];
  const double B = X_BC[1];
  const double C = X_BC[2];
  const double D = X_BC[3];
  const double E = X_BC[4];
  const double F = X_BC[5];
  const double G = X_BC[6];
  const double H = X_BC[7];
  const double I = X_BC[8];

  const double subXx = subtract_XYZ_xyz[1];
  const double subYy = subtract_XYZ_xyz[2];
  const double subZz = subtract_XYZ_xyz[3];
  // Column stu:                                         s       t       u     =
  __m256d stu_ = _mm256_mul_pd(adg_, four(subXx));  //  a(X-x)  d(X-x)  g(X-x)
  stu_ = _mm256_fmadd_pd(beh_, four(subYy), stu_);  // +b(Y-y) +e(Y-y) +h(Y-y)
  stu_ = _mm256_fmadd_pd(cfi_, four(subZz), stu_);  // +c(Z-z) +f(Z-z) +i(Z-z)

  // Column jkl:                                    j   k   l   _  =
  __m256d jkl_ = _mm256_mul_pd(adg_, four(A));  //  aA  dA  gA  _
  jkl_ = _mm256_fmadd_pd(beh_, four(B), jkl_);  // +bB +eB +hB  _
  jkl_ = _mm256_fmadd_pd(cfi_, four(C), jkl_);  // +cC +fC +iC  _

  // Column mno:                                    m   n   o  _  =
  __m256d mno_ = _mm256_mul_pd(adg_, four(D));  //  aD  dD  gD _
  mno_ = _mm256_fmadd_pd(beh_, four(E), mno_);  // +bE +eE +hE _
  mno_ = _mm256_fmadd_pd(cfi_, four(F), mno_);  // +cF +fF +iF _

  // Column pqr:                                    p   q   r   _  =
  __m256d pqr_ = _mm256_mul_pd(adg_, four(G));  //  aG  dG  gG  _
  pqr_ = _mm256_fmadd_pd(beh_, four(H), pqr_);  // +bH +eH +hH  _
  pqr_ = _mm256_fmadd_pd(cfi_, four(I), pqr_);  // +cI +fI +iI  _

  // Store the result.
  _mm256_storeu_pd(X_AC, jkl_);      // 4-wide write temporarily overwrites m
  _mm256_storeu_pd(X_AC + 3, mno_);  // 4-wide write temporarily overwrites p
  _mm256_storeu_pd(X_AC + 6, pqr_);  // 4-wide write temporarily overwrites s
  _mm256_maskstore_pd(X_AC + 9, mask, stu_);  // 3-wide write to stay in bounds

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

bool AvxSupported() {
  return false;
}

void ComposeRRAvx(const RotationMatrix<double>&, const RotationMatrix<double>&,
                  RotationMatrix<double>*) {
  AbortNotEnabledInBuild(__func__);
}

void ComposeRinvRAvx(const RotationMatrix<double>&,
                     const RotationMatrix<double>&, RotationMatrix<double>*) {
  AbortNotEnabledInBuild(__func__);
}

void ComposeXXAvx(const RigidTransform<double>&, const RigidTransform<double>&,
                  RigidTransform<double>*) {
  AbortNotEnabledInBuild(__func__);
}

void ComposeXinvXAvx(const RigidTransform<double>&,
                     const RigidTransform<double>&, RigidTransform<double>*) {
  AbortNotEnabledInBuild(__func__);
}
#endif

}  // namespace internal
}  // namespace math
}  // namespace drake
