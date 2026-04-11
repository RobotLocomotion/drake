/* clang-format off to disable clang-format-includes */
#include "drake/math/fast_pose_composition_functions.h"
/* clang-format on */

#include <algorithm>
#include <cstdint>

// This is the magic juju that compiles our impl functions for multiple CPUs.
#undef HWY_TARGET_INCLUDE
#define HWY_TARGET_INCLUDE "math/fast_pose_composition_functions.cc"
#include "hwy/foreach_target.h"
#include "hwy/highway.h"

#include "drake/common/drake_assert.h"
#include "drake/common/hwy_dynamic_impl.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"

HWY_BEFORE_NAMESPACE();
namespace drake {
namespace math {
namespace internal {
namespace {
namespace HWY_NAMESPACE {
namespace hn = hwy::HWY_NAMESPACE;

// The SIMD approach is only useful when we have registers of size `double[4]`
// or larger. When we have smaller registers (e.g., SSE2's 2-wide lanes, or
// SVE's variable-length vectors) we will fall back to non-SIMD code.
#if HWY_MAX_BYTES >= 32 && HWY_HAVE_SCALABLE == 0

/* =============================================================================
   Notation conventions used in this file
================================================================================

In comments, we'll use "<....>" to indicate a 256-bit AVX register containing 4
lanes of doubles, so e.g. <abcd> denotes the array of doubles [a, b, c, d]. We
use both lowercase and uppercase letters for naming lanes. In variable names,
we leave out the <> marker so the variable name would be just "abcd".

The lower order lanes are spelled on the left, so for example if YMM0 = <abcd>
then we have XMM0 = <ab>.

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

For a Highway function reference, see:

 https://google.github.io/highway/en/master/quick_reference.html#operations

Be very careful when reading Highway docs to distinguish "lane" vs "block".
A lane holds one scalar (e.g., a `double`); a block is 16-bytes worth of lanes,
e.g., two doubles.

============================================================================= */

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
void ComposeRRImpl(const double* R_AB, const double* R_BC, double* R_AC) {
  const hn::FixedTag<double, 4> tag;

  const auto abc_ = hn::LoadU(tag, R_AB);      // (d is loaded but unused)
  const auto def_ = hn::LoadU(tag, R_AB + 3);  // (g is loaded but unused)
  const auto ghi_ = hn::LoadN(tag, R_AB + 6, 3);

  const auto AAA_ = hn::Set(tag, R_BC[0]);
  const auto BBB_ = hn::Set(tag, R_BC[1]);
  const auto CCC_ = hn::Set(tag, R_BC[2]);
  const auto DDD_ = hn::Set(tag, R_BC[3]);
  const auto EEE_ = hn::Set(tag, R_BC[4]);
  const auto FFF_ = hn::Set(tag, R_BC[5]);
  const auto GGG_ = hn::Set(tag, R_BC[6]);
  const auto HHH_ = hn::Set(tag, R_BC[7]);
  const auto III_ = hn::Set(tag, R_BC[8]);

  // Column jkl:                            j   k   l   _  =
  auto jkl_ = hn::Mul(abc_, AAA_);      //  aA  bA  cA  _
  jkl_ = hn::MulAdd(def_, BBB_, jkl_);  // +dB +eB +fB  _
  jkl_ = hn::MulAdd(ghi_, CCC_, jkl_);  // +gC +hC +iC  _

  // Column mno:                            m   n   o   _  =
  auto mno_ = hn::Mul(abc_, DDD_);      //  aD  bD  cD  _
  mno_ = hn::MulAdd(def_, EEE_, mno_);  // +dE +eE +fE  _
  mno_ = hn::MulAdd(ghi_, FFF_, mno_);  // +gF +hF +iF  _

  // Column pqr:                            p   q   r   _  =
  auto pqr_ = hn::Mul(abc_, GGG_);      //  aG  bG  cG  _
  pqr_ = hn::MulAdd(def_, HHH_, pqr_);  // +dH +eH +fH  _
  pqr_ = hn::MulAdd(ghi_, III_, pqr_);  // +gI +hI +iI  _

  // Store the output.
  hn::StoreU(jkl_, tag, R_AC);         // 4-wide write temporarily overwrites m
  hn::StoreU(mno_, tag, R_AC + 3);     // 4-wide write temporarily overwrites p
  hn::StoreN(pqr_, tag, R_AC + 6, 3);  // 3-wide write to stay in bounds
}

/* Loads R_BA, transposed. Imagine R_BA as a array of doubles 'abcdefghi'. Upon
return, the output arguments adg_, beh_, and cfi_ are filled with the like-named
elements of R_BA. (As usual, underscore means undefined / don't care.) */
template <typename Vec4>
void LoadMatrix3Inv(const double* R_BA, Vec4* adg_, Vec4* beh_, Vec4* cfi_) {
  const hn::FixedTag<double, 4> tag;
  const auto abcd = hn::LoadU(tag, R_BA);
  const auto defg = hn::LoadU(tag, R_BA + 3);
  const auto efgh = hn::LoadU(tag, R_BA + 4);
  const auto iiii = hn::Set(tag, R_BA[8]);

  // Take special care in reading the below:
  //
  // - InterleaveEven(a, b) interleaves the even-numbered lanes of `a` and `b`
  //   (and discards the odd-numbered lanes), i.e., the odd-numbered lanes
  //   of `a` are replaced by the even-numbered lanes of `b`.
  //
  // - InterleaveOdd(a, b) interleaves the odd-numbered lanes of `a` and `b`
  //   (and discards the even-numbered lanes), i.e., the even-numbered lanes
  //   of `b` are replaced by the odd-numbered lanes of `a`.
  //
  // - ConcatFooBar(hi, lo) combines half of `hi` with half of `lo`.
  //
  //     Note that "half" here means splitting each 4-lane vector down the
  //     middle, into the Lower two lanes (0 and 1) and Upper two lanes (2
  //     and 3), not even- vs odd-numbered lanes.
  //
  //     The upper half of the result comes from the Foo part of `hi`. The lower
  //     half of the result comes from the Bar half of `lo`. One important quirk
  //     of this particular Highway function is that its argument order (hi, lo)
  //     is the opposite of our usual convention of putting lower on the left.
  //
  //     Thus, for ConcatUpperLower:
  //       The upper half of the result comes from the Upper half of `hi`.
  //       The lower half of the result comes from the Lower half of `lo`.
  //       The result is <lo[0] lo[1] hi[2] hi[3]>.
  //                                  ^^^^^^^^^^^
  //                                  This is "hi:Upper".
  //                      ^^^^^^^^^^^
  //                      This is lo:Lower.
  //
  //     Thus, for ConcatUpperUpper:
  //       The upper half of the result comes is the Upper half of `hi`.
  //       The lower half of the result comes is the Upper half of `lo`.
  //       The result is <lo[2] lo[3] hi[2] hi[3]>.
  //                                  ^^^^^^^^^^^
  //                                  This is "hi:Upper".
  //                      ^^^^^^^^^^^
  //                      This is lo:Upper.
  //
  const auto abgh = hn::ConcatUpperLower(tag, efgh, abcd);  // ..gh|ab.. => abgh
  const auto adgf = hn::InterleaveEven(tag, abgh, defg);    // a.g.|d.f. => adgf
  const auto behg = hn::InterleaveOdd(tag, abgh, defg);     // .b.h|.e.g => behg
  const auto adcf = hn::InterleaveEven(abcd, defg);         // a.c.|d.f. => adcf
  const auto cfii = hn::ConcatUpperUpper(tag, iiii, adcf);  // ..ii|..cf => cfii

  // These don't generate any instructions, but are just a nice way to use
  // variable names with and without the don't-care markers, for clarity.
  *adg_ = adgf;
  *beh_ = behg;
  *cfi_ = cfii;
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
void ComposeRinvRImpl(const double* R_BA, const double* R_BC, double* R_AC) {
  const hn::FixedTag<double, 4> tag;

  // Load the columns of R_AB (rows of the given R_BA).
  auto adg_ = hn::Undefined(tag);
  auto beh_ = hn::Undefined(tag);
  auto cfi_ = hn::Undefined(tag);
  LoadMatrix3Inv(R_BA, &adg_, &beh_, &cfi_);

  // Load R_BC.
  const auto AAA_ = hn::Set(tag, R_BC[0]);
  const auto BBB_ = hn::Set(tag, R_BC[1]);
  const auto CCC_ = hn::Set(tag, R_BC[2]);
  const auto DDD_ = hn::Set(tag, R_BC[3]);
  const auto EEE_ = hn::Set(tag, R_BC[4]);
  const auto FFF_ = hn::Set(tag, R_BC[5]);
  const auto GGG_ = hn::Set(tag, R_BC[6]);
  const auto HHH_ = hn::Set(tag, R_BC[7]);
  const auto III_ = hn::Set(tag, R_BC[8]);

  // Column jkl:                            j   k   l   _  =
  auto jkl_ = hn::Mul(adg_, AAA_);      //  aA  dA  gA  _
  jkl_ = hn::MulAdd(beh_, BBB_, jkl_);  // +bB +eB +hB  _
  jkl_ = hn::MulAdd(cfi_, CCC_, jkl_);  // +cC +fC +iC  _

  // Column mno:                            m   n   o   _  =
  auto mno_ = hn::Mul(adg_, DDD_);      //  aD  dD  gD  _
  mno_ = hn::MulAdd(beh_, EEE_, mno_);  // +bE +eE +hE  _
  mno_ = hn::MulAdd(cfi_, FFF_, mno_);  // +cF +fF +iF  _

  // Column pqr:                            p   q   r   _  =
  auto pqr_ = hn::Mul(adg_, GGG_);      //  aG  dG  gG  _
  pqr_ = hn::MulAdd(beh_, HHH_, pqr_);  // +bH +eH +hH  _
  pqr_ = hn::MulAdd(cfi_, III_, pqr_);  // +cI +fI +iI  _

  // Store the output.
  hn::StoreU(jkl_, tag, R_AC);         // 4-wide write temporarily overwrites m
  hn::StoreU(mno_, tag, R_AC + 3);     // 4-wide write temporarily overwrites p
  hn::StoreN(pqr_, tag, R_AC + 6, 3);  // 3-wide write to stay in bounds
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

Rigid transform composition requires 63 flops (36 multiplies and 27 additions)
but we're doing 84 here and throwing away 21 of them. However, we only issue 12
SIMD floating point instructions (3 multiplies and 9 fused-multiply-adds).

It is OK if the result X_AC overlaps one or both of the inputs. */
void ComposeXXImpl(const double* X_AB, const double* X_BC, double* X_AC) {
  const hn::FixedTag<double, 4> tag;

  // Load the input.
  const auto abc_ = hn::LoadU(tag, X_AB);      // (d is loaded but unused)
  const auto def_ = hn::LoadU(tag, X_AB + 3);  // (g is loaded but unused)
  const auto ghi_ = hn::LoadU(tag, X_AB + 6);  // (x is loaded but unused)
  const auto xyz_ = hn::LoadN(tag, X_AB + 9, 3);

  const auto AAA_ = hn::Set(tag, X_BC[0]);
  const auto BBB_ = hn::Set(tag, X_BC[1]);
  const auto CCC_ = hn::Set(tag, X_BC[2]);
  const auto DDD_ = hn::Set(tag, X_BC[3]);
  const auto EEE_ = hn::Set(tag, X_BC[4]);
  const auto FFF_ = hn::Set(tag, X_BC[5]);
  const auto GGG_ = hn::Set(tag, X_BC[6]);
  const auto HHH_ = hn::Set(tag, X_BC[7]);
  const auto III_ = hn::Set(tag, X_BC[8]);
  const auto XXX_ = hn::Set(tag, X_BC[9]);
  const auto YYY_ = hn::Set(tag, X_BC[10]);
  const auto ZZZ_ = hn::Set(tag, X_BC[11]);

  // Column stu:                            s   t   u   _  =
  auto stu_ = xyz_;                     //  x   y   z   _
  stu_ = hn::MulAdd(abc_, XXX_, stu_);  // +aX +bX +cX  _
  stu_ = hn::MulAdd(def_, YYY_, stu_);  // +dY +eY +fY  _
  stu_ = hn::MulAdd(ghi_, ZZZ_, stu_);  // +gZ +hZ +iZ  _

  // Column jkl:                            j   k   l   _  =
  auto jkl_ = hn::Mul(abc_, AAA_);      //  aA  bA  cA  _
  jkl_ = hn::MulAdd(def_, BBB_, jkl_);  // +dB +eB +fB  _
  jkl_ = hn::MulAdd(ghi_, CCC_, jkl_);  // +gC +hC +iC  _

  // Column mno:                            m   n   o   _  =
  auto mno_ = hn::Mul(abc_, DDD_);      //  aD  bD  cD  _
  mno_ = hn::MulAdd(def_, EEE_, mno_);  // +dE +eE +fE  _
  mno_ = hn::MulAdd(ghi_, FFF_, mno_);  // +gF +hF +iF  _

  // Column pqr:                            p   q   r   _  =
  auto pqr_ = hn::Mul(abc_, GGG_);      //  aG  bG  cG  _
  pqr_ = hn::MulAdd(def_, HHH_, pqr_);  // +dH +eH +fH  _
  pqr_ = hn::MulAdd(ghi_, III_, pqr_);  // +gI +hI +iI  _

  // Store the output.
  hn::StoreU(jkl_, tag, X_AC);         // 4-wide write temporarily overwrites m
  hn::StoreU(mno_, tag, X_AC + 3);     // 4-wide write temporarily overwrites p
  hn::StoreU(pqr_, tag, X_AC + 6);     // 4-wide write temporarily overwrites s
  hn::StoreN(stu_, tag, X_AC + 9, 3);  // 3-wide write to stay in bounds
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
void ComposeXinvXImpl(const double* X_BA, const double* X_BC, double* X_AC) {
  const hn::FixedTag<double, 4> tag;

  // Load the columns of R_AB (rows of the given R_BA).
  auto adg_ = hn::Undefined(tag);
  auto beh_ = hn::Undefined(tag);
  auto cfi_ = hn::Undefined(tag);
  LoadMatrix3Inv(X_BA, &adg_, &beh_, &cfi_);

  // Load both translations.
  const auto _xyz = hn::LoadU(tag, X_BA + 8);
  const auto IXYZ = hn::LoadU(tag, X_BC + 8);  // (_XYZ is a reserved word.)
  const auto subtract_XYZ_xyz = IXYZ - _xyz;
  const auto subXx = hn::BroadcastLane<1>(subtract_XYZ_xyz);
  const auto subYy = hn::BroadcastLane<2>(subtract_XYZ_xyz);
  const auto subZz = hn::BroadcastLane<3>(subtract_XYZ_xyz);

  const auto AAA_ = hn::Set(tag, X_BC[0]);
  const auto BBB_ = hn::Set(tag, X_BC[1]);
  const auto CCC_ = hn::Set(tag, X_BC[2]);
  const auto DDD_ = hn::Set(tag, X_BC[3]);
  const auto EEE_ = hn::Set(tag, X_BC[4]);
  const auto FFF_ = hn::Set(tag, X_BC[5]);
  const auto GGG_ = hn::Set(tag, X_BC[6]);
  const auto HHH_ = hn::Set(tag, X_BC[7]);
  const auto III_ = hn::Set(tag, X_BC[8]);

  // Column stu:                             s       t       u     =
  auto stu_ = hn::Mul(adg_, subXx);      //  a(X-x)  d(X-x)  g(X-x)
  stu_ = hn::MulAdd(beh_, subYy, stu_);  // +b(Y-y) +e(Y-y) +h(Y-y)
  stu_ = hn::MulAdd(cfi_, subZz, stu_);  // +c(Z-z) +f(Z-z) +i(Z-z)

  // Column jkl:                            j   k   l   _  =
  auto jkl_ = hn::Mul(adg_, AAA_);      //  aA  dA  gA  _
  jkl_ = hn::MulAdd(beh_, BBB_, jkl_);  // +bB +eB +hB  _
  jkl_ = hn::MulAdd(cfi_, CCC_, jkl_);  // +cC +fC +iC  _

  // Column mno:                            m   n   o   _  =
  auto mno_ = hn::Mul(adg_, DDD_);      //  aD  dD  gD  _
  mno_ = hn::MulAdd(beh_, EEE_, mno_);  // +bE +eE +hE  _
  mno_ = hn::MulAdd(cfi_, FFF_, mno_);  // +cF +fF +iF  _

  // Column pqr:                            p   q   r   _  =
  auto pqr_ = hn::Mul(adg_, GGG_);      //  aG  dG  gG  _
  pqr_ = hn::MulAdd(beh_, HHH_, pqr_);  // +bH +eH +hH  _
  pqr_ = hn::MulAdd(cfi_, III_, pqr_);  // +cI +fI +iI  _

  // Store the result.
  hn::StoreU(jkl_, tag, X_AC);         // 4-wide write temporarily overwrites m
  hn::StoreU(mno_, tag, X_AC + 3);     // 4-wide write temporarily overwrites p
  hn::StoreU(pqr_, tag, X_AC + 6);     // 4-wide write temporarily overwrites s
  hn::StoreN(stu_, tag, X_AC + 9, 3);  // 3-wide write to stay in bounds
}

#else  // HWY_MAX_BYTES

/* The portable versions are always defined. They should be written to maximize
the chance that a dumb compiler can generate fast code.

Note that, except when explicitly marked "...NoAlias", the methods below must
allow for the output argument's memory to overlap with any of the input
arguments' memory. */

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
void ComposeRinvRNoAlias(const double* R_BA, const double* R_BC, double* R_AC) {
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

/* @pre X_AC is disjoint in memory from the inputs. */
void ComposeXXNoAlias(const double* X_AB, const double* X_BC, double* X_AC) {
  const double* p_AB = X_AB + 9;  // Make some nice aliases.
  const double* p_BC = X_BC + 9;
  double* p_AC = X_AC + 9;

  // X_AB * X_BC = [ R_AB; p_AB ] * [ R_BC; p_BC ]
  //             = [ (R_AB*R_BC); (p_AB + R_AB*p_BC) ]

  ComposeRRNoAlias(X_AB, X_BC, X_AC);  // Just works with first 9 elements.
  p_AC[0] = p_AB[0] + row_x_col(&X_AB[0], p_BC);
  p_AC[1] = p_AB[1] + row_x_col(&X_AB[1], p_BC);
  p_AC[2] = p_AB[2] + row_x_col(&X_AB[2], p_BC);
}

/* @pre X_AC is disjoint in memory from the inputs. */
void ComposeXinvXNoAlias(const double* X_BA, const double* X_BC, double* X_AC) {
  const double* p_BA = X_BA + 9;  // Make some nice aliases.
  const double* p_BC = X_BC + 9;
  double* p_AC = X_AC + 9;

  // X_BA⁻¹ * X_BC = [ R_BA⁻¹; (R_BA⁻¹ * -p_BA) ]  * [ R_BC; p_BC ]
  //               = [ (R_BA⁻¹ * R_BC); (R_BA⁻¹ * (p_BC - p_BA)) ]

  ComposeRinvRNoAlias(X_BA, X_BC, X_AC);  // Just works with first 9 elements.
  const double p_AC_B[3] = {p_BC[0] - p_BA[0], p_BC[1] - p_BA[1],
                            p_BC[2] - p_BA[2]};
  p_AC[0] = col_x_col(&X_BA[0], p_AC_B);  // Note that R_BA⁻¹ = R_BAᵀ so we
  p_AC[1] = col_x_col(&X_BA[3], p_AC_B);  // just need to use columns here
  p_AC[2] = col_x_col(&X_BA[6], p_AC_B);  // rather than rows.
}

void ComposeRRImpl(const double* R_AB, const double* R_BC, double* R_AC) {
  DRAKE_ASSERT(R_AC != nullptr);
  double R_AC_temp[9];  // Protect from overlap with inputs.
  ComposeRRNoAlias(R_AB, R_BC, R_AC_temp);
  std::copy(R_AC_temp, R_AC_temp + 9, R_AC);
}

void ComposeRinvRImpl(const double* R_BA, const double* R_BC, double* R_AC) {
  DRAKE_ASSERT(R_AC != nullptr);
  double R_AC_temp[9];  // Protect from overlap with inputs.
  ComposeRinvRNoAlias(R_BA, R_BC, R_AC_temp);
  std::copy(R_AC_temp, R_AC_temp + 9, R_AC);
}

void ComposeXXImpl(const double* X_AB, const double* X_BC, double* X_AC) {
  DRAKE_ASSERT(X_AC != nullptr);
  double X_AC_temp[12];  // Protect from overlap with inputs.
  ComposeXXNoAlias(X_AB, X_BC, X_AC_temp);
  std::copy(X_AC_temp, X_AC_temp + 12, X_AC);
}

void ComposeXinvXImpl(const double* X_BA, const double* X_BC, double* X_AC) {
  DRAKE_ASSERT(X_AC != nullptr);
  double X_AC_temp[12];  // Protect from overlap with inputs.
  ComposeXinvXNoAlias(X_BA, X_BC, X_AC_temp);
  std::copy(X_AC_temp, X_AC_temp + 12, X_AC);
}

#endif  // HWY_MAX_BYTES

}  // namespace HWY_NAMESPACE
}  // namespace
}  // namespace internal
}  // namespace math
}  // namespace drake
HWY_AFTER_NAMESPACE();

// This part of the file is only compiled once total, instead of once per CPU.
#if HWY_ONCE
namespace drake {
namespace math {
namespace internal {
namespace {

// Create the lookup tables for the per-CPU hwy implementation functions, and
// required functors that select from the lookup tables.
HWY_EXPORT(ComposeRRImpl);
struct ChooseBestComposeRR {
  auto operator()() { return HWY_DYNAMIC_POINTER(ComposeRRImpl); }
};
HWY_EXPORT(ComposeRinvRImpl);
struct ChooseBestComposeRinvR {
  auto operator()() { return HWY_DYNAMIC_POINTER(ComposeRinvRImpl); }
};
HWY_EXPORT(ComposeXXImpl);
struct ChooseBestComposeXX {
  auto operator()() { return HWY_DYNAMIC_POINTER(ComposeXXImpl); }
};
HWY_EXPORT(ComposeXinvXImpl);
struct ChooseBestComposeXinvX {
  auto operator()() { return HWY_DYNAMIC_POINTER(ComposeXinvXImpl); }
};

// These sugar functions convert C++ types into bare arrays.
const double* GetRawData(const RotationMatrix<double>& R) {
  return R.matrix().data();
}
double* GetRawData(RotationMatrix<double>* R) {
  return const_cast<double*>(R->matrix().data());
}
const double* GetRawData(const RigidTransform<double>& X) {
  // In rigid_transform.h we assert that the memory layout is 12 doubles, with
  // the rotation matrix first followed by the translation.
  return X.rotation().matrix().data();
}
double* GetRawData(RigidTransform<double>* X) {
  // In rigid_transform.h we assert that the memory layout is 12 doubles, with
  // the rotation matrix first followed by the translation.
  return const_cast<double*>(X->rotation().matrix().data());
}

}  // namespace

void ComposeRR(const RotationMatrix<double>& R_AB,
               const RotationMatrix<double>& R_BC,
               RotationMatrix<double>* R_AC) {
  LateBoundFunction<ChooseBestComposeRR>::Call(
      GetRawData(R_AB), GetRawData(R_BC), GetRawData(R_AC));
}

void ComposeRinvR(const RotationMatrix<double>& R_BA,
                  const RotationMatrix<double>& R_BC,
                  RotationMatrix<double>* R_AC) {
  LateBoundFunction<ChooseBestComposeRinvR>::Call(
      GetRawData(R_BA), GetRawData(R_BC), GetRawData(R_AC));
}

void ComposeXX(const RigidTransform<double>& X_AB,
               const RigidTransform<double>& X_BC,
               RigidTransform<double>* X_AC) {
  LateBoundFunction<ChooseBestComposeXX>::Call(
      GetRawData(X_AB), GetRawData(X_BC), GetRawData(X_AC));
}

void ComposeXinvX(const RigidTransform<double>& X_BA,
                  const RigidTransform<double>& X_BC,
                  RigidTransform<double>* X_AC) {
  LateBoundFunction<ChooseBestComposeXinvX>::Call(
      GetRawData(X_BA), GetRawData(X_BC), GetRawData(X_AC));
}

}  // namespace internal
}  // namespace math
}  // namespace drake
#endif  // HWY_ONCE
