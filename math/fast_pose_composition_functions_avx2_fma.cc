#include "drake/math/fast_pose_composition_functions_avx2_fma.h"

#if defined(__AVX2__) && defined(__FMA__)
#include <cstdint>

#include <cpuid.h>

#pragma GCC diagnostic push
// TODO(jwnimmer-tri) Ideally we would fix the old-style-cast warnings instead
// of suppressing them, perhaps by submitting a patch upstream.
#pragma GCC diagnostic ignored "-Wold-style-cast"
// By setting the next two HWY_... macros, we're saying that we don't want to
// generate any SIMD code for pre-AVX2 CPUs, and that we're okay not using the
// carry-less multiplication and crypto stuff.
#define HWY_BASELINE_TARGETS HWY_AVX2
#define HWY_DISABLE_PCLMUL_AES 1
#include "hwy/highway.h"
#pragma GCC diagnostic pop
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

// The hn namespace holds the CPU-specific function overloads. By defining it
// using a substitute-able macro, we achieve per-CPU instruction selection.
namespace hn = hwy::HWY_NAMESPACE;

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
lanes of doubles, so e.g. <dcba> denotes the array of doubles [d, c, b, a]. We
use both lowercase and uppercase letters for naming lanes. In variable names,
we leave out the <> marker so the variable name would be just "dcba".

The lower order lanes are spelled on the right, so for example if YMM0 = <dcba>
then we have XMM0 = <ba>.

Sometimes we don't need to give a name to a lane (because we don't care what its
value is). In that case we use the canonical "_" to indicate "don't care", e.g.,
<_cba> indicates the array of doubles [_, c, b, a] where we only care about the
first three lanes. This leads to variables named e.g. "dcb_" which look like C++
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

  <_lkj> =   <_cba> * <AAAA>
           + <_fed> * <BBBB>
           + <_ihg> * <CCCC>

  <_onm> =   <_cba> * <DDDD>
           + <_fed> * <EEEE>
           + <_ihg> * <FFFF>

  <_rqp> =   <_cba> * <GGGG>
           + <_fed> * <HHHH>
           + <_ihg> * <IIII>

We load columns from left matrix and broadcast elements from right, performing
4 operations in parallel but ignoring the 4th result. We must be careful not to
load or store past the last element.

Rotation matrix composition requires 45 flops (27 multiplies and 18 additions)
but we're doing 60 here and throwing away 15 of them. However, we only issue 9
SIMD floating point instructions (3 multiplies and 6 fused-multiply-adds).

It is OK if the result R_AC overlaps one or both of the inputs. */
void ComposeRRAvx(const double* R_AB, const double* R_BC, double* R_AC) {
  const hn::FixedTag<double, 4> tag;

  const auto _cba = hn::LoadU(tag, R_AB);      // (d is loaded but unused)
  const auto _fed = hn::LoadU(tag, R_AB + 3);  // (g is loaded but unused)
  const auto _ihg = hn::LoadN(tag, R_AB + 6, 3);

  const auto AAAA = hn::Set(tag, R_BC[0]);
  const auto BBBB = hn::Set(tag, R_BC[1]);
  const auto CCCC = hn::Set(tag, R_BC[2]);
  const auto DDDD = hn::Set(tag, R_BC[3]);
  const auto EEEE = hn::Set(tag, R_BC[4]);
  const auto FFFF = hn::Set(tag, R_BC[5]);
  const auto GGGG = hn::Set(tag, R_BC[6]);
  const auto HHHH = hn::Set(tag, R_BC[7]);
  const auto IIII = hn::Set(tag, R_BC[8]);

  // Column jkl:                           _   l   k   j
  auto _lkj = hn::Mul(_cba, AAAA);      // _  cA  bA  aA
  _lkj = hn::MulAdd(_fed, BBBB, _lkj);  // _ +fB +eB +dB
  _lkj = hn::MulAdd(_ihg, CCCC, _lkj);  // _ +iC +hC +gC

  // Column mno:                           _   m   n   o
  auto _onm = hn::Mul(_cba, DDDD);      // _  cD  bD  aD
  _onm = hn::MulAdd(_fed, EEEE, _onm);  // _ +fE +eE +dE
  _onm = hn::MulAdd(_ihg, FFFF, _onm);  // _ +iF +hF +gF

  // Column pqr:                           _  r   q   p
  auto _rqp = hn::Mul(_cba, GGGG);      // _  cG  bG  aG
  _rqp = hn::MulAdd(_fed, HHHH, _rqp);  // _ +fH +eH +dH
  _rqp = hn::MulAdd(_ihg, IIII, _rqp);  // _ +iI +hI +gI

  // Store the output.
  hn::StoreU(_lkj, tag, R_AC);         // 4-wide write temporarily overwrites m
  hn::StoreU(_onm, tag, R_AC + 3);     // 4-wide write temporarily overwrites p
  hn::StoreN(_rqp, tag, R_AC + 6, 3);  // 3-wide write to stay in bounds
}

/* Loads R_BA, transposed. Imagine R_BA as a array of doubles 'abcdefghi'. Upon
return, the output arguments _gda, beh_, and cfi_ are filled with the like-named
elements of R_BA. (As usual, underscore means undefined / don't care.) */
template <typename Vec4>
void LoadMatrix3Inv(const double* R_BA, Vec4* _gda, Vec4* _heb, Vec4* _ifc) {
  const hn::FixedTag<double, 4> tag;
  const auto dcba = hn::LoadU(tag, R_BA);
  const auto gfed = hn::LoadU(tag, R_BA + 3);
  const auto hgfe = hn::LoadU(tag, R_BA + 4);
  const auto iiii = hn::Set(tag, R_BA[8]);
  const auto hgba = hn::ConcatUpperLower(tag, hgfe, dcba);  // hg..|..ba => hgba
  *_gda = hn::InterleaveLower(tag, hgba, gfed);             // .g.a|.f.d => fgda
  *_heb = hn::InterleaveUpper(tag, hgba, gfed);             // h.b.|g.e. => gheb
  const auto fc__ = hn::InterleaveLower(dcba, gfed);        // .c.a|.f.d => fcda
  *_ifc = hn::ConcatUpperUpper(tag, iiii, fc__);            // ii..|fc.. => iifc
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

  <_lkj> =   <_gda> * <AAAA>
           + <_heb> * <BBBB>
           + <_ifc> * <CCCD>

  <_onm> =   <_gda> * <DDDD>
           + <_heb> * <EEEE>
           + <_ifc> * <FFFF>

  <_rqp> =   <_gda> * <GGGG>
           + <_heb> * <HHHH>
           + <_ifc> * <IIII>

We load columns from left matrix (tricky here since the inverse is row-major --
we'll pull in contiguous words and then shuffle) and broadcast elements from
right, performing 4 operations in parallel but ignoring the 4th result. We must
be careful not to load or store past the last element.

Rotation matrix composition requires 45 flops (27 multiplies and 18 additions)
but we're doing 60 here and throwing away 15 of them. However, we only issue 9
SIMD floating point instructions (3 multiplies and 6 fused-multiply-adds).

It is OK if the result R_AC overlaps one or both of the inputs. */
void ComposeRinvRAvx(const double* R_BA, const double* R_BC, double* R_AC) {
  const hn::FixedTag<double, 4> tag;

  // Load the columns of R_AB (rows of the given R_BA).
  auto _gda = hn::Undefined(tag);
  auto _heb = hn::Undefined(tag);
  auto _ifc = hn::Undefined(tag);
  LoadMatrix3Inv(R_BA, &_gda, &_heb, &_ifc);

  // Load R_BC.
  const auto AAAA = hn::Set(tag, R_BC[0]);
  const auto BBBB = hn::Set(tag, R_BC[1]);
  const auto CCCC = hn::Set(tag, R_BC[2]);
  const auto DDDD = hn::Set(tag, R_BC[3]);
  const auto EEEE = hn::Set(tag, R_BC[4]);
  const auto FFFF = hn::Set(tag, R_BC[5]);
  const auto GGGG = hn::Set(tag, R_BC[6]);
  const auto HHHH = hn::Set(tag, R_BC[7]);
  const auto IIII = hn::Set(tag, R_BC[8]);

  // Column jkl:                           _  l   k   j   =
  auto _lkj = hn::Mul(_gda, AAAA);      // _  gA  dA  aA
  _lkj = hn::MulAdd(_heb, BBBB, _lkj);  // _ +hB +eB +bB
  _lkj = hn::MulAdd(_ifc, CCCC, _lkj);  // _ +iC +fC +cC

  // Column mno:                           _  o   n   m   =
  auto _onm = hn::Mul(_gda, DDDD);      // _  gD  dD  aD
  _onm = hn::MulAdd(_heb, EEEE, _onm);  // _ +hE +eE +bE
  _onm = hn::MulAdd(_ifc, FFFF, _onm);  // _ +iF +fF +cF

  // Column pqr:                           _  r   q   p   =
  auto _rqp = hn::Mul(_gda, GGGG);      // _  gG  dG  aG
  _rqp = hn::MulAdd(_heb, HHHH, _rqp);  // _ +hH +eH +bH
  _rqp = hn::MulAdd(_ifc, IIII, _rqp);  // _ +iI +fI +cI

  // Store the output.
  hn::StoreU(_lkj, tag, R_AC);         // 4-wide write temporarily overwrites m
  hn::StoreU(_onm, tag, R_AC + 3);     // 4-wide write temporarily overwrites p
  hn::StoreN(_rqp, tag, R_AC + 6, 3);  // 3-wide write to stay in bounds
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
void ComposeXinvXAvx(const double* X_BA, const double* X_BC, double* X_AC) {
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
