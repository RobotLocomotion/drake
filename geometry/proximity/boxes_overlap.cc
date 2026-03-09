#include "drake/geometry/proximity/boxes_overlap.h"

// This is the magic juju that compiles our impl functions for multiple CPUs.
#undef HWY_TARGET_INCLUDE
#define HWY_TARGET_INCLUDE "geometry/proximity/boxes_overlap.cc"
#include "hwy/foreach_target.h"
#include "hwy/highway.h"

#include "drake/common/hwy_dynamic_impl.h"

HWY_BEFORE_NAMESPACE();
namespace drake {
namespace geometry {
namespace internal {

using Eigen::Matrix3d;
using Eigen::Vector3d;
using math::RigidTransformd;

namespace {
namespace HWY_NAMESPACE {
// The hn namespace holds the CPU-specific function overloads. By defining it
// using a substitute-able macro, we achieve per-CPU instruction selection.
namespace hn = hwy::HWY_NAMESPACE;

// The SIMD approach is only useful when we have registers of size `double[4]`
// or larger. When we have smaller registers (e.g., SSE2's 2-wide lanes, or
// SVE's variable-length vectors) we will fall back to non-SIMD code.
#if HWY_MAX_BYTES >= 32 && HWY_HAVE_SCALABLE == 0

template <typename Tag, typename VecT = hn::Vec<Tag>>
bool GreaterThan(const Tag& tag, const VecT& left, const VecT& right) {
  auto cmp = hn::Gt(left, right);
  // Rather than structuring BoxesOverlapImpl so that lane 3 is always zero,
  // here we look for a true value in the first three lanes. Note:
  // FindKnownFirstTrue requires that at least one lane be true. This has a
  // slight performance edge over simply using FindFirstTrue().
  return !hn::AllFalse(tag, cmp) && hn::FindKnownFirstTrue(tag, cmp) < 3;
}

// This implementation follows section 64 (on PDF page 102) of:
//  http://gamma.cs.unc.edu/users/gottschalk/main.pdf

// See note in BoxesOverlap as to why the parameters are pointers.
// We're simply assuming that they "can't" be null.
bool BoxesOverlapImpl(const Vector3d* half_size_a_ptr,
                      const Vector3d* half_size_b_ptr,
                      const math::RigidTransformd* X_AB_ptr) {
  const Vector3d& half_size_a = *half_size_a_ptr;
  const Vector3d& half_size_b = *half_size_b_ptr;
  const math::RigidTransformd& X_AB = *X_AB_ptr;

  const Vector3d& p_AB = X_AB.translation();
  const Matrix3d& R_AB = X_AB.rotation().matrix();
  // We'll be adding epsilon to counteract arithmetic error, e.g., when two
  // edges are parallel. We use the value as specified from Gottschalk's OBB
  // robustness tests.
  const double kEpsilon = 0.000001;

  const double* const R = &R_AB.coeffRef(0, 0);
  const double* const xyz = &p_AB.coeffRef(0);
  const double* const pqr = &half_size_a.coeffRef(0);
  const double* const stu = &half_size_b.coeffRef(0);

  const hn::FixedTag<double, 4> tag;
  using VecT = hn::Vec<decltype(tag)>;

  // Note: the names of these variables represent the value stored in each of
  // the four lanes. When the value is undefined, we use the underscore
  // character with one exception (see below).

  const VecT eps4 = hn::Set(tag, kEpsilon);
  const VecT xyz0 = hn::LoadN(tag, xyz, 3);
  const VecT pqr0 = hn::LoadN(tag, pqr, 3);
  const VecT ssss = hn::Set(tag, stu[0]);
  const VecT tttt = hn::Set(tag, stu[1]);
  const VecT uuuu = hn::Set(tag, stu[2]);
  const VecT abc0 = hn::LoadN(tag, R, 3);
  const VecT def0 = hn::LoadN(tag, R + 3, 3);
  const VecT ghi0 = hn::LoadN(tag, R + 6, 3);
  const VecT ABC_ = hn::Add(hn::Abs((abc0)), eps4);
  const VecT DEF_ = hn::Add(hn::Abs((def0)), eps4);
  const VecT GHI_ = hn::Add(hn::Abs((ghi0)), eps4);

  // First category of cases separating along a's axes.
  VecT left = hn::Abs(xyz0);
  VecT right = pqr0;                      //  p   q   r   _
  right = hn::MulAdd(ssss, ABC_, right);  // +sA +sB +sC  _
  right = hn::MulAdd(tttt, DEF_, right);  // +tD +tE +tF  _
  right = hn::MulAdd(uuuu, GHI_, right);  // +uG +uH +uI  _
  if (GreaterThan(tag, left, right)) {
    return false;
  }

  const VecT stu0 = hn::LoadN(tag, stu, 3);
  const VecT pppp = hn::Set(tag, pqr[0]);
  const VecT qqqq = hn::Set(tag, pqr[1]);
  const VecT rrrr = hn::Set(tag, pqr[2]);
  const VecT xxxx = hn::Set(tag, xyz[0]);
  const VecT yyyy = hn::Set(tag, xyz[1]);
  const VecT zzzz = hn::Set(tag, xyz[2]);

  // Transpose R_AB to R_BA.
  // TODO(SeanCurtis-TRI) This is a duplicate of the logic in the fast pose
  // computations.
  const VecT abcd = hn::LoadU(tag, R);
  const VecT efgh = hn::LoadU(tag, R + 4);
  const VecT abgh = hn::ConcatUpperLower(tag, efgh, abcd);
  const VecT defg = hn::LoadU(tag, R + 3);
  const VecT adg_ = hn::InterleaveLower(tag, abgh, defg);
  const VecT beh_ = hn::InterleaveUpper(tag, abgh, defg);
  const VecT iiii = hn::Set(tag, R[8]);
  // According to the naming convention, this would *actually* be named __cf.
  // In ANSI C++, names can't start with double underscore. So, for this single
  // vector (with limited scope), we'll replace _ with capital O.
  const VecT OOcf = hn::InterleaveLower(abcd, defg);
  const VecT cfi_ = hn::ConcatUpperUpper(tag, iiii, OOcf);

  // Now absolute value plus epsilon.
  const VecT ADG_ = hn::Add(hn::Abs(adg_), eps4);
  const VecT BEH_ = hn::Add(hn::Abs(beh_), eps4);
  const VecT CFI_ = hn::Add(hn::Abs(cfi_), eps4);

  // Second category of cases separating along b's axes.
  left = hn::Mul(xxxx, adg_);           //  xa  xd  xg  _
  left = hn::MulAdd(yyyy, beh_, left);  // +yb +ye +th  _
  left = hn::MulAdd(zzzz, cfi_, left);  // +zc +zf +zi  _
  left = hn::Abs(left);
  right = stu0;                           //  s   t   u   _
  right = hn::MulAdd(pppp, ADG_, right);  // +pA +pD +pG  _
  right = hn::MulAdd(qqqq, BEH_, right);  // +pB +pE +pH  _
  right = hn::MulAdd(rrrr, CFI_, right);  // +rC +rF +rI  _
  if (GreaterThan(tag, left, right)) {
    return false;
  }

  // The template parameters for Per4LaneBlockShuffle() are enumerated *right to
  // left*. So, the first lane in the result is the *last* parameter value, etc.
  const VecT tus_ = hn::Per4LaneBlockShuffle<3, 0, 2, 1>(stu0);
  const VecT ust_ = hn::Per4LaneBlockShuffle<3, 1, 0, 2>(stu0);

  const VecT DGA_ = hn::Per4LaneBlockShuffle<3, 0, 2, 1>(ADG_);
  const VecT GAD_ = hn::Per4LaneBlockShuffle<3, 1, 0, 2>(ADG_);

  // Third category of cases separating along the axes formed from the cross
  // products of a's and b's axes. We'll do it in three groups, for i={0,1,2}.

  // i == 0
  left = hn::Mul(zzzz, beh_);              //  zb  ze  zh  _
  left = hn::NegMulAdd(yyyy, cfi_, left);  // -yc -yf -yi  _
  left = hn::Abs(left);
  right = hn::Mul(qqqq, CFI_);            //  qC  qF  qI  _
  right = hn::MulAdd(rrrr, BEH_, right);  // +rB +rE +rH  _
  right = hn::MulAdd(tus_, GAD_, right);  // +tG +uA +sD  _
  right = hn::MulAdd(ust_, DGA_, right);  // +uD +sG +tA  _
  if (GreaterThan(tag, left, right)) {
    return false;
  }

  const VecT EHB_ = hn::Per4LaneBlockShuffle<3, 0, 2, 1>(BEH_);
  const VecT HBE_ = hn::Per4LaneBlockShuffle<3, 1, 0, 2>(BEH_);

  // i == 1
  left = hn::Mul(xxxx, cfi_);              //  xc  xf  xi  _
  left = hn::NegMulAdd(zzzz, adg_, left);  // -za -zd -zg  _
  left = hn::Abs(left);
  right = hn::Mul(rrrr, ADG_);            //  rA  rD  rG  _
  right = hn::MulAdd(pppp, CFI_, right);  // +pC +pF +pI  _
  right = hn::MulAdd(tus_, HBE_, right);  // +tH +uB +sE  _
  right = hn::MulAdd(ust_, EHB_, right);  // +uE +sH +tB  _
  if (GreaterThan(tag, left, right)) {
    return false;
  }

  const VecT FIC_ = hn::Per4LaneBlockShuffle<3, 0, 2, 1>(CFI_);
  const VecT ICF_ = hn::Per4LaneBlockShuffle<3, 1, 0, 2>(CFI_);

  // i == 2
  left = hn::Mul(yyyy, adg_);              //  ya  yd  yg  _
  left = hn::NegMulAdd(xxxx, beh_, left);  // -xb -xe -xh  _
  left = hn::Abs(left);
  right = hn::Mul(pppp, BEH_);            //  pB  pE  pH  _
  right = hn::MulAdd(qqqq, ADG_, right);  // +qA +qD +qG  _
  right = hn::MulAdd(tus_, ICF_, right);  // +tI +uC +sF  _
  right = hn::MulAdd(ust_, FIC_, right);  // +uF +sI +tC  _
  if (GreaterThan(tag, left, right)) {
    return false;
  }

  return true;
}

#else  // HWY_MAX_BYTES

// See note in BoxesOverlap as to why the parameters are pointers.
// We're simply assuming that they "can't" be null.
bool BoxesOverlapImpl(const Vector3d* half_size_a_ptr,
                      const Vector3d* half_size_b_ptr,
                      const RigidTransformd* X_AB_ptr) {
  const Vector3d& half_size_a = *half_size_a_ptr;
  const Vector3d& half_size_b = *half_size_b_ptr;
  const math::RigidTransformd& X_AB = *X_AB_ptr;

  // We need to split the transform into the position and rotation components,
  // `p_AB` and `R_AB`. For the purposes of streamlining the math below, they
  // will henceforth be named `t` and `r` respectively.
  const Vector3d& t = X_AB.translation();
  const Matrix3d& r = X_AB.rotation().matrix();

  // Compute some common subexpressions and add epsilon to counteract
  // arithmetic error, e.g. when two edges are parallel. We use the value as
  // specified from Gottschalk's OBB robustness tests.
  const double kEpsilon = 0.000001;
  Matrix3d abs_r = r;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      abs_r(i, j) = abs(abs_r(i, j)) + kEpsilon;
    }
  }

  // First category of cases separating along a's axes.
  for (int i = 0; i < 3; ++i) {
    if (abs(t[i]) > half_size_a[i] + half_size_b.dot(abs_r.block<1, 3>(i, 0))) {
      return false;
    }
  }

  // Second category of cases separating along b's axes.
  for (int i = 0; i < 3; ++i) {
    if (abs(t.dot(r.block<3, 1>(0, i))) >
        half_size_b[i] + half_size_a.dot(abs_r.block<3, 1>(0, i))) {
      return false;
    }
  }

  // Third category of cases separating along the axes formed from the cross
  // products of a's and b's axes.
  int i1 = 1;
  for (int i = 0; i < 3; ++i) {
    const int i2 = (i1 + 1) % 3;  // Calculate common sub expressions.
    int j1 = 1;
    for (int j = 0; j < 3; ++j) {
      const int j2 = (j1 + 1) % 3;
      if (abs(t[i2] * r(i1, j) - t[i1] * r(i2, j)) >
          half_size_a[i1] * abs_r(i2, j) + half_size_a[i2] * abs_r(i1, j) +
              half_size_b[j1] * abs_r(i, j2) + half_size_b[j2] * abs_r(i, j1)) {
        return false;
      }
      j1 = j2;
    }
    i1 = i2;
  }

  return true;
}

#endif  // HWY_MAX_BYTES

}  // namespace HWY_NAMESPACE
}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
HWY_AFTER_NAMESPACE();

// This part of the file is only compiled once total, instead of once per CPU.
#if HWY_ONCE
namespace drake {
namespace geometry {
namespace internal {
namespace {

// Create the lookup tables for the per-CPU hwy implementation functions, and
// required functors that select from the lookup tables.
HWY_EXPORT(BoxesOverlapImpl);
struct ChooseBestBoxesOverlapImpl {
  auto operator()() { return HWY_DYNAMIC_POINTER(BoxesOverlapImpl); }
};

}  // namespace

bool BoxesOverlap(const Vector3<double>& half_size_a,
                  const Vector3<double>& half_size_b,
                  const math::RigidTransformd& X_AB) {
  // Note: LateBoundFunction currently copies the parameters (with no obvious
  // immediate solution). For that reason, the impl functions take pointers so
  // the cost of the copy is negligible. When we fix the late-bound
  // infrastructure these can go back to const references.
  return LateBoundFunction<ChooseBestBoxesOverlapImpl>::Call(
      &half_size_a, &half_size_b, &X_AB);
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
#endif  // HWY_ONCE
