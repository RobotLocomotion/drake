#include "drake/geometry/proximity/boxes_overlap.h"

// This is the magic juju that compiles our impl functions for multiple CPUs.
#undef HWY_TARGET_INCLUDE
#define HWY_TARGET_INCLUDE "geometry/proximity/boxes_overlap.cc"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wold-style-cast"
#include "hwy/foreach_target.h"
#include "hwy/highway.h"
#pragma GCC diagnostic pop

#include "drake/common/hwy_dynamic_impl.h"

HWY_BEFORE_NAMESPACE();
namespace drake {
namespace geometry {
namespace internal {

using Eigen::Vector3d;
using Eigen::Matrix3d;
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

// TODO: How do I get this from highway?
// OMG there is really no intrinsic for this?
template <typename VecType>
VecType abs4(VecType x) {
  const hn::FixedTag<double, 4> tag;
  return hn::AndNot(hn::Set(tag, -0.0), x);
}

// This implementation follows section 64 (on PDF page 102) of:
//  http://gamma.cs.unc.edu/users/gottschalk/main.pdf

bool BoxesOverlapImpl(const Vector3d& half_size_a, const Vector3d& half_size_b,
                      const math::RigidTransformd& X_AB) {
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

  const VecT eps4 = hn::Set(tag, kEpsilon);
  const VecT xyz0 = hn::LoadN(tag, xyz, 3);
  const VecT pqr0 = hn::LoadN(tag, pqr, 3);
  const VecT ssss = hn::Set(tag, stu[0]);
  const VecT tttt = hn::Set(tag, stu[1]);
  const VecT uuuu = hn::Set(tag, stu[2]);
  const VecT abc0 = hn::LoadN(tag, R, 3);
  const VecT def0 = hn::LoadN(tag, R + 3, 3);
  const VecT ghi0 = hn::LoadN(tag, R + 6, 3);
  const VecT ABC_ = hn::Add(abs4((abc0)), eps4);
  const VecT DEF_ = hn::Add(abs4((def0)), eps4);
  const VecT GHI_ = hn::Add(abs4((ghi0)), eps4);

  // First category of cases separating along a's axes.
  VecT left = abs4(xyz0);
  VecT right = pqr0;                      //  p   q   r   _
  right = hn::MulAdd(ssss, ABC_, right);  // +sA +sB +sC  _
  right = hn::MulAdd(tttt, DEF_, right);  // +tD +tE +tF  _
  right = hn::MulAdd(uuuu, GHI_, right);  // +uG +uG +uI  _
  auto cmp = hn::Gt(left, right);
  if (!hn::AllFalse(tag, cmp) && hn::FindFirstTrue(tag, cmp) < 3) {
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
  const VecT abcd = hn::LoadU(tag, R);
  const VecT efgh = hn::LoadU(tag, R + 4);
  const VecT abgh = hn::ConcatUpperLower(tag, efgh, abcd);
  const VecT defg = hn::LoadU(tag, R + 3);
  const VecT adg_ = hn::InterleaveLower(tag, abgh, defg);
  const VecT beh_ = hn::InterleaveUpper(tag, abgh, defg);
  const VecT iiii = hn::Set(tag, R[8]);
  const VecT __cf = hn::InterleaveLower(abcd, defg);
  const VecT cfi_ = hn::ConcatUpperUpper(tag, iiii, __cf);

  // Now absolute value plus epsilon.
  const VecT ADG_ = hn::Add(abs4(adg_), eps4);
  const VecT BEH_ = hn::Add(abs4(beh_), eps4);
  const VecT CFI_ = hn::Add(abs4(cfi_), eps4);

  // Second category of cases separating along b's axes.
  left = hn::Mul(xxxx, adg_);           //  xa  xd  xg  _
  left = hn::MulAdd(yyyy, beh_, left);  // +yb +ye +th  _
  left = hn::MulAdd(zzzz, cfi_, left);  // +zc +zf +zi  _
  left = abs4(left);
  right = stu0;                           //  s   t   u   _
  right = hn::MulAdd(pppp, ADG_, right);  // +pA +pD +pG  _
  right = hn::MulAdd(qqqq, BEH_, right);  // +pB +pE +pH  _
  right = hn::MulAdd(rrrr, CFI_, right);  // +rC +rF +rI  _
  cmp = hn::Gt(left, right);
  if (!hn::AllFalse(tag, cmp) && hn::FindFirstTrue(tag, cmp) < 3) {
    return false;
  }

  const VecT tus_ = hn::Per4LaneBlockShuffle<3, 0, 2, 1>(stu0);
  const VecT ust_ = hn::Per4LaneBlockShuffle<3, 1, 0, 2>(stu0);

  const VecT DGA_ = hn::Per4LaneBlockShuffle<3, 0, 2, 1>(ADG_);
  const VecT GAD_ = hn::Per4LaneBlockShuffle<3, 1, 0, 2>(ADG_);

  // Third category of cases separating along the axes formed from the cross
  // products of a's and b's axes. We'll do it in three groups, for i={0,1,2}.

  // i == 0
  left = hn::Mul(zzzz, beh_);
  left = hn::NegMulAdd(yyyy, cfi_, left);
  left = abs4(left);
  right = hn::Mul(qqqq, CFI_);
  right = hn::MulAdd(rrrr, BEH_, right);
  right = hn::MulAdd(tus_, GAD_, right);
  right = hn::MulAdd(ust_, DGA_, right);
  cmp = hn::Gt(left, right);
  if (!hn::AllFalse(tag, cmp) && hn::FindFirstTrue(tag, cmp) < 3) {
    return false;
  }

  const VecT EHB_ = hn::Per4LaneBlockShuffle<3, 0, 2, 1>(BEH_);
  const VecT HBE_ = hn::Per4LaneBlockShuffle<3, 1, 0, 2>(BEH_);

  // i == 1
  left = hn::Mul(xxxx, cfi_);
  left = hn::NegMulAdd(zzzz, adg_, left);
  left = abs4(left);
  right = hn::Mul(rrrr, ADG_);
  right = hn::MulAdd(pppp, CFI_, right);
  right = hn::MulAdd(tus_, HBE_, right);
  right = hn::MulAdd(ust_, EHB_, right);
  cmp = hn::Gt(left, right);
  if (!hn::AllFalse(tag, cmp) && hn::FindFirstTrue(tag, cmp) < 3) {
    return false;
  }

  const VecT FIC_ = hn::Per4LaneBlockShuffle<3, 0, 2, 1>(CFI_);
  const VecT ICF_ = hn::Per4LaneBlockShuffle<3, 1, 0, 2>(CFI_);

  // i == 2
  left = hn::Mul(yyyy, adg_);
  left = hn::NegMulAdd(xxxx, beh_, left);
  left = abs4(left);
  right = hn::Mul(pppp, BEH_);
  right = hn::MulAdd(qqqq, ADG_, right);
  right = hn::MulAdd(tus_, ICF_, right);
  right = hn::MulAdd(ust_, FIC_, right);
  cmp = hn::Gt(left, right);
  if (!hn::AllFalse(tag, cmp) && hn::FindFirstTrue(tag, cmp) < 3) {
    return false;
  }

  return true;
}

#else  // HWY_MAX_BYTES

bool BoxesOverlapImpl(const Vector3d& half_size_a, const Vector3d& half_size_b,
                      const RigidTransformd& X_AB) {
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

}  // HWY_NAMESPACE
}  //
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
  return LateBoundFunction<ChooseBestBoxesOverlapImpl>::Call(
      half_size_a, half_size_b, X_AB);
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
#endif  // HWY_ONCE
