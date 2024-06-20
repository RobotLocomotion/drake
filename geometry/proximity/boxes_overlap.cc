#include "drake/geometry/proximity/boxes_overlap.h"

#if defined(__AVX2__) && defined(__FMA__)
#include <cstdint>

#include <cpuid.h>

// TODO(SeanCurtis-TRI): This pragma/define boilerplate was first defined (and
// documented) in fast_pose_composition_functions_avx2_fma.cc. It would be
// better if we refactored it so that including highway simply did the right
// configuring for Drake.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wold-style-cast"
#define HWY_BASELINE_TARGETS HWY_AVX2
#define HWY_DISABLE_PCLMUL_AES 1
#include "hwy/highway.h"
#include "hwy/print-inl.h"

#pragma GCC diagnostic pop
#else
// TODO: I need to handle the case where AVX2/FMA is not supported.
// Presumably, that's simply the old code preserved.
#error
#endif

namespace drake {
namespace geometry {
namespace internal {
#if defined(__AVX2__) && defined(__FMA__) || 1
using Eigen::Matrix3d;
using Eigen::Vector3d;

namespace {

// The hn namespace holds the CPU-specific function overloads. By defining it
// using a substitute-able macro, we achieve per-CPU instruction selection.
namespace hn = hwy::HWY_NAMESPACE;

// TODO: How do I get this from highway?
// OMG there is really no intrinsic for this?
template <typename VecType>
VecType abs4(VecType x) {
  const hn::FixedTag<double, 4> tag;
  return hn::AndNot(hn::Set(tag, -0.0), x);
}

}  // namespace

// This implementation follows section 64 (on PDF page 102) of:
//  http://gamma.cs.unc.edu/users/gottschalk/main.pdf

bool BoxesOverlap(const Vector3d& half_size_a, const Vector3d& half_size_b,
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
#else

bool BoxesOverlap(const Vector3d&, const Vector3d&, const Vector3d&,
                  const Matrix3d&) {
  // TODO: Use the CPU implementation here.
  std::cerr << "abort: BoxesOverlap() currently not enabled. " << std::endl;
  std::abort();
}

#endif

}  // namespace internal
}  // namespace geometry
}  // namespace drake
