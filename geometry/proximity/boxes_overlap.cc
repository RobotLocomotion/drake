#include "drake/geometry/proximity/boxes_overlap.h"

#if defined(__AVX2__) && defined(__FMA__)
#include <cstdint>

#include <immintrin.h>
#else
#error
#endif

namespace drake {
namespace geometry {
namespace internal {

using Eigen::Matrix3d;
using Eigen::Vector3d;

namespace {
// Turns d into <dddd>.
__m256d four(double d) {
  return _mm256_set1_pd(d);
}
// OMG there is really no intrinsic for this?
__m256d abs4(__m256d x) {
  return _mm256_andnot_pd(_mm256_set1_pd(-0.0), x);
}
}

// This implementation follows section 64 (on PDF page 102) of:
//  http://gamma.cs.unc.edu/users/gottschalk/main.pdf

bool BoxesOverlap(const Vector3d& pqr /* (aka half_size_a) */,
                  const Vector3d& stu /* (aka half_size_b) */,
                  const math::RigidTransformd& X_AB) {
  const Vector3d& xyz = X_AB.translation();
  const Matrix3d& R_AB = X_AB.rotation().matrix();
  // We'll be adding epsilon to counteract arithmetic error, e.g., when two
  // edges are parallel. We use the value as specified from Gottschalk's OBB
  // robustness tests.
  const double kEpsilon = 0.000001;

  constexpr uint64_t yes = uint64_t(1ull << 63);
  constexpr uint64_t no = uint64_t(0);
  const __m256i mask = _mm256_setr_epi64x(yes, yes, yes, no);
  const double* const R = &R_AB.coeffRef(0, 0);

  const __m256d xyz0 = _mm256_maskload_pd(&xyz.coeffRef(0), mask);
  const __m256d pqr0 = _mm256_maskload_pd(&pqr.coeffRef(0), mask);
  const __m256d ssss = four(stu[0]);
  const __m256d tttt = four(stu[1]);
  const __m256d uuuu = four(stu[2]);
  const __m256d abc0 = _mm256_maskload_pd(R, mask);
  const __m256d def0 = _mm256_maskload_pd(R + 3, mask);
  const __m256d ghi0 = _mm256_maskload_pd(R + 6, mask);
  const __m256d ABC_ = _mm256_add_pd(abs4(abc0), four(kEpsilon));
  const __m256d DEF_ = _mm256_add_pd(abs4(def0), four(kEpsilon));
  const __m256d GHI_ = _mm256_add_pd(abs4(ghi0), four(kEpsilon));

  // First category of cases separating along a's axes.
  __m256d left = abs4(xyz0);
  __m256d right = pqr0;                        //  p   q   r   _
  right = _mm256_fmadd_pd(ssss, ABC_, right);  // +sA +sB +sC  _
  right = _mm256_fmadd_pd(tttt, DEF_, right);  // +tD +tE +tF  _
  right = _mm256_fmadd_pd(uuuu, GHI_, right);  // +uG +uG +uI  _
  int cmp = _mm256_movemask_pd(_mm256_cmp_pd(left, right, _CMP_GT_OQ));
  if (cmp & 7) {
    return false;
  }

  const __m256d stu0 = _mm256_maskload_pd(&stu.coeffRef(0), mask);
  const __m256d pppp = four(pqr[0]);
  const __m256d qqqq = four(pqr[1]);
  const __m256d rrrr = four(pqr[2]);
  const __m256d xxxx = four(xyz[0]);
  const __m256d yyyy = four(xyz[1]);
  const __m256d zzzz = four(xyz[2]);
  const __m256d abcd = _mm256_loadu_pd(R);
  const __m256d efgh = _mm256_loadu_pd(R + 4);
  const __m256d eb_h = _mm256_blend_pd(abcd, efgh, 0b1101);
  const __m256d beh_ = _mm256_permute_pd(eb_h, 0b0101);
  const __m256d cdg_ = _mm256_permute2f128_pd(abcd, efgh, 0b00110001);
  const __m256d adg_ = _mm256_blend_pd(abcd, cdg_, 0b1110);
  const __m256d fghi = _mm256_loadu_pd(R + 5);
  const __m256d _fi_ = _mm256_permute_pd(fghi, 0b0100);
  const __m256d cfi_ = _mm256_blend_pd(cdg_, _fi_, 0b0110);
  const __m256d ADG_ = _mm256_add_pd(abs4(adg_), four(kEpsilon));
  const __m256d BEH_ = _mm256_add_pd(abs4(beh_), four(kEpsilon));
  const __m256d CFI_ = _mm256_add_pd(abs4(cfi_), four(kEpsilon));
  
  // Second category of cases separating along b's axes.
  left = _mm256_mul_pd(xxxx, adg_);          //  xa  xd  xg  _
  left = _mm256_fmadd_pd(yyyy, beh_, left);  // +yb +ye +th  _
  left = _mm256_fmadd_pd(zzzz, cfi_, left);  // +zc +zf +zi  _
  left = abs4(left);
  right = stu0;                                //  s   t   u   _
  right = _mm256_fmadd_pd(pppp, ADG_, right);  // +pA +pD +pG  _
  right = _mm256_fmadd_pd(qqqq, BEH_, right);  // +pB +pE +pH  _
  right = _mm256_fmadd_pd(rrrr, CFI_, right);  // +rC +rF +rI  _
  cmp = _mm256_movemask_pd(_mm256_cmp_pd(left, right, _CMP_GT_OQ));
  if (cmp & 7) {
    return false;
  }

  __m256d tus_;
  tus_[0] = stu0[1];
  tus_[1] = stu0[2];
  tus_[2] = stu0[0];
  tus_[3] = stu0[3];

  __m256d ust_;
  ust_[0] = stu0[2];
  ust_[1] = stu0[0];
  ust_[2] = stu0[1];
  ust_[3] = stu0[3];

  __m256d GAD_;
  GAD_[0] = ADG_[2];
  GAD_[1] = ADG_[0];
  GAD_[2] = ADG_[1];
  GAD_[3] = ADG_[3];

  __m256d DGA_;
  DGA_[0] = ADG_[1];
  DGA_[1] = ADG_[2];
  DGA_[2] = ADG_[0];
  DGA_[3] = ADG_[3];

  // Third category of cases separating along the axes formed from the cross
  // products of a's and b's axes. We'll do it in three groups, for i={0,1,2}.

  // i == 0
  left = _mm256_mul_pd(zzzz, beh_);
  left = _mm256_fnmadd_pd(yyyy, cfi_, left);
  left = abs4(left);
  right = _mm256_mul_pd(qqqq, CFI_);
  right = _mm256_fmadd_pd(rrrr, BEH_, right);
  right = _mm256_fmadd_pd(tus_, GAD_, right);
  right = _mm256_fmadd_pd(ust_, DGA_, right);
  cmp = _mm256_movemask_pd(_mm256_cmp_pd(left, right, _CMP_GT_OQ));
  if (cmp & 7) {
    return false;
  }

  __m256d EHB_;
  EHB_[0] = BEH_[1];
  EHB_[1] = BEH_[2];
  EHB_[2] = BEH_[0];
  EHB_[3] = BEH_[3];

  __m256d HBE_;
  HBE_[0] = BEH_[2];
  HBE_[1] = BEH_[0];
  HBE_[2] = BEH_[1];
  HBE_[3] = BEH_[3];

  // i == 1
  left = _mm256_mul_pd(xxxx, cfi_);
  left = _mm256_fnmadd_pd(zzzz, adg_, left);
  left = abs4(left);
  right = _mm256_mul_pd(rrrr, ADG_);
  right = _mm256_fmadd_pd(pppp, CFI_, right);
  right = _mm256_fmadd_pd(tus_, HBE_, right);
  right = _mm256_fmadd_pd(ust_, EHB_, right);
  cmp = _mm256_movemask_pd(_mm256_cmp_pd(left, right, _CMP_GT_OQ));
  if (cmp & 7) {
    return false;
  }

  __m256d FIC_;
  FIC_[0] = CFI_[1];
  FIC_[1] = CFI_[2];
  FIC_[2] = CFI_[0];
  FIC_[3] = CFI_[3];

  __m256d ICF_;
  ICF_[0] = CFI_[2];
  ICF_[1] = CFI_[0];
  ICF_[2] = CFI_[1];
  ICF_[3] = CFI_[3];

  // i == 2
  left = _mm256_mul_pd(yyyy, adg_);
  left = _mm256_fnmadd_pd(xxxx, beh_, left);
  left = abs4(left);
  right = _mm256_mul_pd(pppp, BEH_);
  right = _mm256_fmadd_pd(qqqq, ADG_, right);
  right = _mm256_fmadd_pd(tus_, ICF_, right);
  right = _mm256_fmadd_pd(ust_, FIC_, right);
  cmp = _mm256_movemask_pd(_mm256_cmp_pd(left, right, _CMP_GT_OQ));
  if (cmp & 7) {
    return false;
  }

  return true;
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
