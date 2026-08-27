#include "drake/planning/continuous_collision/certificate.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <fmt/format.h>

#include "drake/common/drake_assert.h"
#include "drake/planning/continuous_collision/certifier_internal.h"
#include "drake/planning/continuous_collision/numerics.h"

namespace drake {
namespace planning {
namespace continuous_collision {
namespace internal {
namespace {

/* Slop allowed between the certifier's arithmetic and the replay's. The two
 compute the same quantities by *different* routes, repeated halving versus a
 pair of arbitrary-u de Casteljau subdivisions, so they agree only to rounding.
 Both routes are sequences of convex combinations, hence numerically benign.
 This tolerance sits far below anything a tamperer could hide in and far above
 the rounding gap. */
constexpr double kReplayTolerance = 1e-9;

/* One de Casteljau subdivision at u ∈ [0, 1]: `left` receives the control
 points of the restriction to [0, u] and `right` those of the restriction to
 [u, 1] (both n × (m+1)). The triangle b_j^r = (1−u)·b_j^{r−1} + u·b_{j+1}^{r−1}
 is built in place inside `right`; its first column after round r is the left
 child's r-th control point and the column it leaves at m−r is the right
 child's. Written out here, rather than reused from the curve module, so that
 the replay is genuinely independent of the code path it audits. */
void SplitAt(const Eigen::MatrixXd& cps, double u, Eigen::MatrixXd* left,
             Eigen::MatrixXd* right) {
  const int m = static_cast<int>(cps.cols()) - 1;
  left->resize(cps.rows(), cps.cols());
  *right = cps;
  left->col(0) = cps.col(0);
  for (int r = 1; r <= m; ++r) {
    for (int j = 0; j <= m - r; ++j) {
      right->col(j) = (1.0 - u) * right->col(j) + u * right->col(j + 1);
    }
    left->col(r) = right->col(0);
  }
}

}  // namespace

void RestrictBezier(const Eigen::MatrixXd& cps, double a, double b,
                    Eigen::MatrixXd* out) {
  DRAKE_DEMAND(out != nullptr);
  const double lo = std::clamp(a, 0.0, 1.0);
  const double hi = std::clamp(b, 0.0, 1.0);
  if (lo <= 0.0 && hi >= 1.0) {
    *out = cps;
    return;
  }
  Eigen::MatrixXd scratch;
  Eigen::MatrixXd tail;
  if (lo <= 0.0) {
    tail = cps;
  } else {
    SplitAt(cps, lo, &scratch, &tail);
  }
  // `tail` is the curve on [lo, 1] in its own parameter v ∈ [0, 1]; the
  // original parameter hi lands at v = (hi − lo)/(1 − lo).
  const double span = 1.0 - lo;
  const double v = (span > 0.0) ? std::clamp((hi - lo) / span, 0.0, 1.0) : 1.0;
  if (v >= 1.0) {
    *out = std::move(tail);
    return;
  }
  SplitAt(tail, v, out, &scratch);
}

Eigen::VectorXd EvaluateBezier(const Eigen::MatrixXd& cps, double u) {
  const int m = static_cast<int>(cps.cols()) - 1;
  Eigen::MatrixXd work = cps;
  for (int r = 1; r <= m; ++r) {
    for (int j = 0; j <= m - r; ++j) {
      work.col(j) = (1.0 - u) * work.col(j) + u * work.col(j + 1);
    }
  }
  return work.col(0);
}

bool ReplayCertificate(const ReplayInput& input, const Certificate& certificate,
                       std::string* message) {
  DRAKE_DEMAND(input.model != nullptr);
  DRAKE_DEMAND(input.oracle != nullptr);
  DRAKE_DEMAND(input.table != nullptr);
  DRAKE_DEMAND(input.path != nullptr);
  DRAKE_DEMAND(input.pairs != nullptr);
  DRAKE_DEMAND(input.tau != nullptr);

  const auto fail = [message](std::string reason) {
    if (message != nullptr) *message = std::move(reason);
    return false;
  };

  const PiecewiseBezierPath& path = *input.path;
  const std::vector<PairRecord>& pairs = *input.pairs;
  const std::vector<double>& tau = *input.tau;
  const MotionBoundTable& table = *input.table;
  const int num_pairs = static_cast<int>(pairs.size());
  const int num_segments = static_cast<int>(path.segments().size());
  const int num_positions = path.num_positions();

  // --- 1. The pair snapshot must be the checker's own table. ---------------
  // Without this the record indices mean nothing, and every later check could
  // be aimed at the wrong geometries.
  if (static_cast<int>(certificate.pairs.size()) != num_pairs) {
    return fail(
        fmt::format("certificate covers {} pair(s) but the checker has {}.",
                    certificate.pairs.size(), num_pairs));
  }
  for (int p = 0; p < num_pairs; ++p) {
    if (certificate.pairs[p].a != pairs[p].id.a ||
        certificate.pairs[p].b != pairs[p].id.b) {
      return fail(fmt::format(
          "certificate pair {} does not match the checker's pair table.", p));
    }
  }
  if (table.num_pairs() != num_pairs) {
    return fail("the motion-bound table does not match the pair table.");
  }

  // --- 2. Replay every record. ---------------------------------------------
  ThreadContext context(*input.model);
  Eigen::MatrixXd restricted;
  Eigen::VectorXd w(num_positions);
  Eigen::VectorXd last_qc;
  std::vector<double> claimed_threshold(
      num_pairs, std::numeric_limits<double>::quiet_NaN());

  for (int r = 0; r < static_cast<int>(certificate.records.size()); ++r) {
    const CertificateRecord& record = certificate.records[r];
    const int p = record.pair_index;
    if (p < 0 || p >= num_pairs) {
      return fail(
          fmt::format("record {} names pair index {}, out of range.", r, p));
    }
    if (record.segment < 0 || record.segment >= num_segments) {
      return fail(fmt::format("record {} names segment {}, out of range.", r,
                              record.segment));
    }
    if (!(record.s_start >= 0.0) || !(record.s_end <= 1.0) ||
        !(record.s_start < record.s_end)) {
      return fail(fmt::format(
          "record {} has a degenerate or out-of-range interval [{}, {}].", r,
          record.s_start, record.s_end));
    }
    if (record.qc.size() != num_positions) {
      return fail(fmt::format(
          "record {} carries a representative configuration of size {}; the "
          "plant has {} positions.",
          r, record.qc.size(), num_positions));
    }
    if (!std::isfinite(record.phi_hat) || !std::isfinite(record.motion_bound) ||
        !std::isfinite(record.threshold) || record.motion_bound < 0.0) {
      return fail(
          fmt::format("record {} carries non-finite or negative data.", r));
    }
    // Every record of a pair must claim the same threshold: a certificate that
    // silently lowers m_p on some intervals proves nothing coherent.
    if (std::isnan(claimed_threshold[p])) {
      claimed_threshold[p] = record.threshold;
    } else if (claimed_threshold[p] != record.threshold) {
      return fail(fmt::format(
          "pair {} is certified against two different thresholds ({} and {}).",
          p, claimed_threshold[p], record.threshold));
    }
    // ... and the threshold it claims must be at least the one the caller
    // expects. Checking only self-consistency would let a certificate whose
    // records all say "threshold = -1e9" verify: it would be a true statement
    // about a claim nobody asked for.
    const double expected = pairs[p].threshold;
    if (!(record.threshold >=
          expected - kReplayTolerance * std::max(1.0, std::abs(expected)))) {
      return fail(fmt::format(
          "record {}: pair {} is certified only against threshold {}, which is "
          "below the {} the checker's options call for.",
          r, p, record.threshold, expected));
    }

    const bool is_static = table.pair_is_static(p);
    // A static pair's J(p) is empty, so MotionBound() would return exactly the
    // carve-out slack for any w: the residual of the coordinates the carve-out
    // removed, which is nonzero only when some of them are constant merely to
    // within Options::continuity_tolerance. Charging it here keeps the replay's
    // Δ at least as large as the certifier's: a certificate emitted against
    // a slack-inflated bound must not verify against a smaller one.
    double motion_bound = table.carveout_slack(p);
    if (!is_static) {
      // Re-restrict the segment's control points to the record's interval and
      // recompute w about the record's qc from scratch. This is the half of
      // the certificate the checker must not be believed on.
      RestrictBezier(path.segments()[record.segment].control_points,
                     record.s_start, record.s_end, &restricted);

      // qc must be the node's own midpoint apex, i.e. a configuration exactly
      // on the trajectory. (A qc merely inside the control box would still be
      // sound by the displacement lemma, but pinning it to the apex is what
      // the certifier emits, and it makes a tampered qc detectable.)
      const Eigen::VectorXd apex = EvaluateBezier(restricted, 0.5);
      const double qc_error = (apex - record.qc).cwiseAbs().maxCoeff();
      // Relative: a plant with large coordinate values (an unbounded prismatic
      // joint, say) rounds proportionally, and the check must not turn into a
      // scale-dependent false alarm.
      const double qc_limit =
          kReplayTolerance * std::max(1.0, record.qc.cwiseAbs().maxCoeff());
      if (!(qc_error <= qc_limit)) {
        return fail(fmt::format(
            "record {}: the stored representative configuration is not the "
            "midpoint of the interval it claims (off by {}).",
            r, qc_error));
      }

      w.setZero();
      for (int j = 0; j < restricted.cols(); ++j) {
        for (int i = 0; i < num_positions; ++i) {
          w[i] = std::max(w[i], std::abs(restricted(i, j) - record.qc[i]));
        }
      }
      motion_bound = table.MotionBound(p, w);
      if (!(record.motion_bound >=
            motion_bound - kReplayTolerance * std::max(1.0, motion_bound))) {
        return fail(fmt::format(
            "record {}: the stored motion bound {} understates the recomputed "
            "bound {}.",
            r, record.motion_bound, motion_bound));
      }
    }
    // For a static pair J(p) = ∅: no coordinate the trajectory *moves* changes
    // the pair's relative pose, so Δ_p is the constant carve-out slack (0 in
    // every case but a tolerance-constant coordinate) and one measurement
    // certifies the whole domain. A *non*-static pair cannot smuggle in such a
    // record: the recomputed Δ above would be the full node's bound and the
    // test below would reject it.
    //
    // "Static" is relative to the constant-coordinate carve-out, so
    // coordinates this path happens to hold fixed still move the pair in
    // general. The representative configuration therefore has to be pinned to
    // the path, exactly as the certifier pins it (q(t0)), or a record could be
    // re-based onto an off-path configuration that measures more clearance.
    if (is_static) {
      const Eigen::VectorXd q0 = path.segments()[0].control_points.col(0);
      const double qc_error = (q0 - record.qc).cwiseAbs().maxCoeff();
      if (!(qc_error <= kReplayTolerance *
                            std::max(1.0, record.qc.cwiseAbs().maxCoeff()))) {
        return fail(fmt::format(
            "record {}: pair {} is certified statically from a configuration "
            "that is not the path's start (off by {}).",
            r, p, qc_error));
      }
    }

    // Re-measure the distance ourselves. Records are emitted sorted, so the
    // several pairs certified at one node arrive adjacently and share a qc;
    // skipping the redundant SetPositions saves that many forward-kinematics
    // evaluations on what is otherwise a linear scan of the whole audit trail.
    if (last_qc.size() != record.qc.size() ||
        !(last_qc.array() == record.qc.array()).all()) {
      context.SetPositions(record.qc);
      last_qc = record.qc;
    }
    const double phi_replay =
        input.oracle->SignedDistance(context.query_object(), pairs[p]);
    const double tau_p = tau[p];
    // Coherence: a record may legitimately store *less* than the narrowphase
    // reports (the sphere-prefilter branch stores a lower bound on ϕ), but it
    // may never claim more than the oracle's own contract allows.
    if (!(record.phi_hat <= phi_replay + tau_p + kReplayTolerance)) {
      return fail(fmt::format(
          "record {}: the stored clearance {} over-reports the independently "
          "measured {} (pair {}).",
          r, record.phi_hat, phi_replay, p));
    }
    // The certificate test runs on min(stored, re-measured), so an inflated
    // ϕ̂ can never buy a record anything: only the value this replay measured
    // for itself can carry the inequality. Both are lower bounds we are
    // entitled to charge τ_p against, and for an untampered record the stored
    // value is the smaller one (identical for a narrowphase record, the
    // sphere bound for a prefilter record), so nothing legitimate is lost.
    const double effective_phi = std::min(record.phi_hat, phi_replay);
    if (!IsCertified(effective_phi, tau_p, motion_bound, record.threshold,
                     input.slack)) {
      return fail(fmt::format(
          "record {}: phi_hat {} - tau {} - Delta {} does not exceed threshold "
          "{} + slack {} (pair {}, segment {}, [{}, {}]).",
          r, effective_phi, tau_p, motion_bound, record.threshold, input.slack,
          p, record.segment, record.s_start, record.s_end));
    }
  }

  // --- 3. Coverage. --------------------------------------------------------
  // The certified intervals must tile [0, 1] of every segment for every pair.
  // Without this a certificate could consist of a handful of perfectly valid
  // records and still prove nothing about the parts of the path they miss.
  struct Interval {
    int pair;
    int segment;
    double lo;
    double hi;
  };
  std::vector<Interval> intervals;
  intervals.reserve(certificate.records.size());
  for (const CertificateRecord& record : certificate.records) {
    intervals.push_back(Interval{record.pair_index, record.segment,
                                 record.s_start, record.s_end});
  }
  std::sort(intervals.begin(), intervals.end(),
            [](const Interval& a, const Interval& b) {
              if (a.pair != b.pair) return a.pair < b.pair;
              if (a.segment != b.segment) return a.segment < b.segment;
              return a.lo < b.lo;
            });

  std::size_t cursor = 0;
  for (int p = 0; p < num_pairs; ++p) {
    for (int k = 0; k < num_segments; ++k) {
      const std::size_t begin = cursor;
      while (cursor < intervals.size() && intervals[cursor].pair == p &&
             intervals[cursor].segment == k) {
        ++cursor;
      }
      double covered_to = 0.0;
      for (std::size_t i = begin; i < cursor; ++i) {
        // Sorted by lo, so a start beyond the covered prefix is a real gap.
        // Compared exactly: the certifier's intervals are
        // dyadic and abut bit-for-bit (a child's endpoint *is* the parent's
        // computed midpoint), so any slack here would only buy a forged
        // certificate the right to excise a sliver at every one of its
        // thousands of record boundaries.
        if (intervals[i].lo > covered_to) break;
        covered_to = std::max(covered_to, intervals[i].hi);
      }
      if (!(covered_to >= 1.0)) {
        return fail(fmt::format(
            "pair {} is certified only up to s = {} of segment {}; the "
            "certificate does not cover the whole path.",
            p, covered_to, k));
      }
    }
  }

  if (message != nullptr) message->clear();
  return true;
}

}  // namespace internal
}  // namespace continuous_collision
}  // namespace planning
}  // namespace drake
