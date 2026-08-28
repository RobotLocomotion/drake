// Tests VerifyCertificate, which replays every certification event from the
// checker's public seams, re-restricting control points, recomputing motion
// bounds and re-querying distances, then checks that the certified intervals
// tile the whole domain for every pair.
//
// The corpus is three certified runs: one hand-built world whose two pairs are
// built to certify at very different depths, plus two small random worlds.
// Below it is a table of mutations, each applied to every corpus case; a
// mutation any case accepts is a hole in the audit.

#include <algorithm>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/planning/continuous_collision/test/test_utilities.h"

namespace drake {
namespace planning {
namespace continuous_collision {
namespace {

using Eigen::Vector3d;
using Eigen::VectorXd;
using test::BezierCurve;
using test::Box;
using test::Friction;
using test::Inertia;
using test::MakeRandomWorld;
using test::MultibodyPlant;
using test::Parallelism;
using test::PrismaticJoint;
using test::RigidBody;
using test::RigidTransformd;
using test::RobotDiagram;
using test::RobotDiagramBuilder;
using test::Sphere;

// A non-zero margin and a non-zero environment padding, so m_p = margin +
// padding is a number a tamperer could plausibly try to lower and the
// "threshold below what the options call for" branch has something to bite on.
constexpr double kMargin = 0.005;
constexpr double kEnvPadding = 0.002;

Options AuditOptions() {
  Options options;
  options.margin = kMargin;
  options.parallelism = Parallelism::None();
  options.emit_certificate = true;
  return options;
}

std::unique_ptr<ContinuousCollisionChecker> MakeAuditChecker(
    std::shared_ptr<const RobotDiagram<double>> model) {
  PaddingSpec padding;
  padding.env_padding = kEnvPadding;
  padding.self_padding = kEnvPadding;
  return test::MakeCheckerPtr(std::move(model), AuditOptions(), padding);
}

// A 2-dof Cartesian gantry (prismatic x, prismatic y) carrying a 5 mm sphere,
// with exactly two unfiltered pairs:
//
//   * tool vs. "near_plate": a 1 mm plate offset in y so the clearance is a
//     constant 12 mm. With m_p = 0.007 and λ = 1 for the moving x coordinate,
//     certification needs Δ = w_x < 0.012 − 0.007 − τ ≈ 0.005 against 0.6 m of
//     travel, so it first certifies at depth 6 and produces dozens of records.
//   * tool vs. "far_ball": 3 m away, certified by the sphere prefilter at the
//     root, so exactly one record per segment.
//
// The record-relabelling mutation needs two pairs this far apart in difficulty.
std::unique_ptr<RobotDiagram<double>> MakeDesignedWorld() {
  RobotDiagramBuilder<double> builder;
  MultibodyPlant<double>& plant = builder.plant();
  const RigidBody<double>& carriage = plant.AddRigidBody("carriage", Inertia());
  const RigidBody<double>& tool = plant.AddRigidBody("tool", Inertia());
  plant.AddJoint<PrismaticJoint>("gantry_x", plant.world_body(), {}, carriage,
                                 {}, Vector3d::UnitX());
  plant.AddJoint<PrismaticJoint>("gantry_y", carriage, {}, tool, {},
                                 Vector3d::UnitY());
  plant.RegisterCollisionGeometry(tool, RigidTransformd(), Sphere(0.005),
                                  "tool_geom", Friction());

  const RigidBody<double>& plate = plant.AddRigidBody("near_plate", Inertia());
  plant.WeldFrames(plant.world_frame(), plate.body_frame(),
                   RigidTransformd(Vector3d(0.0, 0.0175, 0.0)));
  plant.RegisterCollisionGeometry(plate, RigidTransformd(),
                                  Box(0.9, 0.001, 0.6), "near_plate_geom",
                                  Friction());

  const RigidBody<double>& ball = plant.AddRigidBody("far_ball", Inertia());
  plant.WeldFrames(plant.world_frame(), ball.body_frame(),
                   RigidTransformd(Vector3d(0.0, 3.0, 0.0)));
  plant.RegisterCollisionGeometry(ball, RigidTransformd(), Sphere(0.05),
                                  "far_ball_geom", Friction());
  return builder.Build();
}

// A cubic Bézier whose control points are equally spaced from `start` to `end`.
// It is the straight segment, but with four control points, so a mutation can
// perturb an interior one without moving either endpoint; moving an endpoint
// would change the path's start configuration and short-circuit the check.
Eigen::MatrixXd CubicControlPoints(const VectorXd& start, const VectorXd& end) {
  Eigen::MatrixXd points(start.size(), 4);
  for (int j = 0; j < 4; ++j) {
    const double u = j / 3.0;
    points.col(j) = (1.0 - u) * start + u * end;
  }
  return points;
}

struct AuditCase {
  std::string name;
  std::shared_ptr<const RobotDiagram<double>> model;
  std::unique_ptr<ContinuousCollisionChecker> checker;
  Eigen::MatrixXd control_points;
  std::optional<PiecewiseBezierPath> path;
  Certificate certificate;
  // True when this case's two pairs were built to have wildly different
  // certification depths (only the hand-built world).
  bool designed{false};

  // The path a verifier would be handed if one control point were nudged.
  PiecewiseBezierPath PerturbedPath(double delta) const {
    Eigen::MatrixXd points = control_points;
    points(0, 1) += delta;
    return checker->Normalize(BezierCurve<double>(0.0, 1.0, points),
                              AuditOptions());
  }

  bool Verify(const Certificate& certificate_in) const {
    return VerifyCertificate(*checker, *path, certificate_in);
  }
};

// Builds the corpus once. Every entry is a run that ended
// Verdict::kCertifiedFree with an emitted certificate; a case that failed to
// certify is dropped rather than added, so CorpusIsBuiltAndVerifies is the
// single place that reports a short corpus. No gtest assertion belongs here:
// this initializer runs inside whichever test touches Corpus() first, which
// changes under --gtest_filter or --gtest_shuffle.
//
// The vector is allocated and never freed because it owns RobotDiagrams and
// checkers whose destruction would otherwise race Drake's static teardown.
const std::vector<std::unique_ptr<AuditCase>>& Corpus() {
  static const std::vector<std::unique_ptr<AuditCase>>* corpus = [] {
    auto* cases = new std::vector<std::unique_ptr<AuditCase>>();
    const Options options = AuditOptions();
    const auto add = [&cases, &options](std::unique_ptr<AuditCase> entry) {
      const BezierCurve<double> trajectory(0.0, 1.0, entry->control_points);
      const CertificationResult result =
          entry->checker->CheckTrajectory(trajectory, options);
      if (result.verdict != Verdict::kCertifiedFree) return;
      entry->path = entry->checker->Normalize(trajectory, options);
      entry->certificate = *result.certificate;
      cases->push_back(std::move(entry));
    };

    {  // 1. The designed world.
      auto entry = std::make_unique<AuditCase>();
      entry->name = "designed_gantry";
      entry->designed = true;
      entry->model = MakeDesignedWorld();
      entry->checker = MakeAuditChecker(entry->model);
      VectorXd start(2), end(2);
      start << -0.3, 0.0;
      end << 0.3, 0.0;
      entry->control_points = CubicControlPoints(start, end);
      add(std::move(entry));
    }

    // 2. Small random worlds: the first two seeds whose trajectory certifies.
    //    Sweeping deterministically, rather than hard-coding lucky seeds, still
    //    fills the corpus if the geometry ever shifts underneath it.
    for (uint64_t seed = 1; seed <= 60 && cases->size() < 3; ++seed) {
      auto entry = std::make_unique<AuditCase>();
      entry->name = "random_world_seed_" + std::to_string(seed);
      test::WorldSpec spec;
      spec.num_links = 3;
      spec.num_obstacles = 3;
      spec.floor = false;
      entry->model = MakeRandomWorld(seed, spec);
      entry->checker = MakeAuditChecker(entry->model);
      const int n = entry->model->plant().num_positions();
      VectorXd start = VectorXd::Zero(n);
      VectorXd end = VectorXd::Zero(n);
      for (int i = 0; i < n; ++i) {
        start[i] = 0.15 * ((i % 2 == 0) ? 1.0 : -1.0);
        end[i] = start[i] + 0.25;
      }
      entry->control_points = CubicControlPoints(start, end);
      add(std::move(entry));
    }
    return cases;
  }();
  return *corpus;
}

// Record counts per pair, for picking "the hardest" and "the easiest" pair.
std::vector<int> RecordsPerPair(const AuditCase& entry) {
  std::vector<int> counts(entry.certificate.pairs.size(), 0);
  for (const CertificateRecord& record : entry.certificate.records) {
    ++counts[record.pair_index];
  }
  return counts;
}

// True iff `pair`'s records cover [0, 1] of every segment. This is the coverage
// property VerifyCertificate checks, re-derived here so a test can assert that
// a mutation left coverage intact and therefore had to be caught by the
// per-record arithmetic instead.
bool TilesEverySegment(const Certificate& certificate, int pair,
                       std::size_t num_segments) {
  for (std::size_t segment = 0; segment < num_segments; ++segment) {
    std::vector<std::pair<double, double>> intervals;
    for (const CertificateRecord& record : certificate.records) {
      if (record.pair_index == pair &&
          record.segment == static_cast<int>(segment)) {
        intervals.emplace_back(record.s_start, record.s_end);
      }
    }
    std::sort(intervals.begin(), intervals.end());
    double covered_to = 0.0;
    for (const auto& [lo, hi] : intervals) {
      if (lo > covered_to) break;
      covered_to = std::max(covered_to, hi);
    }
    if (!(covered_to >= 1.0)) return false;
  }
  return true;
}

// Index of a record whose pair the trajectory actually moves and whose interval
// is a proper sub-interval, which is the kind a tamperer would target.
int MovingRecordIndex(const AuditCase& entry) {
  const MotionBoundTable table =
      entry.checker->ComputeMotionBounds(*entry.path);
  for (int i = 0; i < static_cast<int>(entry.certificate.records.size()); ++i) {
    const CertificateRecord& record = entry.certificate.records[i];
    if (!table.pair_is_static(record.pair_index) && record.s_end < 1.0) {
      return i;
    }
  }
  return -1;
}

// ---------------------------------------------------------------------------
// 1. Baseline.
// ---------------------------------------------------------------------------

GTEST_TEST(CertificateAuditTest, CorpusIsBuiltAndVerifies) {
  const auto& corpus = Corpus();
  ASSERT_GE(corpus.size(), 3u)
      << "the corpus needs the designed world plus at least two random ones; a "
         "case that failed to certify is dropped rather than added empty, so a "
         "short corpus means a run stopped certifying";
  ASSERT_TRUE(corpus.front()->designed)
      << "the designed world must be first: the mutations that need its pair "
         "structure index Corpus().front()";
  for (const auto& entry : corpus) {
    SCOPED_TRACE(entry->name);
    EXPECT_FALSE(entry->certificate.records.empty());
    EXPECT_EQ(entry->certificate.pairs.size(), entry->checker->pairs().size());
    EXPECT_TRUE(entry->Verify(entry->certificate));
  }
}

GTEST_TEST(CertificateAuditTest, DesignedWorldHasTheIntendedPairStructure) {
  ASSERT_FALSE(Corpus().empty());
  const AuditCase& entry = *Corpus().front();
  ASSERT_TRUE(entry.designed);
  ASSERT_EQ(entry.certificate.pairs.size(), 2u)
      << "the designed world should present exactly the tool/plate and "
         "tool/ball pairs";
  const std::vector<int> counts = RecordsPerPair(entry);
  // The far pair certifies at the root: exactly one record, for the path's one
  // segment. The 12 mm pair needs Δ = w_x < 0.012 − 0.007 − τ ≈ 0.005 against
  // 0.6 m of travel, i.e. a node half-width of 0.3/2^d < 0.005 => d = 6, and a
  // constant clearance means every depth-6 node certifies it: 2^6 = 64 records.
  // Pinned exactly, so a regression that loosened or tightened the motion bound
  // by even one bisection level shows up here rather than hiding behind an
  // inequality.
  EXPECT_EQ(*std::min_element(counts.begin(), counts.end()), 1);
  EXPECT_EQ(*std::max_element(counts.begin(), counts.end()), 64);
  // Every pair's records must claim the same, correct threshold.
  for (const CertificateRecord& record : entry.certificate.records) {
    EXPECT_DOUBLE_EQ(record.threshold, kMargin + kEnvPadding);
  }
}

// ---------------------------------------------------------------------------
// 2. Mutations, each applied to every corpus case.
// ---------------------------------------------------------------------------

using Mutation = std::function<bool(const AuditCase&, Certificate*)>;

// Applies `mutate` to every corpus case and requires the result to be rejected.
// `mutate` returns false when the case cannot host the mutation.
void ExpectRejectedEverywhere(const std::string& what, const Mutation& mutate) {
  int applied = 0;
  for (const auto& entry : Corpus()) {
    SCOPED_TRACE(what + " on " + entry->name);
    Certificate certificate = entry->certificate;
    if (!mutate(*entry, &certificate)) continue;
    ++applied;
    EXPECT_FALSE(entry->Verify(certificate))
        << "VerifyCertificate accepted a certificate mutated by: " << what;
  }
  EXPECT_GT(applied, 0) << "the mutation '" << what
                        << "' was never applicable to any corpus case";
}

GTEST_TEST(CertificateAuditTest, RejectsTamperedClearance) {
  ExpectRejectedEverywhere("inflate phi_hat",
                           [](const AuditCase&, Certificate* certificate) {
                             if (certificate->records.empty()) return false;
                             certificate->records.front().phi_hat += 1.0;
                             return true;
                           });
  // ... and the mirror image: a record whose claimed clearance no longer
  // exceeds its own threshold proves nothing.
  ExpectRejectedEverywhere("shrink phi_hat to the threshold",
                           [](const AuditCase&, Certificate* certificate) {
                             if (certificate->records.empty()) return false;
                             certificate->records.front().phi_hat =
                                 certificate->records.front().threshold;
                             return true;
                           });
}

GTEST_TEST(CertificateAuditTest, RejectsWidenedInterval) {
  ExpectRejectedEverywhere(
      "widen a certified interval",
      [](const AuditCase& entry, Certificate* certificate) {
        const int index = MovingRecordIndex(entry);
        if (index < 0) return false;
        CertificateRecord& record = certificate->records[index];
        const double width = record.s_end - record.s_start;
        record.s_end = std::min(1.0, record.s_end + width);
        return record.s_end > entry.certificate.records[index].s_end;
      });
}

GTEST_TEST(CertificateAuditTest, RejectsShiftedRepresentativeConfiguration) {
  ExpectRejectedEverywhere(
      "shift qc off the trajectory",
      [](const AuditCase& entry, Certificate* certificate) {
        const int index = MovingRecordIndex(entry);
        if (index < 0) return false;
        certificate->records[index].qc[0] += 0.05;
        return true;
      });
}

GTEST_TEST(CertificateAuditTest, RejectsMissingRecords) {
  // The certifier's intervals tile the domain disjointly, so deleting any
  // record punches a coverage hole, even one whose own arithmetic was sound.
  ExpectRejectedEverywhere(
      "delete a record", [](const AuditCase&, Certificate* certificate) {
        if (certificate->records.size() < 2) return false;
        certificate->records.erase(certificate->records.begin());
        return true;
      });
  ExpectRejectedEverywhere(
      "truncate the record list",
      [](const AuditCase&, Certificate* certificate) {
        if (certificate->records.size() < 4) return false;
        certificate->records.resize(certificate->records.size() * 3 / 4);
        return true;
      });
}

GTEST_TEST(CertificateAuditTest, RejectsLoweredThreshold) {
  // Lower every record of one pair, so the replay's self-consistency check
  // ("all records of a pair claim the same threshold") passes and the mutation
  // has to be caught by the check that actually matters: the claimed threshold
  // must be at least the margin + padding the options call for.
  ExpectRejectedEverywhere(
      "lower one pair's threshold below margin + padding",
      [](const AuditCase&, Certificate* certificate) {
        if (certificate->records.empty()) return false;
        const int pair = certificate->records.front().pair_index;
        for (CertificateRecord& record : certificate->records) {
          if (record.pair_index == pair) record.threshold -= 0.003;
        }
        return true;
      });
  // Self-consistency is not enough either: a certificate whose records *all*
  // agree on a threshold nobody asked for proves a claim nobody asked for.
  ExpectRejectedEverywhere(
      "lower every threshold uniformly",
      [](const AuditCase&, Certificate* certificate) {
        if (certificate->records.empty()) return false;
        for (CertificateRecord& record : certificate->records) {
          record.threshold = -1e9;
        }
        return true;
      });
}

GTEST_TEST(CertificateAuditTest, RejectsPairTableMismatch) {
  ExpectRejectedEverywhere("drop a pair from the snapshot",
                           [](const AuditCase&, Certificate* certificate) {
                             if (certificate->pairs.size() < 2) return false;
                             certificate->pairs.pop_back();
                             return true;
                           });
  ExpectRejectedEverywhere("swap two entries of the pair snapshot",
                           [](const AuditCase&, Certificate* certificate) {
                             if (certificate->pairs.size() < 2) return false;
                             std::swap(certificate->pairs.front(),
                                       certificate->pairs.back());
                             return true;
                           });
}

GTEST_TEST(CertificateAuditTest, RejectsRelabelledPairRecords) {
  // Relabelling records between two pairs of similar difficulty can be a true
  // statement about a claim nobody made, so this mutation is only meaningful
  // where the pair structure is designed: give the 12 mm pair the far ball's
  // single root-wide record and its motion bound, half the 0.6 m travel, swamps
  // its 5 mm of slack.
  ASSERT_FALSE(Corpus().empty());
  const AuditCase& entry = *Corpus().front();
  ASSERT_TRUE(entry.designed);
  const std::vector<int> counts = RecordsPerPair(entry);
  const int hardest = static_cast<int>(
      std::max_element(counts.begin(), counts.end()) - counts.begin());
  const int easiest = static_cast<int>(
      std::min_element(counts.begin(), counts.end()) - counts.begin());
  ASSERT_NE(hardest, easiest);

  Certificate certificate = entry.certificate;
  for (CertificateRecord& record : certificate.records) {
    if (record.pair_index == hardest) {
      record.pair_index = easiest;
    } else if (record.pair_index == easiest) {
      record.pair_index = hardest;
    }
  }
  // Relabelling permutes two complete tilings, so coverage is not what catches
  // this. That is verified rather than assumed, because a mutation that
  // happened to break coverage would make the test pass for the wrong reason
  // and leave the arithmetic untested.
  for (int pair = 0; pair < static_cast<int>(certificate.pairs.size());
       ++pair) {
    EXPECT_TRUE(
        TilesEverySegment(certificate, pair, entry.path->segments().size()))
        << "pair " << pair
        << " lost its full tiling, so this mutation would "
           "have been caught by the coverage check instead of the arithmetic";
  }
  EXPECT_FALSE(entry.Verify(certificate));
}

GTEST_TEST(CertificateAuditTest, RejectsPerturbedPath) {
  // The certificate is a statement about one specific path. A path with a
  // nudged interior control point must not verify: every record's qc stops
  // being the midpoint apex of the interval it names.
  for (const auto& entry : Corpus()) {
    SCOPED_TRACE(entry->name);
    const PiecewiseBezierPath perturbed = entry->PerturbedPath(0.05);
    EXPECT_FALSE(
        VerifyCertificate(*entry->checker, perturbed, entry->certificate));
  }
}

GTEST_TEST(CertificateAuditTest, AcceptsReorderedRecords) {
  // Re-ordering is the one item on the classic mutation list that must NOT be
  // rejected: a permutation of a valid proof is still a valid proof. Pinning
  // this keeps a future "records must arrive sorted" shortcut from being
  // mistaken for a security property, and it rules out a verifier that rejects
  // everything, which would pass every mutation above.
  std::mt19937 rng(20260826);
  int shuffled = 0;
  for (const auto& entry : Corpus()) {
    SCOPED_TRACE(entry->name);
    Certificate certificate = entry->certificate;
    if (certificate.records.size() < 2) continue;
    ++shuffled;
    std::shuffle(certificate.records.begin(), certificate.records.end(), rng);
    EXPECT_TRUE(entry->Verify(certificate))
        << "a permutation of a valid certificate is still a valid certificate";
  }
  EXPECT_GE(shuffled, 3) << "every corpus case should have had a record list "
                            "long enough to permute";
}

// ---------------------------------------------------------------------------
// 3. Runs whose certificate is not a proof.
// ---------------------------------------------------------------------------

GTEST_TEST(CertificateAuditTest, NoCertificateUnlessRequested) {
  ASSERT_FALSE(Corpus().empty());
  const AuditCase& entry = *Corpus().front();
  Options options = AuditOptions();
  options.emit_certificate = false;
  const BezierCurve<double> trajectory(0.0, 1.0, entry.control_points);
  const CertificationResult result =
      entry.checker->CheckTrajectory(trajectory, options);
  ASSERT_EQ(result.verdict, Verdict::kCertifiedFree);
  EXPECT_FALSE(result.certificate.has_value());
}

GTEST_TEST(CertificateAuditTest, NonFreeVerdictCertificateIsNotAProof) {
  // The certificate field is present whenever emit_certificate was asked for,
  // and the records the run did make are individually valid. A run that found a
  // violation dropped that pair from the subtree instead of certifying it, so
  // the trail cannot cover the domain and the replay refuses it: usable as an
  // audit trail, unusable as a proof.
  ASSERT_FALSE(Corpus().empty());
  const AuditCase& entry = *Corpus().front();
  const Options options = AuditOptions();
  // The designed world driven straight through the 1 mm plate at y = 0.0175:
  // q(t) sweeps y from 0 to 0.05 while x crosses the plate's span.
  VectorXd start(2), end(2);
  start << -0.3, 0.0;
  end << 0.3, 0.05;
  const BezierCurve<double> trajectory(0.0, 1.0,
                                       CubicControlPoints(start, end));
  const PiecewiseBezierPath path =
      entry.checker->Normalize(trajectory, options);

  const CertificationResult violating =
      entry.checker->CheckTrajectory(trajectory, options);
  ASSERT_EQ(violating.verdict, Verdict::kViolationFound);
  ASSERT_TRUE(violating.certificate.has_value());
  EXPECT_FALSE(VerifyCertificate(*entry.checker, path, *violating.certificate))
      << "a certificate from a violating run must not read as a proof";

  // Same for a run stopped by the node budget. That needs the free trajectory:
  // a definite violation outranks budget exhaustion in the verdict reduction,
  // so the budget branch is only reachable when nothing violates.
  Options budgeted = options;
  budgeted.max_nodes = 3;
  const BezierCurve<double> free_trajectory(0.0, 1.0, entry.control_points);
  const CertificationResult truncated =
      entry.checker->CheckTrajectory(free_trajectory, budgeted);
  ASSERT_EQ(truncated.verdict, Verdict::kBudgetExhausted);
  ASSERT_TRUE(truncated.certificate.has_value());
  EXPECT_FALSE(entry.Verify(*truncated.certificate));

  // ... and for kFindFirstViolation, which additionally prunes the search:
  // every node starting after the witness is skipped, so whole stretches of the
  // domain are never visited at all.
  Options find_first = options;
  find_first.mode = SearchMode::kFindFirstViolation;
  const CertificationResult pruned =
      entry.checker->CheckTrajectory(trajectory, find_first);
  ASSERT_EQ(pruned.verdict, Verdict::kViolationFound);
  ASSERT_TRUE(pruned.certificate.has_value());
  EXPECT_FALSE(VerifyCertificate(*entry.checker, path, *pruned.certificate));
}

GTEST_TEST(CertificateAuditTest,
           FindFirstViolationOnAFreeTrajectoryStillCovers) {
  // The complement, pinned so the rule above is not mistaken for "the mode
  // invalidates certificates": with nothing to find, kFindFirstViolation has
  // nothing to prune against, explores the same tree as kCertifyAll, and its
  // trail is a complete proof.
  ASSERT_FALSE(Corpus().empty());
  const AuditCase& entry = *Corpus().front();
  Options options = AuditOptions();
  options.mode = SearchMode::kFindFirstViolation;
  const BezierCurve<double> trajectory(0.0, 1.0, entry.control_points);
  const CertificationResult result =
      entry.checker->CheckTrajectory(trajectory, options);
  ASSERT_EQ(result.verdict, Verdict::kCertifiedFree);
  ASSERT_TRUE(result.certificate.has_value());
  EXPECT_TRUE(entry.Verify(*result.certificate));
}

}  // namespace
}  // namespace continuous_collision
}  // namespace planning
}  // namespace drake
