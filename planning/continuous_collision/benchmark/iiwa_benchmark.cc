/// @file
/// The `continuous_collision` performance benchmark suite (the performance
/// targets and the benchmark deliverable of the white paper), adapted to what
/// exists on this machine: no trajectory optimizer is invoked, the smooth
/// composite Bézier trajectories are hand-constructed in
/// benchmark/scenario_worlds.cc, and every scenario's *true* swept clearance is
/// verified by dense sampling before it is benchmarked.
///
/// Scenarios
///   a) iiwa14 + bookcase, three tiers at ~2 mm / 1 cm / 5 cm swept clearance
///   b) a two-waypoint PWL edge in the same world
///   c) dual-arm iiwa handover (self-collision heavy)
///   d) the grazing pathological case (kInconclusive cost at the floor)
///   e) thread scaling over a 1000-check batch, two ways
///
/// Scenarios (a, 1 cm tier) and (b) are additionally compared against Drake's
/// own sampled `SceneGraphCollisionChecker` on the *same* RobotDiagram.
///
/// Usage: iiwa_benchmark [--out DIR] [--reps N] [--warmup N]
///                       [--dense-samples N] [--batch N] [--only NAME]
///                       [--drake_commit SHA]
///
/// `--out` defaults to the current directory, or to $TEST_TMPDIR when that is
/// set — the sandbox is the only writable directory under `bazel test`, and
/// the smoke-test rule in BUILD.bazel relies on this so it needs no --out of
/// its own. `--drake_commit` (the Drake revision this binary was built from,
/// "unknown" by default) is recorded verbatim in every result file so a JSON
/// result identifies the code it measured.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>

#include "drake/common/parallelism.h"
#include "drake/common/trajectories/bezier_curve.h"
#include "drake/geometry/query_object.h"
#include "drake/planning/collision_checker_params.h"
#include "drake/planning/continuous_collision/benchmark/benchmark_util.h"
#include "drake/planning/continuous_collision/benchmark/scenario_worlds.h"
#include "drake/planning/continuous_collision/continuous_collision_checker.h"
#include "drake/planning/scene_graph_collision_checker.h"

namespace drake {
namespace planning {
namespace continuous_collision {
namespace internal {
namespace {

using drake::Parallelism;
using drake::planning::CollisionCheckerParams;
using drake::planning::RobotDiagram;
using drake::planning::SceneGraphCollisionChecker;
using drake::trajectories::CompositeTrajectory;
using drake::trajectories::Trajectory;
using Eigen::MatrixXd;
using Eigen::VectorXd;

/// drake::planning::CollisionCheckerParams::edge_step_size has NO library
/// default: the field is value-initialized to 0 and the CollisionChecker
/// constructor rejects any non-positive value, so every caller must choose
/// one. 0.05 rad is the value that appears most often in Drake's own tests
/// and examples; the other common choices (0.125, 0.1, 0.01) are measured and
/// reported too, so the comparison cannot be accused of picking a flattering
/// resolution.
constexpr double kEdgeStepSize = 0.05;
constexpr double kReportedEdgeStepSizes[] = {0.125, 0.1, 0.05, 0.01};

/// Distances beyond this are irrelevant to every scenario here; the ground
/// truth sampler saturates at it.
constexpr double kMaxProbeDistance = 0.30;

struct Config {
  std::string out_dir = ".";
  std::string drake_commit = "unknown";
  int reps = 20;
  int warmup = 3;
  int dense_samples = 100000;
  int tune_samples = 3000;
  int tune_iterations = 24;
  int batch = 1000;
  int max_threads = 16;
  std::string only;
};

std::string VerdictName(Verdict v) {
  switch (v) {
    case Verdict::kCertifiedFree:
      return "kCertifiedFree";
    case Verdict::kViolationFound:
      return "kViolationFound";
    case Verdict::kInconclusive:
      return "kInconclusive";
    case Verdict::kBudgetExhausted:
      return "kBudgetExhausted";
  }
  return "unknown";
}

std::string ModeName(SearchMode m) {
  return m == SearchMode::kCertifyAll ? "kCertifyAll" : "kFindFirstViolation";
}

/// A world plus both checkers built on the *same* RobotDiagram. The sampled
/// checker is constructed first on purpose: its constructor pushes its
/// nominal filtered-collision matrix into the SceneGraph, so building our
/// checker afterwards guarantees the two see a bit-identical unfiltered pair
/// set. Anything else would make the comparison unfair in our favour.
struct World {
  std::shared_ptr<RobotDiagram<double>> diagram;
  std::unique_ptr<SceneGraphCollisionChecker> sampled;
  std::unique_ptr<ContinuousCollisionChecker> certified;
  std::unordered_set<drake::geometry::GeometryId> env_ids;
  int pair_count{0};
  /// SceneGraph's own unfiltered-candidate count *after* the sampled checker
  /// pushed its filters in. Equality with pair_count is the evidence that
  /// both checkers are looking at exactly the same pairs.
  int scene_graph_candidates{0};
};

World MakeWorld(std::shared_ptr<RobotDiagram<double>> diagram,
                const std::vector<std::string>& robot_instance_names) {
  World world;
  world.diagram = std::move(diagram);
  const auto& plant = world.diagram->plant();

  CollisionCheckerParams params;
  params.model = world.diagram;
  for (const std::string& name : robot_instance_names) {
    params.robot_model_instances.push_back(plant.GetModelInstanceByName(name));
  }
  params.edge_step_size = kEdgeStepSize;
  params.env_collision_padding = 0.0;
  params.self_collision_padding = 0.0;
  params.implicit_context_parallelism = Parallelism::None();
  world.sampled =
      std::make_unique<SceneGraphCollisionChecker>(std::move(params));

  ContinuousCollisionChecker::Params cparams;
  cparams.model = world.diagram;
  world.certified = std::make_unique<ContinuousCollisionChecker>(cparams);

  world.env_ids = CollectGeometryIds(*world.diagram, {"environment"});
  world.pair_count = static_cast<int>(world.certified->pairs().size());
  world.scene_graph_candidates = static_cast<int>(world.diagram->scene_graph()
                                                      .model_inspector()
                                                      .GetCollisionCandidates()
                                                      .size());
  return world;
}

Options MakeOptions(SearchMode mode, Parallelism parallelism,
                    double min_interval = 1e-9) {
  Options options;
  options.margin = 0.0;
  options.mode = mode;
  options.parallelism = parallelism;
  options.min_interval = min_interval;
  return options;
}

void WriteOptions(JsonWriter* json, const Options& options) {
  json->BeginObject("options");
  json->Write("margin", options.margin);
  json->Write("query_tolerance", options.query_tolerance);
  json->Write("certificate_slack", options.certificate_slack);
  json->Write("min_interval", options.min_interval);
  json->Write("mode", ModeName(options.mode));
  json->Write("max_reported_findings", options.max_reported_findings);
  json->Write("emit_certificate", options.emit_certificate);
  json->Write("parallelism", options.parallelism.num_threads());
  json->EndObject();
}

void WriteStats(JsonWriter* json, const Statistics& stats) {
  json->BeginObject("stats");
  json->Write("nodes", stats.nodes);
  json->Write("narrowphase_queries", stats.narrowphase_queries);
  json->Write("sphere_certifications", stats.sphere_certifications);
  json->Write("max_depth", stats.max_depth);
  json->EndObject();
}

void WriteClearance(JsonWriter* json, const ClearanceReport& clearance) {
  json->BeginObject("achieved_clearance");
  json->Write("min_all_pairs_m", clearance.min_all);
  json->Write("t_at_min_all", clearance.t_all);
  json->Write("min_robot_env_pairs_m", clearance.min_env);
  json->Write("t_at_min_env", clearance.t_env);
  json->Write("dense_samples", clearance.samples);
  json->Write("note",
              "ground truth from dense sampling plus golden-section "
              "refinement; the iiwa14 dense-sphere model has an intrinsic "
              "~25.2 mm self-clearance floor (link_0 vs link_2 spheres) that "
              "no shelf placement can raise, so min_all_pairs saturates "
              "there once the environment clearance exceeds it");
  json->EndObject();
}

/// One certification measurement.
struct CertRun {
  Verdict verdict{};
  Statistics stats;
  TimingSummary timing;
  int num_findings{0};
};

CertRun MeasureCertify(const ContinuousCollisionChecker& checker,
                       const Trajectory<double>& trajectory,
                       const Options& options, int warmup, int reps) {
  CertRun run;
  run.timing = TimeRepeatedly(warmup, reps, [&]() {
    const CertificationResult result =
        checker.CheckTrajectory(trajectory, options);
    run.verdict = result.verdict;
    run.stats = result.stats;
    run.num_findings = static_cast<int>(result.findings.size());
  });
  return run;
}

CertRun MeasureCertifyEdge(const ContinuousCollisionChecker& checker,
                           const VectorXd& q1, const VectorXd& q2,
                           const Options& options, int warmup, int reps) {
  CertRun run;
  run.timing = TimeRepeatedly(warmup, reps, [&]() {
    const CertificationResult result = checker.CheckEdge(q1, q2, options);
    run.verdict = result.verdict;
    run.stats = result.stats;
    run.num_findings = static_cast<int>(result.findings.size());
  });
  return run;
}

void WriteCertRun(JsonWriter* json, const std::string& key,
                  const CertRun& run) {
  json->BeginObject(key);
  json->Write("verdict", VerdictName(run.verdict));
  json->Write("num_findings", run.num_findings);
  WriteStats(json, run.stats);
  WriteTiming(json, "wall_ms", run.timing);
  json->EndObject();
}

// ---------------------------------------------------------------------------
// The sampled-checker comparison (the performance requirements, the headline
// number)
// ---------------------------------------------------------------------------

/// For a curved trajectory a practitioner checks it the only way a sampled
/// checker allows: walk the path and call CheckConfigCollisionFree at the
/// same resolution the checker would use for an edge, i.e. one sample per
/// `edge_step_size` of path length in the plant's edge metric. We report the
/// implied sample count and the wall time of exactly that sweep.
void MeasureSampledPathSweep(JsonWriter* json,
                             const SceneGraphCollisionChecker& sampled,
                             const Trajectory<double>& trajectory, int warmup,
                             int reps) {
  const double length = PathLengthInEdgeMetric(trajectory, 20001);
  json->BeginObject("sampled_comparison");
  json->Write("checker", "drake::planning::SceneGraphCollisionChecker");
  json->Write("edge_step_size_default_in_drake",
              "none - CollisionCheckerParams::edge_step_size is a required "
              "positive parameter with no library default");
  json->Write("edge_step_size", sampled.edge_step_size());
  json->Write("edge_metric",
              "unweighted Euclidean (LinearDistanceAndInterpolationProvider "
              "default weights = 1)");
  json->Write("path_length_edge_metric_rad", length);
  json->BeginArray("sweeps");
  for (const double step : kReportedEdgeStepSizes) {
    const int implied = static_cast<int>(std::ceil(length / step)) + 1;
    const std::vector<VectorXd> configs = SampleTrajectory(trajectory, implied);
    bool free = true;
    const TimingSummary timing = TimeRepeatedly(warmup, reps, [&]() {
      bool ok = true;
      for (const VectorXd& q : configs) {
        ok = sampled.CheckConfigCollisionFree(q) && ok;
      }
      free = ok;
    });
    json->BeginObject();
    json->Write("edge_step_size", step);
    json->Write("implied_samples", implied);
    json->Write("collision_free", free);
    WriteTiming(json, "sampled_wall_ms", timing);
    json->EndObject();
  }
  json->EndArray();
  json->EndObject();
}

void MeasureSampledEdge(JsonWriter* json,
                        const SceneGraphCollisionChecker& sampled,
                        const VectorXd& q1, const VectorXd& q2, int warmup,
                        int reps) {
  const double length = sampled.ComputeConfigurationDistance(q1, q2);
  json->BeginObject("sampled_comparison");
  json->Write("checker", "drake::planning::SceneGraphCollisionChecker");
  json->Write("edge_step_size_default_in_drake",
              "none - CollisionCheckerParams::edge_step_size is a required "
              "positive parameter with no library default");
  json->Write("edge_metric",
              "unweighted Euclidean (LinearDistanceAndInterpolationProvider "
              "default weights = 1)");
  json->Write("path_length_edge_metric_rad", length);
  json->BeginArray("sweeps");
  // A SceneGraphCollisionChecker is not copy-assignable, so vary the step
  // size on a mutable clone rather than rebuilding the model.
  std::unique_ptr<drake::planning::CollisionChecker> clone = sampled.Clone();
  for (const double step : kReportedEdgeStepSizes) {
    clone->set_edge_step_size(step);
    const int implied = static_cast<int>(std::ceil(length / step)) + 1;
    bool free = true;
    const TimingSummary timing = TimeRepeatedly(warmup, reps, [&]() {
      free = clone->CheckEdgeCollisionFree(q1, q2);
    });
    json->BeginObject();
    json->Write("edge_step_size", step);
    json->Write("implied_samples", implied);
    json->Write("collision_free", free);
    WriteTiming(json, "sampled_wall_ms", timing);
    json->EndObject();
  }
  json->EndArray();
  json->EndObject();
}

// ---------------------------------------------------------------------------
// Shared plumbing
// ---------------------------------------------------------------------------

/// Places the bookcase so the fixed trajectory's robot-vs-environment swept
/// clearance equals `target` (bisection on the shelf scale, which is monotone
/// non-decreasing over [0.010, 0.090]).
double TuneShelfScale(const Config& config, double target) {
  const MatrixXd waypoints = ShelfTrajectoryWaypoints();
  const auto trajectory =
      MakeQuinticCompositeBezier(waypoints, ShelfTrajectoryTimes());
  const auto clearance_of = [&](double scale) {
    const auto diagram = MakeShelfWorld(scale);
    const auto env_ids = CollectGeometryIds(*diagram, {"environment"});
    return MeasureSweptClearance(*diagram, *trajectory, env_ids,
                                 config.tune_samples, config.max_threads,
                                 kMaxProbeDistance)
        .min_env;
  };
  return BisectMonotone(clearance_of, 0.010, 0.090, target,
                        config.tune_iterations);
}

/// Repetition policy. Every millisecond-scale measurement gets the full
/// `--reps` after `--warmup` untimed runs. The grazing scenario at the
/// default 1e-9 resolution floor costs tens of seconds per call, where 20
/// repetitions would blow the suite's time budget for no statistical gain
/// (the relative spread of a 40 s measurement is far below that of a 2 ms
/// one), so expensive cases fall back to a small fixed count. The chosen
/// count is recorded in every timing block, so no result is silently
/// under-sampled.
void PlanReps(const Config& config, double single_run_ms, int* warmup,
              int* reps) {
  if (single_run_ms > 1000.0) {
    *warmup = 0;
    *reps = std::min(config.reps, 3);
  } else {
    *warmup = config.warmup;
    *reps = config.reps;
  }
}

/// Times one certification once, untimed, to price the case for PlanReps.
double ProbeCost(const ContinuousCollisionChecker& checker,
                 const Trajectory<double>& trajectory, const Options& options) {
  const auto t0 = std::chrono::steady_clock::now();
  checker.CheckTrajectory(trajectory, options);
  const auto t1 = std::chrono::steady_clock::now();
  return std::chrono::duration<double, std::milli>(t1 - t0).count();
}

void PrintHeader() {
  std::printf("  %-26s %-18s %8s %8s %10s %10s %7s\n", "case", "verdict",
              "med_ms", "min_ms", "nodes", "np_query", "depth");
}

void PrintRow(const std::string& label, const CertRun& run) {
  std::printf(
      "  %-26s %-18s %8.3f %8.3f %10llu %10llu %7d\n", label.c_str(),
      VerdictName(run.verdict).c_str(), run.timing.median_ms, run.timing.min_ms,
      static_cast<unsigned long long>(run.stats.nodes),  // NOLINT(runtime/int)
      static_cast<unsigned long long>(                   // NOLINT(runtime/int)
          run.stats.narrowphase_queries),
      run.stats.max_depth);
}

// ---------------------------------------------------------------------------
// (b) the PWL edge, run inside the 1 cm shelf world.
// ---------------------------------------------------------------------------

void RunPwlEdge(const Config& config, const MachineInfo& machine,
                const World& world, const MatrixXd& shelf_waypoints,
                double scale) {
  const VectorXd q1 = shelf_waypoints.col(0);
  const VectorXd q2 = shelf_waypoints.col(1);
  MatrixXd edge(q1.size(), 2);
  edge.col(0) = q1;
  edge.col(1) = q2;
  const auto edge_trajectory = MakeQuinticCompositeBezier(edge, {0.0, 1.0});
  const ClearanceReport clearance = MeasureSweptClearance(
      *world.diagram, *edge_trajectory, world.env_ids, config.dense_samples,
      config.max_threads, kMaxProbeDistance);
  const CertRun certify_all = MeasureCertifyEdge(
      *world.certified, q1, q2,
      MakeOptions(SearchMode::kCertifyAll, Parallelism::None()), config.warmup,
      config.reps);
  const CertRun find_first = MeasureCertifyEdge(
      *world.certified, q1, q2,
      MakeOptions(SearchMode::kFindFirstViolation, Parallelism::None()),
      config.warmup, config.reps);

  std::printf(
      "[b pwl edge] length=%.4f rad  clearance all=%.6f m "
      "env=%.6f m\n",
      (q2 - q1).norm(), clearance.min_all, clearance.min_env);
  PrintHeader();
  PrintRow("certify_all serial", certify_all);
  PrintRow("find_first serial", find_first);
  std::printf("\n");

  JsonWriter json;
  json.BeginObject();
  json.Write("scenario", "b_pwl_edge");
  // The certified object and the ground-truth object are not the same
  // trajectory, only the same point set: CheckEdge certifies a single order-1
  // Bezier segment, while the clearance written below is measured on the
  // quintic composite Bezier through the same two waypoints. With two
  // waypoints that quintic's endpoint velocities are zero, so its control
  // points collapse to {q1, q1, q1, q2, q2, q2} and it traces exactly the same
  // straight joint-space segment under a different time parametrization —
  // which is why the clearance it measures is the certified edge's clearance.
  json.Write("description",
             "two-waypoint PWL edge in the 1 cm shelf world, healthy "
             "clearance; certified as a single order-1 Bezier segment, with "
             "the clearance ground truth measured on a quintic composite "
             "Bezier tracing the same joint-space point set");
  json.Write("model", kIiwaUrl);
  json.Write("shelf_scale", scale);
  json.Write("pair_count", world.pair_count);
  json.Write("scene_graph_collision_candidates", world.scene_graph_candidates);
  json.Write("num_positions", world.diagram->plant().num_positions());
  json.Write("edge_length_rad", (q2 - q1).norm());
  WriteClearance(&json, clearance);
  WriteOptions(&json,
               MakeOptions(SearchMode::kCertifyAll, Parallelism::None()));
  json.Write("verdict", VerdictName(certify_all.verdict));
  WriteStats(&json, certify_all.stats);
  WriteTiming(&json, "wall_ms", certify_all.timing);
  WriteCertRun(&json, "certify_all_serial", certify_all);
  WriteCertRun(&json, "find_first_serial", find_first);
  MeasureSampledEdge(&json, *world.sampled, q1, q2, config.warmup, config.reps);
  WriteMachine(&json, machine);
  json.EndObject();
  WriteTextFile(config.out_dir + "/pwl_edge.json", json.str());
}

// ---------------------------------------------------------------------------
// (e) thread scaling.
// ---------------------------------------------------------------------------

void RunThreadScaling(const Config& config, const MachineInfo& machine,
                      const World& world, const MatrixXd& shelf_waypoints,
                      double scale) {
  std::printf(
      "[e threads] building a batch of %d certified-free "
      "trajectories ...\n",
      config.batch);
  std::mt19937 rng(20260826);
  std::uniform_real_distribution<double> jitter(-0.02, 0.02);
  std::vector<MatrixXd> candidates;
  const int max_candidates = 8 * config.batch;
  candidates.reserve(max_candidates);
  for (int i = 0; i < max_candidates; ++i) {
    MatrixXd w = shelf_waypoints;
    if (i > 0) {
      for (int c = 0; c < w.cols(); ++c) {
        for (int r = 0; r < w.rows(); ++r) w(r, c) += jitter(rng);
      }
    }
    candidates.push_back(w);
  }

  // Screen in parallel: only trajectories the checker *proves* free join the
  // batch, so the throughput numbers are all full certifications.
  std::vector<char> ok(candidates.size(), 0);
  {
    std::atomic<size_t> cursor{0};
    const auto screen = [&]() {
      const Options options =
          MakeOptions(SearchMode::kCertifyAll, Parallelism::None());
      for (;;) {
        const size_t i = cursor.fetch_add(1);
        if (i >= candidates.size()) return;
        const auto traj =
            MakeQuinticCompositeBezier(candidates[i], ShelfTrajectoryTimes());
        ok[i] = world.certified->CheckTrajectory(*traj, options).verdict ==
                Verdict::kCertifiedFree;
      }
    };
    std::vector<std::thread> pool;
    pool.reserve(config.max_threads);
    for (int t = 0; t < config.max_threads; ++t) pool.emplace_back(screen);
    for (auto& th : pool) th.join();
  }

  std::vector<std::shared_ptr<CompositeTrajectory<double>>> accepted;
  int screened = 0;
  for (size_t i = 0; i < candidates.size() &&
                     static_cast<int>(accepted.size()) < config.batch;
       ++i) {
    ++screened;
    if (!ok[i]) continue;
    accepted.push_back(
        MakeQuinticCompositeBezier(candidates[i], ShelfTrajectoryTimes()));
  }
  const double acceptance =
      screened > 0
          ? static_cast<double>(accepted.size()) / static_cast<double>(screened)
          : 0.0;
  std::printf("[e threads] accepted %zu of %d screened (%.1f%%)\n",
              accepted.size(), screened, 100.0 * acceptance);

  // Independent re-verification of a sample of the accepted batch: the
  // certificate says free, dense sampling must agree.
  double verify_min = kMaxProbeDistance;
  const int verify_count = std::min<int>(16, static_cast<int>(accepted.size()));
  for (int i = 0; i < verify_count; ++i) {
    const size_t index = static_cast<size_t>(i) * accepted.size() /
                         static_cast<size_t>(verify_count);
    const ClearanceReport r =
        MeasureSweptClearance(*world.diagram, *accepted[index], world.env_ids,
                              20000, config.max_threads, kMaxProbeDistance);
    verify_min = std::min(verify_min, r.min_all);
  }
  std::printf(
      "[e threads] re-verified %d sampled members; worst dense "
      "clearance %.6f m\n",
      verify_count, verify_min);

  JsonWriter json;
  json.BeginObject();
  json.Write("scenario", "e_thread_scaling");
  json.Write("description",
             "batch of certification calls on the 1 cm tier trajectory and "
             "jittered variants (+/-0.02 rad on every waypoint coordinate) "
             "that remain certified free");
  json.Write("model", kIiwaUrl);
  json.Write("shelf_scale", scale);
  json.Write("pair_count", world.pair_count);
  json.Write("scene_graph_collision_candidates", world.scene_graph_candidates);
  json.Write("batch_size", static_cast<int>(accepted.size()));
  json.Write("candidates_screened", screened);
  json.Write("acceptance_rate", acceptance);
  json.Write("reverified_members", verify_count);
  json.Write("reverified_worst_clearance_m", verify_min);
  WriteOptions(&json,
               MakeOptions(SearchMode::kCertifyAll, Parallelism::None()));
  WriteMachine(&json, machine);

  const int thread_counts[] = {1, 8, 16};
  json.BeginArray("thread_scaling");
  double baseline_per_call = 0.0;
  for (const int p : thread_counts) {
    const Options options =
        MakeOptions(SearchMode::kCertifyAll, Parallelism(p));
    const auto t0 = std::chrono::steady_clock::now();
    for (const auto& traj : accepted) {
      world.certified->CheckTrajectory(*traj, options);
    }
    const auto t1 = std::chrono::steady_clock::now();
    const double seconds = std::chrono::duration<double>(t1 - t0).count();
    const double throughput = static_cast<double>(accepted.size()) / seconds;
    if (p == 1) baseline_per_call = throughput;
    json.BeginObject();
    json.Write("mode", "per_call_parallelism");
    json.Write("threads", p);
    json.Write("wall_s", seconds);
    json.Write("checks_per_s", throughput);
    json.Write("speedup", throughput / baseline_per_call);
    json.EndObject();
    std::printf(
        "[e threads] per-call p=%2d  %8.3f s  %9.1f checks/s  "
        "%5.2fx\n",
        p, seconds, throughput, throughput / baseline_per_call);
  }
  double baseline_caller = 0.0;
  for (const int t : thread_counts) {
    const Options options =
        MakeOptions(SearchMode::kCertifyAll, Parallelism::None());
    std::atomic<size_t> cursor{0};
    const auto worker = [&]() {
      for (;;) {
        const size_t i = cursor.fetch_add(1);
        if (i >= accepted.size()) return;
        world.certified->CheckTrajectory(*accepted[i], options);
      }
    };
    const auto s0 = std::chrono::steady_clock::now();
    std::vector<std::thread> pool;
    pool.reserve(t);
    for (int k = 0; k < t; ++k) pool.emplace_back(worker);
    for (auto& th : pool) th.join();
    const auto s1 = std::chrono::steady_clock::now();
    const double seconds = std::chrono::duration<double>(s1 - s0).count();
    const double throughput = static_cast<double>(accepted.size()) / seconds;
    if (t == 1) baseline_caller = throughput;
    json.BeginObject();
    json.Write("mode", "caller_threads_serial_checks");
    json.Write("threads", t);
    json.Write("wall_s", seconds);
    json.Write("checks_per_s", throughput);
    json.Write("speedup", throughput / baseline_caller);
    json.EndObject();
    std::printf(
        "[e threads] caller  t=%2d  %8.3f s  %9.1f checks/s  "
        "%5.2fx\n",
        t, seconds, throughput, throughput / baseline_caller);
  }
  json.EndArray();
  json.EndObject();
  WriteTextFile(config.out_dir + "/thread_scaling.json", json.str());
  std::printf("\n");
}

// ---------------------------------------------------------------------------
// (c) dual-arm handover.
// ---------------------------------------------------------------------------

void RunDualArm(const Config& config, const MachineInfo& machine) {
  constexpr double kBaseSeparation = 1.00;
  const MatrixXd waypoints = DualArmTrajectoryWaypoints();
  const auto trajectory =
      MakeQuinticCompositeBezier(waypoints, DualArmTrajectoryTimes());
  World world =
      MakeWorld(MakeDualArmWorld(kBaseSeparation), {"iiwa14", "iiwa14_1"});
  const ClearanceReport clearance = MeasureSweptClearance(
      *world.diagram, *trajectory, world.env_ids, config.dense_samples,
      config.max_threads, kMaxProbeDistance);
  std::printf(
      "[c dual arm] separation=%.3f m  clearance all=%.6f m "
      "env=%.6f m  pairs=%d\n",
      kBaseSeparation, clearance.min_all, clearance.min_env, world.pair_count);

  const CertRun serial_all =
      MeasureCertify(*world.certified, *trajectory,
                     MakeOptions(SearchMode::kCertifyAll, Parallelism::None()),
                     config.warmup, config.reps);
  const CertRun par16 =
      MeasureCertify(*world.certified, *trajectory,
                     MakeOptions(SearchMode::kCertifyAll, Parallelism(16)),
                     config.warmup, config.reps);
  PrintHeader();
  PrintRow("certify_all serial", serial_all);
  PrintRow("certify_all 16 threads", par16);
  std::printf("\n");

  JsonWriter json;
  json.BeginObject();
  json.Write("scenario", "c_dual_arm_handover");
  json.Write("description",
             "two iiwa14 dense-sphere arms welded 1.00 m apart facing each "
             "other; 4-segment quintic composite Bezier bringing the "
             "end-effectors past each other and back");
  json.Write("model", kIiwaUrl);
  json.Write("base_separation_m", kBaseSeparation);
  json.Write("pair_count", world.pair_count);
  json.Write("scene_graph_collision_candidates", world.scene_graph_candidates);
  json.Write("num_positions", world.diagram->plant().num_positions());
  json.Write("trajectory_segments", static_cast<int>(waypoints.cols()) - 1);
  json.Write("trajectory_degree", 5);
  WriteClearance(&json, clearance);
  WriteOptions(&json,
               MakeOptions(SearchMode::kCertifyAll, Parallelism::None()));
  json.Write("verdict", VerdictName(serial_all.verdict));
  WriteStats(&json, serial_all.stats);
  WriteTiming(&json, "wall_ms", serial_all.timing);
  WriteCertRun(&json, "certify_all_serial", serial_all);
  WriteCertRun(&json, "certify_all_16_threads", par16);
  WriteMachine(&json, machine);
  json.EndObject();
  WriteTextFile(config.out_dir + "/dual_arm.json", json.str());
}

// ---------------------------------------------------------------------------
// (d) grazing.
// ---------------------------------------------------------------------------

void RunGrazing(const Config& config, const MachineInfo& machine,
                const Trajectory<double>& shelf_trajectory) {
  std::printf("[d grazing] tuning shelf placement for zero clearance ...\n");
  const double scale = TuneShelfScale(config, 0.0);
  World world = MakeWorld(MakeShelfWorld(scale), {"iiwa14"});
  const ClearanceReport clearance = MeasureSweptClearance(
      *world.diagram, shelf_trajectory, world.env_ids, config.dense_samples,
      config.max_threads, kMaxProbeDistance);
  std::printf(
      "[d grazing] shelf_scale=%.6f  clearance env=%.9f m "
      "all=%.9f m\n",
      scale, clearance.min_env, clearance.min_all);

  JsonWriter json;
  json.BeginObject();
  json.Write("scenario", "d_grazing");
  json.Write("description",
             "the same shelf world placed so the trajectory's swept "
             "clearance sits within the oracle tolerance of zero: the "
             "conservative certifier must refine to the resolution floor and "
             "report kInconclusive rather than a certificate");
  json.Write("model", kIiwaUrl);
  json.Write("shelf_scale", scale);
  json.Write("pair_count", world.pair_count);
  json.Write("scene_graph_collision_candidates", world.scene_graph_candidates);
  WriteClearance(&json, clearance);

  // The resolution floor is the knob that prices the pathological case: cost
  // at the floor grows like log2(1 / min_interval) (the soundness argument's
  // termination proof).
  constexpr double kFloors[] = {1e-9, 1e-6, 1e-4, 1e-2};
  CertRun default_run;
  json.BeginArray("min_interval_sweep");
  PrintHeader();
  for (const double floor : kFloors) {
    const Options options =
        MakeOptions(SearchMode::kCertifyAll, Parallelism::None(), floor);
    int warmup = 0;
    int reps = 0;
    PlanReps(config, ProbeCost(*world.certified, shelf_trajectory, options),
             &warmup, &reps);
    const CertRun run = MeasureCertify(*world.certified, shelf_trajectory,
                                       options, warmup, reps);
    if (floor == 1e-9) default_run = run;
    json.BeginObject();
    json.Write("min_interval", floor);
    json.Write("verdict", VerdictName(run.verdict));
    json.Write("num_findings", run.num_findings);
    WriteStats(&json, run.stats);
    WriteTiming(&json, "wall_ms", run.timing);
    json.EndObject();
    char label[64];
    std::snprintf(label, sizeof(label), "min_interval=%g", floor);
    PrintRow(label, run);
  }
  json.EndArray();
  // kFindFirstViolation at the same floor: with no definite violation
  // anywhere on the trajectory the earliest-witness bound never prunes, so
  // this is expected to cost the same as kCertifyAll — measured, not assumed.
  const Options first_options =
      MakeOptions(SearchMode::kFindFirstViolation, Parallelism::None());
  int first_warmup = 0;
  int first_reps = 0;
  PlanReps(config, ProbeCost(*world.certified, shelf_trajectory, first_options),
           &first_warmup, &first_reps);
  const CertRun find_first =
      MeasureCertify(*world.certified, shelf_trajectory, first_options,
                     first_warmup, first_reps);
  PrintRow("find_first serial", find_first);
  std::printf("\n");

  WriteOptions(&json,
               MakeOptions(SearchMode::kCertifyAll, Parallelism::None()));
  json.Write("verdict", VerdictName(default_run.verdict));
  WriteStats(&json, default_run.stats);
  WriteTiming(&json, "wall_ms", default_run.timing);
  WriteCertRun(&json, "certify_all_serial", default_run);
  WriteCertRun(&json, "find_first_serial", find_first);
  WriteMachine(&json, machine);
  json.EndObject();
  WriteTextFile(config.out_dir + "/grazing.json", json.str());
}

// ---------------------------------------------------------------------------
// (f) performance review: where the time goes, and why per-call parallelism
// saturates. Not a standard scenario — this exists to back the gap analysis
// in the benchmark write-up with measurements rather than assertions.
// ---------------------------------------------------------------------------

void RunProfile(const Config& config, const MachineInfo& machine,
                const Trajectory<double>& shelf_trajectory) {
  std::printf("[f profile] rebuilding the 1 cm world ...\n");
  const double scale = TuneShelfScale(config, 0.010);
  World world = MakeWorld(MakeShelfWorld(scale), {"iiwa14"});

  JsonWriter json;
  json.BeginObject();
  json.Write("scenario", "f_profile");
  json.Write("description",
             "cost attribution and parallel-granularity probe backing the "
             "gap analysis in the benchmark write-up; not one of the standard "
             "scenarios");
  json.Write("model", kIiwaUrl);
  json.Write("shelf_scale", scale);
  json.Write("pair_count", world.pair_count);
  json.Write("scene_graph_collision_candidates", world.scene_graph_candidates);

  // --- Cost attribution -----------------------------------------------------
  // Two microbenchmarks over the same inner loop isolate the marginal cost of
  // a narrowphase query from the fixed per-configuration cost (SetPositions
  // plus the pose/broadphase update the first query forces).
  constexpr int kInner = 2000;
  constexpr int kManyQueries = 27;  // ~ the observed queries per node
  const auto& oracle = world.certified->distance_oracle();
  const auto& pairs = world.certified->pairs();
  const auto& plant = world.diagram->plant();
  const auto& scene_graph = world.diagram->scene_graph();
  auto root = world.diagram->CreateDefaultContext();
  const std::vector<VectorXd> configs =
      SampleTrajectory(shelf_trajectory, kInner);
  // Accumulator so the optimizer cannot discard the timed queries.
  double sink = 0.0;
  const auto sweep = [&](int queries_per_config) {
    return TimeRepeatedly(config.warmup, config.reps, [&]() {
      for (int i = 0; i < kInner; ++i) {
        plant.SetPositions(&world.diagram->mutable_plant_context(root.get()),
                           configs[i]);
        const auto& query_object =
            scene_graph.get_query_output_port()
                .Eval<drake::geometry::QueryObject<double>>(
                    world.diagram->scene_graph_context(*root));
        for (int k = 0; k < queries_per_config; ++k) {
          sink += oracle.SignedDistance(
              query_object,
              pairs[static_cast<size_t>(i * kManyQueries + k) % pairs.size()]);
        }
      }
    });
  };
  const TimingSummary one = sweep(1);
  const TimingSummary many = sweep(kManyQueries);
  const double us_per_query =
      1000.0 * (many.median_ms - one.median_ms) / (kInner * (kManyQueries - 1));
  const double us_per_config = 1000.0 * one.median_ms / kInner - us_per_query;

  const CertRun reference =
      MeasureCertify(*world.certified, shelf_trajectory,
                     MakeOptions(SearchMode::kCertifyAll, Parallelism::None()),
                     config.warmup, config.reps);
  const double predicted_ms =
      (static_cast<double>(reference.stats.nodes) * us_per_config +
       static_cast<double>(reference.stats.narrowphase_queries) *
           us_per_query) /
      1000.0;

  json.BeginObject("cost_attribution");
  json.Write("us_per_configuration_fk_and_pose_update", us_per_config);
  json.Write("us_per_narrowphase_query", us_per_query);
  json.Write("nodes", reference.stats.nodes);
  json.Write("narrowphase_queries", reference.stats.narrowphase_queries);
  json.Write("sphere_certifications", reference.stats.sphere_certifications);
  json.Write("predicted_ms", predicted_ms);
  json.Write("measured_ms", reference.timing.median_ms);
  json.Write("residual_ms", reference.timing.median_ms - predicted_ms);
  json.Write("residual_note",
             "residual covers the sphere prefilter, the lambda/Delta sparse "
             "dot products, de Casteljau splitting and driver bookkeeping");
  json.Write("summed_distances_m", sink);
  json.EndObject();
  std::printf(
      "[f profile] %.3f us / configuration, %.3f us / narrowphase "
      "query\n",
      us_per_config, us_per_query);
  std::printf(
      "[f profile] predicted %.3f ms vs measured %.3f ms "
      "(residual %.3f ms)\n",
      predicted_ms, reference.timing.median_ms,
      reference.timing.median_ms - predicted_ms);

  // --- Per-segment work distribution ---------------------------------------
  // This measures the ceiling that *segment-root seeding* imposes: if the
  // parallel driver's only work units are whole segments, the best per-call
  // speedup a 6-segment trajectory can reach is total work / heaviest segment.
  // Certifying each segment on its own measures it directly. The driver no
  // longer works that way — it shares sub-segment nodes on demand (see
  // certifier_internal.h) — so this row is now a *reference* bound
  // that the measured per-call speedup is allowed to exceed, and the record of
  // why the old driver could not.
  {
    const PiecewiseBezierPath path = world.certified->Normalize(
        shelf_trajectory,
        MakeOptions(SearchMode::kCertifyAll, Parallelism::None()));
    uint64_t total = 0;
    uint64_t heaviest = 0;
    double heaviest_ms = 0.0;
    double serial_sum_ms = 0.0;
    json.BeginArray("per_segment_work");
    for (size_t i = 0; i < path.segments().size(); ++i) {
      const auto& segment = path.segments()[i];
      const drake::trajectories::BezierCurve<double> curve(
          segment.t_start, segment.t_end, segment.control_points);
      const CertRun run = MeasureCertify(
          *world.certified, curve,
          MakeOptions(SearchMode::kCertifyAll, Parallelism::None()),
          config.warmup, config.reps);
      total += run.stats.nodes;
      heaviest = std::max(heaviest, run.stats.nodes);
      heaviest_ms = std::max(heaviest_ms, run.timing.median_ms);
      serial_sum_ms += run.timing.median_ms;
      json.BeginObject();
      json.Write("segment", static_cast<int>(i));
      json.Write("nodes", run.stats.nodes);
      json.Write("narrowphase_queries", run.stats.narrowphase_queries);
      WriteTiming(&json, "wall_ms", run.timing);
      json.EndObject();
    }
    json.EndArray();
    json.BeginObject("per_segment_summary");
    json.Write("total_nodes_over_segments", total);
    json.Write("heaviest_segment_nodes", heaviest);
    json.Write("segment_seeding_bound_on_per_call_speedup",
               serial_sum_ms / std::max(heaviest_ms, 1e-9));
    json.Write("note",
               "each segment certified on its own; the sum exceeds the "
               "whole-trajectory node count only by the per-segment "
               "breakpoint work. segment_seeding_bound is the ceiling a "
               "driver seeded with whole segments would hit; the current "
               "driver shares sub-segment nodes on demand and is not bound "
               "by it");
    json.EndObject();
    std::printf(
        "[f profile] heaviest segment = %llu of %llu nodes; "
        "segment-seeding bound on per-call speedup = %.2fx\n",
        static_cast<unsigned long long>(heaviest),  // NOLINT(runtime/int)
        static_cast<unsigned long long>(total),     // NOLINT(runtime/int)
        serial_sum_ms / std::max(heaviest_ms, 1e-9));
  }

  // --- Parallel granularity -------------------------------------------------
  // Per-call parallelism is measured on three workloads spanning three orders
  // of magnitude in node count, holding everything else fixed. The three sit
  // on either side of the driver's lazy-recruitment threshold on purpose: the
  // 15-node edge is below it (and must therefore be exactly serial at every p)
  // while the other two are above it.
  std::printf(
      "[f profile] tuning the grazing world for the long "
      "workload ...\n");
  const double graze_scale = TuneShelfScale(config, 0.0);
  World graze = MakeWorld(MakeShelfWorld(graze_scale), {"iiwa14"});
  const MatrixXd shelf_waypoints = ShelfTrajectoryWaypoints();
  const VectorXd q1 = shelf_waypoints.col(0);
  const VectorXd q2 = shelf_waypoints.col(1);

  constexpr int kParallelism[] = {1, 2, 4, 8, 16};
  json.BeginArray("parallel_granularity");
  std::printf("  %-18s %5s %10s %10s %8s\n", "workload", "p", "nodes", "med_ms",
              "speedup");
  for (const char* which : {"pwl_edge_15_nodes", "shelf_1cm_146_nodes",
                            "grazing_min_interval_1e-6"}) {
    double baseline = 0.0;
    for (const int p : kParallelism) {
      CertRun run;
      if (std::strcmp(which, "pwl_edge_15_nodes") == 0) {
        run = MeasureCertifyEdge(
            *world.certified, q1, q2,
            MakeOptions(SearchMode::kCertifyAll, Parallelism(p)), config.warmup,
            config.reps);
      } else if (std::strcmp(which, "shelf_1cm_146_nodes") == 0) {
        run =
            MeasureCertify(*world.certified, shelf_trajectory,
                           MakeOptions(SearchMode::kCertifyAll, Parallelism(p)),
                           config.warmup, config.reps);
      } else {
        run = MeasureCertify(
            *graze.certified, shelf_trajectory,
            MakeOptions(SearchMode::kCertifyAll, Parallelism(p), 1e-6),
            config.warmup, config.reps);
      }
      if (p == 1) baseline = run.timing.median_ms;
      json.BeginObject();
      json.Write("workload", which);
      json.Write("threads", p);
      json.Write("nodes", run.stats.nodes);
      json.Write("verdict", VerdictName(run.verdict));
      WriteTiming(&json, "wall_ms", run.timing);
      json.Write("speedup", baseline / run.timing.median_ms);
      json.EndObject();
      std::printf("  %-18s %5d %10llu %10.3f %8.2f\n", which, p,
                  static_cast<unsigned long long>(  // NOLINT(runtime/int)
                      run.stats.nodes),
                  run.timing.median_ms, baseline / run.timing.median_ms);
    }
  }
  json.EndArray();
  WriteMachine(&json, machine);
  json.EndObject();
  WriteTextFile(config.out_dir + "/profile.json", json.str());
  std::printf("\n");
}

int Main(int argc, char** argv) {
  Config config;
  if (const char* const test_tmpdir = std::getenv("TEST_TMPDIR")) {
    config.out_dir = test_tmpdir;
  }
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const auto next = [&]() -> std::string {
      if (i + 1 >= argc) throw std::runtime_error("missing value for " + arg);
      return argv[++i];
    };
    if (arg == "--out") {
      config.out_dir = next();
    } else if (arg == "--reps") {
      config.reps = std::stoi(next());
    } else if (arg == "--warmup") {
      config.warmup = std::stoi(next());
    } else if (arg == "--dense-samples") {
      config.dense_samples = std::stoi(next());
    } else if (arg == "--tune-samples") {
      config.tune_samples = std::stoi(next());
    } else if (arg == "--batch") {
      config.batch = std::stoi(next());
    } else if (arg == "--only") {
      config.only = next();
    } else if (arg == "--drake_commit") {
      config.drake_commit = next();
    } else {
      std::fprintf(stderr, "unknown argument: %s\n", arg.c_str());
      return 1;
    }
  }
  const auto wanted = [&](const std::string& name) {
    return config.only.empty() || config.only == name;
  };

  const MachineInfo machine = GetMachineInfo(config.drake_commit);
  std::printf("continuous_collision benchmark suite\n");
  std::printf("  cpu       : %s (%d logical cores)\n",
              machine.cpu_model.c_str(), machine.core_count);
  std::printf("  drake pin : %s (%s)\n", machine.drake_commit.c_str(),
              machine.drake_version_note.c_str());
  std::printf("  model     : %s\n", kIiwaUrl);
  std::printf("  reps      : %d timed after %d warmup\n", config.reps,
              config.warmup);
  std::printf("  output    : %s\n\n", config.out_dir.c_str());

  const MatrixXd shelf_waypoints = ShelfTrajectoryWaypoints();
  const auto shelf_trajectory =
      MakeQuinticCompositeBezier(shelf_waypoints, ShelfTrajectoryTimes());

  struct Tier {
    const char* name;
    const char* file;
    double target;
    bool headline;
  };
  constexpr Tier kTiers[] = {
      {"a shelf 2mm", "shelf_2mm", 0.002, false},
      {"a shelf 1cm", "shelf_1cm", 0.010, true},
      {"a shelf 5cm", "shelf_5cm", 0.050, false},
  };

  for (const Tier& tier : kTiers) {
    const bool need_headline_world =
        tier.headline && (wanted("pwl") || wanted("threads"));
    if (!wanted("shelf") && !need_headline_world) continue;

    std::printf("[%s] tuning shelf placement for %.0f mm ...\n", tier.name,
                1000.0 * tier.target);
    const double scale = TuneShelfScale(config, tier.target);
    World world = MakeWorld(MakeShelfWorld(scale), {"iiwa14"});
    const ClearanceReport clearance = MeasureSweptClearance(
        *world.diagram, *shelf_trajectory, world.env_ids, config.dense_samples,
        config.max_threads, kMaxProbeDistance);
    std::printf(
        "[%s] shelf_scale=%.6f  clearance env=%.6f m all=%.6f m  "
        "pairs=%d\n",
        tier.name, scale, clearance.min_env, clearance.min_all,
        world.pair_count);

    if (wanted("shelf")) {
      const CertRun serial_all = MeasureCertify(
          *world.certified, *shelf_trajectory,
          MakeOptions(SearchMode::kCertifyAll, Parallelism::None()),
          config.warmup, config.reps);
      const CertRun serial_first = MeasureCertify(
          *world.certified, *shelf_trajectory,
          MakeOptions(SearchMode::kFindFirstViolation, Parallelism::None()),
          config.warmup, config.reps);
      const CertRun par8 =
          MeasureCertify(*world.certified, *shelf_trajectory,
                         MakeOptions(SearchMode::kCertifyAll, Parallelism(8)),
                         config.warmup, config.reps);
      const CertRun par16 =
          MeasureCertify(*world.certified, *shelf_trajectory,
                         MakeOptions(SearchMode::kCertifyAll, Parallelism(16)),
                         config.warmup, config.reps);

      PrintHeader();
      PrintRow("certify_all serial", serial_all);
      PrintRow("find_first serial", serial_first);
      PrintRow("certify_all 8 threads", par8);
      PrintRow("certify_all 16 threads", par16);
      std::printf("\n");

      JsonWriter json;
      json.BeginObject();
      json.Write("scenario", std::string("a_") + tier.file);
      json.Write("description",
                 "iiwa14 (dense-sphere collision model) welded to the world, "
                 "seven-box bookcase plus a table slab; 6-segment quintic "
                 "composite Bezier reaching into the shelf bay and back");
      json.Write("model", kIiwaUrl);
      json.Write("shelf_scale", scale);
      json.Write("target_clearance_m", tier.target);
      json.Write("pair_count", world.pair_count);
      json.Write("scene_graph_collision_candidates",
                 world.scene_graph_candidates);
      json.Write("num_positions", world.diagram->plant().num_positions());
      json.Write("trajectory_segments",
                 static_cast<int>(shelf_waypoints.cols()) - 1);
      json.Write("trajectory_degree", 5);
      WriteClearance(&json, clearance);
      WriteOptions(&json,
                   MakeOptions(SearchMode::kCertifyAll, Parallelism::None()));
      json.Write("verdict", VerdictName(serial_all.verdict));
      WriteStats(&json, serial_all.stats);
      WriteTiming(&json, "wall_ms", serial_all.timing);
      WriteCertRun(&json, "certify_all_serial", serial_all);
      WriteCertRun(&json, "find_first_serial", serial_first);
      WriteCertRun(&json, "certify_all_8_threads", par8);
      WriteCertRun(&json, "certify_all_16_threads", par16);
      if (tier.headline) {
        MeasureSampledPathSweep(&json, *world.sampled, *shelf_trajectory,
                                config.warmup, config.reps);
      }
      WriteMachine(&json, machine);
      json.EndObject();
      WriteTextFile(config.out_dir + "/" + tier.file + ".json", json.str());
    }

    if (tier.headline && wanted("pwl")) {
      RunPwlEdge(config, machine, world, shelf_waypoints, scale);
    }
    if (tier.headline && wanted("threads")) {
      RunThreadScaling(config, machine, world, shelf_waypoints, scale);
    }
  }

  if (wanted("dual")) RunDualArm(config, machine);
  if (wanted("grazing")) RunGrazing(config, machine, *shelf_trajectory);
  if (wanted("profile")) RunProfile(config, machine, *shelf_trajectory);

  std::printf("done; results in %s\n", config.out_dir.c_str());
  return 0;
}

}  // namespace
}  // namespace internal
}  // namespace continuous_collision
}  // namespace planning
}  // namespace drake

int main(int argc, char** argv) {
  try {
    return drake::planning::continuous_collision::internal::Main(argc, argv);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "benchmark failed: %s\n", e.what());
    return 1;
  }
}
