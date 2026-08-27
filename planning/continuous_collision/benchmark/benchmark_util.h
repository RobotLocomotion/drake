#pragma once

/// @file
/// Small, deliberately boring helpers shared by the benchmark scenarios
/// (the benchmark suite): a hand-rolled JSON writer, steady_clock timing with
/// medians, machine identification, quintic composite-Bézier construction, and
/// a dense ground-truth swept-clearance sampler used to *verify* — never to
/// certify — the clearance of every scenario trajectory.
///
/// No third-party benchmark framework is used on purpose: the measurements
/// here are milliseconds-scale wall clock repeated by hand, and the JSON is
/// consumed by the white-paper author and by CI tracking.

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/trajectories/composite_trajectory.h"
#include "drake/common/trajectories/trajectory.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/planning/robot_diagram.h"

namespace drake {
namespace planning {
namespace continuous_collision {
namespace benchmark {

// ---------------------------------------------------------------------------
// JSON
// ---------------------------------------------------------------------------

/// Minimal streaming JSON writer: enough for the fixed result schema, with no
/// dependency and no cleverness. Callers must balance Begin*/End* calls.
class JsonWriter {
 public:
  JsonWriter() = default;

  void BeginObject();
  void BeginObject(const std::string& key);
  void EndObject();
  void BeginArray(const std::string& key);
  void EndArray();

  void Write(const std::string& key, double value);
  void Write(const std::string& key, int value);
  void Write(const std::string& key, long value);           // NOLINT
  void Write(const std::string& key, unsigned long value);  // NOLINT
  void Write(const std::string& key, bool value);
  void Write(const std::string& key, const char* value);
  void Write(const std::string& key, const std::string& value);
  /// Appends a bare double to the innermost array.
  void WriteArrayValue(double value);
  void WriteArrayValue(const std::string& value);

  std::string str() const { return out_ + "\n"; }

 private:
  void Separator();
  void Indent();

  std::string out_;
  std::vector<bool> first_;  // per open container: "nothing written yet"
  int depth_{0};
};

/// Writes `text` to `path`, creating parent directories as needed.
void WriteTextFile(const std::string& path, const std::string& text);

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------

struct TimingSummary {
  double median_ms{0.0};
  double min_ms{0.0};
  double max_ms{0.0};
  int reps{0};
};

/// Runs `body` `warmup` times untimed, then `reps` times timed, and reduces
/// the sample to median/min/max. No pinning, no frequency control: the numbers
/// are what a user on this machine would see (the benchmark suite).
template <typename F>
TimingSummary TimeRepeatedly(int warmup, int reps, F&& body) {
  for (int i = 0; i < warmup; ++i) {
    body();
  }
  std::vector<double> ms;
  ms.reserve(reps);
  for (int i = 0; i < reps; ++i) {
    const auto t0 = std::chrono::steady_clock::now();
    body();
    const auto t1 = std::chrono::steady_clock::now();
    ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
  }
  std::sort(ms.begin(), ms.end());
  TimingSummary s;
  s.reps = reps;
  s.min_ms = ms.front();
  s.max_ms = ms.back();
  s.median_ms =
      (reps % 2 == 1) ? ms[reps / 2] : 0.5 * (ms[reps / 2 - 1] + ms[reps / 2]);
  return s;
}

void WriteTiming(JsonWriter* json, const std::string& key,
                 const TimingSummary& t);

// ---------------------------------------------------------------------------
// Machine identification
// ---------------------------------------------------------------------------

struct MachineInfo {
  std::string cpu_model;
  int core_count{0};
  std::string drake_commit;
  std::string drake_version_note;
};

/// Reads the CPU model from /proc/cpuinfo and records `drake_commit` (the
/// Drake revision the caller was built from, passed through verbatim) so
/// every result file self-identifies.
MachineInfo GetMachineInfo(const std::string& drake_commit);

void WriteMachine(JsonWriter* json, const MachineInfo& machine);

// ---------------------------------------------------------------------------
// Trajectories
// ---------------------------------------------------------------------------

/// Builds a C2 composite quintic Bézier through the columns of `waypoints`
/// (n × K) at the given `times` (K values, strictly increasing). Waypoint
/// velocities come from centred finite differences (zero at both ends) and
/// waypoint accelerations are zero, which is exactly the smooth composite
/// Bézier a GCS/B-spline planner would hand us — degree 5, K−1 segments.
///
/// Control points per segment (duration h, endpoint velocities v0, v1):
///   P0 = q0, P5 = q1,
///   P1 = P0 + h v0/5,        P4 = P5 − h v1/5,
///   P2 = P0 + 2 h v0/5,      P3 = P5 − 2 h v1/5,
/// which reproduces q(t0)=q0, q̇(t0)=v0, q̈(t0)=0 and likewise at t1.
std::shared_ptr<drake::trajectories::CompositeTrajectory<double>>
MakeQuinticCompositeBezier(const Eigen::MatrixXd& waypoints,
                           const std::vector<double>& times);

/// Path length in the plant's default edge metric: the unweighted Euclidean
/// configuration distance (LinearDistanceAndInterpolationProvider's default
/// weights are 1 for every non-quaternion coordinate), integrated along the
/// trajectory with `num_samples` chords. Used to derive the number of samples
/// a sampled checker would take at a given edge_step_size.
double PathLengthInEdgeMetric(const drake::trajectories::Trajectory<double>& t,
                              int num_samples);

/// Samples `count` configurations uniformly in trajectory time (inclusive of
/// both endpoints).
std::vector<Eigen::VectorXd> SampleTrajectory(
    const drake::trajectories::Trajectory<double>& trajectory, int count);

// ---------------------------------------------------------------------------
// Ground-truth swept clearance
// ---------------------------------------------------------------------------

/// True minimum signed distance along a trajectory, obtained by dense
/// sampling plus a golden-section refinement of the sampled argmin. This is
/// the benchmark's independent oracle: it is what "achieved clearance" means
/// in the result files. `min_env` restricts the minimum to robot-vs-
/// environment pairs (the quantity a shelf shift/scale actually controls);
/// `min_all` also includes robot-vs-robot pairs.
struct ClearanceReport {
  double min_all{0.0};
  double t_all{0.0};
  double min_env{0.0};
  double t_env{0.0};
  int samples{0};
};

/// Geometry ids belonging to bodies of the named model instances.
std::unordered_set<drake::geometry::GeometryId> CollectGeometryIds(
    const drake::planning::RobotDiagram<double>& diagram,
    const std::vector<std::string>& model_instance_names);

/// Dense-samples `trajectory` (`num_samples` configurations, split over
/// `num_threads` cloned contexts) and refines the minimum by golden section.
/// Distances beyond `max_distance` are not resolved; if no pair comes within
/// it the reported minimum saturates at `max_distance`.
ClearanceReport MeasureSweptClearance(
    const drake::planning::RobotDiagram<double>& diagram,
    const drake::trajectories::Trajectory<double>& trajectory,
    const std::unordered_set<drake::geometry::GeometryId>& env_ids,
    int num_samples, int num_threads, double max_distance);

/// Bisects `f` (assumed non-decreasing) on [lo, hi] for f(x) = target.
/// Returns x. Used to place the shelf at a requested swept clearance.
double BisectMonotone(const std::function<double(double)>& f, double lo,
                      double hi, double target, int iterations);

}  // namespace benchmark
}  // namespace continuous_collision
}  // namespace planning
}  // namespace drake
