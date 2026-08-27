#pragma once

// Helpers shared by the benchmark scenarios: a hand-rolled JSON writer,
// steady_clock timing with medians, machine identification, quintic
// composite-Bézier construction, and a dense ground-truth swept-clearance
// sampler that verifies, but never certifies, the clearance of every scenario
// trajectory. No third-party benchmark framework is involved: the measurements
// here are millisecond-scale wall clock repeated by hand, and the JSON is
// consumed by CI tracking.

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>

#include "drake/common/trajectories/composite_trajectory.h"
#include "drake/common/trajectories/trajectory.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/planning/robot_diagram.h"

namespace drake {
namespace planning {
namespace continuous_collision {
namespace internal {

// ---------------------------------------------------------------------------
// JSON
// ---------------------------------------------------------------------------

// Minimal streaming JSON writer: enough for the fixed result schema, with no
// dependency. Callers must balance Begin*/End* calls.
class JsonWriter {
 public:
  JsonWriter() = default;

  // Opens an object as the next element of the innermost array.
  void BeginObject();
  // Opens an object under `key` in the innermost object.
  void BeginObject(const std::string& key);
  // Closes the innermost object.
  void EndObject();
  // Opens an array under `key` in the innermost object.
  void BeginArray(const std::string& key);
  // Closes the innermost array.
  void EndArray();

  // Writes one `key`: `value` member into the innermost object.
  void Write(const std::string& key, double value);
  void Write(const std::string& key, int value);
  void Write(const std::string& key, long value);           // NOLINT
  void Write(const std::string& key, unsigned long value);  // NOLINT
  void Write(const std::string& key, bool value);
  void Write(const std::string& key, const char* value);
  void Write(const std::string& key, const std::string& value);
  // Appends a bare double to the innermost array.
  void WriteArrayValue(double value);
  // Appends a bare string to the innermost array.
  void WriteArrayValue(const std::string& value);

  // Returns the document written so far, newline-terminated.
  std::string str() const { return out_ + "\n"; }

 private:
  void Separator();
  void Indent();

  std::string out_;
  std::vector<bool> first_;  // per open container: "nothing written yet"
  int depth_{0};
};

// Writes `text` to `path`, creating parent directories as needed.
void WriteTextFile(const std::string& path, const std::string& text);

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------

// Wall-clock summary of one repeated measurement, in milliseconds.
struct TimingSummary {
  double median_ms{0.0};
  double min_ms{0.0};
  double max_ms{0.0};
  int reps{0};
};

// Runs `body` `warmup` times untimed, then `reps` times timed, and reduces the
// sample to median/min/max. No pinning and no frequency control: the numbers
// are what a user on this machine would see. An exception thrown by `body`
// propagates and no summary is produced.
// @pre reps >= 1; the summary reads the ends of a sample of `reps` entries.
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

// Writes `t` as an object under `key`: median, min, max and reps.
void WriteTiming(JsonWriter* json, const std::string& key,
                 const TimingSummary& t);

// ---------------------------------------------------------------------------
// Machine identification
// ---------------------------------------------------------------------------

// Identification of the machine and the build a result file was produced on.
struct MachineInfo {
  std::string cpu_model;
  int core_count{0};
  std::string drake_commit;
  std::string drake_version_note;
};

// Reads the CPU model from /proc/cpuinfo and records `drake_commit` (the
// Drake revision the caller was built from, passed through verbatim) so
// every result file self-identifies.
MachineInfo GetMachineInfo(const std::string& drake_commit);

// Writes `machine` as the "machine" object of a result file.
void WriteMachine(JsonWriter* json, const MachineInfo& machine);

// ---------------------------------------------------------------------------
// Trajectories
// ---------------------------------------------------------------------------

// Builds a C2 composite quintic Bézier through the columns of `waypoints`
// (n × K) at `times` (K strictly increasing values): the smooth composite
// Bézier a GCS/B-spline planner would hand us, degree 5 with K−1 segments.
// Waypoint velocities are centred finite differences (zero at both ends) and
// waypoint accelerations are zero. Per segment, duration h, velocities v0, v1:
// clang-format off
//   P0 = q0, P5 = q1,
//   P1 = P0 + h v0/5,        P4 = P5 − h v1/5,
//   P2 = P0 + 2 h v0/5,      P3 = P5 − 2 h v1/5,
// clang-format on
// so q(t0)=q0, q̇(t0)=v0, q̈(t0)=0, and likewise at t1.
// @throws std::exception if waypoints.cols() < 2.
// @throws std::exception if times.size() != waypoints.cols().
// @pre times is strictly increasing; the centred differences divide by
// times[i+1] - times[i-1].
std::shared_ptr<trajectories::CompositeTrajectory<double>>
MakeQuinticCompositeBezier(const Eigen::MatrixXd& waypoints,
                           const std::vector<double>& times);

// Path length in the plant's default edge metric: the unweighted Euclidean
// configuration distance (LinearDistanceAndInterpolationProvider's default
// weights are 1 for every non-quaternion coordinate), integrated along the
// trajectory with `num_samples` chords. Used to derive the number of samples
// a sampled checker would take at a given edge_step_size.
double PathLengthInEdgeMetric(const trajectories::Trajectory<double>& t,
                              int num_samples);

// Samples `count` configurations uniformly in trajectory time (inclusive of
// both endpoints).
std::vector<Eigen::VectorXd> SampleTrajectory(
    const trajectories::Trajectory<double>& trajectory, int count);

// ---------------------------------------------------------------------------
// Ground-truth swept clearance
// ---------------------------------------------------------------------------

// The result of MeasureSweptClearance. `min_env` restricts the minimum to
// robot-vs-environment pairs (the quantity a shelf shift/scale actually
// controls); `min_all` also includes robot-vs-robot pairs. `t_all` and `t_env`
// are the trajectory times at which those minima occur.
struct ClearanceReport {
  double min_all{0.0};
  double t_all{0.0};
  double min_env{0.0};
  double t_env{0.0};
  int samples{0};
};

// Geometry ids belonging to bodies of the named model instances.
std::unordered_set<geometry::GeometryId> CollectGeometryIds(
    const RobotDiagram<double>& diagram,
    const std::vector<std::string>& model_instance_names);

// True minimum signed distance along `trajectory`, obtained by dense sampling
// (`num_samples` configurations split over `num_threads` cloned contexts) plus
// a golden-section refinement of the sampled argmin. This is the benchmark's
// independent oracle: it is what "achieved clearance" means in the result
// files. Distances beyond `max_distance` are not resolved; if no pair comes
// within it the reported minimum saturates at `max_distance`. A `num_threads`
// below 1 is treated as 1.
// @throws std::exception if a configuration on `trajectory` does not have
// diagram.plant().num_positions() rows, or holds a non-finite value.
// @pre num_samples >= 2; the sample times divide by num_samples - 1.
ClearanceReport MeasureSweptClearance(
    const RobotDiagram<double>& diagram,
    const trajectories::Trajectory<double>& trajectory,
    const std::unordered_set<geometry::GeometryId>& env_ids, int num_samples,
    int num_threads, double max_distance);

// Bisects `f` (assumed non-decreasing) on [lo, hi] for f(x) = target.
// Returns x. Used to place the shelf at a requested swept clearance.
double BisectMonotone(const std::function<double(double)>& f, double lo,
                      double hi, double target, int iterations);

}  // namespace internal
}  // namespace continuous_collision
}  // namespace planning
}  // namespace drake
