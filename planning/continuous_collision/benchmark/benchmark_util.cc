#include "drake/planning/continuous_collision/benchmark/benchmark_util.h"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/trajectories/bezier_curve.h"
#include "drake/geometry/query_object.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace planning {
namespace continuous_collision {
namespace internal {
namespace {

using drake::geometry::GeometryId;
using drake::geometry::QueryObject;
using drake::geometry::SignedDistancePair;
using drake::planning::RobotDiagram;
using drake::systems::Context;
using drake::trajectories::BezierCurve;
using drake::trajectories::CompositeTrajectory;
using drake::trajectories::Trajectory;
using Eigen::MatrixXd;
using Eigen::VectorXd;

/// Formats a double with enough digits to round-trip through the JSON.
std::string FormatDouble(double v) {
  if (std::isnan(v)) return "null";
  if (std::isinf(v)) return v > 0 ? "1e999" : "-1e999";
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.10g", v);
  return buf;
}

std::string Escape(const std::string& s) {
  std::string out;
  for (const char c : s) {
    switch (c) {
      case '"':
        out += "\\\"";
        break;
      case '\\':
        out += "\\\\";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        out += c;
    }
  }
  return out;
}

/// One (t, min-distance-over-all-pairs, min-distance-over-env-pairs) probe.
struct Probe {
  double all{0.0};
  double env{0.0};
};

Probe ProbeAt(const RobotDiagram<double>& diagram, Context<double>* root,
              const VectorXd& q, const std::unordered_set<GeometryId>& env_ids,
              const drake::geometry::SceneGraphInspector<double>& inspector,
              double max_distance) {
  diagram.plant().SetPositions(&diagram.mutable_plant_context(root), q);
  const auto& query_object = diagram.scene_graph()
                                 .get_query_output_port()
                                 .template Eval<QueryObject<double>>(
                                     diagram.scene_graph_context(*root));
  const std::vector<SignedDistancePair<double>> pairs =
      query_object.ComputeSignedDistancePairwiseClosestPoints(max_distance);
  Probe p{max_distance, max_distance};
  for (const auto& pair : pairs) {
    if (pair.distance < p.all) p.all = pair.distance;
    const bool a_env = env_ids.count(pair.id_A) > 0;
    const bool b_env = env_ids.count(pair.id_B) > 0;
    if (a_env != b_env && pair.distance < p.env) p.env = pair.distance;
    (void)inspector;
  }
  return p;
}

/// Golden-section minimization of `f` on [lo, hi]; the sampled bracket around
/// a dense-sample argmin is unimodal in practice for these smooth curves.
std::pair<double, double> GoldenSectionMin(
    const std::function<double(double)>& f, double lo, double hi,
    int iterations) {
  constexpr double kInvPhi = 0.6180339887498949;
  double a = lo;
  double b = hi;
  double c = b - kInvPhi * (b - a);
  double d = a + kInvPhi * (b - a);
  double fc = f(c);
  double fd = f(d);
  for (int i = 0; i < iterations; ++i) {
    if (fc < fd) {
      b = d;
      d = c;
      fd = fc;
      c = b - kInvPhi * (b - a);
      fc = f(c);
    } else {
      a = c;
      c = d;
      fc = fd;
      d = a + kInvPhi * (b - a);
      fd = f(d);
    }
  }
  return (fc < fd) ? std::make_pair(fc, c) : std::make_pair(fd, d);
}

}  // namespace

// ---------------------------------------------------------------------------
// JsonWriter
// ---------------------------------------------------------------------------

void JsonWriter::Indent() {
  out_.append(static_cast<size_t>(2 * depth_), ' ');
}

void JsonWriter::Separator() {
  if (!first_.empty()) {
    if (first_.back()) {
      first_.back() = false;
    } else {
      out_ += ",";
    }
    out_ += "\n";
    Indent();
  }
}

void JsonWriter::BeginObject() {
  Separator();
  out_ += "{";
  first_.push_back(true);
  ++depth_;
}

void JsonWriter::BeginObject(const std::string& key) {
  Separator();
  out_ += "\"" + Escape(key) + "\": {";
  first_.push_back(true);
  ++depth_;
}

void JsonWriter::EndObject() {
  const bool empty = first_.back();
  first_.pop_back();
  --depth_;
  if (!empty) {
    out_ += "\n";
    Indent();
  }
  out_ += "}";
}

void JsonWriter::BeginArray(const std::string& key) {
  Separator();
  out_ += "\"" + Escape(key) + "\": [";
  first_.push_back(true);
  ++depth_;
}

void JsonWriter::EndArray() {
  const bool empty = first_.back();
  first_.pop_back();
  --depth_;
  if (!empty) {
    out_ += "\n";
    Indent();
  }
  out_ += "]";
}

void JsonWriter::Write(const std::string& key, double value) {
  Separator();
  out_ += "\"" + Escape(key) + "\": " + FormatDouble(value);
}

void JsonWriter::Write(const std::string& key, int value) {
  Separator();
  out_ += "\"" + Escape(key) + "\": " + std::to_string(value);
}

void JsonWriter::Write(const std::string& key, long value) {  // NOLINT
  Separator();
  out_ += "\"" + Escape(key) + "\": " + std::to_string(value);
}

void JsonWriter::Write(const std::string& key,
                       unsigned long value) {  // NOLINT
  Separator();
  out_ += "\"" + Escape(key) + "\": " + std::to_string(value);
}

void JsonWriter::Write(const std::string& key, bool value) {
  Separator();
  out_ += "\"" + Escape(key) + "\": " + (value ? "true" : "false");
}

void JsonWriter::Write(const std::string& key, const char* value) {
  Write(key, std::string(value));
}

void JsonWriter::Write(const std::string& key, const std::string& value) {
  Separator();
  out_ += "\"" + Escape(key) + "\": \"" + Escape(value) + "\"";
}

void JsonWriter::WriteArrayValue(double value) {
  Separator();
  out_ += FormatDouble(value);
}

void JsonWriter::WriteArrayValue(const std::string& value) {
  Separator();
  out_ += "\"" + Escape(value) + "\"";
}

void WriteTextFile(const std::string& path, const std::string& text) {
  const std::filesystem::path p(path);
  if (p.has_parent_path()) {
    std::filesystem::create_directories(p.parent_path());
  }
  std::ofstream file(path);
  if (!file) throw std::runtime_error("cannot open for writing: " + path);
  file << text;
}

void WriteTiming(JsonWriter* json, const std::string& key,
                 const TimingSummary& t) {
  json->BeginObject(key);
  json->Write("median", t.median_ms);
  json->Write("min", t.min_ms);
  json->Write("max", t.max_ms);
  json->Write("reps", t.reps);
  json->EndObject();
}

// ---------------------------------------------------------------------------
// Machine
// ---------------------------------------------------------------------------

MachineInfo GetMachineInfo(const std::string& drake_commit) {
  MachineInfo info;
  info.core_count = static_cast<int>(std::thread::hardware_concurrency());
  {
    std::ifstream cpuinfo("/proc/cpuinfo");
    std::string line;
    while (std::getline(cpuinfo, line)) {
      const size_t colon = line.find(':');
      if (colon == std::string::npos) continue;
      if (line.compare(0, 10, "model name") != 0) continue;
      info.cpu_model = line.substr(colon + 1);
      const size_t start = info.cpu_model.find_first_not_of(" \t");
      if (start != std::string::npos)
        info.cpu_model = info.cpu_model.substr(start);
      break;
    }
  }
  // The Drake revision is not discoverable from inside the binary, so the
  // caller passes it in (--drake_commit) and it is recorded verbatim.
  info.drake_commit = drake_commit;
  info.drake_version_note = "built from the Drake source tree";
  return info;
}

void WriteMachine(JsonWriter* json, const MachineInfo& machine) {
  json->BeginObject("machine");
  json->Write("cpu_model", machine.cpu_model);
  json->Write("core_count", machine.core_count);
  json->Write("drake_commit", machine.drake_commit);
  json->Write("drake_version", machine.drake_version_note);
  json->EndObject();
}

// ---------------------------------------------------------------------------
// Trajectories
// ---------------------------------------------------------------------------

std::shared_ptr<CompositeTrajectory<double>> MakeQuinticCompositeBezier(
    const MatrixXd& waypoints, const std::vector<double>& times) {
  const int n = static_cast<int>(waypoints.rows());
  const int k = static_cast<int>(waypoints.cols());
  if (k < 2 || static_cast<int>(times.size()) != k) {
    throw std::runtime_error("MakeQuinticCompositeBezier: bad sizes");
  }
  MatrixXd velocity = MatrixXd::Zero(n, k);
  for (int i = 1; i + 1 < k; ++i) {
    velocity.col(i) = (waypoints.col(i + 1) - waypoints.col(i - 1)) /
                      (times[i + 1] - times[i - 1]);
  }
  std::vector<drake::copyable_unique_ptr<Trajectory<double>>> segments;
  for (int i = 0; i + 1 < k; ++i) {
    const double h = times[i + 1] - times[i];
    MatrixXd cps(n, 6);
    const VectorXd p0 = waypoints.col(i);
    const VectorXd p5 = waypoints.col(i + 1);
    const VectorXd v0 = velocity.col(i);
    const VectorXd v1 = velocity.col(i + 1);
    cps.col(0) = p0;
    cps.col(1) = p0 + h * v0 / 5.0;
    cps.col(2) = p0 + 2.0 * h * v0 / 5.0;
    cps.col(3) = p5 - 2.0 * h * v1 / 5.0;
    cps.col(4) = p5 - h * v1 / 5.0;
    cps.col(5) = p5;
    segments.emplace_back(
        std::make_unique<BezierCurve<double>>(times[i], times[i + 1], cps));
  }
  return std::make_shared<CompositeTrajectory<double>>(std::move(segments));
}

std::vector<VectorXd> SampleTrajectory(const Trajectory<double>& trajectory,
                                       int count) {
  const double t0 = trajectory.start_time();
  const double t1 = trajectory.end_time();
  std::vector<VectorXd> out;
  out.reserve(count);
  for (int i = 0; i < count; ++i) {
    const double t = (count == 1) ? t0
                                  : t0 + (t1 - t0) * static_cast<double>(i) /
                                             static_cast<double>(count - 1);
    out.push_back(trajectory.value(t).col(0));
  }
  return out;
}

double PathLengthInEdgeMetric(const Trajectory<double>& trajectory,
                              int num_samples) {
  const std::vector<VectorXd> qs = SampleTrajectory(trajectory, num_samples);
  double length = 0.0;
  for (size_t i = 1; i < qs.size(); ++i) {
    length += (qs[i] - qs[i - 1]).norm();
  }
  return length;
}

// ---------------------------------------------------------------------------
// Ground-truth swept clearance
// ---------------------------------------------------------------------------

std::unordered_set<GeometryId> CollectGeometryIds(
    const RobotDiagram<double>& diagram,
    const std::vector<std::string>& model_instance_names) {
  const auto& plant = diagram.plant();
  const auto& inspector = diagram.scene_graph().model_inspector();
  std::unordered_set<GeometryId> ids;
  for (const std::string& name : model_instance_names) {
    if (!plant.HasModelInstanceNamed(name)) continue;
    const auto instance = plant.GetModelInstanceByName(name);
    for (const auto& body_index : plant.GetBodyIndices(instance)) {
      const auto frame_id = plant.GetBodyFrameIdOrThrow(body_index);
      for (const auto& id : inspector.GetGeometries(
               frame_id, drake::geometry::Role::kProximity)) {
        ids.insert(id);
      }
    }
  }
  return ids;
}

ClearanceReport MeasureSweptClearance(
    const RobotDiagram<double>& diagram, const Trajectory<double>& trajectory,
    const std::unordered_set<GeometryId>& env_ids, int num_samples,
    int num_threads, double max_distance) {
  const double t0 = trajectory.start_time();
  const double t1 = trajectory.end_time();
  const auto& inspector = diagram.scene_graph().model_inspector();

  const int threads = std::max(1, num_threads);
  std::vector<double> best_all(threads, max_distance);
  std::vector<double> best_env(threads, max_distance);
  std::vector<int> arg_all(threads, 0);
  std::vector<int> arg_env(threads, 0);

  const auto worker = [&](int tid) {
    auto root = diagram.CreateDefaultContext();
    for (int i = tid; i < num_samples; i += threads) {
      const double t = t0 + (t1 - t0) * static_cast<double>(i) /
                                static_cast<double>(num_samples - 1);
      const Probe p = ProbeAt(diagram, root.get(), trajectory.value(t).col(0),
                              env_ids, inspector, max_distance);
      if (p.all < best_all[tid]) {
        best_all[tid] = p.all;
        arg_all[tid] = i;
      }
      if (p.env < best_env[tid]) {
        best_env[tid] = p.env;
        arg_env[tid] = i;
      }
    }
  };

  if (threads == 1) {
    worker(0);
  } else {
    std::vector<std::thread> pool;
    pool.reserve(threads);
    for (int i = 0; i < threads; ++i) pool.emplace_back(worker, i);
    for (auto& th : pool) th.join();
  }

  ClearanceReport report;
  report.samples = num_samples;
  report.min_all = max_distance;
  report.min_env = max_distance;
  int i_all = 0;
  int i_env = 0;
  for (int i = 0; i < threads; ++i) {
    if (best_all[i] < report.min_all) {
      report.min_all = best_all[i];
      i_all = arg_all[i];
    }
    if (best_env[i] < report.min_env) {
      report.min_env = best_env[i];
      i_env = arg_env[i];
    }
  }

  // Refine each sampled argmin by golden section on the neighbouring bracket.
  auto root = diagram.CreateDefaultContext();
  const double dt = (t1 - t0) / static_cast<double>(num_samples - 1);
  const auto time_of = [&](int i) {
    return std::min(t1, std::max(t0, t0 + dt * static_cast<double>(i)));
  };
  const auto refine = [&](int index, bool env_only, double* value,
                          double* argt) {
    const double lo = time_of(index - 1);
    const double hi = time_of(index + 1);
    if (hi <= lo) {
      *argt = time_of(index);
      return;
    }
    const auto f = [&](double t) {
      const Probe p = ProbeAt(diagram, root.get(), trajectory.value(t).col(0),
                              env_ids, inspector, max_distance);
      return env_only ? p.env : p.all;
    };
    const auto [best, at] = GoldenSectionMin(f, lo, hi, 60);
    if (best < *value) *value = best;
    *argt = at;
  };
  refine(i_all, false, &report.min_all, &report.t_all);
  refine(i_env, true, &report.min_env, &report.t_env);
  return report;
}

double BisectMonotone(const std::function<double(double)>& f, double lo,
                      double hi, double target, int iterations) {
  double a = lo;
  double b = hi;
  for (int i = 0; i < iterations; ++i) {
    const double mid = 0.5 * (a + b);
    if (f(mid) < target) {
      a = mid;
    } else {
      b = mid;
    }
  }
  return 0.5 * (a + b);
}

}  // namespace internal
}  // namespace continuous_collision
}  // namespace planning
}  // namespace drake
