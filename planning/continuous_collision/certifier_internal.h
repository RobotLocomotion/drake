#pragma once

/// @file
/// Internal driver of the adaptive interval certifier (the search algorithm)
/// and of the independent certificate replay (the search algorithm,
/// "certificate audit trail").
///
/// Nothing in this header is part of the public API; it exists so the facade
/// (`continuous_collision_checker.cc`), the certificate replay
/// (`certificate.cc`) and the node loop (`certifier_internal.cc`) can share
/// one set of per-call data structures without the core module depending on
/// the api layer.

#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/query_object.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/planning/continuous_collision/certificate.h"
#include "drake/planning/continuous_collision/distance_oracle.h"
#include "drake/planning/continuous_collision/motion_bound_table.h"
#include "drake/planning/continuous_collision/options.h"
#include "drake/planning/continuous_collision/piecewise_bezier_path.h"
#include "drake/planning/robot_diagram.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace planning {
namespace continuous_collision {
namespace internal {

/** One thread's view of the model: a root diagram context plus the plant and
scene-graph sub-contexts pulled out of it once, so the hot loop pays a single
`SetPositions` per node (the performance requirements, P2) and no context
bookkeeping. */
class ThreadContext {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ThreadContext);

  /** Allocates a root context of `model`. `model` is aliased and must outlive
  this object. */
  explicit ThreadContext(const RobotDiagram<double>& model);

  /** The one FK trigger per node: sets the plant's generalized positions.
  Drake caches forward kinematics per context afterwards, so body poses and
  the query object are pulled lazily and only for the bodies/pairs that are
  still active. */
  void SetPositions(const Eigen::VectorXd& q);

  /** The scene graph's query object at the configuration last set. */
  const geometry::QueryObject<double>& query_object() const;

  /** World pose of `body` at the configuration last set (Drake's cache
  computes it on first use and reuses it afterwards). */
  const math::RigidTransform<double>& EvalBodyPose(
      multibody::BodyIndex body) const;

 private:
  const RobotDiagram<double>* model_{};
  std::unique_ptr<systems::Context<double>> root_;
  systems::Context<double>* plant_context_{};
  const systems::Context<double>* scene_graph_context_{};
};

/** A checkout pool of ThreadContexts (parallelism and determinism:
"construction allocates `parallelism.num_threads()` RobotDiagram contexts").

The pool is a *checkout* pool rather than a thread-indexed array so that the
public Check* methods stay safe to call concurrently from several threads:
each call leases the contexts it needs for its duration and no two workers can
ever share one. A lease larger than the pre-warmed pool grows it (a cold-path
allocation); nothing shrinks it. */
class ContextPool {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ContextPool);

  /** Pre-warms `initial_size` contexts of `model`, which is aliased and must
  outlive this pool. */
  ContextPool(const RobotDiagram<double>& model, int initial_size);

  /** RAII handle for a set of leased contexts. */
  class Lease {
   public:
    Lease(const Lease&) = delete;
    Lease& operator=(const Lease&) = delete;
    /* Hand-written rather than defaulted: the moved-from lease must stop
     owning its slots, and move-assignment must return the slots it already
     holds, or those pool entries stay marked in-use forever. */
    Lease(Lease&& other) noexcept { *this = std::move(other); }
    Lease& operator=(Lease&& other) noexcept;
    ~Lease();

    int size() const { return static_cast<int>(contexts_.size()); }
    ThreadContext& operator[](int i) const { return *contexts_[i]; }

   private:
    friend class ContextPool;
    Lease(const ContextPool* pool, std::vector<ThreadContext*> contexts,
          std::vector<int> slots)
        : pool_(pool),
          contexts_(std::move(contexts)),
          slots_(std::move(slots)) {}

    const ContextPool* pool_{};
    std::vector<ThreadContext*> contexts_;
    std::vector<int> slots_;
  };

  /** Leases exactly `count` contexts, growing the pool if it is exhausted. */
  Lease Acquire(int count) const;

  /** Number of contexts currently held by the pool (for tests/diagnostics). */
  int size() const;

 private:
  void Release(const std::vector<int>& slots) const;

  const RobotDiagram<double>* model_{};
  mutable std::mutex mutex_;
  /* A deque so that growing never invalidates the ThreadContext addresses
   already handed out. */
  mutable std::deque<std::unique_ptr<ThreadContext>> slots_;
  mutable std::vector<bool> in_use_;
};

/** Per-pair broadphase data for the free-sphere prefilter (the interval
certificate): geometry bounding spheres in their body frames, indexed by dense
slots so the node loop can cache one world-frame center per geometry per node.
*/
struct PrefilterTable {
  struct Geometry {
    multibody::BodyIndex body;
    Eigen::Vector3d center_L{Eigen::Vector3d::Zero()};
    double radius{0.0};
  };
  /** Dense geometry slots; only geometries that *have* a bounding sphere
  appear (HalfSpace has none). */
  std::vector<Geometry> geometries;
  /** Per pair: slot of geometry a / b, or -1 when that geometry has no sphere
  (a HalfSpace), in which case the pair skips the prefilter and goes straight
  to the (cheap, analytic) oracle route. */
  std::vector<int> slot_a;
  std::vector<int> slot_b;
};

/** Everything one certification run needs; assembled by the facade. All
pointers are aliased and must outlive the call. */
struct CertifierInput {
  const RobotDiagram<double>* model{};
  const DistanceOracle* oracle{};
  const MotionBoundTable* table{};
  const PiecewiseBezierPath* path{};
  /** Pair records with `threshold` = margin + padding resolved for this call.
  Indexed consistently with `table`, `tau` and `prefilter`. */
  const std::vector<PairRecord>* pairs{};
  /** Per-pair oracle tolerance τ_p (a refinement of the numerical policy; see
  the table in the facade). */
  const std::vector<double>* tau{};
  const PrefilterTable* prefilter{};
  Options options;
};

/** Result of one run, converted to a CertificationResult by the facade. */
struct CertifierOutput {
  Verdict verdict{Verdict::kCertifiedFree};
  /** Earliest-first, capped at Options::max_reported_findings. */
  std::vector<Finding> findings;
  Statistics stats;
  /** Filled iff Options::emit_certificate; records are sorted by
  (segment, s_start, pair_index) so a run is comparable across thread counts
  and across time reparametrizations.

  A *complete* audit trail — one whose certified intervals cover the whole
  domain for every pair, which is what ReplayCertificate() demands — is
  produced only by a run that ends Verdict::kCertifiedFree. A run that found a
  violation, hit the resolution floor, exhausted its budget, or pruned the
  search (kFindFirstViolation) leaves the uncertified parts uncovered by
  construction; its records are still individually valid, but they do not
  amount to a proof and ReplayCertificate() will say so. */
  Certificate certificate;
};

/** Runs the breakpoint pre-pass, the static-pair resolution and the adaptive
node recursion of the search algorithm over every segment of `input.path`,
serially or in parallel according to `input.options.parallelism`.

<h3>Parallel driver (supersedes the white paper's seeding sketch)</h3>

The white paper sketches "a work-stealing deque of nodes (seeded with all
segments' roots, or the top few bisection levels for small segment counts)".
That *static* seeding is what the first implementation did, and it does not
work: the certifier's trees are wildly unbalanced (a grazing trajectory
concentrates all of its subdivision in a band a few 10⁻³ wide in segment
parameter), so whatever fixed set of seeds is cut, one of them holds essentially
the whole tree. Measured: 0.98× at 16 threads on a 12.5k-node workload. Three
policies replace it; those *contracts* (per-thread contexts, one atomic
earliest- violation bound, findings sink under a mutex, only those three shared)
are unchanged.

- **Sharing policy — occupancy-driven, not depth-driven.** There is one shared
  LIFO work source. A worker that has just split a node consults the queue's
  length: if it is below the number of live workers, the worker pushes its
  *right* child there and keeps the left one on its local stack; otherwise it
  keeps both. Sharing is therefore self-throttling (a saturated queue costs
  nothing) and, crucially, does not stop at any depth — a worker sitting on the
  last deep subtree with every other worker idle hands out a node per level
  until the tail is spread. Giving away the right child keeps each worker's own
  descent left-first, which is what makes kFindFirstViolation's bound tighten
  early.
- **Recruitment policy — lazy.** The call starts as a plain serial descent on
  the calling thread with sharing disabled, and hires helpers only after it has
  visited `kNodesBeforeHiringHelpers` nodes. A check whose whole workload is
  smaller than that (a PWL edge, a shallow shelf check) therefore runs at
  exactly serial speed no matter what `Options::parallelism` says — which
  matters because `Parallelism::Max()` is the default. Helpers are call-scoped
  threads, spawned once per check when (and only when) that threshold is
  crossed and joined before the call returns; nothing here owns a background
  thread between calls.
- **Determinism policy — unchanged, because sharing moves nodes between
  workers without changing which nodes exist.** Every node's decisions depend
  only on its own control points and its inherited active set, so the tree, the
  statistics summed over workers, and the findings are identical serially and
  at any thread count in kCertifyAll. The two documented order-dependent
  features are untouched: kFindFirstViolation's branch-and-bound (which prunes
  only nodes starting at or after a witness already found, so the *reported*
  witness is invariant while the statistics are not) and the `max_nodes` budget
  (which truncates at a thread-count dependent place).

Determinism (performance requirement P7; parallelism and
determinism): the serial path is bit-deterministic. In
kFindFirstViolation the *reported witness* is identical serially and at any
thread count — the branch-and-bound bound only ever prunes nodes that start at
or after a witness already found, so no node that could hold an earlier one is
ever skipped — while the statistics are not. Two documented exceptions to the
witness claim: a run that exhausts `max_nodes` truncates at a thread-count
dependent place, and on a degenerate segment with t_start == t_end every node
maps to the same time, so the bound prunes on a tie and the reported
configuration (not its time) may differ.

`pool` supplies the per-thread contexts. Helper threads, if any are hired, are
created and joined within this call.

@throws std::exception if the oracle throws for any pair; a parallel run waits
for every worker first and rethrows the first failure. */
CertifierOutput RunCertifier(const CertifierInput& input, ContextPool* pool);

// ---------------------------------------------------------------------------
// Certificate assembly + independent replay (implemented in certificate.cc).
// ---------------------------------------------------------------------------

/** Restricts the Bézier control points `cps` (n × (m+1)) of a segment to the
sub-interval [a, b] ⊆ [0, 1] by two de Casteljau subdivisions, writing the
n × (m+1) control points of the restricted curve into `out`.

This is a local, cold-path implementation used only by the certificate replay:
the replay must not reuse the certifier's own subdivision code path if it is to
be an independent check. */
void RestrictBezier(const Eigen::MatrixXd& cps, double a, double b,
                    Eigen::MatrixXd* out);

/** Evaluates the Bézier curve with control points `cps` at u ∈ [0, 1] by de
Casteljau (the apex of the triangle). Cold path. */
Eigen::VectorXd EvaluateBezier(const Eigen::MatrixXd& cps, double u);

/** Inputs of the independent certificate replay. All pointers are aliased. */
struct ReplayInput {
  const RobotDiagram<double>* model{};
  const DistanceOracle* oracle{};
  const MotionBoundTable* table{};
  const PiecewiseBezierPath* path{};
  const std::vector<PairRecord>* pairs{};
  const std::vector<double>* tau{};
  double slack{1e-9};
};

/** Independently re-evaluates every record of `certificate` and checks that
the certified intervals cover the whole domain for every pair (the search
algorithm). Returns true iff the certificate is a complete, self-consistent
proof that every pair stays above its recorded threshold everywhere on the path.
When it returns false and `message` is non-null, `*message` explains why. */
bool ReplayCertificate(const ReplayInput& input, const Certificate& certificate,
                       std::string* message);

}  // namespace internal
}  // namespace continuous_collision
}  // namespace planning
}  // namespace drake
