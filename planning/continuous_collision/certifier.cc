#include "drake/planning/continuous_collision/certifier.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <functional>
#include <future>
#include <limits>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/parallelism.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace planning {
namespace continuous_collision {
namespace internal {
namespace {

using drake::geometry::QueryObject;
using drake::math::RigidTransformd;
using drake::multibody::BodyIndex;

constexpr double kInfinity = std::numeric_limits<double>::infinity();

/* Global (trajectory) time of parameter s in `seg`. Segment times are pure
 bookkeeping: the recursion runs in the segment parameter s ∈ [0, 1] and only
 the *reported* times go through this map, which is why the proof is invariant
 under time reparametrization. */
double TimeOf(const BezierSegment& seg, double s) {
  return seg.t_start + s * (seg.t_end - seg.t_start);
}

Finding MakeFinding(double time, const Eigen::VectorXd& q,
                    const PairRecord& pair, double distance,
                    const Eigen::Vector3d& nearest_a_W,
                    const Eigen::Vector3d& nearest_b_W) {
  Finding finding;
  finding.time = time;
  finding.q = q;
  finding.geometry_a = pair.a;
  finding.geometry_b = pair.b;
  finding.body_a = pair.body_a;
  finding.body_b = pair.body_b;
  finding.distance = distance;
  finding.nearest_a_W = nearest_a_W;
  finding.nearest_b_W = nearest_b_W;
  return finding;
}

/* Caches one world-frame bounding-sphere center per geometry per node. The
 poses behind them are pulled lazily from Drake's FK cache and only for
 geometries of still-active pairs. Invalidation is a stamp bump, so switching
 to a new configuration is O(1). */
class GeometryCache {
 public:
  GeometryCache(const PrefilterTable& table, const ThreadContext& context)
      : table_(&table),
        context_(&context),
        center_W_(table.geometries.size()),
        stamp_of_(table.geometries.size(), 0) {}

  /* Invalidates every cached center; call once per configuration. */
  void NewConfiguration() { ++stamp_; }

  const Eigen::Vector3d& Center(int slot) {
    if (stamp_of_[slot] != stamp_) {
      const PrefilterTable::Geometry& g = table_->geometries[slot];
      center_W_[slot] = context_->EvalBodyPose(g.body) * g.center_L;
      stamp_of_[slot] = stamp_;
    }
    return center_W_[slot];
  }

  /* The free-sphere lower bound on the pair's signed distance at the
   configuration last set: phi >= ||c_A - c_B|| - rho_A - rho_B. */
  double LowerBound(int slot_a, int slot_b) {
    return (Center(slot_a) - Center(slot_b)).norm() -
           table_->geometries[slot_a].radius -
           table_->geometries[slot_b].radius;
  }

 private:
  const PrefilterTable* table_{};
  const ThreadContext* context_{};
  std::vector<Eigen::Vector3d> center_W_;
  std::vector<std::uint64_t> stamp_of_;
  std::uint64_t stamp_{1};
};

/* Collects the earliest violation and the earliest inconclusive witness from
 every worker. Cold path: guarded by one mutex. */
class FindingSink {
 public:
  void AddDefinite(Finding finding) {
    const double time = finding.time;
    {
      std::lock_guard<std::mutex> guard(mutex_);
      if (!definite_.has_value() || time < definite_->time) {
        definite_ = std::move(finding);
      }
    }
    // Branch-and-bound bound: workers skip nodes whose interval starts at or
    // after the earliest witness known so far. The bound decreases
    // monotonically, so a node that could hold an earlier witness is never
    // pruned and the answer does not depend on timing.
    double previous = best_violation_time_.load(std::memory_order_relaxed);
    while (time < previous && !best_violation_time_.compare_exchange_weak(
                                  previous, time, std::memory_order_relaxed)) {
    }
  }

  void AddInconclusive(Finding finding) {
    std::lock_guard<std::mutex> guard(mutex_);
    if (!inconclusive_.has_value() || finding.time < inconclusive_->time) {
      inconclusive_ = std::move(finding);
    }
  }

  double best_violation_time() const {
    return best_violation_time_.load(std::memory_order_relaxed);
  }

  std::optional<Finding>& definite() { return definite_; }
  std::optional<Finding>& inconclusive() { return inconclusive_; }

 private:
  std::mutex mutex_;
  std::optional<Finding> definite_;
  std::optional<Finding> inconclusive_;
  std::atomic<double> best_violation_time_{kInfinity};
};

/* One unit of shared work: a node, self-contained so a worker can pick it up
 without touching any other worker's arenas.

 Work items carry copies (control points and the active-pair span) rather than
 pointing into the producing worker's arenas, because the producer walks on
 immediately. The steady-state loop still allocates nothing, because the queue
 recycles item *shells*: a popped shell goes back on a free list and is handed
 to the next producer, whose `resize`/`assign` then reuse the buffers already
 attached to it. Allocation happens while the free list is filling up and never
 again. */
struct WorkItem {
  int segment{};
  double s_lo{0.0};
  double s_hi{1.0};
  Eigen::MatrixXd control_points;
  std::vector<int> active;
};

/* Mutex-guarded LIFO work source with quiescence detection, shell recycling
 and the occupancy counter that drives the sharing policy. The *only* shared
 mutable state of the parallel driver is this queue, the FindingSink, and the
 atomic node counter, which is what makes the driver TSan-clean by
 construction. */
class WorkQueue {
 public:
  /* Moves `*item` into the queue and hands back a recycled shell (or an empty
   one) so the producer can fill it again without allocating. */
  void Push(WorkItem* item) {
    {
      std::lock_guard<std::mutex> guard(mutex_);
      items_.push_back(std::move(*item));
      if (free_.empty()) {
        *item = WorkItem{};
      } else {
        *item = std::move(free_.back());
        free_.pop_back();
      }
      size_.store(static_cast<int>(items_.size()), std::memory_order_relaxed);
    }
    condition_.notify_one();
  }

  /* Blocks until an item is available, or until every worker is idle and the
   queue is empty (returns false), or until Abort() (returns false). `*item`'s
   previous contents are recycled into the free list. */
  bool Pop(WorkItem* item) {
    std::unique_lock<std::mutex> lock(mutex_);
    while (true) {
      if (aborted_ || done_) return false;
      if (!items_.empty()) {
        free_.push_back(std::move(*item));
        *item = std::move(items_.back());
        items_.pop_back();
        size_.store(static_cast<int>(items_.size()), std::memory_order_relaxed);
        ++busy_;
        return true;
      }
      if (busy_ == 0) {
        done_ = true;
        condition_.notify_all();
        return false;
      }
      condition_.wait(lock);
    }
  }

  void FinishItem() {
    {
      std::lock_guard<std::mutex> guard(mutex_);
      --busy_;
      if (busy_ > 0 && items_.empty()) return;
    }
    condition_.notify_all();
  }

  void Abort() {
    {
      std::lock_guard<std::mutex> guard(mutex_);
      aborted_ = true;
    }
    condition_.notify_all();
  }

  /* The sharing policy (see certifier.h): a worker gives one child away
   whenever the queue holds fewer items than there are live workers. Reading
   the length through a relaxed atomic keeps the *test* off the queue's mutex,
   so only an actual share pays for the lock; a stale answer costs at most one
   redundant or one skipped share. `num_workers` is 0 until helpers are hired,
   which is exactly how lazy recruitment disables sharing. */
  bool ShouldShare() const {
    return size_.load(std::memory_order_relaxed) <
           num_workers_.load(std::memory_order_relaxed);
  }

  void set_num_workers(int count) {
    num_workers_.store(count, std::memory_order_relaxed);
  }

 private:
  std::mutex mutex_;
  std::condition_variable condition_;
  std::vector<WorkItem> items_;
  std::vector<WorkItem> free_;
  std::atomic<int> size_{0};
  std::atomic<int> num_workers_{0};
  int busy_{0};
  bool done_{false};
  bool aborted_{false};
};

/* Lazy-recruitment state. Only the lead worker ever touches it, so it needs no
 synchronization of its own: `hire` is called from inside the lead's node loop
 the first time the run has visited enough nodes to be worth spreading. */
struct Recruitment {
  std::uint64_t nodes{0};
  std::function<void()> hire;
};

/* How many nodes a run must have visited before it hires helpers.

 Hiring costs one ContextPool lease, the construction of the helper Worker
 objects, one thread creation per helper and, at the end of the run, one join
 per helper. Thread creation dominates that list at tens of microseconds per
 worker, while a node costs ~7-13 us on a modern desktop core, so 64 nodes of
 work already done is roughly a 3x margin over the price of a full fifteen
 helpers. It also bounds the one case lazy recruitment cannot avoid, a check
 that ends immediately after hiring, to a few hundred microseconds. Below the
 threshold a run is exactly serial at any Options::parallelism, which matters
 because Parallelism::Max() is that field's default. */
constexpr std::uint64_t kNodesBeforeHiringHelpers = 64;

/* One frame of the explicit LIFO node stack. The frame at stack index k owns
 control-point slab k of the worker's pool, and the pair indices it is still
 active for live in arena[active_offset, active_offset + active_length). */
struct NodeFrame {
  double s_lo{0.0};
  double s_hi{1.0};
  int active_offset{0};
  int active_length{0};
};

/* A worker owns all per-thread scratch of the node recursion; nothing in the
 steady-state loop allocates:

  - `slabs_` is the node pool: slab k is the n × (m+1) control-point matrix of
    the frame at stack index k. Splitting a node writes its left child into
    slab k+1 and swaps its right child into slab k (an O(1) Eigen buffer
    swap), so a split costs zero copies and zero allocations. Slabs are
    (re)sized only when the worker moves to a segment of a different Bézier
    order.
  - `arena_` is the survivor arena: each frame's active pair list is an index
    span into it. Both children of a node share one span (their active sets
    are identical), and a popped frame writes its survivors immediately above
    its own span, which is free because every frame still on the stack owns a
    span at or below that point.
  - `stack_` grows by one per level, so all three arrays are bounded by the
    depth the resolution floor allows. */
class Worker {
 public:
  Worker(const CertifierInput& input, ThreadContext* context, FindingSink* sink,
         std::atomic<std::uint64_t>* node_counter, WorkQueue* queue,
         Recruitment* recruit)
      : input_(input),
        context_(context),
        sink_(sink),
        node_counter_(node_counter),
        queue_(queue),
        recruit_(recruit),
        geometry_(*input.prefilter, *context) {
    const int n = input_.path->num_positions();
    q_mid_.resize(n);
    w_.resize(n);
  }

  /* Parallel entry point: pulls items until the work source is quiescent. The
   item buffer is reused across iterations and recycled through the queue's
   free list, so the loop allocates nothing after the first few rounds. */
  void Run() {
    while (queue_->Pop(&item_)) {
      RunItem(&item_);
      queue_->FinishItem();
    }
  }

  /* Serial entry point (and the body of the parallel one). */
  void RunItem(WorkItem* item);

 private:
  /* Ensures the pool holds `count` slabs of the given shape. Cold path: hit
   once per worker and again whenever the Bézier order changes. */
  void EnsureSlabs(int count, int rows, int cols) {
    if (rows != slab_rows_ || cols != slab_cols_) {
      for (Eigen::MatrixXd& slab : slabs_) slab.resize(rows, cols);
      split_scratch_.resize(rows, cols);
      slab_rows_ = rows;
      slab_cols_ = cols;
    }
    while (static_cast<int>(slabs_.size()) < count) {
      slabs_.emplace_back(rows, cols);
    }
  }

  void EnsureArena(int size) {
    if (static_cast<int>(arena_.size()) < size) {
      arena_.resize(std::max(size, 2 * static_cast<int>(arena_.size()) + 64));
    }
  }

  const CertifierInput& input_;
  ThreadContext* context_{};
  FindingSink* sink_{};
  std::atomic<std::uint64_t>* node_counter_{};
  WorkQueue* queue_{};
  /* Non-null only for the lead worker, and only until it has hired. */
  Recruitment* recruit_{};
  GeometryCache geometry_;

  /* Reused buffers for the queue's two directions (see WorkItem). */
  WorkItem item_;
  WorkItem share_;

  std::vector<Eigen::MatrixXd> slabs_;
  Eigen::MatrixXd split_scratch_;
  int slab_rows_{-1};
  int slab_cols_{-1};
  std::vector<NodeFrame> stack_;
  std::vector<int> arena_;
  Eigen::VectorXd q_mid_;
  Eigen::VectorXd w_;
  Eigen::Vector3d nearest_a_;
  Eigen::Vector3d nearest_b_;
};

void Worker::RunItem(WorkItem* item) {
  const BezierSegment& segment = input_.path->segments()[item->segment];
  const MotionBoundTable& table = *input_.table;
  const DistanceOracle& oracle = *input_.oracle;
  const std::vector<PairRecord>& pairs = *input_.pairs;
  const std::vector<double>& tau = *input_.tau;
  const PrefilterTable& prefilter = *input_.prefilter;
  const double threshold = input_.options.margin;
  const double resolution = input_.options.distance_resolution;
  const int rows = static_cast<int>(item->control_points.rows());
  const int cols = static_cast<int>(item->control_points.cols());

  // Seed the local stack with this work item.
  EnsureSlabs(2, rows, cols);
  slabs_[0] = item->control_points;
  EnsureArena(static_cast<int>(item->active.size()) + 1);
  std::copy(item->active.begin(), item->active.end(), arena_.begin());
  stack_.clear();
  stack_.push_back(NodeFrame{item->s_lo, item->s_hi, 0,
                             static_cast<int>(item->active.size())});

  while (!stack_.empty()) {
    const NodeFrame frame = stack_.back();
    stack_.pop_back();
    const int k = static_cast<int>(stack_.size());

    // Branch-and-bound on time: a node starting at or after the earliest
    // witness known so far cannot contain an earlier one.
    if (TimeOf(segment, frame.s_lo) >= sink_->best_violation_time()) continue;
    node_counter_->fetch_add(1, std::memory_order_relaxed);

    // Lazy recruitment (see certifier.h): the lead worker runs alone until the
    // run has visited enough nodes to pay for helpers, then hires them once
    // and drops the hook. Every other worker carries a null `recruit_`.
    if (recruit_ != nullptr && ++recruit_->nodes >= kNodesBeforeHiringHelpers) {
      Recruitment* const recruitment = recruit_;
      recruit_ = nullptr;
      recruitment->hire();
    }

    EnsureSlabs(k + 2, rows, cols);
    const Eigen::MatrixXd& control_points = slabs_[k];

    // The split *is* the evaluation: the apex of the de Casteljau triangle at
    // the midpoint is exactly q(s_mid), so the node's representative
    // configuration comes for free.
    DeCasteljauSplitAtHalf(control_points, &slabs_[k + 1], &split_scratch_,
                           &q_mid_);

    // w_i = max_j |P_{j,i} − qc_i|. By the convex-hull property of the
    // Bernstein basis, |q_i(s) − qc_i| ≤ w_i for every s in this node.
    w_.setZero();
    for (int j = 0; j < cols; ++j) {
      for (int i = 0; i < rows; ++i) {
        w_[i] = std::max(w_[i], std::abs(control_points(i, j) - q_mid_[i]));
      }
    }

    // One FK per node; body poses and the query object are pulled lazily
    // below, and only for pairs that survive that far.
    context_->SetPositions(q_mid_);
    geometry_.NewConfiguration();
    const QueryObject<double>& query_object = context_->query_object();

    const double s_mid = 0.5 * (frame.s_lo + frame.s_hi);
    const double t_mid = TimeOf(segment, s_mid);
    // Hard floating-point backstop: once the midpoint no longer separates the
    // endpoints in double arithmetic the node cannot be split any further,
    // whatever the resolution says. Without it a pathologically small
    // resolution would spin forever.
    const bool fp_backstop = !(s_mid > frame.s_lo && s_mid < frame.s_hi);

    const int survivor_offset = frame.active_offset + frame.active_length;
    int survivor_count = 0;
    EnsureArena(survivor_offset + frame.active_length + 1);

    for (int e = frame.active_offset;
         e < frame.active_offset + frame.active_length; ++e) {
      const int p = arena_[e];
      const PairRecord& pair = pairs[p];
      const double tau_p = tau[p];
      // Δ_p(ν) = carveout_slack(p) + Σ_{j ∈ J(p)} λ(j,p)·w_j, a sparse dot
      // product over this pair's CSR row.
      const double travel = table.TravelBound(p, w_);
      const double motion_bound = table.carveout_slack(p) + travel;
      // The resolution floor is per pair and in meters: once this pair's
      // relative motion over the node is bounded by the requested resolution,
      // splitting further cannot decide it any better than the oracle
      // tolerance already allows, so the pair is decided here or reported as
      // inconclusive. Only the travel term is tested, because the carve-out
      // residual does not shrink with splitting.
      const bool at_floor = fp_backstop || travel <= resolution;

      // --- Early-out 1: the free-sphere prefilter. ---
      // ϕ_p ≥ ‖c_A − c_B‖ − ρ_A − ρ_B with the bounding spheres posed at qc,
      // so the lower bound stands in for ϕ̂ in the certificate test below and
      // is sound by the same displacement-lemma argument. It needs no
      // narrowphase and no allocation, only the lazily pulled body poses. It
      // is charged the same τ_p as the oracle even though it is exact given
      // the poses: that costs nothing (τ ~ 1e-6 m against centimetre-scale
      // sphere gaps) and keeps the arithmetic uniform.
      const int slot_a = prefilter.slot_a[p];
      const int slot_b = prefilter.slot_b[p];
      if (slot_a >= 0 && slot_b >= 0 &&
          IsCertified(geometry_.LowerBound(slot_a, slot_b), tau_p, motion_bound,
                      threshold)) {
        continue;
      }

      // --- Narrowphase. ----------------------------------------------------
      const double phi_hat =
          oracle.SignedDistance(query_object, pair, &nearest_a_, &nearest_b_);

      if (IsDefiniteViolation(phi_hat, tau_p, threshold)) {
        // qc is exactly on the trajectory (it is the de Casteljau apex), so
        // ϕ_true(qc) ≤ ϕ̂ + τ_p < m is a definite violation of the continuum
        // statement, not a sampling artifact.
        sink_->AddDefinite(
            MakeFinding(t_mid, q_mid_, pair, phi_hat, nearest_a_, nearest_b_));
        // A floor node has no children to refine into; otherwise keep p active
        // so the branch-and-bound recursion can refine the witness toward the
        // earliest violating time.
        if (at_floor) continue;
      } else if (IsCertified(phi_hat, tau_p, motion_bound, threshold)) {
        // Displacement lemma: for every s in this node,
        //   ϕ_p(q(s)) ≥ ϕ_true(qc) − Σ_{j∈J(p)} λ(j,p)·|q_j(s) − qc_j|
        //            ≥ (ϕ̂ − τ_p) − Δ_p(ν) > m + ε,
        // using |q_j(s) − qc_j| ≤ w_j from the convex-hull property. The whole
        // closed parameter interval of the node is therefore certified and the
        // pair drops out of the entire subtree, which is the dominant work
        // saver.
        continue;
      } else if (at_floor) {
        // --- Gray at the resolution floor. ---
        sink_->AddInconclusive(
            MakeFinding(t_mid, q_mid_, pair, phi_hat, nearest_a_, nearest_b_));
        continue;
      }

      arena_[survivor_offset + survivor_count] = p;
      ++survivor_count;
    }

    if (survivor_count == 0) continue;

    // At this point the split has left the *left* child in slab k+1 and the
    // *right* child in split_scratch_.
    const NodeFrame right{s_mid, frame.s_hi, survivor_offset, survivor_count};
    const NodeFrame left{frame.s_lo, s_mid, survivor_offset, survivor_count};
    if (queue_ != nullptr && queue_->ShouldShare()) {
      // Occupancy-driven sharing (see certifier.h): the shared queue is
      // running dry, so hand the right child over and carry on down the left
      // one. This is the only mechanism that spreads a deep tree, and because
      // it is driven by how hungry the other workers are rather than by depth,
      // it keeps spreading right down to the last subtree, which is exactly
      // what a fixed seeding depth cannot do.
      share_.segment = item->segment;
      share_.s_lo = right.s_lo;
      share_.s_hi = right.s_hi;
      share_.control_points = split_scratch_;
      share_.active.assign(arena_.begin() + survivor_offset,
                           arena_.begin() + survivor_offset + survivor_count);
      queue_->Push(&share_);
      // The frame at stack index k must own slab k, so move the left child
      // down into it (an O(1) Eigen buffer swap, like the split itself).
      slabs_[k].swap(slabs_[k + 1]);
      stack_.push_back(left);  // slab k holds the left child.
    } else {
      slabs_[k].swap(split_scratch_);  // O(1): slab k = right child.
      // LIFO with the left child on top => a left-to-right sweep in time, so
      // the serial driver walks the trajectory in order.
      stack_.push_back(right);  // slab k holds the right child.
      stack_.push_back(left);   // slab k+1 holds the left child.
    }
  }
}

/* Evaluates one breakpoint configuration against every pair. Breakpoints are
 the finitely many configurations the midpoint recursion only approaches in the
 limit (t0, every junction, tf), so checking them discretely is what gives
 violations *at* interval endpoints clean semantics.

 When `resolve_static` is true (the t0 breakpoint) the pairs with J(p) = ∅ are
 also resolved here, once and for all: no motion of the trajectory can change
 their relative pose, so their status at q(t0) is their status everywhere. */
void RunBreakpointPass(const CertifierInput& input, ThreadContext* context,
                       GeometryCache* geometry, const Eigen::VectorXd& q,
                       double time, bool resolve_static, FindingSink* sink) {
  const std::vector<PairRecord>& pairs = *input.pairs;
  const std::vector<double>& tau = *input.tau;
  const PrefilterTable& prefilter = *input.prefilter;
  const MotionBoundTable& table = *input.table;
  const double threshold = input.options.margin;

  context->SetPositions(q);
  geometry->NewConfiguration();
  const QueryObject<double>& query_object = context->query_object();

  Eigen::Vector3d nearest_a;
  Eigen::Vector3d nearest_b;
  for (int p = 0; p < static_cast<int>(pairs.size()); ++p) {
    const PairRecord& pair = pairs[p];
    const double tau_p = tau[p];
    const bool is_static = table.pair_is_static(p);
    // Δ_p for a static pair: J(p) is empty, so the sparse dot product is empty
    // and MotionBound() collapses to the pair's carve-out slack whatever w is.
    // That slack is normally exactly 0, and "static" then means genuinely
    // immobile, but a pair whose whole J_topo(p) was carved out on a
    // *tolerance* can still drift by that much, and the discrete test below
    // has to charge it or the carved coordinates' residual would go
    // unaccounted for on exactly the pairs made entirely of them.
    const double static_bound = table.carveout_slack(p);
    // A static pair's clearance is the same at every configuration of the
    // trajectory, so the t0 pass settles it for good; re-testing it at every
    // junction would only pay a narrowphase query per junction.
    if (is_static && !resolve_static) continue;

    double lower_bound = -kInfinity;
    const int slot_a = prefilter.slot_a[p];
    const int slot_b = prefilter.slot_b[p];
    if (slot_a >= 0 && slot_b >= 0) {
      lower_bound = geometry->LowerBound(slot_a, slot_b);
    }

    if (is_static) {
      // Δ_p is the constant `static_bound` for a static pair, so the node
      // certificate degenerates to a single discrete test that holds for the
      // whole domain.
      if (IsCertified(lower_bound, tau_p, static_bound, threshold)) continue;
    } else if (lower_bound >= threshold) {
      // A definite violation needs ϕ̂ + τ_p < m, and ϕ̂ ≥ ϕ_true − τ_p ≥
      // lower_bound − τ_p, so lower_bound ≥ m rules one out with no query.
      continue;
    }

    const double phi_hat = input.oracle->SignedDistance(query_object, pair,
                                                        &nearest_a, &nearest_b);

    if (IsDefiniteViolation(phi_hat, tau_p, threshold)) {
      sink->AddDefinite(
          MakeFinding(time, q, pair, phi_hat, nearest_a, nearest_b));
      continue;
    }
    if (!is_static) continue;
    if (IsCertified(phi_hat, tau_p, static_bound, threshold)) continue;
    // Neither certified nor violating, and no subdivision can help: this
    // pair's clearance is constant along the trajectory (up to the carve-out
    // residual) and sits within oracle tolerance of the threshold.
    sink->AddInconclusive(
        MakeFinding(time, q, pair, phi_hat, nearest_a, nearest_b));
  }
}

}  // namespace

void ThreadContext::SetPositions(const Eigen::VectorXd& q) {
  model_->plant().SetPositions(&context_.mutable_plant_context(), q);
}

const RigidTransformd& ThreadContext::EvalBodyPose(BodyIndex body) const {
  return model_->plant().EvalBodyPoseInWorld(context_.plant_context(),
                                             model_->plant().get_body(body));
}

ContextPool::ContextPool(const drake::planning::RobotDiagram<double>& model,
                         int initial_size)
    : model_(&model) {
  for (int i = 0; i < std::max(1, initial_size); ++i) {
    slots_.push_back(std::make_unique<ThreadContext>(model));
    in_use_.push_back(false);
  }
}

ContextPool::Lease ContextPool::Acquire(int count) const {
  DRAKE_DEMAND(count >= 1);
  std::vector<ThreadContext*> contexts;
  std::vector<int> slots;
  contexts.reserve(count);
  slots.reserve(count);
  std::lock_guard<std::mutex> guard(mutex_);
  for (int i = 0; i < static_cast<int>(slots_.size()) &&
                  static_cast<int>(slots.size()) < count;
       ++i) {
    if (!in_use_[i]) {
      in_use_[i] = true;
      slots.push_back(i);
      contexts.push_back(slots_[i].get());
    }
  }
  while (static_cast<int>(slots.size()) < count) {
    slots_.push_back(std::make_unique<ThreadContext>(*model_));
    in_use_.push_back(true);
    slots.push_back(static_cast<int>(slots_.size()) - 1);
    contexts.push_back(slots_.back().get());
  }
  return Lease(this, std::move(contexts), std::move(slots));
}

void ContextPool::Release(const std::vector<int>& slots) const {
  std::lock_guard<std::mutex> guard(mutex_);
  for (const int slot : slots) in_use_[slot] = false;
}

ContextPool::Lease& ContextPool::Lease::operator=(Lease&& other) noexcept {
  if (this == &other) return *this;
  if (pool_ != nullptr && !slots_.empty()) pool_->Release(slots_);
  pool_ = other.pool_;
  contexts_ = std::move(other.contexts_);
  slots_ = std::move(other.slots_);
  other.pool_ = nullptr;
  other.contexts_.clear();
  other.slots_.clear();
  return *this;
}

ContextPool::Lease::~Lease() {
  if (pool_ != nullptr && !slots_.empty()) pool_->Release(slots_);
}

Result RunCertifier(const CertifierInput& input, ContextPool* pool) {
  DRAKE_DEMAND(input.oracle != nullptr);
  DRAKE_DEMAND(input.table != nullptr);
  DRAKE_DEMAND(input.path != nullptr);
  DRAKE_DEMAND(input.pairs != nullptr);
  DRAKE_DEMAND(input.tau != nullptr);
  DRAKE_DEMAND(input.prefilter != nullptr);
  DRAKE_DEMAND(pool != nullptr);

  const PiecewiseBezierPath& path = *input.path;
  const int num_pairs = static_cast<int>(input.pairs->size());
  const int num_segments = static_cast<int>(path.segments().size());

  FindingSink sink;
  std::atomic<std::uint64_t> node_counter{0};

  // Bounding the width by the machine's keeps a program that runs many
  // concurrent parallel checks from multiplying threads without limit; the
  // bound comes from Parallelism::Max() rather than hardware_concurrency()
  // directly, so it honours DRAKE_NUM_THREADS like the rest of Drake.
  const int num_threads =
      std::min(std::max(1, input.options.parallelism.num_threads()),
               Parallelism::Max().num_threads());
  // Only the lead worker's context is leased up front. Helpers lease theirs
  // when (if) they are hired, so a small check under the default
  // Parallelism::Max() never pays for sixteen leases it will not use.
  ContextPool::Lease lease = pool->Acquire(1);

  // --- Steps 1 and 2: breakpoints and static pairs (serial, O(#segments)). --
  if (num_segments > 0 && num_pairs > 0) {
    GeometryCache geometry(*input.prefilter, lease[0]);
    for (int k = 0; k <= num_segments; ++k) {
      // Segment k's start, plus the last segment's end. At a junction the two
      // sides are the same physical configuration (they may differ by 2πk in a
      // continuous-revolute coordinate, which forward kinematics ignores), so
      // one evaluation per breakpoint suffices. Endpoints are Bézier control
      // points, so they are exact; no curve evaluation needed.
      const Eigen::VectorXd q =
          (k < num_segments)
              ? Eigen::VectorXd(path.segments()[k].control_points.col(0))
              : Eigen::VectorXd(
                    path.segments()[k - 1].control_points.rightCols(1));
      const double time = (k < num_segments) ? path.segments()[k].t_start
                                             : path.segments()[k - 1].t_end;
      RunBreakpointPass(input, &lease[0], &geometry, q, time,
                        /* resolve_static = */ k == 0, &sink);
    }
  }

  // --- Step 3: the adaptive recursion over every segment. ------------------
  std::vector<int> moving_pairs;
  moving_pairs.reserve(num_pairs);
  for (int p = 0; p < num_pairs; ++p) {
    if (!input.table->pair_is_static(p)) moving_pairs.push_back(p);
  }

  if (!moving_pairs.empty() && num_segments > 0 && num_threads <= 1) {
    // Serial: one worker, one local stack, no shared queue and no thread
    // interleaving => bit-deterministic results.
    Worker worker(input, &lease[0], &sink, &node_counter, nullptr, nullptr);
    for (int k = 0; k < num_segments; ++k) {
      WorkItem item;
      item.segment = k;
      item.control_points = path.segments()[k].control_points;
      item.active = moving_pairs;
      worker.RunItem(&item);
    }
  } else if (!moving_pairs.empty() && num_segments > 0) {
    // Parallel driver: lazy recruitment + occupancy-driven sharing. The full
    // rationale, and why static seeding is not used, is documented on
    // RunCertifier() in certifier.h.
    WorkQueue queue;
    {
      // Seeded in reverse so the LIFO hands segment 0 out first. Before any
      // helper exists that reproduces the serial left-to-right sweep exactly,
      // and once helpers arrive it still lets the earliest-violation bound
      // tighten from the front of the trajectory.
      WorkItem seed;
      for (int k = num_segments - 1; k >= 0; --k) {
        seed.segment = k;
        seed.s_lo = 0.0;
        seed.s_hi = 1.0;
        seed.control_points = path.segments()[k].control_points;
        seed.active = moving_pairs;
        queue.Push(&seed);
      }
    }

    // The oracle is documented to throw, and any allocation can. A worker that
    // let an exception escape would terminate the process, and, because it
    // would skip WorkQueue::FinishItem(), would also strand every other
    // worker in Pop(). So every worker catches, aborts the work source, and
    // the first exception is rethrown once all of them have finished.
    std::exception_ptr first_error;
    std::mutex error_mutex;
    const auto record_error = [&]() {
      std::lock_guard<std::mutex> guard(error_mutex);
      if (first_error == nullptr) first_error = std::current_exception();
    };

    // The futures are declared last so that they are destroyed, and therefore
    // waited on, before the workers, contexts and lease their tasks reference,
    // on every path including the throwing one.
    std::optional<ContextPool::Lease> helper_lease;
    std::vector<std::unique_ptr<Worker>> helpers;
    std::vector<std::future<void>> helper_futures;

    Recruitment recruitment;
    // Hiring is a per-call cold path: it runs at most once per check, only
    // after the run has proved itself worth spreading, and it is the only
    // place in the driver that allocates or creates a thread once the node
    // loop is turning; only the steady state is allocation-free.
    recruitment.hire = [&]() {
      const int hired = num_threads - 1;
      if (hired <= 0) return;
      helper_lease.emplace(pool->Acquire(hired));
      helpers.reserve(hired);
      for (int i = 0; i < hired; ++i) {
        helpers.push_back(std::make_unique<Worker>(
            input, &(*helper_lease)[i], &sink, &node_counter, &queue, nullptr));
      }
      // Every consumer of the queue, the lead included: this count is the
      // occupancy target of the sharing policy, and setting it from zero is
      // what switches sharing on.
      queue.set_num_workers(hired + 1);
      helper_futures.reserve(hired);
      for (int i = 0; i < hired; ++i) {
        // std::async throws std::system_error when the system refuses a
        // thread. The helpers that did start are still joined below, and the
        // lead's catch turns the refusal into the same aborted run any other
        // throw out of the node loop produces.
        helper_futures.push_back(std::async(std::launch::async, [&, i]() {
          try {
            helpers[i]->Run();
          } catch (...) {
            record_error();
            queue.Abort();
          }
        }));
      }
    };

    Worker lead(input, &lease[0], &sink, &node_counter, &queue, &recruitment);
    try {
      lead.Run();
    } catch (...) {
      record_error();
      queue.Abort();
    }
    // The helper tasks swallow their own exceptions into `first_error`, so
    // get() here is a join and never throws.
    for (std::future<void>& helper : helper_futures) helper.get();
    if (first_error != nullptr) std::rethrow_exception(first_error);
  }

  Result result;
  result.num_nodes = node_counter.load(std::memory_order_relaxed);
  if (sink.definite().has_value()) {
    // The branch-and-bound recursion refines toward the earliest witness, so
    // this *is* the earliest witness, identical serially and in parallel.
    result.verdict = Verdict::kViolationFound;
    result.finding = std::move(sink.definite());
  } else if (sink.inconclusive().has_value()) {
    result.verdict = Verdict::kInconclusive;
    result.finding = std::move(sink.inconclusive());
  } else {
    result.verdict = Verdict::kCertifiedFree;
  }
  return result;
}

}  // namespace internal
}  // namespace continuous_collision
}  // namespace planning
}  // namespace drake
