#pragma once

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/trajectories/bezier_curve.h"
#include "drake/common/trajectories/composite_trajectory.h"
#include "drake/geometry/optimization/convex_set.h"
#include "drake/geometry/optimization/graph_of_convex_sets.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace planning {
namespace trajectory_optimization {

/**
GcsTrajectoryOptimization implements a simplified motion planning optimization
problem introduced in the paper ["Motion Planning around Obstacles with Convex
Optimization"](https://arxiv.org/abs/2205.04422) by Tobia Marcucci, Mark
Petersen, David von Wrangel, Russ Tedrake.

Instead of using the full time-scaling curve, this problem uses a single
time-scaling variable for each region. This formulation yields continuous
trajectories, which are not differentiable at the transition times between the
regions since non-convex continuity constraints are not supported yet. However,
it supports continuity on the path r(s) for arbitrary degree. The path
r(s) can be reconstructed from the gcs solution q(t) with
`NormalizeSegmentTimes()` and post-processed with e.g. Toppra to enforce
acceleration bounds.

The ith piece of the composite trajectory is defined as q(t) = r((t - tᵢ) /
hᵢ). r : [0, 1] → ℝⁿ is a the path, parametrized as a Bézier curve with order
n. tᵢ and hᵢ are the initial time and duration of the ith sub-trajectory.

This class supports the notion of a Subgraph of regions. This has proven useful
to facilitate multi-modal motion planning such as: Subgraph A: find a
collision-free trajectory for the robot to a grasping posture, Subgraph B: find
a collision-free trajectory for the robot with the object in its hand to a
placing posture, etc.
*/
class GcsTrajectoryOptimization final {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GcsTrajectoryOptimization);

  /** Constructs the motion planning problem.
  @param num_positions is the dimension of the configuration space.
  @param continuous_revolute_joints is a list of indices corresponding to
  continuous revolute joints, i.e., revolute joints which don't have any joint
  limits, and hence "wrap around" at 2π. Each entry in
  continuous_revolute_joints must be non-negative, less than num_positions, and
  unique. This feature is currently only supported within a single subgraph:
  continuous revolute joints won't be taken into account when constructing edges
  between subgraphs or checking if sets intersect through a subspace.
  */
  explicit GcsTrajectoryOptimization(
      int num_positions,
      std::vector<int> continuous_revolute_joints = std::vector<int>());

  ~GcsTrajectoryOptimization();

  /** A Subgraph is a subset of the larger graph. It is defined by a set of
  regions and edges between them based on intersection. From an API standpoint,
  a Subgraph is useful to define a multi-modal motion planning problem. Further,
  it allows different constraints and objects to be added to different
  subgraphs. Note that the the GraphOfConvexSets does not differentiate between
  subgraphs and can't be mixed with other instances of
  GcsTrajectoryOptimization.
  */
  class Subgraph final {
   public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Subgraph);

    ~Subgraph();

    /** Returns the name of the subgraph. */
    const std::string& name() const { return name_; }

    /** Returns the order of the Bézier trajectory within the region. */
    int order() const { return order_; }

    /** Returns the number of vertices in the subgraph. */
    int size() const { return vertices_.size(); }

    /** Returns the regions associated with this subgraph before the
    CartesianProduct. */
    const geometry::optimization::ConvexSets& regions() const {
      return regions_;
    }

    /** Adds a minimum time cost to all regions in the subgraph. The cost is the
    sum of the time scaling variables.
    @param weight is the relative weight of the cost.
    */
    void AddTimeCost(double weight = 1.0);

    /** Adds multiple L2Norm Costs on the upper bound of the path length.
    Since we cannot directly compute the path length of a Bézier curve, we
    minimize the upper bound of the path integral by minimizing the sum of
    distances between control points. For Bézier curves, this is equivalent to
    the sum of the L2Norm of the derivative control points of the curve divided
    by the order.

    @param weight_matrix is the relative weight of each component for the cost.
    The diagonal of the matrix is the weight for each dimension. The
    off-diagonal elements are the weight for the cross terms, which can be used
    to penalize diagonal movement.
    @pre weight_matrix must be of size num_positions() x num_positions().
    */
    void AddPathLengthCost(const Eigen::MatrixXd& weight_matrix);

    /** Adds multiple L2Norm Costs on the upper bound of the path length.
    We upper bound the trajectory length by the sum of the distances between
    control points. For Bézier curves, this is equivalent to the sum
    of the L2Norm of the derivative control points of the curve divided by the
    order.

    @param weight is the relative weight of the cost.
    */
    void AddPathLengthCost(double weight = 1.0);

    /** Adds a linear velocity constraint to the subgraph `lb` ≤ q̇(t) ≤
    `ub`.
    @param lb is the lower bound of the velocity.
    @param ub is the upper bound of the velocity.

    @throws std::exception if subgraph order is zero, since the velocity is
    defined as the derivative of the Bézier curve.
    @throws std::exception if lb or ub are not of size num_positions().
    */
    void AddVelocityBounds(const Eigen::Ref<const Eigen::VectorXd>& lb,
                           const Eigen::Ref<const Eigen::VectorXd>& ub);

    /** Enforces derivative continuity constraints on the subgraph.
     @param continuity_order is the order of the continuity constraint.

    Note that the constraints are on the control points of the
    derivatives of r(s) and not q(t). This may result in discontinuities of the
    trajectory return by `SolvePath()` since the r(s) will get rescaled by the
    duration h to yield q(t). `NormalizeSegmentTimes()` will return r(s) with
    valid continuity.

    @throws std::exception if the continuity order is not equal or less than
        the order the subgraphs.
    @throws std::exception if the continuity order is less than one since path
    continuity is enforced by default.
    */
    void AddPathContinuityConstraints(int continuity_order);

   private:
    /* Constructs a new subgraph and copies the regions. */
    Subgraph(const geometry::optimization::ConvexSets& regions,
             const std::vector<std::pair<int, int>>& regions_to_connect,
             int order, double h_min, double h_max, std::string name,
             GcsTrajectoryOptimization* traj_opt,
             std::optional<const std::vector<Eigen::VectorXd>> edge_offsets);

    /* Convenience accessor, for brevity. */
    int num_positions() const { return traj_opt_.num_positions(); }

    /* Convenience accessor, for brevity. */
    const std::vector<int>& continuous_revolute_joints() const {
      return traj_opt_.continuous_revolute_joints();
    }

    /* Throw an error if any convex set in regions violates the convexity
    radius. */
    void ThrowsForInvalidConvexityRadius() const;

    /* Extracts the control points variables from a vertex. */
    Eigen::Map<const MatrixX<symbolic::Variable>> GetControlPoints(
        const geometry::optimization::GraphOfConvexSets::Vertex& v) const;

    /* Extracts the time scaling variable from a vertex. */
    symbolic::Variable GetTimeScaling(
        const geometry::optimization::GraphOfConvexSets::Vertex& v) const;

    const geometry::optimization::ConvexSets regions_;
    const int order_;
    const double h_min_;
    const std::string name_;
    GcsTrajectoryOptimization& traj_opt_;

    std::vector<geometry::optimization::GraphOfConvexSets::Vertex*> vertices_;
    std::vector<geometry::optimization::GraphOfConvexSets::Edge*> edges_;

    // r(s) is a BezierCurve of the right shape and order, which can be used to
    // design costs and constraints for the underlying vertices and edges.
    trajectories::BezierCurve<double> r_trajectory_;

    friend class GcsTrajectoryOptimization;
  };

  /** EdgesBetweenSubgraphs are defined as the connecting edges between two
  given subgraphs. These edges are a subset of the many other edges in the
  larger graph. From an API standpoint, EdgesBetweenSubgraphs enable transitions
  between Subgraphs, which can enable transitions between modes. Further, it
  allows different constraints to be added in the transition between subgraphs.
  Note that the EdgesBetweenSubgraphs can't be separated from the actual edges
  in the GraphOfConvexSets framework, thus mixing it with other instances of
  GCSTrajetoryOptimization is not supported.
  */
  class EdgesBetweenSubgraphs final {
   public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(EdgesBetweenSubgraphs);

    ~EdgesBetweenSubgraphs();

    /** Adds a linear velocity constraint to the control point connecting the
    subgraphs `lb` ≤ q̇(t) ≤ `ub`.
    @param lb is the lower bound of the velocity.
    @param ub is the upper bound of the velocity.

    @throws std::exception if both subgraphs order is zero, since the velocity
    is defined as the derivative of the Bézier curve. At least one of the
    subgraphs must have an order of at least 1.
    @throws std::exception if lb or ub are not of size num_positions().
    */
    void AddVelocityBounds(const Eigen::Ref<const Eigen::VectorXd>& lb,
                           const Eigen::Ref<const Eigen::VectorXd>& ub);

    /** Enforces derivative continuity constraints on the edges between the
    subgraphs.
     @param continuity_order is the order of the continuity constraint.

    Note that the constraints are on the control points of the
    derivatives of r(s) and not q(t). This may result in discontinuities of the
    trajectory return by `SolvePath()` since the r(s) will get rescaled by the
    duration h to yield q(t). `NormalizeSegmentTimes()` will return r(s) with
    valid continuity.

    @throws std::exception if the continuity order is not equal or less than
        the order of both subgraphs.
    @throws std::exception if the continuity order is less than one since path
    continuity is enforced by default.
    */
    void AddPathContinuityConstraints(int continuity_order);

   private:
    EdgesBetweenSubgraphs(const Subgraph& from_subgraph,
                          const Subgraph& to_subgraph,
                          const geometry::optimization::ConvexSet* subspace,
                          GcsTrajectoryOptimization* traj_opt);

    /* Convenience accessor, for brevity. */
    int num_positions() const { return traj_opt_.num_positions(); }

    /* Convenience accessor, for brevity. */
    const std::vector<int>& continuous_revolute_joints() const {
      return traj_opt_.continuous_revolute_joints();
    }

    bool RegionsConnectThroughSubspace(
        const geometry::optimization::ConvexSet& A,
        const geometry::optimization::ConvexSet& B,
        const geometry::optimization::ConvexSet& subspace);

    /* Extracts the control points variables from an edge. */
    Eigen::Map<const MatrixX<symbolic::Variable>> GetControlPointsU(
        const geometry::optimization::GraphOfConvexSets::Edge& e) const;

    /* Extracts the control points variables from an edge. */
    Eigen::Map<const MatrixX<symbolic::Variable>> GetControlPointsV(
        const geometry::optimization::GraphOfConvexSets::Edge& e) const;

    /* Extracts the time scaling variable from a edge. */
    symbolic::Variable GetTimeScalingU(
        const geometry::optimization::GraphOfConvexSets::Edge& e) const;

    /* Extracts the time scaling variable from a edge. */
    symbolic::Variable GetTimeScalingV(
        const geometry::optimization::GraphOfConvexSets::Edge& e) const;

    GcsTrajectoryOptimization& traj_opt_;
    const int from_subgraph_order_;
    const int to_subgraph_order_;

    trajectories::BezierCurve<double> ur_trajectory_;
    trajectories::BezierCurve<double> vr_trajectory_;

    std::vector<geometry::optimization::GraphOfConvexSets::Edge*> edges_;

    friend class GcsTrajectoryOptimization;
  };

  /** Returns the number of position variables. */
  int num_positions() const { return num_positions_; }

  /** Returns a list of indices corresponding to continuous revolute joints. */
  const std::vector<int>& continuous_revolute_joints() {
    return continuous_revolute_joints_;
  }

  /** Returns a Graphviz string describing the graph vertices and edges.  If
  `results` is supplied, then the graph will be annotated with the solution
  values.
  @param show_slacks determines whether the values of the intermediate
  (slack) variables are also displayed in the graph.
  @param precision sets the floating point precision (how many digits are
  generated) of the annotations.
  @param scientific sets the floating point formatting to scientific (if true)
  or fixed (if false).
  */
  std::string GetGraphvizString(
      const std::optional<solvers::MathematicalProgramResult>& result =
          std::nullopt,
      bool show_slack = true, int precision = 3,
      bool scientific = false) const {
    return gcs_.GetGraphvizString(result, show_slack, precision, scientific);
  }

  /** Creates a Subgraph with the given regions and indices.
  @param regions represent the valid set a control point can be in. We retain a
  copy of the regions since other functions may access them. If any of the
  positions represent revolute joints without limits, each region has a maximum
  width of strictly less than π along dimensions corresponding to those joints.
  @param edges_between_regions is a list of pairs of indices into the regions
  vector. For each pair representing an edge between two regions, an edge is
  added within the subgraph. Note that the edges are directed so (i,j) will only
  add an edge from region i to region j.
  @param order is the order of the Bézier curve.
  @param h_max is the maximum duration to spend in a region (seconds). Some
  solvers struggle numerically with large values.
  @param h_min is the minimum duration to spend in a region (seconds) if that
  region is visited on the optimal path. Some cost and constraints are only
  convex for h > 0. For example the perspective quadratic cost of the path
  energy ||ṙ(s)||² / h becomes non-convex for h = 0. Otherwise h_min can be set
  to 0.
  @param name is the name of the subgraph. If the passed name is an empty
  string, a default name will be provided.
  @param edge_offsets is an optional list of vectors. If defined, the list must
  contain the same number of entries as edges_between_regions. In other words,
  if defined, there must be one edge offset for each specified edge. For each
  pair of sets listed in edges_between_regions, the first set is translated (in
  configuration space) by the corresponding vector in edge_offsets before
  computing the constraints associated to that edge. This is used to add edges
  between sets that "wrap around" 2π along some dimension, due to, e.g., a
  continuous revolute joint. This edge offset corresponds to the translation
  component of the affine map τ_uv in equation (11) of "Non-Euclidean Motion
  Planning with Graphs of Geodesically-Convex Sets", and per the discussion in
  Subsection VI A, τ_uv has no rotation component.
  */
  Subgraph& AddRegions(
      const geometry::optimization::ConvexSets& regions,
      const std::vector<std::pair<int, int>>& edges_between_regions, int order,
      double h_min = 0, double h_max = 20, std::string name = "",
      std::optional<const std::vector<Eigen::VectorXd>> edge_offsets =
          std::nullopt);

  /** Creates a Subgraph with the given regions.
  This function will compute the edges between the regions based on the set
  intersections.
  @param regions represent the valid set a control point can be in. We retain a
  copy of the regions since other functions may access them. If any of the
  positions represent continuous revolute joints, each region must have a
  maximum width of strictly less than π along dimensions corresponding to those
  joints.
  @param order is the order of the Bézier curve.
  @param h_min is the minimum duration to spend in a region (seconds) if that
  region is visited on the optimal path. Some cost and constraints are only
  convex for h > 0. For example the perspective quadratic cost of the path
  energy ||ṙ(s)||² / h becomes non-convex for h = 0. Otherwise h_min can be set
  to 0.
  @param h_max is the maximum duration to spend in a region (seconds). Some
  solvers struggle numerically with large values.
  @param name is the name of the subgraph. A default name will be provided.
  @throws std::exception if any of the regions has a width of π or greater along
  dimensions corresponding to continuous revolute joints.
  */
  Subgraph& AddRegions(const geometry::optimization::ConvexSets& regions,
                       int order, double h_min = 0, double h_max = 20,
                       std::string name = "");

  /** Connects two subgraphs with directed edges.
  @param from_subgraph is the subgraph to connect from. Must have been created
  from a call to AddRegions() on this object, not some other optimization
  program.
  @param to_subgraph is the subgraph to connect to. Must have been created from
  a call to AddRegions() on this object, not some other optimization program.
  @param subspace is the subspace that the connecting control points must be in.
  Subspace is optional. Only edges that connect through the subspace will be
  added, and the subspace is added as a constraint on the connecting control
  points. Subspaces of type point or HPolyhedron are supported since other sets
  require constraints that are not yet supported by the GraphOfConvexSets::Edge
  constraint, e.g., set containment of a HyperEllipsoid is formulated via
  LorentzCone constraints. Workaround: Create a subgraph of zero order with the
  subspace as the region and connect it between the two subgraphs. This works
  because GraphOfConvexSet::Vertex , supports arbitrary instances of ConvexSets.
  */
  EdgesBetweenSubgraphs& AddEdges(
      const Subgraph& from_subgraph, const Subgraph& to_subgraph,
      const geometry::optimization::ConvexSet* subspace = nullptr);

  /** Adds a minimum time cost to all regions in the whole graph. The cost is
  the sum of the time scaling variables.

  This cost will be added to the entire graph. Note that this cost
  will be applied even to subgraphs added in the future.

  @param weight is the relative weight of the cost.
  */
  void AddTimeCost(double weight = 1.0);

  /** Adds multiple L2Norm Costs on the upper bound of the path length.
  Since we cannot directly compute the path length of a Bézier curve, we
  minimize the upper bound of the path integral by minimizing the sum of
  (weighted) distances between control points: ∑ |weight_matrix * (rᵢ₊₁ − rᵢ)|₂.

  This cost will be added to the entire graph. Since the path length is only
  defined for Bézier curves that have two or more control points, this cost will
  only added to all subgraphs with order greater than zero. Note that this cost
  will be applied even to subgraphs added in the future.

  @param weight_matrix is the relative weight of each component for the cost.
  The diagonal of the matrix is the weight for each dimension. The
  off-diagonal elements are the weight for the cross terms, which can be used
  to penalize diagonal movement.
  @pre weight_matrix must be of size num_positions() x num_positions().
  */
  void AddPathLengthCost(const Eigen::MatrixXd& weight_matrix);

  /** Adds multiple L2Norm Costs on the upper bound of the path length.
  Since we cannot directly compute the path length of a Bézier curve, we
  minimize the upper bound of the path integral by minimizing the sum of
  distances between control points. For Bézier curves, this is equivalent to the
  sum of the L2Norm of the derivative control points of the curve divided by the
  order.

  This cost will be added to the entire graph. Since the path length is only
  defined for Bézier curves that have two or more control points, this cost will
  only added to all subgraphs with order greater than zero. Note that this cost
  will be applied even to subgraphs added in the future.

  @param weight is the relative weight of the cost.
  */
  void AddPathLengthCost(double weight = 1.0);

  /** Adds a linear velocity constraint to the entire graph `lb` ≤ q̇(t) ≤
  `ub`.
  @param lb is the lower bound of the velocity.
  @param ub is the upper bound of the velocity.

  This constraint will be added to the entire graph. Since the velocity requires
  forming the derivative of the Bézier curve, this constraint will only added to
  all subgraphs with order greater than zero. Note that this constraint will be
  applied even to subgraphs added in the future.

  @throws std::exception if lb or ub are not of size num_positions().
  */
  void AddVelocityBounds(const Eigen::Ref<const Eigen::VectorXd>& lb,
                         const Eigen::Ref<const Eigen::VectorXd>& ub);

  /** Enforces derivative continuity constraints on the entire graph.
  @param continuity_order is the order of the continuity constraint.

  Note that the constraints are on the control points of the
  derivatives of r(s) and not q(t). This may result in discontinuities of the
  trajectory return by `SolvePath()` since the r(s) will get rescaled by the
  duration h to yield q(t). `NormalizeSegmentTimes()` will return r(s) with
  valid continuity.

  @throws std::exception if the continuity order is less than one since path
  continuity is enforced by default.
  */
  void AddPathContinuityConstraints(int continuity_order);

  /** Formulates and solves the mixed-integer convex formulation of the
  shortest path problem on the whole graph. @see
  `geometry::optimization::GraphOfConvexSets::SolveShortestPath()` for further
  details.

  @param source specifies the source subgraph. Must have been created from a
  call to AddRegions() on this object, not some other optimization program. If
  the source is a subgraph with more than one region, an empty set will be
  added and optimizer will choose the best region to start in. To start in a
  particular point, consider adding a subgraph of order zero with a single
  region of type Point.
  @param target specifies the target subgraph. Must have been created from a
  call to AddRegions() on this object, not some other optimization program. If
  the target is a subgraph with more than one region, an empty set will be
  added and optimizer will choose the best region to end in. To end in a
  particular point, consider adding a subgraph of order zero with a single
  region of type Point.
  @param options include all settings for solving the shortest path problem.
  The following default options will be used if they are not provided in
  `options`:
  - `options.convex_relaxation = true`,
  - `options.max_rounded_paths = 5`,
  - `options.preprocessing = true`.

  @see `geometry::optimization::GraphOfConvexSetsOptions` for further details.
  */
  std::pair<trajectories::CompositeTrajectory<double>,
            solvers::MathematicalProgramResult>
  SolvePath(
      const Subgraph& source, const Subgraph& target,
      const geometry::optimization::GraphOfConvexSetsOptions& options = {});

  /** Provide a heuristic estimate of the complexity of the underlying
  GCS mathematical program, for regression testing purposes.
  Here we sum the total number of variable appearances in our costs and
  constraints as a rough approximation of the complexity of the subproblems. */
  double EstimateComplexity() const;

  /** Getter for the underlying GraphOfConvexSets. This is intended primarily
  for inspecting the resulting programs. */
  const geometry::optimization::GraphOfConvexSets& graph_of_convex_sets()
      const {
    return gcs_;
  }

  /** Normalizes each trajectory segment to one second in duration.
  Reconstructs the path r(s) from the solution trajectory q(t) of
  `SolvePath()` s.t. each segment of the resulting trajectory
  will be one second long. The start time will match the original start time.
  @param trajectory The solution trajectory returned by `SolvePath()`.

  @throws std::exception if not all trajectory segments of the
  CompositeTrajectory are of type BezierCurve<double>
  */
  static trajectories::CompositeTrajectory<double> NormalizeSegmentTimes(
      const trajectories::CompositeTrajectory<double>& trajectory);

 private:
  const int num_positions_;
  const std::vector<int> continuous_revolute_joints_;

  // Adds a Edge to gcs_ with the name "{u.name} -> {v.name}".
  geometry::optimization::GraphOfConvexSets::Edge* AddEdge(
      geometry::optimization::GraphOfConvexSets::Vertex* u,
      geometry::optimization::GraphOfConvexSets::Vertex* v);

  geometry::optimization::GraphOfConvexSets gcs_;

  // Store the subgraphs by reference.
  std::vector<std::unique_ptr<Subgraph>> subgraphs_;
  std::vector<std::unique_ptr<EdgesBetweenSubgraphs>> subgraph_edges_;
  std::map<const geometry::optimization::GraphOfConvexSets::Vertex*, Subgraph*>
      vertex_to_subgraph_;
  std::vector<double> global_time_costs_;
  std::vector<Eigen::MatrixXd> global_path_length_costs_;
  std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>
      global_velocity_bounds_{};
  std::vector<int> global_continuity_constraints_{};
};

/** Returns a list of indices in the plant's generalized positions which
correspond to a continuous revolute joint (a revolute joint with no joint
limits). This includes the revolute component of a planar joint */
std::vector<int> GetContinuousRevoluteJointIndices(
    const multibody::MultibodyPlant<double>& plant);

}  // namespace trajectory_optimization
}  // namespace planning
}  // namespace drake
