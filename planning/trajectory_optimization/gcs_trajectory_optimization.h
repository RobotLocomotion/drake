#pragma once

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/trajectories/bezier_curve.h"
#include "drake/common/trajectories/composite_trajectory.h"
#include "drake/geometry/optimization/convex_set.h"
#include "drake/geometry/optimization/graph_of_convex_sets.h"

namespace drake {
namespace planning {
namespace trajectory_optimization {

/**
GCSTrajectoryOptimization implements a simplified motion planning optimization
problem introduced in the paper "Motion Planning around Obstacles with Convex
Optimization."

"Motion Planning around Obstacles with Convex Optimization" by Tobia Marcucci,
Mark Petersen, David von Wrangel, Russ Tedrake. https://arxiv.org/abs/2205.04422

Instead of using the full time-scaling curve, this problem uses a single
time-scaling variable for each region. This formulation yields continuous
trajectories, which are not differentiable at the transition times between the
regions since non-convex continuity constraints are not supported yet. However,
it supports continuity on the path for arbitrary degree. The resulting
trajectories can be post-processed with e.g. Toppra in order to smooth out the
timing rescaling.

The ith piece of the composite trajectory is defined as q(t) = r((t - tᵢ) /
hᵢ). r : [0, 1] → ℝⁿ is a the path, parametrized as a Bézier curve with order
n. tᵢ and hᵢ are the initial time and duration of the ith sub-trajectory.

This class supports the notion of a Subgraph of regions. This has proven useful
to facilitate multi-modal motion planning such as: Subgraph A: find a
collision-free trajectory for the robot to a grasping posture, Subgraph B: find
a collision-free trajectory for the robot with the object in its hand to a
placing posture, etc.
*/
class GCSTrajectoryOptimization final {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GCSTrajectoryOptimization);

  /** Constructs the motion planning problem.
  @param num_positions is the dimension of the configuration space. */
  explicit GCSTrajectoryOptimization(int num_positions);

  ~GCSTrajectoryOptimization();

  /** A Subgraph is a subset of the larger graph. It is defined by a set of
  regions and edges between them based on intersection. From an API standpoint,
  a Subgraph is useful to define a multi-modal motion planning problem. Further,
  it allows different constraints and objects to be added to different
  subgraphs. Note that the the GraphOfConvexSets does not differentiate between
  subgraphs and can't be mixed with other instances of
  GCSTrajectoryOptimization.
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

    /** Adds multiple L2Norm Costs on the upper bound of the path length.
    We upper bound the trajectory length by the sum of the distances between
    control points. For Bézier curves, this is equivalent to the sum
    of the L2Norm of the derivative control points of the curve divided by the
    order.

    @param weight_matrix is the relative weight of each component for the cost.
    The diagonal of the matrix is the weight for each dimension.
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

   private:
    /* Constructs a new subgraph and copies the regions. */
    Subgraph(const geometry::optimization::ConvexSets& regions,
             const std::vector<std::pair<int, int>>& regions_to_connect,
             int order, double h_min, double h_max, std::string name,
             GCSTrajectoryOptimization* traj_opt);

    /* Convenience accessor, for brevity. */
    int num_positions() const { return traj_opt_.num_positions(); }

    const geometry::optimization::ConvexSets regions_;
    const int order_;
    const std::string name_;
    GCSTrajectoryOptimization& traj_opt_;

    std::vector<geometry::optimization::GraphOfConvexSets::Vertex*> vertices_;
    std::vector<geometry::optimization::GraphOfConvexSets::Edge*> edges_;

    // We keep track of the edge variables and trajectory since other
    // constraints and costs will use these.
    VectorX<symbolic::Variable> u_h_;
    VectorX<symbolic::Variable> u_vars_;

    // r(s)
    trajectories::BezierCurve<symbolic::Expression> u_r_trajectory_;

    friend class GCSTrajectoryOptimization;
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

   private:
    EdgesBetweenSubgraphs(const Subgraph& from, const Subgraph& to,
                          const geometry::optimization::ConvexSet* subspace,
                          GCSTrajectoryOptimization* traj_opt);

    /* Convenience accessor, for brevity. */
    int num_positions() const { return traj_opt_.num_positions(); }

    bool RegionsConnectThroughSubspace(
        const geometry::optimization::ConvexSet& A,
        const geometry::optimization::ConvexSet& B,
        const geometry::optimization::ConvexSet& subspace);

    GCSTrajectoryOptimization& traj_opt_;

    std::vector<geometry::optimization::GraphOfConvexSets::Edge*> edges_;

    // We keep track of the edge variables and trajectory since other
    // constraints and costs will use these.
    VectorX<symbolic::Variable> u_h_;
    VectorX<symbolic::Variable> u_vars_;
    trajectories::BezierCurve<symbolic::Expression> u_r_trajectory_;

    VectorX<symbolic::Variable> v_h_;
    VectorX<symbolic::Variable> v_vars_;
    trajectories::BezierCurve<symbolic::Expression> v_r_trajectory_;

    friend class GCSTrajectoryOptimization;
  };

  /** Returns the number of position variables. */
  int num_positions() const { return num_positions_; }

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
  copy of the regions since other functions may access them.
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
  @param name is the name of the subgraph. A default name will be provided.
  */
  Subgraph& AddRegions(
      const geometry::optimization::ConvexSets& regions,
      const std::vector<std::pair<int, int>>& edges_between_regions, int order,
      double h_min = 1e-6, double h_max = 20, std::string name = "");

  /** Creates a Subgraph with the given regions.
  This function will compute the edges between the regions based on the set
  intersections.
  @param regions represent the valid set a control point can be in. We retain a
  copy of the regions since other functions may access them.
  @param order is the order of the Bézier curve.
  @param h_max is the maximum duration to spend in a region (seconds). Some
  solvers struggle numerically with large values.
  @param h_min is the minimum duration to spend in a region (seconds) if that
  region is visited on the optimal path. Some cost and constraints are only
  convex for h > 0. For example the perspective quadratic cost of the path
  energy ||ṙ(s)||² / h becomes non-convex for h = 0. Otherwise h_min can be set
  to 0.
  @param name is the name of the subgraph. A default name will be provided.
  */
  Subgraph& AddRegions(const geometry::optimization::ConvexSets& regions,
                       int order, double h_min = 1e-6, double h_max = 20,
                       std::string name = "");

  /** Connects two subgraphs with directed edges.
  @param from is the subgraph to connect from. Must have been created from a
  call to AddRegions() on this object, not some other optimization program.
  @param to is the subgraph to connect to. Must have been created from a call to
  AddRegions() on this object, not some other optimization program.
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
      const Subgraph& from, const Subgraph& to,
      const geometry::optimization::ConvexSet* subspace = nullptr);

  /** Adds multiple L2Norm Costs on the upper bound of the path length.
  We upper bound the path integral by the sum of the distances between
  control points. For Bézier curves, this is equivalent to the sum
  of the L2Norm of the derivative control points of the curve divided by the
  order.

  This cost will be added to the entire graph. Since the path length is only
  defined for Bézier curves that have two or more control points, this cost will
  only added to all subgraphs with order greater than zero. Note that this cost
  will be applied even to subgraphs added in the future.

  @param weight_matrix is the relative weight of each component for the cost.
  The diagonal of the matrix is the weight for each dimension.
  @pre weight_matrix must be of size num_positions() x num_positions().
  */
  void AddPathLengthCost(const Eigen::MatrixXd& weight_matrix);

  /** Adds multiple L2Norm Costs on the upper bound of the path length.
  We upper bound the path integral by the sum of the distances between
  control points. For Bézier curves, this is equivalent to the sum
  of the L2Norm of the derivative control points of the curve divided by the
  order.

  This cost will be added to the entire graph. Since the path length is only
  defined for Bézier curves that have two or more control points, this cost will
  only added to all subgraphs with order greater than zero. Note that this cost
  will be applied even to subgraphs added in the future.

  @param weight is the relative weight of the cost.
  */
  void AddPathLengthCost(double weight = 1.0);

  /** Formulates and solves the mixed-integer convex formulation of the
  shortest path problem on the whole graph. @see
  `geometry::optimization::GraphOfConvexSets::SolveShortestPath()` for further
  details.

  @param source specifies the source subgraph. Must have been created from a
  call to AddRegions() on this object, not some other optimization program. If
  the source is a subgraph with more than one region, an empty set will be added
  and optimizer will choose the best region to start in. To start in a
  particular point, consider adding a subgraph of order zero with a single
  region of type Point.
  @param target specifies the target subgraph. Must have been created from a
  call to AddRegions() on this object, not some other optimization program. If
  the target is a subgraph with more than one region, an empty set will be added
  and optimizer will choose the best region to end in. To end in a particular
  point, consider adding a subgraph of order zero with a single region of type
  Point.
  @param options include all settings for solving the shortest path problem.
  @see `geometry::optimization::GraphOfConvexSetsOptions` for further details.
  */
  std::pair<trajectories::CompositeTrajectory<double>,
            solvers::MathematicalProgramResult>
  SolvePath(
      const Subgraph& source, const Subgraph& target,
      const geometry::optimization::GraphOfConvexSetsOptions& options = {});

 private:
  const int num_positions_;

  // Adds a Edge to gcs_ with the name "{u.name} -> {v.name}".
  geometry::optimization::GraphOfConvexSets::Edge* AddEdge(
      const geometry::optimization::GraphOfConvexSets::Vertex& u,
      const geometry::optimization::GraphOfConvexSets::Vertex& v);

  geometry::optimization::GraphOfConvexSets gcs_;

  // Store the subgraphs by reference.
  std::vector<std::unique_ptr<Subgraph>> subgraphs_;
  std::vector<std::unique_ptr<EdgesBetweenSubgraphs>> subgraph_edges_;
  std::vector<Eigen::MatrixXd> global_path_length_costs_;
};

}  // namespace trajectory_optimization
}  // namespace planning
}  // namespace drake
