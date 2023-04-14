#pragma once

#include <map>
#include <memory>
#include <string>
#include <tuple>
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
time-scaling variable for each region. This formulation can not support convex
constraints on continuity greater than order 0 on the trajectory, but can
support continuity on the path for arbitrary degree. The resulting trajectories
can be post-processed with e.g. Toppra in order to smooth out the timing
rescaling.

The trajectory is defined as q(t) = r(h(t)).
r(s) is the path, parametrized as a Bézier curve.
h(s) = duration * s is the time-scaling function.

This class supports the notion of a Subgraph of regions. This has proven useful
to facilitate multi-modal motion planning such as: Subgraph A: find a
collision-free trajectory for the robot to a grasping posture, Subgraph B: find
a collision-free trajectory for the robot with the object in its hand to a
placing posture, etc.
*/
class GCSTrajectoryOptimization {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GCSTrajectoryOptimization);

  ~GCSTrajectoryOptimization() = default;

  /**
  Constructs the motion planning problem.
  @param num_positions is the dimension of the configuration space.
  */
  explicit GCSTrajectoryOptimization(int num_positions);

  class Subgraph final {
   public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Subgraph);

    ~Subgraph() = default;

    /** Returns the name of the subgraph.*/
    const std::string& name() const { return name_; }

    /** Returns the order of the Bézier trajectory within the region.*/
    int order() const { return order_; }

    /** Returns the number of vertices in the subgraph.*/
    int size() const { return vertices_.size(); }

    /** Returns the regions associated with this subgraph before the
     * CartesianProduct.*/
    const geometry::optimization::ConvexSets& regions() const {
      return regions_;
    }

    /** Adds multiple L2Norm Costs on the upper bound of the path length.
    We upper bound the path integral by the sum of the distances between
    control points. For Bézier curves, this is equivalent to the sum
    of the L2Norm of the derivative control points of the curve divided by the
    order.

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

    @param weight is the relative weight of the cost.
    */
    void AddPathLengthCost(double weight = 1.0);

   private:
    /* Constructs a new subgraph and copies the regions.*/
    Subgraph(const geometry::optimization::ConvexSets& regions,
             const std::vector<std::pair<int, int>>& regions_to_connect,
             int order, double d_min, double d_max, const std::string& name,
             GCSTrajectoryOptimization* gcs);

    const geometry::optimization::ConvexSets regions_;
    int order_;
    const std::string name_;
    GCSTrajectoryOptimization* gcs_;

    std::vector<geometry::optimization::GraphOfConvexSets::Vertex*> vertices_;
    std::vector<geometry::optimization::GraphOfConvexSets::Edge*> edges_;

    // We keep track of the edge variables and trajectory since other
    // constraints and costs will use these.
    VectorX<symbolic::Variable> u_duration_;
    VectorX<symbolic::Variable> u_vars_;

    // r(s)
    trajectories::BezierCurve<symbolic::Expression> u_r_trajectory_;

    friend class GCSTrajectoryOptimization;
  };

  class EdgesBetweenSubgraphs final {
   public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(EdgesBetweenSubgraphs);

    ~EdgesBetweenSubgraphs() = default;

   private:
    EdgesBetweenSubgraphs(const Subgraph* from, const Subgraph* to,
                          const geometry::optimization::ConvexSet* subspace,
                          GCSTrajectoryOptimization* gcs);

    bool RegionsConnectThroughSubspace(
        const geometry::optimization::ConvexSet& A,
        const geometry::optimization::ConvexSet& B,
        const geometry::optimization::ConvexSet& subspace);

    GCSTrajectoryOptimization* gcs_;

    std::vector<geometry::optimization::GraphOfConvexSets::Edge*> edges_;

    // We keep track of the edge variables and trajectory since other
    // constraints and costs will use these.
    VectorX<symbolic::Variable> u_duration_;
    VectorX<symbolic::Variable> u_vars_;
    trajectories::BezierCurve<symbolic::Expression> u_r_trajectory_;

    VectorX<symbolic::Variable> v_duration_;
    VectorX<symbolic::Variable> v_vars_;
    trajectories::BezierCurve<symbolic::Expression> v_r_trajectory_;

    friend class GCSTrajectoryOptimization;
  };

  /** Returns the number of position variables. */
  int num_positions() const { return num_positions_; }

  /**
  @param show_slacks determines whether the values of the intermediate
  (slack) variables are also displayed in the graph.
  @param precision sets the floating point precision (how many digits are
  generated) of the annotations.
  @param scientific sets the floating point formatting to scientific (if true)
  or fixed (if false).
  */
  std::string GetGraphvizString(bool show_slack = true, int precision = 3,
                                bool scientific = false) const {
    return gcs_.GetGraphvizString(std::nullopt, show_slack, precision,
                                  scientific);
  }

  /** Creates a Subgraph with the given regions and indices.
  @param regions represent the valid set a control point can be in.
    We retain copy of the regions since other functions may access them.
  @param edges_between_regions is a list of pairs of indices into the regions
    vector. For each pair representing an edge between two regions, an edge is
  added within the subgraph. Note that the edges are directed so (i,j) will only
  add an edge from region i to region j.
  @param order is the order of the Bézier curve.
  @param d_max is the maximum duration to spend in a region (seconds). Some
  solvers struggle numerically with large values.
  @param d_min is the minimum duration to spend in a region (seconds) if that
  region is visited on the optimal path.
    Some cost and constraints are only convex for d > 0. For example the
  perspective quadratic cost of the path energy ||ṙ(s)||² / d becomes non-convex
  for d = 0. Otherwise d_min can be set to 0.
  @param name is the name of the subgraph. A default name will be provided.
  */
  Subgraph& AddRegions(
      const geometry::optimization::ConvexSets& regions,
      const std::vector<std::pair<int, int>>& edges_between_regions, int order,
      double d_min = 1e-6, double d_max = 20, std::string name = "");

  /** Creates a Subgraph with the given regions.
  @param regions represent the valid set a control point can be in.
    We retain copy of the regions since other functions may access them.
  @param order is the order of the Bézier curve.
  @param d_max is the maximum duration to spend in a region (seconds). Some
  solvers struggle numerically with large values.
  @param d_min is the minimum duration to spend in a region (seconds) if that
  region is visited on the optimal path. Some cost and constraints are only
  convex for d > 0. For example the perspective quadratic cost of the path
  energy ||ṙ(s)||² / d becomes non-convex for d = 0. Otherwise d_min can be set
  to 0.
  @param name is the name of the subgraph. A default name will be provided.
  */
  Subgraph& AddRegions(const geometry::optimization::ConvexSets& regions,
                       int order, double d_min = 1e-6, double d_max = 20,
                       std::string name = "");

  /** Connects two subgraphs with directed edges.
  @param from is the subgraph to connect from.
  @param to is the subgraph to connect to.
  @param subspace is the subspace that the connecting control points must be in.
    Subspace is optional. Only edges that connect through the subspace will be
  added, and the subspace is added as a constraint on the connecting control
  points. Subspaces of type point or HPolyhedron are supported since other sets
  require constraints that are not yet supported by the GraphOfConvexSets::Edge
  constraint, e.g. set containment of a HyperEllipsoid is formulated via
  LorentzCone constraints. Workaround: Create a subgraph of zero order with the
  subspace as the region and connect it between the two subgraphs. This works
  because GraphOfConvexSet::Vertex , supports arbitrary instances of ConvexSets.
  */
  EdgesBetweenSubgraphs& AddEdges(
      const Subgraph* from, const Subgraph* to,
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

  std::pair<trajectories::CompositeTrajectory<double>,
            solvers::MathematicalProgramResult>
  SolvePath(const Subgraph& source, const Subgraph& target,
            const geometry::optimization::GraphOfConvexSetsOptions& options =
                geometry::optimization::GraphOfConvexSetsOptions());

 private:
  // store the subgraphs by reference
  std::vector<std::unique_ptr<Subgraph>> subgraphs_{};
  std::vector<std::unique_ptr<EdgesBetweenSubgraphs>> subgraph_edges_{};

  std::vector<Eigen::MatrixXd> global_path_length_costs_{};

  int num_positions_;

  geometry::optimization::GraphOfConvexSets gcs_{
      geometry::optimization::GraphOfConvexSets()};
};

}  // namespace trajectory_optimization
}  // namespace planning
}  // namespace drake
