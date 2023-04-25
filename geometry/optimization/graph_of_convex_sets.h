#pragma once

#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_types.h"
#include "drake/common/symbolic/expression.h"
#include "drake/geometry/optimization/convex_set.h"
#include "drake/solvers/mathematical_program_result.h"
#include "drake/solvers/solver_interface.h"
#include "drake/solvers/solver_options.h"

namespace drake {
namespace geometry {
namespace optimization {

struct GraphOfConvexSetsOptions {
  GraphOfConvexSetsOptions() = default;

  /** Flag to solve the relaxed version of the problem.  As discussed in the
  paper, we know that this relaxation cannot solve the original NP-hard problem
  for all instances, but there are also many instances for which the convex
  relaxation is tight. */
  bool convex_relaxation{true};

  /** Performs a preprocessing step to remove edges that cannot lie on the
  path from source to target. In most cases, preprocessing causes a net
  reduction in computation by reducing the size of the optimization solved.
  Note that this preprocessing is not exact. There may be edges that cannot
  lie on the path from source to target that this does not detect. */
  bool preprocessing{false};

  /** Maximum number of distinct paths to compare during random rounding; only
  the lowest cost path is returned. If convex_relaxation is false or this is
  less than or equal to zero, rounding is not performed. */
  int max_rounded_paths{0};

  /** Maximum number of trials to find a novel path during random rounding. If
  convex_relaxation is false or max_rounded_paths is less than or equal to zero,
  this option is ignored. */
  int max_rounding_trials{100};

  /** Tolerance for ignoring flow along a given edge during random rounding. If
  convex_relaxation is false or max_rounded_paths is less than or equal to zero,
  this option is ignored. */
  double flow_tolerance{1e-5};

  /** Random seed to use for random rounding. If convex_relaxation is false or
  max_rounded_paths is less than or equal to zero, this option is ignored. */
  int rounding_seed{0};

  /** Optimizer to be used to solve the shortest path optimization problem. If
  not set, the best solver for the given problem is selected. Note that if the
  solver cannot handle the type of optimization problem generated, the calling
  solve method will throw. */
  const solvers::SolverInterface* solver{nullptr};

  /** Options passed to the solver when solving the generated problem.*/
  solvers::SolverOptions solver_options;
};

/**
GraphOfConvexSets implements the design pattern and optimization problems first
introduced in the paper "Shortest Paths in Graphs of Convex Sets".

"Shortest Paths in Graphs of Convex Sets" by Tobia Marcucci, Jack Umenberger,
Pablo A. Parrilo, Russ Tedrake. https://arxiv.org/abs/2101.11565

@experimental

Each vertex in the graph is associated with a convex set over continuous
variables, edges in the graph contain convex costs and constraints on these
continuous variables.  We can then formulate optimization problems over this
graph, such as the shortest path problem where each visit to a vertex also
corresponds to selecting an element from the convex set subject to the costs and
constraints.  Behind the scenes, we construct efficient mixed-integer convex
transcriptions of the graph problem using MathematicalProgram.

Design note: This class avoids providing any direct access to the
MathematicalProgram that it constructs nor to the decision variables /
constraints.  The users should be able to write constraints against
"placeholder" decision variables on the vertices and edges, but these get
translated in non-trivial ways to the underlying program.

@ingroup geometry_optimization
*/
class GraphOfConvexSets {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GraphOfConvexSets)

  /** Constructs an empty graph. */
  GraphOfConvexSets() = default;

  virtual ~GraphOfConvexSets();

  class Edge;  // forward declaration.

  using VertexId = Identifier<class VertexTag>;
  using EdgeId = Identifier<class EdgeTag>;

  /** Each vertex in the graph has a corresponding ConvexSet, and a std::string
  name. */
  class Vertex final {
   public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Vertex)

    ~Vertex();

    /** Returns the unique identifier associated with this Vertex. */
    VertexId id() const { return id_; }

    /** Returns the ambient dimension of the ConvexSet. */
    int ambient_dimension() const { return set_->ambient_dimension(); }

    /** Returns the name of the vertex. */
    const std::string& name() const { return name_; }

    /** Returns a decision variable corresponding to an element of the
    ConvexSet, which can be used for constructing symbolic::Expression costs
    and constraints. */
    const VectorX<symbolic::Variable>& x() const { return placeholder_x_; }

    /** Returns a const reference to the underlying ConvexSet. */
    const ConvexSet& set() const { return *set_; }

    /** Adds a cost to this vertex, described by a symbolic::Expression @p e
    containing *only* elements of x() as variables.  For technical reasons
    relating to being able to "turn-off" the cost on inactive vertices, all
    costs are eventually implemented with a slack variable and a constraint:
    @verbatim
    min g(x) ⇒ min ℓ, s.t. ℓ ≥ g(x)
    @endverbatim
    @note Linear costs lead to negative costs if decision variables are not
    properly constrained. Users may want to check that the solution does not
    contain negative costs.
    @returns the pair <ℓ, g(x)>.
    @throws std::exception if e.GetVariables() is not a subset of x().
    @pydrake_mkdoc_identifier{expression}
    */
    std::pair<symbolic::Variable, solvers::Binding<solvers::Cost>> AddCost(
        const symbolic::Expression& e);

    /** Adds a cost to this vertex.  @p binding must contain *only* elements of
    x() as variables. For technical reasons relating to being able to "turn-off"
    the cost on inactive vertices, all costs are eventually implemented with a
    slack variable and a constraint:
    @verbatim
    min g(x) ⇒ min ℓ, s.t. ℓ ≥ g(x)
    @endverbatim
    @note Linear costs lead to negative costs if decision variables are not
    properly constrained. Users may want to check that the solution does not
    contain negative costs.
    @returns the pair <ℓ, g(x)>.
    @throws std::exception if binding.variables() is not a subset of x().
    @pydrake_mkdoc_identifier{binding}
    */
    std::pair<symbolic::Variable, solvers::Binding<solvers::Cost>> AddCost(
        const solvers::Binding<solvers::Cost>& binding);

    /** Adds a constraint to this vertex, described by a symbolic::Formula @p f
    containing *only* elements of x() as variables.
    @throws std::exception if f.GetFreeVariables() is not a subset of x().
    @pydrake_mkdoc_identifier{formula}
    */
    solvers::Binding<solvers::Constraint> AddConstraint(
        const symbolic::Formula& f);

    /** Adds a constraint to this vertex.  @p binding must contain *only*
    elements of x() as variables.
    @throws std::exception if binding.variables() is not a subset of x().
    @pydrake_mkdoc_identifier{binding}
    */
    solvers::Binding<solvers::Constraint> AddConstraint(
        const solvers::Binding<solvers::Constraint>& binding);

    /** Returns all costs on this vertex. */
    const std::vector<solvers::Binding<solvers::Cost>>& GetCosts() const {
      return costs_;
    }

    /** Returns all constraints on this vertex. */
    const std::vector<solvers::Binding<solvers::Constraint>>& GetConstraints()
        const {
      return constraints_;
    }

    /** Returns the sum of the costs associated with this vertex in a
    solvers::MathematicalProgramResult. */
    double GetSolutionCost(
        const solvers::MathematicalProgramResult& result) const;

    /** Returns the solution of x() in a MathematicalProgramResult.  This
    solution is NaN if the vertex is not in the shortest path (or if we are
    solving the the convex relaxation and the total flow through this vertex at
    the solution is numerically close to zero).  We prefer to return NaN than a
    value not contained in set().
    */
    Eigen::VectorXd GetSolution(
        const solvers::MathematicalProgramResult& result) const;

    // TODO(russt): Support AddCost/AddConstraint directly on the vertices.

   private:
    // Constructs a new vertex.
    Vertex(VertexId id, const ConvexSet& set, std::string name);

    const VertexId id_{};
    const std::unique_ptr<const ConvexSet> set_;
    const std::string name_{};
    const VectorX<symbolic::Variable> placeholder_x_{};
    // Note: ell_[i] is associated with costs_[i].
    solvers::VectorXDecisionVariable ell_{};
    std::vector<solvers::Binding<solvers::Cost>> costs_{};
    std::vector<solvers::Binding<solvers::Constraint>> constraints_{};

    friend class GraphOfConvexSets;
  };

  // Note: We think of this as a directed edge in the shortest path problem, but
  // there is nothing specific to it being a directed edge here in this class.
  /** An edge in the graph connects between vertex `u` and vertex `v`.  The
  edge also holds a list of cost and constraints associated with the continuous
  variables. */
  class Edge final {
   public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Edge)

    ~Edge();

    /** Returns the unique identifier associated with this Edge. */
    EdgeId id() const { return id_; }

    /** Returns the string name associated with this edge. */
    const std::string& name() const { return name_; }

    /** Returns a const reference to the "left" Vertex that this edge connects
    to. */
    const Vertex& u() const { return *u_; }

    /** Returns a const reference to the "right" Vertex that this edge connects
    to. */
    const Vertex& v() const { return *v_; }

    /** Returns the binary variable associated with this edge. It can be used
    to determine whether this edge was active in the solution to an
    optimization problem, by calling GetSolution(phi()) on a returned
    MathematicalProgramResult. */
    const symbolic::Variable& phi() const { return phi_; }

    /** Returns the continuous decision variables associated with vertex `u`.
    This can be used for constructing symbolic::Expression costs and
    constraints.*/
    const VectorX<symbolic::Variable>& xu() const { return u_->x(); }

    /** Returns the continuous decision variables associated with vertex `v`.
    This can be used for constructing symbolic::Expression costs and
    constraints.*/
    const VectorX<symbolic::Variable>& xv() const { return v_->x(); }

    /** Adds a cost to this edge, described by a symbolic::Expression @p e
    containing *only* elements of xu() and xv() as variables.  For technical
    reasons relating to being able to "turn-off" the cost on inactive edges, all
    costs are eventually implemented with a slack variable and a constraint:
    @verbatim
    min g(xu, xv) ⇒ min ℓ, s.t. ℓ ≥ g(xu,xv)
    @endverbatim
    @note Linear costs lead to negative costs if decision variables are not
    properly constrained. Users may want to check that the solution does not
    contain negative costs.
    @returns the pair <ℓ, g(xu, xv)>.
    @throws std::exception if e.GetVariables() is not a subset of xu() ∪ xv().
    @pydrake_mkdoc_identifier{expression}
    */
    std::pair<symbolic::Variable, solvers::Binding<solvers::Cost>> AddCost(
        const symbolic::Expression& e);

    /** Adds a cost to this edge.  @p binding must contain *only* elements of
    xu() and xv() as variables. For technical reasons relating to being able to
    "turn-off" the cost on inactive edges, all costs are eventually implemented
    with a slack variable and a constraint:
    @verbatim
    min g(xu, xv) ⇒ min ℓ, s.t. ℓ ≥ g(xu,xv)
    @endverbatim
    @note Linear costs lead to negative costs if decision variables are not
    properly constrained. Users may want to check that the solution does not
    contain negative costs.
    @returns the pair <ℓ, g(xu, xv)>.
    @throws std::exception if binding.variables() is not a subset of xu() ∪
    xv().
    @pydrake_mkdoc_identifier{binding}
    */
    std::pair<symbolic::Variable, solvers::Binding<solvers::Cost>> AddCost(
        const solvers::Binding<solvers::Cost>& binding);

    /** Adds a constraint to this edge, described by a symbolic::Formula @p f
    containing *only* elements of xu() and xv() as variables.
    @throws std::exception if f.GetFreeVariables() is not a subset of xu() ∪
    xv().
    @pydrake_mkdoc_identifier{formula}
    */
    solvers::Binding<solvers::Constraint> AddConstraint(
        const symbolic::Formula& f);

    /** Adds a constraint to this edge.  @p binding must contain *only*
    elements of xu() and xv() as variables.
    @throws std::exception if binding.variables() is not a subset of xu() ∪
    xv().
    @pydrake_mkdoc_identifier{binding}
    */
    solvers::Binding<solvers::Constraint> AddConstraint(
        const solvers::Binding<solvers::Constraint>& binding);

    /** Adds a constraint on the binary variable associated with this edge.
    @note We intentionally do not return a binding to the constraint created by
    this call, as that would allow the caller to make nonsensical modifications
    to its bounds (i.e. requiring phi == 0.5). */
    void AddPhiConstraint(bool phi_value);

    /** Removes any constraints added with AddPhiConstraint. */
    void ClearPhiConstraints();

    /** Returns all costs on this edge. */
    const std::vector<solvers::Binding<solvers::Cost>>& GetCosts() const {
      return costs_;
    }

    /** Returns all constraints on this edge. */
    const std::vector<solvers::Binding<solvers::Constraint>>& GetConstraints()
        const {
      return constraints_;
    }

    /** Returns the sum of the costs associated with this edge in a
    solvers::MathematicalProgramResult. */
    double GetSolutionCost(
        const solvers::MathematicalProgramResult& result) const;

    /** Returns the vector value of the slack variables associated with ϕxᵤ in a
    solvers::MathematicalProgramResult. */
    Eigen::VectorXd GetSolutionPhiXu(
        const solvers::MathematicalProgramResult& result) const;

    /** Returns the vector value of the slack variables associated with ϕxᵥ in
    a solvers::MathematicalProgramResult. */
    Eigen::VectorXd GetSolutionPhiXv(
        const solvers::MathematicalProgramResult& result) const;

   private:
    // Constructs a new edge.
    Edge(const EdgeId& id, const Vertex* u, const Vertex* v, std::string name);

    const EdgeId id_{};
    const Vertex* const u_{};
    const Vertex* const v_{};
    symbolic::Variables allowed_vars_{};
    symbolic::Variable phi_{};
    const std::string name_{};

    // We construct placeholder variables for y and z here so that they can be
    // accessed later from a MathematicalProgramResult.  We intentionally do
    // *not* provide direct access to them for the user.
    const VectorX<symbolic::Variable> y_{};
    const VectorX<symbolic::Variable> z_{};

    std::unordered_map<symbolic::Variable, symbolic::Variable> x_to_yz_{};
    // Note: ell_[i] is associated with costs_[i].
    solvers::VectorXDecisionVariable ell_{};
    std::vector<solvers::Binding<solvers::Cost>> costs_{};
    std::vector<solvers::Binding<solvers::Constraint>> constraints_{};
    std::optional<bool> phi_value_{};

    friend class GraphOfConvexSets;
  };

  /** Adds a vertex to the graph.  A copy of @p set is cloned and stored inside
  the graph. If @p name is empty then a default name will be provided. */
  Vertex* AddVertex(const ConvexSet& set, std::string name = "");

  // TODO(russt): Provide helper methods to add multiple vertices which share
  // the same ConvexSet.

  /** Adds an edge to the graph from VertexId @p u_id to VertexId @p v_id.  The
  ids must refer to valid vertices in this graph. If @p name is empty then a
  default name will be provided.
  @pydrake_mkdoc_identifier{by_id}
  */
  Edge* AddEdge(VertexId u_id, VertexId v_id, std::string name = "");

  /** Adds an edge to the graph from Vertex @p u to Vertex @p v.  The
  vertex references must refer to valid vertices in this graph. If @p name is
  empty then a default name will be provided.
  @pydrake_mkdoc_identifier{by_reference}
  */
  Edge* AddEdge(const Vertex& u, const Vertex& v, std::string name = "");

  /** Removes vertex @p vertex_id from the graph as well as any edges from or to
  the vertex. Runtime is O(nₑ) where nₑ is the number of edges in the graph.
  @pre The vertex must be part of the graph.
  @pydrake_mkdoc_identifier{by_id}
  */
  void RemoveVertex(VertexId vertex_id);

  /** Removes vertex @p vertex from the graph as well as any edges from or to
  the vertex. Runtime is O(nₑ) where nₑ is the number of edges in the graph.
  @pre The vertex must be part of the graph.
  @pydrake_mkdoc_identifier{by_reference}
  */
  void RemoveVertex(const Vertex& vertex);

  /** Removes edge @p edge_id from the graph.
  @pre The edge must be part of the graph.
  @pydrake_mkdoc_identifier{by_id}
  */
  void RemoveEdge(EdgeId edge_id);

  /** Removes edge @p edge from the graph.
  @pre The edge must be part of the graph.
  @pydrake_mkdoc_identifier{by_reference}
  */
  void RemoveEdge(const Edge& edge);

  /** Returns mutable pointers to the vertices stored in the graph. */
  std::vector<Vertex*> Vertices();

  /** Returns pointers to the vertices stored in the graph.
  @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.} */
  std::vector<const Vertex*> Vertices() const;

  /** Returns mutable pointers to the edges stored in the graph. */
  std::vector<Edge*> Edges();

  /** Returns pointers to the edges stored in the graph.
  @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.} */
  std::vector<const Edge*> Edges() const;

  /** Removes all constraints added to any edge with AddPhiConstraint. */
  void ClearAllPhiConstraints();

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
      bool show_slacks = true, int precision = 3,
      bool scientific = false) const;

  /** Formulates and solves the mixed-integer convex formulation of the
  shortest path problem on the graph, as discussed in detail in

  "Shortest Paths in Graphs of Convex Sets" by Tobia Marcucci, Jack
  Umenberger, Pablo A. Parrilo, Russ Tedrake. https://arxiv.org/abs/2101.11565

  @param source specifies the source set.  The solver will choose any point in
  that set; to start at a particular continuous state consider adding a Point
  set to the graph and using that as the source.
  @param target specifies the target set.  The solver will choose any point in
  that set.
  @param options include all settings for solving the shortest path problem. See
  `GraphOfConvexSetsOptions` for further details.

  @throws std::exception if any of the costs or constraints in the graph are
  incompatible with the shortest path formulation or otherwise unsupported.
  All costs must be non-negative for all values of the continuous variables.

  @pydrake_mkdoc_identifier{by_id}
  */
  solvers::MathematicalProgramResult SolveShortestPath(
      VertexId source_id, VertexId target_id,
      const GraphOfConvexSetsOptions& options = GraphOfConvexSetsOptions(),
      std::vector<solvers::MathematicalProgramResult>* all_results =
          nullptr) const;

  /** Convenience overload that takes const reference arguments for source and
  target.
  @pydrake_mkdoc_identifier{by_reference}
  */
  solvers::MathematicalProgramResult SolveShortestPath(
      const Vertex& source, const Vertex& target,
      const GraphOfConvexSetsOptions& options = GraphOfConvexSetsOptions(),
      std::vector<solvers::MathematicalProgramResult>* all_results =
          nullptr) const;

 private:
  /* Facilitates testing. */
  friend class PreprocessShortestPathTest;

  std::set<EdgeId> PreprocessShortestPath(VertexId source_id,
                                          VertexId target_id) const;

  // Adds a perspective constraint to the mathematical program to upper bound
  // the cost below a slack variable, ℓ. Specifically given a cost g(x) to
  // minimize, this method implements it with a slack variable and a constraint:
  // min g(x) ⇒ min ℓ, s.t. ℓ ≥ ϕ g(ϕx)
  // `vars` is a vector of variables to be used in the cost and constraint
  // consisting of ℓ, ϕ, and ϕ times the variables in the original cost.
  void AddPerspectiveCost(solvers::MathematicalProgram* prog,
                          const solvers::Binding<solvers::Cost>& binding,
                          const solvers::VectorXDecisionVariable& vars) const;

  // Adds a perspective version of the constraint to the mathematical program.
  // Specifically given a constraint h(x) ≤ b, this method implements its
  // perspective:
  // h(x) ≤ b ⇒ h(ϕx) ≤ ϕb
  // vars` is a vector of variables to be used in the constraint consisting of
  // ϕ, and ϕ times the variables in the original constraint.
  void AddPerspectiveConstraint(
      solvers::MathematicalProgram* prog,
      const solvers::Binding<solvers::Constraint>& binding,
      const solvers::VectorXDecisionVariable& vars) const;

  void FillResult(
      std::map<VertexId, std::vector<Edge*>> incoming_edges,
      std::map<VertexId, std::vector<Edge*>> outgoing_edges,
      std::vector<Edge*> excluded_edges,
      std::map<VertexId, solvers::MatrixXDecisionVariable> vertex_edge_ell,
      std::map<EdgeId, symbolic::Variable> relaxed_phi,
      std::vector<symbolic::Variable> excluded_phi, VertexId target_id,
      const solvers::MathematicalProgram& prog,
      const GraphOfConvexSetsOptions& options,
      solvers::MathematicalProgramResult* result) const;

  std::map<VertexId, std::unique_ptr<Vertex>> vertices_{};
  std::map<EdgeId, std::unique_ptr<Edge>> edges_{};
};

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
