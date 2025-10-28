#pragma once

#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/common/parallelism.h"
#include "drake/common/symbolic/expression.h"
#include "drake/geometry/optimization/convex_set.h"
#include "drake/solvers/mathematical_program_result.h"
#include "drake/solvers/solver_interface.h"
#include "drake/solvers/solver_options.h"

namespace drake {
namespace geometry {
namespace optimization {

struct GraphOfConvexSetsOptions {
  /** Passes this object to an Archive.
  Refer to @ref yaml_serialization "YAML Serialization" for background. Note:
  This only serializes options that are YAML built-in types.  */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(convex_relaxation));
    a->Visit(DRAKE_NVP(max_rounded_paths));
    a->Visit(DRAKE_NVP(preprocessing));
    a->Visit(DRAKE_NVP(max_rounding_trials));
    a->Visit(DRAKE_NVP(flow_tolerance));
    a->Visit(DRAKE_NVP(rounding_seed));
    // N.B. We skip the DRAKE_NVP(solver), DRAKE_NVP(restriction_solver), and
    // DRAKE_NVP(preprocessing_solver), because it cannot be serialized.
    // TODO(#20967) Serialize the DRAKE_NVP(solver_options).
    // TODO(#20967) Serialize the DRAKE_NVP(restriction_solver_options).
    // TODO(#20967) Serialize the DRAKE_NVP(preprocessing_solver_options).
  }

  /** Flag to solve the relaxed version of the problem.  As discussed in the
  paper, we know that this relaxation cannot solve the original NP-hard problem
  for all instances, but there are also many instances for which the convex
  relaxation is tight. If convex_relaxation=nullopt, then each GCS method is
  free to choose an appropriate default. */
  std::optional<bool> convex_relaxation{std::nullopt};

  /** Maximum number of distinct paths to compare during random rounding; only
  the lowest cost path is returned. If convex_relaxation is false or this is
  less than or equal to zero, rounding is not performed. If
  max_rounded_paths=nullopt, then each GCS method is free to choose an
  appropriate default. */
  std::optional<int> max_rounded_paths{std::nullopt};

  /** Performs a preprocessing step to remove edges that cannot lie on the
  path from source to target. In most cases, preprocessing causes a net
  reduction in computation by reducing the size of the optimization solved.
  Note that this preprocessing is not exact. There may be edges that cannot
  lie on the path from source to target that this does not detect. If
  preprocessing=nullopt, then each GCS method is free to choose an appropriate
  default. */
  std::optional<bool> preprocessing{std::nullopt};

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

  // TODO(#20969) The following solver interfaces may need to be moved to fully
  // serialize the options.

  /** Optimizer to be used to solve the MIP, the relaxation of the shortest path
  optimization problem and the convex restriction if no restriction_solver is
  provided. If not set, the best solver for the given problem is selected.
  Note that if the solver cannot handle the type of optimization problem
  generated, the calling solvers::SolverInterface::Solve()  method will
  throw. */
  const solvers::SolverInterface* solver{nullptr};

  /** Optimizer to be used in SolveConvexRestriction(), which is also called
  during the rounding stage of SolveShortestPath() given the relaxation.
  If not set, the interface at .solver will be used, if provided, otherwise the
  best solver for the given problem is selected. Note that if the solver cannot
  handle the type of optimization problem generated, then calling the
  solvers::SolverInterface::Solve() method will throw. */
  const solvers::SolverInterface* restriction_solver{nullptr};

  /** Optimizer to be used in the preprocessing stage of GCS, which is
  performed when SolveShortestPath is called when the `preprocessing` setting
  has been set to true. If not set, the interface at .solver will be used, if
  provided, otherwise the best solver for the given problem is selected. Note
  that if the solver cannot handle the type of optimization problem generated,
  then calling the solvers::SolverInterface::Solve() method will throw. */
  const solvers::SolverInterface* preprocessing_solver{nullptr};

  /** Options passed to the solver when solving the generated problem.*/
  solvers::SolverOptions solver_options{};

  /** Optional solver options to be used in SolveConvexRestriction(), which is
  also used during the rounding stage of SolveShortestPath() given the
  relaxation. If not set, solver_options is used.
  For instance, one might want to set tighter (i.e., lower) tolerances for
  running the relaxed problem and looser (i.e., higher) tolerances for final
  solves during rounding. */
  std::optional<solvers::SolverOptions> restriction_solver_options{
      std::nullopt};

  /** Optional solver options to be used by preprocessing_solver in the
  preprocessing stage of GCS, which is used in SolveShortestPath. If
  preprocessing_solver is set but this parameter is not then solver_options is
  used. For instance, one might want to print solver logs for the main
  optimization, but not from the many smaller preprocessing optimizations. */
  std::optional<solvers::SolverOptions> preprocessing_solver_options{
      std::nullopt};

  /** Some steps in GCS can be parallelized. This is the maximum number of
  threads used in all places in the algorithm.
  @note Some solvers will choose their own level of parallelization, independent
  of this setting. To limit the number of threads, add
  @ref solvers::CommonSolverOption::kMaxThreads to the solver_options. */
  Parallelism parallelism{Parallelism::Max()};
};

struct GcsGraphvizOptions {
  /** Passes this object to an Archive.
  Refer to @ref yaml_serialization "YAML Serialization" for background. */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(show_slacks));
    a->Visit(DRAKE_NVP(show_vars));
    a->Visit(DRAKE_NVP(show_flows));
    a->Visit(DRAKE_NVP(show_costs));
    a->Visit(DRAKE_NVP(scientific));
    a->Visit(DRAKE_NVP(precision));
  }

  /** Determines whether the values of the intermediate (slack) variables are
  also displayed in the graph. */
  bool show_slacks{true};

  /** Determines whether the solution values for decision variables in each set
  are shown. */
  bool show_vars{true};

  /** Determines whether the flow value results are shown. The flow values are
  shown both with a numeric value and through the transparency value on the
  edge, where a flow of 0.0 will correspond to an (almost) invisible edge,
  and a flow of 1.0 will display as a fully black edge. */
  bool show_flows{true};

  /** Determines whether the cost value results are shown. This will show both
  edge and vertex costs. */
  bool show_costs{true};

  /** Sets the floating point formatting to scientific (if true) or fixed (if
  false). */
  bool scientific{false};

  /** Sets the floating point precision (how many digits are generated) of the
  annotations. */
  int precision{3};
};

/**
GraphOfConvexSets (GCS) implements the design pattern and optimization problems
first introduced in the paper "Shortest Paths in Graphs of Convex Sets".

"Shortest Paths in Graphs of Convex Sets" by Tobia Marcucci, Jack Umenberger,
Pablo A. Parrilo, Russ Tedrake. https://arxiv.org/abs/2101.11565

@experimental

Each vertex in the graph is associated with a convex set over continuous
variables, edges in the graph contain convex costs and constraints on these
continuous variables.  We can then formulate optimization problems over this
graph, such as the shortest path problem where each visit to a vertex also
corresponds to selecting an element from the convex set subject to the costs
and constraints.  Behind the scenes, we construct efficient mixed-integer
convex transcriptions of the graph problem using MathematicalProgram.
However, we provide the option to solve an often tight convex relaxation of the
problem with GraphOfConvexSetsOptions::convex_relaxation and employ a cheap
rounding stage which solves the convex restriction along potential paths to
find a feasible solution to the original problem.

Design note: This class avoids providing any direct access to the
MathematicalProgram that it constructs nor to the decision variables /
constraints.  The users should be able to write constraints against
"placeholder" decision variables on the vertices and edges, but these get
translated in non-trivial ways to the underlying program.

@anchor nonconvex_graph_of_convex_sets
<b>Advanced Usage: Guiding Non-convex Optimization with the
%GraphOfConvexSets</b>

Solving a GCS problem using convex relaxation involves two components:
- Convex Relaxation: The relaxation of the binary variables (edge activations)
  and perspective operations on the convex cost/constraints leads to a
  convex problem that considers the graph as a whole.
- Rounding: After solving the relaxation, a randomized rounding scheme is
  applied to obtain a feasible solution for the original problem. We interpret
  the relaxed flow variables as edge probabilities to guide the maximum
  likelyhood depth first search from the source to target vertices.
  Each rounding is calling SolveConvexRestriction.

To handle non-convex constraints, one can provide convex surrogates to the
relaxation and the true non-convex constraints to the rounding problem.
These surrogates approximate the non-convex constraints, making the relaxation
solvable as a convex optimization to guide the non-convex rounding. This can be
controlled by the Transcription enum in the AddConstraint method. We
encourage users to provide a strong convex surrogate, when possible, to better
approximate the original non-convex problem.

Users can also specify a GCS implicitly, which can be important for very large
or infinite graphs, by deriving from ImplicitGraphOfConvexSets.

@ingroup geometry_optimization
*/
class GraphOfConvexSets {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GraphOfConvexSets);

  /** Specify the transcription of the optimization problem to which a
  constraint or cost should be added, or from which they should be retrieved.*/
  enum class Transcription {
    kMIP,          ///< The mixed integer formulation of the GCS problem.
    kRelaxation,   ///< The relaxation of the GCS problem.
    kRestriction,  ///< The restrction of the GCS problem where the path is
                   ///< fixed.
  };

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
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Vertex);

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
    min g(x) ⇒ min ℓ, s.t. ℓ ≥ g(x).
    @endverbatim
    You must use GetSolutionCost() to retrieve the cost of the solution, rather
    than evaluating the cost directly, in order to get consistent behavior when
    solving with the different GCS transcriptions.
    @param use_in_transcription specifies the components of the problem to
    which the constraint should be added.
    @note Linear costs lead to negative costs if decision variables are not
    properly constrained. Users may want to check that the solution does not
    contain negative costs.
    @returns the added cost, g(x).
    @throws std::exception if e.GetVariables() is not a subset of x().
    @throws std::exception if no transcription is specified.
    @pydrake_mkdoc_identifier{expression}
    */
    solvers::Binding<solvers::Cost> AddCost(
        const symbolic::Expression& e,
        const std::unordered_set<Transcription>& use_in_transcription = {
            Transcription::kMIP, Transcription::kRelaxation,
            Transcription::kRestriction});

    /** Adds a cost to this vertex.  @p binding must contain *only* elements of
    x() as variables. For technical reasons relating to being able to "turn-off"
    the cost on inactive vertices, all costs are eventually implemented with a
    slack variable and a constraint:
    @verbatim
    min g(x) ⇒ min ℓ, s.t. ℓ ≥ g(x).
    @endverbatim
    You must use GetSolutionCost() to retrieve the cost of the solution, rather
    than evaluating the cost directly, in order to get consistent behavior when
    solving with the different GCS transcriptions.
    @param use_in_transcription specifies the components of the problem to
    which the constraint should be added.
    @note Linear costs lead to negative costs if decision variables are not
    properly constrained. Users may want to check that the solution does not
    contain negative costs.
    @returns the added cost, g(x).
    @throws std::exception if binding.variables() is not a subset of x().
    @throws std::exception if no transcription is specified.
    @pydrake_mkdoc_identifier{binding}
    */
    solvers::Binding<solvers::Cost> AddCost(
        const solvers::Binding<solvers::Cost>& binding,
        const std::unordered_set<Transcription>& use_in_transcription = {
            Transcription::kMIP, Transcription::kRelaxation,
            Transcription::kRestriction});

    /** Adds a constraint to this vertex.
    @param f must contain *only* elements of x() as variables.
    @param use_in_transcription specifies the components of the problem to
    which the constraint should be added.
    @throws std::exception if f.GetFreeVariables() is not a subset of x().
    @throws std::exception if ambient_dimension() == 0.
    @throws std::exception if no transcription is specified.
    @pydrake_mkdoc_identifier{formula}
    */
    solvers::Binding<solvers::Constraint> AddConstraint(
        const symbolic::Formula& f,
        const std::unordered_set<Transcription>& use_in_transcription = {
            Transcription::kMIP, Transcription::kRelaxation,
            Transcription::kRestriction});

    /** Adds a constraint to this vertex.
    @param binding must contain *only* elements of x() as variables.
    @param use_in_transcription specifies the components of the problem to
    which the constraint should be added.
    @throws std::exception if binding.variables() is not a subset of x().
    @throws std::exception if ambient_dimension() == 0.
    @throws std::exception if no transcription is specified.
    @pydrake_mkdoc_identifier{binding}
    */
    solvers::Binding<solvers::Constraint> AddConstraint(
        const solvers::Binding<solvers::Constraint>& binding,
        const std::unordered_set<Transcription>& use_in_transcription = {
            Transcription::kMIP, Transcription::kRelaxation,
            Transcription::kRestriction});

    /** Returns costs on this vertex.
    @param used_in_transcription specifies the components of the problem from
    which the constraint should be retrieved.
    @throws std::exception if no transcription is specified.
    */
    std::vector<solvers::Binding<solvers::Cost>> GetCosts(
        const std::unordered_set<Transcription>& used_in_transcription = {
            Transcription::kMIP, Transcription::kRelaxation,
            Transcription::kRestriction}) const;

    /** Returns constraints on this vertex.
    @param used_in_transcription specifies the components of the problem from
    which the constraint should be retrieved.
    @throws std::exception if no transcription is specified.
    */
    std::vector<solvers::Binding<solvers::Constraint>> GetConstraints(
        const std::unordered_set<Transcription>& used_in_transcription = {
            Transcription::kMIP, Transcription::kRelaxation,
            Transcription::kRestriction}) const;

    /** Returns the sum of the costs associated with this vertex in `result`, or
    std::nullopt if no solution for this vertex is available. */
    std::optional<double> GetSolutionCost(
        const solvers::MathematicalProgramResult& result) const;

    /** Returns the cost associated with the `cost` binding on this vertex in
    `result`, or std::nullopt if no solution for this vertex is available.
    @throws std::exception if cost is not associated with this vertex. */
    std::optional<double> GetSolutionCost(
        const solvers::MathematicalProgramResult& result,
        const solvers::Binding<solvers::Cost>& cost) const;

    /** Returns the solution of x() in `result`, or std::nullopt if no solution
    for this vertex is available. std::nullopt can happen if the vertex is
    deactivated (e.g. not in the shorest path) in the solution. */
    std::optional<Eigen::VectorXd> GetSolution(
        const solvers::MathematicalProgramResult& result) const;

    const std::vector<Edge*>& incoming_edges() const { return incoming_edges_; }
    const std::vector<Edge*>& outgoing_edges() const { return outgoing_edges_; }

   private:
    // Constructs a new vertex.
    Vertex(VertexId id, const ConvexSet& set, std::string name);

    void AddIncomingEdge(Edge* e);
    void AddOutgoingEdge(Edge* e);
    void RemoveIncomingEdge(Edge* e);
    void RemoveOutgoingEdge(Edge* e);

    const VertexId id_{};
    const std::unique_ptr<const ConvexSet> set_;
    const std::string name_{};
    const VectorX<symbolic::Variable> placeholder_x_{};
    // Note: ell_[i] is associated with costs_[i].
    solvers::VectorXDecisionVariable ell_{};
    std::vector<std::pair<solvers::Binding<solvers::Cost>,
                          std::unordered_set<Transcription>>>
        costs_{};
    std::vector<std::pair<solvers::Binding<solvers::Constraint>,
                          std::unordered_set<Transcription>>>
        constraints_;

    std::vector<Edge*> incoming_edges_{};
    std::vector<Edge*> outgoing_edges_{};

    friend class GraphOfConvexSets;
  };

  // Note: We think of this as a directed edge in the shortest path problem, but
  // there is nothing specific to it being a directed edge here in this class.
  /** An edge in the graph connects between vertex `u` and vertex `v`.  The
  edge also holds a list of cost and constraints associated with the continuous
  variables. */
  class Edge final {
   public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Edge);

    ~Edge();

    /** Returns the unique identifier associated with this Edge. */
    EdgeId id() const { return id_; }

    /** Returns the string name associated with this edge. */
    const std::string& name() const { return name_; }

    /** Returns a const reference to the "left" Vertex that this edge connects
    to. */
    const Vertex& u() const { return *u_; }

    /** Returns a mutable reference to the "left" Vertex that this edge connects
    to. */
    Vertex& u() { return *u_; }

    /** Returns a const reference to the "right" Vertex that this edge connects
    to. */
    const Vertex& v() const { return *v_; }

    /** Returns a mutable reference to the "right" Vertex that this edge
    connects to. */
    Vertex& v() { return *v_; }

    /** Returns the binary variable associated with this edge. It can be used
    to determine whether this edge was active in the solution to an
    optimization problem, by calling GetSolution(phi()) on a returned
    MathematicalProgramResult. */
    const symbolic::Variable& phi() const { return phi_; }

    /** Returns the continuous decision variables associated with vertex `u`.
    This can be used for constructing symbolic::Expression costs and
    constraints.

    See also GetSolutionPhiXu(); using `result.GetSolution(xu())` may not
    be what you want.
    */
    const VectorX<symbolic::Variable>& xu() const { return u_->x(); }

    /** Returns the continuous decision variables associated with vertex `v`.
    This can be used for constructing symbolic::Expression costs and
    constraints.

    See also GetSolutionPhiXv(); using `result.GetSolution(xv())` may not
    be what you want.
    */
    const VectorX<symbolic::Variable>& xv() const { return v_->x(); }

    /** Creates continuous slack variables for this edge, appending them to
    an internal vector of existing slack variables. These slack variables
    can be used in any cost or constraint on this edge only, and allows for
    modeling more complex costs and constraints.
     */
    solvers::VectorXDecisionVariable NewSlackVariables(int rows,
                                                       const std::string& name);

    /** Adds a cost to this edge, described by a symbolic::Expression @p e
    containing *only* elements of xu() and xv() as variables.  For technical
    reasons relating to being able to "turn-off" the cost on inactive edges, all
    costs are eventually implemented with a slack variable and a constraint:
    @verbatim
    min g(xu, xv) ⇒ min ℓ, s.t. ℓ ≥ g(xu,xv)
    @endverbatim
    You must use GetSolutionCost() to retrieve the cost of the solution, rather
    than evaluating the cost directly, in order to get consistent behavior when
    solving with the different GCS transcriptions.
    @param use_in_transcription specifies the components of the problem to
    which the constraint should be added.
    @note Linear costs lead to negative costs if decision variables are not
    properly constrained. Users may want to check that the solution does not
    contain negative costs.
    @returns the added cost, g(xu, xv).
    @throws std::exception if e.GetVariables() is not a subset of xu() ∪ xv().
    @throws std::exception if no transcription is specified.
    @pydrake_mkdoc_identifier{expression}
    */
    solvers::Binding<solvers::Cost> AddCost(
        const symbolic::Expression& e,
        const std::unordered_set<Transcription>& use_in_transcription = {
            Transcription::kMIP, Transcription::kRelaxation,
            Transcription::kRestriction});

    /** Adds a cost to this edge.  @p binding must contain *only* elements of
    xu() and xv() as variables. For technical reasons relating to being able to
    "turn-off" the cost on inactive edges, all costs are eventually implemented
    with a slack variable and a constraint:
    @verbatim
    min g(xu, xv) ⇒ min ℓ, s.t. ℓ ≥ g(xu,xv)
    @endverbatim
    You must use GetSolutionCost() to retrieve the cost of the solution, rather
    than evaluating the cost directly, in order to get consistent behavior when
    solving with the different GCS transcriptions.
    @param use_in_transcription specifies the components of the problem to
    which the constraint should be added.
    @note Linear costs lead to negative costs if decision variables are not
    properly constrained. Users may want to check that the solution does not
    contain negative costs.
    @returns the added cost, g(xu, xv).
    @throws std::exception if binding.variables() is not a subset of xu() ∪
    xv().
    @throws std::exception if no transcription is specified.
    @pydrake_mkdoc_identifier{binding}
    */
    solvers::Binding<solvers::Cost> AddCost(
        const solvers::Binding<solvers::Cost>& binding,
        const std::unordered_set<Transcription>& use_in_transcription = {
            Transcription::kMIP, Transcription::kRelaxation,
            Transcription::kRestriction});

    /** Adds a constraint to this edge.
    @param f must contain *only* elements of xu() and xv() as variables.
    @param use_in_transcription specifies the components of the problem to
    which the constraint should be added.

    @throws std::exception if f.GetFreeVariables() is not a subset of xu() ∪
    xv().
    @throws std::exception if xu() ∪ xv() is empty, i.e., when both vertices
    have an ambient dimension of zero.
    @throws std::exception if no transcription is specified.
    @pydrake_mkdoc_identifier{formula}
    */
    solvers::Binding<solvers::Constraint> AddConstraint(
        const symbolic::Formula& f,
        const std::unordered_set<Transcription>& use_in_transcription = {
            Transcription::kMIP, Transcription::kRelaxation,
            Transcription::kRestriction});

    /** Adds a constraint to this edge.
    @param binding must contain *only* elements of xu() and xv() as variables.
    @param use_in_transcription specifies the components of the problem to
    which the constraint should be added.

    @throws std::exception if binding.variables() is not a subset of xu() ∪
    xv().
    @throws std::exception if xu() ∪ xv() is empty, i.e., when both vertices
    have an ambient dimension of zero.
    @throws std::exception if no transcription is specified.
    @pydrake_mkdoc_identifier{binding}
    */
    solvers::Binding<solvers::Constraint> AddConstraint(
        const solvers::Binding<solvers::Constraint>& binding,
        const std::unordered_set<Transcription>& use_in_transcription = {
            Transcription::kMIP, Transcription::kRelaxation,
            Transcription::kRestriction});

    /** Adds a constraint on the binary variable associated with this edge.
    @note We intentionally do not return a binding to the constraint created by
    this call, as that would allow the caller to make nonsensical modifications
    to its bounds (i.e. requiring phi == 0.5). */
    void AddPhiConstraint(bool phi_value);

    /** Removes any constraints added with AddPhiConstraint. */
    void ClearPhiConstraints();

    /** Returns costs on this edge.
    @param used_in_transcription specifies the components of the problem from
    which the constraint should be retrieved.
    @throws std::exception if no transcription is specified.
    */
    std::vector<solvers::Binding<solvers::Cost>> GetCosts(
        const std::unordered_set<Transcription>& used_in_transcription = {
            Transcription::kMIP, Transcription::kRelaxation,
            Transcription::kRestriction}) const;

    /** Returns constraints on this edge.
    @param used_in_transcription specifies the components of the problem from
    which the constraint should be retrieved.
    @throws std::exception if no transcription is specified.
    */
    std::vector<solvers::Binding<solvers::Constraint>> GetConstraints(
        const std::unordered_set<Transcription>& used_in_transcription = {
            Transcription::kMIP, Transcription::kRelaxation,
            Transcription::kRestriction}) const;

    /** Returns the sum of the costs associated with this edge in `result`, or
    std::nullopt if no solution for this edge is available. */
    std::optional<double> GetSolutionCost(
        const solvers::MathematicalProgramResult& result) const;

    /** Returns the cost associated with the `cost` binding on this edge in
    `result`, or std::nullopt if no solution for this edge is available.
    @throws std::exception if cost is not associated with this edge. */
    std::optional<double> GetSolutionCost(
        const solvers::MathematicalProgramResult& result,
        const solvers::Binding<solvers::Cost>& cost) const;

    /** Returns the vector value of the slack variables associated with ϕxᵤ in
    `result`, or std::nullopt if no solution for this edge is available. This
    can obtain a different value than the Vertex::GetSolution(), e.g. from
    `edge->xu().GetSolution(result)`. First, a deactivated edge (defined by Phi
    ~= 0) will return the zero vector here, while Vertex::GetSolution() will
    return std::nullopt (rather than divide by zero to recover Xu). Second, in
    the case of a loose convex relaxation, the vertex version will return the
    *averaged* value of the edge slacks for all non-zero-flow edges. */
    std::optional<Eigen::VectorXd> GetSolutionPhiXu(
        const solvers::MathematicalProgramResult& result) const;

    /** Returns the vector value of the slack variables associated with ϕxᵥ in
    `result`, or std::nullopt if no solution for this edge is available.
    See GetSolutionPhiXu() for more details. */
    std::optional<Eigen::VectorXd> GetSolutionPhiXv(
        const solvers::MathematicalProgramResult& result) const;

   private:
    // Constructs a new edge.
    Edge(const EdgeId& id, Vertex* u, Vertex* v, std::string name);

    const EdgeId id_{};
    Vertex* const u_{};
    Vertex* const v_{};
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
    std::vector<std::pair<solvers::Binding<solvers::Cost>,
                          std::unordered_set<Transcription>>>
        costs_{};
    solvers::VectorXDecisionVariable slacks_{};
    std::vector<std::pair<solvers::Binding<solvers::Constraint>,
                          std::unordered_set<Transcription>>>
        constraints_;
    std::optional<bool> phi_value_{};

    friend class GraphOfConvexSets;
  };

  /** Returns a deep copy of this graph.
  @throws std::exception if edges have slack variables. We can add this support
  once it's needed.
  */
  std::unique_ptr<GraphOfConvexSets> Clone() const;

  /** Adds a vertex to the graph.  A copy of @p set is cloned and stored inside
  the graph. If @p name is empty then a default name will be provided. */
  Vertex* AddVertex(const ConvexSet& set, std::string name = "");

  /** Adds a new vertex to the graph (and assigns a new unique VertexId) by
  taking the name, costs, and constraints (but not any edges) from
  `template_vertex`. `template_vertex` does not need to be registered with this
  GCS instance; this method can be used to effectively copy a Vertex from
  another GCS instance into `this`. */
  Vertex* AddVertexFromTemplate(const Vertex& template_vertex);

  // TODO(russt): Provide helper methods to add multiple vertices which share
  // the same ConvexSet.

  /** Adds an edge to the graph from Vertex @p u to Vertex @p v.  The
  vertex references must refer to valid vertices in this graph. If @p name is
  empty then a default name will be provided.
  @throws std::exception if `u` or `v` are not valid vertices in this graph. */
  Edge* AddEdge(Vertex* u, Vertex* v, std::string name = "");

  /** Adds an edge to the graph from Vertex `u` to Vertex `v` (and assigns a
  new unique EdgeId), by taking the name, costs, and constraints from
  `template_edge`. `template_edge` does not need to be registered with this
  GCS instance; this method can be used to effectively copy an Edge from
  another GCS instance into `this`.
  @throws std::exception if `u` or `v` are not valid vertices in this graph.
  @throws std::exception if `u` or `v` do not match the sizes of the
  `template_edge.u()` and `template_edge.v()` vertices.
  @throws std::exception if edges have slack variables. We can add this support
  once it's needed.
  */
  Edge* AddEdgeFromTemplate(Vertex* u, Vertex* v, const Edge& template_edge);

  /** Returns the first vertex (by the order added to `this`) with the given
  name, or nullptr if no such vertex exists. */
  const Vertex* GetVertexByName(const std::string& name) const;

  /** Returns the first vertex (by the order added to `this`) with the given
  name, or nullptr if no such vertex exists. */
  Vertex* GetMutableVertexByName(const std::string& name);

  /** Returns the first edge (by the order added to `this`) with the given
  name, or nullptr if no such edge exists. */
  const Edge* GetEdgeByName(const std::string& name) const;

  /** Returns the first edge (by the order added to `this`) with the given
  name, or nullptr if no such edge exists. */
  Edge* GetMutableEdgeByName(const std::string& name);

  /** Removes vertex @p vertex from the graph as well as any edges from or to
  the vertex. Runtime is O(nₑ) where nₑ is the number of edges connected to @p
  vertex
  @pre The vertex must be part of the graph.
  */
  void RemoveVertex(Vertex* vertex);

  /** Removes edge @p edge from the graph.
  @pre The edge must be part of the graph.
  */
  void RemoveEdge(Edge* edge);

  int num_vertices() const { return vertices_.size(); }
  int num_edges() const { return edges_.size(); }

  /** Returns mutable pointers to the vertices stored in the graph. */
  std::vector<Vertex*> Vertices();

  /** Returns pointers to the vertices stored in the graph.
  @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.} */
  std::vector<const Vertex*> Vertices() const;

  /** Returns true iff `v` is registered as a vertex with `this`.

  @pydrake_mkdoc_identifier{vertex}
  */
  bool IsValid(const Vertex& v) const;

  /** Returns mutable pointers to the edges stored in the graph. */
  std::vector<Edge*> Edges();

  /** Returns pointers to the edges stored in the graph.
  @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.} */
  std::vector<const Edge*> Edges() const;

  /** Returns true iff `e` is registered as an edge with `this`.

  @pydrake_mkdoc_identifier{edge}
  */
  bool IsValid(const Edge& e) const;

  /** Removes all constraints added to any edge with AddPhiConstraint. */
  void ClearAllPhiConstraints();

  /** Returns a Graphviz string describing the graph vertices and edges. If
  `result` is supplied, then the graph will be annotated with the solution
  values, according to `options`.
  @param result the optional result from a solver.
  @param options the struct containing various options for visualization.
  @param active_path optionally highlights a given path in the graph. The path
  is displayed as dashed edges in red, displayed in addition to the original
  graph edges.
  */
  std::string GetGraphvizString(
      const solvers::MathematicalProgramResult* result = nullptr,
      const GcsGraphvizOptions& options = GcsGraphvizOptions(),
      const std::vector<const Edge*>* active_path = nullptr) const;

  /** Formulates and solves the mixed-integer convex formulation of the
  shortest path problem on the graph, as discussed in detail in

  "Shortest Paths in Graphs of Convex Sets" by Tobia Marcucci, Jack Umenberger,
  Pablo A. Parrilo, Russ Tedrake. https://arxiv.org/abs/2101.11565

  @param source specifies the source set.  The solver will choose any point in
  that set; to start at a particular continuous state consider adding a Point
  set to the graph and using that as the source.
  @param target specifies the target set.  The solver will choose any point in
  that set.
  @param options include all settings for solving the shortest path problem.
  See `GraphOfConvexSetsOptions` for further details. The following default
  options will be used if they are not provided in `options`:
  - `options.convex_relaxation = false`,
  - `options.max_rounded_paths = 0`,
  - `options.preprocessing = false`.

  @throws std::exception if any of the costs or constraints in the graph are
  incompatible with the shortest path formulation or otherwise unsupported. All
  costs must be non-negative for all values of the continuous variables.
  */
  solvers::MathematicalProgramResult SolveShortestPath(
      const Vertex& source, const Vertex& target,
      const GraphOfConvexSetsOptions& options =
          GraphOfConvexSetsOptions()) const;

  /** Extracts a path from `source` to `target` described by the `result`
  returned by SolveShortestPath(), via depth-first search following the largest
  values of the edge binary variables.
  @param tolerance defines the threshold for checking the integrality
  conditions of the binary variables for each edge. `tolerance` = 0 would
  demand that the binary variables are exactly 1 for the edges on the path.
  `tolerance` = 1 would allow the binary variables to be any value in [0, 1].
  The default value is 1e-3.
  @throws std::exception if !result.is_success() or no path from `source` to
  `target` can be found in the solution.
  */
  std::vector<const Edge*> GetSolutionPath(
      const Vertex& source, const Vertex& target,
      const solvers::MathematicalProgramResult& result,
      double tolerance = 1e-3) const;

  /** Samples a collection of unique paths from `source` to `target`, where the
   flow values (the relaxed binary variables associated with each `Edge`)
   `flows` are interpreted as the probabilities of transitioning an edge.
   The returned paths are guaranteed to be unique, and the number of returned
   paths can be 0 if no paths are found. This function implements the first part
   of the rounding scheme put forth in Section 4.2 of "Motion Planning around
   Obstacles with Convex Optimization": https://arxiv.org/abs/2205.04422

   @param source specifies the source vertex.
   @param target specifies the target vertex.
   @param flows specifies the edge flows, which are interprested as the
   probability of transition an edge. Edge flows that are not specified are
   taken to be zero.
   @param options include all settings for sampling the paths. Specifically,
   the behavior of this function is determined through `options.rounding_seed`,
   `options.max_rounded_paths`, `options.max_rounding_trials`, and
   `options.flow_tolerance`, as described in `GraphOfConvexSetsOptions`.
   @returns A vector of paths, where each path is a vector of `Edge`s.
   @throws std::exception if options.max_rounded_path < 1.
   @pydrake_mkdoc_identifier{flows}
   */
  std::vector<std::vector<const Edge*>> SamplePaths(
      const Vertex& source, const Vertex& target,
      const std::unordered_map<const Edge*, double>& flows,
      const GraphOfConvexSetsOptions& options) const;

  /** Samples a collection of unique paths from `source` to `target`, where the
   flow values (the relaxed binary variables associated with each `Edge`)
   in `result` are interpreted as the probabilities of transitioning an edge.
   The returned paths are guaranteed to be unique, and the number of returned
   paths can be 0 if no paths are found. This function implements the first part
   of the rounding scheme put forth in Section 4.2 of "Motion Planning around
   Obstacles with Convex Optimization": https://arxiv.org/abs/2205.04422

   @param source specifies the source vertex.
   @param target specifies the target vertex.
   @param options include all settings for sampling the paths. Specifically,
   the behavior of this function is determined through `options.rounding_seed`,
   `options.max_rounded_paths`, `options.max_rounding_trials`, and
   `options.flow_tolerance`, as described in `GraphOfConvexSetsOptions`.
   @returns A vector of paths, where each path is a vector of `Edge`s.
   @throws std::exception if options.max_rounded_path < 1.
   @pydrake_mkdoc_identifier{result}
   */
  std::vector<std::vector<const Edge*>> SamplePaths(
      const Vertex& source, const Vertex& target,
      const solvers::MathematicalProgramResult& result,
      const GraphOfConvexSetsOptions& options) const;

  /** The non-convexity in a GCS problem comes from the binary variables (phi)
  associated with the edges being active or inactive in the solution. If those
  binary variables are fixed, then the problem is convex -- this is a so-called
  "convex restriction" of the original problem.

  The convex restriction can often be solved much more efficiently than solving
  the full GCS problem with additional constraints to fix the binaries; it can
  be written using less decision variables, and needs only to include the
  vertices associated with at least one of the active edges. Decision variables
  for all other convex sets will be set to NaN.

  Note that one can specify additional non-convex constraints, which may be
  not supported by all solvers. In this case, the provided solver will throw
  an exception.

  If an @p initial_guess is provided, the solution inside this result will be
  used to set the initial guess for the convex restriction. Typically, this will
  be the result obtained by solving the convex relaxation.

  @throws std::exception if the @p initial_guess does not contain solutions for
  the decision variables required in this convex restriction.
  */
  solvers::MathematicalProgramResult SolveConvexRestriction(
      const std::vector<const Edge*>& active_edges,
      const GraphOfConvexSetsOptions& options = GraphOfConvexSetsOptions(),
      const solvers::MathematicalProgramResult* initial_guess = nullptr) const;

 private: /* Facilitates testing. */
  friend class PreprocessShortestPathTest;

  // Construct a prog so that it contains the variables and constraints of the
  // preprocessing program for a given edge.
  std::unique_ptr<solvers::MathematicalProgram> ConstructPreprocessingProgram(
      EdgeId edge_id,
      const std::map<VertexId, std::vector<int>>& incoming_edges,
      const std::map<VertexId, std::vector<int>>& outgoing_edges,
      VertexId source_id, VertexId target_id) const;

  // Construct a prog that can be used to solve the convex restriction for a
  // given set of active edges (and optionally populate with an initial guess if
  // one is provided).
  std::unique_ptr<solvers::MathematicalProgram> ConstructRestrictionProgram(
      const std::vector<const Edge*>& active_edges,
      const solvers::MathematicalProgramResult* initial_guess) const;

  // Add results for additional variables in `result` to make it comparable with
  // other transcriptions.
  void MakeRestrictionResultLookLikeMixedInteger(
      const solvers::MathematicalProgram& prog,
      solvers::MathematicalProgramResult* result,
      const std::vector<const Edge*>& active_edges) const;

  std::set<EdgeId> PreprocessShortestPath(
      VertexId source_id, VertexId target_id,
      const GraphOfConvexSetsOptions& options) const;

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
  // `vars` is a vector of variables to be used in the constraint consisting of
  // ϕ, and ϕ times the variables in the original constraint.
  void AddPerspectiveConstraint(
      solvers::MathematicalProgram* prog,
      const solvers::Binding<solvers::Constraint>& binding,
      const solvers::VectorXDecisionVariable& vars) const;

  // Note: we use VertexId and EdgeId (vs e.g. Vertex* and Edge*) here to
  // provide consistent ordering of the vertices/edges. This is important for
  // producing consistent MathematicalPrograms (the order of costs and
  // constraints can change the behavior). But prefer using Vertex* and Edge*
  // over VertexId and EdgeId in the public API; this means avoiding any sorted
  // containers (like std::set or std::map) using their default ordering.
  std::map<VertexId, std::unique_ptr<Vertex>> vertices_{};
  std::map<EdgeId, std::unique_ptr<Edge>> edges_{};
};

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
