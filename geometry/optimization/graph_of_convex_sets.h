#pragma once

#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/common/symbolic.h"
#include "drake/geometry/optimization/convex_set.h"
#include "drake/solvers/mathematical_program_result.h"

namespace drake {
namespace geometry {
namespace optimization {

/**
GraphOfConvexSets implements the design pattern and optimization problems first
introduced in

"Shortest Paths in Graphs of Convex Sets" by Tobia Marcucci, Jack Umenberger,
Pablo A. Parrilo, Russ Tedrake. https://arxiv.org/abs/2101.11565

Each vertex in the graph is associated with a convex set over continuous
variables, edges in the graph contain convex costs and constraints on these
continuous variables.  We can then formulate optimization problems over this
graph, such as the shortest path problem where each visit to a vertex also
corresponds to selecting an element from the convex set subject to the costs and
constraints.  Behind the scenes, we construct efficient mixed-integer convex
transcriptions of the graph problem using MathematicalProgram.

Design note: This class avoids providing any direct access to the
MathematicalProgram that it constructs nor to the decision variables /
constraints.  The users should be able to write constraints against faux
decision variables on the vertices and edges, but these get translated in
non-trivial ways to the underlying program.

@ingroup geometry_optimization
*/
class GraphOfConvexSets {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GraphOfConvexSets)

  /** Constructs an empty graph. */
  GraphOfConvexSets() {}

  class Edge;  // forward declaration.

  /** Each vertex in the graph has a corresponding ConvexSet, and a std::string
  name. */
  class Vertex final {
   public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Vertex)

    // Note: This is public instead of private for compatibility with smart
    // pointers (e.g. https://abseil.io/tips/134) and testing.
    /** Constructs a new vertex.  Most user should not call this directly;
    prefer adding edges with GraphOfConvexSets::AddVertex().  */
    Vertex(const ConvexSet& set, std::string name);

    ~Vertex();

    /** Returns the ambient dimens of the ConvexSet. */
    int ambient_dimension() const { return set_.ambient_dimension(); }

    /** Returns the name of the vertex. */
    const std::string& name() const { return name_; }

    /** Returns a decision variable corresponding to an element of the
    ConvexSet, which can be used for constructing symbolic::Expression costs
    and constraints. */
    const VectorX<symbolic::Variable>& x() const { return placeholder_x_; }

    /** Returns a const reference to the underlying ConvexSet. */
    const ConvexSet& set() const { return set_; }

    /** Returns the solution of x() in a MathematicalProgramResult. */
    Eigen::VectorXd GetSolution(
        const solvers::MathematicalProgramResult& result) const;

    // TODO(russt): Support AddCost/AddConstraint directly on the vertices.

   private:
    const ConvexSet& set_;
    const std::string name_{};
    const VectorX<symbolic::Variable> placeholder_x_{};
  };

  // Note: We think of this as a directed edge in the shortest path problem, but
  // there is nothing specific to it being a directed edge here in this class.
  /** An edge in the graph connects between vertex `u` and vertex `v`.  The
  edge also holds a list of cost and constraints associated with the continuous
  variables. */
  class Edge final {
   public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Edge)

    /** Constructs a new edge.  Most user should not call this directly; prefer
    adding edges with GraphOfConvexSets::AddEdge().  */
    Edge(const Vertex& u, const Vertex& v, std::string name);

    ~Edge();

    /** Returns the string name associated with this edge. */
    const std::string& name() const { return name_; }

    /** Returns a const reference to the "left" Vertex that this edge connects
    to. */
    const Vertex& u() const { return u_; }

    /** Returns a const reference to the "right" Vertex that this edge connects
     * to. */
    const Vertex& v() const { return v_; }

    /** Returns the binary variable associated with this edge. It can be used
    to determine whether this edge was active in the solution to an
    optimization problem, by calling GetSolution(phi()) on a returned
    MathematicalProgramResult. */
    const symbolic::Variable& phi() const { return phi_; }

    /* Returns the continuous decision variables associated with vertex `u`.
    This can be used for constructing symbolic::Expression costs and
    constraints.*/
    const VectorX<symbolic::Variable>& xu() const { return u_.x(); }

    /* Returns the continuous decision variables associated with vertex `v`.
    This can be used for constructing symbolic::Expression costs and
    constraints.*/
    const VectorX<symbolic::Variable>& xv() const { return v_.x(); }

    /** Adds a cost to this edge, described by a symbolic::Expression @p e
    containing *only* elements of xu() and xv() as variables.  For technical
    reasons relating to being able to "turn-off" the cost on inactive edges, all
    costs are eventually implemented with a slack variable and a constraint:
    @verbatim
    min g(xu, xv) ⇒ min ℓ, s.t. ℓ ≥ g(xu,xv)
    @endverbatim
    @returns the pair <ℓ, Binding<Cost>>.
    @throws std::exception if e.GetVariables() is not a subset of xu() ∩ xv().
    */
    std::pair<symbolic::Variable, solvers::Binding<solvers::Cost>> AddCost(
        const symbolic::Expression& e);

    /** Adds a cost to this vertex.  @p binding must must contain *only*
    elements of xu() and xv() as variables. For technical
    reasons relating to being able to "turn-off" the cost on inactive edges, all
    costs are eventually implemented with a slack variable and a constraint:
    @verbatim
    min g(xu, xv) ⇒ min ℓ, s.t. ℓ ≥ g(xu,xv)
    @endverbatim
    @returns the pair <ℓ, Binding<Cost>>.
    @throws std::exception if vars is not a subset of xu() ∩ xv(). */
    std::pair<symbolic::Variable, solvers::Binding<solvers::Cost>> AddCost(
        const solvers::Binding<solvers::Cost>& binding);

    /** Adds a cost to this edge, described by a symbolic::Formula @p f
    containing *only* elements of xu() and xv() as variables.
    @throws std::exception if f.GetVariables() is not a subset of xu() ∩ xv().
    */
    solvers::Binding<solvers::Constraint> AddConstraint(
        const symbolic::Formula& f);

    /** Adds a constraint to this vertex.  @p binding must must contain *only*
    elements of xu() and xv() as variables.
    @throws std::exception if vars is not a subset of xu() ∩ xv(). */
    solvers::Binding<solvers::Constraint> AddConstraint(
        const solvers::Binding<solvers::Constraint>& binding);

    /** Returns the sum of the costs associated with this edge in a
    MathematicalProgramResult. */
    double GetSolutionCost(
        const solvers::MathematicalProgramResult& result) const;

    friend class GraphOfConvexSets;

   private:
    const Vertex& u_;
    const Vertex& v_;
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
    std::unordered_set<solvers::Binding<solvers::Constraint>> constraints_{};
  };

  /** Adds a vertex to the graph, taking ownership @p set. If @p name is empty
  then a default name will be provided.
  @exclude_from_pydrake_mkdoc{Not bound in pydrake.} */
  Vertex* AddVertex(std::unique_ptr<ConvexSet> set, std::string name = "");

  /** Adds a vertex to the graph.  A copy of @p set is cloned and stored inside
  the graph. If @p name is empty then a default name will be provided. */
  Vertex* AddVertex(const ConvexSet& set, std::string name = "");

  // TODO(russt): Provide AddVertexWithCopies or DuplicateVertex to facilitate
  // multiple visits to the same set.  This is one reason why this class
  // owns the sets, instead of the vertices.

  /** Adds an edge to the graph from Vertex @p u to Vertex @p v.
  @pydrake_mkdoc_identifier{vertex} */
  Edge* AddEdge(const Vertex& u, const Vertex& v, std::string name = "");

  // TODO(russt): std::string GetGraphvizString(const
  // std::optional<solvers::MathematicalProgramResult>& = std::nullopt) const;

  // TODO(russt): Most all of the interesting work will happen in this method.
  // The only reason that it is not included in this initial PR is that we have
  // separted it for code review.
  //
  // solvers::MathematicalProgramResult SolveShortestPath(const Vertex& source,
  // const Vertex& target, bool convex_relaxation = false);

 private:
  ConvexSets sets_{};
  std::unordered_set<std::unique_ptr<Vertex>> vertices_{};
  std::unordered_set<std::unique_ptr<Edge>> edges_{};
};

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
