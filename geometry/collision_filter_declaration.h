#pragma once

#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/geometry_set.h"

namespace drake {
namespace geometry {

#ifndef DRAKE_DOXYGEN_CXX
// Forward declarations.
namespace internal {
class CollisionFilter;
}  // namespace internal
#endif

/** Class for articulating changes to the configuration of SceneGraph's
 "collision filters"; collision filters limit the scope of various proximity
 queries.

 This class provides the basis for *declaring* what pairs should or should not
 be included in the set of proximity query candidates C (see documentation for
 CollisionFilterManager for details on the set C).

 A declaration consists of zero or more *statements*. Each statement can
 declare, for example, that the pair (g₁, g₂) should be excluded from C (also
 known as "filtering the pair"). That statement is added to the declaration by
 invoking the corresponding statement method (see below), and the result of the
 invocation, is that the statement is appended to the declaration.

 Each statement method returns a reference to the declaration instance, so a
 number of statements can be chained together, i.e.,

 ```
 collision_filter_manager.Apply(
   CollisionFilterDeclaration()
       .ExcludeWithin(some_geometry_set)
       .ExcludeBetween(set_A, set_B));
 ```

 It's worth noting, that the statements are evaluated in *invocation* order such
 that a later statement can partially or completely undo the effect of an
 earlier statement. The full declaration is evaluated by
 CollisionFilterManager::Apply(). */
class CollisionFilterDeclaration {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CollisionFilterDeclaration)

  CollisionFilterDeclaration() = default;

  /** @name  Allowing pairs in consideration (removing collision filters)

   These methods provide mechanisms which implicitly define a set of pairs and
   add each pair into the set of proximity query candidates C (see the
   documentation for CollisionFilterManager for definition of set C). Each
   method provides the definition for the set of pairs.

   SceneGraph maintains some invariants about pairs which will never be
   considered in proximity queries (the sets `Aₚ × Aₚ`, `Fₚ`, or `Iₚ` -- again
   see CollisionFilterManager for explanation of those sets). If any pair in
   those sets are included in declarations which allow collisions, those pairs
   will simply be ignored.

   The *declared* pairs can be invalid (e.g., containing GeometryId values that
   aren't part of the SceneGraph data). This will only be detected when applying
   the declaration (see CollisionFilterManager::Apply()).  */
  //@{

  /** Allows geometry pairs in proximity evaluation by updating the
   candidate pair set `C ← C ⋃ P*`, where `P = {(a, b)}, ∀ a ∈ A, b ∈ B, a ≠ b`
   and `A = {a₀, a₁, ..., aₘ}` and `B = {b₀, b₁, ..., bₙ}` are the input sets of
   geometries `set_A` and `set_B`, respectively. Where
   `P* = P - (Aₚ × Aₚ) - Fₚ - Iₚ`, the set of pairs after we remove the
   SceneGraph-mandated invariants (see CollisionFilterManager for details on
   those invariants).

   To be explicit, in contrast to AllowWithin, this does _not_ clear filters
   between members of the _same_ set (e.g., `(aᵢ, aⱼ)` or `(bᵢ, bⱼ)`). */
  CollisionFilterDeclaration& AllowBetween(const GeometrySet& set_A,
                                           const GeometrySet& set_B) {
    statements_.emplace_back(kAllowBetween, std::move(set_A), std::move(set_B));
    return *this;
  }

  /** Allows geometry pairs in proximity evaluation by updating the candidate
   pair set `C ← C ⋃ P*`, where `P = {(gᵢ, gⱼ)}, ∀ gᵢ, gⱼ ∈ G, i ≠ j` and
   `G = {g₀, g₁, ..., gₘ}` is the input `geometry_set` of geometries. Where
   `P* = P - (Aₚ × Aₚ) - Fₚ - Iₚ`, the set of pairs after we remove the
   SceneGraph-mandated invariants (see CollisionFilterManager for details on
   those invariants). */
  CollisionFilterDeclaration& AllowWithin(const GeometrySet& geometry_set) {
    statements_.emplace_back(kAllowWithin, std::move(geometry_set),
                             GeometrySet{});
    return *this;
  }

  //@}

  /** @name  Excluding pairs from consideration (adding collision filters)

   These methods provide mechanisms which implicitly define a set of pairs and
   subtracts each implied pair from the set of proximity query candidates C (see
   the documentation for CollisionFilterManager for definition of set C). The
   documentation of each method explains the relationship between the input
   parameters and the set of implied geometry pairs.

   The *declared* pairs can be invalid (e.g., containing GeometryId values that
   aren't part of the SceneGraph data). This will only be detected when applying
   the declaration (see CollisionFilterManager::Apply()). */
  //@{

  /** Excludes geometry pairs from proximity evaluation by updating the
   candidate pair set `C ← C - P`, where `P = {(a, b)}, ∀ a ∈ A, b ∈ B` and
   `A = {a₀, a₁, ..., aₘ}` and `B = {b₀, b₁, ..., bₙ}` are the input sets of
   geometries `set_A` and `set_B`, respectively. */
  CollisionFilterDeclaration& ExcludeBetween(GeometrySet set_A,
                                             GeometrySet set_B) {
    statements_.emplace_back(kExcludeBetween, std::move(set_A),
                             std::move(set_B));
    return *this;
  }

  /** Excludes geometry pairs from proximity evaluation by updating the
   candidate pair set `C ← C - P`, where `P = {(gᵢ, gⱼ)}, ∀ gᵢ, gⱼ ∈ G` and
   `G = {g₀, g₁, ..., gₘ}` is the input `geometry_set` of geometries. */
  CollisionFilterDeclaration& ExcludeWithin(GeometrySet geometry_set) {
    statements_.emplace_back(kExcludeWithin, std::move(geometry_set),
                             GeometrySet{});
    return *this;
  }

  //@}

 private:
  friend class CollisionFilterDeclTester;
  friend class internal::CollisionFilter;

  // The various kinds of statements that can be made.
  enum StatementOp {
    kAllowBetween,
    kAllowWithin,
    kExcludeBetween,
    kExcludeWithin
  };

  // The record of a single statement.
  struct Statement {
    Statement(StatementOp op_in, GeometrySet A_in, GeometrySet B_in)
        : operation(op_in), set_A(std::move(A_in)), set_B(std::move(B_in)) {}
    StatementOp operation;
    GeometrySet set_A;
    GeometrySet set_B;  // May be unused for unary statements.
  };

  // Although we've given CollisionFilter friend access, rather than have it
  // access statements_ directly, we'll provide an API that will better insulate
  // it from the declaration implementation.
  const std::vector<Statement>& statements() const { return statements_; }

  // The sequence of statements in this declaration.
  std::vector<Statement> statements_;
};

}  // namespace geometry
}  // namespace drake
