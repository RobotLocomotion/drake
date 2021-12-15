#pragma once

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "drake/common/drake_deprecated.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"
#include "drake/geometry/query_results/signed_distance_pair.h"
#include "drake/geometry/query_results/signed_distance_to_point.h"
#include "drake/geometry/render/render_camera.h"
#include "drake/geometry/scene_graph_inspector.h"
#include "drake/math/rigid_transform.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace geometry {

template <typename T>
class SceneGraph;

/** The %QueryObject serves as a mechanism to perform geometry queries on the
 world's geometry. The SceneGraph has an abstract-valued port that contains
 a  %QueryObject (i.e., a %QueryObject-valued output port).

 To perform geometry queries on SceneGraph:
   - a LeafSystem must have a %QueryObject-valued input port and connect it to
     the corresponding query output port on SceneGraph,
   - the querying LeafSystem can evaluate the input port, retrieving a `const
     QueryObject&` in return, and, finally,
   - invoke the appropriate method on the %QueryObject.

 The const reference returned by the input port is considered "live" - it is
 linked to the context, system, and cache (making full use of all of those
 mechanisms). This const reference should _never_ be persisted; doing so can
 lead to erroneous query results. It is simpler and more advisable to acquire it
 for evaluation in a limited scope (e.g., CalcTimeDerivatives()) and then
 discard it. If a %QueryObject is needed for many separate functions in a
 LeafSystem, each should re-evaluate the input port. The underlying caching
 mechanism should make the cost of this negligible.

 The %QueryObject _can_ be copied. The copied instance is no longer "live"; it
 is now "baked". Essentially, it freezes the state of the live scene graph in
 its current configuration and disconnects it from the system and context. This
 means, even if the original context changes values, the copied/baked instance
 will always reproduce the same query results. This baking process is not cheap
 and should not be done without consideration.

 @anchor query_object_precision_methodology
 <h2>Queries and scalar type</h2>

 A %QueryObject _cannot_ be converted to a different scalar type. A %QueryObject
 of scalar type T can only be acquired from the output port of a SceneGraph
 of type T evaluated on a corresponding Context, also of type T.

 %QueryObject's support for arbitrary scalar type is incomplete. Not all queries
 support all scalar types to the same degree. Furthermore, the queries are
 typically served by *families* of algorithms. The evaluation of a query between
 a particular pair of geometries will depend on the query, the pair of geometry
 types involved, and the scalar type. From query to query, the treatment of
 a geometry (or geometry pair) for a given scalar type can vary in many ways,
 including but not limited to: ignoring the geometry, throwing an exception,
 results with limited precision, or full, accurate support. The queries below
 provide tables to help inform your expectations when evaluating queries. The
 tables all use a common methodology and admit a common interpretation.

 For each (query, geometry-pair, scalar) combination, we create a set of
 geometric configurations with known answers. We evaluate the precision of the
 query result (if supported at all) over the entire set and report the *worst*
 observed error. This is a purely empirical approach and doesn't fully
 characterize the families of underlying algorithms, and the reported error
 may be misleading in that we've missed far worse latent error or that the
 error reported doesn't well represent the average case.

 The families of algorithms also differ in how their inherent errors scale with
 the scale of the problem (e.g., size of geometries, magnitude of
 distance/depth, etc.) Attempting to fully characterize that aspect is both
 arduous and problematic, so, we've chosen a more *representative* approach.

 Because Drake is primarily intended for robot simulation, we've created
 geometric configurations on the scale of common robot manipulators (on the
 order of 20cm). We position them with a known penetration depth (or separating
 distance) of 2 mm. The error reported is the deviation from that expected
 result.

 When interpreting the tables, keep the following in mind:
   - The table illustrates trends in *broad* strokes, only. It does not
     represent an exhaustive analysis.
   - If your problem involves larger geometries, greater penetration depths, or
     larger separating distances, the error will vary. Do not assume that
     observed error in this context is necessarily relative -- there truly is
     that much variability in the families of algorithms.
   - These values are an attempt to capture the *worst* case outcome. The
     error shown is a single-significant-digit approximation of that observed
     error.
   - These values may not actually represent the true worst case; discovering
     the true worst case is generally challenging. These represent our best
     effort to date. If you find outcomes that are worse those reported here,
     please <a href="https://github.com/RobotLocomotion/drake/issues/new">
     submit a bug</a>.
   - These tables represent Drake's transient state. The eventual goal is to
     report no more than 1e-14 error across all supportable geometry pairs
     and scalars. At that point, the table will simply disappear.

 @tparam_nonsymbolic_scalar
*/
template <typename T>
class QueryObject {
 public:
  /** Constructs a default QueryObject (all pointers are null). */
  QueryObject() = default;

  /** @name Implements CopyConstructible, CopyAssignable, \
                       MoveConstructible, MoveAssignable

   Calling the copy constructor or assignment will turn a _live_ %QueryObject
   into a _baked_ %QueryObject (an expensive operation). Copying baked
   QueryObjects is cheap.  */
  //@{

  QueryObject(const QueryObject& other);
  QueryObject& operator=(const QueryObject&);
  QueryObject(QueryObject&&) = default;
  QueryObject& operator=(QueryObject&&) = default;

  //@}

  // Note to developers on adding queries:
  //  All queries should call ThrowIfNotCallable() before taking any action.
  //  Furthermore, an invocation of that query method should be included in
  //  query_object_test.cc in the DefaultQueryThrows test to confirm that the
  //  query *is* calling ThrowIfNotCallable().

  /** Provides an inspector for the topological structure of the underlying
   scene graph data (see SceneGraphInspector for details).  */
  const SceneGraphInspector<T>& inspector() const {
    return inspector_;
  }

  /** @name                Pose-dependent Introspection

   These methods provide access to introspect geometry and frame quantities that
   directly depend on the poses of the frames.  For geometry and frame
   quantities that do not depend on the poses of frames, such as  X_FG, use
   inspector() to access the SceneGraphInspector.  */
  //@{

  /** Reports the position of the frame indicated by `frame_id` relative to the
   world frame.
   @throws std::exception if the frame `frame_id` is not valid.  */
  const math::RigidTransform<T>& GetPoseInWorld(FrameId frame_id) const;

  /** Reports the position of the frame indicated by `frame_id` relative to its
   parent frame. If the frame was registered with the world frame as its parent
   frame, this value will be identical to that returned by GetPoseInWorld().
   @note This is analogous to but distinct from
   SceneGraphInspector::GetPoseInParent(). In this case, the pose will *always*
   be relative to another frame.
   @throws std::exception if the frame `frame_id` is not valid.  */
  const math::RigidTransform<T>& GetPoseInParent(FrameId frame_id) const;

  /** Reports the position of the geometry indicated by `geometry_id` relative
   to the world frame.
   @throws std::exception if the geometry `geometry_id` is not valid.  */
  const math::RigidTransform<T>& GetPoseInWorld(GeometryId geometry_id) const;

  //@}

  /**
   @anchor collision_queries
   @name                Collision Queries

   These queries detect _collisions_ between geometry. Two geometries collide
   if they overlap each other and are not explicitly excluded through
   @ref scene_graph_collision_filter_manager "collision filtering".
   These algorithms find those colliding cases, characterize them, and report
   the essential characteristics of that collision.

   For two colliding geometries g_A and g_B, it is guaranteed that they will
   map to `id_A` and `id_B` in a fixed, repeatable manner, where `id_A` and
   `id_B` are GeometryId's of geometries g_A and g_B respectively.

   These methods are affected by collision filtering; element pairs that
   have been filtered will not produce contacts, even if their collision
   geometry is penetrating.     */
  //@{

  /** Computes the penetrations across all pairs of geometries in the world
   with the penetrations characterized by pairs of points (see
   PenetrationAsPointPair), providing some measure of the penetration "depth" of
   the two objects, but _not_ the overlapping volume.

   Only reports results for _penetrating_ geometries; if two geometries are
   separated, there will be no result for that pair. Geometries whose surfaces
   are just touching (osculating) are not considered in penetration. Surfaces
   whose penetration is within an epsilon of osculation, are likewise not
   considered penetrating. Pairs of _anchored_ geometry are also not reported.
   This method is affected by collision filtering.

   For two penetrating geometries g_A and g_B, it is guaranteed that they will
   map to `id_A` and `id_B` in a fixed, repeatable manner.

   <h3>Characterizing the returned values</h3>

   As discussed in the
   @ref query_object_precision_methodology "class's documentation", these tables
   document the support given by this query for pairs of geometry types and
   scalar. See the description in the link for details on how to interpret the
   tables' results. The query is symmetric with respect to shape *ordering*, the
   pair (ShapeA, ShapeB) will be the same as (ShapeB, ShapeA), so we only fill
   in half of each table.

   |           |   %Box  | %Capsule | %Convex | %Cylinder | %Ellipsoid | %HalfSpace |  %Mesh  | %Sphere |
   | --------: | :-----: | :------: | :-----: | :-------: | :--------: | :--------: | :-----: | :-----: |
   | Box       |  2e-15  |  ░░░░░░  |  ░░░░░  |  ░░░░░░░  |   ░░░░░░   |   ░░░░░░   |  ░░░░░  |  ░░░░░  |
   | Capsule   |  3e-5ᶜ  |   2e-5ᶜ  |  ░░░░░  |  ░░░░░░░  |   ░░░░░░   |   ░░░░░░   |  ░░░░░  |  ░░░░░  |
   | Convex    |  2e-15ᶜ |   3e-5ᶜ  | 2e-15ᶜ  |  ░░░░░░░  |   ░░░░░░   |   ░░░░░░   |  ░░░░░  |  ░░░░░  |
   | Cylinder  |  1e-3ᶜ  |   4e-5ᶜ  |  1e-3ᶜ  |   2e-3ᶜ   |   ░░░░░░   |   ░░░░░░   |  ░░░░░  |  ░░░░░  |
   | Ellipsoid |  4e-4ᶜ  |   2e-4ᶜ  |  4e-4ᶜ  |   2e-3ᶜ   |    5e-4ᶜ   |   ░░░░░░   |  ░░░░░  |  ░░░░░  |
   | HalfSpace |  6e-15  |   4e-15  | 3e-15ᶜ  |   4e-15   |   3e-15    |   throwsᵃ  |  ░░░░░  |  ░░░░░  |
   | Mesh      |    ᵇ    |    ᵇ     |    ᵇ    |     ᵇ     |      ᵇ     |     ᵇ      |    ᵇ    |  ░░░░░  |
   | Sphere    |  3e-15  |   5e-15  |  3e-5ᶜ  |   5e-15   |    2e-4ᶜ   |   3e-15    |    ᵇ    |  5e-15  |
   __*Table 1*__: Worst observed error (in m) for 2mm penetration between
   geometries approximately 20cm in size for `T` = `double`.

   |           |   %Box  | %Capsule | %Convex | %Cylinder | %Ellipsoid | %HalfSpace |  %Mesh  | %Sphere |
   | --------: | :-----: | :------: | :-----: | :-------: | :--------: | :--------: | :-----: | :-----: |
   | Box       | throwsᵈ |  ░░░░░░  |  ░░░░░  |  ░░░░░░░  |   ░░░░░░   |   ░░░░░░   |  ░░░░░  |  ░░░░░  |
   | Capsule   | throwsᵈ | throwsᵈ  |  ░░░░░  |  ░░░░░░░  |   ░░░░░░   |   ░░░░░░   |  ░░░░░  |  ░░░░░  |
   | Convex    | throwsᵈ | throwsᵈ  | throwsᵈ |  ░░░░░░░  |   ░░░░░░   |   ░░░░░░   |  ░░░░░  |  ░░░░░  |
   | Cylinder  | throwsᵈ | throwsᵈ  | throwsᵈ |  throwsᵈ  |   ░░░░░░   |   ░░░░░░   |  ░░░░░  |  ░░░░░  |
   | Ellipsoid | throwsᵈ | throwsᵈ  | throwsᵈ |  throwsᵈ  |   throwsᵈ  |   ░░░░░░   |  ░░░░░  |  ░░░░░  |
   | HalfSpace | throwsᵈ | throwsᵈ  | throwsᵈ |  throwsᵈ  |   throwsᵈ  |   throwsᵃ  |  ░░░░░  |  ░░░░░  |
   | Mesh      |    ᵇ    |    ᵇ     |    ᵇ    |     ᵇ     |      ᵇ     |     ᵇ      |    ᵇ    |  ░░░░░  |
   | Sphere    |  2e-15  |  3e-15   | throwsᵈ |   2e-15   |   throwsᵈ  |   2e-15    |    ᵇ    |  5e-15  |
   __*Table 2*__: Worst observed error (in m) for 2mm penetration between
   geometries approximately 20cm in size for `T` = @ref drake::AutoDiffXd
   "AutoDiffXd".

   - ᵃ Penetration depth between two HalfSpace instances has no meaning; either
       they don't intersect, or they have infinite penetration.
   - ᵇ Meshes are represented by the *convex* hull of the mesh, therefore the
       results for Mesh are assumed to be the same as for Convex.
   - ᶜ These results are computed using an iterative algorithm. For particular
       configurations, the solution may be correct to machine precision. The
       values reported here are confirmed, observed worst case answers.
   - ᵈ These results are simply not supported for
       `T` = @ref drake::AutoDiffXd "AutoDiffXd" at this time.

   <!-- Note to developers: the tests that support the assertions here are
   located in penetration_as_point_pair_characterize_test.cc. The values in this
   table should be reflected in the expected values there.  -->

   @returns A vector populated with all detected penetrations characterized as
            point pairs. The ordering of the results is guaranteed to be
            consistent -- for fixed geometry poses, the results will remain
            the same.
   @warning For Mesh shapes, their convex hulls are used in this query. It is
            *not* computationally efficient or particularly accurate.
   @throws std::exception if a Shape-Shape pair is in collision and indicated as
           `throws` in the support table above.  */
  std::vector<PenetrationAsPointPair<T>> ComputePointPairPenetration() const;

  /** Reports pairwise intersections and characterizes each non-empty
   intersection as a ContactSurface for hydroelastic contact model. The
   computation is subject to collision filtering.

   For two intersecting geometries g_A and g_B, it is guaranteed that they will
   map to `id_A` and `id_B` in a fixed, repeatable manner, where `id_A` and
   `id_B` are GeometryId's of geometries g_A and g_B respectively.

   In the current incarnation, this function represents an incomplete
   implementation. That has several implications, as described below:

     - This table shows which shapes can be declared for use in hydroelastic
       contact, and what compliance can be assigned.

       |   Shape   | Compliant | Rigid |
       | :-------: | :-------: | :---: |
       | Sphere    |    yes    |  yes  |
       | Cylinder  |    yes    |  yes  |
       | Box       |    yes    |  yes  |
       | Capsule   |    yes    |  yes  |
       | Ellipsoid |    yes    |  yes  |
       | HalfSpace |    yes    |  yes  |
       | Mesh      |    no     |  yes  |
       | Convex    |    no     |  yes  |

     - We do not currently support contact between two geometries with
       the *same* compliance type; one geometry *must* be compliant, and the
       other *must* be rigid. If geometries with the same compliance type
       collide, an exception will be thrown. More particularly, if such a
       geometry pair *cannot be culled* an exception will be thrown. No
       exception is thrown if the pair has been filtered.
     - The hydroelastic modulus (N/m^2) of each compliant geometry is set in
       ProximityProperties by AddSoftHydroelasticProperties().
     - The tessellation of the corresponding meshes is controlled by the
       resolution hint (where appropriate), as defined by
       AddSoftHydroelasticProperties() and AddRigidHydroelasticProperties().

   <h3>Scalar support</h3>

   This method provides support for both double and AutoDiffXd. Like with the
   other proximity queries, derivatives can only be introduced via geometry
   *poses*. We cannot differentiate w.r.t. geometric properties (e.g., radius,
   length, etc.)

   @param representation  Controls the mesh representation of the contact
                          surface. See
                          @ref contact_surface_discrete_representation
                          "contact surface representation" for more details.

   @returns A vector populated with all detected intersections characterized as
            contact surfaces. The ordering of the results is guaranteed to be
            consistent -- for fixed geometry poses, the results will remain
            the same.  */
  std::vector<ContactSurface<T>> ComputeContactSurfaces(
      HydroelasticContactRepresentation representation) const;

  /** Reports pairwise intersections and characterizes each non-empty
   intersection as a ContactSurface _where possible_ and as a
   PenetrationAsPointPair where not.

   This method can be thought of as a combination of ComputeContactSurfaces()
   and ComputePointPairPenetration(). For each geometry pair, we attempt to
   compute a ContactSurface. If that fails, rather than throwing, we attempt to
   characterize the contact as a point pair. If that fails, we throw. See the
   documentation of those constituent methods to understand the circumstances
   in which they fail.

   The ordering of the _added_ results is guaranteed to be consistent -- for
   fixed geometry poses, the results will remain the same.

   <h3>Scalar support</h3>

   The scalar support is a combination of the scalar support offered by
   ComputeContactSurfaces() and ComputePointPairPenetration(). This method
   supports double and AutoDiffXd to the extent that those constituent methods
   do.

   @param representation    Controls the mesh representation of the contact
                            surface. See
                            @ref contact_surface_discrete_representation
                            "contact surface representation" for more details.
   @param[out] surfaces     The vector that contact surfaces will be added to.
                            The vector will _not_ be cleared.
   @param[out] point_pairs  The vector that fall back point pair data will be
                            added to. The vector will _not_ be cleared.
   @pre Neither `surfaces` nor `point_pairs` is nullptr.
   @throws std::exception for the reasons described in ComputeContactSurfaces()
                          and ComputePointPairPenetration().
   @note The `surfaces` and `point_pairs` are output pointers in C++, but are
   return values in the Python bindings. */
  void ComputeContactSurfacesWithFallback(
      HydroelasticContactRepresentation representation,
      std::vector<ContactSurface<T>>* surfaces,
      std::vector<PenetrationAsPointPair<T>>* point_pairs) const;

  /** Applies a conservative culling mechanism to create a subset of all
   possible geometry pairs based on non-zero intersections. A geometry pair
   that is *absent* from the results is either a) culled by collision filters or
   b) *known* to be separated. The caller is responsible for confirming that
   the remaining, unculled geometry pairs are *actually* in collision.

   @returns A vector populated with collision pair candidates (the order will
            remain constant for a fixed population but can change as geometry
            ids are added/removed).  */
  std::vector<SortedPair<GeometryId>> FindCollisionCandidates() const;

  /** Reports true if there are _any_ collisions between unfiltered pairs in the
   world.
   @warning For Mesh shapes, their convex hulls are used in this query. It is
            *not* computationally efficient or particularly accurate.  */
  bool HasCollisions() const;

  //@}

  //---------------------------------------------------------------------------
  // TODO(DamrongGuoy): Write a better documentation for Signed Distance
  // Queries.
  /**
   @anchor signed_distance_query
   @name                   Signed Distance Queries

   These queries provide the signed distance between two objects. Each query
   has a specific definition of the signed distance being positive, negative,
   or zero associated with some notions of being outside, inside, or on
   the boundary.

   These queries provide bookkeeping data like geometry id(s) of the geometries
   involved and the important locations on the boundaries of these geometries.

   The signed distance function is a continuous function. Its partial
   derivatives are continuous almost everywhere.
  */
  //@{

  // TODO(DamrongGuoy): Refactor documentation of
  // ComputeSignedDistancePairwiseClosestPoints(). Move the common sections
  // into Signed Distance Queries.
  /**
   Computes the signed distance together with the nearest points across all
   pairs of geometries in the world. Reports both the separating geometries
   and penetrating geometries.

   This query provides φ(A, B), the signed distance between two objects A and B.

   If the objects do not overlap (i.e., A ⋂ B = ∅), φ > 0 and represents the
   minimal distance between the two objects. More formally:
   φ = min(|Aₚ - Bₚ|)
   ∀ Aₚ ∈ A and Bₚ ∈ B.
   @note The pair (Aₚ, Bₚ) is a "witness" of the distance.
   The pair need not be unique (think of two parallel planes).

   If the objects touch or overlap (i.e., A ⋂ B ≠ ∅), φ ≤ 0 and can be
   interpreted as the negative penetration depth. It is the smallest length of
   the vector v, such that by shifting one object along that vector relative to
   the other, the two objects will no longer be overlapping. More formally,
   φ(A, B) = -min |v|.
   s.t (Tᵥ · A) ⋂ B = ∅
   where Tᵥ is a rigid transformation that displaces A by the vector v, namely
   Tᵥ · A = {u + v | ∀ u ∈ A}.
   By implication, there exist points Aₚ and Bₚ on the surfaces of objects A and
   B, respectively, such that Aₚ + v = Bₚ, Aₚ ∈ A ∩ B, Bₚ ∈ A ∩ B. These points
   are the witnesses to the penetration.

   This method is affected by collision filtering; geometry pairs that
   have been filtered will not produce signed distance query results.

   For a geometry pair (A, B), the returned results will always be reported in
   a fixed order (e.g., always (A, B) and never (B, A)). The _basis_ for the
   ordering is arbitrary (and therefore undocumented), but guaranteed to be
   fixed and repeatable.

   Notice that this is an O(N²) operation, where N
   is the number of geometries remaining in the world after applying collision
   filter. We report the distance between dynamic objects, and between dynamic
   and anchored objects. We DO NOT report the distance between two anchored
   objects.

   <h3>Using maximum distance</h3>

   While the algorithm generally has O(N²) complexity in time and space, that
   can be reduced by the judicious use of the `max_distance` parameter. If
   `φ(A, B) > max_distance`, the pair (A, B) will not be included in the
   results (making it O(M²) in space where M < N). Furthermore, the broadphase
   culling algorithm can exploit `max_distance` to *cheaply* eliminate pairs of
   geometry that are "obviously" too far (likewise reducing the time
   complexity).

   Passing `max_distance = 0` is conceptually related to calling
   HasCollisions(). If contact is sparse (very few actually contacting geometry
   pairs), the two invocations will be quite similar in cost. However, the
   distinction between the two is that *this* method would have to include *all*
   pairs that satisfy `φ(A, B) <= 0`, whereas HasCollisions() stops at the
   first. So, the more actually colliding geometry pairs there are, the bigger
   the difference in cost between the two approaches.

   @anchor query_object_compute_pairwise_distance_table
   <h3>Characterizing the returned values</h3>

   As discussed in the
   @ref query_object_precision_methodology "class's documentation", this table
   documents the support given by this query for pairs of geometry types and
   scalar. See the description in the link for details on how to interpret the
   table results. The query is symmetric with respect to shape *ordering*, the
   pair (ShapeA, ShapeB) will be the same as (ShapeB, ShapeA), so we only fill
   in half the table.

   |           |   %Box  | %Capsule | %Convex | %Cylinder | %Ellipsoid | %HalfSpace |  %Mesh  | %Sphere |
   | --------: | :-----: | :------: | :-----: | :-------: | :--------: | :--------: | :-----: | :-----: |
   | Box       |  4e-15  |  ░░░░░░  |  ░░░░░  |  ░░░░░░░  |   ░░░░░░   |   ░░░░░░   |  ░░░░░  |  ░░░░░  |
   | Capsule   |  3e-6   |   2e-5   |  ░░░░░  |  ░░░░░░░  |   ░░░░░░   |   ░░░░░░   |  ░░░░░  |  ░░░░░  |
   | Convex    |  3e-15  |   2e-5   |  3e-15  |  ░░░░░░░  |   ░░░░░░   |   ░░░░░░   |  ░░░░░  |  ░░░░░  |
   | Cylinder  |  6e-6   |   1e-5   |   6e-6  |   2e-5    |   ░░░░░░   |   ░░░░░░   |  ░░░░░  |  ░░░░░  |
   | Ellipsoid |  9e-6   |   5e-6   |   9e-6  |   5e-5    |    2e-5    |   ░░░░░░   |  ░░░░░  |  ░░░░░  |
   | HalfSpace | throwsᵃ |  throwsᵃ | throwsᵃ |  throwsᵃ  |  throwsᵃ   |   throwsᵃ  |  ░░░░░  |  ░░░░░  |
   | Mesh      |    ᶜ    |    ᶜ     |    ᶜ    |     ᶜ     |      ᶜ     |   throwsᵃ  |    ᶜ    |  ░░░░░  |
   | Sphere    |  3e-15  |  6e-15   |   3e-6  |   5e-15   |    4e-5    |    3e-15   |    ᶜ    |  6e-15  |
   __*Table 3*__: Worst observed error (in m) for 2mm penetration/separation
   between geometries approximately 20cm in size for `T` = `double`.

   |           |   %Box  | %Capsule | %Convex | %Cylinder | %Ellipsoid | %HalfSpace |  %Mesh  | %Sphere |
   | --------: | :-----: | :------: | :-----: | :-------: | :--------: | :--------: | :-----: | :-----: |
   | Box       | throwsᵇ |  ░░░░░░  |  ░░░░░  |  ░░░░░░░  |   ░░░░░░   |   ░░░░░░   |  ░░░░░  |  ░░░░░  |
   | Capsule   | throwsᵇ |  throwsᵇ |  ░░░░░  |  ░░░░░░░  |   ░░░░░░   |   ░░░░░░   |  ░░░░░  |  ░░░░░  |
   | Convex    | throwsᵇ |  throwsᵇ | throwsᵇ |  ░░░░░░░  |   ░░░░░░   |   ░░░░░░   |  ░░░░░  |  ░░░░░  |
   | Cylinder  | throwsᵇ |  throwsᵇ | throwsᵇ |  throwsᵇ  |   ░░░░░░   |   ░░░░░░   |  ░░░░░  |  ░░░░░  |
   | Ellipsoid | throwsᵇ |  throwsᵇ | throwsᵇ |  throwsᵇ  |  throwsᵇ   |   ░░░░░░   |  ░░░░░  |  ░░░░░  |
   | HalfSpace | throwsᵃ |  throwsᵃ | throwsᵃ |  throwsᵃ  |  throwsᵃ   |   throwsᵃ  |  ░░░░░  |  ░░░░░  |
   | Mesh      |    ᶜ    |    ᶜ     |    ᶜ    |     ᶜ     |      ᶜ     |      ᵃ     |    ᶜ    |  ░░░░░  |
   | Sphere    |  2e-15  |  throwsᵇ | throwsᵇ |  throwsᵇ  |  throwsᵇ   |    2e-15   |    ᶜ    |  5e-15  |
   __*Table 4*__: Worst observed error (in m) for 2mm penetration/separation
   between geometries approximately 20cm in size for `T` =
   @ref drake::AutoDiffXd "AutoDiffXd".

   - ᵃ We don't currently support queries between HalfSpace and any other shape
       (except for Sphere).
   - ᵇ These results are simply not supported for
       `T` = @ref drake::AutoDiffXd "AutoDiffXd" at this time.
   - ᶜ Meshes are represented by the *convex* hull of the mesh, therefore the
       results for Mesh are the same as for Convex.

   <!-- Note to developers: the tests that support the assertions here are
   located in distance_to_shape_characterize_test.cc. The values in this
   table should be reflected in the expected values there.  -->

  <!-- Numerous values show deviation at 1e-6. This is a hard-coded iteration
   limit in proximity engine. Consider exposing that parameter to the query
   so that the user can choose to improve the answer.  -->

   <!-- TODO(SeanCurtis-TRI): Support queries of halfspace-A, where A is _not_ a
   halfspace. See https://github.com/RobotLocomotion/drake/issues/10905 -->

   @param max_distance  The maximum distance at which distance data is reported.

   @returns The signed distance (and supporting data) for all unfiltered
            geometry pairs whose distance is less than or equal to
            `max_distance`.
   @throws std::exception as indicated in the table above.
   @warning For Mesh shapes, their convex hulls are used in this query. It is
            *not* computationally efficient or particularly accurate.  */
  std::vector<SignedDistancePair<T>> ComputeSignedDistancePairwiseClosestPoints(
      const double max_distance =
          std::numeric_limits<double>::infinity()) const;

  /** A variant of ComputeSignedDistancePairwiseClosestPoints() which computes
   the signed distance (and witnesses) between a specific pair of geometries
   indicated by id. This function has the same restrictions on scalar report
   as ComputeSignedDistancePairwiseClosestPoints().

   <h3>Characterizing the returned values</h3>

   This method merely exercises the same mechanisms as
   ComputeSignedDistancePairwiseClosestPoints() for evaluating signed distance.
   Refer to @ref query_object_compute_pairwise_distance_table
   "the table for ComputeSignedDistancePairwiseClosestPoints()" for details.

   @throws std::exception if either geometry id is invalid, the pair (A, B) has
                          been marked as filtered, or according to the scalar
                          support table.
   @warning For Mesh shapes, their convex hulls are used in this query. It is
            *not* computationally efficient or particularly accurate.  */
  SignedDistancePair<T> ComputeSignedDistancePairClosestPoints(
      GeometryId geometry_id_A, GeometryId geometry_id_B) const;

  // TODO(DamrongGuoy): Improve and refactor documentation of
  // ComputeSignedDistanceToPoint(). Move the common sections into Signed
  // Distance Queries. Update documentation as we add more functionality.
  // Right now it only supports spheres and boxes.
  /**
   Computes the signed distances and gradients to a query point from each
   geometry in the scene.

   This query provides φᵢ(p), φᵢ:ℝ³→ℝ, the signed distance to the position
   p of a query point from geometry Gᵢ in the scene.  It returns an array of
   the signed distances from all geometries.

   Optionally you can specify a threshold distance that will filter out any
   object beyond the threshold. By default, we report distances from the query
   point to every object.

   This query also provides the gradient vector ∇φᵢ(p) of the signed distance
   function from geometry Gᵢ. Note that, in general, if p is outside Gᵢ, then
   ∇φᵢ(p) equals the unit vector in the direction from the nearest point Nᵢ on
   Gᵢ's surface to p. If p is inside Gᵢ, then ∇φᵢ(p) is in the direction from
   p to Nᵢ. This observation is written formally as:

   ∇φᵢ(p) = (p - Nᵢ)/|p - Nᵢ| if p is outside Gᵢ

   ∇φᵢ(p) = -(p - Nᵢ)/|p - Nᵢ| if p is inside Gᵢ

   Note that ∇φᵢ(p) is also defined on Gᵢ's surface, but we cannot use the
   above formula.

   <h3>Characterizing the returned values</h3>

   This table is a variant of that described in this
   @ref query_object_precision_methodology "class's documentation". The query
   evaluates signed distance between *one* shape and a point (in contrast to
   other queries which involve two shapes). Therefore, we don't need a matrix
   of shape pairs, but a list of shapes. Otherwise, the methodology is the same
   as described, with the point being represented as a zero-radius sphere.

   | Scalar |   %Box  | %Capsule | %Convex | %Cylinder | %Ellipsoid | %HalfSpace |  %Mesh  | %Sphere |
   | :----: | :-----: | :------: | :-----: | :-------: | :--------: | :--------: | :-----: | :-----: |
   | double |  2e-15  |   4e-15  |    ᵃ    |   3e-15   |    3e-5ᵇ   |    5e-15   |    ᵃ    |  4e-15  |
   | ADXd   |  1e-15  |   4e-15  |    ᵃ    |     ᵃ     |      ᵃ     |    5e-15   |    ᵃ    |  3e-15  |
   __*Table 5*__: Worst observed error (in m) for 2mm penetration/separation
   between geometry approximately 20cm in size and a point.

   - ᵃ Unsupported geometry/scalar combinations are simply ignored; no results
       are reported for that geometry.
   - ᵇ This uses an *iterative* algorithm which introduces a relatively large
       and variable error. For example, as the eccentricity of the ellipsoid
       increases, this error may get worse. It also depends on the location of
       the projection of the query point on the ellipsoid; the closer that point
       is to the high curvature area, the bigger the effect. It is not
       immediately clear how much worse the answer will get.

   @note For a sphere G, the signed distance function φᵢ(p) has an undefined
   gradient vector at the center of the sphere--every point on the sphere's
   surface has the same distance to the center.  In this case, we will assign
   ∇φᵢ(p) the unit vector Gx (x-directional vector of G's frame) expressed
   in World frame.

   @note For a box, at a point p on an edge or a corner of the box, the signed
   distance function φᵢ(p) has an undefined gradient vector.  In this case, we
   will assign a unit vector in the direction of the average of the outward
   face unit normals of the incident faces of the edge or the corner.
   A point p is considered being on a face, or an edge, or a corner of the
   box if it lies within a certain tolerance from them.

   @note For a box B, if a point p is inside the box, and it is equidistant to
   multiple nearest faces, the signed distance function φᵢ(p) at p will have
   an undefined gradient vector. There is a nearest point candidate associated
   with each nearest face. In this case, we arbitrarily pick the point Nᵢ
   associated with one of the nearest faces.  Please note that, due to the
   possible round off error arising from applying a pose X_WG to B, there is no
   guarantee which of the nearest faces will be used.

   @note The signed distance function is a continuous function with respect to
   the position of the query point, but its gradient vector field may
   not be continuous. Specifically at a position equidistant to multiple
   nearest points, its gradient vector field is not continuous.

   @note For a convex object, outside the object at positive distance from
   the boundary, the signed distance function is smooth (having continuous
   first-order partial derivatives).

   @param[in] p_WQ            Position of a query point Q in world frame W.
   @param[in] threshold       We ignore any object beyond this distance.
                              By default, it is infinity, so we report
                              distances from the query point to every object.
   @retval signed_distances   A vector populated with per-object signed distance
                              values (and supporting data) for every supported
                              geometry as shown in the table. See
                              SignedDistanceToPoint. */
  std::vector<SignedDistanceToPoint<T>>
  ComputeSignedDistanceToPoint(const Vector3<T> &p_WQ,
                               const double threshold
                               = std::numeric_limits<double>::infinity()) const;
  //@}


  //---------------------------------------------------------------------------
  /**
   @anchor render_queries
   @name                Render Queries

   The methods support queries along the lines of "What do I see?" They support
   simulation of sensors. External entities define a sensor camera -- its
   extrinsic and intrinsic properties and %QueryObject renders into the
   provided image.

   <!-- TODO(SeanCurtis-TRI): Currently, pose is requested as a transform of
   double. This puts the burden on the caller to be compatible. Provide
   specializations for AutoDiff and symbolic (the former extracts a
   double-valued transform and the latter throws). -->
   */
  //@{

  /** Renders an RGB image for the given `camera` posed with respect to the
   indicated parent frame P.

   @param camera                The camera to render from.
   @param parent_frame          The id for the camera's parent frame.
   @param X_PC                  The pose of the camera body in the parent frame.
   @param[out] color_image_out  The rendered color image. */
  void RenderColorImage(const render::ColorRenderCamera& camera,
                        FrameId parent_frame, const math::RigidTransformd& X_PC,
                        systems::sensors::ImageRgba8U* color_image_out) const;

  /** Renders a depth image for the given `camera` posed with respect to the
   indicated parent frame P.

   In contrast to the other rendering methods, rendering depth images doesn't
   provide the option to display the window; generally, basic depth images are
   not readily communicative to humans.

   @param camera                The camera to render from.
   @param parent_frame          The id for the camera's parent frame.
   @param X_PC                  The pose of the camera body in the parent frame.
   @param[out] depth_image_out  The rendered depth image. */
  void RenderDepthImage(const render::DepthRenderCamera& camera,
                        FrameId parent_frame, const math::RigidTransformd& X_PC,
                        systems::sensors::ImageDepth32F* depth_image_out) const;

  /** Renders a label image for the given `camera` posed with respect to the
   indicated parent frame P.

   @param camera                The camera to render from.
   @param parent_frame          The id for the camera's parent frame.
   @param X_PC                  The pose of the camera body in the parent frame.
   @param[out] label_image_out  The rendered label image. */
  void RenderLabelImage(const render::ColorRenderCamera& camera,
                        FrameId parent_frame, const math::RigidTransformd& X_PC,
                        systems::sensors::ImageLabel16I* label_image_out) const;


  /** Returns the named render engine, if it exists. The RenderEngine is
   guaranteed to be up to date w.r.t. the poses and data in the context. */
  const render::RenderEngine* GetRenderEngineByName(
      const std::string& name) const;

  //@}

 private:
  // SceneGraph is the only class that may call set().
  friend class SceneGraph<T>;
  // Convenience class for testing.
  friend class QueryObjectTest;

  // Access the GeometryState associated with this QueryObject.
  // @pre ThrowIfNotCallable() has been invoked prior to this.
  const GeometryState<T>& geometry_state() const;

  // Sets the query object to be *live*. That means the `context` and
  // `scene_graph` cannot be null.
  void set(const systems::Context<T>* context,
           const SceneGraph<T>* scene_graph) {
    DRAKE_DEMAND(context != nullptr);
    DRAKE_DEMAND(scene_graph != nullptr);
    state_.reset();
    context_ = context;
    scene_graph_ = scene_graph;
    inspector_.set(&geometry_state());
  }

  // Update all poses. This method does no work if this is a "baked" query
  // object (see class docs for discussion).
  void FullPoseUpdate() const {
    // TODO(SeanCurtis-TRI): Modify this when the cache system is in place.
    if (scene_graph_) scene_graph_->FullPoseUpdate(*context_);
  }

  // Reports true if this object is configured so that it can support a query.
  bool is_callable() const {
    const bool live_condition = context_ != nullptr && scene_graph_ != nullptr;
    const bool baked_condition = state_ != nullptr;
    // I.e., only one of the two conditions can be satisfied.
    return live_condition != baked_condition;
  }

  // Reports true if this object is in default configuration (not callable).
  bool is_default() const {
    return context_ == nullptr && scene_graph_ == nullptr && state_ == nullptr;
  }

  // Reports if the object can be copied; it must either be callable or default.
  bool is_copyable() const {
    return is_callable() || is_default();
  }

  // Throws an exception if the QueryObject is neither "live" nor "baked" (see
  // class docs for discussion).
  void ThrowIfNotCallable() const {
    if (!is_callable()) {
      throw std::runtime_error(
          "Attempting to perform query on invalid QueryObject.");
    }
  }

  // TODO(SeanCurtis-TRI): Consider an alternate formulation. This stores
  // pointers to context and systems which raise some red flags in order to
  // enable other systems to evaluate queries without having a copy of the
  // geometry system or its context.
  //
  // Alternatively, this could store the *index* of the system in its parent
  // diagram. The context shares the same index in the parent diagram context.
  // Then the LeafSystem desiring to perform a query would pass itself and its
  // own context in (along with the query parameters). The QueryObject would
  // use those and the index to get the SceneGraph and Context.
  //
  // Several issues:
  //  1. Leads to a clunky API (passing self and context into *every* query).
  //  2. The index value would be insufficient if the SceneGraph were buried
  //     in a diagram with its query object port exported in the diagram.
  // This is documented for future consideration, and should not necessarily be
  // interpreted as a guaranteed task.

  // The contents of the "live" query object. It has pointers to the system and
  // context from which it spawned. It uses these to compute geometry queries
  // on the current context (fully-dependent on context). These pointers must
  // _both_ be non-null for a live QueryObject and both be null for a baked
  // QueryObject.
  const systems::Context<T>* context_{nullptr};
  const SceneGraph<T>* scene_graph_{nullptr};

  SceneGraphInspector<T> inspector_;

  // When a QueryObject is copied to a "baked" version, it contains a fully
  // updated GeometryState. Copies of bakes all share the same version.
  std::shared_ptr<const GeometryState<T>> state_{};
};

}  // namespace geometry
}  // namespace drake
