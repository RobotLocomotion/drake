/** @defgroup collision_concepts Collision Concepts

In the real world, we can rely on the axiom that no two objects can occupy the
same space at the same time; the universe provides this constraint for free. In
simulation, this cannot be taken for granted. In fact, without active efforts
to introduce this constraint, bodies in a simulated dynamical system would pass
through each other without regard to this fundamental physical principle. Within
the parameters of our models, we need to enforce the constraint that no two
objects occupy the same space.

Collision detection consists of geometric techniques which allow us to detect
when bodies occupy the same space and, to a certain extent, quantify the degree
to which they invalidate the collision constraint.

Next topic: @ref collision_filter_concepts
**/

/** @defgroup collision_filter_concepts Collision Filter Concepts
@ingroup collision_concepts

Collision filters are a purely *non-physical* concept.  It is about
computational efficiency or accommodating models which rely on approximations
of physical quantities which might otherwise interfere with a meaningful
interpretation of the results of geometric computations.

For example, take a robot arm.  The components of the arm consist of intricately
designed surfaces. In general, the more complex a surface is, the more complex
the algorithms are for evaluating properties for those surfaces. So, rather than
including the exact surface representation for the links in the robot arm, we
replace them with simpler approximations. We may pay a cost with this
simplification. The approximations are generally conservative and coarser than
the actual robot arm surfaces.  With these approximations, it is possible for
two links in the arm to *report* collision even if the actual links wouldn't
be in collision at the same configuration.  This is *particularly* likely for
links connected by a common joint.  In this case, reported collisions would be
meaningless and it would be best if we simply *filtered* them out.

Alternatively, consider using filters as an optimization process. We may only
be focused on a single aspect of the robot, e.g., foot-step planning.  As such,
we may decide that the only contacts/collisions that are important are between
the feet and the ground.  An arm that swings and strikes the torso can safely
be ignored.  In this case, we would want to filter out all potential collisions
except for those between feet and ground.

Drake provides a means for filtering collisions in cases such as these.  In
fact, Drake uses two different mechanisms to achieve this effect:
@ref collision_clique and @ref collision_filter_group. The two representations
are @ref collision_filter_mapping "related but complementary".


Before looking at the details of the collision filter mechanisms, there are a
few key principles to make note of.

 <!-- TODO: Change this with the advent of GeometryWorld to reflect current
 state -->
\section collision_elements Collision Elements

_Collision Elements_ are the basis for performing geometric queries. A collision
element is comprised of a transform and a geometric shape.  The shape can be
any of a number supported primitives (e.g., sphere, box, triangular mesh, etc.).
The transform moves the shape from its canonical space to its *parent's* space.
A collision element's parent is a rigid body.

Each body has *zero* or more collision elements associated with it.  If there is
no collision element  associated with a body, then that body is effectively
invisible with respect to contact.  However, a rigid body can have multiple
collision elements as children.

One can think and talk about "colliding bodies".  In implementation, that would
be true, if and only if at least one child collision element in each body is
colliding.  Similarly, the abstraction can be taken up to the *model* leve.
A model would be a sub-tree of rigid bodies.  Model A can be said to be
colliding with model B if there is a body in A that is colliding with a body in
B.

NOTE: collision elements are independent of geometry used for other purposes,
such as visualization or even sensor simulation.

\section collision_queries Collision Queries and Filtering

Fundamentally, all collisions consist of pairs of collision elements (we assume
that a collision element *cannot* collide with itself.)  That means, collision
filtering is about *pairs* of collision elements. It is not meaningful to talk
about filtering a single collision element. If the *intent* is to suggest that
that single element should not be considered in *any* collisions, we are
implicitly referring to filtering out all *pairs* of collision elements which
include that element.  (Alternatively, one could simply omit the collision
element in the first place.)

\section collision_graph Collision Filters and Graphs

It is worthwhile to think about collision filtering in a graph-theory context.
We define a graph `G = {V, E}` where:

  - `V = {v₁, v₂, ..., vₙ}` is the set of graph vertices.  There is one graph
  vertex for each collision element in the simulation.
  - 'E = {e₁, e₂, ..., eₘ}' is the set of graph edges.  The edge eᵢⱼ exists if
  the pair of collision elements (vᵢ, vⱼ) is a filtered pair.

Next topic: @ref collision_clique
**/

/** @defgroup collision_clique Collision Cliques
 @ingroup collision_filter_concepts

\section clique_principle  The Principle

Collision cliques filter collisions by defining a group (or *clique*) of
collision elements. Membership in a clique precludes collision with any other
member of that same clique.
The underlying idea of collision cliques is that of the mathematical
<a href="https://en.wikipedia.org/wiki/Clique_(graph_theory)">clique</a>. This
mathematical concept applies very naturally to the graph-based discussion of
collision filtering.  A clique in a graph is, loosely, a completely connected
subgraph; there is an edge between each of the nodes in the graph. For
collision filtering, that implies that all of the pairs that can be created
from combinations of the clique members are filtered out.

It is common to simulate a robot such that collisions between any two bodies
of the robot are ignored (see @ref collision_filter_sdf_semantics).  This
behavior is easily represented with collision cliques; the whole robot belongs
to a single clique.

\section clique_impl The Implementation

In Drake, cliques are represented as integer identifiers. Each collision element
tracks the set of cliques to which it belongs. When a pair of collision elements
are considered for collision computation, their two sets of cliques are
intersected; a non-empty resultant set means that this pair is filtered.
The cost of the intersection is O(n + m), where the two elements belong to
n and m different cliques, respectively.

Drake automatically applies cliques in two different cases:
    -# If a body has multiple child collision elements, all of those elements
    are put into the same click.
    -# The collision elements of two bodies are all put into the same click if:
       -# the bodies are *adjacent* (one is the parent of the other in the
       tree), and
       -# the joint connecting them is *not* a floating joint.

There are several implications of this:
    -# A body that is represented by multiple, overlapping collision elements
    will *not* report meaningless collisions between those elements.
    -# Collisions between adjacent bodies will *not* provide constraints on the
    valid range of joint motion.  These constraints will have to act directly
    on the joint state value.

@warning There is currently no interface for manipulating cliques
programmatically. This will be implemented when the use case becomes clear.

Next topic: @ref collision_filter_group
**/

/** @defgroup collision_filter_group Collision Filter Groups
 @ingroup collision_filter_concepts

\section cfg_principle  The Principle

Collision filter groups represent collision filtering by defining groups of
collision elements.  That *group* then defines "ignore" relationships with
other groups.  A collision element's membership to a group does not guarantee
that that element will be included in *any* filtered pairs. To be filtered,
it must:
   -# Appear in at least one collision filter group, and
   -# Be flagged as an "ignore" group by *another* collision filter group with
   at least one member that is *not* the collision element in question.

The collision filter can easily be used to represent a
@ref clique_principle "clique"; simply add all of the collision elements to a
single group and have that group ignore itself.  But collision filter groups'
best strength arises from thinking about *classes* of collision elements. It
serves as a short hand for communicating that no collision element in one class
can collide with elements belonging to another class. Each unique collision
filter group defines such a class.

This can be used to indicate that two different models cannot collide. Simply
create two collision filter groups, assigning all of the collision elements of
one model to the first group, and all elements of the second model to the second
group.  Set either group to *ignore* the other (or both, redundancy doesn't
hurt.)

\section cfg_impl The Implementation

Collision filter groups are implemented as a pair of fixed-width bitmasks. Each
collision filter group corresponds to a single bit.  One bitmask represents the
set of groups a collision element belongs to.  The second represents the groups
that this collision element *ignores*.  When a pair of collision elements are
considered for collision, the membership mask of each is ANDed with the
ignore mask of the other.  If either operations produces a non-zero result, the
pair is filtered.  The cost of the comparison is constant, but related to the
bitmask width 

The width is defined at compile time and is defined by
DrakeCollision::kMaxNumCollisionFilterGroups. The fixed width is a performance
optimization but it means that there is a finite number of collision filter
groups available in the system. Each RigidBodyTree instance has its *own*
space of collision filter group identifiers.
<!-- TODO(SeanCurtis-TRI): This will have to be handled carefully in Geometry
 World. Each source of collision elements will get its own space of filters.
 Sources will need to be differentiated. -->

Collision filter groups are instantiated by specifying them in URDF files as
follows:

<pre>
    <collision_filter_group name="group1">
        <member link="body1"/>
        <member link="body2"/>
        <ignored_collision_filter_group collision_filter_group="group2"/>
        <ignored_collision_filter_group collision_filter_group="group3"/>
    </collision_filter_group>
</pre>

This XML-snippet declares a collision filter group named `group1`. It has
two members, `body1` and `body2`.  It ignores two groups: `group2` and
`group3`.  It is not considered an error to ignore a non-existing group, but
it is considered an error to reference a link that hasn't been defined.

By default, all elements belong to a common, universal group which corresponds
to bit zero. A collision element can be rendered "invisible" by having it
ignore this universal group.

@warning Beyond instantiating the collision filter groups from parsing a URDF
file, there is currently no interface for programmatically altering group
membership or ignore relationships.

Next topic: @ref collision_filter_mapping
**/

/** @defgroup collision_filter_mapping Relationship Between Collision Filter Implementations
 @ingroup collision_filter_concepts

In principle, @ref collision_clique "collision cliques" and
@ref collision_filter_group "collision filter groups" are equivalent
implementations of filtering.  In theory, there is no filtering scenario that
can be represented by one that *cannot* be represented by the other. This
might suggest that a single representation is sufficient.  However, in practice
this proves not to be true.  Each representation has its particular strengths
and weaknesses.  The two mechanisms work well in concert to implement an
overall collision filtering policy.

This can best be illustrated through several examples of mapping filtering
scenarios to the two representations.

@section grp_clique_ex1 Example 1: Adjacent links

In the discussion of @ref collision_clique "collision cliques", we have seen
that we want to filter collisions between collision elements that belong to
*adjacent* bodies.

<b>Collision Cliques</b> As previously indicated, we can simply instantiate a
unique clique identifier and assign it to each of the collision elements.

<b>Collision Filter Groups</b> The simplest way to implement this using
collision filter groups is to create a new group which ignores itself and
make all of the collision elements of both bodies members of that group.

<b>Comparision</b> Both representations offer a straightforward implementation.
However, because of the fixed-width bitmask, we have a finite number of
collision filter groups. For a *single* robot with `k` links, we would need, on
the average `k - 1` groups to handle this case.  If we extend this to multiple
instances of that robot, we can easily consume the majority of the available
collision filter groups, just in accounting for link adjacency.

<b>Verdict</b> Prefer cliques.

@section grp_clique_ex2 Example 2: Disallow model self collision

In this case, we want to implement the @ref collision_filter_file_semantics
"sdf semantics" and turn off self collision for the whole robot.

<b>Collision Cliques</b> Same as @ref grp_clique_ex1 "before".  Create a unique
clique identifier and assign all collision elements in the model to that clique.

<b>Collision Filter Groups</b> Again, same as @ref grp_clique_ex1 "before".
Assign all elements to a unique group that ignores itself.

<b>Comparison</b> This scenario is very similar to the one before.  Both cases
deal with a *clique*, in the mathematical sense.  A set of collision elements
where all pair-wise combinations of elements are filtered.  It is unsurprising
that there is an obvious mapping between these cases and the clique filtering
mechanism.

<b>Verdict</b> For the same reasons as before, collision cliques are preferred.

@section grp_clique_ex3 Example 3: URDF `<collision_filter_group>` Tag Specifies Two Non-colliding Groups

In this case, the filtering policy is not *automatic*. It is specified by
definitions of groups in a URDF file.  In this case, we'll deal with a
theoretical file with two groups (assuming all bodies/links have been
properly defined prior to this XML snipped).

<pre>
    <collision_filter_group name="groupA">
        <member link="body1"/>
        <member link="body2"/>
        <ignored_collision_filter_group collision_filter_group="groupB"/>
    </collision_filter_group>
    <collision_filter_group name="groupB">
        <member link="body3"/>
        <member link="body4"/>
        <ignored_collision_filter_group collision_filter_group="groupA"/>
    </collision_filter_group>
</pre>

The filtering implications of these groups is that we filter collision between
the bodies (1, 3), (1, 4), (2, 3), and (2, 4) (the cartesian product of the
two groups).

<b>Collision Cliques</b> There is a simple mapping from these groups to cliques.
For each member in `groupA`, create a clique id and assign it to that member
and *all* the members in `groupB`.  We are essentially implementing the
cartesian coordinates.  This greedy approach will instantiate `N` different
cliques (where `N = |groupA|`).  And the number of cliques that each member of
of `groupB` has will increase by that same number `N`.  Remember that the
evaluation of cliques for filtering is linear in the number of cliques to which
a body belongs.

Ideally, we would want to reduce the number of cliques to which a body belongs.
To do so, we would have to look at the full graph of vertices and edges. The
optimal number of cliques is related to finding the
<a href="https://en.wikipedia.org/wiki/Clique_(graph_theory)">maximal
cliques</a> of the graph. This is known to be an NP-complete problem.

<b>Collision Filter Groups</b> The collision filter implementation of this is
straightforward and obvious and requires no elaboration.

<b>Comparision</b> Collision filtering based on *classes* of bodies fits
naturally with the collision filter group model.  Its representation is compact
and its runtime cost remains unchanged.  The efficacy of the clique approach
depends on the size of the groups and, handled greedily, could lead to an
explosion in redundant clique assignments.

<b>Verdict</b> Prefer collision filter groups.

These relative strengths and weaknesses are the reasons we maintain both systems
simultaneously.  Collision cliques provide an (in practice) unbounded set of
filter parameters and are well suited for mutually exclusive sets of bodies.
Collision filter groups offer a compact representation with O(1) computation
costs and are ideally suited for excluding large sets of of bodies from
collsion computation.

Next topic: @ref collision_filter_file_semantics
**/

/** @defgroup collision_filter_file_semantics Input File Collision Semantics
 @ingroup collision_filter_concepts

@section collision_filter_sdf_semantics SDF File Collision Semantics

 @todo Document the SDF collision filter semantics.

@section collision_filter_urdf_semantics URDF File Collision Semantics

 @todo Document the URDF collision filtering semantics.

 Next topic: @ref collision_filter_future
**/

/** @defgroup collision_filter_future Future Collision Filter Features
 @ingroup collision_filter_concepts

   - High-level filtering semantics (Specify colliding/non-colliding state using
   high level abstractions: tree, model, body, body collection, collision
   element.
   - The ability to abstractly specify filtering semantics independent of
   representation based on a series of refining *rules*.  e.g.,
      - `SetNoSelfCollision(tree);`
         `//` nothing in the tree collides with itself.
      - `SetNoCollisions(tree1, tree2);`
         `//` nothing in tree 1 collides with anything in tree 2.
      - `SetCanCollide(tree1->bodies[i], tree1->bodies[j]);`
         `//` Exception to previous rule, such that body `i` in tree 1 can
         collide with body `j` in tree 2.
 **/