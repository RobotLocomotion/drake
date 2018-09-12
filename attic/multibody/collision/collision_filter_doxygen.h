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

Collision filters are a purely _non-physical_ concept.  They are about
computational efficiency or accommodating models which rely on approximations
of physical quantities which might otherwise interfere with a meaningful
interpretation of the results of geometric computations.

For example, consider a robot arm.  The components of the arm can consist of
intricate surfaces. In general, the more complex a surface is, the more complex
the algorithms are for evaluating properties of those surfaces. Rather than
including the exact surface representation for the links in the robot arm, we
replace them with simpler approximations. We may pay a cost with this
simplification. The approximations are generally conservative and coarser than
the actual robot arm surfaces.  With these approximations, it is possible for
two links in the arm to _report_ collision even if the actual links wouldn't
be in collision at the same configuration.  This is _particularly_ likely for
links connected by a common joint.  In this case, reported collisions would be
meaningless and it would be best if we simply _filtered_ them out.

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

 <!-- TODO(SeanCurtis-TRI): Change this with the advent of SceneGraph to
 reflect current state -->
\section collision_elements Collision Elements

_Collision Elements_ are the basis for performing geometric collision queries.
Each body has _zero_ or more collision elements associated with (or _fixed_ to)
it.  If there is no collision element associated with a body, then that body is
effectively invisible with respect to contact.  On the other hand, a rigid body
can have multiple collision elements -- a common solution for representing
complex real-world shapes with a union of computationally efficient, simple
shapes.

A collision element is comprised of a geometric shape and a transform.  The
shape can be one of a number of supported primitives (e.g., sphere, box,
triangular mesh, etc.).

The collision element's geometric shape is defined with respect to its own local
frame `E` (e.g., a cylinder could be centered on `E`'s origin and the axis of
the cylinder could be aligned with `E`'s y-axis).  The body to which the
collision element belongs has a body frame `B`.  The collision element has a
transform, `X_BE`, which positions the `E` frame (and, therefore, the geometric
shape) with respect to the `B` frame.

Although collisions are defined strictly in terms of collision elements, it is
often convenient to speak about colliding _bodies_ `A` and `B` as shorthand for
some collision element fixed to `A` colliding with some collision element fixed
to `B`. Similarly, a model is a collection of bodies and we can speak of
colliding _models_ `M` and `N` as shorthand meaning that some body in `M` is
colliding with some body in `N`.

NOTE: collision elements do not necessarily have anything to do with geometry
used for other purposes, such as visualization or sensor simulation.

\section collision_queries Collision Queries and Filtering

Fundamentally, all collisions consist of _pairs_ of collision elements.
Therefore, collision filtering similarly involves _pairs_ of collision elements.
It is not meaningful to talk about filtering a single collision element. If the
_intent_ is to eliminate that single element from consideration in *any*
collisions, we are implicitly referring to filtering out all *pairs* of
collision elements which include that element.  (Alternatively, one could simply
omit the collision element in the first place.)

\section collision_graph Collision Filters and Graphs

It is worthwhile to think about collision filtering in a graph-theory context.
We define a graph `G = {V, E}` where:

  - `V = {v₁, v₂, ..., vₙ}` is the set of graph vertices.  There is one graph
  vertex for each collision element in the simulation.
  - `E = {e₁, e₂, ..., eₘ}` is the set of graph edges.  The edge eᵢⱼ exists if
  the pair of collision elements (vᵢ, vⱼ) is a _filtered_ pair.

This representation is particularly helpful in discussing @ref collision_clique
and in comparing @ref collision_clique with @ref collision_filter_group.

Next topic: @ref collision_clique
**/

/** @defgroup collision_clique Collision Cliques
 @ingroup collision_filter_concepts

\section clique_principle  The Principle

Collision cliques filter collisions by defining a group (or _clique_) of
collision elements. Membership in a clique precludes collision with any other
member of that same clique.
The underlying idea of collision cliques is that of the
<a href="https://en.wikipedia.org/wiki/Clique_(graph_theory)">mathematical clique</a>.
This mathematical concept applies very naturally to the graph-based discussion
of collision filtering.  A clique in a graph is, loosely, a completely connected
subgraph; there is an edge between each of the nodes in the subgraph. For
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
The cost of the intersection is `O(n + m)`, where the two elements belong to
`n` and `m` different cliques, respectively. This is the worst case. In
principle, it would come up whenever the pair of collision elements are _not_
filtered by a clique.  In practice, this cost is mitigated as follows:

    <!-- TODO(SeanCurtis-TRI): Confirm that this is still true for FCL -->
   -# Clique comparisons are only evaluated when the underlying bounding
     volume hierarchy (BVH) is restructured.  In most cases, only a small, local
     portion of the BVH is restructured at any time, reducing the number of
     actual clique tests done.
   -# The representation of the clique sets to which a collision element belongs
     provides opportunities to short-circuit the full `O(n + m)` test in common
     cases.

Drake automatically applies cliques in two different cases:
    -# If a body has multiple collision elements, all of those elements
    are put into the same clique.
    -# The collision elements of two bodies are all put into the same clique if:
       -# the bodies are _adjacent_ (one is the parent of the other in the
       tree), and
       -# the joint connecting them is _not_ a floating joint.

There are several implications of this:
    -# A body that is represented by multiple, overlapping collision elements
    will _not_ report meaningless collisions between those elements.
    -# Collisions between adjacent bodies will _not_ provide constraints on the
    valid range of joint motion.  These constraints will have to act directly
    on the joint state value.

@note These heuristics _may_ lead to important collision element pairs
being filtered. There is currently no interface for manipulating cliques
programmatically. This will be implemented when the use case becomes clear.
However, there is a workaround. As an example, consider two adjacent bodies, `A`
and `B`. Their collision elements will automatically be put into a common
clique. However, we may want collisions between element `E` on `B` and collision
elements on `A` to _not_ be filtered. We can create a massless body `Z` to
hold `E` and use a weld (fixed) joint to attach `Z` to `B`. `Z` will not be
adjacent to A, so `E` will interact with `A`'s elements.

Next topic: @ref collision_filter_group
**/

/** @defgroup collision_filter_group Collision Filter Groups
 @ingroup collision_filter_concepts

\section cfg_principle  The Principle

A collision filter group represents collision filtering by defining a collection
of collision elements.  That _group_ then defines "ignore" relationships with
other groups.  A collision element's membership in a group does not guarantee
inclusion in _any_ filtered collision pairs. This group must be included in at
least one group's ignore list; inclusion in its own ignore list would be
sufficient.)

A collision-filter-group-based filtering system's unique benefit
arises from thinking about _classes_ of collision elements. It
serves as a short hand for communicating that no collision element in one class
can collide with elements belonging to another class. Each unique collision
filter group defines such a class.

This can be used to indicate that two different models cannot collide. Simply
create two collision filter groups, assigning all of the collision elements of
one model to the first group, and all elements of the second model to the second
group.  Set either group to _ignore_ the other (or both, redundancy doesn't
hurt).

\section cfg_impl Declaring Collision Filter Groups

\subsection cfg_impl_in_file Declaring collision filter groups in URDF/SDF files

Collision filter groups can be instantiated by specifying them in URDF/SDF
files.

_Declaration_

```xml
    <collision_filter_group name="group1">
        <member link="body1"/>
        <member link="body2"/>
        <ignored_collision_filter_group collision_filter_group="group2"/>
        <ignored_collision_filter_group collision_filter_group="group3"/>
    </collision_filter_group>
```

This XML-snippet illustrates the syntax for declaring a collision filter group.
It declares a collision filter group named `group1`. It has
two members, `body1` and `body2` (although it could have any number of links).
This is short-hand for communicating that the _collision elements_ of `body1`
and `body2` belong to `group1`. Furthermore, `group1` ignores two groups:
`group2` and `group3`.  It is not considered an error to ignore a non-existing
group, but it is considered an error to reference a link that hasn't been
defined.

One possible use of collision filter groups is to create a set of collision
elements which _cannot_ collide with each other (i.e., no self-collisions in
the group). This is achieved by having the group ignore _itself_. E.g.,

```xml
    <collision_filter_group name="no_self_collision_group">
        <member link="body1"/>
        <member link="body2"/>
        <ignored_collision_filter_group collision_filter_group="no_self_collision_group"/>
    </collision_filter_group>
```

Urdf files support the definition of a single robot. As such, all
`<collision_filter_group>` (`<cfg>` for brevity) tags are children of the
 `<robot>` tag.

Sdf files support the definition of multiple robots (aka "models"). As such, the
`<cfg>` tags are children of each `<model>` tag. The `<cfg>` tags can only
include links that are defined *in that model* in its membership lists. The
implication of that is collision filter groups defined in sdf files contain
bodies from a single model only. However, a collision filter group in *one*
model can ignore a collision filter group in another model. See
<a href="https://github.com/RobotLocomotion/drake/blob/master/multibody/collision/test/multi_model_groups.sdf">multi_model_groups.sdf</a>
for an example.

\subsection cfg_impl_in_code Declaring collision filter groups in code

In addition to parsing the collision filter groups from the urdf/sdf files,
there is an API on the RigidBodyTree that allows manipulation. However, the API
is constrained. The workflow is as follows:

 - Add one or more bodies
 - Add one or more collision filter groups
 - Add bodies to the collision filter groups
 - Add groups to the "ignore set" of the created groups
 - Compile the tree

The API only supports _building up_ collision filter groups and _adding_ bodies
to groups. There is no API for removing bodies from previously declared groups
or removing groups from another group's ignore list. The API is strictly
additive.

Furthermore, once declared collision filter groups have been compiled (via
invocation of RigidBodyTree::compile()) the _names_ can no longer be referenced.
This is what allows the same urdf/sdf file to be read multiple times but not
have the named collision filter groups in the file end up spanning all of the
models.

However, Drake _does_ support some post-compile filtering modifications.
These include:
  - Adding a new collision filter group. The previously compiled bodies and
    newly added, uncompiled bodies can all be assigned to the group. However,
    it can't ignore previously compiled groups (as those names no longer exist).
  - Attaching a new body with collision elements to a previously compiled
    body (via a joint). If both bodies have collision elements, collisions
    between elements on the new and old body will be appropriately filtered.
  - Compiled bodies can have _new_ collision elements added. They will inherit
    the same filter behavior as the other elements on the compiled body. (And
    pick up any new filter behavior after compilation as appropriate.)

In _typical_ use, parsing a model from a file calls RigidBodyTree::compile() and
any named collision filter groups in that file cannot be subsequently
referenced. The parsing API provides a mechanism for suppressing the automatic
tree compilation. There is a parallel set of parse functions which take an
additional `bool`, indicating if the tree should be compiled (`true`) or not
upon parse completion. By passing `false`, the parsing process will _not_
compile the tree, but it becomes the caller's responsibility to make sure that
RigidBodyTree::compile() is invoked before doing any work on the tree. For
example:

```
 const bool do_compile = false;
 const std::string file_name = "some_file.urdf";
 RigidBodyTree<double> tree;
 AddModelInstanceFromUrdfFileToWorld(file_name, kRollPitchYaw, do_compile, &tree);
 tree.DefineCollisionFilterGroup("new_group");  // doesn't already exist.
 tree.AddCollisionFilterGroupMember("new_group", "body1", 0); // Assumes known body name and model instance id.
 tree.AddCollisionFilterIgnoreTarget("new_group", "group_from_urdf");
 tree.compile();
```

\subsection cfg_impl_code In-code representation

Collision filter groups are implemented as a pair of fixed-width bitmasks,
stored with each collision element. Each
collision filter group corresponds to a single bit.  One bitmask represents the
set of groups a collision element belongs to.  The second represents the groups
that this collision element _ignores_.  When a pair of collision elements are
considered for collision, the membership mask of each is ANDed with the
ignore mask of the other.  If either operations produces a non-zero result, the
pair is filtered.  The cost of the comparison is constant, but related to the
bitmask width.

The width is defined at compile time and is defined by
drake::multibody::collision::kMaxNumCollisionFilterGroups. The fixed width is a
performance optimization but it means that there is a finite number of
collision filter groups available in the system. Each RigidBodyTree instance
has its _own_ space of collision filter group identifiers.
<!-- TODO(SeanCurtis-TRI): This will have to be handled carefully in Geometry
 World. Each source of collision elements will get its own space of filters.
 Sources will need to be differentiated. -->

By default, all elements belong to a common, universal group which corresponds
to bit zero. A collision element can be rendered "invisible" by assigning it
to a group that ignores this universal group.

Next topic: @ref collision_filter_mapping
**/

/** @defgroup collision_filter_mapping Relationship Between Collision Filter Implementations
 @ingroup collision_filter_concepts

In principle, @ref collision_clique "collision cliques" and
@ref collision_filter_group "collision filter groups" are equivalent
implementations of filtering.  In theory, there is no filtering scenario that
can be represented by one that _cannot_ be represented by the other. This
might suggest that a single representation is sufficient.  For example,
collision filter groups can easily be used to represent cliques; simply add all
of the clique's collision elements to a single group and have that group
ignore itself.

However, in practice the _implementation_ of the two filtering mechanisms
introduces differences.  Each representation has its particular strengths
and weaknesses.  The two mechanisms work well in concert to implement an
overall collision filtering policy.

This can best be illustrated through several examples of mapping filtering
scenarios to the two representations.

@section grp_clique_ex1 Example 1: Adjacent Links

In the discussion of @ref collision_clique "collision cliques", we have seen
that we want to filter collisions between collision elements that belong to
_adjacent_ bodies.

<b>Collision Cliques</b> As previously indicated, we can simply instantiate a
unique clique identifier and assign it to all of the two bodies` collision
elements.

<b>Collision Filter Groups</b> The simplest way to implement this using
collision filter groups is to create a new group which ignores itself and
make all of the collision elements of both bodies members of that group.

<b>Comparison</b> Both representations offer a straightforward implementation.
However, because of the fixed-width bitmask, we have a finite number of
collision filter groups. For a _single_ robot with `k` links, we would need, on
the average, `k - 1` groups to handle this case.  If we extend this to multiple
instances of that robot, we can easily consume the majority of the available
collision filter groups, just in accounting for link adjacency.

<b>Verdict</b> Prefer cliques.

@section grp_clique_ex2 Example 2: Disallow Model Self-collision

In this case, we want to implement the @ref collision_filter_file_semantics
"sdf semantics" and turn off self collision for the whole robot.

<b>Collision Cliques</b> Similar to @ref grp_clique_ex1 "example 1".  Create a
unique clique identifier and assign all collision elements in the _model_ to
that clique.

<b>Collision Filter Groups</b> Again, same as @ref grp_clique_ex1 "example 1".
Assign all collision elements in the _model_ to a unique group that ignores
itself.

<b>Comparison</b> This scenario is very similar to the one before.  Both cases
deal with a _clique_, in the mathematical sense.  A set of collision elements
where all pair-wise combinations of elements are filtered.  It is unsurprising
that there is an obvious mapping between these cases and the clique filtering
mechanism.

<b>Verdict</b> For the same reasons as before, collision cliques are preferred.

@section grp_clique_ex3 Example 3: URDF Specifies Two Non-colliding Groups

In this case, the filtering policy is not _automatic_. It is specified by
definitions of groups in a URDF file.  In this case, we'll deal with a
theoretical file with two groups (assuming all bodies/links have been
properly defined prior to this XML snippet).

```xml
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
```

The filtering implications of these groups is that we filter collisions between
all the collision elements of the bodies (1, 3), (1, 4), (2, 3), and (2, 4)
(i.e., the Cartesian product of the two groups).

<b>Collision Cliques</b> There is a simple mapping from these groups to cliques.
For each member in `groupA`, create a clique id and assign it to that member
and _all_ the members in `groupB`.  We are essentially implementing the
Cartesian product.  This greedy approach will instantiate `N` different
cliques (where `N = |groupA|`).  And the number of cliques that each member of
of `groupB` has will increase by that same number `N`.  Remember that the
evaluation of cliques for filtering is linear in the number of cliques to which
a collision element belongs.

Ideally, we would want to reduce the number of cliques to which a body belongs.
To do so, we would have to look at the full graph of vertices and edges. The
optimal number of cliques is related to finding the
<a href="https://en.wikipedia.org/wiki/Clique_(graph_theory)">maximal
cliques</a> of the graph. This is known to be an NP-complete problem.

<b>Collision Filter Groups</b> The collision filter implementation of this is
straightforward. Two groups are created (we'll call them `A` and `B`). The
collision elements of `body1` and `body2` belong to group `A`. The collision
elements of `body3` and `body4` belong to group `B`. `A` is set to ignore `B`
and vice versa.  This would be represented by the following bitmasks:

Group  |  Bitmask Type | Bitmask Value
-------|---------------|--------------
 `A`   |  membership   |  `0...0011`
 `A`   |  ignore       |  `0...0100`
 `B`   |  membership   |  `0...0101`
 `B`   |  ignore       |  `0...0010`

In this representation, the right-most bit is the low-order bit.  Remember that
all elements belong to the universal group (bit 0). It should be clear that
`groupA` and `groupB` were assigned to bits 1 and 2, respectively.

<b>Comparison</b> Collision filtering based on _classes_ of bodies fits
naturally with the collision filter group model.  Its representation is compact
and its runtime cost remains unchanged.  The efficacy of the clique approach
depends on the size of the groups and, handled greedily, could lead to an
explosion in redundant clique assignments.

<b>Verdict</b> Prefer collision filter groups.

These relative strengths and weaknesses are the reasons we maintain both systems
simultaneously.  Collision cliques provide an (in practice) unbounded set of
filter parameters and are well suited for mutually exclusive sets of bodies.
Collision filter groups offer a compact representation with O(1) computation
costs and are ideally suited for excluding large classes of bodies from
collision computation.

Next topic: @ref collision_filter_file_semantics
**/

/** @defgroup collision_filter_file_semantics Input File Collision Semantics
 @ingroup collision_filter_concepts

@section collision_filter_sdf_semantics SDF File Collision Semantics

 <!-- TODO(SeanCurtis-TRI): Provide the missing details. See issue #5211. -->

 @note This section is incomplete.

@section collision_filter_urdf_semantics URDF File Collision Semantics

 <!-- TODO(SeanCurtis-TRI): Provide the missing details. See issue #5211. -->

 @note This section is incomplete.

 Next topic: @ref collision_filter_future
**/

/** @defgroup collision_filter_future Future Collision Filter Features
 @ingroup collision_filter_concepts

   - High-level filtering semantics: specify colliding/non-colliding state using
   high level abstractions: tree, model, body, body collection, collision
   element.
   - The ability to abstractly specify filtering semantics independent of
   representation based on a series of refining _rules_.  e.g.,
      - `SetNoSelfCollision(tree);`
         `//` nothing in the tree collides with itself.
      - `SetNoCollisions(tree1, tree2);`
         `//` nothing in tree 1 collides with anything in tree 2.
      - `SetCanCollide(tree1->bodies[i], tree1->bodies[j]);`
         `//` Exception to previous rule, such that body `i` in tree 1 can
         collide with body `j` in tree 2.
 **/
