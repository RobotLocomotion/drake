#pragma once

#include <map>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/sorted_pair.h"
#include "drake/common/ssize.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {
namespace internal {

class SpanningForestModel;

/* Terminology note: we use "Link" here to mean what MultibodyPlant calls a
"Body". For purposes of graph analysis we want to clearly distinguish the
input graph from the spanning forest model we're going to build. To do that we
will use Link/Joint for the nodes and parent/child directed edges of the user's
input structure, and Body/Mobilizer (a.k.a. MobilizedBody) for the nodes and
inboard/outboard directed edges of the generated forest. */
// TODO(sherm1) Consider switching to "Link" in MultibodyPlant for clarity
//  and consistency.

using JointTypeIndex = TypeSafeIndex<class JointTypeTag>;
using ConstraintTypeIndex = TypeSafeIndex<class ConstraintTypeTag>;
using LinkIndex = BodyIndex;  // These are interchangeable.
using ConstraintIndex = TypeSafeIndex<class ConstraintTag>;
using CompositeLinkIndex = TypeSafeIndex<class CompositeLinkTag>;
using MobodIndex = TypeSafeIndex<class MobodTag>;

/** Link properties that can affect how the Forest model gets built. Or-ing
these also produces a LinkFlags object. */
enum class LinkFlags : unsigned {
  Default            = 0,
  Static             = 1 << 0,  ///< Implicitly welded to World
  MustBeBaseBody     = 1 << 1,  ///< Ensure connection to World if none
  TreatAsMassless    = 1 << 2,  ///< Can't be a terminal body in a tree
  Shadow             = 1 << 3   ///< Link is a shadow (internal use only)
};

/** Joint properties that can affect how the Forest model gets built. Or-ing
these also produces a JointFlags object. */
enum class JointFlags : unsigned {
  Default            = 0,
  MustBeModeled      = 1 << 0  ///< Model explicitly even if ignorable weld.
};

/** Options for how to build the spanning forest model. Or-ing these also
produces a ModelingOptions object. These can be provided as per-model instance
options to override global options. */
enum class ModelingOptions : unsigned {
  Default               = 0,
  Static                = 1 << 0,  ///< All Links are static in this instance
  UseFixedBase          = 1 << 1,  ///< Use welds rather than floating joints
  UseRpyFloatingJoints  = 1 << 2,  ///< For floating, use RPY not quaternion
  CombineCompositeLinks = 1 << 3   ///< Make a single Mobod for welded Links
};

/** Defines a graph consisting of Links (rigid bodies) interconnected by Joints.
This is a directed, possibly cyclic, graph defined by a sequence of calls to
AddLink() and AddJoint(). At any point, a user can ask graph specific questions
such as how Links are connected, by which Joints or even perform more complex
queries such as what sets of Links are connected by Weld Joints.

LinkJointGraph is intended to directly represent the user-specified
connectivity, for example as supplied in an sdf file. For efficient computation
in Drake (using joint-space coordinates) we need to create a model of this
graph as a forest of spanning trees plus loop-closing constraints. We call that
a SpanningForestModel, one of which is owned by the graph.

When we build a SpanningForestModel for a LinkJointGraph, the process
may add _modeling_ Links, Joints, and Constraints to the graph. Those are
ephemeral and kept distinct from _user_ Links and Joints so we can easily
restore the graph to its original condition.

In general during modeling:
  - A user-specified Link may get split into a "primary" Link and one or more
    "shadow" Links in order to break loops. Each of those has its own Mobod, so
    a user Link can generate multiple Mobods. (Geometry should remain attached
    to the primary Link.)
  - A primary Link and its shadows must be welded together by Weld Constraints
    which will be added to this Graph during modeling.
  - Modeling may require additional Joints to mobilize free bodies or
    immobilize static bodies. Each Joint maps to at most one Mobod; some Joints
    may instead be represented as Constraints or (for welds) implicitly as
    part of a composite body.
  - Composite bodies (Links welded together) can be represented by a single
    Mobod, so many Links may follow one Mobod.
  - We never delete any of the user's Links or Joints; we may add new ones
    during modeling but those are distinct from the user's.
*/
// TODO(amcastro-tri): consider moving outside internal:: and available to users
//  of MBP, towards #11307.
class LinkJointGraph {
 public:
  // The classes are defined later on in this header file.
  class Link;
  class Joint;
  class Constraint;
  struct JointType;
  struct ConstraintType;

  /** Default construction defines well-known joint types. */
  LinkJointGraph();

  /** Copies both the graph and the model if there is one. */
  LinkJointGraph(const LinkJointGraph& source);

  /** Assigns both the graph and the model if there is one. */
  LinkJointGraph& operator=(const LinkJointGraph& source);

  /** Move construction leaves the source as though it had just been
  default constructed. */
  LinkJointGraph(LinkJointGraph&& source);

  /** Move assignment leaves the source as though it had just been
  default constructed. */
  LinkJointGraph& operator=(LinkJointGraph&& source);

  /** Models this LinkJointGraph as a SpanningForestModel and returns a
  reference to the model. The old model is cleared and rebuilt. The returned
  reference is always to the same SpanningForestModel object; we do not
  create a new one when remodeling.
  @see ClearModel() */
  const SpanningForestModel& BuildModel(
      ModelingOptions global_options = ModelingOptions::Default,
      std::map<ModelInstanceIndex, ModelingOptions> instance_options = {});

  /** Unmodels this graph and removes ephemeral modeling elements that were
  added to the graph during modeling.
  @param[in] keep_modeling_additions
    Optionally leave modeling Links, Joints, and Constraints in place as
    though the user had added them. A subsequent BuildModel() using the same
    options should leave the graph unchanged. */
  void ClearModel(bool keep_modeling_additions = false);

  /** Has BuildModel() been called since the most recent change to this
  LinkJointGraph? */
  bool is_model_valid() const { return data_.model_is_valid; }

  /** Returns a reference to the internal SpanningForestModel whether it is
  valid or not.
  @see is_model_valid() */
  const SpanningForestModel& model() const {
    return *data_.model;
  }

  /** Adds a new Link to the graph.
  @param[in] name
    A unique name for the new Link (body) in the given `model_instance`.
    Several Links can have the same name within a %LinkJointGraph as long as
    they are unique within their model instances.
  @param[in] model_instance
    The model instance to which this Link belongs, see @ref model_instance.
  @param[in] flags
    Any special handling required for this Link.
  @note The first call to AddLink() defines the "world" Link's `name`. For this
  first call `model_instance` must be world_model_instance().
  @returns The unique LinkIndex for the added joint in the graph.
  @throws std::exception if `name` is duplicated within `model_instance`.
  @throws std::exception if the first call isn't for the World Link. */
  LinkIndex AddLink(const std::string& name, ModelInstanceIndex model_instance,
                    LinkFlags flags = LinkFlags::Default);

  /** Adds a new Joint to the graph.
  @param[in] name
    A unique name for the new Joint in the given `model_instance`. Several
    Joints can have the same name within a %LinkJointGraph as long as they are
    unique within their model instances.
  @param[in] model_instance
    The model instance to which this joint belongs, see @ref model_instance.
  @param[in] type
    A string designating the type of this joint, such as "revolute" or
    "ball". This must be chosen from the set of joint types previously
    registered with calls to RegisterJointType().
  @param[in] parent_link_index
    This must be the index of a Link previously obtained with a call to
    AddLink(), or it must be LinkIndex(0) for the World Link.
  @param[in] child_link_index
    This must be the index of a Link previously obtained with a call to
    AddLink(), or it must be LinkIndex(0) for the World Link.
  @returns The unique JointIndex for the added joint in the graph.
  @throws std::exception if `name` is duplicated within `model_instance`.
  @throws std::exception if `type` has not been registered with
      RegisterJointType().
  @throws std::exception if `parent_link_index` or `child_link_index` are
      not valid Link indexes for `this` graph, if they are the same index,
      or if there is already a Joint between them.
  @throws std::exception if a static Link is connected to World by anything
      other than a Weld Joint. */
  JointIndex AddJoint(const std::string& name,
                      ModelInstanceIndex model_instance,
                      const std::string& type, LinkIndex parent_link_index,
                      LinkIndex child_link_index,
                      JointFlags flags = JointFlags::Default);

  /** Returns the Link that corresponds to World. This Link is added via the
  first call to AddLink().
  @throws std::exception iff AddLink() was not called even once yet. */
  const Link& world_link() const;

  /** Returns the unique index that identifies the "weld" joint type.
  @note The SDF format calls this type a "fixed" joint. */
  JointTypeIndex weld_type_index() const { return data_.weld_type_index; }

  /** Returns the unique index that identifies the "quaternion floating" joint
  type. This is a joint that permits unlimited 6 dof translation and rotation
  without any rotational singularities. That requires an extra coordinate and
  a corresponding constraint. */
  JointTypeIndex quaternion_floating_type_index() const {
    return data_.quaternion_floating_type_index;
  }

  /** Returns the unique index that identifies the "roll-pitch-yaw floating"
  joint type. This is a joint that permits 6 dof translation and rotation
  using just 6 coordinates and no constraint but which necessarily has a
  singularity at some orientations. */
  JointTypeIndex rpy_floating_type_index() const {
    return data_.rpy_floating_type_index;
  }

  /** Returns the unique name reserved to identify weld joints (always "weld").
   */
  static std::string weld_type_name() { return "weld"; }

  /** Registers a joint type by name.
  @param[in] joint_type_name
    A unique string identifying a joint type, such as "revolute" or
    "prismatic".
  @param[in] nq
    Number of generalized position coordinates q needed for implementation
    of this type of Joint.
  @param[in] nv
    Number of generalized velocity coordinates v needed for implementation
    of this type of Joint.
  @param[in] has_quaternion
    Whether the first four q values represent a quaternion in w[xyz] order.
  @retval joint_type_index
    Unique index assigned to the new joint type.
  @throws std::exception if `joint_type_name` already identifies a
    previously registered joint type.
  @pre 0≤nq≤7, 0≤nv≤6, nv≤nq, !has_quaternion or nq≥4.

  @note LinkJointGraph is preloaded with Drake-specific Joint types it needs to
  know about including "weld", "quaternion_floating", and "rpy_floating". */
  JointTypeIndex RegisterJointType(const std::string& joint_type_name,
                                   int nq, int nv, bool has_quaternion = false);

  /** @returns `true` iff the given `joint_type_name` was previously registered
  via a call to RegisterJointType(), or iff it equals weld_type_name(). */
  bool IsJointTypeRegistered(const std::string& joint_type_name) const;

  /** Returns the number of registered joint types. */
  int num_joint_types() const;

  /** Gets a JointType by index. */
  const JointType& get_joint_type(JointTypeIndex index) const {
    return data_.joint_types[index];
  }

  const std::vector<Link>& links() const { return data_.links; }
  const Link& links(LinkIndex link_index) const {
    return links().at(link_index);
  }
  const std::vector<Joint>& joints() const { return data_.joints; }
  const Joint& joints(JointIndex joint_index) const {
    return joints().at(joint_index);
  }
  const std::vector<Constraint>& constraints() const {
    return data_.constraints;
  }
  const Constraint& constraints(ConstraintIndex constraint_index) const {
    return constraints().at(constraint_index);
  }

  /** Links with this index or higher were added during modeling. */
  int num_user_links() const { return data_.num_user_links; }

  /** Joints with this index or higher were added during modeling. */
  int num_user_joints() const { return data_.num_user_joints; }

  /** Constraints with this index or higher were added during modeling. */
  int num_user_constraints() const { return data_.num_user_constraints; }

  /** After modeling, returns the mobilized body (Mobod) followed by this Link.
  If the Link is part of a composite, this will be the mobilized body for the
  whole composite. If the Link was split into a primary and shadows, this is
  the mobilized body followed by the primary. */
  MobodIndex link_to_mobod(LinkIndex index) const;  // See below

  /** Links that were explicitly designated "Static" in AddLink. Links that
  are part of a Static ModelInstance are not included here unless they were
  also explicitly designated. */
  const std::vector<LinkIndex>& static_links() const {
    return data_.static_links;
  }

  /** Non-static Links that were designated "MustBeBaseBody" in AddLink. */
  const std::vector<LinkIndex>& must_be_base_body_links() const {
    return data_.must_be_base_body_links;
  }

  /** After modeling, returns groups of Links that are welded together.
  Each group may be modeled with a single mobilized body or multiple mobilized
  bodies depending on modeling options. The first entry in each group is
  the "representative link". Composites are present here only for groups of
  two or more welded Links. */
  const std::vector<std::vector<LinkIndex>>& composite_links() const {
    return data_.composite_links;
  }

  /** @returns `true` if a Link named `name` was added to `model_instance`.
  @see AddLink().
  @throws std::exception if `model_instance` is not a valid index. */
  bool HasLinkNamed(const std::string& name,
                    ModelInstanceIndex model_instance) const;

  /** @returns `true` if a joint named `name` was added to `model_instance`.
  @see AddLink().
  @throws std::exception if `model_instance` is not a valid index. */
  bool HasJointNamed(const std::string& name,
                     ModelInstanceIndex model_instance) const;

  /** If there is a Joint connecting the given Links, return its index.
  Otherwise the returned index will be invalid. You can call this any time
  and it will work with whatever Joints have been defined. But note that
  additional Joints may be added by modeling, so you may get a different answer
  before and after modeling. Cost is O(j) where j=min(j₁,j₂) with jᵢ the
  number of Joints attached to linkᵢ. */
  JointIndex MaybeGetJointBetween(LinkIndex link1_index,
                                  LinkIndex link2_index) const;

  /** After modeling, returns the sequence of Links from the given one to World,
  inclusive of both. Cost is O(ℓ) where ℓ is the give Link's level in
  the SpanningForestModel.
  @throws std::exception if called prior to modeling */
  std::vector<LinkIndex> FindPathToWorld(LinkIndex link_index) const;

  /** After modeling, finds the first Link common to the paths to World from
  each of two Links to World in the SpanningForestModel. Returns World
  immediately if the Links are on different trees of the forest. Otherwise the
  cost is O(ℓ) where ℓ is the length of the longer path from one of the Links
  to the ancestor. */
  LinkIndex FindFirstCommonAncestor(LinkIndex link1_index,
                                    LinkIndex link2_index) const;

  /** This method partitions the LinkJointGraph into subgraphs such that (a)
  every Link is in one and only one subgraph, and (b) two Links are in the
  same subgraph iff there is a path between them which includes only weld
  joints (see weld_type_name()). Each subgraph of welded Links is represented
  as a set of Link indexes. By definition, these subgraphs will be disconnected
  by any non-weld joint between two Links. The first subgraph will have all of
  the Links welded to the world, including the World Link itself; all
  subsequent subgraphs will be in no particular order. A few more notes:
    - Each Link in the LinkJointGraph is included in one set and one set only.
    - The maximum number of returned subgraphs equals the number of Links in
      the graph (num_links()). This corresponds to a graph with no weld joints.
    - The world Link is also included in a set of welded Links, and this set is
      element zero in the returned vector.
    - The minimum number of subgraphs is one. This corresponds to a graph with
      all Links welded to the world. */
  std::vector<std::set<LinkIndex>> FindSubgraphsOfWeldedLinks() const;

  /** Returns all Links that are transitively welded, or rigidly affixed, to
  `link_index_`, per these two definitions:
    1. A Link is always considered welded to itself.
    2. Two unique Links are considered welded together exclusively by the
       presence of a weld joint, not by other constructs such as constraints.
  Therefore, if `link_index` is a valid index to a Link in this graph, then the
  return vector will always contain at least one entry storing `link_index`.
  @throws std::exception iff `link_index` does not correspond to a Link in this
  graph. */
  std::set<LinkIndex> FindLinksWeldedTo(LinkIndex link_index) const;

  /** Returns true if the given Link should be treated as massless. That
  requires that the Link was marked TreatAsMassless and is not connected by
  a Weld Joint to a massful Link or Composite. */
  bool must_treat_as_massless(LinkIndex link_index) const;

  /** (Internal use only) For testing -- clears the model. */
  void change_link_flags(LinkIndex link_index, LinkFlags flags);

  /** (Internal use only) For testing -- clears the model. */
  void change_joint_flags(JointIndex joint_index, JointFlags flags);

  /** (Internal use only) For testing. */
  void DumpGraph(std::string title) const;

 private:
  friend class SpanningForestModel;

  Link& mutable_link(LinkIndex link_index) {
    return data_.links[link_index];
  }

  Joint& mutable_joint(JointIndex joint_index) {
    return data_.joints[joint_index];
  }

  // For use by SpanningForestModel.
  void set_primary_mobod_for_link(LinkIndex link_index,
                                  MobodIndex primary_mobod_index,
                                  JointIndex primary_joint_index);
  void set_mobod_for_joint(JointIndex joint_index, MobodIndex mobod_index);
  void ignore_loop_joint(JointIndex joint_index);
  void RenumberMobodIndexes(const std::vector<MobodIndex>& old_to_new);

  // During modeling, we're trying to add the given Joint outboard of the
  // given Mobod. At least one of the Joint's two Links must already be
  // modeled with that Mobod. That one will be the inboard Link. If that's
  // the parent Link then parent->child and inboard->outboard will match,
  // otherwise the mobilizer must be reversed. The bool return is true if
  // we're reversing.
  // tuple is: inboard, outboard, is_reversed
  std::tuple<LinkIndex, LinkIndex, bool> FindInboardOutboardLinks(
      MobodIndex mobod_index, JointIndex joint_index) const;

  // During modeling, add a new Shadow Link to the given Primary Link, with
  // the Shadow mobilized by the given Joint.
  // We'll derive a name for the shadow from the primary, create the Link
  // with appropriate bookkeeping, and add a Weld Constraint between
  // the primary and shadow (primary will be Weld's "parent").
  LinkIndex AddShadowLink(LinkIndex primary_link_index,
                          JointIndex shadow_joint_index);

  ConstraintIndex AddLoopClosingWeldConstraint(LinkIndex primary_link_index,
                                               LinkIndex shadow_link_index);

  // Adds the implicit Joint for a floating or fixed base Link, with World
  // as the parent and the base Link as the child.
  JointIndex AddModelingJointToWorld(JointTypeIndex type_index,
                                     LinkIndex child_link_index);

  // Adds the new link to the composite of which the existing_link is a
  // member. If the existing_link is not a member of any composite, then we
  // create a new composite with the composite_link as the representative.
  CompositeLinkIndex AddToCompositeLink(LinkIndex existing_link_index,
                                        LinkIndex new_link_index);

  // Notes that we didn't model this joint because it is just a weld to an
  // existing Composite.
  void AddUnmodeledJointToComposite(JointIndex unmodeled_joint_index,
                                    CompositeLinkIndex which);

  // Finds the assigned index for a joint type from the type name. Returns an
  // invalid index if `joint_type_name` was not previously registered with a
  // call to RegisterJointType().
  JointTypeIndex GetJointTypeIndex(const std::string& joint_type_name) const;

  // Recursive helper method for FindSubgraphsOfWeldedLinks().
  // The first thing this helper does is to mark `parent_link` as "visited" in
  // the output array `visited`.
  // Next, it scans the sibling Links connected to `parent_index` by a joint.
  // If the sibling was already visited, it moves on to the next sibling, if
  // any. For a sibling visited for the first time there are two options:
  //   1) if it is connected by a weld joint, it gets added to parent_subgraph.
  //      Recursion continues starting with parent_index = sibling and the
  //      parent sub-graph. Otherwise,
  //   2) a new sub-graph is created for the sibling and gets added to the
  //      list of all `subgraphs`. Recursion continues starting with
  //      parent_index = sibling and the new sub-graph for this sibling.
  void FindSubgraphsOfWeldedLinksRecurse(
      const Link& parent_link, std::set<LinkIndex>* parent_subgraph,
      std::vector<std::set<LinkIndex>>* subgraphs,
      std::vector<bool>* visited) const;

  void ThrowIfModelNotBuiltYet(const char* func) const;

  // Ensure that all data members are set or restored to their
  // default-constructed condition.
  void DefaultConstruct();

  // Group data members so we can have the compiler generate most of the
  // copy/move/assign methods for us while still permitting pointer fixup.
  struct Data {
    std::vector<JointType> joint_types;
    std::vector<ConstraintType> constraint_types;

    // The first entry in links is the World Link with LinkIndex(0) and name
    // world_link().name().
    std::vector<Link> links;
    std::vector<Joint> joints;
    std::vector<Constraint> constraints;

    // The first num_user_links etc. elements are user-supplied; the rest were
    // added during modeling.
    int num_user_links{0};
    int num_user_joints{0};
    int num_user_constraints{0};

    std::vector<LinkIndex> static_links;
    std::vector<LinkIndex> must_be_base_body_links;  // but not static

    // Every user Link, organized by Model Instance.
    std::map<ModelInstanceIndex, std::vector<LinkIndex>>
        model_instance_to_links;

    // This must always have the same number of entries as joint_types_.
    std::unordered_map<std::string, JointTypeIndex> joint_type_name_to_index;

    // The xxx_name_to_index_ structures are multimaps because
    // links/joints/actuators/etc may appear with the same name in different
    // model instances. The index values are still unique across the graph.
    // These include only user-supplied links and joints.
    std::unordered_multimap<std::string, LinkIndex> link_name_to_index;
    std::unordered_multimap<std::string, JointIndex> joint_name_to_index;

    // These pre-registered joint types are set in the constructor.
    JointTypeIndex weld_type_index;
    JointTypeIndex rpy_floating_type_index;
    JointTypeIndex quaternion_floating_type_index;

    // Every Composite has at least two Links and the first one listed is the
    // representative. If there is anything welded to World, that is the 0th
    // Composite and World (listed first) is its representative Link.
    std::vector<std::vector<LinkIndex>> composite_links;

    bool model_is_valid{false};  // set false whenever changes are made
    std::unordered_multimap<std::string, LinkIndex> model_link_name_to_index;
    std::unordered_multimap<std::string, JointIndex> model_joint_name_to_index;

    // Contains a back pointer to the graph so needs fixup post copy or move.
    copyable_unique_ptr<SpanningForestModel> model;
  } data_;
};

/* Overloads to make LinkFlags behave in a civilized manner. */
inline LinkFlags operator|(LinkFlags left, LinkFlags right) {
  return static_cast<LinkFlags>(static_cast<unsigned>(left) |
      static_cast<unsigned>(right));
}
inline LinkFlags operator&(LinkFlags left, LinkFlags right) {
  return static_cast<LinkFlags>(static_cast<unsigned>(left) &
      static_cast<unsigned>(right));
}
inline LinkFlags operator~(LinkFlags flags) {
  return static_cast<LinkFlags>(~static_cast<unsigned>(flags));
}

/** Represents a link in the LinkJointGraph. This includes links provided
via user input and also those added during modeling as Shadow links created
when we cut a user link in order to break a kinematic loop. Links may be
modeled individually or can be combined into Composite Links comprising groups
of Links that were connected by weld joints. */
class LinkJointGraph::Link {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Link)

  Link(LinkIndex index, const std::string& name,
       ModelInstanceIndex model_instance, LinkFlags flags)
      : index_(index), name_(name), model_instance_(model_instance),
        flags_(flags) {}

  /** @returns this %Link's unique index in the graph. */
  LinkIndex index() const { return index_; }

  /** @returns this %Link's model instance. */
  ModelInstanceIndex model_instance() const { return model_instance_; }

  /** @returns this %Link's name, unique within model_instance(). */
  const std::string& name() const { return name_; }

  /** @returns indexes of all the Joints that connect to this %Link. This is
  the union of joints_as_parent() and joints_as_child(). */
  const std::vector<JointIndex>& joints() const { return joints_; }

  /** @returns indexes of all the Joints that connect to this %Link in which
  this is the parent %Link. */
  const std::vector<JointIndex>& joints_as_parent() const {
    return joints_as_parent_; }

  /** @returns indexes of all the joints that connect to this %Link in which
  this is the child %Link. */
  const std::vector<JointIndex>& joints_as_child() const {
    return joints_as_child_; }

  /** @returns indexes of all the Constraints that connect to this %Link. */
  const std::vector<ConstraintIndex>& constraints() const {
    return constraints_;
  }

  /** Returns `true` only if this is the World %Link. Static Links and Links
  in the World Composite are not included; see is_anchored() if you want to
  include everything that is fixed with respect to World. */
  bool is_world() const { return index_ == LinkIndex(0); }

  /** After modeling, returns `true` if this Link is fixed with respect to
  World. That includes World itself, static Links, and any Link that is part
  of the World Composite (that is, it is directly or indirectly welded to
  World). */
  bool is_anchored() const {
    return is_world() || is_static() ||
           (composite().is_valid() && composite() == CompositeLinkIndex(0));
  }

  bool is_static() const {
    return static_cast<bool>(flags_ & LinkFlags::Static);
  }

  bool must_be_base_body() const {
    return static_cast<bool>(flags_ & LinkFlags::MustBeBaseBody);
  }

  bool treat_as_massless() const {
    return static_cast<bool>(flags_ & LinkFlags::TreatAsMassless);
  }

  bool is_shadow() const {
    return static_cast<bool>(flags_ & LinkFlags::Shadow);
  }

  /** If this Link is a Shadow, returns the primary Link it shadows. If
  not a Shadow then it is its own primary Link so returns index(). */
  const LinkIndex primary_link() const { return primary_link_; }

  int num_shadows() const { return ssize(shadow_links_); }

  // (For testing) If `to_set` is LinkFlags::Default sets the flags to
  // Default. Otherwise or's in the given flags to the current set. Returns
  // the updated value of this Link's flags.
  LinkFlags set_flags(LinkFlags to_set) {
    return flags_ = (to_set == LinkFlags::Default ? LinkFlags::Default
                                                  : flags_ | to_set);
  }

  // (For testing) Resets the given flags leaving others unchanged. Returns
  // the updated value of this Link's flags.
  LinkFlags clear_flags(LinkFlags to_clear) {
    return flags_ = flags_ & ~to_clear;
  }

  /** Returns the index of the mobilized body that mobilizes this Link. If
  this Link is part of a Composite, this is the Mobod that mobilizes the
  Composite as a whole via the Composite's representative Link. If you ask
  this Mobod what Joint it represents, it will report the Joint that was used
  to mobilize the Composite; that won't necessarily be a Joint connected to
  this Link. See inboard_joint_index() to find the Joint that connected this
  Link to its Composite. */
  MobodIndex mobod_index() const { return mobod_; }

  /** Returns the Joint that was used to associate this Link with its
  mobilized body. For a Composite, returns the Joint that connects this
  Link to the Composite, not necessarily the Joint that is modeled by
  the Mobod returned by mobod_index(). */
  JointIndex inboard_joint_index() const { return joint_; }

  /** Returns the index of the Composite this Link is part of, if any.
  Otherwise returns an invalid index. */
  CompositeLinkIndex composite() const { return composite_link_index_; }

 private:
  friend class LinkJointGraph;

  void renumber_mobod_indexes(const std::vector<MobodIndex>& old_to_new) {
    if (mobod_.is_valid())
      mobod_ = old_to_new[mobod_];
  }

  // Notes that this Link is connected by `joint`.
  void add_joint_as_parent(JointIndex joint) {
    joints_as_parent_.push_back(joint);
    joints_.push_back(joint);
  }
  void add_joint_as_child(JointIndex joint) {
    joints_as_child_.push_back(joint);
    joints_.push_back(joint);
  }

  void add_constraint(ConstraintIndex constraint) {
    constraints_.push_back(constraint);
  }

  void clear_model(int num_user_joints) {
    mobod_ = {};
    joint_ = {};
    primary_link_ = {};
    shadow_links_.clear();
    composite_link_index_ = {};

    auto remove_model_joints =
        [num_user_joints](std::vector<JointIndex>& joints) {
          while (!joints.empty() && joints.back() >= num_user_joints)
            joints.pop_back();
        };

    remove_model_joints(joints_as_parent_);
    remove_model_joints(joints_as_child_);
    remove_model_joints(joints_);
  }

  LinkIndex index_;
  std::string name_;
  ModelInstanceIndex model_instance_;
  LinkFlags flags_{LinkFlags::Default};

  // Members below here may contain as-modeled information that has to be
  // removed when the spanning forest model is cleared or rebuilt. The joint
  // lists always have the as-modeled extra joints at the end.

  std::vector<JointIndex> joints_as_parent_;
  std::vector<JointIndex> joints_as_child_;
  // All joints connecting this Link in order of arrival. This is the union of
  // joints_as_parent_ and joints_as_child_,
  std::vector<JointIndex> joints_;

  std::vector<ConstraintIndex> constraints_;

  MobodIndex mobod_;  // Which Mobod mobilizes this Link?
  JointIndex joint_;  // Which Joint connected us to the Mobod?

  LinkIndex primary_link_;  // Same as index_ if this is a primary link.
  std::vector<LinkIndex> shadow_links_;

  CompositeLinkIndex composite_link_index_;  // Invalid if not in composite.
};

inline MobodIndex LinkJointGraph::link_to_mobod(LinkIndex index) const {
  return links()[index].mobod_;
}

inline void LinkJointGraph::set_primary_mobod_for_link(
    LinkIndex link_index, MobodIndex primary_mobod_index,
    JointIndex primary_joint_index) {
  Link& link = data_.links[link_index];
  DRAKE_DEMAND(!link.mobod_.is_valid());
  link.mobod_ = primary_mobod_index;
  link.joint_ = primary_joint_index;
}

inline void LinkJointGraph::change_link_flags(LinkIndex link_index,
                                              LinkFlags flags) {
  ClearModel();
  mutable_link(link_index).set_flags(flags);
}

inline bool LinkJointGraph::must_treat_as_massless(LinkIndex link_index) const {
  const Link& link = links(link_index);
  // TODO(sherm1) If part of a Composite then this is only massless if the
  //  entire Composite is composed of massless Links.
  return link.treat_as_massless();
}

/* Overloads to make JointFlags behave in a civilized manner. */
inline JointFlags operator|(JointFlags left, JointFlags right) {
  return static_cast<JointFlags>(static_cast<unsigned>(left) |
      static_cast<unsigned>(right));
}
inline JointFlags operator&(JointFlags left, JointFlags right) {
  return static_cast<JointFlags>(static_cast<unsigned>(left) &
      static_cast<unsigned>(right));
}
inline JointFlags operator~(JointFlags flags) {
  return static_cast<JointFlags>(~static_cast<unsigned>(flags));
}

class LinkJointGraph::Joint {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Joint)

  Joint(JointIndex index, const std::string& name,
        ModelInstanceIndex model_instance, JointTypeIndex joint_type_index,
        LinkIndex parent_link_index, LinkIndex child_link_index,
        JointFlags flags)
      : index_(index),
        name_(name),
        model_instance_(model_instance),
        flags_(flags),
        type_index_(joint_type_index),
        parent_link_index_(parent_link_index),
        child_link_index_(child_link_index) {}

  ModelInstanceIndex model_instance() const { return model_instance_; }

  const std::string& name() const { return name_; }

  LinkIndex parent_link() const { return parent_link_index_; }
  LinkIndex child_link() const { return child_link_index_; }

  JointTypeIndex type_index() const { return type_index_; }

  JointIndex index() const { return index_; }

  bool connects(LinkIndex link) const {
    return link == parent_link() || link == child_link();
  }

  bool connects(LinkIndex link1, LinkIndex link2) const {
    return (parent_link() == link1 && child_link() == link2) ||
           (parent_link() == link2 && child_link() == link1);
  }

  // Any moving (articulated) Joint must be modeled (i.e. it must be represented
  // by some Mobod). Welds typically won't be modeled though unless specifically
  // requested, likely because a user thinks reaction forces here are important.
  bool must_be_modeled() const {
    return static_cast<bool>(flags_ & JointFlags::MustBeModeled);
  }

  MobodIndex mobod_index() const {
    if (!std::holds_alternative<MobodIndex>(how_modeled_))
      return MobodIndex();
    return std::get<MobodIndex>(how_modeled_);
  }

  bool has_been_processed() const {
    return !std::holds_alternative<std::monostate>(how_modeled_);
  }

  // Given one of Links connected by this Joint, return the other one.
  LinkIndex other_link_index(LinkIndex link_index) const {
    DRAKE_DEMAND((parent_link() == link_index) ^
        (child_link() == link_index));
    return parent_link() == link_index ? child_link() : parent_link();
  }

  // (For testing) If `to_set` is JointFlags::Default sets the flags to
  // Default. Otherwise or's in the given flags to the current set. Returns
  // the updated value of this Joint's flags.
  JointFlags set_flags(JointFlags to_set) {
    return flags_ = (to_set == JointFlags::Default ? JointFlags::Default
                                                   : flags_ | to_set);
  }

  // (For testing) Resets the given flags leaving others unchanged. Returns
  // the updated value of this Joint's flags.
  JointFlags clear_flags(JointFlags to_clear) {
    return flags_ = flags_ & ~to_clear;
  }

 private:
  friend class LinkJointGraph;

  void clear_model() {
    how_modeled_ = std::monostate{};
  }

  void renumber_mobod_indexes(const std::vector<MobodIndex>& old_to_new) {
    if (std::holds_alternative<MobodIndex>(how_modeled_))
      how_modeled_ = old_to_new[std::get<MobodIndex>(how_modeled_)];
  }

  struct IgnoredLoopJoint {};

  JointIndex index_;
  std::string name_;
  ModelInstanceIndex model_instance_;
  JointFlags flags_{JointFlags::Default};

  JointTypeIndex type_index_;
  LinkIndex parent_link_index_;
  LinkIndex child_link_index_;

  // Below here is the as-modeled information; must be flushed when the
  // model is cleared or rebuilt.

  // Meaning of the variants:
  // - monostate: not yet processed
  // - MobodIndex: modeled directly by a mobilizer
  // - ConstraintIndex: modeled as a constraint
  // - CompositeLinkIndex: not modeled because this is a weld interior to
  //     the indicated composite and we are combining so that one Mobod serves
  //     the whole composite.
  // - IgnoredLoopJoint: not modeled because we intentionally ignored the Joint
  //     (used with the IgnoreLoopJoints modeling option)
  std::variant<std::monostate, MobodIndex, ConstraintIndex, CompositeLinkIndex,
               IgnoredLoopJoint>
      how_modeled_;
};

inline void LinkJointGraph::set_mobod_for_joint(JointIndex joint_index,
                                                MobodIndex mobod_index) {
  Joint& joint = mutable_joint(joint_index);
  DRAKE_DEMAND(joint.how_modeled_.index() == 0);  // I.e., empty.
  joint.how_modeled_ = mobod_index;
}

inline void LinkJointGraph::ignore_loop_joint(JointIndex joint_index) {
  Joint& joint = mutable_joint(joint_index);
  DRAKE_DEMAND(joint.how_modeled_.index() == 0);  // I.e., empty.
  joint.how_modeled_ = Joint::IgnoredLoopJoint();
}

inline void LinkJointGraph::change_joint_flags(JointIndex joint_index,
                                               JointFlags flags) {
  ClearModel();
  mutable_joint(joint_index).set_flags(flags);
}

/** A constraint that restricts the relative motion of two Links. The
parent/child distinction sets the sign convention for the constraint
multipliers. Added welds between a primary Link and one of its shadow Links
always make the primary Link the parent. A constraint used to model a Joint
preserves parent/child order. */
class LinkJointGraph::Constraint {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Constraint)

  Constraint(ConstraintIndex index,
             std::string name,
             ModelInstanceIndex model_instance,
             LinkIndex parent_link_index,
             LinkIndex child_link_index) :
             index_(index), name_(name), model_instance_(model_instance),
             parent_link_index_(parent_link_index),
             child_link_index_(child_link_index) {}

  ModelInstanceIndex model_instance() const { return model_instance_; }

  const std::string& name() const { return name_; }

  LinkIndex parent_link() const { return parent_link_index_; }
  LinkIndex child_link() const { return child_link_index_; }

  ConstraintIndex index() const { return index_; }

 private:
  ConstraintIndex index_;
  std::string name_;
  ModelInstanceIndex model_instance_;
  JointTypeIndex type_index_;
  LinkIndex parent_link_index_;
  LinkIndex child_link_index_;
};

struct LinkJointGraph::JointType {
  std::string type_name;
  int nq{-1};
  int nv{-1};
  bool has_quaternion;  // If so, the first 4 qs as wxyz.
};

struct LinkJointGraph::ConstraintType {
  std::string type_name;
};

/* Overloads to make ModelingOptions behave in a civilized manner. */
inline ModelingOptions operator|(ModelingOptions left, ModelingOptions right) {
  return static_cast<ModelingOptions>(static_cast<unsigned>(left) |
                                      static_cast<unsigned>(right));
}
inline ModelingOptions operator&(ModelingOptions left, ModelingOptions right) {
  return static_cast<ModelingOptions>(static_cast<unsigned>(left) &
                                      static_cast<unsigned>(right));
}
inline ModelingOptions operator~(ModelingOptions flags) {
  return static_cast<ModelingOptions>(~static_cast<unsigned>(flags));
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
