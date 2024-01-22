#pragma once

#ifndef DRAKE_MULTIBODY_TOPOLOGY_FOREST_INCLUDED
#error Do not include this file. Use "drake/multibody/topology/forest.h".
#endif

#include <functional>
#include <map>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/ssize.h"
#include "drake/multibody/topology/graph.h"

namespace drake {
namespace multibody {
// TODO(sherm1) Promote from internal once API has stabilized: issue #11307.
namespace internal {

using WeldedMobodsIndex = TypeSafeIndex<class CompositeMobodTag>;
using LoopConstraintIndex = TypeSafeIndex<class LoopConstraintTag>;

/** SpanningForest models a LinkJointGraph via a set of spanning trees and
loop-closing constraints. */
// TODO(sherm1) Just a skeleton here; much more to come.
class SpanningForest {
 public:
  // Constructors and assignment are private; only LinkJointGraph may access.
  // These can't be default because there are back pointers to clean up.

  /** Clears the existing model and builds a new one. Any as-built information
  in the owning LinkJointGraph is removed and replaced with new as-built
  information. */
  void BuildForest(
      ModelingOptions global_options = ModelingOptions::Default,
      std::map<ModelInstanceIndex, ModelingOptions> instance_options = {});

  /** Returns a reference to the graph that owns this forest (as set during
  construction). */
  const LinkJointGraph& graph() const {
    DRAKE_ASSERT(data_.graph != nullptr);
    return *data_.graph;
  }

  /** Returns the global ModelingOptions that were used when this model was last
  built with BuildForest(). You can also provide a ModelInstanceIndex to see
  ModelingOptions specific to that Model Instance. */
  ModelingOptions options() const { return data_.global_modeling_options; }

  /** Returns the ModelingOptions that were last used for elements of the given
  ModelInstance. If we don't have specific options for this instance, we
  return the global ModelingOptions as returned by options(). */
  ModelingOptions options(ModelInstanceIndex index) const {
    const auto instance_options = data_.instance_modeling_options.find(index);
    return instance_options == data_.instance_modeling_options.end()
               ? data_.global_modeling_options
               : instance_options->second;
  }

 private:
  friend class LinkJointGraph;
  friend class copyable_unique_ptr<SpanningForest>;

  // Only the owning LinkJointGraph may call this constructor. The owner must
  // outlive this model, which may be built and rebuilt repeatedly for the same
  // owner. For copy and move, the owner is responsible for repairing the back
  // pointer via SetNewOwner().
  // @pre `graph` is non-null
  explicit SpanningForest(LinkJointGraph* graph) {
    DRAKE_DEMAND(graph != nullptr);
    data_.graph = graph;
  }

  // The caller (only LinkJointGraph) must provide a new back pointer after copy
  // or move so these can't be default.
  SpanningForest(const SpanningForest& source);
  SpanningForest(SpanningForest&& source);
  SpanningForest& operator=(const SpanningForest& source);
  SpanningForest& operator=(SpanningForest&& source);

  // LinkJointGraph uses this to fix the graph back pointer after copy or move.
  void SetNewOwner(LinkJointGraph* graph) {
    DRAKE_DEMAND(graph != nullptr);
    data_.graph = graph;
  }

  // After copy/move/assign: any element that has a pointer back to its owning
  // SpanningForest should replace the pointer with `this`.
  void FixInternalPointers();

  // Restores this SpanningTree to the state it has immediately after
  // construction. The owning LinkJointGraph remains unchanged.
  void Clear();

  struct Data {
    // These are all default but definitions deferred to .cc file so
    // that all the local classes are defined (Mac requirement).
    Data();
    Data(const Data&);
    Data(Data&&);
    ~Data();
    Data& operator=(const Data&);
    Data& operator=(Data&&);

    LinkJointGraph* graph{};  // The graph we're modeling.
    ModelingOptions global_modeling_options{ModelingOptions::Default};
    std::map<ModelInstanceIndex, ModelingOptions> instance_modeling_options;

    // TODO(sherm1) More to come
  } data_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
