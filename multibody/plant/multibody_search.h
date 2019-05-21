#pragma once

#include <functional>
#include <iostream>
#include <set>

#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/body.h"
#include "drake/multibody/tree/frame.h"
#include "drake/multibody/tree/joint.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {

namespace {
template <typename A>
std::set<A> AllIds(const int num_ids) {
  std::set<A> result_set;
  for (A i{0}; i < num_ids; i++) {
    result_set.insert(i);
  }
  return result_set;
}

template <typename A, typename B>
std::set<B> MapAndFlattenFromSet(
    const std::set<A>& in,
    std::function<std::vector<B>(const A&)> func) {
  std::set<B> result_set;
  for (const A& input_item : in) {
    for (const B& output_item : func(input_item)) {
      result_set.insert(output_item);
    }
  }
  return result_set;
}
}  // namespace

/// A database-like system for interrogating a MultibodyPlant about all of the
/// entities and relationships it contains.  This is meant for the benefit of
/// users whose code must apply to different MBPs and who need to manage the
/// resulting proliferation of IDs, state indexes, and tiny accessor objects
///
/// Model instances, joints, bodies, geometries, and state vector indices form
/// a roughly database table-like topology.  We attempt here to provide
/// typesafe query-language semantics for selections and closures over those
/// entities.
///
/// Example queries can be found in the unit test in
/// `multibody_plant_introspection_test.cc` and include expressions like:
///
/// *  search.WorldModel().Bodies().Named("WorldBody").ids()
/// *  search.AllMultibodyFrames().AttachedTo(
///        search.AllModels().Bodies().Named("pelvis")
///        ).count()
///
/// The `MultibodySearch` object itself is just a placeholder for selecting
/// the root set of a search.  Most of the actual logic lives in the typesafe
/// search state classes that collect the intermediates states and results.
template <typename T>
class MultibodySearch {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultibodySearch)

  using GeometrySet = std::set<geometry::GeometryId>;
  using GeometryFrameSet = std::set<geometry::FrameId>;
  using JointSet = std::set<JointIndex>;
  using BodySet = std::set<BodyIndex>;
  using MultibodyFrameSet = std::set<FrameIndex>;
  using ModelInstanceSet = std::set<ModelInstanceIndex>;

  /// Construct a MultibodyQuery query interface for the given @p plant.
  /// `plant` is aliased and must remain valid for the life of this object.
  MultibodySearch(const MultibodyPlant<T>* plant,
                  const geometry::SceneGraph<T>* scene_graph)
      : plant_(plant),
        scene_graph_(scene_graph) {
    if (!plant_->is_finalized()) {
      throw std::logic_error(
          "Searching on a MultibodyPlant before `Finalize` is "
          "not allowed; you must call Finalize() first.");
    }
  }

  /// Trivial mixin class with common search-state methods to avoid copypasta.
  //
  // `IdType` should name a type with integer-like semantics; `Derived`
  // should name the inheritor's most-derived type, which must have both a
  // copy ctor and a two-argument ctor analogous to the protected one here.
  template <typename IdType_, typename Derived>
  class IdResultsBase {
   public:
    using IdType = IdType_;
    using SetType = std::set<IdType>;

    const SetType ids() { return ids_; }
    int count() { return ids_.size(); }
    bool empty() { return ids_.empty(); }

    Derived Union(const Derived& other) {
      DRAKE_DEMAND(this->search_ == other.search_);
      typename Derived::SetType result_set = ids_;
      result_set.insert(other.ids_.begin(), other.ids_.end());
      return Derived(result_set, this->search_);
    }

    Derived FilterIds(std::function<bool(const IdType)> predicate) {
      SetType result;
      for (const IdType id : this->ids_) {
        if (predicate(id)) {
          result.insert(id);
        }
      }
      return Derived(result, this->search_);
    }

   protected:
    IdResultsBase(const SetType& ids, const MultibodySearch* search)
        : search_(search),
          ids_(ids) {}
    const MultibodySearch* search_;
    const SetType ids_;
  };


  /// Extended mixin class for search states on objects that have both an ID
  /// type and a value type, eg JointIndex vs. Joint.
  //
  // Similar to `IdResultsBase` but takes an additional template parameter
  // `ValueType` naming the concrete type indexed by `IdType` and an
  // additional ctor parameter for a function to convert ids to values.
  //
  // This also provides names() and Named() methods because all `ValueType`
  // thus far implemented happen to have a `name` method.  This is probably a
  // bit brittle.
  template <typename IdType, typename ValueType, typename Derived>
  class ValueResultsBase : public IdResultsBase<IdType, Derived> {
   public:
    using ValueFunction = std::function<const ValueType*(IdType)>;

    std::set<const ValueType*> values() {
      std::set<const ValueType*> result;
      for (const IdType id : this->ids_) { result.insert(value_fn_(id)); }
      return result;
    }

    Derived Filter(std::function<bool(const ValueType&)> predicate) {
      return this->FilterIds([&](const IdType& id){
          return predicate(*value_fn_(id)); });
    }

    Derived Named(const std::string& name) {
      return this->Filter([&](const ValueType& b) {
          return b.name() == name; });
    }

    std::set<std::string> Names() {
      std::set<std::string> result;
      for (const ValueType* val : values()) { result.insert(val->name()); }
      return result;
    }

   protected:
    ValueResultsBase(const std::set<IdType>& ids,
                     const MultibodySearch* search,
                     ValueFunction value_fn)
        : IdResultsBase<IdType, Derived>(ids, search),
          value_fn_(value_fn) {}
    const ValueFunction value_fn_;
  };

  class ModelInstanceResults;


  class JointResults final
      : public ValueResultsBase<JointIndex, Joint<T>, JointResults> {
   public:
    using Base = ValueResultsBase<JointIndex, Joint<T>, JointResults>;
    JointResults(const JointResults& state)
        : Base(state.ids_, state.search_, state.value_fn_) {}
    JointResults(const JointSet& ids, const MultibodySearch* search)
        : Base(ids, search,
               [this](JointIndex id) {
                 return &this->search_->plant_->get_joint(id);
               }) {}

    JointResults WithType(const std::string& type) {
      return this->Filter([&](const Joint<T>& b){
          return b.type_name() == type; });
    }
  };


  class BodyResults final
      : public ValueResultsBase<BodyIndex, Body<T>, BodyResults> {
   public:
    using Base = ValueResultsBase<BodyIndex, Body<T>, BodyResults>;
    BodyResults(const BodyResults& state)
        : Base(state.ids_, state.search_, state.value_fn_) {}
    BodyResults(const BodySet& ids, const MultibodySearch* search)
        : Base(ids, search,
               [this](BodyIndex id) {
                 return &this->search_->plant_->get_body(id);
               }) {}

    // TODO(ggould) The below structure recurs many times in this file and
    // should be able to be encapsulated in a template function (set<A>,
    // function<B(A)>)->set<B> but I have not so far been able to do so in a
    // way template type deduction will accept.
    ModelInstanceResults Models() {
      ModelInstanceSet result;
      for (const BodyIndex id: this->ids_) {
        result.insert(this->search_->plant_->get_body(id).model_instance());
      }
      return ModelInstanceResults(result, this->search_);
    }
  };


  class GeometryFrameResults final
      : public IdResultsBase<geometry::FrameId, GeometryFrameResults> {
   public:
    using Base = IdResultsBase<geometry::FrameId, GeometryFrameResults>;
    GeometryFrameResults(const GeometryFrameResults& state)
        : Base(state.ids_, state.search_) {}
    GeometryFrameResults(const GeometryFrameSet& ids,
                         const MultibodySearch* search)
        : Base(ids, search) {}

    BodyResults Bodies() {
      BodySet result;
      for (const geometry::FrameId id : this->ids_) {
        result.insert(this->search_->plant_->GetBodyFromFrameId(id)->index());
      }
      return BodyResults(result, this->search_);
    }
  };


  class GeometryResults final
      : public IdResultsBase<geometry::GeometryId, GeometryResults> {
   public:
    using Base = IdResultsBase<geometry::GeometryId, GeometryResults>;
    GeometryResults(const GeometryResults& state)
        : Base(state.ids_, state.search_) {}
    GeometryResults(const GeometrySet& ids, const MultibodySearch* search)
        : Base(ids, search) {}

    GeometryFrameResults Frames() {
      GeometryFrameSet result;
      for (const geometry::GeometryId geom_id : this->ids_) {
        result.insert(
            this->search_->scene_graph_->model_inspector().GetFrameId(geom_id));
      }
      return GeometryFrameResults(result, this->search_);
    }
  };


  class MultibodyFrameResults final
      : public ValueResultsBase<FrameIndex, multibody::Frame<T>,
                                MultibodyFrameResults> {
   public:
    using Base = ValueResultsBase<FrameIndex, multibody::Frame<T>,
                                MultibodyFrameResults>;
    MultibodyFrameResults(const MultibodyFrameResults& state)
        : Base(state.ids_, state.search_, state.value_fn_) {}
    MultibodyFrameResults(const MultibodyFrameSet& ids,
                          const MultibodySearch* search)
        : Base(ids, search,
               [this](const FrameIndex id) {
                 return &this->search_->plant_->get_frame(id);
               }) {}

    BodyResults Bodies() {
      BodySet result;
      for (const FrameIndex id : this->ids_) {
        result.insert(this->search_->plant_->get_frame(id).body().index());
      }
      return BodyResults(result, this->search_);
    }

    MultibodyFrameResults AttachedTo(BodyResults bodies) {
      return this->Filter([&](const multibody::Frame<T>& f){
          return bodies.ids().count(f.body().index()); });
    }
  };


  class ModelInstanceResults final
      : public IdResultsBase<ModelInstanceIndex, ModelInstanceResults> {
   public:
    using Base = IdResultsBase<ModelInstanceIndex, ModelInstanceResults>;
    ModelInstanceResults(const ModelInstanceResults& state)
        : Base(state.ids_, state.search_) {}
    ModelInstanceResults(const ModelInstanceSet& ids,
                             const MultibodySearch* search)
        : Base(ids, search) {}

    BodyResults Bodies() {
      BodySet result = MapAndFlattenFromSet<ModelInstanceIndex, BodyIndex>(
          this->ids_, [&](const auto& model){
            return this->search_->plant_->GetBodyIndices(model);
          });
      return BodyResults(result, this->search_);
    }

    ModelInstanceResults Named(const std::string& name) {
      return this->FilterIds([&](const ModelInstanceIndex id) {
          return this->search_->plant_->GetModelInstanceName(id) == name;
        });
    }
  };


  ModelInstanceResults DefaultModel() {
    return ModelInstanceResults({default_model_instance()}, this);
  }

  ModelInstanceResults WorldModel() {
    return ModelInstanceResults({world_model_instance()}, this);
  }

  ModelInstanceResults AllModels() {
    return ModelInstanceResults(
        AllIds<ModelInstanceIndex>(plant_->num_model_instances()), this);
  }

  JointResults AllJoints() {
    return JointResults(AllIds<JointIndex>(plant_->num_joints()), this);
  }

  BodyResults AllBodies() {
    return BodyResults(AllIds<BodyIndex>(plant_->num_bodies()), this);
  }

  GeometryResults AllGeometries() {
    auto geom_ids = scene_graph_->model_inspector().all_geometry_ids();
    return GeometryResults(GeometrySet(geom_ids.begin(), geom_ids.end()), this);
  }

  GeometryFrameResults AllGeometryFrames() {
    auto frame_ids = scene_graph_->model_inspector().all_frame_ids();
    return GeometryFrameResults(
        GeometryFrameSet(frame_ids.begin(), frame_ids.end()), this);
  }

  GeometryFrameResults WorldGeometryFrame() {
    return GeometryFrameResults({scene_graph_->world_frame_id()}, this);
  }

  MultibodyFrameResults AllMultibodyFrames() {
    return MultibodyFrameResults(
        AllIds<FrameIndex>(plant_->num_frames()), this);
  }

  // TODO(ggould-tri) The below operator() overloads should be a single
  // templated overload, once I figure out how to do that with crtp.

  BodyResults WorldBody() {
    return BodyResults({world_index()}, this);
  }

  GeometryResults operator() (const geometry::GeometryId id) {
    return GeometryResults({id}, this);
  }

  GeometryFrameResults operator() (const geometry::FrameId id) {
    return GeometryFrameResults({id}, this);
  }

  JointResults operator() (const JointIndex id) {
    return JointResults({id}, this);
  }

  BodyResults operator() (const BodyIndex id) {
    return BodyResults({id}, this);
  }

  MultibodyFrameResults operator() (const FrameIndex id) {
    return MultibodyFrameResults({id}, this);
  }

  ModelInstanceResults operator() (const ModelInstanceIndex id) {
    return ModelInstanceResults({id}, this);
  }

  private:
  const MultibodyPlant<T>* plant_;
  const geometry::SceneGraph<T>* scene_graph_;
};


}  // namespace multibody
}  // namespace drake
