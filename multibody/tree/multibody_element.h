#pragma once

#include <set>
#include <string>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/multibody/tree/multibody_tree_system.h"

namespace drake {
namespace multibody {

template <typename T>
class MultibodyPlant;

template <typename T>
class DeformableModel;

/// A class representing an element (subcomponent) of a MultibodyPlant or
/// (internally) a MultibodyTree. Examples of multibody elements are bodies,
/// joints, force elements, and actuators. After a Finalize() call, multibody
/// elements get assigned a type-specific index that uniquely identifies them.
/// By convention, every subclass of MultibodyElement provides an `index()`
/// member function that returns the assigned index, e.g.,
///
/// @code
/// /** Returns this element's unique index. */
/// BodyIndex index() const { return this->template index_impl<BodyIndex>(); }
/// @endcode
///
/// Some multibody elements are added during Finalize() and are not part of
/// the user-specified model. These are called "ephemeral" elements and can
/// be identified using the `is_ephemeral()` function here. Examples include
///   - free joints added to connect lone bodies or free-floating trees
///     to World
///   - fixed offset frames added when joints are modeled by mobilizers
///   - all mobilizers.
///
/// @tparam_default_scalar
template <typename T>
class MultibodyElement {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultibodyElement);

  virtual ~MultibodyElement();

  /// Returns the ModelInstanceIndex of the model instance to which this
  /// element belongs.
  ModelInstanceIndex model_instance() const { return model_instance_; }

  /// Returns the MultibodyPlant that owns this %MultibodyElement.
  ///
  /// @note You can only invoke this method if you have a definition of
  /// %MultibodyPlant available. That is, you must include
  /// `drake/multibody/plant/multibody_plant.h` in the translation unit that
  /// invokes this method; multibody_element.h cannot do that for you.
  ///
  /// @throws std::exception if there is no %MultibodyPlant owner.
  template <typename = void>
  const MultibodyPlant<T>& GetParentPlant() const;
  // N.B. The implementation of GetParentPlant is provided as part of the plant
  // library in multibody_element_getter.cc, to avoid a dependency cycle.

  /// Declares MultibodyTreeSystem Parameters at MultibodyTreeSystem::Finalize()
  /// time. NVI to the virtual method DoDeclareParameters().
  /// @param[in] tree_system A mutable copy of the parent MultibodyTreeSystem.
  /// @pre 'tree_system' must be the same as the parent tree system (what's
  /// returned from GetParentTreeSystem()).
  void DeclareParameters(internal::MultibodyTreeSystem<T>* tree_system);

  /// Sets default values of parameters belonging to each %MultibodyElement in
  /// `parameters` at a call to MultibodyTreeSystem::SetDefaultParameters().
  /// @param[out] parameters A mutable collections of parameters in a context.
  /// @pre parameters != nullptr
  void SetDefaultParameters(systems::Parameters<T>* parameters) const;

  /// Declares MultibodyTreeSystem discrete states. NVI to the virtual method
  /// DoDeclareDiscreteState().
  /// @param[in] tree_system A mutable copy of the parent MultibodyTreeSystem.
  /// @pre 'tree_system' must be the same as the parent tree system (what's
  /// returned from GetParentTreeSystem()).
  void DeclareDiscreteState(internal::MultibodyTreeSystem<T>* tree_system);

  /// (Advanced) Declares all cache entries needed by this element.
  /// This method is called by MultibodyTree on `this` element during
  /// MultibodyTree::Finalize(). It subsequently calls DoDeclareCacheEntries().
  /// Custom elements that need to declare cache entries must override
  /// DoDeclareCacheEntries().
  void DeclareCacheEntries(internal::MultibodyTreeSystem<T>* tree_system);

  /// Returns `true` if this %MultibodyElement was added during Finalize()
  /// rather than something a user added. (See class comments.)
  bool is_ephemeral() const { return is_ephemeral_; }

  /// (Internal use only) Sets the `is_ephemeral` flag to the indicated value.
  /// The default if this is never called is `false`. Any element that is added
  /// during Finalize() should set this flag to `true`.
  void set_is_ephemeral(bool is_ephemeral) { is_ephemeral_ = is_ephemeral; }

 protected:
  /// Default constructor made protected so that sub-classes can still declare
  /// their default constructors if they need to.
  MultibodyElement();

  /// Constructor which allows specifying a model instance.
  explicit MultibodyElement(ModelInstanceIndex model_instance);

  /// Both the model instance and element index are specified.
  explicit MultibodyElement(ModelInstanceIndex model_instance, int64_t index);

  /// Returns this element's unique index.
  template <typename ElementIndexType>
  ElementIndexType index_impl() const {
    DRAKE_ASSERT(index_ >= 0);
    return ElementIndexType{index_};
  }

  /// Returns this element's unique ordinal.
  /// @note The int64_t default is present for backwards compatibility but
  /// you should not use it. Instead, define a ThingOrdinal specialization of
  /// TypeSafeIndex for any element Thing that has a meaningful ordinal. Then
  /// use that type explicitly in Thing's public `ordinal()` method.
  template <typename ElementOrdinalType = int64_t>
  ElementOrdinalType ordinal_impl() const {
    DRAKE_ASSERT(ordinal_ >= 0);
    return ElementOrdinalType{ordinal_};
  }

  /// Returns a constant reference to the parent MultibodyTree that owns this
  /// element.
  /// @pre has_parent_tree is true.
  const internal::MultibodyTree<T>& get_parent_tree() const {
    if constexpr (kDrakeAssertIsArmed) {
      if (parent_tree_ == nullptr) {
        ThrowNoParentTree();
      }
    }
    return *parent_tree_;
  }

  /// Returns a constant reference to the parent MultibodyTreeSystem that
  /// owns the parent MultibodyTree that owns this element.
  /// @throws std::exception if has_parent_tree() is false.
  const internal::MultibodyTreeSystem<T>& GetParentTreeSystem() const {
    if (parent_tree_ == nullptr) {
      ThrowNoParentTree();
    }
    return get_parent_tree().tree_system();
  }

  /// (Internal use only) Gives MultibodyElement-derived objects the opportunity
  /// to set data members that depend on topology and coordinate assignments
  /// having been finalized. This is invoked at the end of
  /// MultibodyTree::Finalize(). NVI to pure virtual method DoSetTopology().
  void SetTopology() { DoSetTopology(); }

  /// Implementation of the NVI SetTopology(). For advanced use only for
  /// developers implementing new MultibodyTree components.
  virtual void DoSetTopology() = 0;

  /// Implementation of the NVI DeclareParameters(). MultibodyElement-derived
  /// objects may override to declare their specific parameters.
  virtual void DoDeclareParameters(internal::MultibodyTreeSystem<T>*);

  /// Implementation of the NVI SetDefaultParameters(). MultibodyElement-derived
  /// objects may override to set default values of their specific parameters.
  virtual void DoSetDefaultParameters(systems::Parameters<T>*) const;

  /// Implementation of the NVI DeclareDiscreteState(). MultibodyElement-derived
  /// objects may override to declare their specific state variables.
  virtual void DoDeclareDiscreteState(internal::MultibodyTreeSystem<T>*);

  /// Derived classes must override this method to declare cache entries
  /// needed by `this` element. The default implementation is a no-op.
  virtual void DoDeclareCacheEntries(internal::MultibodyTreeSystem<T>*);

  /// To be used by MultibodyElement-derived objects when declaring parameters
  /// in their implementation of DoDeclareParameters(). For an example, see
  /// RigidBody::DoDeclareParameters().
  systems::NumericParameterIndex DeclareNumericParameter(
      internal::MultibodyTreeSystem<T>* tree_system,
      const systems::BasicVector<T>& model_vector);

  /// To be used by MultibodyElement-derived objects when declaring parameters
  /// in their implementation of DoDeclareParameters(). For an example, see
  /// Joint::DoDeclareParameters().
  systems::AbstractParameterIndex DeclareAbstractParameter(
      internal::MultibodyTreeSystem<T>* tree_system,
      const AbstractValue& model_value);

  /// To be used by MultibodyElement-derived objects when declaring discrete
  /// states in their implementation of DoDeclareDiscreteStates(). For an
  /// example, see DeformableBody::DoDeclareDiscreteStates().
  systems::DiscreteStateIndex DeclareDiscreteState(
      internal::MultibodyTreeSystem<T>* tree_system,
      const VectorX<T>& model_value);

  /// To be used by MultibodyElement-derived objects when declaring cache
  /// entries in their implementation of DoDeclareCacheEntries(). For an
  /// example, see DeformableBody::DoDeclareCacheEntries().
  systems::CacheEntry& DeclareCacheEntry(
      internal::MultibodyTreeSystem<T>* tree_system, std::string description,
      systems::ValueProducer value_producer,
      std::set<systems::DependencyTicket> prerequisites_of_calc);

  /// Returns true if this multibody element has a parent tree, otherwise false.
  bool has_parent_tree() const { return parent_tree_ != nullptr; }

 private:
  // MultibodyTree<T> is a natural friend of MultibodyElement objects and
  // therefore it can set the owning parent tree and unique index in that tree.
  friend class internal::MultibodyTree<T>;
  // Similarly, friend DeformableModel<T> to allow it to set the parent tree and
  // index for deformable bodies.
  friend class DeformableModel<T>;

  // Give unit tests access to the tree.
  friend class MultibodyElementTester;

  void set_parent_tree(const internal::MultibodyTree<T>* tree, int64_t index) {
    index_ = index;
    parent_tree_ = tree;
  }

  void set_ordinal(int64_t ordinal) { ordinal_ = ordinal; }

  void set_model_instance(ModelInstanceIndex model_instance) {
    model_instance_ = model_instance;
  }

  // @throws std::exception that this element is not in a MultibodyTree.
  [[noreturn]] void ThrowNoParentTree() const;

  // Checks whether this MultibodyElement belongs to the provided
  // MultibodyTree `tree` and throws an exception if not.
  // @throws std::exception if `this` element is not in the given `tree`.
  void HasThisParentTreeOrThrow(const internal::MultibodyTree<T>* tree) const;

  const internal::MultibodyTree<T>* parent_tree_{nullptr};

  // The default index value is *invalid*. This must be set to a valid index
  // value before the element is released to the wild.
  int64_t index_{-1};

  // Keeps track of the index into contiguous containers that have an entry
  // for each of a concrete MultibodyElement type (Joint, RigidBody, etc.)
  // Ordinals should be updated upon element removal so that the ordinals always
  // form a contiguous sequence from 0 to N-1, where N is the number of a
  // particular element type. Default ordinal value is *invalid*. Concrete
  // MultibodyElements may choose to not expose this ordinal if not needed (e.g.
  // if MultibodyPlant does not expose any port that has an entry per concrete
  // MultibodyElement type.) This must be set to a valid ordinal value before
  // the element is released to the wild.
  int64_t ordinal_{-1};

  // The default model instance id is *invalid*. This must be set to a
  // valid index value before the element is released to the wild.
  ModelInstanceIndex model_instance_;

  bool is_ephemeral_{false};
};

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::MultibodyElement);
