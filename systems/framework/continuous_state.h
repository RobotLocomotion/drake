#pragma once

#include <memory>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/systems/framework/framework_common.h"
#include "drake/systems/framework/scalar_conversion_traits.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace systems {

// TODO(sherm1) The ordering of the composite xc is useless and prevents us
//              from describing every xc as a sequence [q v z]. Consider
//              reimplementing so that xc=[q₁q₂ v₁v₂ z₁z₂].
/// %ContinuousState is a view of, and optionally a container for, all the
/// continuous state variables `xc` of a Drake System. Continuous state
/// variables are those whose values are defined by differential equations,
/// so we expect there to be a well-defined time derivative `xcdot` ≜ `d/dt xc`.
///
/// The contents of `xc` are conceptually partitioned into three groups:
///
/// - `q` is generalized position
/// - `v` is generalized velocity
/// - `z` is other continuous state
///
/// For a Drake LeafSystem these partitions are stored contiguously in memory
/// in this sequence: xc=[q v z]. But because a Drake System may be a Diagram
/// composed from subsystems, each with its own continuous state variables
/// ("substates"), the composite continuous state will not generally be stored
/// in contiguous memory. In that case the most we can say is that xc={q,v,z},
/// that is, it consists of all the q's, v's, and z's, in some order.
///
/// Nevertheless, this %ContinuousState class provides a vector
/// view of the data that groups together all the q partitions, v
/// partitions, and z partitions. For example, if there are three subsystems
/// (possibly Diagrams) whose continuous state variables are respectively
/// xc₁={q₁,v₁,z₁}, xc₂={q₂,v₂,z₂}, and xc₃={q₃,v₃,z₃} the composite xc includes
/// all the partitions in an undefined order. However, composite q, v, and z
/// appear ordered as q=[q₁ q₂ q₃], v=[v₁ v₂ v₃], z=[z₁ z₂ z₃]. Note that the
/// element ordering of the composite xc is _not_ a concatenation of the
/// composite subgroups. Do not index elements of the full state xc unless you
/// know it is the continuous state of a LeafSystem (a LeafSystem looking at its
/// own Context can depend on that).
///
/// Any of the groups may be empty. However, groups q and v must be either both
/// present or both empty, because the time derivative `qdot` of the
/// second-order state variables `q` must be computable using a linear mapping
/// `qdot=N(q)*v`.
///
/// The time derivative `xcdot` has the identical substructure to `xc`, with the
/// partitions interpreted as `qdot`, `vdot`, and `zdot`. We use identical
/// %ContinuousState objects for both.
///
/// <h4>Memory ownership</h4>
/// When a %ContinuousState represents the state of a LeafSystem, it always
/// owns the memory that is used for the state variables and is responsible
/// for destruction. For a Diagram, %ContinuousState can instead be a _view_
/// of the underlying LeafSystem substates, so that modifying the Diagram's
/// continuous state affects the LeafSystems appropriately. In that case, the
/// memory is owned by the underlying LeafSystems. However, when a
/// %ContinuousState object of any structure is cloned, the resulting object
/// _always_ owns all its underlying memory, which is initialized with a copy
/// of the original state variable values but is otherwise independent. The
/// cloned object retains the structure and ordering of the elements and does
/// not guarantee contiguous storage.
/// @see DiagramContinuousState for more information.
///
/// @tparam_default_scalar
template <typename T>
class ContinuousState {
 public:
  // ContinuousState is not copyable or moveable, but can be cloned with some
  // caveats.
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ContinuousState)

  /// Constructs a ContinuousState for a system that does not have second-order
  /// structure. The `q` and `v` partitions are empty; all of the state `xc` is
  /// miscellaneous continuous state `z`.
  explicit ContinuousState(std::unique_ptr<VectorBase<T>> state);

  /// Constructs a ContinuousState that exposes second-order structure.
  ///
  /// @param state The source xc of continuous state information.
  /// @param num_q The number of position variables q.
  /// @param num_v The number of velocity variables v.
  /// @param num_z The number of other continuous variables z.
  ///
  /// We require that `num_q ≥ num_v` and that the sum of the partition sizes
  /// adds up to the size of `state`.
  ContinuousState(std::unique_ptr<VectorBase<T>> state, int num_q, int num_v,
                  int num_z);

  /// Constructs a zero-length ContinuousState.
  ContinuousState();

  virtual ~ContinuousState();

  /// Creates a deep copy of this object with the same substructure but with all
  /// data owned by the copy. That is, if the original was a Diagram continuous
  /// state that merely referenced substates, the clone will not include any
  /// references to the original substates and is thus decoupled from the
  /// Context containing the original. The concrete type of the BasicVector
  /// underlying each leaf ContinuousState is preserved. See the class comments
  /// above for more information.
  std::unique_ptr<ContinuousState<T>> Clone() const;

  /// Returns the size of the entire continuous state vector, which is
  /// necessarily `num_q + num_v + num_z`.
  int size() const { return get_vector().size(); }

  /// Returns the number of generalized positions q in this state vector.
  int num_q() const { return get_generalized_position().size(); }

  /// Returns the number of generalized velocities v in this state vector.
  int num_v() const { return get_generalized_velocity().size(); }

  /// Returns the number of miscellaneous continuous state variables z
  /// in this state vector.
  int num_z() const { return get_misc_continuous_state().size(); }

  T& operator[](std::size_t idx) { return (*state_)[idx]; }
  const T& operator[](std::size_t idx) const { return (*state_)[idx]; }

  /// Returns a reference to the entire continuous state vector.
  const VectorBase<T>& get_vector() const {
    DRAKE_ASSERT(state_ != nullptr);
    return *state_;
  }

  /// Returns a mutable reference to the entire continuous state vector.
  VectorBase<T>& get_mutable_vector() {
    DRAKE_ASSERT(state_ != nullptr);
    return *state_.get();
  }

  /// Returns a const reference to the subset of the state vector that is
  /// generalized position `q`. May be zero length.
  const VectorBase<T>& get_generalized_position() const {
    return *generalized_position_;
  }

  /// Returns a mutable reference to the subset of the state vector that is
  /// generalized position `q`. May be zero length.
  VectorBase<T>& get_mutable_generalized_position() {
    return *generalized_position_.get();
  }

  /// Returns a const reference to the subset of the continuous state vector
  /// that is generalized velocity `v`. May be zero length.
  const VectorBase<T>& get_generalized_velocity() const {
    return *generalized_velocity_;
  }

  /// Returns a mutable reference to the subset of the continuous state vector
  /// that is generalized velocity `v`. May be zero length.
  VectorBase<T>& get_mutable_generalized_velocity() {
    return *generalized_velocity_.get();
  }

  /// Returns a const reference to the subset of the continuous state vector
  /// that is other continuous state `z`. May be zero length.
  const VectorBase<T>& get_misc_continuous_state() const {
    return *misc_continuous_state_;
  }

  /// Returns a mutable reference to the subset of the continuous state vector
  /// that is other continuous state `z`. May be zero length.
  VectorBase<T>& get_mutable_misc_continuous_state() {
    return *misc_continuous_state_.get();
  }

  /// Copies the values from `other` into `this`, converting the scalar type as
  /// necessary.
  template <typename U>
  void SetFrom(const ContinuousState<U>& other) {
    DRAKE_THROW_UNLESS(size() == other.size());
    DRAKE_THROW_UNLESS(num_q() == other.num_q());
    DRAKE_THROW_UNLESS(num_v() == other.num_v());
    DRAKE_THROW_UNLESS(num_z() == other.num_z());
    SetFromVector(other.CopyToVector().unaryExpr(
        scalar_conversion::ValueConverter<T, U>{}));
  }

  /// Sets the entire continuous state vector from an Eigen expression.
  void SetFromVector(const Eigen::Ref<const VectorX<T>>& value) {
    DRAKE_ASSERT(value.size() == state_->size());
    this->get_mutable_vector().SetFromVector(value);
  }

  /// Returns a copy of the entire continuous state vector into an Eigen vector.
  VectorX<T> CopyToVector() const { return this->get_vector().CopyToVector(); }

  /** @name System compatibility
  See @ref system_compatibility. */
  //@{
  /** (Internal) Gets the id of the subsystem that created this state. */
  internal::SystemId get_system_id() const { return system_id_; }

  /** (Internal) Records the id of the subsystem that created this state. */
  void set_system_id(internal::SystemId id) { system_id_ = id; }
  //@}

 protected:
  /// Constructs a continuous state that exposes second-order structure, with
  /// no particular constraints on the layout.
  ///
  /// @pre The q, v, z are all views into the same storage as @p state.
  ///
  /// @param state The entire continuous state.
  /// @param q The subset of state that is generalized position.
  /// @param v The subset of state that is generalized velocity.
  /// @param z The subset of state that is neither position nor velocity.
  ContinuousState(std::unique_ptr<VectorBase<T>> state,
                  std::unique_ptr<VectorBase<T>> q,
                  std::unique_ptr<VectorBase<T>> v,
                  std::unique_ptr<VectorBase<T>> z);

  /// DiagramContinuousState must override this to maintain the necessary
  /// internal substructure, and to perform a deep copy so that the result
  /// owns all its own data. The default implementation here requires that the
  /// full state is a BasicVector (that is, this is a leaf continuous state).
  /// The BasicVector is cloned to preserve its concrete type and contents,
  /// then the q, v, z Subvectors are created referencing it.
  /// The implementation should not set_system_id on the result, the caller
  /// will set an id on the state after this method returns.
  virtual std::unique_ptr<ContinuousState> DoClone() const;

 private:
  // Demand that the representation invariants hold.
  void DemandInvariants() const;

  // The entire continuous state vector.  May or may not own the underlying
  // data.
  std::unique_ptr<VectorBase<T>> state_;

  // Generalized coordinates representing System configuration, conventionally
  // denoted `q`. These are second-order state variables.
  // This is a subset of state_ and does not own the underlying data.
  std::unique_ptr<VectorBase<T>> generalized_position_;

  // Generalized speeds representing System velocity. Conventionally denoted
  // `v`. These are first-order state variables that the System can linearly
  // map to time derivatives `qdot` of `q` above.
  // This is a subset of state_ and does not own the underlying data.
  std::unique_ptr<VectorBase<T>> generalized_velocity_;

  // Additional continuous, first-order state variables not representing
  // multibody system motion.  Conventionally denoted `z`.
  // This is a subset of state_ and does not own the underlying data.
  std::unique_ptr<VectorBase<T>> misc_continuous_state_;

  // Unique id of the subsystem that created this state.
  internal::SystemId system_id_;
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::ContinuousState)
