#pragma once

#include <vector>

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* A leafsystem that manages state and element data in a FEM model.
 @tparam_nonsymbolic_scalar */
template <typename T>
class FemStateManager : public systems::LeafSystem<T> {
 public:
  /* Constructs a new FemStateManager with the given model states. No element
   data is declared.
   @pre model_q, model_v, model_a all have the same size. */
  FemStateManager(const VectorX<T>& model_q, const VectorX<T>& model_v,
                  const VectorX<T>& model_a);

  /* Declares per-element data that depends on all states in this manager.
   @param[in] num_elements       The number of elements/element data.
   @param[in] calc_element_data  A callback that evaluates element data in each
                                 FEM element using the given states.
   @pre num_elements > 0.
   @tparam Data  The data type of the per-element data. */
  template <typename Data>
  void DeclareElementData(
      int num_elements,
      std::function<void(const VectorX<T>& q, const VectorX<T>& v,
                         const VectorX<T>& a, std::vector<Data>* element_data)>
          calc_element_data) {
    DRAKE_THROW_UNLESS(num_elements > 0);
    element_data_index_ =
        this->DeclareCacheEntry(
                "FEM element data",
                systems::ValueProducer(
                    std::function<void(const systems::Context<T>&,
                                       std::vector<Data>*)>{
                        [this, num_elements, calc_element_data](
                            const systems::Context<T>& context,
                            std::vector<Data>* element_data) {
                          DRAKE_DEMAND(element_data != nullptr);
                          element_data->resize(num_elements);
                          const VectorX<T>& q =
                              context.get_discrete_state(q_index_).value();
                          const VectorX<T>& v =
                              context.get_discrete_state(v_index_).value();
                          const VectorX<T>& a =
                              context.get_discrete_state(a_index_).value();
                          calc_element_data(q, v, a, element_data);
                        }}),
                {this->discrete_state_ticket(q_index_),
                 this->discrete_state_ticket(v_index_),
                 this->discrete_state_ticket(a_index_)})
            .cache_index();
  }

  /* Returns the discrete state index. */
  systems::DiscreteStateIndex fem_position_index() const { return q_index_; }
  systems::DiscreteStateIndex fem_velocity_index() const { return v_index_; }
  systems::DiscreteStateIndex fem_acceleration_index() const {
    return a_index_;
  }

  /* Returns the element data cache index.
   @throws std::exception if DeclareElementData hasn't been called. */
  systems::CacheIndex element_data_index() const {
    DRAKE_THROW_UNLESS(element_data_index_.is_valid());
    return element_data_index_;
  }

 private:
  systems::DiscreteStateIndex q_index_;
  systems::DiscreteStateIndex v_index_;
  systems::DiscreteStateIndex a_index_;
  systems::CacheIndex element_data_index_;
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
