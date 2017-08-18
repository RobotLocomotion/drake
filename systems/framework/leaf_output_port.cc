#include "drake/systems/framework/leaf_output_port.h"

#include <memory>
#include <sstream>
#include <string>

#include "drake/common/autodiff.h"
#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/nice_type_name.h"
#include "drake/common/symbolic.h"
#include "drake/systems/framework/cache_entry.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace systems {

template <typename T>
LeafOutputPort<T>::LeafOutputPort(
    AllocCallback alloc_function, CalcCallback calc_function,
    std::vector<DependencyTicket> calc_prerequisites, System<T>* system)
    : OutputPort<T>(kAbstractValued, 0 /* size */, system) {
  CompleteConstruction(std::move(alloc_function), std::move(calc_function),
                       std::move(calc_prerequisites), system);
}

template <typename T>
LeafOutputPort<T>::LeafOutputPort(
    int fixed_size, AllocCallback alloc_function,
    CalcVectorCallback vector_calc_function,
    std::vector<DependencyTicket> calc_prerequisites, System<T>* system)
    : OutputPort<T>(kVectorValued, fixed_size, system) {

  CompleteConstruction(std::move(alloc_function),
                       ConvertVectorCallback(std::move(vector_calc_function)),
                       std::move(calc_prerequisites), system);
}

// Private method to be called from cached output port constructors, after the
// base class construction is complete. Records the functors and Calc()
// prerequisites, and allocates an appropriate cache entry.
template <typename T>
void LeafOutputPort<T>::CompleteConstruction(
    AllocCallback alloc_function, CalcCallback calc_function,
    std::vector<DependencyTicket> calc_prerequisites, System<T>* system) {
  DRAKE_DEMAND(system != nullptr);
  alloc_function_ = std::move(alloc_function);
  calc_function_ = std::move(calc_function);

  if (calc_prerequisites.empty())
    calc_prerequisites.push_back(system->all_sources_ticket());

  auto cache_alloc_function = [alloc = alloc_function_](
      const ContextBase& context_base) {
    const Context<T>& context = dynamic_cast<const Context<T>&>(context_base);
    return alloc(context);
  };
  auto cache_calc_function = [calc = calc_function_](
      const ContextBase& context_base, AbstractValue* result) {
    const Context<T>& context = dynamic_cast<const Context<T>&>(context_base);
    return calc(context, result);
  };
  cache_index_ =
      system
          ->DeclareCacheEntry(
              "output port " + std::to_string(this->get_index()) + " cache",
              std::move(cache_alloc_function), std::move(cache_calc_function),
              std::move(calc_prerequisites))
          .cache_index();
}

template <typename T>
auto LeafOutputPort<T>::ConvertVectorCallback(
    CalcVectorCallback vector_calc_function) -> CalcCallback {
  if (!vector_calc_function) return nullptr;

  // Wrap the vector-writing function with an AbstractValue-writing function.
  return [this, vector_calc_function](const Context<T>& context,
                                      AbstractValue* abstract) {
    // The abstract value must be a Value<BasicVector<T>>.
    auto value = dynamic_cast<Value<BasicVector<T>>*>(abstract);
    if (value == nullptr) {
      std::ostringstream oss;
      oss << "LeafOutputPort::Calc(): Expected a vector output type for "
          << this->GetPortIdString() << " but got a "
          << NiceTypeName::Get(*abstract) << " instead.";
      throw std::logic_error(oss.str());
    }
    vector_calc_function(context, &value->get_mutable_value());
  };
}

template <typename T>
std::unique_ptr<AbstractValue> LeafOutputPort<T>::DoAllocate(
    const Context<T>& context) const {
  std::unique_ptr<AbstractValue> result;

  // Use the allocation function if available, otherwise clone the model
  // value.
  if (alloc_function_) {
    result = alloc_function_(context);
  } else {
    throw std::logic_error(
        "LeafOutputPort::DoAllocate(): " + this->GetPortIdString() +
        " has no allocation function so cannot be allocated.");
  }
  if (result.get() == nullptr) {
    throw std::logic_error(
        "LeafOutputPort::DoAllocate(): allocator returned a nullptr for " +
            this->GetPortIdString());
  }
  return result;
}

template <typename T>
void LeafOutputPort<T>::DoCalc(const Context<T>& context,
                               AbstractValue* value) const {
  if (calc_function_) {
    calc_function_(context, value);
  } else {
    throw std::logic_error("LeafOutputPort::DoCalc(): " +
                           this->GetPortIdString() +
                           " had no calculation function available.");
  }
}

template <typename T>
const AbstractValue& LeafOutputPort<T>::DoEval(
    const Context<T>& context) const {
  if (eval_function_)
    return eval_function_(context);
  const CacheEntry& entry =
      this->get_system().get_cache_entry(cache_index_);
  return entry.EvalAbstract(context);
}

template <typename T>
std::pair<optional<SubsystemIndex>, DependencyTicket>
LeafOutputPort<T>::DoGetPrerequisite() const {
  const CacheEntry& entry =
      this->get_system().get_cache_entry(cache_index_);
  return std::make_pair(nullopt, entry.ticket());
}

// The Vector2/3 instantiations here are for the benefit of some
// older unit tests but are not otherwise advertised.
template class LeafOutputPort<Eigen::AutoDiffScalar<Eigen::Vector2d>>;
template class LeafOutputPort<Eigen::AutoDiffScalar<Eigen::Vector3d>>;

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::LeafOutputPort)
