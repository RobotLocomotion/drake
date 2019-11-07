#include "drake/systems/primitives/constant_vector_source.h"

#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/extract_double.h"

namespace drake {
namespace systems {
namespace {

// Given a ConstantVectorSource with one scalar type, convert its default
// output port value (i.e., its default parameter value) to an Eigen::Vector
// with a different scalar type, by demoting the `U`s down to doubles and then
// promoting back to `T`s.  This is only supported when the output port is
// exactly typed as BasicVector (not a subclass of BasicVector) because
// BasicVector does not support scalar conversion in a way that preserves
// subtyping (see #5454 for some related discussion).
template <typename T, typename U>
VectorX<T> ConvertDefaultValue(const ConstantVectorSource<U>& other) {
  const int size = other.get_output_port().size();
  auto context = other.CreateDefaultContext();
  const BasicVector<U>& old_default = other.get_source_value(*context);
  DRAKE_DEMAND(old_default.size() == size);
  DRAKE_THROW_UNLESS(typeid(old_default) == typeid(BasicVector<U>));
  VectorX<T> new_default = VectorX<T>::Zero(size);
  for (int i = 0; i < size; ++i) {
    new_default[i] = ExtractDoubleOrThrow(old_default[i]);
  }
  return new_default;
}

}  // namespace

// For this overload, we can support scalar conversion because our constant
// value is stored as a parameter and our output port is definitely typed as
// BasicVector (and not some subtype of BasicVector).
template <typename T>
ConstantVectorSource<T>::ConstantVectorSource(
    const Eigen::Ref<const VectorX<T>>& source_value)
    : ConstantVectorSource(
          SystemTypeTag<ConstantVectorSource>{},
          BasicVector<T>(source_value)) {}

// (N.B. This overload also indirectly supports scalar conversion.)
template <typename T>
ConstantVectorSource<T>::ConstantVectorSource(const T& source_value)
    : ConstantVectorSource(Vector1<T>::Constant(source_value)) {}

// This overload cannot support scalar conversion, until we have a way to
// convert a BasicVector subtype between scalar types.
template <typename T>
ConstantVectorSource<T>::ConstantVectorSource(
    const BasicVector<T>& source_value)
    : ConstantVectorSource<T>({}, source_value) {}

template <typename T>
template <typename U>
ConstantVectorSource<T>::ConstantVectorSource(
    const ConstantVectorSource<U>& other)
    : ConstantVectorSource<T>(ConvertDefaultValue<T, U>(other)) {}

// All other constructors delegate here.
template <typename T>
ConstantVectorSource<T>::ConstantVectorSource(
    SystemScalarConverter converter, const BasicVector<T>& source_value)
    : SingleOutputVectorSource<T>(std::move(converter), source_value),
      source_value_index_(this->DeclareNumericParameter(source_value)) {
  // This condition is always true because of the way our constructors delegate.
  DRAKE_DEMAND(this->get_system_scalar_converter().empty() ||
               typeid(source_value) == typeid(BasicVector<T>));
}

template <typename T>
ConstantVectorSource<T>::~ConstantVectorSource() = default;

template <typename T>
void ConstantVectorSource<T>::DoCalcVectorOutput(
    const Context<T>& context, Eigen::VectorBlock<VectorX<T>>* output) const {
  *output = get_source_value(context).get_value();
}

template <typename T>
const BasicVector<T>& ConstantVectorSource<T>::get_source_value(
    const Context<T>& context) const {
  return this->GetNumericParameter(context, source_value_index_);
}

template <typename T>
BasicVector<T>& ConstantVectorSource<T>::get_mutable_source_value(
    Context<T>* context) {
  return this->GetMutableNumericParameter(context, source_value_index_);
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::ConstantVectorSource)
