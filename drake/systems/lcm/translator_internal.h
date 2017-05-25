#pragma once

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {
namespace lcm {
namespace translator_internal {

template <typename DataType, bool is_vector>
struct DataTypeTraits {};

template <typename DataType>
struct DataTypeTraits<DataType, true> {
  static const DataType& get_data(const System<double>& sys,
                                  const Context<double>& context) {
    const DataType* const vector =
        dynamic_cast<const DataType*>(sys.EvalVectorInput(context, 0));
    DRAKE_DEMAND(vector != nullptr);
    return *vector;
  }

  static DataType& get_mutable_data(SystemOutput<double>* output) {
    DataType* const vector =
        dynamic_cast<DataType*>(output->GetMutableVectorData(0));
    DRAKE_DEMAND(vector != nullptr);
    return *vector;
  }
};

template <typename DataType>
struct DataTypeTraits<DataType, false> {
  static const DataType& get_data(const System<double>& sys,
                                  const Context<double>& context) {
    const AbstractValue* const value = sys.EvalAbstractInput(context, 0);
    DRAKE_DEMAND(value != nullptr);
    return value->GetValue<DataType>();
  }

  static DataType& get_mutable_data(SystemOutput<double>* output) {
    return output->GetMutableData(0)->GetMutableValue<DataType>();
  }
};

}  // namespace translator_internal
}  // namespace lcm
}  // namespace systems
}  // namespace drake
