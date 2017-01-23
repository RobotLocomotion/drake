#include "drake/systems/primitives/diagram_logger.h"

#include <memory>

namespace drake {
namespace systems {

template <typename T>
bool DiagramLogger<T>::LogOutputPort(const std::string& trace_name,
                                  const OutputPortDescriptor<T>& src,
                                  DiagramBuilder<T>* diagram_builder) {
  if (loggers_.find(trace_name) != loggers_.end()) {
    return false;
  }
  if (src.get_data_type() != kVectorValued) {
    return false;
  }

  const SignalLogger<T>* logger = diagram_builder->AddSystem(
      std::make_unique<SignalLogger<T>>(src.size()));
  loggers_.emplace(trace_name, logger);

  diagram_builder->Connect(src, logger->get_input_port(0));
  return true;
}

template <typename T>
const SignalLogger<T>* DiagramLogger<T>::get_logger(
    const std::string& trace_name) const {
  auto it = loggers_.find(trace_name);
  if (it != loggers_.end()) return it->second;
  return nullptr;
}

template class DiagramLogger<double>;
template class DiagramLogger<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
