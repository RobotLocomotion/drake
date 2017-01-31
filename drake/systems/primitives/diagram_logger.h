#pragma once

#include <string>
#include <unordered_map>

#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/signal_logger.h"

namespace drake {
namespace systems {

template <typename T>
class DiagramLogger {
 public:
  explicit DiagramLogger(int batch_allocation_size = 1000)
    : batch_allocation_size_(batch_allocation_size) {}
  DiagramLogger(const DiagramLogger<T>&) = delete;
  DiagramLogger& operator=(const DiagramLogger<T>&) = delete;

  /**
   * Add a SignalLogger system to @p diagram_builder that is connected to
   * @p src. This port can be accessed later with @p trace_name.
   * @param trace_name Name for this logger.
   * @param src Output source to be logged.
   * @param diagram_builder Diagram builder
   * @return true if a SignalLogger is successfully added. False if
   * @p trace_name already exists, or @p src is not vector valued.
   */
  bool LogOutputPort(const std::string& trace_name,
                     const OutputPortDescriptor<T>& src,
                     DiagramBuilder<T>* diagram_builder);

  /**
   * Returns a const pointer to the SignalLogger whose name is @p trace_name.
   * nullptr is returned if @p trace_name cannot be found.
   */
  const SignalLogger<T>* get_logger(const std::string& trace_name) const;

 private:
  int batch_allocation_size_{1000};
  std::unordered_map<std::string, const SignalLogger<T>*> loggers_;
};

}  // namespace systems
}  // namespace drake
