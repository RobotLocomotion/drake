#pragma once

#include <memory>
#include <string>

#include "drake/automotive/maliput/api/rules/traffic_light_book.h"

namespace drake {
namespace maliput {

/// Instantiates and returns an api::rules::TrafficLightBook instance based
/// on the specified @p input document.
///
/// @param input The YAML TrafficLights document.
///
/// @return The newly created api::rules::TrafficLightBook instance.
std::unique_ptr<api::rules::TrafficLightBook> LoadTrafficLightBook(
    const std::string& input);

/// Instantiates and returns an api::rules::TrafficLightBook instance based
/// on the specified @p filename.
///
/// @param filename The YAML file that contains a TrafficLights document.
///
/// @return The newly created api::rules::TrafficLightBook instance.
std::unique_ptr<api::rules::TrafficLightBook> LoadTrafficLightBookFromFile(
    const std::string& filename);

}  // namespace maliput
}  // namespace drake
