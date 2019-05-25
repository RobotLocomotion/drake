#pragma once

#include <memory>
#include <string>

#include "drake/automotive/maliput/api/intersection_book.h"
#include "drake/automotive/maliput/api/rules/phase_ring_book.h"
#include "drake/automotive/maliput/api/rules/road_rulebook.h"
#include "drake/automotive/maliput/base/manual_phase_provider.h"

namespace drake {
namespace maliput {

/// Instantiates and returns an api::IntersectionBook instance based on the
/// specified @p input document.
///
/// @param input The YAML Intersections document.
/// @param road_rulebook The book containing the road rules.
/// @param phase_ring_book The book containing the phase rings.
/// @param phase_provider The phase provider. Adds PhaseRings and sets their
/// initial states.
///
/// @return The newly created api::IntersectionBook instance.
std::unique_ptr<api::IntersectionBook> LoadIntersectionBook(
    const std::string& input, const api::rules::RoadRulebook& road_rulebook,
    const api::rules::PhaseRingBook& phase_ring_book,
    ManualPhaseProvider* phase_provider);

/// Instantiates and returns an api::IntersectionBook instance based on the
/// specified @p filename.
///
/// @param filename The YAML file that contains a Intersections document.
/// @param road_rulebook The book containing the road rules.
/// @param phase_ring_book The book containing the phase rings.
/// @param phase_provider The phase provider. Adds PhaseRings and sets their
/// initial states.
///
/// @return The newly created api::IntersectionBook instance.
std::unique_ptr<api::IntersectionBook> LoadIntersectionBookFromFile(
    const std::string& filename, const api::rules::RoadRulebook& road_rulebook,
    const api::rules::PhaseRingBook& phase_ring_book,
    ManualPhaseProvider* phase_provider);

}  // namespace maliput
}  // namespace drake
