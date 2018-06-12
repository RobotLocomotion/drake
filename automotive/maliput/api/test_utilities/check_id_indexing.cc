#include "drake/automotive/maliput/api/test_utilities/check_id_indexing.h"

#include "drake/automotive/maliput/api/test_utilities/rules_test_utilities.h"

namespace drake {
namespace maliput {
namespace api {

namespace rules {
namespace test {
template <typename T>
::testing::AssertionResult IsEqual(const char* a_expression,
                                   const char* b_expression,
                                   const T* a,
                                   const T* b) {
  return ::testing::internal::CmpHelperEQ(a_expression, b_expression, a, b);
}
}  // namespace test
}  // namespace rules

namespace test {


::testing::AssertionResult CheckIdIndexing(const RoadGeometry* road_geometry) {
  rules::test::AssertionResultCollector c;
  for (int ji = 0; ji < road_geometry->num_junctions(); ++ji) {
    const api::Junction* junction = road_geometry->junction(ji);
    MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(
        road_geometry->ById().GetJunction(junction->id()), junction));
    for (int si = 0; si < junction->num_segments(); ++si) {
      const api::Segment* segment = junction->segment(si);
      MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(
          road_geometry->ById().GetSegment(segment->id()), segment));
      for (int li = 0; li < segment->num_lanes(); ++li) {
        const api::Lane* lane = segment->lane(li);
        MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(
            road_geometry->ById().GetLane(lane->id()), lane));
      }
    }
    for (int bi = 0; bi < road_geometry->num_branch_points(); ++bi) {
      const api::BranchPoint* branch_point = road_geometry->branch_point(bi);
      MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(
          road_geometry->ById().GetBranchPoint(branch_point->id()),
          branch_point));
    }
  }
  return c.result();
}


}  // namespace test
}  // namespace api
}  // namespace maliput
}  // namespace drake
