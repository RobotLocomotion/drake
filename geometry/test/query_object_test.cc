#include "drake/geometry/query_object.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/geometry/geometry_context.h"
#include "drake/geometry/geometry_system.h"
#include "drake/geometry/test_utilities/expect_error_message.h"

namespace drake {
namespace geometry {

// Helper class for testing the query object.
class QueryObjectTester {
 public:
  QueryObjectTester() = delete;

  template <typename T>
  static std::unique_ptr<QueryObject<T>> MakeQueryObject() {
    return std::unique_ptr<QueryObject<T>>(new QueryObject<T>());
  }

  template <typename T>
  static std::unique_ptr<QueryObject<T>> MakeQueryObject(
      const GeometryContext<T>* context, const GeometrySystem<T>* system) {
    auto q = std::unique_ptr<QueryObject<T>>(new QueryObject<T>());
    q->system_ = system;
    q->context_ = context;
    return q;
  }

  template <typename T>
  static void expect_default(const QueryObject<T>& object) {
    EXPECT_EQ(object.system_, nullptr);
    EXPECT_EQ(object.context_, nullptr);
  }

  template <typename T>
  static void expect_live(const QueryObject<T>& object) {
    EXPECT_NE(object.system_, nullptr);
    EXPECT_NE(object.context_, nullptr);
  }

  template <typename T>
  static void ThrowIfDefault(const QueryObject<T>& object) {
    object.ThrowIfDefault();
  }
};

namespace {

using std::unique_ptr;
using systems::Context;

class QueryObjectTest : public ::testing::Test {
 protected:
  using QOT = QueryObjectTester;

  void SetUp() override {
    context_ = system_.AllocateContext();
    geom_context_ = dynamic_cast<GeometryContext<double>*>(context_.get());
    ASSERT_NE(geom_context_, nullptr);
    query_object_ = QOT::MakeQueryObject(geom_context_, &system_);

    QueryObjectTester::expect_live(*query_object_);
  }

  GeometrySystem<double> system_;
  unique_ptr<Context<double>> context_;
  GeometryContext<double>* geom_context_{nullptr};
  unique_ptr<QueryObject<double>> query_object_;
};

// Confirm copy semantics.
TEST_F(QueryObjectTest, CopySemantics) {
  // Default query object *can* be copied and assigned.
  unique_ptr<QueryObject<double>> default_object =
      QOT::MakeQueryObject<double>();
  QOT::expect_default(*default_object);

  QueryObject<double> from_default{*default_object};
  QOT::expect_default(from_default);

  QueryObject<double> from_live{*query_object_};
  QOT::expect_default(from_live);
}

TEST_F(QueryObjectTest, DefaultQueryThrows) {
  unique_ptr<QueryObject<double>> default_object =
      QOT::MakeQueryObject<double>();
  QOT::expect_default(*default_object);

#define EXPECT_DEFAULT_ERROR(expression) \
  EXPECT_ERROR_MESSAGE(expression, std::runtime_error, \
      "Attempting to perform query on invalid QueryObject.+");

  EXPECT_DEFAULT_ERROR(QOT::ThrowIfDefault(*default_object));

  // Enumerate *all* queries to confirm they throw the proper exception.
  EXPECT_DEFAULT_ERROR(default_object->GetFrameId(GeometryId::get_new_id()));
  EXPECT_DEFAULT_ERROR(default_object->GetSourceName(SourceId::get_new_id()));

#undef EXPECT_DEFAULT_ERROR
}

// NOTE: This doesn't test the specific queries; GeometryQuery simply wraps
// the class (GeometrySystem) that actually *performs* those queries. The
// correctness of those queries is handled in geometry_state_test.cc. The
// wrapper merely confirms that the state is correct and that wrapper
// functionality is tested in DefaultQueryThrows.

}  // namespace
}  // namespace geometry
}  // namespace drake
