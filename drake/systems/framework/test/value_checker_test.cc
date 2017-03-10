#include "drake/systems/framework/value_checker.h"

#include <memory>
#include <stdexcept>

#include "gtest/gtest.h"

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/value.h"

using std::make_unique;

namespace drake {
namespace systems {
namespace {

// This is what DoClone should look like.
class GoodVector : public BasicVector<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GoodVector)
  GoodVector() : BasicVector<double>(2) {}
  GoodVector* DoClone() const override {
    return new GoodVector;
  }
};

// This one forgot the DoClone override entirely.
class MissingCloneVector : public BasicVector<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MissingCloneVector)
  MissingCloneVector() : BasicVector<double>(2) {}
};

// This one forgot to override DoClone again in a subclass; relying on the
// superclass implementation is not good enough.
class NotSoGoodVector : public GoodVector {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(NotSoGoodVector)
  NotSoGoodVector() {}
};

// This one remembers to DoClone in the subclass, but does it wrong.
class NotQuiteGoodVector : public GoodVector {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(NotQuiteGoodVector)
  NotQuiteGoodVector() {}
  GoodVector* DoClone() const override {
    return new GoodVector;
  }
};

// A convenience wrapper for the "device under test".
void CheckVector(const BasicVector<double>* basic_vector) {
  drake::systems::detail::CheckBasicVectorInvariants(basic_vector);
}

GTEST_TEST(ValueCheckerTest, CheckBasicVectorInvariantsTest) {
  const BasicVector<double> basic(2);
  const GoodVector good;
  const MissingCloneVector missing_clone;
  const NotSoGoodVector not_so_good;
  const NotQuiteGoodVector not_quite_good;

  EXPECT_NO_THROW(CheckVector(&basic));
  EXPECT_NO_THROW(CheckVector(&good));
  EXPECT_THROW(CheckVector(nullptr), std::exception);
  EXPECT_THROW(CheckVector(&missing_clone), std::exception);
  EXPECT_THROW(CheckVector(&not_so_good), std::exception);
  EXPECT_THROW(CheckVector(&not_quite_good), std::exception);
}

// A convenience wrapper for the "device under test".
void CheckValue(const AbstractValue* abstract_value) {
  drake::systems::detail::CheckVectorValueInvariants<double>(abstract_value);
}

GTEST_TEST(ValueCheckerTest, CheckVectorValueInvariantsTest) {
  const VectorValue<double> basic(BasicVector<double>::Make({1, 2}));
  const VectorValue<double> good(make_unique<GoodVector>());
  const VectorValue<double> missing_clone(make_unique<MissingCloneVector>());
  const VectorValue<double> not_so_good(make_unique<NotSoGoodVector>());
  const VectorValue<double> not_quite_good(make_unique<NotQuiteGoodVector>());

  EXPECT_NO_THROW(CheckValue(&basic));
  EXPECT_NO_THROW(CheckValue(&good));
  EXPECT_THROW(CheckValue(nullptr), std::exception);
  EXPECT_THROW(CheckValue(&missing_clone), std::exception);
  EXPECT_THROW(CheckValue(&not_so_good), std::exception);
  EXPECT_THROW(CheckValue(&not_quite_good), std::exception);

  // Other unrelated Values are ignored.
  const Value<int> int_value(2);
  EXPECT_NO_THROW(CheckValue(&int_value));
}

}  // namespace
}  // namespace systems
}  // namespace drake
