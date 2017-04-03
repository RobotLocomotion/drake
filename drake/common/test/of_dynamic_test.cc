#include "drake/common/test/of_dynamic.h"

#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/nice_type_name.h"

namespace drake {
namespace {
class Car {
 public:
  virtual ~Car() {}
};
class Prius : public Car {
 public:
  ~Prius() override {}
};
class Camry : public Car {
 public:
  ~Camry() override {}
};

using std::make_shared;
using std::shared_ptr;
using std::string;

GTEST_TEST(OfDynamicTest, RawPointer) {
  const Car* car = new Car{};
  const Car* prius = new Prius{};
  const Car* camry = new Camry{};

  EXPECT_TRUE(of_dynamic<Car>(car));
  EXPECT_FALSE(of_dynamic<Prius>(car));
  EXPECT_FALSE(of_dynamic<Camry>(car));

  EXPECT_TRUE(of_dynamic<Car>(prius));
  EXPECT_TRUE(of_dynamic<Prius>(prius));
  EXPECT_FALSE(of_dynamic<Camry>(prius));

  EXPECT_TRUE(of_dynamic<Car>(camry));
  EXPECT_FALSE(of_dynamic<Prius>(camry));
  EXPECT_TRUE(of_dynamic<Camry>(camry));

  delete camry;
  delete prius;
  delete car;
}

GTEST_TEST(OfDynamicTest, SharedPtr) {
  const shared_ptr<Car> car{make_shared<Car>()};
  const shared_ptr<Car> prius{make_shared<Prius>()};
  const shared_ptr<Car> camry{make_shared<Camry>()};

  EXPECT_TRUE(of_dynamic<Car>(car));
  EXPECT_FALSE(of_dynamic<Prius>(car));
  EXPECT_FALSE(of_dynamic<Camry>(car));

  EXPECT_TRUE(of_dynamic<Car>(prius));
  EXPECT_TRUE(of_dynamic<Prius>(prius));
  EXPECT_FALSE(of_dynamic<Camry>(prius));

  EXPECT_TRUE(of_dynamic<Car>(camry));
  EXPECT_FALSE(of_dynamic<Prius>(camry));
  EXPECT_TRUE(of_dynamic<Camry>(camry));
}

string ExpectedMessage(const string& derived, const string& base,
                       const string& dynamic) {
  return "of_dynamic<" + derived + ">(" + base +
         "* ptr) failed because ptr is of dynamic type " + dynamic + ".";
}

GTEST_TEST(OfDynamicTest, DiagnosticMessage) {
  const string car_typename{NiceTypeName::Get<Car>()};
  const string prius_typename{NiceTypeName::Get<Prius>()};
  const string camry_typename{NiceTypeName::Get<Camry>()};

  const shared_ptr<Car> prius{make_shared<Prius>()};
  const shared_ptr<Car> camry{make_shared<Camry>()};

  const auto assertion_result1 = of_dynamic<Camry>(prius);
  ASSERT_FALSE(assertion_result1);
  EXPECT_EQ(assertion_result1.failure_message(),
            ExpectedMessage(camry_typename, car_typename, prius_typename));

  const auto assertion_result2 = of_dynamic<Prius>(camry);
  ASSERT_FALSE(assertion_result2);
  EXPECT_EQ(assertion_result2.failure_message(),
            ExpectedMessage(prius_typename, car_typename, camry_typename));
}
}  // namespace
}  // namespace drake
