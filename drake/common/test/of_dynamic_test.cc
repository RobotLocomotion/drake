#include "drake/common/test/of_dynamic.h"

#include <memory>

#include <gtest/gtest.h>

namespace drake {
namespace {
class Car {
 public:
  virtual ~Car() {}
};
class Prius : public Car {
 public:
  virtual ~Prius() {}
};
class Camry : public Car {
 public:
  virtual ~Camry() {}
};

using std::make_shared;
using std::shared_ptr;

GTEST_TEST(OfDynamicTest, RawPointer) {
  const Car* car = new Car{};
  const Car* prius = new Prius{};
  const Car* camry = new Camry{};
  EXPECT_FALSE(of_dynamic<Prius>(car));
  EXPECT_FALSE(of_dynamic<Camry>(car));
  EXPECT_TRUE(of_dynamic<Prius>(prius));
  EXPECT_FALSE(of_dynamic<Camry>(prius));
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
  EXPECT_FALSE(of_dynamic<Prius>(car));
  EXPECT_FALSE(of_dynamic<Camry>(car));
  EXPECT_TRUE(of_dynamic<Prius>(prius));
  EXPECT_FALSE(of_dynamic<Camry>(prius));
  EXPECT_FALSE(of_dynamic<Prius>(camry));
  EXPECT_TRUE(of_dynamic<Camry>(camry));
}
}  // namespace
}  // namespace drake
