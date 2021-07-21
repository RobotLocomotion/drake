#include "drake/common/never_destroyed.h"

#include <random>
#include <unordered_map>

#include <gtest/gtest.h>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace {

class Boom : public std::exception { };
struct DtorGoesBoom {
  ~DtorGoesBoom() noexcept(false) { throw Boom(); }
};

// Confirm that we see booms by default.
GTEST_TEST(NeverDestroyedTest, BoomTest) {
  try {
    { DtorGoesBoom foo; }
    GTEST_FAIL();
  } catch (const Boom&) {
    ASSERT_TRUE(true);
  }
}

// Confirm that our wrapper stops the booms.
GTEST_TEST(NeverDestroyedTest, NoBoomTest) {
  try {
    { never_destroyed<DtorGoesBoom> foo; }
    ASSERT_TRUE(true);
  } catch (const Boom& e) {
    GTEST_FAIL();
  }
}

// This is an example from the class overview API docs; we repeat it here to
// ensure it remains valid.
class Singleton {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Singleton)
  static Singleton& getInstance() {
    static never_destroyed<Singleton> instance;
    return instance.access();
  }
 private:
  friend never_destroyed<Singleton>;
  Singleton() = default;
};

GTEST_TEST(NeverDestroyedExampleTest, Singleton) {
  const Singleton* get1 = &Singleton::getInstance();
  const Singleton* get2 = &Singleton::getInstance();
  EXPECT_EQ(get1, get2);
}

// This is an example from the class overview API docs; we repeat it here to
// ensure it remains valid.
enum class Foo { kBar, kBaz };
Foo ParseFoo(const std::string& foo_string) {
  using Dict = std::unordered_map<std::string, Foo>;
  static const drake::never_destroyed<Dict> string_to_enum{
    std::initializer_list<Dict::value_type>{
      {"bar", Foo::kBar},
      {"baz", Foo::kBaz},
    }
  };
  return string_to_enum.access().at(foo_string);
}

GTEST_TEST(NeverDestroyedExampleTest, ParseFoo) {
  EXPECT_EQ(ParseFoo("bar"), Foo::kBar);
  EXPECT_EQ(ParseFoo("baz"), Foo::kBaz);
}

// This is an example from the class overview API docs; we repeat it here to
// ensure it remains valid.
const std::vector<double>& GetConstantMagicNumbers() {
  static const drake::never_destroyed<std::vector<double>> result{[]() {
    std::vector<double> prototype;
    std::mt19937 random_generator;
    for (int i = 0; i < 10; ++i) {
      double new_value = random_generator();
      prototype.push_back(new_value);
    }
    return prototype;
  }()};
  return result.access();
}

GTEST_TEST(NeverDestroyedExampleTest, GetConstantMagicNumbers) {
  const auto& numbers = GetConstantMagicNumbers();
  EXPECT_EQ(numbers.size(), 10);
}

}  // namespace
}  // namespace drake
