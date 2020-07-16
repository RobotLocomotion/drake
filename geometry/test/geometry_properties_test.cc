#include "drake/geometry/geometry_properties.h"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/drake_copyable.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/unused.h"
#include "drake/geometry/rgba.h"

namespace drake {
namespace geometry {
namespace {

using Eigen::Vector4d;
using std::string;

// A constructible sub-class of GeometryProperties.
class TestProperties : public GeometryProperties {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(TestProperties);

  TestProperties() = default;
};

// Tests adding properties (successfully and otherwise). Uses a call to
// Get() to confirm successful add.
GTEST_TEST(GeometryProperties, AddProperty) {
  TestProperties properties;

  // Confirm property doesn't exist.
  const string prop_name("some_property");
  ASSERT_FALSE(properties.HasProperty(prop_name));

  // Add the property.
  const int int_value{7};
  DRAKE_EXPECT_NO_THROW(properties.Add(prop_name, int_value));

  // Confirm existence.
  ASSERT_TRUE(properties.HasProperty(prop_name));
  ASSERT_EQ(properties.Get<int>(prop_name), int_value);

  // Redundant add fails.
  DRAKE_EXPECT_THROWS_MESSAGE(
      properties.Add(prop_name, int_value), std::logic_error,
      fmt::format(".*Trying to add property '{}'.+ name already exists",
                  prop_name));
  ASSERT_TRUE(properties.HasProperty(prop_name));
}

// Tests updating properties (successfully and otherwise). Uses a call to
// Get() to confirm successful add.
GTEST_TEST(GeometryProperties, UpdateProperty) {
  TestProperties properties;

  // Initialize with a single property.
  const string prop_name("some_property");
  const int int_value{7};
  DRAKE_EXPECT_NO_THROW(properties.Add(prop_name, int_value));
  EXPECT_EQ(properties.Get<int>(prop_name), int_value);

  // Update adds new property.
  const string& prop_name2("other_property");
  EXPECT_FALSE(properties.HasProperty(prop_name2));
  EXPECT_NO_THROW(properties.Update(prop_name2, "from_update"));
  EXPECT_EQ(properties.Get<string>(prop_name2), "from_update");

  // Update alias works for changing value (with same type).
  EXPECT_NO_THROW(properties.Update(prop_name, int_value + 10));
  EXPECT_EQ(properties.Get<int>(prop_name), int_value + 10);

  // Update alias fails for changing type.
  DRAKE_EXPECT_THROWS_MESSAGE(
      properties.Update(prop_name, 17.0), std::logic_error,
      fmt::format(".*Trying to update property '{}'.+ is of different type.+",
                  prop_name));
}

GTEST_TEST(GeometryProperties, RemoveProperty) {
  TestProperties properties;
  const string prop1{"prop1"};
  const string prop2{"prop2"};

  // Simply add a couple of properties.
  properties.Add(prop1, 1);
  properties.Add(prop2, 2);
  ASSERT_TRUE(properties.HasProperty(prop1));
  ASSERT_TRUE(properties.HasProperty(prop2));

  // Remove property, make sure all others are still intact.
  EXPECT_TRUE(properties.Remove(prop1));
  ASSERT_FALSE(properties.HasProperty(prop1));
  ASSERT_TRUE(properties.HasProperty(prop2));

  // Trying to remove it again has no effect.
  EXPECT_FALSE(properties.Remove(prop1));
  ASSERT_FALSE(properties.HasProperty(prop1));
  ASSERT_TRUE(properties.HasProperty(prop2));
}

// Struct for the AddPropertyStruct test.
struct TestData {
  int i{};
  double d{};
  string s;
};

// Tests the case where the property value is a struct.
GTEST_TEST(GeometryProperties, AddPropertyStruct) {
  TestProperties properties;

  const string prop_name("test data");
  TestData data{1, 2., "3"};
  DRAKE_EXPECT_NO_THROW(properties.Add(prop_name, data));

  const TestData& read = properties.Get<TestData>(prop_name);
  EXPECT_EQ(data.i, read.i);
  EXPECT_EQ(data.d, read.d);
  EXPECT_EQ(data.s, read.s);
}

// Tests property access with default.
GTEST_TEST(GeometryProperties, GetPropertyOrDefault) {
  // Create one group with a single property.
  TestProperties properties;
  const double double_value{7};
  const double default_value = double_value - 1;
  const string prop_name("some_property");
  DRAKE_EXPECT_NO_THROW(properties.Add(prop_name, double_value));

  // Case: default value that can be *implicitly* converted to the desired
  // type requires explicit template declaration.
  DRAKE_EXPECT_THROWS_MESSAGE(
      properties.GetPropertyOrDefault(prop_name, 3), std::logic_error,
      fmt::format(".*The property '{}' exists, but is of a different type. "
                  "Requested 'int', but found 'double'",
                  prop_name));
  DRAKE_EXPECT_NO_THROW(properties.GetPropertyOrDefault<double>(prop_name, 3));

  // Case: read an existing property.
  int read_value = properties.GetPropertyOrDefault(prop_name, default_value);
  EXPECT_EQ(double_value, read_value);

  // Case: read invalid property.
  read_value = properties.GetPropertyOrDefault("invalid_prop", default_value);
  EXPECT_EQ(default_value, read_value);

  // Case: Property exists of different type.
  DRAKE_EXPECT_THROWS_MESSAGE(
      properties.GetPropertyOrDefault(prop_name, "test"), std::logic_error,
      fmt::format(".*The property '{}' exists, but is of a different type. "
                  "Requested 'std::string', but found 'double'",
                  prop_name));

  // Using r-values as defaults; this tests both compatibility and correctness.
  properties.Add("valid_string", "valid_string");
  const string valid_value =
      properties.GetPropertyOrDefault("valid_string", "missing");
  EXPECT_EQ("valid_string", valid_value);
  const string default_value_return =
      properties.GetPropertyOrDefault("invalid_string", "rvalue_string");
  EXPECT_EQ("rvalue_string", default_value_return);
}

// Tests the unsuccessful access to properties (successful access has been
// implicitly tested in the functions that added/set properties).
GTEST_TEST(GeometryProperties, GetPropertyFailure) {
  TestProperties properties;
  const string prop_name("some_property");

  // Getter errors
  // Case: Asking for non-existent property.
  DRAKE_EXPECT_THROWS_MESSAGE(
      properties.Get<int>(prop_name), std::logic_error,
      fmt::format(".*There is no property '{}'.", prop_name));

  // Case: Property exists, but property is of different type.
  DRAKE_EXPECT_NO_THROW(properties.Add(prop_name, 7.0));
  DRAKE_EXPECT_THROWS_MESSAGE(
      properties.Get<int>(prop_name), std::logic_error,
      fmt::format(".*The property '{}' exists, but is of a different type. "
                  "Requested 'int', but found 'double'",
                  prop_name));
}

// Tests iteration through a group's properties.
GTEST_TEST(GeometryProperties, PropertyIteration) {
  TestProperties properties;
  std::unordered_map<string, int> reference{{"prop1", 10}, {"prop2", 20}};
  for (const auto& [name, value] : reference) {
    properties.Add(name, value);
  }

  std::set<string> visited_properties;
  for (const auto& [name, abstract_value] : properties.GetAllProperties()) {
    EXPECT_GT(reference.count(name), 0);
    EXPECT_EQ(reference[name], abstract_value->get_value<int>());
    visited_properties.insert(name);
  }
  EXPECT_EQ(reference.size(), visited_properties.size());
}

// Confirms that derived classes *can* be copied/moved.
GTEST_TEST(GeometryProperties, CopyMoveSemantics) {
  // Populate a property set with an arbitrary set of properties. In this case,
  // they are all int-valued to facilitate comparison between property sets.
  auto make_properties = []() {
    TestProperties props;
    props.Add("prop1", 1);
    props.Add("prop2", 2);
    return props;
  };

  // Only works for int-valued properties.
  auto properties_equal =
      [](const TestProperties& reference,
         const TestProperties& test) -> ::testing::AssertionResult {
    if (reference.GetAllProperties().size() != test.GetAllProperties().size()) {
      return ::testing::AssertionFailure()
             << "Different number of properties.\n  Expected "
             << reference.GetAllProperties().size() << "\n  Found "
             << test.GetAllProperties().size();
    }
    for (const auto& [name, abstract_value] : reference.GetAllProperties()) {
      if (!test.HasProperty(name)) {
        return ::testing::AssertionFailure()
               << "Missing expected property: " << name;
      }
      const int expected_value = abstract_value->get_value<int>();
      const int test_value = test.Get<int>(name);
      if (expected_value != test_value) {
        return ::testing::AssertionFailure()
               << "Mismatch on property '" << name << "'.\n  Expected "
               << expected_value << "\n  Found " << test_value;
      }
    }
    // Note: We're not explicitly testing to see what properties may exist in
    // test that *aren't* in reference. There's no point; we know they're the
    // same size. If test has properties absent from reference, that means
    // reference has properties absent in test (which are caught above).
    return ::testing::AssertionSuccess();
  };

  TestProperties source = make_properties();
  TestProperties reference = make_properties();

  // Copy construction.
  TestProperties copy_construct(source);
  EXPECT_TRUE(properties_equal(reference, copy_construct));

  // Copy assignment.
  TestProperties copy_assign;
  EXPECT_FALSE(properties_equal(reference, copy_assign));
  copy_assign = source;
  EXPECT_TRUE(properties_equal(reference, copy_assign));

  // Strictly speaking, confirming that the move *source* has changed isn't
  // necessary. The move semantics aren't documented. However, given that this
  // is using default move semantics on unordered_map, we can assume that the
  // source is modified by the move. So, we'll go ahead and test that.

  // Move construction.
  TestProperties move_construct(std::move(source));
  EXPECT_FALSE(properties_equal(reference, source));
  EXPECT_TRUE(properties_equal(reference, move_construct));

  // Move assignment.
  TestProperties move_assign;
  EXPECT_FALSE(properties_equal(reference, move_assign));
  move_assign = std::move(move_construct);
  EXPECT_FALSE(properties_equal(reference, move_construct));
  EXPECT_TRUE(properties_equal(reference, move_assign));
}

// Counts the number of instances constructed. Ignores destruction.
// TODO(eric.cousineau): Hoist this to more general testing code (e.g. for
// value_test.cc).
class GloballyCounted {
 public:
  struct Stats {
    int num_copies{};
    int num_moves{};

    ::testing::AssertionResult Equal(Stats other) {
      if (num_copies != other.num_copies || num_moves != other.num_moves) {
        return ::testing::AssertionFailure() << fmt::format(
                   "(num_copies, num_moves): ({}, {}) != ({}, {})", num_copies,
                   num_moves, other.num_copies, other.num_moves);
      }
      return ::testing::AssertionSuccess();
    }
  };

  static Stats get_stats_and_reset() {
    Stats out = stats;
    stats = {0, 0};
    return out;
  }

  GloballyCounted() {}

  GloballyCounted(GloballyCounted&&) { stats.num_moves++; }
  GloballyCounted& operator=(GloballyCounted&&) {
    stats.num_moves++;
    return *this;
  }

  GloballyCounted(const GloballyCounted&) { stats.num_copies++; }
  GloballyCounted& operator=(const GloballyCounted&) {
    stats.num_copies++;
    return *this;
  }

 private:
  static Stats stats;
};

GloballyCounted::Stats GloballyCounted::stats;

GTEST_TEST(GeometryProperties, GloballyCounted) {
  // Unittest basic utility.
  const GloballyCounted value;
  EXPECT_TRUE(GloballyCounted::get_stats_and_reset().Equal({0, 0}));

  // Copy construction.
  {
    const GloballyCounted copy = value;
    unused(copy);
    EXPECT_TRUE(GloballyCounted::get_stats_and_reset().Equal({1, 0}));
  }

  // Copy assignment.
  {
    GloballyCounted copy;
    copy = value;
    EXPECT_TRUE(GloballyCounted::get_stats_and_reset().Equal({1, 0}));
  }

  // Move construction.
  {
    GloballyCounted moved_from;
    GloballyCounted moved_to = std::move(moved_from);
    unused(moved_to);
    EXPECT_TRUE(GloballyCounted::get_stats_and_reset().Equal({0, 1}));
  }

  // Move assigment.
  {
    GloballyCounted moved_from;
    GloballyCounted moved_to;
    moved_to = std::move(moved_from);
    EXPECT_TRUE(GloballyCounted::get_stats_and_reset().Equal({0, 1}));
  }
}

// Confirms the amount of copying that occurs.
GTEST_TEST(GeometryProperties, CopyCountCheck) {
  TestProperties properties;
  const string name_1("name_1");
  const string name_2("name_2");

  // When adding a property, 2 copies should occur: once when constructing a
  // value, then another when cloning it.
  const GloballyCounted value;
  properties.AddAbstract(name_1, Value(value));
  EXPECT_TRUE(GloballyCounted::get_stats_and_reset().Equal({2, 0}));

  // Same as above.
  properties.Add(name_2, value);
  EXPECT_TRUE(GloballyCounted::get_stats_and_reset().Equal({2, 0}));

  // No copies upon retrieving the type.
  properties.Get<GloballyCounted>(name_1);
  EXPECT_TRUE(GloballyCounted::get_stats_and_reset().Equal({0, 0}));
}

GTEST_TEST(GeometryProperties, RgbaAndVector4) {
  const Rgba color(0.75, 0.5, 0.25, 1.);
  const Vector4d vector(0.75, 0.5, 0.25, 1.);

  TestProperties properties;
  const string color_name("color_name");
  const string fake_name("fake_name");

  // Adding an Rgba should be accessible as both Rgba and Vector4d.
  properties.Add(color_name, color);
  EXPECT_EQ(color, properties.Get<Rgba>(color_name));
  EXPECT_EQ(vector, properties.Get<Vector4d>(color_name));
  EXPECT_EQ(vector,
            properties.GetPropertyOrDefault<Vector4d>(fake_name, vector));

  // Adding a Vector4d should be accessible as both Rgba and Vector4d.
  const string vector_name("vector_name");
  properties.Add(vector_name, vector);
  EXPECT_EQ(color, properties.Get<Rgba>(vector_name));
  EXPECT_EQ(vector, properties.Get<Vector4d>(vector_name));
}

// Confirms that properties can be written in a chained sequence.
GTEST_TEST(GeometryProperties, ChainingWrites) {
  TestProperties props;
  props.Add("p0", 0)
      .Update("p1", 1)
      .AddAbstract("p2", Value(2))
      .UpdateAbstract("p3", Value(3));
  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(props.Get<int>(fmt::format("p{}", i)), i);
  }
}

// TODO(SeanCurtis-TRI) The tests below relate to the old group-based API. Some
//  of the tests are unique to group-specific behavior. Some are simply repeats
//  of the previous tests while exercising the group, property variants. When
//  that API is removed, remove all tests from the test fixture:
//  GeometryPropertiesOld.

GTEST_TEST(GeometryPropertiesOld, ManagingGroups) {
  TestProperties properties;
  const string& group_name{"some_group"};
  // Only contains the default group.
  ASSERT_EQ(1, properties.num_groups());
  ASSERT_FALSE(properties.HasGroup(group_name));
  ASSERT_TRUE(properties.HasGroup(TestProperties::default_group_name()));

  // Add the group for the first time by adding a property.
  properties.AddProperty(group_name, "junk_value", 1);
  ASSERT_TRUE(properties.HasGroup(group_name));
  ASSERT_EQ(2, properties.num_groups());

  // Retrieve the group.
  using PropertyGroup = GeometryProperties::Group;
  const PropertyGroup& group = properties.GetPropertiesInGroup(group_name);
  EXPECT_EQ(1u, group.size());

  EXPECT_EQ(properties.GetPropertiesInGroup("invalid_name").size(), 0);
}

// Tests adding properties (successfully and otherwise). Uses a call to
// GetProperty() to confirm successful add.
GTEST_TEST(GeometryPropertiesOld, AddProperty) {
  TestProperties properties;
  const string& group_name{"some_group"};

  // Confirm property doesn't exist.
  const string prop_name("some_property");
  ASSERT_FALSE(properties.HasProperty(group_name, prop_name));

  // Add the property.
  const int int_value{7};
  DRAKE_EXPECT_NO_THROW(
      properties.AddProperty(group_name, prop_name, int_value));

  // Confirm existence.
  ASSERT_TRUE(properties.HasProperty(group_name, prop_name));
  ASSERT_EQ(properties.GetProperty<int>(group_name, prop_name), int_value);

  // Redundant add fails.
  DRAKE_EXPECT_THROWS_MESSAGE(
      properties.AddProperty(group_name, prop_name, int_value),
      std::logic_error,
      fmt::format(".*Trying to add property '{}/{}'.+ name already exists",
                  group_name, prop_name));
  ASSERT_TRUE(properties.HasProperty(group_name, prop_name));
}

// Tests updating properties (successfully and otherwise). Uses a call to
// GetProperty() to confirm successful add.
GTEST_TEST(GeometryPropertiesOld, UpdateProperty) {
  TestProperties properties;

  // Initialize with a single property.
  const string group_name{"some_group"};
  const string prop_name("some_property");
  const int int_value{7};
  DRAKE_EXPECT_NO_THROW(
      properties.AddProperty(group_name, prop_name, int_value));
  EXPECT_EQ(properties.GetProperty<int>(group_name, prop_name), int_value);

  // UpdateProperty adds new property.
  const string& prop_name2("other_property");
  EXPECT_FALSE(properties.HasProperty(group_name, prop_name2));
  EXPECT_NO_THROW(
      properties.UpdateProperty(group_name, prop_name2, "from_update"));
  EXPECT_EQ(properties.GetProperty<string>(group_name, prop_name2),
            "from_update");

  // UpdateProperty alias works for changing value (with same type).
  EXPECT_NO_THROW(
      properties.UpdateProperty(group_name, prop_name, int_value + 10));
  EXPECT_EQ(properties.GetProperty<int>(group_name, prop_name), int_value + 10);

  // UpdateProperty alias fails for changing type.
  DRAKE_EXPECT_THROWS_MESSAGE(
      properties.UpdateProperty(group_name, prop_name, 17.0), std::logic_error,
      fmt::format(
          ".*Trying to update property '{}/{}'.+ is of different type.+",
          group_name, prop_name));
}

GTEST_TEST(GeometryPropertiesOld, RemoveProperty) {
  TestProperties properties;
  const string group1{"group1"};
  const string group2{"group2"};
  const string prop1{"prop1"};
  const string prop2{"prop2"};

  // Add two groups with two properties each.
  properties.AddProperty(group1, prop1, 1);
  properties.AddProperty(group1, prop2, 2);
  properties.AddProperty(group2, prop1, 3);
  properties.AddProperty(group2, prop2, 4);
  ASSERT_TRUE(properties.HasProperty(group1, prop1));
  ASSERT_TRUE(properties.HasProperty(group1, prop2));
  ASSERT_TRUE(properties.HasProperty(group2, prop1));
  ASSERT_TRUE(properties.HasProperty(group2, prop2));

  // Remove property, make sure all others are still intact.
  EXPECT_TRUE(properties.RemoveProperty(group1, prop1));
  ASSERT_FALSE(properties.HasProperty(group1, prop1));
  ASSERT_TRUE(properties.HasProperty(group1, prop2));
  ASSERT_TRUE(properties.HasProperty(group2, prop1));
  ASSERT_TRUE(properties.HasProperty(group2, prop2));

  // Trying to remove it again has no effect.
  EXPECT_FALSE(properties.RemoveProperty(group1, prop1));
  EXPECT_FALSE(properties.HasProperty(group1, prop1));
  EXPECT_TRUE(properties.HasProperty(group1, prop2));
  EXPECT_TRUE(properties.HasProperty(group2, prop1));
  EXPECT_TRUE(properties.HasProperty(group2, prop2));
}

// Tests the case where the property value is a struct.
GTEST_TEST(GeometryPropertiesOld, AddPropertyStruct) {
  TestProperties properties;

  const string prop_name("test data");
  TestData data{1, 2., "3"};
  DRAKE_EXPECT_NO_THROW(properties.AddProperty(
      TestProperties::default_group_name(), prop_name, data));

  const TestData& read = properties.GetProperty<TestData>(
      TestProperties::default_group_name(), prop_name);
  EXPECT_EQ(data.i, read.i);
  EXPECT_EQ(data.d, read.d);
  EXPECT_EQ(data.s, read.s);
}

// Tests property access with default.
GTEST_TEST(GeometryPropertiesOld, GetPropertyOrDefault) {
  // Create one group with a single property.
  TestProperties properties;
  const string group_name{"some_group"};
  const double double_value{7};
  const double default_value = double_value - 1;
  const string prop_name("some_property");
  DRAKE_EXPECT_NO_THROW(
      properties.AddProperty(group_name, prop_name, double_value));

  // Case: default value that can be *implicitly* converted to the desired
  // type requires explicit template declaration.
  DRAKE_EXPECT_THROWS_MESSAGE(
      properties.GetPropertyOrDefault(group_name, prop_name, 3),
      std::logic_error,
      fmt::format(".*The property '{}/{}' exists, but is of a different type. "
                  "Requested 'int', but found 'double'",
                  group_name, prop_name));
  DRAKE_EXPECT_NO_THROW(
      properties.GetPropertyOrDefault<double>(group_name, prop_name, 3));

  // Case: read an existing property.
  int read_value =
      properties.GetPropertyOrDefault(group_name, prop_name, default_value);
  EXPECT_EQ(double_value, read_value);

  // Case: read from valid group, but invalid property.
  read_value = properties.GetPropertyOrDefault(group_name, "invalid_prop",
                                               default_value);
  EXPECT_EQ(default_value, read_value);

  // Case: read from invalid group.
  read_value = properties.GetPropertyOrDefault("invalid_group", "invalid_prop",
                                               default_value);
  EXPECT_EQ(default_value, read_value);

  // Case: Property exists of different type.
  DRAKE_EXPECT_THROWS_MESSAGE(
      properties.GetPropertyOrDefault(group_name, prop_name, "test"),
      std::logic_error,
      fmt::format(".*The property '{}/{}' exists, but is of a different type. "
                  "Requested 'std::string', but found 'double'",
                  group_name, prop_name));

  // Using r-values as defaults; this tests both compatibility and correctness.
  properties.AddProperty("strings", "valid_string", "valid_string");
  string valid_value =
      properties.GetPropertyOrDefault("strings", "valid_string", "missing");
  EXPECT_EQ("valid_string", valid_value);
  string default_value_return = properties.GetPropertyOrDefault(
      "strings", "invalid_string", "rvalue_string");
  EXPECT_EQ("rvalue_string", default_value_return);
}

// Tests the unsuccessful access to properties (successful access has been
// implicitly tested in the functions that added/set properties).
GTEST_TEST(GeometryPropertiesOld, GetPropertyFailure) {
  TestProperties properties;
  const string& group_name{"some_group"};
  const string prop_name("some_property");

  // Getter errors

  // Case: Group exists, property does not.
  properties.AddProperty(group_name, prop_name + "_alt", 1);
  DRAKE_EXPECT_THROWS_MESSAGE(
      properties.GetProperty<int>(group_name, prop_name), std::logic_error,
      fmt::format(".*There is no property '{}/{}'.", group_name, prop_name));

  // Case: Group and property exists, but property is of different type.
  DRAKE_EXPECT_NO_THROW(properties.AddProperty(group_name, prop_name, 7.0));
  DRAKE_EXPECT_THROWS_MESSAGE(
      properties.GetProperty<int>(group_name, prop_name), std::logic_error,
      fmt::format(".*The property '{}/{}' exists, but is of a different type. "
                  "Requested 'int', but found 'double'",
                  group_name, prop_name));
}

// Tests iteration through a group's properties.
GTEST_TEST(GeometryPropertiesOld, PropertyIteration) {
  TestProperties properties;
  const string& default_group = TestProperties::default_group_name();
  std::unordered_map<string, int> reference{{"prop1", 10}, {"prop2", 20}};
  for (const auto& pair : reference) {
    properties.AddProperty(default_group, pair.first, pair.second);
  }

  // Confirm that all properties have the right value and get visited.
  std::set<string> visited_properties;
  for (const auto& pair : properties.GetPropertiesInGroup(default_group)) {
    const string& name = pair.first;
    EXPECT_GT(reference.count(name), 0);
    EXPECT_EQ(reference[name],
              properties.GetProperty<int>(default_group, name));
    visited_properties.insert(name);
  }
  EXPECT_EQ(reference.size(), visited_properties.size());
}

// Confirms that derived classes *can* be copied/moved.
GTEST_TEST(GeometryPropertiesOld, CopyMoveSemantics) {
  // Populate a property set with an arbitrary set of properties. In this case,
  // they are all int-valued to facilitate comparison between property sets.
  auto make_properties = []() {
    TestProperties props;
    const string& default_group = TestProperties::default_group_name();
    props.AddProperty(default_group, "prop1", 1);
    props.AddProperty(default_group, "prop2", 2);

    const string group1("group1");
    // NOTE: Duplicate property name differentiated by different group.
    props.AddProperty(group1, "prop1", 3);
    props.AddProperty(group1, "prop3", 4);
    props.AddProperty(group1, "prop4", 5);

    const string group2("group2");
    props.AddProperty(group2, "prop5", 6);
    return props;
  };

  // Only works for int-valued properties.
  auto properties_equal =
      [](const TestProperties& reference,
         const TestProperties& test) -> ::testing::AssertionResult {
    if (reference.num_groups() != test.num_groups()) {
      return ::testing::AssertionFailure()
             << "Different number of groups. Expected "
             << reference.num_groups() << " found " << test.num_groups();
    }

    for (const auto& group_name : reference.GetGroupNames()) {
      if (test.HasGroup(group_name)) {
        for (const auto& pair : reference.GetPropertiesInGroup(group_name)) {
          const string& name = pair.first;
          int expected_value = pair.second->get_value<int>();
          if (test.HasProperty(group_name, name)) {
            int test_value = test.GetProperty<int>(group_name, name);
            if (expected_value != test_value) {
              return ::testing::AssertionFailure()
                     << "Expected value for '" << group_name << "':'" << name
                     << "' to be " << expected_value << ". Found "
                     << test_value;
            }
          } else {
            return ::testing::AssertionFailure()
                   << "Expected group '" << group_name << "' to have property '"
                   << name << "'. It does not exist.";
          }
        }
      } else {
        return ::testing::AssertionFailure()
               << "Expected group '" << group_name
               << "' is missing from test properties";
      }
    }
    return ::testing::AssertionSuccess();
  };

  TestProperties source = make_properties();
  TestProperties reference = make_properties();

  // Copy construction.
  TestProperties copy_construct(source);
  EXPECT_TRUE(properties_equal(reference, copy_construct));

  // Copy assignment.
  TestProperties copy_assign;
  EXPECT_FALSE(properties_equal(reference, copy_assign));
  copy_assign = source;
  EXPECT_TRUE(properties_equal(reference, copy_assign));

  // Strictly speaking, confirming that the move *source* has changed isn't
  // necessary. The move semantics aren't documented. However, given that this
  // is using default move semantics on unordered_map, we can assume that the
  // source is modified by the move. So, we'll go ahead and test that.

  // Move construction.
  TestProperties move_construct(std::move(source));
  EXPECT_FALSE(properties_equal(reference, source));
  EXPECT_TRUE(properties_equal(reference, move_construct));

  // Move assignment.
  TestProperties move_assign;
  EXPECT_FALSE(properties_equal(reference, move_assign));
  move_assign = std::move(move_construct);
  EXPECT_FALSE(properties_equal(reference, move_construct));
  EXPECT_TRUE(properties_equal(reference, move_assign));
}

// Confirms the amount of copying that occurs.
GTEST_TEST(GeometryPropertiesOld, CopyCountCheck) {
  TestProperties properties;
  const string& group_name{"some_group"};
  const string name_1("name_1");
  const string name_2("name_2");

  // When adding a property, 2 copies should occur: once when constructing a
  // value, then another when cloning it.
  const GloballyCounted value;
  properties.AddPropertyAbstract(group_name, name_1, Value(value));
  EXPECT_TRUE(GloballyCounted::get_stats_and_reset().Equal({2, 0}));

  // Same as above.
  properties.AddProperty(group_name, name_2, value);
  EXPECT_TRUE(GloballyCounted::get_stats_and_reset().Equal({2, 0}));

  // No copies upon retrieving the type.
  properties.GetProperty<GloballyCounted>(group_name, name_1);
  EXPECT_TRUE(GloballyCounted::get_stats_and_reset().Equal({0, 0}));
}

GTEST_TEST(GeometryPropertiesOld, RgbaAndVector4) {
  const Rgba color(0.75, 0.5, 0.25, 1.);
  const Vector4d vector(0.75, 0.5, 0.25, 1.);

  TestProperties properties;
  const string& group_name{"some_group"};
  const string color_name("color_name");
  const string fake_name("fake_name");

  // Add<Rgba>.
  properties.AddProperty(group_name, color_name, color);
  // - Get<Rgba>.
  EXPECT_EQ(color, properties.GetProperty<Rgba>(group_name, color_name));
  // - Get<Vector4d>.
  EXPECT_EQ(vector, properties.GetProperty<Vector4d>(group_name, color_name));
  EXPECT_EQ(vector, properties.GetPropertyOrDefault<Vector4d>(
                        group_name, fake_name, vector));

  // Add<Vector4d>.
  const string vector_name("vector_name");
  properties.AddProperty(group_name, vector_name, vector);
  // - Get<Rgba>.
  EXPECT_EQ(color, properties.GetProperty<Rgba>(group_name, vector_name));
  // - Get<Vector4d>.
  EXPECT_EQ(vector, properties.GetProperty<Vector4d>(group_name, vector_name));
}

}  // namespace
}  // namespace geometry
}  // namespace drake
