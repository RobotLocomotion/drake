#include "drake/geometry/dev/geometry_properties.h"

#include <gtest/gtest.h>

#include "drake/common/drake_copyable.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace geometry {
namespace dev {
namespace {

// A constructible sub-class of GeometryProperties.
class TestProperties : public GeometryProperties {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(TestProperties);

  TestProperties() = default;
};

GTEST_TEST(GeometryProperties, ManagingGroups) {
  TestProperties properties;
  const std::string& group_name{"some_group"};
  // Only contains the default group.
  ASSERT_EQ(1, properties.num_groups());
  ASSERT_FALSE(properties.has_group(group_name));
  ASSERT_TRUE(properties.has_group(properties.default_group_name()));

  // Add the group for the first time.
  properties.AddGroup(group_name);
  ASSERT_TRUE(properties.has_group(group_name));
  ASSERT_EQ(2, properties.num_groups());

  // Redundant add does not change the outcome.
  properties.AddGroup(group_name);
  ASSERT_TRUE(properties.has_group(group_name));
  ASSERT_EQ(2, properties.num_groups());

  // Retrieve the group.
  using PropertyGroup = GeometryProperties::PropertyGroup;
  const PropertyGroup& group = properties.get_group(group_name);
  EXPECT_EQ(0u, group.size());

  DRAKE_EXPECT_THROWS_MESSAGE(properties.get_group("invalid_name"),
                              std::logic_error,
                              "There is no property group with the name .*");
}

// Tests the successful execution of adding properties. Uses a call to
// GetProperty() to confirm successful add.
GTEST_TEST(GeometryProperties, AddProperty) {
  TestProperties properties;
  const std::string& group_name{"some_group"};
  properties.AddGroup(group_name);

  // Confirm property doesn't exist.
  const std::string prop_name("some_property");
  ASSERT_FALSE(properties.HasProperty(group_name, prop_name));

  // Add the property.
  const int int_value{7};
  ASSERT_TRUE(properties.AddProperty(group_name, prop_name, int_value));

  // Confirm existence.
  ASSERT_TRUE(properties.HasProperty(group_name, prop_name));
  int read_value = properties.GetProperty<int>(group_name, prop_name);
  ASSERT_EQ(int_value, read_value);

  // Redundant add.
  ASSERT_FALSE(properties.AddProperty(group_name, prop_name, int_value));
  ASSERT_TRUE(properties.HasProperty(group_name, prop_name));

  // Adding property to non-existant group.
  DRAKE_EXPECT_THROWS_MESSAGE(
      properties.AddProperty("invalid_group", prop_name, 7), std::logic_error,
      "Trying to add property .* to group .*. But the group does not exist.");
}

// Struct for the AddPropertyStruct test.
struct TestData {
  int i {};
  double d{};
  std::string s;
};

// Tests the case where the property value is a struct.
GTEST_TEST(GeometryProperties, AddPropertyStruct) {
  TestProperties properties;

  const std::string prop_name("test data");
  TestData data{1, 2., "3"};
  ASSERT_TRUE(properties.AddProperty(properties.default_group_name(), prop_name,
                                     data));

  const TestData& read = properties.GetProperty<TestData>(
      properties.default_group_name(), prop_name);
  EXPECT_EQ(data.i, read.i);
  EXPECT_EQ(data.d, read.d);
  EXPECT_EQ(data.s, read.s);
}

GTEST_TEST(GeometryProperties, SetProperty) {
  TestProperties properties;
  const std::string& group_name{"some_group"};
  properties.AddGroup(group_name);

  // Confirm property doesn't exist.
  const std::string prop_name("some_property");
  ASSERT_FALSE(properties.HasProperty(group_name, prop_name));

  // Add the property and confirm existence.
  const int int_value{7};
  properties.SetProperty(group_name, prop_name, int_value);
  ASSERT_NO_THROW(properties.SetProperty(group_name, prop_name, int_value));
  ASSERT_TRUE(properties.HasProperty(group_name, prop_name));
  int read_value = properties.GetProperty<int>(group_name, prop_name);
  ASSERT_EQ(int_value, read_value);

  // Overwrite the value and confirm existence.
  const int int_value_2{14};
  ASSERT_NO_THROW(properties.SetProperty(group_name, prop_name, int_value_2));
  ASSERT_TRUE(properties.HasProperty(group_name, prop_name));
  read_value = properties.GetProperty<int>(group_name, prop_name);
  ASSERT_EQ(int_value_2, read_value);

  // Overwrite the value with a *different* type and confirm existence.
  const double double_value{13.5};
  ASSERT_NO_THROW(properties.SetProperty(group_name, prop_name, double_value));
  ASSERT_TRUE(properties.HasProperty(group_name, prop_name));
  double read_double = properties.GetProperty<double>(group_name, prop_name);
  ASSERT_EQ(double_value, read_double);

  // Error condition -- setting property in invalid group.
  DRAKE_EXPECT_THROWS_MESSAGE(
      properties.SetProperty("bad", prop_name, ""),
      std::logic_error,
      "Trying to add property .* to group .* But the group does not exist.");
}

// Tests property access with default.
GTEST_TEST(GeometryProperties, GetPropertyOrDefault) {
  // Create one group with a single property.
  TestProperties properties;
  const std::string& group_name{"some_group"};
  properties.AddGroup(group_name);
  const int int_value{7};
  const int default_value = int_value - 1;
  const std::string prop_name("some_property");
  ASSERT_TRUE(properties.AddProperty(group_name, prop_name, int_value));

  // Case: read an existing property.
  int read_value = properties.GetPropertyOrDefault(group_name, prop_name,
                                                   default_value);
  EXPECT_EQ(int_value, read_value);

  // Case: read from valid group, but invalid property.
  read_value = properties.GetPropertyOrDefault(group_name, "invalid_prop",
                                               default_value);
  EXPECT_EQ(default_value, read_value);

  // Case: read from invalid group.
  read_value = properties.GetPropertyOrDefault("invalid_group", "invalid_prop",
                                               default_value);
  EXPECT_EQ(default_value, read_value);
}

// Tests the unsuccessful access to properties (successful access has been
// implicitly tested in the functions that added/set properties).
GTEST_TEST(GeometryProperties, GetPropertyFailure) {
  TestProperties properties;
  const std::string& group_name{"some_group"};
  const std::string prop_name("some_property");

  // Getter errors
  // Case: Asking for property from non-existant group.
  DRAKE_EXPECT_THROWS_MESSAGE(
      properties.GetProperty<int>(group_name, prop_name), std::logic_error,
      "Trying to read property .* from group .*. But the group does not "
      "exist.");

  // Case: Group exists, property does not.
  properties.AddGroup(group_name);
  DRAKE_EXPECT_THROWS_MESSAGE(
      properties.GetProperty<int>(group_name, prop_name), std::logic_error,
      "There is no property .* in group .*.");

  // Case: Group and property exists, but property is of different type.
  ASSERT_TRUE(properties.AddProperty(group_name, prop_name, 7.0));
  DRAKE_EXPECT_THROWS_MESSAGE(
      properties.GetProperty<int>(group_name, prop_name), std::logic_error,
      ".* 'int' failed .* actual type was 'double'.");
}

GTEST_TEST(GeometryProperties, PropertyCounts) {
  TestProperties properties;
  // Initial condition.
  ASSERT_EQ(0, properties.NumProperties());

  // Querying for non-existant group.
  DRAKE_EXPECT_THROWS_MESSAGE(
      properties.NumProperties("invalid group"), std::logic_error,
      "Cannot report number of properties for a non-existant group: .*");

  // Set up some groups.
  std::vector<std::string> group_names{"group1", "group2"};

  int added = 0;
  const int per_group = 3;
  for (const std::string& group_name : group_names) {
    properties.AddGroup(group_name);
    for (int i = 0; i < per_group; ++i) {
      ASSERT_TRUE(properties.AddProperty(group_name,
                                         fmt::format("prop{}", i),
                                         i));
      ++added;
    }
  }
  // The two added groups plus the default.
  ASSERT_EQ(3, properties.num_groups());

  for (const std::string& group_name : group_names) {
    ASSERT_EQ(per_group, properties.NumProperties(group_name))
                  << "For group " << group_name;
  }
  ASSERT_EQ(added, properties.NumProperties());
}

// Tests iteration through a group's properties.
GTEST_TEST(GeometryProperties, PropertyIteration) {
  TestProperties properties;
  const std::string& default_group = properties.default_group_name();
  std::unordered_map<std::string, int> reference{{"prop1", 10}, {"prop2", 20}};
  for (const auto& pair : reference) {
    properties.AddProperty(default_group, pair.first, pair.second);
  }

  // Get exception for non-existant group.
  DRAKE_EXPECT_THROWS_MESSAGE(
      properties.GetGroupProperties("bad group"), std::logic_error,
      "Can't retrieve properties for a group that doesn't exist.*");

  // Confirm that all properties have the right value and get visited.
  std::set<std::string> visited_properties;
  for (const auto& pair : properties.GetGroupProperties(default_group)) {
    const std::string& name = pair.first;
    EXPECT_GT(reference.count(name), 0);
    EXPECT_EQ(reference[name],
              properties.GetProperty<int>(default_group, name));
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
    const std::string& default_group = props.default_group_name();
    props.AddProperty(default_group, "prop1", 1);
    props.AddProperty(default_group, "prop2", 2);

    const std::string group1("group1");
    props.AddGroup(group1);
    // NOTE: Duplicate property name differentiated by different group.
    props.AddProperty(group1, "prop1", 3);
    props.AddProperty(group1, "prop3", 4);
    props.AddProperty(group1, "prop4", 5);

    const std::string group2("group2");
    props.AddGroup(group2);
    props.AddProperty(group2, "prop5", 6);
    return props;
  };

  // Only works for int-valued properties.
  auto properties_equal = [](
      const TestProperties& reference,
      const TestProperties& test) -> ::testing::AssertionResult {
    if (reference.NumProperties() != test.NumProperties()) {
      return ::testing::AssertionFailure()
             << "Different number of total properties. Expected "
             << reference.NumProperties() << " found " << test.NumProperties();
    }
    if (reference.num_groups() != test.num_groups()) {
      return ::testing::AssertionFailure()
          << "Different number of groups. Expected "
          << reference.num_groups() << " found " << test.num_groups();
    }

    for (const auto& group_name : reference.GroupNames()) {
      if (test.has_group(group_name)) {
        for (const auto& pair : reference.GetGroupProperties(group_name)) {
          const std::string& name = pair.first;
          int expected_value = pair.second->get_value<int>();
          if (test.HasProperty(group_name, name)) {
            int test_value = test.GetProperty<int>(group_name, name);
            if (expected_value != test_value) {
              return ::testing::AssertionFailure()
                  << "Expected value for '" << group_name << "':'" << name
                  << "' to be " << expected_value << ". Found " << test_value;
            }
          } else {
            return ::testing::AssertionFailure()
                << "Expected group '" << group_name << "' to have property '"
                << name <<"'. It does not exist.";
          }
        }
      } else {
        return ::testing::AssertionFailure()
            << "Expected group " << group_name
            << " is missing from test properties";
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

}  // namespace
}  // namespace dev
}  // namespace geometry
}  // namespace drake
