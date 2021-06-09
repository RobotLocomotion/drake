#include "drake/bindings/pydrake/common/type_pack.h"

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/nice_type_name.h"

using std::string;
using std::vector;

namespace drake {
namespace {

template <typename... Ts>
struct SimpleTemplate {};

using Pack = type_pack<int, double, char, void>;

// Mostly, this just checks for compilation failures.
GTEST_TEST(TypeUtilTest, TypeAt) {
  using T_0 = type_at<0, int, double, char, void>::type;
  EXPECT_TRUE((std::is_same_v<T_0, int>));
  using T_1 = type_at<1, int, double, char, void>::type;
  EXPECT_TRUE((std::is_same_v<T_1, double>));
  using T_2 = type_at<2, int, double, char, void>::type;
  EXPECT_TRUE((std::is_same_v<T_2, char>));
  using T_3 = type_at<3, int, double, char, void>::type;
  EXPECT_TRUE((std::is_same_v<T_3, void>));

  EXPECT_TRUE((std::is_same_v<Pack::type_at<0>, int>));
  EXPECT_TRUE((std::is_same_v<Pack::type_at<1>, double>));
  EXPECT_TRUE((std::is_same_v<Pack::type_at<2>, char>));
  EXPECT_TRUE((std::is_same_v<Pack::type_at<3>, void>));
}

GTEST_TEST(TypeUtilTest, TypeTags) {
  // Ensure that we can default-construct tags for types that are not
  // default-constructible.
  auto tag_obj = type_tag<void>{};
  EXPECT_TRUE((std::is_same_v<decltype(tag_obj), type_tag<void>>));
  auto pack_obj_empty = type_pack<>{};
  EXPECT_TRUE((std::is_same_v<decltype(pack_obj_empty), type_pack<>>));
  auto pack_obj = type_pack<void, void>{};
  EXPECT_TRUE((std::is_same_v<decltype(pack_obj), type_pack<void, void>>));
}

GTEST_TEST(TypeUtilTest, Bind) {
  using T_0 = Pack::bind<SimpleTemplate>;
  using T_0_expected = SimpleTemplate<int, double, char, void>;
  EXPECT_TRUE((std::is_same_v<T_0, T_0_expected>));
  Pack pack;
  using T_1 = decltype(type_bind<SimpleTemplate>(pack));
  using T_1_expected = SimpleTemplate<int, double, char, void>;
  EXPECT_TRUE((std::is_same_v<T_1, T_1_expected>));
}

GTEST_TEST(TypeUtilTest, Extract) {
  using T = SimpleTemplate<int, double, char, void>;
  using TPack = type_pack_extract<T>;
  EXPECT_TRUE((std::is_same_v<TPack, Pack>));
}

/// Example usages of `type_visit`.
GTEST_TEST(TypeUtilTest, Visit) {
  using PackTags = type_pack<  // BR
      type_tag<int>,           //
      type_tag<double>,        //
      type_tag<char>,          //
      type_tag<void>>;
  vector<string> names;
  const vector<string> names_expected = {"int", "double", "char", "void"};

  // This is the (optional) state of the loop.
  int counter = 0;
  // This it the 'body' of the loop.
  auto visitor = [&counter, &names](auto tag) {
    using T = typename decltype(tag)::type;
    ++counter;
    names.push_back(NiceTypeName::Get<T>());
  };
  // This executes the loop.
  type_visit(visitor, PackTags{});

  // Check output.
  EXPECT_EQ(counter, 4);
  EXPECT_EQ(names, names_expected);

  names.clear();
  type_visit<type_visit_with_tag<>>(visitor, Pack{});
  EXPECT_EQ(names, names_expected);

  // N.B. `Check` will operate types contained within the pack as dictated by
  // the `VisitWith` parameter in `type_visit`.
  // As an example, see below that the two visit calls will be visiting the
  // same types (with the same results), but `Check` will operate on
  // (a) the direct types with `type_visit_with_tag<>` and (b) the tag types
  // with `type_visit_with_default` (the default parameter).
  const vector<string> names_expected_sub = {"int", "char", "void"};

  names.clear();
  type_visit<type_visit_with_tag<>, type_check_different_from<double>::type>(
      visitor, Pack{});
  EXPECT_EQ(names, names_expected_sub);

  names.clear();
  using CheckTag =
      template_single_tag<type_check_different_from<type_tag<double>>::type>;
  type_visit(visitor, PackTags{}, CheckTag{});
  EXPECT_EQ(names, names_expected_sub);
};

GTEST_TEST(TypeUtilTest, Hash) {
  using T = int;
  using U = int;
  using V = double;
  EXPECT_EQ(type_hash<T>(), type_hash<U>());
  EXPECT_NE(type_hash<T>(), type_hash<V>());
}

}  // namespace
}  // namespace drake
