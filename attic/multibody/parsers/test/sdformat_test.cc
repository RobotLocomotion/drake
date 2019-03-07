#include <iostream>

#include <gtest/gtest.h>
#include <sdf/sdf.hh>

namespace drake {
namespace multibody {
namespace parsers {
namespace test {
namespace {

GTEST_TEST(SDFormatTest, TestBasic) {
  const std::string sdf_str("<?xml version='1.0'?>"
                            "<sdf version='1.6'>"
                            "  <model name='my_model'>"
                            "    <link name='link'/>"
                            "  </model>"
                            "</sdf>");
  sdf::SDFPtr sdf_parsed(new sdf::SDF());
  ASSERT_TRUE(sdf::init(sdf_parsed));

  EXPECT_TRUE(sdf::readString(sdf_str, sdf_parsed));

  sdf::ElementPtr model = sdf_parsed->Root()->GetElement("model");
  std::string model_name = model->Get<std::string>("name");

  EXPECT_EQ(model_name, std::string("my_model"));

  std::string link_name = model->GetElement("link")->Get<std::string>("name");
  EXPECT_EQ(link_name, std::string("link"));
}

}  // namespace
}  // namespace test
}  // namespace parsers
}  // namespace multibody
}  // namespace drake
