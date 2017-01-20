#include <iostream>

#include <gtest/gtest.h>

#include <sdf/sdf.hh>

namespace drake {
namespace multibody {
namespace parsers {
namespace test {
namespace {

GTEST_TEST(SDFormatTest, TestBasic) {
  std::string sdfstr("<?xml version='1.0'?><sdf version='1.6'><model name='my_model'><link name='link'/></model></sdf>");
  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);

  EXPECT_TRUE(sdf::readString(sdfstr, sdfParsed));

  sdf::ElementPtr model = sdfParsed->Root()->GetElement("model");
  std::string modelName = model->Get<std::string>("name");

  EXPECT_EQ(modelName, std::string("my_model"));

  std::string linkName = model->GetElement("link")->Get<std::string>("name");
  EXPECT_EQ(linkName, std::string("link"));
}

}  // namespace
}  // namespace test
}  // namespace parsers
}  // namespace multibody
}  // namespace drake
