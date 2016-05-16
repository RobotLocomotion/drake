#include "drake/common/unique_ptr_ref.h"

#include <string>
#include <utility>
#include <vector>
#include <unordered_map>

#include "gtest/gtest.h"

using std::string;

namespace drake {
namespace {

class AbstractJoint {
 public:
  virtual ~AbstractJoint() {}
  int get_joint_num() const { return joint_num_; }

 protected:
  AbstractJoint() {}

 private:
  friend class Mechanism;
  void set_joint_num(int num) { joint_num_ = num; }
  int joint_num_{-1};  // assigned by Mechanism
};

class PinJoint : public AbstractJoint {
 public:
  double GetAngle() const { return 1.; }
};

class SlidingJoint : public AbstractJoint {
 public:
  double GetLength() const { return 10.; }
};

class Mechanism {
 public:
  void addJoint(std::unique_ptr<AbstractJoint> joint) {
    joint->set_joint_num((int)joints_.size());
    joints_.push_back(std::move(joint));
  }

  AbstractJoint* get_joint(int num) const { return joints_[num].get(); }

 private:
  std::vector<std::unique_ptr<AbstractJoint>> joints_;
};

GTEST_TEST(UniquePtrRefTest, APITest) {
  auto pj = make_unique_ref<PinJoint>();
  auto sj = make_unique_ref<SlidingJoint>();

  // Verify that we initially have ownership.
  EXPECT_TRUE(pj.is_owner());
  EXPECT_TRUE(sj.is_owner());

  // Transfer ownership to compound class object, but retain references.
  Mechanism mech;
  mech.addJoint(pj.move());
  mech.addJoint(sj.move());

  // We shouldn't have ownership any more.
  EXPECT_FALSE(pj.is_owner());
  EXPECT_FALSE(sj.is_owner());

  // Access base class member.
  EXPECT_EQ(pj->get_joint_num(), 0);
  EXPECT_EQ(sj->get_joint_num(), 1);

  // Check that the original object got moved into Mechanism.
  EXPECT_EQ(mech.get_joint(0), pj.get());
  EXPECT_EQ(mech.get_joint(1), sj.get());

  EXPECT_NE(dynamic_cast<PinJoint*>(mech.get_joint(0)), nullptr);
  EXPECT_NE(dynamic_cast<SlidingJoint*>(mech.get_joint(1)), nullptr);

  // Can still access concrete class methods.
  EXPECT_EQ(pj->GetAngle(), 1.);
  EXPECT_EQ(sj->GetLength(), 10.);
}

GTEST_TEST(UniquePtrRefTest, Construction) {
  // These should be equivalent.
  unique_ptr_ref<int> ip(new int(5));
  auto ip2 = make_unique_ref<int>(5);
  EXPECT_EQ(*ip, *ip2);

  auto pj = make_unique_ref<PinJoint>();
  auto sj = make_unique_ref<SlidingJoint>();

  unique_ptr_ref<AbstractJoint> aj(pj);
  EXPECT_TRUE(pj.is_owner());
  EXPECT_FALSE(aj.is_owner());
  EXPECT_EQ(aj.get(), pj.get());

  unique_ptr_ref<AbstractJoint> aj2(std::move(pj));
  EXPECT_FALSE(pj.is_owner());
  EXPECT_TRUE(aj2.is_owner());
  EXPECT_EQ(aj2.get(), pj.get());

  // Copy construction from a unique_ptr should copy the pointer but 
  // not take ownership.
  std::unique_ptr<int> upi(new int(10));
  unique_ptr_ref<int> upri(upi);
  EXPECT_FALSE(upri.is_owner());
  EXPECT_EQ(upri.get(), upi.get());

  // Move construction from a unique_ptr should take ownership and leave
  // the source empty.
  unique_ptr_ref<int> upri2(std::move(upi));
  EXPECT_FALSE(upi);
  EXPECT_TRUE(upri2.is_owner());
  EXPECT_TRUE(upri2);

}

GTEST_TEST(UniquePtrRefTest, Hash) {
    // Hash result should be the same as the built-in hash of the contained
    // pointer.
    auto toBeHashed = make_unique_ref<SlidingJoint>();
    EXPECT_EQ(std::hash<unique_ptr_ref<SlidingJoint>>()(toBeHashed), 
              std::hash<SlidingJoint*>()(toBeHashed.get()));

    // Should be possible to store in an unordered map using the default
    // hash function (since the specialization is in std:: namespace).
    auto i1 = make_unique_ref<int>(5);
    auto i2 = make_unique_ref<int>(10);

    std::unordered_map<unique_ptr_ref<int>,std::string> map;
    map[i1] = "i1";
    map[i2] = "i2";
    EXPECT_EQ(map[i1], "i1");
    EXPECT_EQ(map[i2], "i2");
}

}  // namespace
}  // namespace drake
