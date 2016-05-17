#include "drake/common/unique_ptr_ref.h"

#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

using std::string;

namespace drake {
namespace {

// This is a skeletal Joint element, with some concrete examples.
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

// This represents a composite class to which we must pass ownership of
// the elements. This can only hold abstract joints.
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

// Test that the intended basic use of the class as an API cleanup works.
GTEST_TEST(UniquePtrRefTest, APITest) {
  auto pj = make_unique_ref<PinJoint>();
  auto sj = make_unique_ref<SlidingJoint>();

  // Verifies that we initially have ownership.
  EXPECT_TRUE(pj.is_owner());
  EXPECT_TRUE(sj.is_owner());

  // Transfers ownership to compound class object, but retain references.
  Mechanism mech;
  mech.addJoint(pj.move());
  mech.addJoint(sj.move());

  // We shouldn't have ownership any more.
  EXPECT_FALSE(pj.is_owner());
  EXPECT_FALSE(sj.is_owner());

  // Access base class member.
  EXPECT_EQ(pj->get_joint_num(), 0);
  EXPECT_EQ(sj->get_joint_num(), 1);

  // Checks that the original object got moved into Mechanism.
  EXPECT_EQ(mech.get_joint(0), pj.get());
  EXPECT_EQ(mech.get_joint(1), sj.get());

  EXPECT_NE(dynamic_cast<PinJoint*>(mech.get_joint(0)), nullptr);
  EXPECT_NE(dynamic_cast<SlidingJoint*>(mech.get_joint(1)), nullptr);

  // Can still access concrete class methods.
  EXPECT_EQ(pj->GetAngle(), 1.);
  EXPECT_EQ(sj->GetLength(), 10.);
}

// Test that constructors and some comparison operators work properly.
GTEST_TEST(UniquePtrRefTest, Construction) {
  unique_ptr_ref<std::vector<int>> dummy, dummy2(nullptr);
  EXPECT_FALSE(dummy);  // i.e., it's nullptr
  EXPECT_FALSE(dummy2);
  EXPECT_TRUE(dummy.empty());
  EXPECT_TRUE(dummy == dummy2 && dummy <= dummy2 && dummy >= dummy2);
  EXPECT_FALSE(dummy != dummy2 && dummy < dummy2 && dummy > dummy2);
  EXPECT_FALSE(dummy.is_owner());
  EXPECT_TRUE(dummy == nullptr && nullptr == dummy);
  EXPECT_FALSE(dummy > nullptr && nullptr > dummy);
  EXPECT_FALSE(dummy < nullptr && nullptr < dummy);
  EXPECT_TRUE(dummy >= nullptr && nullptr >= dummy);
  EXPECT_TRUE(dummy <= nullptr && nullptr <= dummy);

  // These should be equivalent.
  unique_ptr_ref<int> ip(new int(5));
  auto ip2 = make_unique_ref<int>(5);
  EXPECT_EQ(*ip, *ip2);

  // Copy construction; pointer copies but not ownership.
  unique_ptr_ref<int> copy_ip(ip);
  EXPECT_EQ(copy_ip.get(), ip.get());
  EXPECT_EQ(copy_ip, ip);
  EXPECT_FALSE(copy_ip.is_owner());
  EXPECT_TRUE(ip.is_owner());

  // Move construction; pointer copied, ownership stolen.
  unique_ptr_ref<int> move_ip(std::move(ip));
  EXPECT_EQ(move_ip.get(), ip.get());
  EXPECT_EQ(move_ip, ip);
  EXPECT_TRUE(move_ip.is_owner());
  EXPECT_FALSE(ip.is_owner());

  auto pj = make_unique_ref<PinJoint>();
  auto sj = make_unique_ref<SlidingJoint>();

  // Copy construction from compatible pointer; pointer copied but not owned.
  unique_ptr_ref<AbstractJoint> aj(pj);
  EXPECT_EQ(aj.get(), pj.get());
  EXPECT_EQ(aj, pj);
  EXPECT_TRUE(pj.is_owner());
  EXPECT_FALSE(aj.is_owner());

  // Move construction from compatible pointer; ownership stolen.
  unique_ptr_ref<AbstractJoint> aj2(std::move(pj));
  EXPECT_EQ(aj2.get(), pj.get());
  EXPECT_EQ(aj2, pj);
  EXPECT_FALSE(pj.is_owner());
  EXPECT_TRUE(aj2.is_owner());

  // Copy construction from a unique_ptr is not permitted.
  // Move construction from a unique_ptr should take ownership and leave
  // the source empty.
  std::unique_ptr<int> upi(new int(10));
  unique_ptr_ref<int> upri2(std::move(upi));
  EXPECT_FALSE(upi);
  EXPECT_TRUE(upri2.is_owner());
  EXPECT_TRUE(upri2);
}

// Test assignment operators, plus dereferencing.
GTEST_TEST(UniquePtrRefTest, Assignment) {
  unique_ptr_ref<std::vector<int>> dummy;

  auto ip = make_unique_ref<int>(5);
  // Quick check of dereferencing first.
  EXPECT_EQ(*ip, 5);

  unique_ptr_ref<int> copy_ip;
  copy_ip = ip;  // copies pointer but not ownership
  EXPECT_EQ(copy_ip.get(), ip.get());
  EXPECT_EQ(copy_ip, ip);
  EXPECT_FALSE(copy_ip.is_owner());
  EXPECT_TRUE(ip.is_owner());

  // Move assignment; pointer copied, ownership stolen.
  unique_ptr_ref<int> move_ip;
  move_ip = std::move(ip);
  EXPECT_EQ(move_ip.get(), ip.get());
  EXPECT_EQ(move_ip, ip);
  EXPECT_TRUE(move_ip.is_owner());
  EXPECT_FALSE(ip.is_owner());

  auto pj = make_unique_ref<PinJoint>();
  auto sj = make_unique_ref<SlidingJoint>();

  // Copy assignment from compatible pointer; pointer copied but not owned.
  unique_ptr_ref<AbstractJoint> aj;
  aj = pj;
  EXPECT_EQ(aj.get(), pj.get());
  EXPECT_EQ(aj, pj);
  EXPECT_TRUE(pj.is_owner());
  EXPECT_FALSE(aj.is_owner());

  // Move assignment from compatible pointer; ownership stolen.
  unique_ptr_ref<AbstractJoint> aj2;
  aj2 = std::move(pj);
  EXPECT_EQ(aj2.get(), pj.get());
  EXPECT_EQ(aj2, pj);
  EXPECT_FALSE(pj.is_owner());
  EXPECT_TRUE(aj2.is_owner());

  // Copy assignment from a unique_ptr is not permitted.
  // Move assignment from a unique_ptr should take ownership and leave
  // the source empty.
  std::unique_ptr<int> upi(new int(10));
  unique_ptr_ref<int> upri2;
  upri2 = std::move(upi);
  EXPECT_FALSE(upi);
  EXPECT_TRUE(upri2.is_owner());
  EXPECT_TRUE(upri2);

  // Assignment to nullptr should clear.
  upri2 = nullptr;
  EXPECT_TRUE(upri2.empty());
  EXPECT_FALSE(upri2);
  EXPECT_FALSE(upri2.is_owner());
}

GTEST_TEST(UniquePtrRefTest, Utility) {
  auto ip = make_unique_ref<int>(5);
  EXPECT_EQ(*ip, 5);
  EXPECT_TRUE(ip.is_owner());

  ip.reset();  // back to empty
  EXPECT_TRUE(ip.empty());
  EXPECT_FALSE(ip);
  EXPECT_FALSE(ip.is_owner());

  ip.reset(new int(10));
  EXPECT_FALSE(ip.empty());
  EXPECT_EQ(*ip, 10);
  EXPECT_TRUE(ip.is_owner());

  // Test release and reset from compatible pointer (not same type).
  auto pj = make_unique_ref<PinJoint>();
  unique_ptr_ref<AbstractJoint> aj;
  aj.reset(pj.release());  // reset from compatible type
  EXPECT_EQ(pj, aj);       // same pointer
  EXPECT_FALSE(pj.is_owner());
  EXPECT_TRUE(aj.is_owner());

  // release() from non-owner container should return null.
  EXPECT_EQ(pj.release(), nullptr);

  auto s1 = make_unique_ref<std::string>("hello");
  auto s2 = make_unique_ref<std::string>("goodbye");
  std::string* sp1 = s1.get();
  std::string* sp2 = s2.get();

  using std::swap;
  swap(s1, s2);  // Should pick drake::swap by argument dependent resolution.
  EXPECT_EQ(*s1, "goodbye");
  EXPECT_EQ(*s2, "hello");
  EXPECT_EQ(s1.get(), sp2);
  EXPECT_EQ(s2.get(), sp1);

  // Let's make sure the pointer in sp1 is less than the one in sp2.
  if (s2.get() < s1.get()) swap(s1, s2);
  EXPECT_TRUE(s1 < s2 && s2 > s1);
  EXPECT_TRUE(s1 <= s2 && s2 >= s1);
  EXPECT_FALSE(s1 == s2);
  EXPECT_TRUE(s1 != s2);

  // String insertion should just insert the pointer representation.
  std::ostringstream ss1, ss2;
  ss1 << s1;
  ss2 << s1.get();
  EXPECT_EQ(ss1.str(), ss2.str());
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

  std::unordered_map<unique_ptr_ref<int>, std::string> map;
  map[i1] = "i1";
  map[i2] = "i2";
  EXPECT_EQ(map[i1], "i1");
  EXPECT_EQ(map[i2], "i2");
}

}  // namespace
}  // namespace drake
