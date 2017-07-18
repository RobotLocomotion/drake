#include "drake/systems/framework/system_transmogrifier.h"

#include <gtest/gtest.h>

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace {

// A System that has non-trivial non-T-typed member data.
template <typename T>
class MagicSystem : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MagicSystem);

  int magic() const { return magic_; }

 protected:
  explicit MagicSystem(int magic) : magic_{magic} {}

 private:
  int magic_{};
};

// A System that can transmogrify into *anything*.
template <typename T>
class SystemA : public MagicSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SystemA);

  // User constructor.
  explicit SystemA(int magic) : MagicSystem<T>(magic) {
    this->template SetConcreteSubclass<systems::SystemA>();
  }

  // Transmogrification constructor.
  template <typename U>
  SystemA(const TransmogrifierTag&, const SystemA<U>& other)
      : SystemA(other.magic()) {}
};

// A non-symbolic-compatible helper function.  If the transmogrification
// implementation fails to avoid instantiating System<symbolic::Expression>
// when told not to, then we would see compile errors that this function does
// not compile with T = symbolic::Expression.
template <typename T>
T magic_copysign(int magic, const T& value) {
  if ((magic < 0) == (value < 0)) {
    return value;
  } else {
    return -value;
  }
}

// A System that *cannot* transmogrify into symbolic form.
template <typename T>
class SystemB : public MagicSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SystemB);

  // User constructor.
  explicit SystemB(int magic) : MagicSystem<T>(magic) {
    this->template SetConcreteSubclass<systems::SystemB>();

    // Declare input-output relation of `u[0] = copysign(magic, y[0])`.
    this->DeclareVectorInputPort(BasicVector<T>(1));
    this->DeclareVectorOutputPort(
        BasicVector<T>(1),
        [this](const Context<T>& context, BasicVector<T>* output) {
          const BasicVector<T>& input = *(this->EvalVectorInput(context, 0));
          (*output)[0] = magic_copysign(this->magic(), input[0]);
        });
  }

  // Transmogrification constructor.
  template <typename U>
  SystemB(const TransmogrifierTag&, const SystemB<U>& other)
      : SystemB(other.magic()) {}
};

}  // namespace

namespace system_transmogrifier {
template <> struct Traits<SystemB> : public NonSymbolicTraits {};
}

namespace {

GTEST_TEST(SystemTransmogrifierTest, TestSystemA) {
  const SystemA<double> system_a_double{22};
  const SystemTransmogrifier dut = MakeDefaultSystemTransmogrifier<SystemA>();

  // Transmogrify to AutoDiffXd.
  const std::unique_ptr<System<AutoDiffXd>> system_autodiffxd =
      dut.Convert<AutoDiffXd, double>(system_a_double);
  ASSERT_TRUE(system_autodiffxd != nullptr);

  // Extract the subtype.
  const auto* system_a_autodiffxd =
      dynamic_cast<const SystemA<AutoDiffXd>*>(system_autodiffxd.get());
  ASSERT_TRUE(system_a_autodiffxd != nullptr);

  // Confirm that the magic was preserved.
  EXPECT_EQ(system_a_autodiffxd->magic(), system_a_double.magic());

  // Transmogrify to symbolic.
  const std::unique_ptr<System<symbolic::Expression>> system_symbolic =
      dut.Convert<symbolic::Expression, double>(system_a_double);
  ASSERT_TRUE(system_symbolic != nullptr);

  // Extract the subtype.
  const auto* system_a_symbolic =
      dynamic_cast<const SystemA<symbolic::Expression>*>(system_symbolic.get());
  ASSERT_TRUE(system_a_symbolic != nullptr);

  // Confirm that the magic was preserved.
  EXPECT_EQ(system_a_symbolic->magic(), system_a_double.magic());
}

GTEST_TEST(SystemTransmogrifierTest, TestSystemB) {
  const SystemB<double> system_b_double{22};
  const SystemTransmogrifier dut = MakeDefaultSystemTransmogrifier<SystemB>();

  // Transmogrify to AutoDiffXd.
  const std::unique_ptr<System<AutoDiffXd>> system_autodiffxd =
      dut.Convert<AutoDiffXd, double>(system_b_double);
  ASSERT_TRUE(system_autodiffxd != nullptr);

  // Extract the subtype.
  const auto* system_b_autodiffxd =
      dynamic_cast<const SystemB<AutoDiffXd>*>(system_autodiffxd.get());
  ASSERT_TRUE(system_b_autodiffxd != nullptr);

  // Confirm that the magic was preserved.
  EXPECT_EQ(system_b_autodiffxd->magic(), system_b_double.magic());

  // Transmogrify to symbolic should not work.
  const std::unique_ptr<System<symbolic::Expression>> system_symbolic =
      dut.Convert<symbolic::Expression, double>(system_b_double);
  ASSERT_TRUE(system_symbolic == nullptr);
}

}  // namespace
}  // namespace systems
}  // namespace drake
