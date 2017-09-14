//------------------------------------------------------------------------------
/** @defgroup system_scalar_conversion System Scalar Conversion

System scalar conversion refers to cloning a System templatized by one scalar
type into an identical System that is templatized by a different scalar type.
For example, a `MySystem<double>` could be cloned to create a
`MySystem<AutoDiffXd>` in order to compute the partial numerical derivatives.

Common scalar types include:
- `double`
- drake::AutoDiffXd, an automatic differentation scalar providing partial
  derivatives of any numerical result of the System with respect to any of the
  numerical values that can be contained in a Context (time, inputs,
  parameters, and state).
- drake::symbolic::Expression, a symbolic scalar providing the symbolic form of
  arithmetic expressions.


<h2>Example use of system scalar conversion</h2>

This is a small example to illustrate the concept.

@internal
N.B. The code sample immediately below must be kept in sync with the identical
code in test/system_scalar_conversion_doxygen_test.cc.  This ensures that the
code sample compiles and runs successfully.
@endinternal
@code
  // Establish the plant and its initial conditions:
  //   tau = 0, theta = 0.1, thetadot = 0.2.
  auto plant = std::make_unique<PendulumPlant<double>>();
  auto context = plant->CreateDefaultContext();
  context->FixInputPort(0, Vector1d::Zero());  // tau
  auto* state = dynamic_cast<PendulumState<double>*>(
      context->get_mutable_continuous_state_vector());
  state->set_theta(0.1);
  state->set_thetadot(0.2);
  double energy = plant->CalcTotalEnergy(*context);
  ASSERT_NEAR(energy, -4.875, 0.001);

  // Convert the plant and its context to use AutoDiff.
  auto autodiff_plant = System<double>::ToAutoDiffXd(*plant);
  auto autodiff_context = autodiff_plant->CreateDefaultContext();
  autodiff_context->SetTimeStateAndParametersFrom(*context);
  autodiff_plant->FixInputPortsFrom(*plant, *context, autodiff_context.get());

  // Differentiate with respect to theta by setting dtheta/dtheta = 1.0.
  constexpr int kNumDerivatives = 1;
  auto& xc = *autodiff_context->get_mutable_continuous_state_vector();
  xc[PendulumStateIndices::kTheta].derivatives() =
      MatrixXd::Identity(kNumDerivatives, kNumDerivatives).col(0);

  // Compute denergy/dtheta around its initial conditions.
  AutoDiffXd autodiff_energy =
      autodiff_plant->CalcTotalEnergy(*autodiff_context);
  ASSERT_NEAR(autodiff_energy.value(), -4.875, 0.001);
  ASSERT_EQ(autodiff_energy.derivatives().size(), 1);
  ASSERT_NEAR(autodiff_energy.derivatives()[0], 0.490, 0.001);
@endcode

For a more thorough example, refer to the implementation of
drake::systems::Linearize.


<h2>How to write a System that supports scalar conversion</h2>

In the typical case, Drake users' Systems should be marked with the C++ keyword
`final` to prevent them from being subclassed.  Whether or not the System is
`final` determines the best way to support scalar conversion.

In the examples below, we use `MySystem` as the name of a System class being
implemented by a Drake user.

<h3>Systems marked as `final`</h3>

For system marked with `final`,
the two examples below show how to enable system scalar conversion.

Example using drake::systems::LeafSystem as the base class:
@code
namespace sample {
template <typename T>
class MySystem final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MySystem);

  // Constructs a system with a gain of 1.0.
  MySystem() : MySystem(1.0) {}

  // Constructs a system with the given `gain`.
  explicit MySystem(double gain)
    : LeafSystem<T>(SystemTypeTag<sample::MySystem>{}), gain_{gain} {}

  // Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit MySystem(const MySystem<U>& other) : MySystem<T>(other.gain()) {}

  // Returns the gain of this system.
  double gain() const { return gain_; }

  ...
@endcode

Example using drake::systems::VectorSystem as the base class:
@code
namespace sample {
template <typename T>
class MySystem final : public VectorSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MySystem);

  // Default constructor.
  MySystem() : VectorSystem<T>(SystemTypeTag<sample::MySystem>{}, 1, 1) {}

  // Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit MySystem(const MySystem<U>&) : MySystem<T>() {}

  ...
@endcode

The relevant details of the examples are:
- `MySystem` is templated on a scalar type `T`;
- `MySystem` is marked `final`;
- `DRAKE_NO_COPY_...` disables the built-in copy and move constructors;
- `MySystem` has whatever usual constructors make sense;
   - sometimes it will have a default constructor;
   - sometimes it will be given arguments in its constructor;
   - as a C++ best practice, constructors should delegate into one primary
     constructor;
     - we see that above in the first example where the default constructor
       delegates to the one-argument constructor;
- `MySystem` has a public scalar-converting copy constructor;
  - the constructor takes a reference to `MySystem<U>` and delegates to another
    constructor;
  - the section *How to write a scalar-converting copy constructor* below
    provides more details on how to implement this constructor.

<h3>Systems not marked as `final`</h3>

TODO(jwnimmer-tri) Document how to write a class hierarchy of Systems that
support scalar conversion.

<h3>Limiting the supported scalar types</h3>

The framework's default implementation System scalar-type conversion only
converts between a limited set of scalar types, as enumerated by the
drake::systems::SystemScalarConverter::SystemScalarConverter(SystemTypeTag<S>)
constructor.

Systems may specialize their drake::systems::scalar_conversion::Traits to
govern the supported scalar types.  The recommended mechanism is to use
drake::systems::scalar_conversion::NonSymbolicTraits or
drake::systems::scalar_conversion::FromDoubleTraits.

<h2>How to write a scalar-converting copy constructor</h2>

The scalar-converting copy constructor uses the following form:
@code
template <typename T>
class Foo {
  // Scalar-converting copy constructor.
  template <typename U>
  explicit Foo(const Foo<U>& other);
};
@endcode

Here, `U` is the donor scalar type (to convert from), and `T` the resulting
scalar type (to convert into).  For example, in the second line of
@code
Foo<double> foo;
Foo<AutoDiffXd> autodiff_foo{foo};
@endcode
we are calling the constructor `Foo<AutoDiffXd>::Foo(const Foo<double>&)` so we
have `U = double` and `T = AutoDiffXd`.  In other words, we start with a
`double`-valued object and use it to create an `AutoDiffXd`-valued object.

<h3>Delegation</h3>

In almost all cases, the implementation of the scalar-converting copy
constructor should delegate to another regular constructor, rather than
re-implementing its logic.  For example, in the `VectorSystem`-based example
above we have:
@code
  MySystem() : VectorSystem<T>(SystemTypeTag<sample::MySystem>{}, 1, 1) {}
  template <typename U> explicit MySystem(const MySystem<U>&) : MySystem<T>() {}
@endcode
The default constructor configures the System to have a `input_size == 1` and
`output_size == 1`.  The scalar-converting copy constructor delegates to the
default constructor to re-use that logic by stating `: MySystem<T>()`.

Without delegation, we would  have to duplicate those arguments:
@code
  MySystem() : VectorSystem<T>(SystemTypeTag<sample::MySystem>{}, 1, 1) {}
  // NOT RECOMMENDED because it duplicates the details of calling VectorSystem.
  template <typename U> explicit MySystem(const MySystem<U>&)
    : VectorSystem<T>(SystemTypeTag<sample::MySystem>{}, 1, 1) {}
@endcode

<h3>Instance data</h3>

If the object being copied from (usually named `other`) has any instance data
or customizations, the scalar-converting copy constructor should propagate them
from `other` to `this`.  For example, in the `LeafSystem`-based example above,
we have:
@code
  template <typename U>
  explicit MySystem(const MySystem<U>& other) : MySystem<T>(other.gain()) {}
@endcode

*/
