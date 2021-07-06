/** @file
 Doxygen-only documentation for @ref system_scalar_conversion.  */

//------------------------------------------------------------------------------
/** @defgroup system_scalar_conversion System Scalar Conversion
    @ingroup technical_notes

System scalar conversion refers to cloning a System templatized by one scalar
type into an identical System that is templatized by a different scalar type.
For example, a `MySystem<double>` could be cloned to create a
`MySystem<AutoDiffXd>` in order to compute the partial numerical derivatives.

See the @ref default_scalars "default scalars" for list of supported types.

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
  plant->get_input_port(0).FixValue(context.get(), 0.0);  // tau
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


@anchor system_scalar_conversion_how_to_write_a_system
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
    : LeafSystem<T>(SystemTypeTag<MySystem>{}), gain_{gain} {}

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
  MySystem() : VectorSystem<T>(SystemTypeTag<MySystem>{}, 1, 1) {}

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
   - all `MySystem` constructors must pass a `SystemTypeTag` to their superclass
     constructor; if all `MySystem` constructors delegate to a single one, this
     is easy because only that one needs to provide the `SystemTypeTag` value;
- `MySystem` has a public scalar-converting copy constructor;
  - the constructor takes a reference to `MySystem<U>` and delegates to another
    constructor;
  - the section *How to write a scalar-converting copy constructor* below
    provides more details on how to implement this constructor.

<h3>Systems not marked as `final`</h3>

For a class hierarchy of Systems that support scalar conversion, a slightly
different pattern is required.

@code
namespace sample {
template <typename T>
class MySystemBase : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MySystemBase);

  // Constructs a system with the given `gain`.
  // Subclasses must use the protected constructor, not this one.
  explicit MySystemBase(double gain)
    : LeafSystem<T>(SystemTypeTag<MySystemBase>{}), gain_{gain} {}

  // Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit MySystemBase(const MySystemBase<U>& other)
    : MySystemBase<T>(other.gain()) {}

  // Returns the gain of this system.
  double gain() const { return gain_; }

 protected:
  // Constructor that specifies scalar-type conversion support.
  // @param converter scalar-type conversion support helper (i.e., AutoDiff,
  // etc.); pass a default-constructed object if such support is not desired.
  explicit MySystemBase(SystemScalarConverter converter, double gain)
    : LeafSystem<T>(std::move(converter)), gain_{gain} {}

  ...
namespace sample {
template <typename T>
class MySystemDerived final : public MySystemBase<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MySystemDerived);

  // Constructs a system with a gain of 1.0.
  MySystemDerived() : MySystemBase<T>(
      SystemTypeTag<MySystemDerived>{},
      1.0) {}

  // Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit MySystemDerived(const MySystemDerived<U>&) : MySystemDerived<T>() {}

  ...
@endcode

The relevant details of the examples are:
- Non-`final` classes like `MySystemBase` must offer a protected constructor
  that takes a SystemScalarConverter as the first argument.
- Constructors for derived classes such as `MySystemDerived` must delegate
  to a base class protected constructor that takes a %SystemScalarConverter,
  never to a public constructor without one.
- `MySystemBase` and `MySystemDerived` both have a public scalar-converting copy
  constructor;
  - if the base system is abstract (cannot be constructed), then it may omit
    this constructor.

<h3>Limiting the supported scalar types</h3>

The framework's default implementation System scalar-type conversion only
converts between a limited set of scalar types, as enumerated by the
drake::systems::SystemScalarConverter::SystemScalarConverter(SystemTypeTag<S>)
constructor documentation.

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

Here, `U` is the source scalar type (to convert from), and `T` the resulting
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
  MySystem() : VectorSystem<T>(SystemTypeTag<MySystem>{}, 1, 1) {}
  template <typename U> explicit MySystem(const MySystem<U>&) : MySystem<T>() {}
@endcode
The default constructor configures the System to have a `input_size == 1` and
`output_size == 1`.  The scalar-converting copy constructor delegates to the
default constructor to re-use that logic by stating `: MySystem<T>()`.

Without delegation, we would  have to duplicate those arguments:
@code
  MySystem() : VectorSystem<T>(SystemTypeTag<MySystem>{}, 1, 1) {}
  // NOT RECOMMENDED because it duplicates the details of calling VectorSystem.
  template <typename U> explicit MySystem(const MySystem<U>&)
    : VectorSystem<T>(SystemTypeTag<MySystem>{}, 1, 1) {}
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


<h2>How to create a Diagram that supports scalar conversion</h2>

In the typical case, no special effort is needed to create a Diagram that
support scalar-type conversion.  The Diagram does not even need to be templated
on a scalar type `T`.

Example using DiagramBuilder::BuildInto:
@code
namespace sample {
class MyDiagram : public Diagram<double> {
 public:
  MyDiagram() {
    DiagramBuilder<double> builder;
    const auto* integrator = builder.AddSystem<Integrator<double>>(1);
    builder.ExportInput(integrator->get_input_port());
    builder.ExportOutput(integrator->get_output_port());
    builder.BuildInto(this);
  }
};
@endcode

In this example, `MyDiagram` will support the same scalar types as the
Integrator.  If any sub-system had been added that did not support, e.g.,
symbolic form, then the Diagram would also not support symbolic form.

By default, even subclasses of a `Diagram<U>` will convert to a `Diagram<T>`,
discarding the diagram subclass details.  For example, in the above sample
code, `MyDiagram::ToAutoDiffXd()` will return an object of runtime type
`Diagram<AutoDiffXd>`, not type `MyDiagram<AutoDiffXd>`.  (There is no such
class as `MyDiagram<AutoDiffXd>` anyway, because `MyDiagram` is not templated.)

In the unusual case that the Diagram's subclass must be preserved during
conversion, a ::drake::systems::SystemTypeTag should be used:

Example using DiagramBuilder::BuildInto along with a `SystemTypeTag`:
@code
namespace sample {
template <typename T>
class SpecialDiagram<T> final : public Diagram<T> {
 public:
  SpecialDiagram() : Diagram<T>(SystemTypeTag<SpecialDiagram>{}) {
    DiagramBuilder<T> builder;
    const auto* integrator = builder.template AddSystem<Integrator<T>>(1);
    builder.ExportInput(integrator->get_input_port());
    builder.ExportOutput(integrator->get_output_port());
    builder.BuildInto(this);
  }
};
@endcode

*/
