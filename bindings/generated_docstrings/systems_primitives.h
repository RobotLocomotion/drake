#pragma once

// GENERATED FILE DO NOT EDIT
// This file contains docstrings for the Python bindings that were
// automatically extracted by mkdoc.py.

#include <array>
#include <utility>

#if defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif

// #include "drake/systems/primitives/adder.h"
// #include "drake/systems/primitives/affine_system.h"
// #include "drake/systems/primitives/barycentric_system.h"
// #include "drake/systems/primitives/bus_creator.h"
// #include "drake/systems/primitives/bus_selector.h"
// #include "drake/systems/primitives/constant_value_source.h"
// #include "drake/systems/primitives/constant_vector_source.h"
// #include "drake/systems/primitives/demultiplexer.h"
// #include "drake/systems/primitives/discrete_derivative.h"
// #include "drake/systems/primitives/discrete_time_delay.h"
// #include "drake/systems/primitives/discrete_time_integrator.h"
// #include "drake/systems/primitives/first_order_low_pass_filter.h"
// #include "drake/systems/primitives/gain.h"
// #include "drake/systems/primitives/integrator.h"
// #include "drake/systems/primitives/linear_system.h"
// #include "drake/systems/primitives/linear_transform_density.h"
// #include "drake/systems/primitives/matrix_gain.h"
// #include "drake/systems/primitives/multilayer_perceptron.h"
// #include "drake/systems/primitives/multiplexer.h"
// #include "drake/systems/primitives/pass_through.h"
// #include "drake/systems/primitives/port_switch.h"
// #include "drake/systems/primitives/random_source.h"
// #include "drake/systems/primitives/saturation.h"
// #include "drake/systems/primitives/selector.h"
// #include "drake/systems/primitives/shared_pointer_system.h"
// #include "drake/systems/primitives/sine.h"
// #include "drake/systems/primitives/sparse_matrix_gain.h"
// #include "drake/systems/primitives/symbolic_vector_system.h"
// #include "drake/systems/primitives/trajectory_affine_system.h"
// #include "drake/systems/primitives/trajectory_linear_system.h"
// #include "drake/systems/primitives/trajectory_source.h"
// #include "drake/systems/primitives/transfer_function.h"
// #include "drake/systems/primitives/vector_log.h"
// #include "drake/systems/primitives/vector_log_sink.h"
// #include "drake/systems/primitives/wrap_to_system.h"
// #include "drake/systems/primitives/zero_order_hold.h"

// Symbol: pydrake_doc_systems_primitives
constexpr struct /* pydrake_doc_systems_primitives */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::systems
    struct /* systems */ {
      // Symbol: drake::systems::AddRandomInputs
      struct /* AddRandomInputs */ {
        // Source: drake/systems/primitives/random_source.h
        const char* doc =
R"""(For each subsystem input port in ``builder`` that is (a) not yet
connected and (b) labeled as random in the InputPort, this method will
add a new RandomSource system of the appropriate type and connect it
to the subsystem input port.

Parameter ``sampling_interval_sec``:
    interval to be used for all new sources.

Returns:
    the total number of RandomSource systems added.

See also:
    stochastic_systems)""";
      } AddRandomInputs;
      // Symbol: drake::systems::Adder
      struct /* Adder */ {
        // Source: drake/systems/primitives/adder.h
        const char* doc =
R"""(An adder for arbitrarily many inputs of equal size.

.. pydrake_system::

    name: Adder
    input_ports:
    - u0
    - ...
    - u(N-1)
    output_ports:
    - sum)""";
        // Symbol: drake::systems::Adder::Adder<T>
        struct /* ctor */ {
          // Source: drake/systems/primitives/adder.h
          const char* doc =
R"""(Construct an Adder System.

Parameter ``num_inputs``:
    is the number of input ports to be added.

Parameter ``size``:
    number of elements in each input and output signal.)""";
          // Source: drake/systems/primitives/adder.h
          const char* doc_copyconvert =
R"""(Scalar-converting copy constructor. See system_scalar_conversion.)""";
        } ctor;
      } Adder;
      // Symbol: drake::systems::AffineSystem
      struct /* AffineSystem */ {
        // Source: drake/systems/primitives/affine_system.h
        const char* doc =
R"""(A discrete OR continuous affine system (with constant coefficients).

.. pydrake_system::

    name: AffineSystem
    input_ports:
    - u0
    output_ports:
    - y0

Let ``u`` denote the input vector, ``x`` denote the state vector, and
``y`` denote the output vector.

If ``time_period > 0.0``, the affine system will have the following
discrete-time state update:

.. math:: x(t+h) = A x(t) + B u(t) + f_0,

where ``h`` is the time_period. If ``time_period == 0.0``, the affine
system will have the following continuous-time state update:

.. math:: \dot{x} = A x + B u + f_0.

In both cases, the system will have the output:

.. math:: y = C x + D u + y_0,

See also:
    LinearSystem

See also:
    MatrixGain)""";
        // Symbol: drake::systems::AffineSystem::A
        struct /* A */ {
          // Source: drake/systems/primitives/affine_system.h
          const char* doc_0args = R"""(@name Helper getter methods.)""";
          // Source: drake/systems/primitives/affine_system.h
          const char* doc_1args =
R"""(@name Implementations of TimeVaryingAffineSystem<T>'s pure virtual
methods.)""";
        } A;
        // Symbol: drake::systems::AffineSystem::AffineSystem<T>
        struct /* ctor */ {
          // Source: drake/systems/primitives/affine_system.h
          const char* doc_7args =
R"""(Constructs an Affine system with a fixed set of coefficient matrices
``A``, `B`,``C``, and ``D`` as well as fixed initial velocity offset
``xDot0`` and output offset ``y0``. The coefficient matrices must obey
the following dimensions : | Matrix | Num Rows | Num Columns |
|:-------:|:-----------:|:-----------:| | A | num states | num states
| | B | num states | num inputs | | f0 | num_states | 1 | | C | num
outputs | num states | | D | num outputs | num inputs | | y0 |
num_outputs | 1 |

Empty matrices are treated as zero matrices with the appropriate
number of rows and columns.

Parameter ``time_period``:
    Defines the period of the discrete time system; use
    time_period=0.0 to denote a continuous time system. $*Default:*
    0.0

Subclasses must use the protected constructor, not this one.)""";
          // Source: drake/systems/primitives/affine_system.h
          const char* doc_copyconvert =
R"""(Scalar-converting copy constructor. See system_scalar_conversion.)""";
          // Source: drake/systems/primitives/affine_system.h
          const char* doc_8args =
R"""(Constructor that specifies scalar-type conversion support.

Parameter ``converter``:
    scalar-type conversion support helper (i.e., AutoDiff, etc.); pass
    a default-constructed object if such support is not desired. See
    system_scalar_conversion for detailed background and examples
    related to scalar-type conversion support.)""";
        } ctor;
        // Symbol: drake::systems::AffineSystem::B
        struct /* B */ {
          // Source: drake/systems/primitives/affine_system.h
          const char* doc = R"""()""";
        } B;
        // Symbol: drake::systems::AffineSystem::C
        struct /* C */ {
          // Source: drake/systems/primitives/affine_system.h
          const char* doc = R"""()""";
        } C;
        // Symbol: drake::systems::AffineSystem::D
        struct /* D */ {
          // Source: drake/systems/primitives/affine_system.h
          const char* doc = R"""()""";
        } D;
        // Symbol: drake::systems::AffineSystem::MakeAffineSystem
        struct /* MakeAffineSystem */ {
          // Source: drake/systems/primitives/affine_system.h
          const char* doc =
R"""(Creates a unique pointer to AffineSystem<T> by decomposing
``dynamics`` and ``outputs`` using ``state_vars`` and ``input_vars``.

Raises:
    RuntimeError if either ``dynamics`` or ``outputs`` is not affine
    in ``state_vars`` and ``input_vars``.)""";
        } MakeAffineSystem;
        // Symbol: drake::systems::AffineSystem::UpdateCoefficients
        struct /* UpdateCoefficients */ {
          // Source: drake/systems/primitives/affine_system.h
          const char* doc =
R"""(Updates the coefficients of the affine system. The new coefficients
must have the same size as existing coefficients.)""";
        } UpdateCoefficients;
        // Symbol: drake::systems::AffineSystem::f0
        struct /* f0 */ {
          // Source: drake/systems/primitives/affine_system.h
          const char* doc = R"""()""";
        } f0;
        // Symbol: drake::systems::AffineSystem::y0
        struct /* y0 */ {
          // Source: drake/systems/primitives/affine_system.h
          const char* doc = R"""()""";
        } y0;
      } AffineSystem;
      // Symbol: drake::systems::BarycentricMeshSystem
      struct /* BarycentricMeshSystem */ {
        // Source: drake/systems/primitives/barycentric_system.h
        const char* doc =
R"""(A (stateless) vector system implemented as a multi-linear
(barycentric) interpolation on a mesh over the inputs.

.. pydrake_system::

    name: BarycentricMeshSystem
    input_ports:
    - u0
    output_ports:
    - y0

This has many potential uses, including representing the policies that
are generated by numerical control design methods like approximate
dynamic programming.

See also:
    math∷BarycentricMesh)""";
        // Symbol: drake::systems::BarycentricMeshSystem::BarycentricMeshSystem<T>
        struct /* ctor */ {
          // Source: drake/systems/primitives/barycentric_system.h
          const char* doc =
R"""(Constructs the system from a mesh and the associated mesh values.
``output_values`` is a matrix with each column representing the value
to output if the input is the associated mesh point. It must have the
same number of columns as mesh.get_num_mesh_points();
mesh.MeshValuesFrom(function) is one useful tool for creating it.)""";
        } ctor;
        // Symbol: drake::systems::BarycentricMeshSystem::get_mesh
        struct /* get_mesh */ {
          // Source: drake/systems/primitives/barycentric_system.h
          const char* doc = R"""(Returns a reference to the mesh.)""";
        } get_mesh;
        // Symbol: drake::systems::BarycentricMeshSystem::get_output_values
        struct /* get_output_values */ {
          // Source: drake/systems/primitives/barycentric_system.h
          const char* doc =
R"""(Returns a reference to the output values.)""";
        } get_output_values;
      } BarycentricMeshSystem;
      // Symbol: drake::systems::BusCreator
      struct /* BusCreator */ {
        // Source: drake/systems/primitives/bus_creator.h
        const char* doc =
R"""(This system packs values from heterogeneous input ports into a single
output port of type BusValue.

.. pydrake_system::

    name: BusCreator
    input_ports:
    - u0
    - ...
    - u(N-1)
    output_ports:
    - y0

The port names shown in the figure above are the defaults. Custom
names may be specified when setting up the BusCreator.

When an input port is not connected, it is not an error; its value
will simply not appear as part of the BusValue on the output port.

See also:
    BusSelector, BusValue)""";
        // Symbol: drake::systems::BusCreator::BusCreator<T>
        struct /* ctor */ {
          // Source: drake/systems/primitives/bus_creator.h
          const char* doc =
R"""(Constructs a BusCreator with no inputs, and the given output port
name. Use DeclareAbstractInputPort() and DeclareVectorInputPort() to
add ports.)""";
          // Source: drake/systems/primitives/bus_creator.h
          const char* doc_copyconvert =
R"""(Scalar-converting copy constructor. See system_scalar_conversion.)""";
        } ctor;
        // Symbol: drake::systems::BusCreator::DeclareAbstractInputPort
        struct /* DeclareAbstractInputPort */ {
          // Source: drake/systems/primitives/bus_creator.h
          const char* doc =
R"""(Declares an abstract input port with the given attributes. The port
name will also be used as the name of this signal in the BusValue
output port.)""";
        } DeclareAbstractInputPort;
        // Symbol: drake::systems::BusCreator::DeclareVectorInputPort
        struct /* DeclareVectorInputPort */ {
          // Source: drake/systems/primitives/bus_creator.h
          const char* doc =
R"""(Declares a vector input port with the given attributes. The port name
will also be used as the name of this signal in the BusValue output
port. The type of the signal on the output bus will be
``BasicVector<T>``.)""";
        } DeclareVectorInputPort;
      } BusCreator;
      // Symbol: drake::systems::BusSelector
      struct /* BusSelector */ {
        // Source: drake/systems/primitives/bus_selector.h
        const char* doc =
R"""(This system unpacks values from a single input port of type BusValue
onto heterogeneous output ports, where each output port's value comes
from the same- named signal on the bus.

The value on input port may contain additional bus value entries which
are not selected (because there is no output port with that name);
this is not an error.

.. pydrake_system::

    name: BusSelector
    input_ports:
    - u0
    output_ports:
    - y0
    - ...
    - y(N-1)

The port names shown in the figure above are the defaults. Custom
names may be specified when setting up the BusSelector.

When an output port is evaluated but the input port's bus doesn't
contain that signal name, it is an error.

Because of the all-encompassing nature of AbstractValue, a
vector-valued bus signal can be output on an abstract-valued port. Of
course, the vector value can still be retrieved from the
AbstractValue. However, we recommend declaring output ports as
vector-valued when the corresponding input signal is known to be
vector-valued to maximize utility and minimize surprise.

See also:
    BusCreator, BusValue)""";
        // Symbol: drake::systems::BusSelector::BusSelector<T>
        struct /* ctor */ {
          // Source: drake/systems/primitives/bus_selector.h
          const char* doc =
R"""(Constructs a BusSelector with the given input port name, and no
outputs. Use DeclareVectorOutputPort() and DeclareAbstractOutputPort()
to add ports.)""";
          // Source: drake/systems/primitives/bus_selector.h
          const char* doc_copyconvert =
R"""(Scalar-converting copy constructor. See system_scalar_conversion.)""";
        } ctor;
        // Symbol: drake::systems::BusSelector::DeclareAbstractOutputPort
        struct /* DeclareAbstractOutputPort */ {
          // Source: drake/systems/primitives/bus_selector.h
          const char* doc =
R"""(Declares an abstract output port with the given attributes. The port
name will also be used as the name of the signal to find in the
BusValue input port.)""";
        } DeclareAbstractOutputPort;
        // Symbol: drake::systems::BusSelector::DeclareVectorOutputPort
        struct /* DeclareVectorOutputPort */ {
          // Source: drake/systems/primitives/bus_selector.h
          const char* doc =
R"""(Declares a vector output port with the given attributes. The port name
will also be used as the name of the signal to find in the BusValue
input port. The type of the signal on the input bus must be
``BasicVector<T>``.)""";
        } DeclareVectorOutputPort;
      } BusSelector;
      // Symbol: drake::systems::ConstantValueSource
      struct /* ConstantValueSource */ {
        // Source: drake/systems/primitives/constant_value_source.h
        const char* doc =
R"""(A source block that always outputs a constant value.

.. pydrake_system::

    name: ConstantValueSource
    output_ports:
    - y0)""";
        // Symbol: drake::systems::ConstantValueSource::ConstantValueSource<T>
        struct /* ctor */ {
          // Source: drake/systems/primitives/constant_value_source.h
          const char* doc =
R"""(Parameter ``value``:
    The constant value to emit, which is copied by this system.)""";
          // Source: drake/systems/primitives/constant_value_source.h
          const char* doc_copyconvert =
R"""(Scalar-converting copy constructor. See system_scalar_conversion.)""";
        } ctor;
      } ConstantValueSource;
      // Symbol: drake::systems::ConstantVectorSource
      struct /* ConstantVectorSource */ {
        // Source: drake/systems/primitives/constant_vector_source.h
        const char* doc =
R"""(A source block with a constant output port at all times. The value of
the output port is a parameter of the system (see Parameters).

.. pydrake_system::

    name: ConstantVectorSource
    output_ports:
    - y0)""";
        // Symbol: drake::systems::ConstantVectorSource::ConstantVectorSource<T>
        struct /* ctor */ {
          // Source: drake/systems/primitives/constant_vector_source.h
          const char* doc =
R"""(Constructs a system with a vector output that is constant and equals
the supplied ``source_value`` at all times.

Parameter ``source_value``:
    the constant value of the output so that ``y = source_value`` at
    all times.)""";
          // Source: drake/systems/primitives/constant_vector_source.h
          const char* doc_copyconvert =
R"""(Scalar-converting copy constructor. See system_scalar_conversion.)""";
        } ctor;
        // Symbol: drake::systems::ConstantVectorSource::get_mutable_source_value
        struct /* get_mutable_source_value */ {
          // Source: drake/systems/primitives/constant_vector_source.h
          const char* doc =
R"""(Return a mutable reference to the source value of this block in the
given ``context``.)""";
        } get_mutable_source_value;
        // Symbol: drake::systems::ConstantVectorSource::get_source_value
        struct /* get_source_value */ {
          // Source: drake/systems/primitives/constant_vector_source.h
          const char* doc =
R"""(Return a read-only reference to the source value of this block in the
given ``context``.)""";
        } get_source_value;
      } ConstantVectorSource;
      // Symbol: drake::systems::ControllabilityMatrix
      struct /* ControllabilityMatrix */ {
        // Source: drake/systems/primitives/linear_system.h
        const char* doc =
R"""(Returns the controllability matrix: R = [B, AB, ..., A^{n-1}B].)""";
      } ControllabilityMatrix;
      // Symbol: drake::systems::Demultiplexer
      struct /* Demultiplexer */ {
        // Source: drake/systems/primitives/demultiplexer.h
        const char* doc =
R"""(This system splits a vector valued signal on its input into multiple
outputs.

The input to this system directly feeds through to its output.

.. pydrake_system::

    name: Demultiplexer
    input_ports:
    - u0
    output_ports:
    - y0
    - ...
    - y(N-1))""";
        // Symbol: drake::systems::Demultiplexer::Demultiplexer<T>
        struct /* ctor */ {
          // Source: drake/systems/primitives/demultiplexer.h
          const char* doc_1args =
R"""(Constructs Demultiplexer with one vector valued output ports with
sizes specified as the vector ``output_ports_sizes``. The number of
output ports is the length of this vector. The size of each output
port is the value of the corresponding element of the vector
``output_ports_sizes``.

.. pydrake_system::

    name: Demultiplexer
    input_ports:
    - u0
    output_ports:
    - y0
    - ...
    - y(output_ports_sizes.size() - 1)

Raises:
    RuntimeError if ``output_ports_sizes`` is a zero length vector.

Raises:
    RuntimeError if any element of the ``output_ports_sizes`` is zero.
    Therefore, the Demultiplexer does not allow zero size output
    ports.

Parameter ``output_ports_sizes``:
    Contains the sizes of each output port. The number of output ports
    is determined by the length of ``output_ports_sizes``. The
    accumulative value of the all the values in ``output_ports_sizes``
    will be the size of the input port.)""";
          // Source: drake/systems/primitives/demultiplexer.h
          const char* doc_2args =
R"""(Constructs Demultiplexer with one vector valued input port of size
``size`` and vector valued output ports of size ``output_ports_size``.

.. pydrake_system::

    name: Demultiplexer
    input_ports:
    - u0
    output_ports:
    - y0
    - ...
    - y((size / output_ports_size) - 1)

Raises:
    RuntimeError if output_ports_sizes can not exactly divide
    ``size``. The number of output ports is therefore ``size /
    output_ports_size``.

Parameter ``size``:
    is the size of the input signal to be demultiplexed into its
    individual components.

Parameter ``output_ports_size``:
    The size of the output ports. ``size`` must be a multiple of
    ``output_ports_size``.)""";
          // Source: drake/systems/primitives/demultiplexer.h
          const char* doc_copyconvert =
R"""(Scalar-converting copy constructor. See system_scalar_conversion.)""";
        } ctor;
        // Symbol: drake::systems::Demultiplexer::get_output_ports_sizes
        struct /* get_output_ports_sizes */ {
          // Source: drake/systems/primitives/demultiplexer.h
          const char* doc = R"""()""";
        } get_output_ports_sizes;
      } Demultiplexer;
      // Symbol: drake::systems::DiscreteDerivative
      struct /* DiscreteDerivative */ {
        // Source: drake/systems/primitives/discrete_derivative.h
        const char* doc =
R"""(System that outputs the discrete-time derivative of its input: y(t) =
(u[n] - u[n-1])/h, where n = floor(t/h), where h is the time period.

This is implemented as the linear system


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    x₀[n+1] = u[n],
      x₁[n+1] = x₀[n],
      y(t) = (x₀[n]-x₁[n])/h.
      x₀[0] and x₁[0] are initialized in the Context (default is zeros).

.. raw:: html

    </details>

Alternatively, when ``suppress_initial_transient = true`` is passed to
the constructor, the output remains zero until u[n] has been sampled
twice.

This is implemented as the non-linear system


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    x₀[n+1] = u[n],
      x₁[n+1] = x₀[n],
      x₂[n+1] = x₂[n] + 1,
      y(t) = { 0.0              if x₂ <  2 }
             { (x₀[n]-x₁[n])/h  if x₂ >= 2 }
      x₀[0], x₁[0], x₂[0] are initialized in the Context (default is zeros).

.. raw:: html

    </details>

Note:
    Calling set_input_history() effectively disables the transient
    suppression by setting x_2 = 2.

Note:
    For dynamical systems, a derivative should not be computed in
    continuous-time, i.e. ``y(t) = (u(t) - u[n])/(t-n*h)``. This is
    numerically unstable since the time interval ``t-n*h`` could be
    arbitrarily close to zero. Prefer the discrete-time implementation
    for robustness.

.. pydrake_system::

    name: DiscreteDerivative
    input_ports:
    - u
    output_ports:
    - dudt)""";
        // Symbol: drake::systems::DiscreteDerivative::DiscreteDerivative<T>
        struct /* ctor */ {
          // Source: drake/systems/primitives/discrete_derivative.h
          const char* doc =
R"""(Constructor taking ``num_inputs``, the size of the vector to be
differentiated, and ``time_step``, the sampling interval. If
``suppress_initial_transient`` is true (the default), then the output
will be zero for the first two time steps (see the class documentation
for details and exceptions).)""";
          // Source: drake/systems/primitives/discrete_derivative.h
          const char* doc_copyconvert =
R"""(Scalar-converting copy constructor. See system_scalar_conversion.)""";
        } ctor;
        // Symbol: drake::systems::DiscreteDerivative::set_input_history
        struct /* set_input_history */ {
          // Source: drake/systems/primitives/discrete_derivative.h
          const char* doc_3args_state_u_n_u_n_minus_1 =
R"""(Sets the input history so that the initial output is fully specified.
This is useful during initialization to avoid large derivative outputs
if u[0] ≠ 0. ``u_n`` and @ u_n_minus_1 must be the same size as the
input/output ports. If suppress_initial_transient() is true, then also
sets x₂ to be >= 2 to disable the suppression for this ``state``.)""";
          // Source: drake/systems/primitives/discrete_derivative.h
          const char* doc_3args_context_u_n_u_n_minus_1 =
R"""(Sets the input history so that the initial output is fully specified.
This is useful during initialization to avoid large derivative outputs
if u[0] ≠ 0. ``u_n`` and @ u_n_minus_1 must be the same size as the
input/output ports. If suppress_initial_transient() is true, then also
sets x₂ to be >= 2 to disable the suppression for this ``context``.)""";
          // Source: drake/systems/primitives/discrete_derivative.h
          const char* doc_2args_context_u =
R"""(Convenience method that sets the entire input history to a constant
vector value (x₀ = x₁ = u,resulting in a derivative = 0). This is
useful during initialization to avoid large derivative outputs if u[0]
≠ 0. ``u`` must be the same size as the input/output ports. If
suppress_initial_transient() is true, then also sets x₂ to be >= 2 to
disable the suppression for this ``context``.)""";
        } set_input_history;
        // Symbol: drake::systems::DiscreteDerivative::suppress_initial_transient
        struct /* suppress_initial_transient */ {
          // Source: drake/systems/primitives/discrete_derivative.h
          const char* doc =
R"""(Returns the ``suppress_initial_transient`` passed to the constructor.)""";
        } suppress_initial_transient;
        // Symbol: drake::systems::DiscreteDerivative::time_step
        struct /* time_step */ {
          // Source: drake/systems/primitives/discrete_derivative.h
          const char* doc = R"""()""";
        } time_step;
      } DiscreteDerivative;
      // Symbol: drake::systems::DiscreteTimeDelay
      struct /* DiscreteTimeDelay */ {
        // Source: drake/systems/primitives/discrete_time_delay.h
        const char* doc =
R"""(A discrete time delay block with input u, which is vector-valued
(discrete or continuous) or abstract, and output delayed_u which is
previously received input, delayed by the given amount. The initial
output will be a vector of zeros for vector-valued or a given value
for abstract-valued until the delay time has passed.

.. pydrake_system::

    name: DiscreteTimeDelay
    input_ports:
    - u
    output_ports:
    - delayed_u

Let t,z ∈ ℕ be the number of delay time steps and the input vector
size. For abstract-valued DiscreteTimeDelay, z is 1. The state x ∈
ℝ⁽ᵗ⁺¹⁾ᶻ is partitioned into t+1 blocks x[0] x[1] ... x[t], each of
size z. The input and output are u,y ∈ ℝᶻ. The discrete state space
dynamics of DiscreteTimeDelay is:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    xₙ₊₁ = xₙ[1] xₙ[2] ... xₙ[t] uₙ  // update
      yₙ = xₙ[0]                       // output
      x₀ = xᵢₙᵢₜ                       // initialize

.. raw:: html

    </details>

where xᵢₙᵢₜ = 0 for vector-valued DiscreteTimeDelay and xᵢₙᵢₜ is a
given value for abstract-valued DiscreteTimeDelay.

See discrete_systems "Discrete Systems" for general information about
discrete systems in Drake, including how they interact with continuous
systems.

Note:
    While the output port can be sampled at any continuous time, this
    system does not interpolate.)""";
        // Symbol: drake::systems::DiscreteTimeDelay::DiscreteTimeDelay<T>
        struct /* ctor */ {
          // Source: drake/systems/primitives/discrete_time_delay.h
          const char* doc_3args_update_sec_delay_time_steps_vector_size =
R"""(Constructs a DiscreteTimeDelay system updating every ``update_sec``
and delaying a vector-valued input of size ``vector_size`` for
``delay_time_steps`` number of updates.)""";
          // Source: drake/systems/primitives/discrete_time_delay.h
          const char* doc_3args_update_sec_delay_time_steps_abstract_model_value =
R"""(Constructs a DiscreteTimeDelay system updating every ``update_sec``
and delaying an abstract-valued input of type ``abstract_model_value``
for ``delay_time_steps`` number of updates.)""";
          // Source: drake/systems/primitives/discrete_time_delay.h
          const char* doc_copyconvert =
R"""(Scalar-type converting copy constructor. See system_scalar_conversion.)""";
        } ctor;
        // Symbol: drake::systems::DiscreteTimeDelay::SaveInputToBuffer
        struct /* SaveInputToBuffer */ {
          // Source: drake/systems/primitives/discrete_time_delay.h
          const char* doc =
R"""((Advanced) Manually samples the input port and updates the state of
the block, sliding the delay buffer forward and placing the sampled
input at the end. This emulates an update event and is mostly useful
for testing.)""";
        } SaveInputToBuffer;
      } DiscreteTimeDelay;
      // Symbol: drake::systems::DiscreteTimeIntegrator
      struct /* DiscreteTimeIntegrator */ {
        // Source: drake/systems/primitives/discrete_time_integrator.h
        const char* doc =
R"""(A discrete-time integrator for a vector input, using explicit Euler
integration.

.. pydrake_system::

    name: DiscreteTimeIntegrator
    input_ports:
    - u
    output_ports:
    - y

The discrete state space dynamics of DiscreteTimeIntegrator with time
step ``h`` is:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    xₙ₊₁ = xₙ + h uₙ  // update
    yₙ = xₙ           // output
    x₀ = xᵢₙᵢₜ        // initialize

.. raw:: html

    </details>

where xᵢₙᵢₜ = 0 by default. Use set_integral_value() to set xₙ in the
context. The output at time ``t`` is ``xₙ`` where ``n = ceil(t/h)``.
See discrete_systems.)""";
        // Symbol: drake::systems::DiscreteTimeIntegrator::DiscreteTimeIntegrator<T>
        struct /* ctor */ {
          // Source: drake/systems/primitives/discrete_time_integrator.h
          const char* doc =
R"""(Constructs an DiscreteTimeIntegrator system.

Parameter ``size``:
    number of elements in the signal to be processed.

Parameter ``time_step``:
    the discrete time step.

Precondition:
    size > 0. time_step > 0.)""";
          // Source: drake/systems/primitives/discrete_time_integrator.h
          const char* doc_copyconvert =
R"""(Scalar-converting copy constructor. See system_scalar_conversion.)""";
        } ctor;
        // Symbol: drake::systems::DiscreteTimeIntegrator::set_integral_value
        struct /* set_integral_value */ {
          // Source: drake/systems/primitives/discrete_time_integrator.h
          const char* doc =
R"""(Sets the value of the integral modifying the state in the context.
``value`` must be a column vector of the appropriate size.)""";
        } set_integral_value;
        // Symbol: drake::systems::DiscreteTimeIntegrator::time_step
        struct /* time_step */ {
          // Source: drake/systems/primitives/discrete_time_integrator.h
          const char* doc =
R"""(Returns the time_step used by the integrator.)""";
        } time_step;
      } DiscreteTimeIntegrator;
      // Symbol: drake::systems::FirstOrderLowPassFilter
      struct /* FirstOrderLowPassFilter */ {
        // Source: drake/systems/primitives/first_order_low_pass_filter.h
        const char* doc =
R"""(An element-wise first order low pass filter system that filters the
i-th input uᵢ into the i-th output zᵢ. This system has one continuous
state per filtered input signal. Therefore, the i-th state of the
system zᵢ evolves according to:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    żᵢ = -1/τᵢ (zᵢ - uᵢ)

.. raw:: html

    </details>

where τᵢ is the time constant of the i-th filter. The i-th output of
the system is given by:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    yᵢ = zᵢ

.. raw:: html

    </details>

The transfer function for the i-th filter corresponds to:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    H(s) = 1 / (1 + τᵢ s)

.. raw:: html

    </details>

The Bode plot for the i-th filter exhibits a cutoff frequency (angular
frequency) at 1/τᵢ and a gain of one. For frequencies higher than the
cutoff frequency, the Bode plot approaches a 20 dB per decade negative
slope. The Bode plot in phase exhibits a -90 degrees shift (lag) for
frequencies much larger than the cutoff frequency and a zero shift for
low frequencies.

.. pydrake_system::

    name: FirstOrderLowPassFilter
    input_ports:
    - u0
    output_ports:
    - y0)""";
        // Symbol: drake::systems::FirstOrderLowPassFilter::FirstOrderLowPassFilter<T>
        struct /* ctor */ {
          // Source: drake/systems/primitives/first_order_low_pass_filter.h
          const char* doc_2args =
R"""(Constructs a FirstOrderLowPassFilter system that filters all input
signals with the same time constant, i.e. τᵢ = τ, ∀ i.

Parameter ``time_constant``:
    the time constant τ of the filter. It must be a positive number.

Parameter ``size``:
    number of elements in the signal to be processed.)""";
          // Source: drake/systems/primitives/first_order_low_pass_filter.h
          const char* doc_1args =
R"""(Constructs a FirstOrderLowPassFilter so that the i-th component of the
input signal vector is low pass filtered with a time constant given in
the i-th component τᵢ of the input ``time_constants`` vector.

Parameter ``time_constants``:
    Vector of time constants. Each entry in this vector must be
    positive.)""";
          // Source: drake/systems/primitives/first_order_low_pass_filter.h
          const char* doc_copyconvert =
R"""(Scalar-converting copy constructor. See system_scalar_conversion.)""";
        } ctor;
        // Symbol: drake::systems::FirstOrderLowPassFilter::get_time_constant
        struct /* get_time_constant */ {
          // Source: drake/systems/primitives/first_order_low_pass_filter.h
          const char* doc =
R"""(Returns the time constant of the filter for filters that have the same
time constant τ for all signals. This method aborts if called on
filters if with different time constants per input signal.

See also:
    get_time_constants_vector().)""";
        } get_time_constant;
        // Symbol: drake::systems::FirstOrderLowPassFilter::get_time_constants_vector
        struct /* get_time_constants_vector */ {
          // Source: drake/systems/primitives/first_order_low_pass_filter.h
          const char* doc =
R"""(Returns the vector of time constants for ``this`` filter.)""";
        } get_time_constants_vector;
        // Symbol: drake::systems::FirstOrderLowPassFilter::set_initial_output_value
        struct /* set_initial_output_value */ {
          // Source: drake/systems/primitives/first_order_low_pass_filter.h
          const char* doc =
R"""(Sets the initial conditions on the output value of the filtered
signal.

Parameter ``context``:
    The current system context.

Parameter ``z0``:
    The vector on initial conditions on the output value.)""";
        } set_initial_output_value;
      } FirstOrderLowPassFilter;
      // Symbol: drake::systems::FirstOrderTaylorApproximation
      struct /* FirstOrderTaylorApproximation */ {
        // Source: drake/systems/primitives/linear_system.h
        const char* doc =
R"""(A first-order Taylor series approximation to a ``system`` in the
neighborhood of an arbitrary point. When Taylor-expanding a system at
a non-equilibrium point, it may be represented either of the form:

.. math:: \dot{x} - \dot{x}_0 = A (x - x_0) + B (u - u_0),

for continuous time, or

.. math:: x[n+1] - x_0[n+1] = A (x[n] - x_0[n]) + B (u[n] - u_0[n]),

for discrete time. As above, we denote :math:`x_0, u_0` to be the
nominal state and input at the provided ``context``. The system
description is affine when the terms :math:`\dot{x}_0 - A x_0 - B u_0`
and :math:`x_0[n+1] - A x_0[n] - B u_0[n]` are nonzero.

More precisely, let x be a state and u be an input. This function
returns an AffineSystem of the form:

.. math:: \dot{x} = A x + B u + f_0,

(CT)

.. math:: x[n+1] = A x[n] + B u[n] + f_0,

(DT) where :math:`f_0 = \dot{x}_0 - A x_0 - B u_0` (CT) and :math:`f_0
= x_0[n+1] - A x[n] - B u[n]` (DT).

This method currently supports approximating around at most a single
vector input port and at most a single vector output port. For systems
with more ports, use ``input_port_index`` and ``output_port_index`` to
select the input for the newly constructed system. Any additional
input ports will be treated as constants (fixed at the value specified
in ``context)``.

Parameter ``system``:
    The system or subsystem to linearize.

Parameter ``context``:
    Defines the nominal operating point about which the system should
    be linearized.

Parameter ``input_port_index``:
    A valid input port index for ``system`` or InputPortSelection.
    $*Default:* kUseFirstInputIfItExists.

Parameter ``output_port_index``:
    A valid output port index for ``system`` or OutputPortSelection.
    $*Default:* kUseFirstOutputIfItExists.

Returns:
    An AffineSystem at this linearization point.

Raises:
    if any abstract inputs are connected, if any vector-valued inputs
    are unconnected, if the system is not (only) continuous or not
    (only) discrete time with a single periodic update.

Note:
    x, u and y are in the same coordinate system as the original
    ``system``, since the terms involving :math:`x_0, u_0` reside in
    :math:`f_0`.

Note:
    This method does *not* (yet) set the initial conditions (default
    nor random) of the AffineSystem based on ``system``.)""";
      } FirstOrderTaylorApproximation;
      // Symbol: drake::systems::Gain
      struct /* Gain */ {
        // Source: drake/systems/primitives/gain.h
        const char* doc =
R"""(An element-wise gain block with input ``u`` and output ``y = k * u``
with ``k`` a constant vector. The input to this system directly feeds
through to its output.

.. pydrake_system::

    name: Gain
    input_ports:
    - u0
    output_ports:
    - y0)""";
        // Symbol: drake::systems::Gain::Gain<T>
        struct /* ctor */ {
          // Source: drake/systems/primitives/gain.h
          const char* doc_2args =
R"""(Constructs a Gain system where the same gain is applied to every input
value.

Parameter ``k``:
    the gain constant so that ``y = k * u``.

Parameter ``size``:
    number of elements in the signal to be processed.)""";
          // Source: drake/systems/primitives/gain.h
          const char* doc_1args =
R"""(Constructs a Gain system where different gains can be applied to each
input value.

Parameter ``k``:
    the gain vector constants so that ``y_i = k_i * u_i`` where
    subscript ``i`` indicates the i-th element of the vector.)""";
          // Source: drake/systems/primitives/gain.h
          const char* doc_copyconvert =
R"""(Scalar-converting copy constructor. See system_scalar_conversion.)""";
        } ctor;
        // Symbol: drake::systems::Gain::get_gain
        struct /* get_gain */ {
          // Source: drake/systems/primitives/gain.h
          const char* doc =
R"""(Returns the gain constant. This method should only be called if the
gain can be represented as a scalar value, i.e., every element in the
gain vector is the same. It will throw an exception if the gain cannot
be represented as a single scalar value.)""";
        } get_gain;
        // Symbol: drake::systems::Gain::get_gain_vector
        struct /* get_gain_vector */ {
          // Source: drake/systems/primitives/gain.h
          const char* doc = R"""(Returns the gain vector constant.)""";
        } get_gain_vector;
      } Gain;
      // Symbol: drake::systems::Integrator
      struct /* Integrator */ {
        // Source: drake/systems/primitives/integrator.h
        const char* doc =
R"""(A continuous-time integrator for a vector input.

.. pydrake_system::

    name: Integrator
    input_ports:
    - u0
    output_ports:
    - y0)""";
        // Symbol: drake::systems::Integrator::Integrator<T>
        struct /* ctor */ {
          // Source: drake/systems/primitives/integrator.h
          const char* doc_1args_size =
R"""(Constructs an Integrator system. The initial output value will be
zero.

Parameter ``size``:
    number of elements in the signal to be processed.)""";
          // Source: drake/systems/primitives/integrator.h
          const char* doc_1args_initial_value =
R"""(Constructs an Integrator system with a particular initial output
value. The size of both input and output are inferred from the given
``initial_value``.

Parameter ``initial_value``:
    the initial output value.)""";
          // Source: drake/systems/primitives/integrator.h
          const char* doc_copyconvert =
R"""(Scalar-converting copy constructor. See system_scalar_conversion.)""";
        } ctor;
        // Symbol: drake::systems::Integrator::set_default_integral_value
        struct /* set_default_integral_value */ {
          // Source: drake/systems/primitives/integrator.h
          const char* doc =
R"""(Sets the initial value of the integral state variable that will be
present in a subsequently-created Context. Overrides any value that
was provided in the constructor or previous calls to this function.
However, the size of ``value`` must be the same as the size indicated
at construction.

See also:
    set_integral_value() to set the initial value in an
    already-allocated Context.

Raises:
    RuntimeError if an attempt is made to change the state size.)""";
        } set_default_integral_value;
        // Symbol: drake::systems::Integrator::set_integral_value
        struct /* set_integral_value */ {
          // Source: drake/systems/primitives/integrator.h
          const char* doc =
R"""(Sets the value of the integral modifying the state in the context.
``value`` must be a column vector of the appropriate size.

See also:
    set_default_integral_value() to set the initial value that will be
    present in a newly-allocated Context.

Raises:
    RuntimeError if an attempt is made to change the state size.)""";
        } set_integral_value;
      } Integrator;
      // Symbol: drake::systems::IsControllable
      struct /* IsControllable */ {
        // Source: drake/systems/primitives/linear_system.h
        const char* doc =
R"""(Returns true iff the controllability matrix is full row rank.)""";
      } IsControllable;
      // Symbol: drake::systems::IsDetectable
      struct /* IsDetectable */ {
        // Source: drake/systems/primitives/linear_system.h
        const char* doc =
R"""(Returns true iff the system is detectable.)""";
      } IsDetectable;
      // Symbol: drake::systems::IsObservable
      struct /* IsObservable */ {
        // Source: drake/systems/primitives/linear_system.h
        const char* doc =
R"""(Returns true iff the observability matrix is full column rank.)""";
      } IsObservable;
      // Symbol: drake::systems::IsStabilizable
      struct /* IsStabilizable */ {
        // Source: drake/systems/primitives/linear_system.h
        const char* doc =
R"""(Returns true iff the system is stabilizable.)""";
      } IsStabilizable;
      // Symbol: drake::systems::LinearSystem
      struct /* LinearSystem */ {
        // Source: drake/systems/primitives/linear_system.h
        const char* doc =
R"""(A discrete OR continuous linear system.

.. pydrake_system::

    name: LinearSystem
    input_ports:
    - u0
    output_ports:
    - y0

If time_period>0.0, then the linear system will have the following
discrete- time state update:

.. math:: x[n+1] = A x[n] + B u[n],

or if time_period==0.0, then the linear system will have the following
continuous-time state update:

.. math:: \dot{x} = A x + B u.

In both cases, the system will have the output:

.. math:: y = C x + D u,

where ``u`` denotes the input vector, ``x`` denotes the state vector,
and ``y`` denotes the output vector.

See also:
    AffineSystem

See also:
    MatrixGain)""";
        // Symbol: drake::systems::LinearSystem::LinearSystem<T>
        struct /* ctor */ {
          // Source: drake/systems/primitives/linear_system.h
          const char* doc_5args =
R"""(Constructs a LinearSystem with a fixed set of coefficient matrices
``A``, `B`,``C``, and ``D``. The coefficient matrices must obey the
following dimensions: | Matrix | Num Rows | Num Columns |
|:-------:|:-----------:|:-----------:| | A | num states | num states
| | B | num states | num inputs | | C | num outputs | num states | | D
| num outputs | num inputs |

Empty matrices are treated as zero matrices with the appropriate
number of rows and columns.

Subclasses must use the protected constructor, not this one.)""";
          // Source: drake/systems/primitives/linear_system.h
          const char* doc_copyconvert =
R"""(Scalar-converting copy constructor. See system_scalar_conversion.)""";
          // Source: drake/systems/primitives/linear_system.h
          const char* doc_6args =
R"""(Constructor that specifies scalar-type conversion support.

Parameter ``converter``:
    scalar-type conversion support helper (i.e., AutoDiff, etc.); pass
    a default-constructed object if such support is not desired. See
    system_scalar_conversion for detailed background and examples
    related to scalar-type conversion support.)""";
        } ctor;
        // Symbol: drake::systems::LinearSystem::MakeLinearSystem
        struct /* MakeLinearSystem */ {
          // Source: drake/systems/primitives/linear_system.h
          const char* doc =
R"""(Creates a unique pointer to LinearSystem<T> by decomposing
``dynamics`` and ``outputs`` using ``state_vars`` and ``input_vars``.

Raises:
    RuntimeError if either ``dynamics`` or ``outputs`` is not linear
    in ``state_vars`` and ``input_vars``.)""";
        } MakeLinearSystem;
      } LinearSystem;
      // Symbol: drake::systems::LinearTransformDensity
      struct /* LinearTransformDensity */ {
        // Source: drake/systems/primitives/linear_transform_density.h
        const char* doc =
R"""(Performs linear transformation on the random signal w_in as w_out =
A*w_in + b. The user can obtain the probability density of w_out. When
the class is instantiated with autodiff scalar, the user can also
obtain the gradient of the probability density of w_out.

.. pydrake_system::

    name: LinearTransformDensity
    input_ports:
    - w_in
    - A
    - b
    output_ports:
    - w_out
    - w_out_density

The ``b`` port can remain disconnected, in which case it defaults to
zero.

``A`` should be a matrix using a column-major order.

The user should make sure that the input port ``w_in`` is connected
from the output port of a RandomSource with the same distribution. A
recommended way is to use ``AddRandomInputs()``.

Warning:
    The code cannot verify that the distribution type of w_in matches
    between where w_in comes from and the w_in input port of this
    system. This class will quietly produce incorrect behavior if the
    distribution types don't match.

See also:
    stochastic_systems)""";
        // Symbol: drake::systems::LinearTransformDensity::CalcDensity
        struct /* CalcDensity */ {
          // Source: drake/systems/primitives/linear_transform_density.h
          const char* doc =
R"""(Compute the density (pdf) of a sampled output w_out.

When T=AutoDiffXd, this function computes the gradient of the function
density(w_out_sample). Namely given an output sample, we want to know
how the probability of drawing this sample would change, when the
parameters of the distribution (like A and b) change. Such information
is locally expressed in the gradient. Note this is different from
computing the density of the input.

Raises:
    RuntimeError if A is not an invertible matrix.)""";
        } CalcDensity;
        // Symbol: drake::systems::LinearTransformDensity::FixConstantA
        struct /* FixConstantA */ {
          // Source: drake/systems/primitives/linear_transform_density.h
          const char* doc =
R"""(Fix the input port ``A`` to a constant value in a given context.

Parameter ``context``:
    The context into which A's value is set.

Parameter ``A``:
    The value to which the port is fixed. The matrix A has num_output
    rows and num_input columns, note that A is column-majored.)""";
        } FixConstantA;
        // Symbol: drake::systems::LinearTransformDensity::FixConstantB
        struct /* FixConstantB */ {
          // Source: drake/systems/primitives/linear_transform_density.h
          const char* doc =
R"""(Fix the input port ``b`` to a constant value in a given context.

Parameter ``context``:
    The context into which b's value is set.

Parameter ``b``:
    The value to which the port is fixed. The vector b has num_output
    rows.)""";
        } FixConstantB;
        // Symbol: drake::systems::LinearTransformDensity::LinearTransformDensity<T>
        struct /* ctor */ {
          // Source: drake/systems/primitives/linear_transform_density.h
          const char* doc =
R"""(Parameter ``distribution``:
    The random input w_in should satisfy this distribution.

Parameter ``input_size``:
    The dimension of the input w_in.

Parameter ``output_size``:
    The dimension of the output w_out.

Note:
    The matrix A will have ``output_size`` columns and ``input_size``
    rows. The vector b will have ``output_size`` columns.)""";
          // Source: drake/systems/primitives/linear_transform_density.h
          const char* doc_copyconvert =
R"""(Scalar-converting copy constructor. See system_scalar_conversion.)""";
        } ctor;
        // Symbol: drake::systems::LinearTransformDensity::get_distribution
        struct /* get_distribution */ {
          // Source: drake/systems/primitives/linear_transform_density.h
          const char* doc = R"""(Gets the random distribution type.)""";
        } get_distribution;
        // Symbol: drake::systems::LinearTransformDensity::get_input_port_A
        struct /* get_input_port_A */ {
          // Source: drake/systems/primitives/linear_transform_density.h
          const char* doc = R"""(Gets the input port for A.)""";
        } get_input_port_A;
        // Symbol: drake::systems::LinearTransformDensity::get_input_port_b
        struct /* get_input_port_b */ {
          // Source: drake/systems/primitives/linear_transform_density.h
          const char* doc = R"""(Gets the input port for b.)""";
        } get_input_port_b;
        // Symbol: drake::systems::LinearTransformDensity::get_input_port_w_in
        struct /* get_input_port_w_in */ {
          // Source: drake/systems/primitives/linear_transform_density.h
          const char* doc = R"""(Gets the input port for w_in.)""";
        } get_input_port_w_in;
        // Symbol: drake::systems::LinearTransformDensity::get_output_port_w_out
        struct /* get_output_port_w_out */ {
          // Source: drake/systems/primitives/linear_transform_density.h
          const char* doc = R"""()""";
        } get_output_port_w_out;
        // Symbol: drake::systems::LinearTransformDensity::get_output_port_w_out_density
        struct /* get_output_port_w_out_density */ {
          // Source: drake/systems/primitives/linear_transform_density.h
          const char* doc = R"""()""";
        } get_output_port_w_out_density;
      } LinearTransformDensity;
      // Symbol: drake::systems::Linearize
      struct /* Linearize */ {
        // Source: drake/systems/primitives/linear_system.h
        const char* doc =
R"""(Takes the first-order Taylor expansion of a System around a nominal
operating point (defined by the Context).

This method currently supports linearizing around at most a single
vector input port and at most a single vector output port. For systems
with more ports, use ``input_port_index`` and ``output_port_index`` to
select the input for the newly constructed system. Any additional
*vector* input ports will be treated as constants (fixed at the value
specified in ``context``). Abstract-valued input ports must be
unconnected (i.e., the system must treat the port as optional and it
must be unused).

Parameter ``system``:
    The system or subsystem to linearize.

Parameter ``context``:
    Defines the nominal operating point about which the system should
    be linearized. See note below.

Parameter ``input_port_index``:
    A valid input port index for ``system`` or InputPortSelection. All
    other inputs are assumed to be fixed to the value described by the
    ``context``. $*Default:* kUseFirstInputIfItExists.

Parameter ``output_port_index``:
    A valid output port index for ``system`` or an
    OutputPortSelection. $*Default:* kUseFirstOutputIfItExists.

Parameter ``equilibrium_check_tolerance``:
    Specifies the tolerance on ensuring that the derivative vector
    isZero at the nominal operating point. $*Default:* 1e-6.

Returns:
    A LinearSystem that approximates the original system in the
    vicinity of the operating point. See note below.

Raises:
    RuntimeError if the operating point is not an equilibrium point of
    the system (within the specified tolerance)

Raises:
    RuntimeError if the system is not (only) continuous or (only)
    discrete time with a single periodic update.

Note:
    All *vector* inputs in the system must be connected, either to the
    output of some upstream System within a Diagram (e.g., if system
    is a reference to a subsystem in a Diagram), or to a constant
    value using, e.g. ``port.FixValue(context, default_input)``. Any
    *abstract* inputs in the system must be unconnected (the port must
    be both optional and unused).

Note:
    The inputs, states, and outputs of the returned system are NOT the
    same as the original system. Denote x0,u0 as the nominal state and
    input defined by the Context, and y0 as the value of the output at
    (x0,u0), then the created systems inputs are (u-u0), states are
    (x-x0), and outputs are (y-y0).

Note:
    This method does *not* (yet) set the initial conditions (default
    nor random) of the LinearSystem based on ``system``.)""";
      } Linearize;
      // Symbol: drake::systems::LogVectorOutput
      struct /* LogVectorOutput */ {
        // Source: drake/systems/primitives/vector_log_sink.h
        const char* doc_3args =
R"""(LogVectorOutput provides a convenience function for adding a
VectorLogSink, initialized to the correct size, and connected to an
output in a DiagramBuilder. This overload supports the default set of
publish triggers. See vector_log_sink_default_triggers "default
triggers description".

Parameter ``src``:
    the output port to attach logging to.

Parameter ``builder``:
    the diagram builder.

Parameter ``publish_period``:
    Period that messages will be published (optional). If the publish
    period is zero, VectorLogSink will use per-step publishing
    instead; see LeafSystem∷DeclarePerStepPublishEvent().

Precondition:
    publish_period is non-negative.)""";
        // Source: drake/systems/primitives/vector_log_sink.h
        const char* doc_4args =
R"""(LogVectorOutput provides a convenience function for adding a
VectorLogSink, initialized to the correct size, and connected to an
output in a DiagramBuilder. This overload supports the full variety of
publish trigger behavior.

Parameter ``src``:
    the output port to attach logging to.

Parameter ``builder``:
    the diagram builder.

Parameter ``publish_triggers``:
    Set of triggers that determine when messages will be published.
    Supported TriggerTypes are {kForced, kPeriodic, kPerStep}. Will
    throw an error if empty or if unsupported types are provided.

Parameter ``publish_period``:
    Period that messages will be published (optional). publish_period
    should only be non-zero if one of the publish_triggers is
    kPeriodic.

Precondition:
    publish_period is non-negative.

Precondition:
    publish_period > 0 if and only if publish_triggers contains
    kPeriodic.)""";
      } LogVectorOutput;
      // Symbol: drake::systems::MatrixGain
      struct /* MatrixGain */ {
        // Source: drake/systems/primitives/matrix_gain.h
        const char* doc =
R"""(A system that specializes LinearSystem by setting coefficient matrices
``A``, `B`, and ``C`` to all be zero. Thus, the only non-zero
coefficient matrix is ``D``. Specifically, given an input signal ``u``
and a state ``x``, the output of this system, ``y``, is:

.. math:: y = D u

.. pydrake_system::

name: MatrixGain input_ports: - u0 output_ports: - y0

See also:
    AffineSystem

See also:
    LinearSystem)""";
        // Symbol: drake::systems::MatrixGain::MatrixGain<T>
        struct /* ctor */ {
          // Source: drake/systems/primitives/matrix_gain.h
          const char* doc_1args_size =
R"""(A constructor where the gain matrix ``D`` is a square identity matrix
of size ``size``.)""";
          // Source: drake/systems/primitives/matrix_gain.h
          const char* doc_1args_D =
R"""(A constructor where the gain matrix ``D`` is ``D``.)""";
          // Source: drake/systems/primitives/matrix_gain.h
          const char* doc_copyconvert =
R"""(Scalar-converting copy constructor. See system_scalar_conversion.)""";
        } ctor;
      } MatrixGain;
      // Symbol: drake::systems::MultilayerPerceptron
      struct /* MultilayerPerceptron */ {
        // Source: drake/systems/primitives/multilayer_perceptron.h
        const char* doc =
R"""(The MultilayerPerceptron (MLP) is one of the most common forms of
neural networks used in reinforcement learning (RL) today. This
implementation provides a System interface to distinguish between the
network's inputs and outputs (via ports), and the parameters, which
are stored in the Context.

Each layer of the network is implemented as xₙ₊₁ = σ(Wₙxₙ+bₙ), where
xₙ is the output of the preceding layer, W are the weights, b are the
biases, and σ() is the activation function. We additionally use the
shorthand x to denote the input layer and y to denote the output
layer: y=xₘ for an m-layer network.

Note: For very large-scale neural network implementations, consider
using a GPU-accelerated machine learning library like PyTorch,
TensorFlow, or JAX. But most MLPs used in controls / RL are actually
quite small. For those networks, the cost of transferring
values/gradients from Drake to e.g. PyTorch is likely not worth the
benefits. Another possible workflow might be to train a network in
PyTorch, but then to copy the weights into an instance of this class
for simulation.

.. pydrake_system::

    name: MultilayerPerceptron
    input_ports:
    - x
    output_ports:
    - y)""";
        // Symbol: drake::systems::MultilayerPerceptron::Backpropagation
        struct /* Backpropagation */ {
          // Source: drake/systems/primitives/multilayer_perceptron.h
          const char* doc =
R"""(Implements the Backpropagation algorithm for the MLP to compute the
gradients of a scalar loss function with respect to the network
parameters.

Note: The class uses the System Cache to minimize the number of
dynamic memory allocations for repeated calls to this function with
the same sized ``X``. Changing the batch size between calls to this
method or BatchOutput requires memory allocations.

Parameter ``X``:
    is a batch input, with one input per column.

Parameter ``loss``:
    is a scalar loss function, where ``Y`` is the columnwise batch
    output of the network. It should return the scalar loss and set
    ``dloss_dY``, the derivatives of the loss with respect to ``Y``,
    which is pre-allocated to be the same size as ``Y``.

Parameter ``dloss_dparams``:
    are the gradients computed. We take the storage as an input
    argument to avoid memory allocations inside the algorithm.

Returns:
    the calculated loss.

Note: It is expected that this algorithm will be used with T=double.
It uses analytical gradients; AutoDiffXd is not required.)""";
        } Backpropagation;
        // Symbol: drake::systems::MultilayerPerceptron::BackpropagationMeanSquaredError
        struct /* BackpropagationMeanSquaredError */ {
          // Source: drake/systems/primitives/multilayer_perceptron.h
          const char* doc =
R"""(Calls Backpropagation with the mean-squared error loss function: loss
= 1/N ∑ᵢ |yᵢ−yᵢᵈ|², where yᵈ is the desired values for y. See
Backpropagation for details.)""";
        } BackpropagationMeanSquaredError;
        // Symbol: drake::systems::MultilayerPerceptron::BatchOutput
        struct /* BatchOutput */ {
          // Source: drake/systems/primitives/multilayer_perceptron.h
          const char* doc =
R"""(Evaluates the batch output for the MLP with a batch input vector. Each
column of ``X`` represents an input, and each column of ``Y`` will be
assigned the corresponding output.

If the output layer of the network has size 1 (scalar output), and
``dYdX != nullptr``, then ``dYdX`` is populated with the batch
gradients of the scalar output ``Y`` relative to the input ``X``: the
(i,j)th element represents the gradient dY(0,j) / dX(i,j).

Note: In python, use numpy.asfortranarray() to allocate the writeable
matrices ``Y`` and (if needed) ``dYdX``.

This methods shares the cache with Backpropagation. If the size of X
changes here or in Backpropagation, it may force dynamic memory
allocations.

Raises:
    RuntimeError if dYdX != nullptr and the network has more than one
    output.)""";
        } BatchOutput;
        // Symbol: drake::systems::MultilayerPerceptron::GetBiases
        struct /* GetBiases */ {
          // Source: drake/systems/primitives/multilayer_perceptron.h
          const char* doc_context =
R"""(Returns the biases used in the mapping from ``layer`` to ``layer+1``.)""";
          // Source: drake/systems/primitives/multilayer_perceptron.h
          const char* doc_vector =
R"""(Returns the biases in ``params`` used in the mapping from ``layer`` to
``layer+1``.)""";
        } GetBiases;
        // Symbol: drake::systems::MultilayerPerceptron::GetMutableParameters
        struct /* GetMutableParameters */ {
          // Source: drake/systems/primitives/multilayer_perceptron.h
          const char* doc =
R"""(Returns a mutable reference to all of the parameters (weights and
biases) as a single vector.)""";
        } GetMutableParameters;
        // Symbol: drake::systems::MultilayerPerceptron::GetParameters
        struct /* GetParameters */ {
          // Source: drake/systems/primitives/multilayer_perceptron.h
          const char* doc =
R"""(Returns a reference to all of the parameters (weights and biases) as a
single vector. Use GetWeights and GetBiases to extract the components.)""";
        } GetParameters;
        // Symbol: drake::systems::MultilayerPerceptron::GetWeights
        struct /* GetWeights */ {
          // Source: drake/systems/primitives/multilayer_perceptron.h
          const char* doc_context =
R"""(Returns the weights used in the mapping from ``layer`` to ``layer+1``.)""";
          // Source: drake/systems/primitives/multilayer_perceptron.h
          const char* doc_vector =
R"""(Returns the weights in ``params`` used in the mapping from ``layer``
to ``layer+1``.)""";
        } GetWeights;
        // Symbol: drake::systems::MultilayerPerceptron::MultilayerPerceptron<T>
        struct /* ctor */ {
          // Source: drake/systems/primitives/multilayer_perceptron.h
          const char* doc_single_activation =
R"""(Constructs the MLP with the same activation type for every layer
(except the output).

Parameter ``layers``:
    is the number of elements in each layer of the network (the
    activation function does *not* count as an additional layer). The
    first element specifies the number of inputs, and the last layer
    specifies the number of outputs.

Parameter ``activation_type``:
    specifies an activation function, σ(), used in *every* hidden
    layer of the network. kIdentity will be used for the output.)""";
          // Source: drake/systems/primitives/multilayer_perceptron.h
          const char* doc_vector_activation =
R"""(Constructs the MLP with an activation_type specified for each
non-input layer.

Parameter ``layers``:
    is the number of elements in each layer of the network (the
    activation function does *not* count as an additional layer). The
    first element specifies the number of inputs, and the last layer
    specifies the number of outputs.

Parameter ``activation_types``:
    specifies the activation function, σ(), used in *each* non-input
    layer of the network (including the last layer).

``activation_types`` should have one less element than ``layers``.)""";
          // Source: drake/systems/primitives/multilayer_perceptron.h
          const char* doc_sin_cos_features =
R"""(Constructs the MLP with an additional option to transform the input
vector so that the function is periodic in 2π.

For instance, for a rotary joint on a robot, this could be used to
apply the transform [x, y] => [sin x, cos x, y]. This would be
accomplished by passing ``use_sin_cos_for_input = [true, false]``.

Note that when this transformation is applied, ``num_inputs() !=
layers()[0]``. `num_inputs() == 2 != layers()[0] == 3`.

Parameter ``use_sin_cos_for_input``:
    is a boolean vector that determines whether the sin/cos transform
    is applied to each element.

Parameter ``remaining_layers``:
    is the number of elements in each layer of the network (the
    activation function does *not* count as an additional layer). The
    first element specifies the size of the first hidden layer, and
    the last layer specifies the number of outputs.

Parameter ``activation_types``:
    specifies the activation function, σ(), used in *each* non-input
    layer of the network (including the last layer).

``activation_types`` should have the same number of elements as
``remaining_layers``.)""";
          // Source: drake/systems/primitives/multilayer_perceptron.h
          const char* doc_copyconvert =
R"""(Scalar-converting copy constructor. See system_scalar_conversion.)""";
        } ctor;
        // Symbol: drake::systems::MultilayerPerceptron::ScalarLossFunction
        struct /* ScalarLossFunction */ {
          // Source: drake/systems/primitives/multilayer_perceptron.h
          const char* doc =
R"""(Helper function signature for Backpropagation().)""";
        } ScalarLossFunction;
        // Symbol: drake::systems::MultilayerPerceptron::SetBiases
        struct /* SetBiases */ {
          // Source: drake/systems/primitives/multilayer_perceptron.h
          const char* doc_context =
R"""(Sets the biases in the ``context`` used in the mapping from ``layer``
to ``layer+1``.)""";
          // Source: drake/systems/primitives/multilayer_perceptron.h
          const char* doc_vector =
R"""(Sets the biases in ``params`` used in the mapping from ``layer`` to
``layer+1``.)""";
        } SetBiases;
        // Symbol: drake::systems::MultilayerPerceptron::SetParameters
        struct /* SetParameters */ {
          // Source: drake/systems/primitives/multilayer_perceptron.h
          const char* doc =
R"""(Sets all of the parameters in the network (weights and biases) using a
single vector. Use SetWeights and SetBiases to extract the components.)""";
        } SetParameters;
        // Symbol: drake::systems::MultilayerPerceptron::SetRandomParameters
        struct /* SetRandomParameters */ {
          // Source: drake/systems/primitives/multilayer_perceptron.h
          const char* doc =
R"""(Sets all of the parameters (all weights and biases) in the
``parameters`` using the "LeCun initialization": a uniform
distribution with mean zero and standard deviation √(1/m), where m is
the number of connections feeding into the corresponding node. See eq.
(16) in http://yann.lecun.com/exdb/publis/pdf/lecun-98b.pdf .

This is typically called via System<T>∷SetRandomContext. By contrast,
System<T>∷SetDefaultContext will set all weights and biases to zero.)""";
        } SetRandomParameters;
        // Symbol: drake::systems::MultilayerPerceptron::SetWeights
        struct /* SetWeights */ {
          // Source: drake/systems/primitives/multilayer_perceptron.h
          const char* doc_context =
R"""(Sets the weights in the ``context`` used in the mapping from ``layer``
to ``layer+1``.)""";
          // Source: drake/systems/primitives/multilayer_perceptron.h
          const char* doc_vector =
R"""(Sets the weights in ``params`` used in the mapping from ``layer`` to
``layer+1``.)""";
        } SetWeights;
        // Symbol: drake::systems::MultilayerPerceptron::activation_type
        struct /* activation_type */ {
          // Source: drake/systems/primitives/multilayer_perceptron.h
          const char* doc =
R"""(Returns the type of the activation function, σ(), used in the MLP.)""";
        } activation_type;
        // Symbol: drake::systems::MultilayerPerceptron::layers
        struct /* layers */ {
          // Source: drake/systems/primitives/multilayer_perceptron.h
          const char* doc =
R"""(Returns the number of elements in each layer of the network.)""";
        } layers;
        // Symbol: drake::systems::MultilayerPerceptron::num_parameters
        struct /* num_parameters */ {
          // Source: drake/systems/primitives/multilayer_perceptron.h
          const char* doc =
R"""(Returns the total number of parameters in the network, including all
weights and biases.)""";
        } num_parameters;
      } MultilayerPerceptron;
      // Symbol: drake::systems::Multiplexer
      struct /* Multiplexer */ {
        // Source: drake/systems/primitives/multiplexer.h
        const char* doc =
R"""(This system combines multiple vector-valued inputs into a
vector-valued output. The input to this system directly feeds through
to its output.

.. pydrake_system::

    name: Multiplexer
    input_ports:
    - u0
    - ...
    - u(N-1)
    output_ports:
    - y0)""";
        // Symbol: drake::systems::Multiplexer::Multiplexer<T>
        struct /* ctor */ {
          // Source: drake/systems/primitives/multiplexer.h
          const char* doc_1args_num_scalar_inputs =
R"""(Constructs a Multiplexer with ``num_scalar_inputs`` scalar-valued
input ports, and one vector-valued output port of size
``num_scalar_inputs``.

.. pydrake_system::

    name: Multiplexer
    input_ports:
    - u0
    - ...
    - u(num_scalar_inputs - 1)
    output_ports:
    - y0)""";
          // Source: drake/systems/primitives/multiplexer.h
          const char* doc_1args_input_sizes =
R"""(Constructs a Multiplexer with ``input_sizes.size()`` vector-valued
input ports where the i-th input has size ``input_sizes[i]``, and one
vector- valued output port of size ``sum(input_sizes)``.

.. pydrake_system::

    name: Multiplexer
    input_ports:
    - u0
    - ...
    - u(input_sizes.size() - 1)
    output_ports:
    - y0)""";
          // Source: drake/systems/primitives/multiplexer.h
          const char* doc_1args_model_vector =
R"""(Constructs a Multiplexer with model_vector.size() scalar-valued inputs
and one vector-valued output port whose size equals the size of
``model_vector``. In addition, the output type derives from that of
``model_vector``.

Note:
    Objects created using this constructor overload do not support
    system scalar conversion. See system_scalar_conversion.)""";
          // Source: drake/systems/primitives/multiplexer.h
          const char* doc_copyconvert =
R"""(Scalar-converting copy constructor. See system_scalar_conversion.)""";
        } ctor;
      } Multiplexer;
      // Symbol: drake::systems::ObservabilityMatrix
      struct /* ObservabilityMatrix */ {
        // Source: drake/systems/primitives/linear_system.h
        const char* doc =
R"""(Returns the observability matrix: O = [ C; CA; ...; CA^{n-1} ].)""";
      } ObservabilityMatrix;
      // Symbol: drake::systems::PassThrough
      struct /* PassThrough */ {
        // Source: drake/systems/primitives/pass_through.h
        const char* doc =
R"""(A pass through system with input ``u`` and output ``y = u``. This is
mathematically equivalent to a Gain system with its gain equal to one.
However this system incurs no computational cost. The input to this
system directly feeds through to its output.

The system can also be used to provide default values for a port in
any diagram. If the input port does not have a value, then the default
value passed in the constructor is passed to the output.

This system is used, for instance, in PidController which is a Diagram
composed of simple framework primitives. In this case a PassThrough is
used to connect the exported input of the Diagram to the inputs of the
Gain systems for the proportional and integral constants of the
controller. This is necessary to provide an output port to which the
internal Gain subsystems connect. In this case the PassThrough is
effectively creating an output port that feeds through the input to
the Diagram and that can now be connected to the inputs of the inner
subsystems to the Diagram. A detailed discussion of the PidController
can be found at https://github.com/RobotLocomotion/drake/pull/3132.

.. pydrake_system::

    name: PassThrough
    input_ports:
    - u
    output_ports:
    - y)""";
        // Symbol: drake::systems::PassThrough::PassThrough<T>
        struct /* ctor */ {
          // Source: drake/systems/primitives/pass_through.h
          const char* doc_1args_vector_size =
R"""(Constructs a pass-through system.

Parameter ``vector_size``:
    number of elements in the signal to be processed. When no input is
    connected, the output will be a vector of all zeros.)""";
          // Source: drake/systems/primitives/pass_through.h
          const char* doc_1args_value =
R"""(Constructs a pass-through system with vector-valued input/output
ports.

Parameter ``value``:
    The model value, which defines the size of the ports and serves as
    the default when no input is connected.)""";
          // Source: drake/systems/primitives/pass_through.h
          const char* doc_1args_abstract_model_value =
R"""(Constructs a pass-through system with abstract-valued input/output
ports.

Parameter ``abstract_model_value``:
    A model value, which defines the type of the ports and serves as
    the default when no input is connected.)""";
          // Source: drake/systems/primitives/pass_through.h
          const char* doc_copyconvert =
R"""(Scalar-type converting copy constructor. See system_scalar_conversion.)""";
        } ctor;
        // Symbol: drake::systems::PassThrough::get_input_port
        struct /* get_input_port */ {
          // Source: drake/systems/primitives/pass_through.h
          const char* doc = R"""(Returns the sole input port.)""";
        } get_input_port;
      } PassThrough;
      // Symbol: drake::systems::PerceptronActivationType
      struct /* PerceptronActivationType */ {
        // Source: drake/systems/primitives/multilayer_perceptron.h
        const char* doc =
R"""(Specifies one of the common activation functions in a neural network.)""";
        // Symbol: drake::systems::PerceptronActivationType::kIdentity
        struct /* kIdentity */ {
          // Source: drake/systems/primitives/multilayer_perceptron.h
          const char* doc = R"""()""";
        } kIdentity;
        // Symbol: drake::systems::PerceptronActivationType::kReLU
        struct /* kReLU */ {
          // Source: drake/systems/primitives/multilayer_perceptron.h
          const char* doc = R"""()""";
        } kReLU;
        // Symbol: drake::systems::PerceptronActivationType::kTanh
        struct /* kTanh */ {
          // Source: drake/systems/primitives/multilayer_perceptron.h
          const char* doc = R"""()""";
        } kTanh;
      } PerceptronActivationType;
      // Symbol: drake::systems::PortSwitch
      struct /* PortSwitch */ {
        // Source: drake/systems/primitives/port_switch.h
        const char* doc =
R"""(A simple system that passes through the value from just one of its
input ports to the output. All inputs (except for the port_selector)
must have the same data type as the output.

This system only evaluates the port_selector port and the input port
that is indicated by port_selector at runtime. Because of the System
framework's "pull architecture", this means that entire sub-diagrams
can potentially be added with minimal runtime cost (their outputs will
not be evaluated until they are selected). Just remember that their
state dynamics *will* still be evaluated when the diagram's dynamics
are evaluated (e.g. during simulation), and their output ports could
be evaluated via other connections.

.. pydrake_system::

    name: PortSwitch
    input_ports:
    - port_selector
    - (user assigned port name)
    - ...
    - (user assigned port name)
    output_ports:
    - value)""";
        // Symbol: drake::systems::PortSwitch::DeclareInputPort
        struct /* DeclareInputPort */ {
          // Source: drake/systems/primitives/port_switch.h
          const char* doc =
R"""(Declares a new input port to the switch with port name ``name``. The
type of this port is already defined by the type of the output port.
This must be called before any Context is allocated.)""";
        } DeclareInputPort;
        // Symbol: drake::systems::PortSwitch::PortSwitch<T>
        struct /* ctor */ {
          // Source: drake/systems/primitives/port_switch.h
          const char* doc =
R"""(Constructs a vector-valued PortSwitch. All input ports declared via
DeclareInputPort() will be vector-valued ports of size
``vector_size``, which must be greater than zero.)""";
          // Source: drake/systems/primitives/port_switch.h
          const char* doc_copyconvert =
R"""(Scalar-type converting copy constructor. See system_scalar_conversion.)""";
        } ctor;
        // Symbol: drake::systems::PortSwitch::get_port_selector_input_port
        struct /* get_port_selector_input_port */ {
          // Source: drake/systems/primitives/port_switch.h
          const char* doc =
R"""(Returns the port-selector input port, which is an abstract-valued port
of type InputPortIndex.)""";
        } get_port_selector_input_port;
      } PortSwitch;
      // Symbol: drake::systems::RandomSource
      struct /* RandomSource */ {
        // Source: drake/systems/primitives/random_source.h
        const char* doc =
R"""(A source block which generates random numbers at a fixed sampling
interval, with a zero-order hold between samples. For continuous-time
systems, this can be interpreted as a band-limited approximation of
continuous white noise (with a power-spectral density of the form Ts *
sinc^2( omega * Ts ), where Ts is the sampling interval.

.. pydrake_system::

    name: RandomSource
    output_ports:
    - output

This system exposes a parameter named ``seed`` for the pseudo-random
number generator that determines the noise output. The ``seed``
parameter behaves as follows:

1. Each newly-created RandomSource chooses a new ``per_instance_seed`` member
field value for itself.  The value will be unique within the current
executable process.

2. By default, ``source.CreateDefaultContext()`` will set the returned
context's ``seed`` parameter to ``per_instance_seed``.  Therefore, for a given
instance of this system, the parameters, state, and outputs will be
identical for all simulations that start from a default context.

3. By default, ``source.SetRandomContext()`` will choose a new, arbitrary
value for the context's ``seed`` parameter, which means that the system's
parameters, state, and outputs will (almost certainly) differ from their
defaults.

4. The user may call ``source.set_fixed_seed(new_seed)`` on this system.
When a ``new_seed`` value is provided, it is used by both
``CreateDefaultContext`` and ``SetRandomContext``.  Therefore, the system's
parameters, state, and outputs will be identical to any other instances
that share the same ``new_seed`` value for their ``seed`` context parameter.
Note that ``set_fixed_seed`` affects subsequently-created contexts; any
pre-existing contexts are unaffected.  The user may call
``source.set_fixed_seed(nullopt)`` to revert to the default the behaviors
described in #2 and #3 again.

5. The context returned by ``source.AllocateContext()`` does not contain a
valid ``seed`` parameter; that context should not be used until its values
are populated via another Context.

Note:
    This system is only defined for the double scalar type.

Note:
    The exact distribution results may vary across multiple platforms
    or revisions of Drake, but will be consistent for all compilations
    on a given platform and Drake revision.

Note:
    The hard-coding of (default) distribution parameters is imposed
    intentionally to simplify analysis (by forcing systems taking
    noise inputs to implement the shifting/scaling, the system itself
    contains all of the necessary information for stochastic
    analysis).

See also:
    stochastic_systems)""";
        // Symbol: drake::systems::RandomSource::RandomSource<T>
        struct /* ctor */ {
          // Source: drake/systems/primitives/random_source.h
          const char* doc =
R"""(Constructs the RandomSource system.

Parameter ``distribution``:
    The RandomDistribution used for each of the outputs.

Parameter ``num_outputs``:
    The dimension of the (single) vector output port.

Parameter ``sampling_interval_sec``:
    The sampling interval in seconds.)""";
        } ctor;
        // Symbol: drake::systems::RandomSource::Seed
        struct /* Seed */ {
          // Source: drake/systems/primitives/random_source.h
          const char* doc = R"""(An integer type for a random seed.)""";
        } Seed;
        // Symbol: drake::systems::RandomSource::get_distribution
        struct /* get_distribution */ {
          // Source: drake/systems/primitives/random_source.h
          const char* doc =
R"""(Returns the ``distribution`` given at construction.)""";
        } get_distribution;
        // Symbol: drake::systems::RandomSource::get_fixed_seed
        struct /* get_fixed_seed */ {
          // Source: drake/systems/primitives/random_source.h
          const char* doc =
R"""(Gets this system's fixed random seed (or else nullopt when the seed is
not fixed). Refer to the class overview documentation for details.)""";
        } get_fixed_seed;
        // Symbol: drake::systems::RandomSource::get_seed
        struct /* get_seed */ {
          // Source: drake/systems/primitives/random_source.h
          const char* doc =
R"""(Returns the value of the ``seed`` parameter in the given context.)""";
        } get_seed;
        // Symbol: drake::systems::RandomSource::set_fixed_seed
        struct /* set_fixed_seed */ {
          // Source: drake/systems/primitives/random_source.h
          const char* doc =
R"""(Sets (or clears) this system's fixed random seed. Refer to the class
overview documentation for details.)""";
        } set_fixed_seed;
      } RandomSource;
      // Symbol: drake::systems::Saturation
      struct /* Saturation */ {
        // Source: drake/systems/primitives/saturation.h
        const char* doc =
R"""(An element-wise hard saturation block with inputs signal ``u``,
saturation values :math:`u_{min}` and/or :math:`u_{max}`, and output
``y`` respectively as in:

.. math:: y = u, u_{min} < u < u_{max}

.. math:: y = u_{min}, u \le u_{min}

.. math:: y = u_{max}, u \ge u_{max}

The input to this system directly feeds through to its output.

Note that :math:`u_{min}`, and :math:`u_{max}`, and :math:`u` are all
vectors of same dimension, and the following condition holds
elementwise in runtime.

.. math:: u_{min} <=  u_{max}

The quantities :math:`u_{min}`, and :math:`u_{max}` can be supplied as
inputs in separate ports or be initialised as constants using the
appropriate constructor by passing their default value. If these
quantities are not defined as constants but they are not connected to
appropriate sources, their values are taken by default to be
:math:`u_{min} = -\infty`, and :math:`u_{max} = \infty` respectively.
In this "variable" configuration, at least one of the input ports must
be connected.

.. pydrake_system::

    name: Saturation
    input_ports:
    - u0
    - <span style="color:gray">u1</span>
    - <span style="color:gray">u2</span>
    output_ports:
    - y0

Ports show in <span style="color:gray">gray</span> may be absent,
depending on how the system is constructed.)""";
        // Symbol: drake::systems::Saturation::Saturation<T>
        struct /* ctor */ {
          // Source: drake/systems/primitives/saturation.h
          const char* doc_1args =
R"""(Constructs a variable Saturation system where the upper and lower
values are represented by vectors of identical size and can be
supplied via the max_value_port and min_value_port respectively.

.. pydrake_system::

    name: Saturation
    input_ports:
    - u0
    - u1
    - u2
    output_ports:
    - y0

Port ``u1`` is the source of :math:`u_{max}`; port ``u2`` is the
source of :math:`u_{min}`.

Parameter ``input_size``:
    sets size of the input and output ports.

Please consult this class's description for the requirements of
``u_min`` and ``u_max`` to be supplied via the corresponding ports.)""";
          // Source: drake/systems/primitives/saturation.h
          const char* doc_2args =
R"""(Constructs a constant Saturation system where the upper and lower
values are represented by vectors of identical size supplied via this
constructor.

.. pydrake_system::

    name: Saturation
    input_ports:
    - u0
    output_ports:
    - y0

Parameter ``min_value``:
    the lower (vector) limit to the saturation.

Parameter ``max_value``:
    the upper (vector) limit to the saturation.

Please consult this class's description for the requirements of
``min_value`` and ``max_value``.)""";
          // Source: drake/systems/primitives/saturation.h
          const char* doc_copyconvert =
R"""(Scalar-converting copy constructor. See system_scalar_conversion.)""";
        } ctor;
        // Symbol: drake::systems::Saturation::get_input_port
        struct /* get_input_port */ {
          // Source: drake/systems/primitives/saturation.h
          const char* doc = R"""(Returns the input port.)""";
        } get_input_port;
        // Symbol: drake::systems::Saturation::get_max_value_port
        struct /* get_max_value_port */ {
          // Source: drake/systems/primitives/saturation.h
          const char* doc = R"""(Returns the max value port.)""";
        } get_max_value_port;
        // Symbol: drake::systems::Saturation::get_min_value_port
        struct /* get_min_value_port */ {
          // Source: drake/systems/primitives/saturation.h
          const char* doc = R"""(Returns the min value port.)""";
        } get_min_value_port;
        // Symbol: drake::systems::Saturation::get_size
        struct /* get_size */ {
          // Source: drake/systems/primitives/saturation.h
          const char* doc = R"""(Returns the size.)""";
        } get_size;
      } Saturation;
      // Symbol: drake::systems::Selector
      struct /* Selector */ {
        // Source: drake/systems/primitives/selector.h
        const char* doc =
R"""(This system combines multiple vector-valued inputs into multiple
vector- valued outputs. It operates at the level of individual
elements of the input and output vectors (i.e., an output port can
provide a mixture of data from multiple input ports). The inputs to
this system directly feed through to its outputs. Refer to
SelectorParams to understand how the selection is specified.

.. pydrake_system::

    name: Selector
    input_ports:
    - u0
    - ...
    - u(N-1)
    output_ports:
    - y0
    - ...
    - y(M-1)

The port names shown in the figure above are the defaults. Custom
names may be specified in the SelectorParams.)""";
        // Symbol: drake::systems::Selector::Selector<T>
        struct /* ctor */ {
          // Source: drake/systems/primitives/selector.h
          const char* doc =
R"""(Constructs a Selector with the given parameters.)""";
          // Source: drake/systems/primitives/selector.h
          const char* doc_copyconvert =
R"""(Scalar-converting copy constructor. See system_scalar_conversion.)""";
        } ctor;
      } Selector;
      // Symbol: drake::systems::SelectorParams
      struct /* SelectorParams */ {
        // Source: drake/systems/primitives/selector.h
        const char* doc =
R"""(The constructor arguments for a Selector.)""";
        // Symbol: drake::systems::SelectorParams::InputPortParams
        struct /* InputPortParams */ {
          // Source: drake/systems/primitives/selector.h
          const char* doc = R"""(Helper struct for ``inputs``.)""";
          // Symbol: drake::systems::SelectorParams::InputPortParams::Serialize
          struct /* Serialize */ {
            // Source: drake/systems/primitives/selector.h
            const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.)""";
          } Serialize;
          // Symbol: drake::systems::SelectorParams::InputPortParams::name
          struct /* name */ {
            // Source: drake/systems/primitives/selector.h
            const char* doc =
R"""((Optional) Specifies a name for the port. When given, the name must be
a valid port name. When unset, the port will use kDefaultName.)""";
          } name;
          // Symbol: drake::systems::SelectorParams::InputPortParams::size
          struct /* size */ {
            // Source: drake/systems/primitives/selector.h
            const char* doc =
R"""(Specifies the size of the port, which must be non-negative.)""";
          } size;
          auto Serialize__fields() const {
            return std::array{
              std::make_pair("name", name.doc),
              std::make_pair("size", size.doc),
            };
          }
        } InputPortParams;
        // Symbol: drake::systems::SelectorParams::OutputPortParams
        struct /* OutputPortParams */ {
          // Source: drake/systems/primitives/selector.h
          const char* doc = R"""(Helper struct for ``outputs``.)""";
          // Symbol: drake::systems::SelectorParams::OutputPortParams::Serialize
          struct /* Serialize */ {
            // Source: drake/systems/primitives/selector.h
            const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.)""";
          } Serialize;
          // Symbol: drake::systems::SelectorParams::OutputPortParams::name
          struct /* name */ {
            // Source: drake/systems/primitives/selector.h
            const char* doc =
R"""((Optional) Specifies a name for the port. When given, the name must be
a valid port name. When unset, the port will use kDefaultName.)""";
          } name;
          // Symbol: drake::systems::SelectorParams::OutputPortParams::selections
          struct /* selections */ {
            // Source: drake/systems/primitives/selector.h
            const char* doc =
R"""(Specifies the values for the output port elements, and by implication
the port's size. The port will have size ``selections.size()``, and
its i'th output value will be determined by the (input_port_index,
input_offset) pair at ``selections[i]``, i.e., for the m'th output
port we have:


.. code-block:: txt

    n = selections[i].input_port_index
    j = selections[i].input_offset
    y_m[i] = u_n[j]

Output elements always come from some input element; it is not
possible to directly set an output to a constant. (If a constant is
necessary, connect a ConstantVectorSource to an input port.))""";
          } selections;
          auto Serialize__fields() const {
            return std::array{
              std::make_pair("name", name.doc),
              std::make_pair("selections", selections.doc),
            };
          }
        } OutputPortParams;
        // Symbol: drake::systems::SelectorParams::OutputSelection
        struct /* OutputSelection */ {
          // Source: drake/systems/primitives/selector.h
          const char* doc =
R"""(Helper struct for ``output_selections``.)""";
          // Symbol: drake::systems::SelectorParams::OutputSelection::Serialize
          struct /* Serialize */ {
            // Source: drake/systems/primitives/selector.h
            const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.)""";
          } Serialize;
          // Symbol: drake::systems::SelectorParams::OutputSelection::input_offset
          struct /* input_offset */ {
            // Source: drake/systems/primitives/selector.h
            const char* doc =
R"""(The element offset within the given input port to use for this single
output element.)""";
          } input_offset;
          // Symbol: drake::systems::SelectorParams::OutputSelection::input_port_index
          struct /* input_port_index */ {
            // Source: drake/systems/primitives/selector.h
            const char* doc =
R"""(The input port to use for this single output element.)""";
          } input_port_index;
          auto Serialize__fields() const {
            return std::array{
              std::make_pair("input_offset", input_offset.doc),
              std::make_pair("input_port_index", input_port_index.doc),
            };
          }
        } OutputSelection;
        // Symbol: drake::systems::SelectorParams::Serialize
        struct /* Serialize */ {
          // Source: drake/systems/primitives/selector.h
          const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.)""";
        } Serialize;
        // Symbol: drake::systems::SelectorParams::inputs
        struct /* inputs */ {
          // Source: drake/systems/primitives/selector.h
          const char* doc =
R"""(Specifies details of the input ports, and by implication the total
number of input ports. There will be ``inputs.size()`` ports in total.)""";
        } inputs;
        // Symbol: drake::systems::SelectorParams::outputs
        struct /* outputs */ {
          // Source: drake/systems/primitives/selector.h
          const char* doc =
R"""(Specifies details of the output ports, and by implication the total
number of output ports. There will be ``outputs.size()`` ports in
total.)""";
        } outputs;
        auto Serialize__fields() const {
          return std::array{
            std::make_pair("inputs", inputs.doc),
            std::make_pair("outputs", outputs.doc),
          };
        }
      } SelectorParams;
      // Symbol: drake::systems::SharedPointerSystem
      struct /* SharedPointerSystem */ {
        // Source: drake/systems/primitives/shared_pointer_system.h
        const char* doc =
R"""(SharedPointerSystem holds a single ``shared_ptr`` that will be
released at System deletion time (i.e., the end of a Diagram
lifespan). It has no input, output, state, nor parameters. This is
useful for storing objects that will be pointed-to by other systems
outside of the usual input/output port connections.

Scalar conversion is supported and will simply increment the reference
count for the contained object. The contained object will not be
scalar-converted, so should not depend on ``T``.)""";
        // Symbol: drake::systems::SharedPointerSystem::AddToBuilder
        struct /* AddToBuilder */ {
          // Source: drake/systems/primitives/shared_pointer_system.h
          const char* doc =
R"""(Creates a system holding the given value and adds it to the builder.
The value is allowed to be ``nullptr``. Returns an alias to the value
(or ``nullptr`` in case ``nullptr`` was passed in as the
``value_to_hold``).

Note:
    To immediately give up ownership at the call site, remember to use
    ``std∷move`` on the ``value_to_hold``.

Template parameter ``Held``:
    The type used to store the given value. Calls to get<>() must
    provide the same type for retrieval.

Precondition:
    builder is non-null)""";
        } AddToBuilder;
        // Symbol: drake::systems::SharedPointerSystem::SharedPointerSystem<T>
        struct /* ctor */ {
          // Source: drake/systems/primitives/shared_pointer_system.h
          const char* doc =
R"""(Creates a system holding the given value. The value is allowed to be
``nullptr``.

Note:
    To immediately give up ownership at the call site, remember to use
    ``std∷move`` on the ``value_to_hold``.

Template parameter ``Held``:
    The type used to store the given value. Calls to get<>() must
    provide the same type for retrieval.)""";
          // Source: drake/systems/primitives/shared_pointer_system.h
          const char* doc_copyconvert =
R"""(Scalar-converting copy constructor. See system_scalar_conversion.)""";
        } ctor;
        // Symbol: drake::systems::SharedPointerSystem::get
        struct /* get */ {
          // Source: drake/systems/primitives/shared_pointer_system.h
          const char* doc =
R"""((Advanced) Retrieves an alias to the stored value. Returns ``nullptr``
in case ``nullptr`` was passed in as the ``value_to_hold``.

Template parameter ``Held``:
    The type used to store the given value, per our constructor.

Raises:
    RuntimeError if Held doesn't match the type used at construction.)""";
        } get;
      } SharedPointerSystem;
      // Symbol: drake::systems::Sine
      struct /* Sine */ {
        // Source: drake/systems/primitives/sine.h
        const char* doc =
R"""(A sine system which outputs ``y = a * sin(f * t + p)`` and first and
second derivatives w.r.t. the time parameter ``t``. The block
parameters are: ``a`` the amplitude, ``f`` the frequency
(radians/second), and ``p`` the phase (radians), all of which are
constant vectors provided at construction time. This system has one or
zero input ports and three vector valued output ports (``y`` and its
first two derivatives). The user can specify whether to use simulation
time as the source of values for the time variable or an external
source. If an external time source is specified, the system is created
with an input port for the time source. Otherwise, the system is
created with zero input ports.

.. pydrake_system::

    name: Sine
    input_ports:
    - <span style="color:gray">u0</span>
    output_ports:
    - y0
    - y1
    - y2

Port ``u0`` is present if and only if the constructor parameter
``is_time_based`` is false.

Port ``y0`` emits the value ``y``; port ``y1`` emits the first
derivative; port ``y2`` emits the second derivative.)""";
        // Symbol: drake::systems::Sine::Sine<T>
        struct /* ctor */ {
          // Source: drake/systems/primitives/sine.h
          const char* doc_5args =
R"""(Constructs a Sine system where the amplitude, frequency, and phase is
applied to every input.

Parameter ``amplitude``:
    the sine wave amplitude

Parameter ``frequency``:
    the sine wave frequency (radians/second)

Parameter ``phase``:
    the sine wave phase (radians)

Parameter ``size``:
    number of elements in the output signal.

Parameter ``is_time_based``:
    indicates whether to use the simulation time as the source for the
    sine wave time variable, or use an external source, in which case
    an input port of size ``size`` is created.)""";
          // Source: drake/systems/primitives/sine.h
          const char* doc_4args =
R"""(Constructs a Sine system where different amplitudes, frequencies, and
phases can be applied to each sine wave.

Parameter ``amplitudes``:
    the sine wave amplitudes

Parameter ``frequencies``:
    the sine wave frequencies (radians/second)

Parameter ``phases``:
    the sine wave phases (radians)

Parameter ``is_time_based``:
    indicates whether to use the simulation time as the source for the
    sine wave time variable, or use an external source, in which case
    an input port is created.)""";
          // Source: drake/systems/primitives/sine.h
          const char* doc_copyconvert =
R"""(Scalar-converting copy constructor. See system_scalar_conversion.)""";
        } ctor;
        // Symbol: drake::systems::Sine::amplitude
        struct /* amplitude */ {
          // Source: drake/systems/primitives/sine.h
          const char* doc =
R"""(Returns the amplitude constant. This method should only be called if
the amplitude can be represented as a scalar value, i.e., every
element in the amplitude vector is the same. It will abort if the
amplitude cannot be represented as a single scalar value.)""";
        } amplitude;
        // Symbol: drake::systems::Sine::amplitude_vector
        struct /* amplitude_vector */ {
          // Source: drake/systems/primitives/sine.h
          const char* doc = R"""(Returns the amplitude vector constant.)""";
        } amplitude_vector;
        // Symbol: drake::systems::Sine::frequency
        struct /* frequency */ {
          // Source: drake/systems/primitives/sine.h
          const char* doc =
R"""(Returns the frequency constant. This method should only be called if
the frequency can be represented as a scalar value, i.e., every
element in the frequency vector is the same. It will abort if the
frequency cannot be represented as a single scalar value.)""";
        } frequency;
        // Symbol: drake::systems::Sine::frequency_vector
        struct /* frequency_vector */ {
          // Source: drake/systems/primitives/sine.h
          const char* doc = R"""(Returns the frequency vector constant.)""";
        } frequency_vector;
        // Symbol: drake::systems::Sine::is_time_based
        struct /* is_time_based */ {
          // Source: drake/systems/primitives/sine.h
          const char* doc =
R"""(Returns a boolean indicting whether to use simulation time as the
source of values for the time variable or an external source. Returns
true if the simulation time is used as the source, and returns false
otherwise.)""";
        } is_time_based;
        // Symbol: drake::systems::Sine::phase
        struct /* phase */ {
          // Source: drake/systems/primitives/sine.h
          const char* doc =
R"""(Returns the phase constant. This method should only be called if the
phase can be represented as a scalar value, i.e., every element in the
phase vector is the same. It will abort if the phase cannot be
represented as a single scalar value.)""";
        } phase;
        // Symbol: drake::systems::Sine::phase_vector
        struct /* phase_vector */ {
          // Source: drake/systems/primitives/sine.h
          const char* doc = R"""(Returns the phase vector constant.)""";
        } phase_vector;
      } Sine;
      // Symbol: drake::systems::SparseMatrixGain
      struct /* SparseMatrixGain */ {
        // Source: drake/systems/primitives/sparse_matrix_gain.h
        const char* doc =
R"""(A variant of MatrixGain which supports multiplication by SparseMatrix,
``D``. Specifically, given an input signal ``u`` and a state ``x``,
the output of this system, ``y``, is:

.. math:: y = D u

Note that, unlike MatrixGain, this system is not derived from
LinearSystem (which does not yet support sparse matrices). However, we
use ``D`` as the name for the gain matrix here to be consistent.

.. pydrake_system::

    name: SparseMatrixGain
    input_ports:
    - u
    output_ports:
    - y

See also:
    MatrixGain)""";
        // Symbol: drake::systems::SparseMatrixGain::D
        struct /* D */ {
          // Source: drake/systems/primitives/sparse_matrix_gain.h
          const char* doc = R"""(Getter for the gain matrix ``D``.)""";
        } D;
        // Symbol: drake::systems::SparseMatrixGain::SparseMatrixGain<T>
        struct /* ctor */ {
          // Source: drake/systems/primitives/sparse_matrix_gain.h
          const char* doc =
R"""(A constructor where the gain matrix ``D`` is ``D``.)""";
          // Source: drake/systems/primitives/sparse_matrix_gain.h
          const char* doc_copyconvert =
R"""(Scalar-converting copy constructor. See system_scalar_conversion.)""";
        } ctor;
        // Symbol: drake::systems::SparseMatrixGain::set_D
        struct /* set_D */ {
          // Source: drake/systems/primitives/sparse_matrix_gain.h
          const char* doc = R"""(Setter for the gain matrix ``D``.)""";
        } set_D;
      } SparseMatrixGain;
      // Symbol: drake::systems::StateInterpolatorWithDiscreteDerivative
      struct /* StateInterpolatorWithDiscreteDerivative */ {
        // Source: drake/systems/primitives/discrete_derivative.h
        const char* doc =
R"""(Supports the common pattern of combining a (feed-through) position
with a velocity estimated with the DiscreteDerivative into a single
output vector with positions and velocities stacked. This assumes that
the velocities are equal to the time derivative of the positions.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    ┌─────┐
    position ───┬───────────────────>│     │
                │                    │ Mux ├──> state
                │   ┌────────────┐   │     │
                └──>│  Discrete  ├──>│     │
                    │ Derivative │   └─────┘
                    └────────────┘

.. raw:: html

    </details>

.. pydrake_system::

    name: StateInterpolatorWithDiscreteDerivative
    input_ports:
    - position
    output_ports:
    - state)""";
        // Symbol: drake::systems::StateInterpolatorWithDiscreteDerivative::StateInterpolatorWithDiscreteDerivative<T>
        struct /* ctor */ {
          // Source: drake/systems/primitives/discrete_derivative.h
          const char* doc =
R"""(Constructor taking ``num_positions``, the size of the position vector
to be differentiated, and ``time_step``, the sampling interval. If
``suppress_initial_transient`` is true (the default), then the
velocity output will be zero for the first two time steps (see the
DiscreteDerivative class documentation for details and exceptions).)""";
        } ctor;
        // Symbol: drake::systems::StateInterpolatorWithDiscreteDerivative::set_initial_position
        struct /* set_initial_position */ {
          // Source: drake/systems/primitives/discrete_derivative.h
          const char* doc_2args_state_position =
R"""(Convenience method that sets the entire position history for the
discrete-time derivative to a constant vector value (resulting in
velocity estimate of zero). This is useful during initialization to
avoid large derivative outputs. ``position`` must be the same size as
the input/output ports. If suppress_initial_transient() is true, then
also disables the suppression for this ``state``.

Warning:
    This only changes the position history used for the velocity half
    of the output port; it has no effect on the feedthrough position.)""";
          // Source: drake/systems/primitives/discrete_derivative.h
          const char* doc_2args_context_position =
R"""(Convenience method that sets the entire position history for the
discrete-time derivative to a constant vector value (resulting in
velocity estimate of zero). This is useful during initialization to
avoid large derivative outputs. ``position`` must be the same size as
the input/output ports. If suppress_initial_transient() is true, then
also disables the suppression for this ``context``.

Warning:
    This only changes the position history used for the velocity half
    of the output port; it has no effect on the feedthrough position.)""";
        } set_initial_position;
        // Symbol: drake::systems::StateInterpolatorWithDiscreteDerivative::set_initial_state
        struct /* set_initial_state */ {
          // Source: drake/systems/primitives/discrete_derivative.h
          const char* doc_3args_state_position_velocity =
R"""(Convenience method that sets the entire position history for the
discrete-time derivative as if the most recent input was ``position``,
and the input before that was whatever was required to produce the
output velocity ``velocity``. ``position`` and ``velocity`` must be
the same size as the input/output ports. If
suppress_initial_transient() is true, then also disables the
suppression for this ``state``.

Warning:
    This only changes the position history used for the velocity half
    of the output port; it has no effect on the feedthrough position.)""";
          // Source: drake/systems/primitives/discrete_derivative.h
          const char* doc_3args_context_position_velocity =
R"""(Convenience method that sets the entire position history for the
discrete-time derivative as if the most recent input was ``position``,
and the input before that was whatever was required to produce the
output velocity ``velocity``. ``position`` and ``velocity`` must be
the same size as the input/output ports. If
suppress_initial_transient() is true, then also disables the
suppression for this ``context``.

Warning:
    This only changes the position history used for the velocity half
    of the output port; it has no effect on the feedthrough position.)""";
        } set_initial_state;
        // Symbol: drake::systems::StateInterpolatorWithDiscreteDerivative::suppress_initial_transient
        struct /* suppress_initial_transient */ {
          // Source: drake/systems/primitives/discrete_derivative.h
          const char* doc =
R"""(Returns the ``suppress_initial_transient`` passed to the constructor.)""";
        } suppress_initial_transient;
      } StateInterpolatorWithDiscreteDerivative;
      // Symbol: drake::systems::SymbolicVectorSystem
      struct /* SymbolicVectorSystem */ {
        // Source: drake/systems/primitives/symbolic_vector_system.h
        const char* doc =
R"""(A LeafSystem that is defined by vectors of symbolic∷Expression
representing the dynamics and output. The resulting system has only
zero or one vector input ports, zero or one vector of continuous or
discrete state (depending on the specified time_period), zero or one
vector of numeric parameters, and only zero or one vector output
ports.

See SymbolicVectorSystemBuilder to make the construction a little
nicer.

For example, to define the system: ẋ = -x + x³, y = x, we could write


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    symbolic∷Variable x("x");
      auto system = SymbolicVectorSystemBuilder().state(x)
                                                 .dynamics(-x + pow(x,3))
                                                 .output(x)
                                                 .Build();

.. raw:: html

    </details>

Note: This will not be as performant as writing your own LeafSystem.
It is meant primarily for rapid prototyping.

.. pydrake_system::

    name: SymbolicVectorSystem
    input_ports:
    - <span style="color:gray">u0</span>
    output_ports:
    - <span style="color:gray">y0</span>

Either port ``u0`` or port ``y0`` may be absent, depending on the
values supplied at construction.)""";
        // Symbol: drake::systems::SymbolicVectorSystem::SymbolicVectorSystem<T>
        struct /* ctor */ {
          // Source: drake/systems/primitives/symbolic_vector_system.h
          const char* doc_7args =
R"""(Construct the SymbolicVectorSystem.

Parameter ``time``:
    an (optional) Variable used to represent time in the dynamics.

Parameter ``state``:
    an (optional) vector of Variables representing the state. The
    order in this vector will determine the order of the elements in
    the state vector. Each element must be unique.

Parameter ``input``:
    an (optional) vector of Variables representing the input. The
    order in this vector will determine the order of the elements in
    the vector-valued input port. Each element must be unique.

Parameter ``parameter``:
    an (optional) vector of Variables representing the numeric
    parameter. The order in this vector will determine the order of
    the elements in the vector-valued parameter. Each element must be
    unique.

Parameter ``dynamics``:
    a vector of Expressions representing the dynamics of the system.
    If ``time_period`` == 0, then this describes the continuous time
    derivatives. If ``time_period`` > 0, then it defines the updates
    of the single discrete-valued state vector. The size of this
    vector must match the number of state variables.

Parameter ``output``:
    a vector of Expressions representing the output of the system. If
    empty, then no output port will be allocated.

Parameter ``time_period``:
    a scalar representing the period of a periodic update. time_period
    == 0.0 implies that the state variables will be declared as
    continuous state and the dynamics will be implemented as time
    derivatives. time_period > 0.0 implies the state variables will be
    declared as discrete state and the dynamics will be implemented as
    a dicraete variable update.)""";
          // Source: drake/systems/primitives/symbolic_vector_system.h
          const char* doc_6args =
R"""(Construct the SymbolicVectorSystem.

Parameter ``time``:
    an (optional) Variable used to represent time in the dynamics.

Parameter ``state``:
    an (optional) vector of Variables representing the state. The
    order in this vector will determine the order of the elements in
    the state vector. Each element must be unique.

Parameter ``input``:
    an (optional) vector of Variables representing the input. The
    order in this vector will determine the order of the elements in
    the vector-valued input port. Each element must be unique.

Parameter ``dynamics``:
    a vector of Expressions representing the dynamics of the system.
    If ``time_period`` == 0, then this describes the continuous time
    derivatives. If ``time_period`` > 0, then it defines the updates
    of the single discrete-valued state vector. The size of this
    vector must match the number of state variables.

Parameter ``output``:
    a vector of Expressions representing the output of the system. If
    empty, then no output port will be allocated.

Parameter ``time_period``:
    a scalar representing the period of a periodic update. time_period
    == 0.0 implies that the state variables will be declared as
    continuous state and the dynamics will be implemented as time
    derivatives. time_period > 0.0 implies the state variables will be
    declared as discrete state and the dynamics will be implemented as
    a dicraete variable update.)""";
          // Source: drake/systems/primitives/symbolic_vector_system.h
          const char* doc_copyconvert =
R"""(Scalar-converting copy constructor. See system_scalar_conversion.)""";
        } ctor;
        // Symbol: drake::systems::SymbolicVectorSystem::dynamics
        struct /* dynamics */ {
          // Source: drake/systems/primitives/symbolic_vector_system.h
          const char* doc = R"""()""";
        } dynamics;
        // Symbol: drake::systems::SymbolicVectorSystem::dynamics_for_variable
        struct /* dynamics_for_variable */ {
          // Source: drake/systems/primitives/symbolic_vector_system.h
          const char* doc =
R"""(Returns the dynamics for the variable ``var``. That is, it returns the
scalar expression corresponding to either ``\dot{var}`` (continuous
case) or ``var[n+1]`` (discrete case).

Raises:
    RuntimeError if this system has no corresponding dynamics for the
    variable ``var``.)""";
        } dynamics_for_variable;
        // Symbol: drake::systems::SymbolicVectorSystem::input_vars
        struct /* input_vars */ {
          // Source: drake/systems/primitives/symbolic_vector_system.h
          const char* doc = R"""()""";
        } input_vars;
        // Symbol: drake::systems::SymbolicVectorSystem::output
        struct /* output */ {
          // Source: drake/systems/primitives/symbolic_vector_system.h
          const char* doc = R"""()""";
        } output;
        // Symbol: drake::systems::SymbolicVectorSystem::parameter_vars
        struct /* parameter_vars */ {
          // Source: drake/systems/primitives/symbolic_vector_system.h
          const char* doc = R"""()""";
        } parameter_vars;
        // Symbol: drake::systems::SymbolicVectorSystem::state_vars
        struct /* state_vars */ {
          // Source: drake/systems/primitives/symbolic_vector_system.h
          const char* doc = R"""()""";
        } state_vars;
        // Symbol: drake::systems::SymbolicVectorSystem::time_var
        struct /* time_var */ {
          // Source: drake/systems/primitives/symbolic_vector_system.h
          const char* doc = R"""(@name Accessor methods.)""";
        } time_var;
      } SymbolicVectorSystem;
      // Symbol: drake::systems::SymbolicVectorSystemBuilder
      struct /* SymbolicVectorSystemBuilder */ {
        // Source: drake/systems/primitives/symbolic_vector_system.h
        const char* doc =
R"""(Builder design pattern to help with all of the optional arguments in
the constructor of SymbolicVectorSystem.

For example, to define the system: ẋ = -x + x³, y = x, we could write


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    symbolic∷Variable x("x");
      auto system = SymbolicVectorSystemBuilder().state(x)
                                                 .dynamics(-x + pow(x,3))
                                                 .output(x)
                                                 .Build();

.. raw:: html

    </details>

See also:
    SymbolicVectorSystem)""";
        // Symbol: drake::systems::SymbolicVectorSystemBuilder::Build
        struct /* Build */ {
          // Source: drake/systems/primitives/symbolic_vector_system.h
          const char* doc =
R"""(Dispatches to the SymbolicVectorSystem constructor with our
accumulated arguments.)""";
        } Build;
        // Symbol: drake::systems::SymbolicVectorSystemBuilder::LinearizeDynamics
        struct /* LinearizeDynamics */ {
          // Source: drake/systems/primitives/symbolic_vector_system.h
          const char* doc =
R"""(Linearizes the system dynamics around ``(x0, u0)`` using the
first-order Taylor Series expansion.

Precondition:
    The length of ``x0`` should be the length of ``state()``.

Precondition:
    The length of ``u0`` should be the length of ``input()``.

Precondition:
    ``x0`` and ``u0`` should not include a state variable or an input
    variable.

Note:
    If ``x0`` or ``u0`` includes a variable new to this system
    builder, it will be added to this system builder as a parameter.)""";
        } LinearizeDynamics;
        // Symbol: drake::systems::SymbolicVectorSystemBuilder::SymbolicVectorSystemBuilder
        struct /* ctor */ {
          // Source: drake/systems/primitives/symbolic_vector_system.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::systems::SymbolicVectorSystemBuilder::dynamics
        struct /* dynamics */ {
          // Source: drake/systems/primitives/symbolic_vector_system.h
          const char* doc_was_unable_to_choose_unambiguous_names =
R"""(Sets the dynamics method (scalar version).)""";
        } dynamics;
        // Symbol: drake::systems::SymbolicVectorSystemBuilder::dynamics_for_variable
        struct /* dynamics_for_variable */ {
          // Source: drake/systems/primitives/symbolic_vector_system.h
          const char* doc =
R"""(Returns the dynamics for the variable ``var``. That is, it returns the
scalar expression corresponding to either ``\dot{var}`` (continuous
case) or ``var[n+1]`` (discrete case).

Raises:
    RuntimeError if this builder has no corresponding dynamics for the
    variable ``var``.)""";
        } dynamics_for_variable;
        // Symbol: drake::systems::SymbolicVectorSystemBuilder::input
        struct /* input */ {
          // Source: drake/systems/primitives/symbolic_vector_system.h
          const char* doc_was_unable_to_choose_unambiguous_names =
R"""(Sets the input variable (scalar version).)""";
        } input;
        // Symbol: drake::systems::SymbolicVectorSystemBuilder::output
        struct /* output */ {
          // Source: drake/systems/primitives/symbolic_vector_system.h
          const char* doc_was_unable_to_choose_unambiguous_names =
R"""(Sets the output method (scalar version).)""";
        } output;
        // Symbol: drake::systems::SymbolicVectorSystemBuilder::parameter
        struct /* parameter */ {
          // Source: drake/systems/primitives/symbolic_vector_system.h
          const char* doc_was_unable_to_choose_unambiguous_names =
R"""(Sets the parameter variable (scalar version).)""";
        } parameter;
        // Symbol: drake::systems::SymbolicVectorSystemBuilder::state
        struct /* state */ {
          // Source: drake/systems/primitives/symbolic_vector_system.h
          const char* doc_was_unable_to_choose_unambiguous_names =
R"""(Sets the state variable (scalar version).)""";
        } state;
        // Symbol: drake::systems::SymbolicVectorSystemBuilder::time
        struct /* time */ {
          // Source: drake/systems/primitives/symbolic_vector_system.h
          const char* doc_1args = R"""(Sets the time variable.)""";
          // Source: drake/systems/primitives/symbolic_vector_system.h
          const char* doc_0args =
R"""(@name Accessor methods. Returns the time variable if exists.)""";
        } time;
        // Symbol: drake::systems::SymbolicVectorSystemBuilder::time_period
        struct /* time_period */ {
          // Source: drake/systems/primitives/symbolic_vector_system.h
          const char* doc_1args =
R"""(Sets the time period (0 is continuous time).)""";
          // Source: drake/systems/primitives/symbolic_vector_system.h
          const char* doc_0args = R"""(Returns the time period.)""";
        } time_period;
      } SymbolicVectorSystemBuilder;
      // Symbol: drake::systems::TimeVaryingAffineSystem
      struct /* TimeVaryingAffineSystem */ {
        // Source: drake/systems/primitives/affine_system.h
        const char* doc =
R"""(Base class for a discrete- or continuous-time, time-varying affine
system, with potentially time-varying coefficients.

.. pydrake_system::

    name: TimeVaryingAffineSystem
    input_ports:
    - u0
    output_ports:
    - y0

If ``time_period > 0.0``, then the affine system will have the state
update:

.. math:: x(t+h) = A(t) x(t) + B(t) u(t) + f_0(t),

where ``h`` is the time_period. If ``time_period == 0.0``, then the
system will have the time derivatives:

.. math:: \dot{x}(t) = A(t) x(t) + B(t) u(t) + f_0(t),

where ``u`` denotes the input vector, ``x`` denotes the state vector.

In both cases, the system will have the output:

.. math:: y(t) = C(t) x(t) + D(t) u(t) + y_0(t),

where ``y`` denotes the output vector.

When configured as a discrete system, the discrete update can be
triggered either by the defined periodic trigger or via a manual
forced update.

*

See also:
    AffineSystem)""";
        // Symbol: drake::systems::TimeVaryingAffineSystem::A
        struct /* A */ {
          // Source: drake/systems/primitives/affine_system.h
          const char* doc =
R"""(@name Methods To Be Implemented by Subclasses

Implementations must define these, and the returned matrices must be
sized to match the ``num_states``, `num_inputs`, and ``num_outputs``
specified in the constructor.)""";
        } A;
        // Symbol: drake::systems::TimeVaryingAffineSystem::B
        struct /* B */ {
          // Source: drake/systems/primitives/affine_system.h
          const char* doc = R"""()""";
        } B;
        // Symbol: drake::systems::TimeVaryingAffineSystem::C
        struct /* C */ {
          // Source: drake/systems/primitives/affine_system.h
          const char* doc = R"""()""";
        } C;
        // Symbol: drake::systems::TimeVaryingAffineSystem::CalcDiscreteUpdate
        struct /* CalcDiscreteUpdate */ {
          // Source: drake/systems/primitives/affine_system.h
          const char* doc =
R"""(Computes

.. math:: x(t+h) = A(t) x(t) + B(t) u(t) + f_0(t),

with by calling ``A(t)``, `B(t)``, and `f0(t)`` with runtime size
checks. This is the event handler for the periodic and forced discrete
update events. Derived classes may override this for performance
reasons.)""";
        } CalcDiscreteUpdate;
        // Symbol: drake::systems::TimeVaryingAffineSystem::CalcOutputY
        struct /* CalcOutputY */ {
          // Source: drake/systems/primitives/affine_system.h
          const char* doc =
R"""(Computes

.. math:: y(t) = C(t) x(t) + D(t) u(t) + y_0(t),

with by calling ``C(t)``, `D(t)``, and `y0(t)`` with runtime size
checks. Derived classes may override this for performance reasons.)""";
        } CalcOutputY;
        // Symbol: drake::systems::TimeVaryingAffineSystem::ConfigureDefaultAndRandomStateFrom
        struct /* ConfigureDefaultAndRandomStateFrom */ {
          // Source: drake/systems/primitives/affine_system.h
          const char* doc =
R"""(Helper method. Derived classes should call this from the
scalar-converting copy constructor.)""";
        } ConfigureDefaultAndRandomStateFrom;
        // Symbol: drake::systems::TimeVaryingAffineSystem::D
        struct /* D */ {
          // Source: drake/systems/primitives/affine_system.h
          const char* doc = R"""()""";
        } D;
        // Symbol: drake::systems::TimeVaryingAffineSystem::DoCalcTimeDerivatives
        struct /* DoCalcTimeDerivatives */ {
          // Source: drake/systems/primitives/affine_system.h
          const char* doc =
R"""(Computes

.. math:: \dot{x}(t) = A(t) x(t) + B(t) u(t) + f_0(t),

with by calling ``A(t)``, `B(t)``, and `f0(t)`` with runtime size
checks. Derived classes may override this for performance reasons.)""";
        } DoCalcTimeDerivatives;
        // Symbol: drake::systems::TimeVaryingAffineSystem::SetDefaultState
        struct /* SetDefaultState */ {
          // Source: drake/systems/primitives/affine_system.h
          const char* doc = R"""(Sets the initial conditions.)""";
        } SetDefaultState;
        // Symbol: drake::systems::TimeVaryingAffineSystem::SetRandomState
        struct /* SetRandomState */ {
          // Source: drake/systems/primitives/affine_system.h
          const char* doc = R"""(Sets the random initial conditions.)""";
        } SetRandomState;
        // Symbol: drake::systems::TimeVaryingAffineSystem::TimeVaryingAffineSystem<T>
        struct /* ctor */ {
          // Source: drake/systems/primitives/affine_system.h
          const char* doc =
R"""(Constructor.

Parameter ``converter``:
    scalar-type conversion support helper (i.e., AutoDiff, etc.); pass
    a default-constructed object if such support is not desired. See
    system_scalar_conversion for detailed background and examples
    related to scalar-type conversion support.

Parameter ``num_states``:
    size of the system's state vector

Parameter ``num_inputs``:
    size of the system's input vector

Parameter ``num_outputs``:
    size of the system's output vector

Parameter ``time_period``:
    discrete update period, or 0.0 to use continuous time)""";
        } ctor;
        // Symbol: drake::systems::TimeVaryingAffineSystem::configure_default_state
        struct /* configure_default_state */ {
          // Source: drake/systems/primitives/affine_system.h
          const char* doc =
R"""(Configures the value that will be assigned to the state vector in
``SetDefaultContext``. `x0` must be a vector of length ``num_states``.)""";
        } configure_default_state;
        // Symbol: drake::systems::TimeVaryingAffineSystem::configure_random_state
        struct /* configure_random_state */ {
          // Source: drake/systems/primitives/affine_system.h
          const char* doc =
R"""(Configures the Gaussian distribution over state vectors used in the
``SetRandomContext`` methods. The mean of the distribution will be the
default state (

See also:
    configure_default_state()). ``covariance`` must have size
    ``num_states`` by ``num_states`` and must be symmetric and
    positive semi-definite.)""";
        } configure_random_state;
        // Symbol: drake::systems::TimeVaryingAffineSystem::f0
        struct /* f0 */ {
          // Source: drake/systems/primitives/affine_system.h
          const char* doc = R"""()""";
        } f0;
        // Symbol: drake::systems::TimeVaryingAffineSystem::get_default_state
        struct /* get_default_state */ {
          // Source: drake/systems/primitives/affine_system.h
          const char* doc =
R"""(Returns the configured default state.

See also:
    configure_default_state().)""";
        } get_default_state;
        // Symbol: drake::systems::TimeVaryingAffineSystem::get_input_port
        struct /* get_input_port */ {
          // Source: drake/systems/primitives/affine_system.h
          const char* doc =
R"""(Returns the input port containing the externally applied input.)""";
        } get_input_port;
        // Symbol: drake::systems::TimeVaryingAffineSystem::get_output_port
        struct /* get_output_port */ {
          // Source: drake/systems/primitives/affine_system.h
          const char* doc =
R"""(Returns the output port containing the output state.)""";
        } get_output_port;
        // Symbol: drake::systems::TimeVaryingAffineSystem::get_random_state_covariance
        struct /* get_random_state_covariance */ {
          // Source: drake/systems/primitives/affine_system.h
          const char* doc =
R"""(Returns the configured random state covariance.)""";
        } get_random_state_covariance;
        // Symbol: drake::systems::TimeVaryingAffineSystem::num_inputs
        struct /* num_inputs */ {
          // Source: drake/systems/primitives/affine_system.h
          const char* doc = R"""()""";
        } num_inputs;
        // Symbol: drake::systems::TimeVaryingAffineSystem::num_outputs
        struct /* num_outputs */ {
          // Source: drake/systems/primitives/affine_system.h
          const char* doc = R"""()""";
        } num_outputs;
        // Symbol: drake::systems::TimeVaryingAffineSystem::num_states
        struct /* num_states */ {
          // Source: drake/systems/primitives/affine_system.h
          const char* doc = R"""()""";
        } num_states;
        // Symbol: drake::systems::TimeVaryingAffineSystem::time_period
        struct /* time_period */ {
          // Source: drake/systems/primitives/affine_system.h
          const char* doc = R"""()""";
        } time_period;
        // Symbol: drake::systems::TimeVaryingAffineSystem::y0
        struct /* y0 */ {
          // Source: drake/systems/primitives/affine_system.h
          const char* doc = R"""()""";
        } y0;
      } TimeVaryingAffineSystem;
      // Symbol: drake::systems::TimeVaryingLinearSystem
      struct /* TimeVaryingLinearSystem */ {
        // Source: drake/systems/primitives/linear_system.h
        const char* doc =
R"""(Base class for a discrete or continuous linear time-varying (LTV)
system.

.. pydrake_system::

    name: TimeVaryingLinearSystem
    input_ports:
    - u0
    output_ports:
    - y0

If ``time_period > 0.0``, the system will have the following
discrete-time state update:

.. math:: x(t+h) = A(t) x(t) + B(t) u(t),

where ``h`` is the time_period. If ``time_period == 0.0``, the system
will have the following continuous-time state update:

.. math:: \dot{x}(t) = A(t) x(t) + B(t) u(t),

both with the output:

.. math:: y(t) = C(t) x(t) + D(t) u(t).)""";
        // Symbol: drake::systems::TimeVaryingLinearSystem::TimeVaryingLinearSystem<T>
        struct /* ctor */ {
          // Source: drake/systems/primitives/linear_system.h
          const char* doc =
R"""(Constructor.

Parameter ``converter``:
    scalar-type conversion support helper (i.e., AutoDiff, etc.); pass
    a default-constructed object if such support is not desired. See
    system_scalar_conversion for detailed background and examples
    related to scalar-type conversion support.

Parameter ``num_states``:
    size of the system's state vector

Parameter ``num_inputs``:
    size of the system's input vector

Parameter ``num_outputs``:
    size of the system's output vector

Parameter ``time_period``:
    discrete update period, or 0.0 to use continuous time)""";
        } ctor;
      } TimeVaryingLinearSystem;
      // Symbol: drake::systems::TrajectoryAffineSystem
      struct /* TrajectoryAffineSystem */ {
        // Source: drake/systems/primitives/trajectory_affine_system.h
        const char* doc =
R"""(A continuous- or discrete-time Affine Time-Varying system with system
matrices described by trajectories.

.. pydrake_system::

    name: TrajectoryAffineSystem
    input_ports:
    - u0
    output_ports:
    - y0)""";
        // Symbol: drake::systems::TrajectoryAffineSystem::A
        struct /* A */ {
          // Source: drake/systems/primitives/trajectory_affine_system.h
          const char* doc =
R"""(@name Implementations of TimeVaryingAffineSystem<T>'s pure virtual
methods.)""";
        } A;
        // Symbol: drake::systems::TrajectoryAffineSystem::B
        struct /* B */ {
          // Source: drake/systems/primitives/trajectory_affine_system.h
          const char* doc = R"""()""";
        } B;
        // Symbol: drake::systems::TrajectoryAffineSystem::C
        struct /* C */ {
          // Source: drake/systems/primitives/trajectory_affine_system.h
          const char* doc = R"""()""";
        } C;
        // Symbol: drake::systems::TrajectoryAffineSystem::D
        struct /* D */ {
          // Source: drake/systems/primitives/trajectory_affine_system.h
          const char* doc = R"""()""";
        } D;
        // Symbol: drake::systems::TrajectoryAffineSystem::TrajectoryAffineSystem<T>
        struct /* ctor */ {
          // Source: drake/systems/primitives/trajectory_affine_system.h
          const char* doc =
R"""(Constructs a TrajectoryAffineSystem from trajectories of matrices.

Parameter ``time_period``:
    Defines the period of the discrete time system; use
    time_period=0.0 to denote a continuous time system. $*Default:*
    0.0)""";
          // Source: drake/systems/primitives/trajectory_affine_system.h
          const char* doc_copyconvert =
R"""(Scalar-converting copy constructor. See system_scalar_conversion.)""";
        } ctor;
        // Symbol: drake::systems::TrajectoryAffineSystem::f0
        struct /* f0 */ {
          // Source: drake/systems/primitives/trajectory_affine_system.h
          const char* doc = R"""()""";
        } f0;
        // Symbol: drake::systems::TrajectoryAffineSystem::y0
        struct /* y0 */ {
          // Source: drake/systems/primitives/trajectory_affine_system.h
          const char* doc = R"""()""";
        } y0;
      } TrajectoryAffineSystem;
      // Symbol: drake::systems::TrajectoryLinearSystem
      struct /* TrajectoryLinearSystem */ {
        // Source: drake/systems/primitives/trajectory_linear_system.h
        const char* doc =
R"""(A continuous- or discrete-time Linear Time-Varying system with system
matrices described by trajectories.

.. pydrake_system::

    name: TrajectoryLinearSystem
    input_ports:
    - u0
    output_ports:
    - y0)""";
        // Symbol: drake::systems::TrajectoryLinearSystem::A
        struct /* A */ {
          // Source: drake/systems/primitives/trajectory_linear_system.h
          const char* doc =
R"""(@name Implementations of PiecewisePolynomialLinearSystem<T>'s pure
virtual methods.)""";
        } A;
        // Symbol: drake::systems::TrajectoryLinearSystem::B
        struct /* B */ {
          // Source: drake/systems/primitives/trajectory_linear_system.h
          const char* doc = R"""()""";
        } B;
        // Symbol: drake::systems::TrajectoryLinearSystem::C
        struct /* C */ {
          // Source: drake/systems/primitives/trajectory_linear_system.h
          const char* doc = R"""()""";
        } C;
        // Symbol: drake::systems::TrajectoryLinearSystem::D
        struct /* D */ {
          // Source: drake/systems/primitives/trajectory_linear_system.h
          const char* doc = R"""()""";
        } D;
        // Symbol: drake::systems::TrajectoryLinearSystem::TrajectoryLinearSystem<T>
        struct /* ctor */ {
          // Source: drake/systems/primitives/trajectory_linear_system.h
          const char* doc =
R"""(Constructs a PiecewisePolynomialLinearSystem from a
LinearTimeVaryingData structure.

Parameter ``time_period``:
    Defines the period of the discrete time system; use
    time_period=0.0 to denote a continuous time system. $*Default:*
    0.0)""";
          // Source: drake/systems/primitives/trajectory_linear_system.h
          const char* doc_copyconvert =
R"""(Scalar-converting copy constructor. See system_scalar_conversion.)""";
        } ctor;
      } TrajectoryLinearSystem;
      // Symbol: drake::systems::TrajectorySource
      struct /* TrajectorySource */ {
        // Source: drake/systems/primitives/trajectory_source.h
        const char* doc =
R"""(Given a Trajectory, this System provides an output port with the value
of the trajectory evaluated at the current time.

If the particular Trajectory is not available at the time the System /
Diagram is being constructed, one can create a TrajectorySource with a
placeholder trajectory (e.g. PiecewisePolynomimal(Eigen∷VectorXd))
with the correct number of rows, and then use UpdateTrajectory().

.. pydrake_system::

    name: TrajectorySource
    output_ports:
    - y0

Note: Scalar conversion is supported from double to any other scalar,
but the stored Trajectory is not automatically scalar converted. You
must call UpdateTrajectory() with an updated Trajectory<T> in order to
fully enable scalar-type support on the trajectory parameters/values.)""";
        // Symbol: drake::systems::TrajectorySource::TrajectorySource<T>
        struct /* ctor */ {
          // Source: drake/systems/primitives/trajectory_source.h
          const char* doc =
R"""(Parameter ``trajectory``:
    Trajectory used by the system.

Parameter ``output_derivative_order``:
    The number of times to take the derivative. Must be greater than
    or equal to zero.

Parameter ``zero_derivatives_beyond_limits``:
    All derivatives will be zero before the start time or after the
    end time of ``trajectory``. However, this clamping is ignored for
    T=Expression.

Precondition:
    The value of ``trajectory`` is a column vector. More precisely,
    trajectory.cols() == 1.)""";
        } ctor;
        // Symbol: drake::systems::TrajectorySource::UpdateTrajectory
        struct /* UpdateTrajectory */ {
          // Source: drake/systems/primitives/trajectory_source.h
          const char* doc =
R"""(Updates the stored trajectory. ``trajectory`` must have the same
number of rows as the trajectory passed to the constructor.)""";
        } UpdateTrajectory;
      } TrajectorySource;
      // Symbol: drake::systems::TransferFunction
      struct /* TransferFunction */ {
        // Source: drake/systems/primitives/transfer_function.h
        const char* doc =
R"""(Represents a linear-time-invariant (LTI) system in transfer function
form, e.g. complex rational functions corresponding to the Laplace
transform or z-transform of the impulse response
(https://en.wikipedia.org/wiki/Transfer_function).

For example, to create the transfer function H(s) = 1/(s + 1), you can
write:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    auto s = TransferFunction∷s();
    TransferFunction H(1.0 / (s + 1.0));

.. raw:: html

    </details>

To create the transfer function H(z) = 1/(z - 0.5), you can write:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    auto z = TransferFunction∷z();
    double time_step = 0.1;
    TransferFunction H(1.0 / (z - 0.5), time_step);

.. raw:: html

    </details>

Note that TransferFunction is *not* a LeafSystem. To use it in the
Systems framework, you must convert it to state-space form,
represented by LinearSystem.)""";
        // Symbol: drake::systems::TransferFunction::H
        struct /* H */ {
          // Source: drake/systems/primitives/transfer_function.h
          const char* doc = R"""(Returns the transfer function matrix.)""";
        } H;
        // Symbol: drake::systems::TransferFunction::TransferFunction
        struct /* ctor */ {
          // Source: drake/systems/primitives/transfer_function.h
          const char* doc_was_unable_to_choose_unambiguous_names = R"""()""";
        } ctor;
        // Symbol: drake::systems::TransferFunction::s
        struct /* s */ {
          // Source: drake/systems/primitives/transfer_function.h
          const char* doc =
R"""(Returns a symbolic∷RationalFunction ``s`` which can be used to build a
continuous-time transfer function symbolically.)""";
        } s;
        // Symbol: drake::systems::TransferFunction::s_var
        struct /* s_var */ {
          // Source: drake/systems/primitives/transfer_function.h
          const char* doc =
R"""(Returns the singleton symbolic∷Variable denoting the complex frequency
(σ+jω) used in the Laplace transform for continuous-time transfer
functions.)""";
        } s_var;
        // Symbol: drake::systems::TransferFunction::time_step
        struct /* time_step */ {
          // Source: drake/systems/primitives/transfer_function.h
          const char* doc = R"""(Returns the time step.)""";
        } time_step;
        // Symbol: drake::systems::TransferFunction::z
        struct /* z */ {
          // Source: drake/systems/primitives/transfer_function.h
          const char* doc =
R"""(Returns a symbolic∷RationalFunction ``s`` which can be used to build a
discrete-time transfer function symbolically.)""";
        } z;
        // Symbol: drake::systems::TransferFunction::z_var
        struct /* z_var */ {
          // Source: drake/systems/primitives/transfer_function.h
          const char* doc =
R"""(Returns the singleton symbolic∷Variable denoting the complex frequency
(σ+jω) used in the Z transform for discrete-time transfer functions.)""";
        } z_var;
      } TransferFunction;
      // Symbol: drake::systems::VectorLog
      struct /* VectorLog */ {
        // Source: drake/systems/primitives/vector_log.h
        const char* doc =
R"""(This utility class serves as an in-memory cache of time-dependent
vector values. Note that this is a standalone class, not a Drake
System. It is primarily intended to support the Drake System primitive
VectorLogSink, but can be used independently.

When the log becomes full, adding more data will cause the allocated
space to double in size. If avoiding memory allocation during some
performance-critical phase is desired, clients can call Reserve() to
pre-allocate log storage.

This object imposes no constraints on the stored data. For example,
times passed to AddData() need not be increasing in order of
insertion, values are allowed to be infinite, NaN, etc.)""";
        // Symbol: drake::systems::VectorLog::AddData
        struct /* AddData */ {
          // Source: drake/systems/primitives/vector_log.h
          const char* doc =
R"""(Adds a ``sample`` to the data set with the associated ``time`` value.
The new sample and time are added to the end of the log. No
constraints are imposed on the values of`time` or ``sample``.

Parameter ``time``:
    The time value for this sample.

Parameter ``sample``:
    A vector of data of the declared size for this log.)""";
        } AddData;
        // Symbol: drake::systems::VectorLog::Clear
        struct /* Clear */ {
          // Source: drake/systems/primitives/vector_log.h
          const char* doc = R"""(Clears the logged data.)""";
        } Clear;
        // Symbol: drake::systems::VectorLog::Reserve
        struct /* Reserve */ {
          // Source: drake/systems/primitives/vector_log.h
          const char* doc =
R"""(Reserve storage for at least ``capacity`` samples. At construction,
there will be at least ``kDefaultCapacity``; use this method to
reserve more.)""";
        } Reserve;
        // Symbol: drake::systems::VectorLog::VectorLog<T>
        struct /* ctor */ {
          // Source: drake/systems/primitives/vector_log.h
          const char* doc =
R"""(Constructs the vector log.

Parameter ``input_size``:
    Dimension of the per-time step data set.)""";
        } ctor;
        // Symbol: drake::systems::VectorLog::data
        struct /* data */ {
          // Source: drake/systems/primitives/vector_log.h
          const char* doc =
R"""(Accesses the logged data.

The InnerPanel parameter of the return type indicates that the
compiler can assume aligned access to the data.)""";
        } data;
        // Symbol: drake::systems::VectorLog::get_input_size
        struct /* get_input_size */ {
          // Source: drake/systems/primitives/vector_log.h
          const char* doc =
R"""(Reports the size of the log's input vector.)""";
        } get_input_size;
        // Symbol: drake::systems::VectorLog::num_samples
        struct /* num_samples */ {
          // Source: drake/systems/primitives/vector_log.h
          const char* doc =
R"""(Returns the number of samples taken since construction or last
Clear().)""";
        } num_samples;
        // Symbol: drake::systems::VectorLog::sample_times
        struct /* sample_times */ {
          // Source: drake/systems/primitives/vector_log.h
          const char* doc = R"""(Accesses the logged time stamps.)""";
        } sample_times;
      } VectorLog;
      // Symbol: drake::systems::VectorLogSink
      struct /* VectorLogSink */ {
        // Source: drake/systems/primitives/vector_log_sink.h
        const char* doc =
R"""(A discrete sink block which logs its vector-valued input to
per-context memory. This data is then retrievable outside of System
operation, e.g. after a simulation. See the warning below.

The stored log (a VectorLog) holds a large, Eigen matrix for data
storage, where each column corresponds to a data point. The
VectorLogSink saves a data point and the context time whenever it
samples its input.

Warning:
    The logged data MUST NOT be used to modify the behavior of a
    simulation. In technical terms, the log is not stored as System
    State, so should not be considered part of that state. This
    distinction allows the implementation to use ``Publish()`` as the
    event handler, rather than one of the state-modifying handlers.

By default, sampling is performed every time the Simulator completes a
trajectory-advancing substep (that is, via a per-step Publish event),
with the first sample occurring during Simulator∷Initialize(). That
means the samples will generally be unevenly spaced in time. If you
prefer regular sampling, you may optionally specify a "publish period"
in which case sampling occurs periodically, with the first sample
occurring at time 0. Alternatively (not common), you can specify that
logging should only occur at "forced publish" events, meaning at
explicit calls to System∷Publish(). The Simulator's "publish every
time step" option also results in forced publish events, so should be
disabled (the default setting) if you want to control logging
yourself.

See also:
    LogVectorOutput() for a convenient way to add logging to a
    Diagram.

.. pydrake_system::

    name: VectorLogSink
    input_ports:
    - data)""";
        // Symbol: drake::systems::VectorLogSink::FindLog
        struct /* FindLog */ {
          // Source: drake/systems/primitives/vector_log_sink.h
          const char* doc =
R"""(Access the log within a containing root context.

Raises:
    RuntimeError if supplied context is not a root context, or was not
    created for the containing diagram.)""";
        } FindLog;
        // Symbol: drake::systems::VectorLogSink::FindMutableLog
        struct /* FindMutableLog */ {
          // Source: drake/systems/primitives/vector_log_sink.h
          const char* doc =
R"""(Access the log as a mutable object within a containing root context.

Raises:
    RuntimeError if supplied context is not a root context, or was not
    created for the containing diagram.)""";
        } FindMutableLog;
        // Symbol: drake::systems::VectorLogSink::GetLog
        struct /* GetLog */ {
          // Source: drake/systems/primitives/vector_log_sink.h
          const char* doc =
R"""(Access the log within this component's context.

Raises:
    RuntimeError if context was not created for this system.)""";
        } GetLog;
        // Symbol: drake::systems::VectorLogSink::GetMutableLog
        struct /* GetMutableLog */ {
          // Source: drake/systems/primitives/vector_log_sink.h
          const char* doc =
R"""(Access the log as a mutable object within this component's context.

Raises:
    RuntimeError if context was not created for this system.)""";
        } GetMutableLog;
        // Symbol: drake::systems::VectorLogSink::VectorLogSink<T>
        struct /* ctor */ {
          // Source: drake/systems/primitives/vector_log_sink.h
          const char* doc_2args =
R"""(Constructs the vector log sink.

Sets the default set of publish triggers: if publish_period = 0,
publishes on forced events and per step, if publish_period > 0,
publishes on forced events and periodically.

Parameter ``input_size``:
    Dimension of the (single) input port. This corresponds to the
    number of rows of the data matrix.

Parameter ``publish_period``:
    Period that messages will be published (optional). If the publish
    period is zero or not supplied, VectorLogSink will use per-step
    publishing instead; see LeafSystem∷DeclarePerStepPublishEvent().

Precondition:
    publish_period is non-negative.

See also:
    LogVectorOutput() helper function for a convenient way to add
    logging.)""";
          // Source: drake/systems/primitives/vector_log_sink.h
          const char* doc_3args =
R"""(Constructs the vector log sink with a specified set of publish
triggers.

Parameter ``input_size``:
    Dimension of the (single) input port. This corresponds to the
    number of rows of the data matrix.

Parameter ``publish_triggers``:
    Set of triggers that determine when messages will be published.
    Supported TriggerTypes are {kForced, kPeriodic, kPerStep}. Will
    throw an error if empty or if unsupported types are provided.

Parameter ``publish_period``:
    Period that messages will be published (optional). publish_period
    should only be non-zero if one of the publish_triggers is
    kPeriodic.

Precondition:
    publish_period is non-negative.

Precondition:
    publish_period > 0 if and only if publish_triggers contains
    kPeriodic.

See also:
    LogVectorOutput() helper function for a convenient way to add
    logging.)""";
          // Source: drake/systems/primitives/vector_log_sink.h
          const char* doc_copyconvert =
R"""(Scalar-converting copy constructor. See system_scalar_conversion.)""";
        } ctor;
      } VectorLogSink;
      // Symbol: drake::systems::WrapToSystem
      struct /* WrapToSystem */ {
        // Source: drake/systems/primitives/wrap_to_system.h
        const char* doc =
R"""(An element-wise wrapping block that transforms the specified indices
of the input signal ``u`` into the interval ``[low, high)``.
Precisely, the output element ``i`` is given the value:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    outputᵢ = inputᵢ + kᵢ*(highᵢ-lowᵢ)

.. raw:: html

    </details>

for the unique integer value ``kᵢ`` that lands the output in the
desired interval.

.. pydrake_system::

    name : WrapToSystem
    input_ports:
    - u0
    output_ports:
    - y0)""";
        // Symbol: drake::systems::WrapToSystem::WrapToSystem<T>
        struct /* ctor */ {
          // Source: drake/systems/primitives/wrap_to_system.h
          const char* doc =
R"""(Constructs a system to pass through a fixed-size input vector to the
output. Additional calls to set_interval() are required to produce any
wrapping behavior.)""";
        } ctor;
        // Symbol: drake::systems::WrapToSystem::get_size
        struct /* get_size */ {
          // Source: drake/systems/primitives/wrap_to_system.h
          const char* doc = R"""(Returns the size.)""";
        } get_size;
        // Symbol: drake::systems::WrapToSystem::set_interval
        struct /* set_interval */ {
          // Source: drake/systems/primitives/wrap_to_system.h
          const char* doc =
R"""(Sets the system to wrap the ``index`` element of the input vector to
the interval ``[low, high)``. If this method is called multiple times
for the same index, then only the last interval will be used. ``low``
and ``high`` should be finite, and low < high.)""";
        } set_interval;
      } WrapToSystem;
      // Symbol: drake::systems::ZeroOrderHold
      struct /* ZeroOrderHold */ {
        // Source: drake/systems/primitives/zero_order_hold.h
        const char* doc =
R"""(A zero order hold block with input u, which may be vector-valued
(discrete or continuous) or abstract, and discrete output y, where the
y is sampled from u with a fixed period (and optional offset).

.. pydrake_system::

    name: ZeroOrderHold
    input_ports:
    - u
    output_ports:
    - y

The discrete state space dynamics of ZeroOrderHold is:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    xₙ₊₁ = uₙ     // update
      yₙ   = xₙ     // output
      x₀   = xᵢₙᵢₜ  // initialize

.. raw:: html

    </details>

where xᵢₙᵢₜ = 0 for vector-valued ZeroOrderHold, and xᵢₙᵢₜ is a given
value for abstract-valued ZeroOrderHold. Use SetVectorState() to set
xₙ in the context for vector-valued ZeroOrderHold.

See discrete_systems "Discrete Systems" for general information about
discrete systems in Drake, including how they interact with continuous
systems.

Note:
    This system defaults to a periodic update with zero offset, in
    which case the first update occurs at t=0. When used with a
    Simulator, the output port is equal to xᵢₙᵢₜ after
    simulator.Initialize(), but is immediately updated to u₀ at the
    start of the first step. If you want to force that initial update,
    use simulator.AdvanceTo(0.0).

Note:
    For an abstract-valued ZeroOrderHold, scalar-type conversion is
    not supported since AbstractValue does not support it.)""";
        // Symbol: drake::systems::ZeroOrderHold::LatchInputPortToState
        struct /* LatchInputPortToState */ {
          // Source: drake/systems/primitives/zero_order_hold.h
          const char* doc =
R"""((Advanced) Manually sample the input port and copy ("latch") the value
into the state. This emulates an update event and is mostly useful for
testing.)""";
        } LatchInputPortToState;
        // Symbol: drake::systems::ZeroOrderHold::SetVectorState
        struct /* SetVectorState */ {
          // Source: drake/systems/primitives/zero_order_hold.h
          const char* doc =
R"""(Sets the value of the state by modifying it in the context. ``value``
must be a column vector of the appropriate size. This can only be used
to initialize a vector-valued state.)""";
        } SetVectorState;
        // Symbol: drake::systems::ZeroOrderHold::ZeroOrderHold<T>
        struct /* ctor */ {
          // Source: drake/systems/primitives/zero_order_hold.h
          const char* doc_3args_period_sec_vector_size_offset_sec =
R"""(Constructs a ZeroOrderHold system with the given ``period_sec``, over
a vector-valued input of size ``vector_size``. The default initial
value for this system will be zero. The first update occurs at
t=offset_sec, which must be >= 0.)""";
          // Source: drake/systems/primitives/zero_order_hold.h
          const char* doc_3args_period_sec_abstract_model_value_offset_sec =
R"""(Constructs a ZeroOrderHold system with the given ``period_sec``, over
a abstract-valued input ``abstract_model_value``. The default initial
value for this system will be ``abstract_model_value``. The first
update occurs at t=offset_sec, which must be >= 0.)""";
          // Source: drake/systems/primitives/zero_order_hold.h
          const char* doc_copyconvert =
R"""(Scalar-type converting copy constructor. See system_scalar_conversion.)""";
        } ctor;
        // Symbol: drake::systems::ZeroOrderHold::offset
        struct /* offset */ {
          // Source: drake/systems/primitives/zero_order_hold.h
          const char* doc =
R"""(Reports the first update time of this hold (in seconds).)""";
        } offset;
        // Symbol: drake::systems::ZeroOrderHold::period
        struct /* period */ {
          // Source: drake/systems/primitives/zero_order_hold.h
          const char* doc =
R"""(Reports the period of this hold (in seconds).)""";
        } period;
      } ZeroOrderHold;
      // Symbol: drake::systems::scalar_conversion
      struct /* scalar_conversion */ {
        // Symbol: drake::systems::scalar_conversion::Traits
        struct /* Traits */ {
          // Source: drake/systems/primitives/trajectory_source.h
          const char* doc =
R"""(Spells out the supported scalar conversions for TrajectorySource.)""";
        } Traits;
      } scalar_conversion;
    } systems;
  } drake;
} pydrake_doc_systems_primitives;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
