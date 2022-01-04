import gc
import unittest
import numpy as np

from pydrake.autodiffutils import AutoDiffXd
from pydrake.common import RandomDistribution, RandomGenerator
from pydrake.common.test_utilities import numpy_compare
from pydrake.common.test_utilities.deprecation import catch_drake_warnings
from pydrake.common.value import AbstractValue
from pydrake.symbolic import Expression, Variable
from pydrake.systems.analysis import (
    Simulator,
    Simulator_,
)
from pydrake.systems.framework import (
    BasicVector,
    DiagramBuilder,
    DiagramBuilder_,
    TriggerType,
    VectorBase,
)
from pydrake.systems.test.test_util import (
    MyVector2,
)
from pydrake.systems.primitives import (
    Adder, Adder_,
    AddRandomInputs,
    AffineSystem, AffineSystem_,
    ConstantValueSource, ConstantValueSource_,
    ConstantVectorSource, ConstantVectorSource_,
    ControllabilityMatrix,
    Demultiplexer, Demultiplexer_,
    DiscreteDerivative, DiscreteDerivative_,
    DiscreteTimeDelay, DiscreteTimeDelay_,
    FirstOrderLowPassFilter,
    FirstOrderTaylorApproximation,
    Gain, Gain_,
    Integrator, Integrator_,
    IsControllable,
    IsObservable,
    Linearize,
    LinearSystem, LinearSystem_,
    LinearTransformDensity, LinearTransformDensity_,
    LogVectorOutput,
    MatrixGain,
    Multiplexer, Multiplexer_,
    ObservabilityMatrix,
    PassThrough, PassThrough_,
    RandomSource,
    Saturation, Saturation_,
    Sine, Sine_,
    StateInterpolatorWithDiscreteDerivative,
    StateInterpolatorWithDiscreteDerivative_,
    SymbolicVectorSystem, SymbolicVectorSystem_,
    TrajectoryAffineSystem, TrajectoryAffineSystem_,
    TrajectoryLinearSystem, TrajectoryLinearSystem_,
    TrajectorySource,
    VectorLog, VectorLogSink, VectorLogSink_,
    WrapToSystem, WrapToSystem_,
    ZeroOrderHold, ZeroOrderHold_,
)
from pydrake.trajectories import PiecewisePolynomial


def compare_value(test, a, b):
    # Compares a vector or abstract value.
    if isinstance(a, VectorBase):
        test.assertTrue(np.allclose(a.get_value(), b.get_value()))
    else:
        test.assertEqual(type(a.get_value()), type(b.get_value()))
        test.assertEqual(a.get_value(), b.get_value())


class TestGeneral(unittest.TestCase):
    def _check_instantiations(self, template, supports_symbolic=True):
        default_cls = template[None]
        self.assertTrue(template[float] is default_cls)
        self.assertTrue(template[AutoDiffXd] is not default_cls)
        if supports_symbolic:
            self.assertTrue(template[Expression] is not default_cls)

    def test_instantiations(self):
        # TODO(eric.cousineau): Refine tests once NumPy functionality is
        # resolved for dtype=object, or dtype=custom is used.
        self._check_instantiations(Adder_)
        self._check_instantiations(AffineSystem_)
        self._check_instantiations(ConstantValueSource_)
        self._check_instantiations(ConstantVectorSource_)
        self._check_instantiations(Demultiplexer_)
        self._check_instantiations(DiscreteDerivative_)
        self._check_instantiations(DiscreteTimeDelay_)
        self._check_instantiations(Gain_)
        self._check_instantiations(Integrator_)
        self._check_instantiations(LinearSystem_)
        self._check_instantiations(LinearTransformDensity_,
                                   supports_symbolic=False)
        self._check_instantiations(Multiplexer_)
        self._check_instantiations(PassThrough_)
        self._check_instantiations(Saturation_)
        self._check_instantiations(Sine_)
        self._check_instantiations(StateInterpolatorWithDiscreteDerivative_)
        self._check_instantiations(SymbolicVectorSystem_)
        self._check_instantiations(TrajectoryAffineSystem_,
                                   supports_symbolic=False)
        self._check_instantiations(TrajectoryLinearSystem_,
                                   supports_symbolic=False)
        self._check_instantiations(VectorLogSink_)
        self._check_instantiations(WrapToSystem_)
        self._check_instantiations(ZeroOrderHold_)

    def test_linear_affine_system(self):
        # Just make sure linear system is spelled correctly.
        A = np.identity(2)
        B = np.array([[0], [1]])
        f0 = np.array([[0], [0]])
        C = np.array([[0, 1]])
        D = [0]
        y0 = [0]
        system = LinearSystem(A, B, C, D)
        context = system.CreateDefaultContext()
        self.assertEqual(system.get_input_port(0).size(), 1)
        self.assertEqual(context
                         .get_mutable_continuous_state_vector().size(), 2)
        self.assertEqual(system.get_output_port(0).size(), 1)
        self.assertTrue((system.A() == A).all())
        self.assertTrue((system.B() == B).all())
        self.assertTrue((system.f0() == f0).all())
        self.assertTrue((system.C() == C).all())
        self.assertEqual(system.D(), D)
        self.assertEqual(system.y0(), y0)
        self.assertEqual(system.time_period(), 0.)

        x0 = np.array([1, 2])
        system.configure_default_state(x0=x0)
        system.SetDefaultContext(context)
        np.testing.assert_equal(
            context.get_continuous_state_vector().CopyToVector(), x0)
        generator = RandomGenerator()
        system.SetRandomContext(context, generator)
        np.testing.assert_equal(
            context.get_continuous_state_vector().CopyToVector(), x0)
        system.configure_random_state(covariance=np.eye(2))
        system.SetRandomContext(context, generator)
        self.assertNotEqual(
            context.get_continuous_state_vector().CopyToVector()[1], x0[1])

        Co = ControllabilityMatrix(system)
        self.assertEqual(Co.shape, (2, 2))
        self.assertFalse(IsControllable(system))
        self.assertFalse(IsControllable(system, 1e-6))
        Ob = ObservabilityMatrix(system)
        self.assertEqual(Ob.shape, (2, 2))
        self.assertFalse(IsObservable(system))

        system = AffineSystem(A, B, f0, C, D, y0, .1)
        self.assertEqual(system.get_input_port(0), system.get_input_port())
        self.assertEqual(system.get_output_port(0), system.get_output_port())
        context = system.CreateDefaultContext()
        self.assertEqual(system.get_input_port(0).size(), 1)
        self.assertEqual(context.get_discrete_state_vector().size(), 2)
        self.assertEqual(system.get_output_port(0).size(), 1)
        self.assertTrue((system.A() == A).all())
        self.assertTrue((system.B() == B).all())
        self.assertTrue((system.f0() == f0).all())
        self.assertTrue((system.C() == C).all())
        self.assertEqual(system.D(), D)
        self.assertEqual(system.y0(), y0)
        self.assertEqual(system.time_period(), .1)

        system.get_input_port(0).FixValue(context, 0)
        linearized = Linearize(system, context)
        self.assertTrue((linearized.A() == A).all())
        taylor = FirstOrderTaylorApproximation(system, context)
        self.assertTrue((taylor.y0() == y0).all())

        system = MatrixGain(D=A)
        self.assertTrue((system.D() == A).all())

        system = TrajectoryAffineSystem(
            PiecewisePolynomial(A),
            PiecewisePolynomial(B),
            PiecewisePolynomial(f0),
            PiecewisePolynomial(C),
            PiecewisePolynomial(D),
            PiecewisePolynomial(y0),
            .1)
        self.assertEqual(system.get_input_port(0), system.get_input_port())
        self.assertEqual(system.get_output_port(0), system.get_output_port())
        context = system.CreateDefaultContext()
        self.assertEqual(system.get_input_port(0).size(), 1)
        self.assertEqual(context.get_discrete_state_vector().size(), 2)
        self.assertEqual(system.get_output_port(0).size(), 1)
        for t in np.linspace(0., 1., 5):
            self.assertTrue((system.A(t) == A).all())
            self.assertTrue((system.B(t) == B).all())
            self.assertTrue((system.f0(t) == f0).all())
            self.assertTrue((system.C(t) == C).all())
            self.assertEqual(system.D(t), D)
            self.assertEqual(system.y0(t), y0)
        self.assertEqual(system.time_period(), .1)
        x0 = np.array([1, 2])
        system.configure_default_state(x0=x0)
        system.SetDefaultContext(context)
        np.testing.assert_equal(
            context.get_discrete_state_vector().CopyToVector(), x0)
        generator = RandomGenerator()
        system.SetRandomContext(context, generator)
        np.testing.assert_equal(
            context.get_discrete_state_vector().CopyToVector(), x0)
        system.configure_random_state(covariance=np.eye(2))
        system.SetRandomContext(context, generator)
        self.assertNotEqual(
            context.get_discrete_state_vector().CopyToVector()[1], x0[1])

        system = TrajectoryLinearSystem(
            A=PiecewisePolynomial(A),
            B=PiecewisePolynomial(B),
            C=PiecewisePolynomial(C),
            D=PiecewisePolynomial(D),
            time_period=0.1)
        self.assertEqual(system.time_period(), .1)
        system.configure_default_state(x0=np.array([1, 2]))
        system.configure_random_state(covariance=np.eye(2))

    def test_linear_system_zero_size(self):
        # Explicitly test #12633.
        num_x = 0
        num_y = 2
        num_u = 2
        A = np.zeros((num_x, num_x))
        B = np.zeros((num_x, num_u))
        C = np.zeros((num_y, num_x))
        D = np.zeros((num_y, num_u))
        self.assertIsNotNone(LinearSystem(A, B, C, D))

    @numpy_compare.check_nonsymbolic_types
    def test_linear_transform_density(self, T):
        dut = LinearTransformDensity_[T](
            distribution=RandomDistribution.kGaussian,
            input_size=3,
            output_size=3)
        w_in = np.array([T(0.5), T(0.1), T(1.5)])
        context = dut.CreateDefaultContext()
        dut.get_input_port_w_in().FixValue(context, w_in)
        self.assertEqual(dut.get_input_port_A().size(), 9)
        self.assertEqual(dut.get_input_port_b().size(), 3)
        self.assertEqual(dut.get_distribution(), RandomDistribution.kGaussian)
        A = np.array([
            [T(0.5), T(1), T(2)], [T(1), T(2), T(3)], [T(3), T(4), T(5)]])
        dut.FixConstantA(context=context, A=A)
        b = np.array([T(1), T(2), T(3)])
        dut.FixConstantB(context=context, b=b)

        dut.CalcDensity(context=context)

        self.assertEqual(dut.get_output_port_w_out().size(), 3)
        self.assertEqual(dut.get_output_port_w_out_density().size(), 1)

    def test_vector_pass_through(self):
        model_value = BasicVector([1., 2, 3])
        system = PassThrough(vector_size=model_value.size())
        context = system.CreateDefaultContext()
        system.get_input_port(0).FixValue(context, model_value)
        output = system.AllocateOutput()
        input_eval = system.EvalVectorInput(context, 0)
        compare_value(self, input_eval, model_value)
        system.CalcOutput(context, output)
        output_value = output.get_vector_data(0)
        compare_value(self, output_value, model_value)

    def test_default_vector_pass_through(self):
        model_value = [1., 2, 3]
        system = PassThrough(value=model_value)
        context = system.CreateDefaultContext()
        np.testing.assert_array_equal(
            model_value, system.get_output_port().Eval(context))

    def test_abstract_pass_through(self):
        model_value = AbstractValue.Make("Hello world")
        system = PassThrough(abstract_model_value=model_value)
        context = system.CreateDefaultContext()
        system.get_input_port(0).FixValue(context, model_value)
        output = system.AllocateOutput()
        input_eval = system.EvalAbstractInput(context, 0)
        compare_value(self, input_eval, model_value)
        system.CalcOutput(context, output)
        output_value = output.get_data(0)
        compare_value(self, output_value, model_value)

    def test_first_order_low_pass_filter(self):
        filter1 = FirstOrderLowPassFilter(time_constant=3.0, size=4)
        self.assertEqual(filter1.get_time_constant(), 3.0)

        alpha = np.array([1, 2, 3])
        filter2 = FirstOrderLowPassFilter(time_constants=alpha)
        np.testing.assert_array_equal(filter2.get_time_constants_vector(),
                                      alpha)

        context = filter2.CreateDefaultContext()
        filter2.set_initial_output_value(context, [0., -0.2, 0.4])

    def test_gain(self):
        k = 42.
        input_size = 10
        systems = [Gain(k=k, size=input_size),
                   Gain(k=k*np.ones(input_size))]

        for system in systems:
            context = system.CreateDefaultContext()
            output = system.AllocateOutput()

            def mytest(input, expected):
                system.get_input_port(0).FixValue(context, input)
                system.CalcOutput(context, output)
                self.assertTrue(np.allclose(output.get_vector_data(
                    0).CopyToVector(), expected))

            test_input = np.arange(input_size)
            mytest(np.arange(input_size), k*np.arange(input_size))

    def test_saturation(self):
        system = Saturation((0., -1., 3.), (1., 2., 4.))
        context = system.CreateDefaultContext()
        output = system.AllocateOutput()

        def mytest(input, expected):
            system.get_input_port(0).FixValue(context, input)
            system.CalcOutput(context, output)
            self.assertTrue(np.allclose(output.get_vector_data(
                0).CopyToVector(), expected))

        mytest((-5., 5., 4.), (0., 2., 4.))
        mytest((.4, 0., 3.5), (.4, 0., 3.5))

    def test_trajectory_source(self):
        ppt = PiecewisePolynomial.FirstOrderHold(
            [0., 1.], [[2., 3.], [2., 1.]])
        system = TrajectorySource(trajectory=ppt,
                                  output_derivative_order=0,
                                  zero_derivatives_beyond_limits=True)
        context = system.CreateDefaultContext()
        output = system.AllocateOutput()

        def mytest(input, expected):
            context.SetTime(input)
            system.CalcOutput(context, output)
            self.assertTrue(np.allclose(output.get_vector_data(
                0).CopyToVector(), expected))

        mytest(0.0, (2.0, 2.0))
        mytest(0.5, (2.5, 1.5))
        mytest(1.0, (3.0, 1.0))

    def test_symbolic_vector_system(self):
        t = Variable("t")
        x = [Variable("x0"), Variable("x1")]
        u = [Variable("u0"), Variable("u1")]
        system = SymbolicVectorSystem(time=t, state=x, input=u,
                                      dynamics=[x[0] + x[1], t],
                                      output=[u[1]],
                                      time_period=0.0)
        context = system.CreateDefaultContext()

        self.assertEqual(context.num_continuous_states(), 2)
        self.assertEqual(context.num_discrete_state_groups(), 0)
        self.assertEqual(system.get_input_port(0).size(), 2)
        self.assertEqual(system.get_output_port(0).size(), 1)
        self.assertEqual(context.num_abstract_parameters(), 0)
        self.assertEqual(context.num_numeric_parameter_groups(), 0)
        self.assertTrue(system.dynamics_for_variable(x[0])
                        .EqualTo(x[0] + x[1]))
        self.assertTrue(system.dynamics_for_variable(x[1])
                        .EqualTo(t))

    def test_symbolic_vector_system_parameters(self):
        t = Variable("t")
        x = [Variable("x0"), Variable("x1")]
        u = [Variable("u0"), Variable("u1")]
        p = [Variable("p0"), Variable("p1")]
        system = SymbolicVectorSystem(time=t, state=x, input=u,
                                      parameter=p,
                                      dynamics=[p[0] * x[0] + x[1] + p[1], t],
                                      output=[u[1]],
                                      time_period=0.0)
        context = system.CreateDefaultContext()

        self.assertEqual(context.num_continuous_states(), 2)
        self.assertEqual(context.num_discrete_state_groups(), 0)
        self.assertEqual(system.get_input_port(0).size(), 2)
        self.assertEqual(system.get_output_port(0).size(), 1)
        self.assertEqual(context.num_abstract_parameters(), 0)
        self.assertEqual(context.num_numeric_parameter_groups(), 1)
        self.assertEqual(context.get_numeric_parameter(0).size(), 2)
        self.assertTrue(system.dynamics_for_variable(x[0])
                        .EqualTo(p[0] * x[0] + x[1] + p[1]))
        self.assertTrue(system.dynamics_for_variable(x[1])
                        .EqualTo(t))

    def test_wrap_to_system(self):
        system = WrapToSystem(2)
        system.set_interval(1, 1., 2.)
        context = system.CreateDefaultContext()
        output = system.AllocateOutput()

        def mytest(input, expected):
            system.get_input_port(0).FixValue(context, input)
            system.CalcOutput(context, output)
            self.assertTrue(np.allclose(output.get_vector_data(
                0).CopyToVector(), expected))

        mytest((-1.5, 0.5), (-1.5, 1.5))
        mytest((.2, .3), (.2, 1.3))

    def test_demultiplexer(self):
        # Test demultiplexer with scalar outputs.
        demux = Demultiplexer(size=4)
        context = demux.CreateDefaultContext()
        self.assertEqual(demux.num_input_ports(), 1)
        self.assertEqual(demux.num_output_ports(), 4)
        numpy_compare.assert_equal(demux.get_output_ports_sizes(),
                                   [1, 1, 1, 1])

        input_vec = np.array([1., 2., 3., 4.])
        demux.get_input_port(0).FixValue(context, input_vec)
        output = demux.AllocateOutput()
        demux.CalcOutput(context, output)

        for i in range(4):
            self.assertTrue(
                np.allclose(output.get_vector_data(i).get_value(),
                            input_vec[i]))

        # Test demultiplexer with vector outputs.
        demux = Demultiplexer(size=4, output_ports_size=2)
        context = demux.CreateDefaultContext()
        self.assertEqual(demux.num_input_ports(), 1)
        self.assertEqual(demux.num_output_ports(), 2)
        numpy_compare.assert_equal(demux.get_output_ports_sizes(), [2, 2])

        demux.get_input_port(0).FixValue(context, input_vec)
        output = demux.AllocateOutput()
        demux.CalcOutput(context, output)

        for i in range(2):
            self.assertTrue(
                np.allclose(output.get_vector_data(i).get_value(),
                            input_vec[2*i:2*i+2]))

        # Test demultiplexer with different output port sizes.
        output_ports_sizes = np.array([1, 2, 1])
        num_output_ports = output_ports_sizes.size
        input_vec = np.array([1., 2., 3., 4.])
        demux = Demultiplexer(output_ports_sizes=output_ports_sizes)
        context = demux.CreateDefaultContext()
        self.assertEqual(demux.num_input_ports(), 1)
        self.assertEqual(demux.num_output_ports(), num_output_ports)
        numpy_compare.assert_equal(demux.get_output_ports_sizes(),
                                   output_ports_sizes)

        demux.get_input_port(0).FixValue(context, input_vec)
        output = demux.AllocateOutput()
        demux.CalcOutput(context, output)

        output_port_start = 0
        for i in range(num_output_ports):
            output_port_size = output.get_vector_data(i).size()
            self.assertTrue(
                np.allclose(output.get_vector_data(i).get_value(),
                            input_vec[output_port_start:
                                      output_port_start+output_port_size]))
            output_port_start += output_port_size

    def test_multiplexer(self):
        my_vector = MyVector2(data=[1., 2.])
        test_cases = [
            dict(has_vector=False, mux=Multiplexer(num_scalar_inputs=4),
                 data=[[5.], [3.], [4.], [2.]]),
            dict(has_vector=False, mux=Multiplexer(input_sizes=[2, 3]),
                 data=[[8., 4.], [3., 6., 9.]]),
            dict(has_vector=True, mux=Multiplexer(model_vector=my_vector),
                 data=[[42.], [3.]]),
        ]
        for case in test_cases:
            mux = case['mux']
            port_size = sum([len(vec) for vec in case['data']])
            self.assertEqual(mux.get_output_port(0).size(), port_size)
            context = mux.CreateDefaultContext()
            output = mux.AllocateOutput()
            num_ports = len(case['data'])
            self.assertEqual(context.num_input_ports(), num_ports)
            for j, vec in enumerate(case['data']):
                mux.get_input_port(j).FixValue(context, vec)
            mux.CalcOutput(context, output)
            self.assertTrue(
                np.allclose(output.get_vector_data(0).get_value(),
                            [elem for vec in case['data'] for elem in vec]))
            if case['has_vector']:
                # Check the type matches MyVector2.
                value = output.get_vector_data(0)
                self.assertTrue(isinstance(value, MyVector2))

    def test_random_source(self):
        source = RandomSource(distribution=RandomDistribution.kUniform,
                              num_outputs=2, sampling_interval_sec=0.01)
        self.assertEqual(source.get_output_port(0).size(), 2)

        builder = DiagramBuilder()
        # Note: There are no random inputs to add to the empty diagram, but it
        # confirms the API works.
        AddRandomInputs(sampling_interval_sec=0.01, builder=builder)

        builder_ad = DiagramBuilder_[AutoDiffXd]()
        AddRandomInputs(sampling_interval_sec=0.01, builder=builder_ad)

    def test_constant_vector_source(self):
        source = ConstantVectorSource(source_value=[1., 2.])
        context = source.CreateDefaultContext()
        source.get_source_value(context)
        source.get_mutable_source_value(context)

    def test_ctor_api(self):
        """Tests construction of systems for systems whose executions semantics
        are not tested above.
        """
        ConstantValueSource(AbstractValue.Make("Hello world"))
        DiscreteTimeDelay(update_sec=0.1, delay_timesteps=5, vector_size=2)
        DiscreteTimeDelay(
            update_sec=0.1, delay_timesteps=5,
            abstract_model_value=AbstractValue.Make("Hello world"))
        ZeroOrderHold(period_sec=0.1, vector_size=2)
        ZeroOrderHold(
            period_sec=0.1,
            abstract_model_value=AbstractValue.Make("Hello world"))

    def test_sine(self):
        # Test scalar output.
        sine_source = Sine(amplitude=1, frequency=2, phase=3,
                           size=1, is_time_based=True)
        self.assertEqual(sine_source.get_output_port(0).size(), 1)
        self.assertEqual(sine_source.get_output_port(1).size(), 1)
        self.assertEqual(sine_source.get_output_port(2).size(), 1)

        # Test vector output.
        sine_source = Sine(amplitude=1, frequency=2, phase=3,
                           size=3, is_time_based=True)
        self.assertEqual(sine_source.get_output_port(0).size(), 3)
        self.assertEqual(sine_source.get_output_port(1).size(), 3)
        self.assertEqual(sine_source.get_output_port(2).size(), 3)

        sine_source = Sine(amplitudes=np.ones(2), frequencies=np.ones(2),
                           phases=np.ones(2), is_time_based=True)
        self.assertEqual(sine_source.get_output_port(0).size(), 2)
        self.assertEqual(sine_source.get_output_port(1).size(), 2)
        self.assertEqual(sine_source.get_output_port(2).size(), 2)

    def test_discrete_derivative(self):
        discrete_derivative = DiscreteDerivative(num_inputs=5, time_step=0.5)
        self.assertEqual(discrete_derivative.get_input_port(0).size(), 5)
        self.assertEqual(discrete_derivative.get_output_port(0).size(), 5)
        self.assertEqual(discrete_derivative.time_step(), 0.5)
        self.assertFalse(discrete_derivative.suppress_initial_transient())

        discrete_derivative = DiscreteDerivative(
            num_inputs=5, time_step=0.5, suppress_initial_transient=True)
        self.assertTrue(discrete_derivative.suppress_initial_transient())

    def test_state_interpolator_with_discrete_derivative(self):
        state_interpolator = StateInterpolatorWithDiscreteDerivative(
            num_positions=5, time_step=0.4)
        self.assertEqual(state_interpolator.get_input_port(0).size(), 5)
        self.assertEqual(state_interpolator.get_output_port(0).size(), 10)
        self.assertFalse(state_interpolator.suppress_initial_transient())

        # test set_initial_position using context
        context = state_interpolator.CreateDefaultContext()
        state_interpolator.set_initial_position(
            context=context, position=5*[1.1])
        np.testing.assert_array_equal(
            context.get_discrete_state(0).CopyToVector(),
            np.array(5*[1.1]))
        np.testing.assert_array_equal(
            context.get_discrete_state(1).CopyToVector(),
            np.array(5*[1.1]))

        # test set_initial_position using state
        context = state_interpolator.CreateDefaultContext()
        state_interpolator.set_initial_position(
            state=context.get_state(), position=5*[1.3])
        np.testing.assert_array_equal(
            context.get_discrete_state(0).CopyToVector(),
            np.array(5*[1.3]))
        np.testing.assert_array_equal(
            context.get_discrete_state(1).CopyToVector(),
            np.array(5*[1.3]))

        state_interpolator = StateInterpolatorWithDiscreteDerivative(
            num_positions=5, time_step=0.4, suppress_initial_transient=True)
        self.assertTrue(state_interpolator.suppress_initial_transient())

    @numpy_compare.check_nonsymbolic_types
    def test_log_vector_output(self, T):
        # Add various redundant loggers to a system, to exercise the
        # LogVectorOutput bindings.
        builder = DiagramBuilder_[T]()
        kSize = 1
        integrator = builder.AddSystem(Integrator_[T](kSize))
        port = integrator.get_output_port(0)
        loggers = []
        loggers.append(LogVectorOutput(port, builder))
        loggers.append(LogVectorOutput(src=port, builder=builder))
        loggers.append(LogVectorOutput(port, builder, 0.125))
        loggers.append(LogVectorOutput(
            src=port, builder=builder, publish_period=0.125))

        loggers.append(LogVectorOutput(port, builder, {TriggerType.kForced}))
        loggers.append(LogVectorOutput(
            src=port, builder=builder, publish_triggers={TriggerType.kForced}))
        loggers.append(LogVectorOutput(
            port, builder, {TriggerType.kPeriodic}, 0.125))
        loggers.append(LogVectorOutput(
            src=port, builder=builder,
            publish_triggers={TriggerType.kPeriodic}, publish_period=0.125))

        # Check the returned loggers by calling some trivial methods.
        diagram = builder.Build()
        context = diagram.CreateDefaultContext()
        self.assertTrue(all(logger.FindLog(context).num_samples() == 0
                            for logger in loggers))

    @numpy_compare.check_nonsymbolic_types
    def test_vector_log(self, T):
        kSize = 1
        dut = VectorLog(kSize)
        self.assertEqual(dut.get_input_size(), kSize)
        dut.AddData(0.1, [22.22])
        self.assertEqual(dut.num_samples(), 1)
        self.assertEqual(dut.sample_times(), [0.1])
        self.assertEqual(dut.data(), [22.22])
        dut.Clear()
        self.assertEqual(dut.num_samples(), 0)
        # There is no good way from python to test the semantics of Reserve(),
        # but test the binding anyway.
        dut.Reserve(VectorLog.kDefaultCapacity * 3)

    @numpy_compare.check_nonsymbolic_types
    def test_vector_log_sink(self, T):
        # Add various redundant loggers to a system, to exercise the
        # VectorLog constructor bindings.
        builder = DiagramBuilder_[T]()
        kSize = 1
        constructors = [VectorLogSink_[T]]
        loggers = []
        if T == float:
            constructors.append(VectorLogSink)
        for constructor in constructors:
            loggers.append(builder.AddSystem(constructor(kSize)))
            loggers.append(builder.AddSystem(constructor(input_size=kSize)))
            loggers.append(builder.AddSystem(constructor(kSize, 0.125)))
            loggers.append(builder.AddSystem(
                constructor(input_size=kSize, publish_period=0.125)))
            loggers.append(builder.AddSystem(
                constructor(kSize, {TriggerType.kForced})))
            loggers.append(builder.AddSystem(
                constructor(input_size=kSize,
                            publish_triggers={TriggerType.kForced})))
            loggers.append(builder.AddSystem(
                constructor(kSize, {TriggerType.kPeriodic}, 0.125)))
            loggers.append(builder.AddSystem(
                constructor(input_size=kSize,
                            publish_triggers={TriggerType.kPeriodic},
                            publish_period=0.125)))

        # Exercise all of the log access methods.
        diagram = builder.Build()
        context = diagram.CreateDefaultContext()
        # FindLog and FindMutableLog find the same object.
        self.assertTrue(
            all(logger.FindLog(context) == logger.FindMutableLog(context)
                for logger in loggers))
        # Build a list of pairs of loggers and their local contexts.
        loggers_and_contexts = [(x, x.GetMyContextFromRoot(context))
                                for x in loggers]
        # GetLog and GetMutableLog find the same object.
        self.assertTrue(
            all(logger.GetLog(logger_context)
                == logger.GetMutableLog(logger_context)
                for logger, logger_context in loggers_and_contexts))
        # GetLog and FindLog find the same object, given the proper contexts.
        self.assertTrue(
            all(logger.GetLog(logger_context) == logger.FindLog(context)
                for logger, logger_context in loggers_and_contexts))
