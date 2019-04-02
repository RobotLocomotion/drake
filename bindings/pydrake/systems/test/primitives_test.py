import unittest
import numpy as np

from pydrake.autodiffutils import AutoDiffXd
from pydrake.symbolic import Expression, Variable
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import (
    AbstractValue,
    BasicVector,
    DiagramBuilder,
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
    ExponentialRandomSource,
    FirstOrderLowPassFilter,
    FirstOrderTaylorApproximation,
    GaussianRandomSource,
    Gain, Gain_,
    Integrator, Integrator_,
    IsControllable,
    IsObservable,
    Linearize,
    LinearSystem, LinearSystem_,
    LogOutput,
    MatrixGain,
    Multiplexer, Multiplexer_,
    ObservabilityMatrix,
    PassThrough, PassThrough_,
    Saturation, Saturation_,
    SignalLogger, SignalLogger_,
    Sine, Sine_,
    SymbolicVectorSystem, SymbolicVectorSystem_,
    UniformRandomSource,
    TrajectorySource,
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
        self._check_instantiations(Gain_)
        self._check_instantiations(Integrator_)
        self._check_instantiations(LinearSystem_)
        self._check_instantiations(Multiplexer_)
        self._check_instantiations(PassThrough_)
        self._check_instantiations(Saturation_)
        self._check_instantiations(SignalLogger_)
        self._check_instantiations(Sine_)
        self._check_instantiations(SymbolicVectorSystem_)
        self._check_instantiations(WrapToSystem_)
        self._check_instantiations(ZeroOrderHold_)

    def test_signal_logger(self):
        # Log the output of a simple diagram containing a constant
        # source and an integrator.
        builder = DiagramBuilder()
        kValue = 2.4
        source = builder.AddSystem(ConstantVectorSource([kValue]))
        kSize = 1
        integrator = builder.AddSystem(Integrator(kSize))
        logger_per_step = builder.AddSystem(SignalLogger(kSize))
        builder.Connect(source.get_output_port(0),
                        integrator.get_input_port(0))
        builder.Connect(integrator.get_output_port(0),
                        logger_per_step.get_input_port(0))

        # Add a redundant logger via the helper method.
        logger_per_step_2 = LogOutput(integrator.get_output_port(0), builder)

        # Add a periodic logger
        logger_periodic = builder.AddSystem(SignalLogger(kSize))
        kPeriod = 0.1
        logger_periodic.set_publish_period(kPeriod)
        builder.Connect(integrator.get_output_port(0),
                        logger_periodic.get_input_port(0))

        diagram = builder.Build()
        simulator = Simulator(diagram)
        kTime = 1.
        simulator.AdvanceTo(kTime)

        # Verify outputs of the every-step logger
        t = logger_per_step.sample_times()
        x = logger_per_step.data()

        self.assertTrue(t.shape[0] > 2)
        self.assertTrue(t.shape[0] == x.shape[1])
        self.assertAlmostEqual(x[0, -1], t[-1]*kValue, places=2)
        np.testing.assert_array_equal(x, logger_per_step_2.data())

        # Verify outputs of the periodic logger
        t = logger_periodic.sample_times()
        x = logger_periodic.data()
        # Should log exactly once every kPeriod, up to and including kTime.
        self.assertTrue(t.shape[0] == np.floor(kTime / kPeriod) + 1.)

        logger_per_step.reset()

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

        Co = ControllabilityMatrix(system)
        self.assertEqual(Co.shape, (2, 2))
        self.assertFalse(IsControllable(system))
        self.assertFalse(IsControllable(system, 1e-6))
        Ob = ObservabilityMatrix(system)
        self.assertEqual(Ob.shape, (2, 2))
        self.assertFalse(IsObservable(system))

        system = AffineSystem(A, B, f0, C, D, y0, .1)
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

        context.FixInputPort(0, BasicVector([0]))
        linearized = Linearize(system, context)
        self.assertTrue((linearized.A() == A).all())
        taylor = FirstOrderTaylorApproximation(system, context)
        self.assertTrue((taylor.y0() == y0).all())

        system = MatrixGain(D=A)
        self.assertTrue((system.D() == A).all())

    def test_vector_pass_through(self):
        model_value = BasicVector([1., 2, 3])
        system = PassThrough(model_value.size())
        context = system.CreateDefaultContext()
        context.FixInputPort(0, model_value)
        output = system.AllocateOutput()
        input_eval = system.EvalVectorInput(context, 0)
        compare_value(self, input_eval, model_value)
        system.CalcOutput(context, output)
        output_value = output.get_vector_data(0)
        compare_value(self, output_value, model_value)

    def test_abstract_pass_through(self):
        model_value = AbstractValue.Make("Hello world")
        system = PassThrough(model_value)
        context = system.CreateDefaultContext()
        context.FixInputPort(0, model_value)
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
                context.FixInputPort(0, BasicVector(input))
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
            context.FixInputPort(0, BasicVector(input))
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

    def test_wrap_to_system(self):
        system = WrapToSystem(2)
        system.set_interval(1, 1., 2.)
        context = system.CreateDefaultContext()
        output = system.AllocateOutput()

        def mytest(input, expected):
            context.FixInputPort(0, BasicVector(input))
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

        input_vec = np.array([1., 2., 3., 4.])
        context.FixInputPort(0, BasicVector(input_vec))
        output = demux.AllocateOutput()
        demux.CalcOutput(context, output)

        for i in range(4):
            self.assertTrue(
                np.allclose(output.get_vector_data(i).get_value(),
                            input_vec[i]))

        # Test demultiplexer with vector outputs.
        demux = Demultiplexer(size=4, output_ports_sizes=2)
        context = demux.CreateDefaultContext()
        self.assertEqual(demux.num_input_ports(), 1)
        self.assertEqual(demux.num_output_ports(), 2)

        context.FixInputPort(0, BasicVector(input_vec))
        output = demux.AllocateOutput()
        demux.CalcOutput(context, output)

        for i in range(2):
            self.assertTrue(
                np.allclose(output.get_vector_data(i).get_value(),
                            input_vec[2*i:2*i+2]))

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
                context.FixInputPort(j, BasicVector(vec))
            mux.CalcOutput(context, output)
            self.assertTrue(
                np.allclose(output.get_vector_data(0).get_value(),
                            [elem for vec in case['data'] for elem in vec]))
            if case['has_vector']:
                # Check the type matches MyVector2.
                value = output.get_vector_data(0)
                self.assertTrue(isinstance(value, MyVector2))

    def test_random_sources(self):
        uniform_source = UniformRandomSource(num_outputs=2,
                                             sampling_interval_sec=0.01)
        self.assertEqual(uniform_source.get_output_port(0).size(), 2)

        gaussian_source = GaussianRandomSource(num_outputs=3,
                                               sampling_interval_sec=0.01)
        self.assertEqual(gaussian_source.get_output_port(0).size(), 3)

        exponential_source = ExponentialRandomSource(num_outputs=4,
                                                     sampling_interval_sec=0.1)
        self.assertEqual(exponential_source.get_output_port(0).size(), 4)

        builder = DiagramBuilder()
        # Note: There are no random inputs to add to the empty diagram, but it
        # confirms the API works.
        AddRandomInputs(sampling_interval_sec=0.01, builder=builder)

    def test_ctor_api(self):
        """Tests construction of systems for systems whose executions semantics
        are not tested above.
        """
        ConstantValueSource(AbstractValue.Make("Hello world"))
        ConstantVectorSource(source_value=[1., 2.])
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
