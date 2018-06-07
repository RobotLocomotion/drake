import unittest
import numpy as np

from pydrake.autodiffutils import AutoDiffXd
from pydrake.symbolic import Expression
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
    ConstantValueSource_,
    ConstantVectorSource, ConstantVectorSource_,
    ControllabilityMatrix,
    ExponentialRandomSource,
    FirstOrderTaylorApproximation,
    GaussianRandomSource,
    Gain, Gain_,
    Integrator, Integrator_,
    IsControllable,
    IsObservable,
    Linearize,
    LinearSystem, LinearSystem_,
    Multiplexer, Multiplexer_,
    ObservabilityMatrix,
    PassThrough, PassThrough_,
    Saturation, Saturation_,
    SignalLogger, SignalLogger_,
    UniformRandomSource,
    WrapToSystem, WrapToSystem_,
    ZeroOrderHold_,
)


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
        self._check_instantiations(Gain_)
        self._check_instantiations(Integrator_)
        self._check_instantiations(LinearSystem_)
        self._check_instantiations(Multiplexer_)
        self._check_instantiations(PassThrough_)
        self._check_instantiations(Saturation_)
        self._check_instantiations(SignalLogger_)
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
        logger = builder.AddSystem(SignalLogger(kSize))
        builder.Connect(source.get_output_port(0),
                        integrator.get_input_port(0))
        builder.Connect(integrator.get_output_port(0),
                        logger.get_input_port(0))

        diagram = builder.Build()
        simulator = Simulator(diagram)

        simulator.StepTo(1)

        t = logger.sample_times()
        x = logger.data()

        self.assertTrue(t.shape[0] > 2)
        self.assertTrue(t.shape[0] == x.shape[1])
        self.assertAlmostEqual(x[0, -1], t[-1]*kValue, places=2)

        logger.reset()

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

    def test_vector_pass_through(self):
        model_value = BasicVector([1., 2, 3])
        system = PassThrough(model_value.size())
        context = system.CreateDefaultContext()
        context.FixInputPort(0, model_value)
        output = system.AllocateOutput(context)
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
        output = system.AllocateOutput(context)
        input_eval = system.EvalAbstractInput(context, 0)
        compare_value(self, input_eval, model_value)
        system.CalcOutput(context, output)
        output_value = output.get_data(0)
        compare_value(self, output_value, model_value)

    def test_gain(self):
        k = 42.
        input_size = 10
        systems = [Gain(k=k, size=input_size),
                   Gain(k=k*np.ones(input_size))]

        for system in systems:
            context = system.CreateDefaultContext()
            output = system.AllocateOutput(context)

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
        output = system.AllocateOutput(context)

        def mytest(input, expected):
            context.FixInputPort(0, BasicVector(input))
            system.CalcOutput(context, output)
            self.assertTrue(np.allclose(output.get_vector_data(
                0).CopyToVector(), expected))

        mytest((-5., 5., 4.), (0., 2., 4.))
        mytest((.4, 0., 3.5), (.4, 0., 3.5))

    def test_wrap_to_system(self):
        system = WrapToSystem(2)
        system.set_interval(1, 1., 2.)
        context = system.CreateDefaultContext()
        output = system.AllocateOutput(context)

        def mytest(input, expected):
            context.FixInputPort(0, BasicVector(input))
            system.CalcOutput(context, output)
            self.assertTrue(np.allclose(output.get_vector_data(
                0).CopyToVector(), expected))

        mytest((-1.5, 0.5), (-1.5, 1.5))
        mytest((.2, .3), (.2, 1.3))

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
            output = mux.AllocateOutput(context)
            num_ports = len(case['data'])
            self.assertEqual(context.get_num_input_ports(), num_ports)
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
