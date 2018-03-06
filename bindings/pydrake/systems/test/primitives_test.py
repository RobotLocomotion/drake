import unittest
import numpy as np

from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import (
    AbstractValue,
    BasicVector,
    DiagramBuilder,
    VectorBase,
)
from pydrake.systems.primitives import (
    Adder,
    AffineSystem,
    ConstantVectorSource,
    Integrator,
    LinearSystem,
    Linearize,
    FirstOrderTaylorApproximation,
    IsControllable,
    ControllabilityMatrix,
    IsObservable,
    ObservabilityMatrix,
    PassThrough,
    Saturation,
    SignalLogger,
    WrapToSystem,
    Multiplexer,
)
from pydrake.systems.test.test_util import (
    MyVector,
)


def compare_value(test, a, b):
    # Compares a vector or abstract value.
    if isinstance(a, VectorBase):
        test.assertTrue(np.allclose(a.get_value(), b.get_value()))
    else:
        test.assertEquals(type(a.get_value()), type(b.get_value()))
        test.assertEquals(a.get_value(), b.get_value())


class TestGeneral(unittest.TestCase):
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

    def test_multiplexer_scalar_inputs(self):
        mux = Multiplexer(num_scalar_inputs=3)
        self.assertEqual(mux.get_output_port(0).size(), 3)
        context = mux.CreateDefaultContext()
        output = mux.AllocateOutput(context)
        self.assertEqual(context.get_num_input_ports(), 3)
        context.FixInputPort(0, BasicVector([5.]))
        context.FixInputPort(1, BasicVector([3.]))
        context.FixInputPort(2, BasicVector([4.]))
        mux.CalcOutput(context, output)
        self.assertTrue(
            np.allclose(output.get_vector_data(0).get_value(), [5., 3., 4.]))

    def test_multiplexer_vector_inputs(self):
        mux = Multiplexer(input_sizes=[2, 3])
        self.assertEqual(mux.get_output_port(0).size(), 5)
        context = mux.CreateDefaultContext()
        output = mux.AllocateOutput(context)
        self.assertEqual(context.get_num_input_ports(), 2)
        context.FixInputPort(0, BasicVector([5., 4.]))
        context.FixInputPort(1, BasicVector([3., 6., 9.]))
        mux.CalcOutput(context, output)
        self.assertTrue(
            np.allclose(output.get_vector_data(0).get_value(),
                        [5., 4., 3., 6., 9.]))

    def test_multiplexer_model_vector(self):
        mux = Multiplexer(model_vector=MyVector(data=[1., 2.]))
        self.assertEqual(mux.get_output_port(0).size(), 2)
        context = mux.CreateDefaultContext()
        output = mux.AllocateOutput(context)
        self.assertEqual(context.get_num_input_ports(), 2)
        context.FixInputPort(0, BasicVector([5.]))
        context.FixInputPort(1, BasicVector([3.]))
        mux.CalcOutput(context, output)
        value = output.get_vector_data(0)
        self.assertTrue(isinstance(value, MyVector))
        self.assertTrue(np.allclose(value.get_value(), [5., 3.]))
