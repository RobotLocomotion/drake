
import unittest
import numpy as np

from pydrake.systems.primitives import (
    Adder,
    AffineSystem,
    ConstantVectorSource,
    Integrator,
    LinearSystem,
    SignalLogger,
)


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

    def test_linear_affine_system(self):
        # Just make sure linear system is spelled correctly.
        A = np.identity(2)
        B = np.array([[0], [1]])
        C = np.array([[0, 1]])
        D = [0]
        f0 = np.array([[0], [0]])
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

        system = AffineSystem(A, B, f0, C, D, y0, .1)
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
        self.assertEqual(system.time_period(), .1)

        linearized = pydrake.systems.primitives.Linearize(system)
        self.assertTrue((linearized.A() == A).all())
        taylor = pydrake.systems.primitives.FirstOrderTaylor(system)
        self.assertTrue((taylor.y0() == y0).all())

        Co = pydrake.systems.primitives.ControllabilityMatrix(system)
        self.assertEqual(Co.shape[0], 2)
        self.assertEqual(Co.shape[1], 2)
        self.assertTrue(pydrake.systems.primitives.IsControllable(system))
        Ob = pydrake.systems.primitives.ObservabilityMatrix(system)
        self.assertEqual(Ob.shape[0], 2)
        self.assertEqual(Ob.shape[1], 2)
        self.assertFalse(pydrake.systems.primitives.IsObservable(system))
