# -*- coding: utf-8 -*-

import unittest

from pydrake.common import RandomGenerator
from pydrake.systems.analysis import (
    MonteCarloSimulation,
    RandomSimulation,
    RandomSimulationResult,
    Simulator,
)
from pydrake.systems.primitives import ConstantVectorSource


class TestMonteCarlo(unittest.TestCase):
    def test_minimal_simulation(self):
        # Create a simple system.
        system = ConstantVectorSource([1.0])

        def make_simulator(generator):
            simulator = Simulator(system)
            simulator.Initialize()
            simulator.set_target_realtime_rate(0)
            return simulator

        def calc_output(system, context):
            return 42.0

        result = RandomSimulation(
            make_simulator=make_simulator,
            output=calc_output,
            final_time=1.0,
            generator=RandomGenerator(),
        )

        self.assertEqual(result, 42.0)

        result = MonteCarloSimulation(
            make_simulator=make_simulator,
            output=calc_output,
            final_time=1.0,
            num_samples=10,
            generator=RandomGenerator(),
        )
        self.assertIsInstance(result, list)
        self.assertEqual(len(result), 10)
        self.assertIsInstance(result[0], RandomSimulationResult)
        self.assertIsInstance(result[0].generator_snapshot, RandomGenerator)
        self.assertEqual(result[0].output, 42.0)
        for i in range(1, len(result)):
            self.assertIsNot(
                result[0].generator_snapshot, result[i].generator_snapshot
            )
