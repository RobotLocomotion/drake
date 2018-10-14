
import unittest
import numpy as np

from pydrake.examples.manipulation_station import StationSimulation
from pydrake.multibody.multibody_tree.multibody_plant import MultibodyPlant


class TestManipulationStation(unittest.TestCase):
    def test_station_simulation(self):
        # Just check the spelling.
        station = StationSimulation(time_step=0.001)
        station.Finalize()
        station.get_mutable_multibody_plant()
        station.get_mutable_scene_graph()
        station.get_controller_plant()

        # Check the setters/getters.
        context = station.CreateDefaultContext()
        q = np.linspace(0.04, 0.6, num=7)
        v = np.linspace(-2.3, 0.5, num=7)
        station.SetIiwaPosition(q, context)
        np.testing.assert_array_equal(q, station.GetIiwaPosition(context))
        station.SetIiwaVelocity(v, context)
        np.testing.assert_array_equal(v, station.GetIiwaVelocity(context))
