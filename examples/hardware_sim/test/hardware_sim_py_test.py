import unittest

from drake.examples.hardware_sim.hardware_sim import Scenario, run
from pydrake.lcm import DrakeLcmParams
from pydrake.manipulation.util import ZeroForceDriver
from pydrake.multibody.parsing import LoadModelDirectivesFromString
from pydrake.systems.sensors import CameraConfig


class HardwareSimTest(unittest.TestCase):
    """Unit tests for hardware_sim.py."""

    def make_add_model_directive(self):
        """Returns a ModelDirective object that specifies an add_model
        directive.
        """
        model_directives = LoadModelDirectivesFromString(R"""directives:
        - add_model:
            name: alice
            file: package://drake/examples/pendulum/Pendulum.urdf
        """)
        return model_directives.directives[0]

    def load_scenario(self):
        """Returns a sample scenario similar to OneOfEverything from the
        test_scenarios.yaml file.
        """
        # TODO(jwnimmer-tri) Actually implement scenario loading.
        # For the moment, we'll just hard-code one.
        scenario = Scenario()
        scenario.random_seed = 1
        scenario.simulation_duration = 3.14
        scenario.simulator_config.target_realtime_rate = 5.0
        scenario.plant_config.stiction_tolerance = 1e-2
        scenario.directives = [self.make_add_model_directive()]
        scenario.lcm_buses["extra_bus"] = DrakeLcmParams()
        scenario.model_drivers["alice"] = ZeroForceDriver()
        scenario.cameras["arbitrary_camera_name"] = CameraConfig()
        scenario.visualization.publish_period = 0.125
        return scenario

    def test_run(self):
        """Checks that the run() function doesn't crash.
        """
        scenario = self.load_scenario()
        scenario.simulation_duration = 0.1
        run(scenario=scenario)
