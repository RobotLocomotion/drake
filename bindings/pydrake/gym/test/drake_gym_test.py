import copy
import unittest

import gymnasium as gym
import numpy as np
import stable_baselines3.common.env_checker

from pydrake.common.value import Value
from pydrake.gym import DrakeGymEnv
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import PassThrough
from pydrake.systems.sensors import ImageRgba8U


class DrakeGymTest(unittest.TestCase):
    """
    Test that a DrakeGymEnv satisfies the OpenAI Gym Env specifications as to
    * API https://www.gymlibrary.ml/content/api/#standard-methods, and
    * semantics https://www.gymlibrary.ml/content/environment_creation/

    Not every Gym optimizer algorithm uses every part of the API (for
    instance none use `reset(seed)` as far as I can tell) but we check them
    anyway because if they ever are used the errors will be hard to find.
    """

    @classmethod
    def setUpClass(cls):
        gym.envs.register(
            id="DrakeCartPole-v0",
            entry_point="pydrake.examples.gym.envs.cart_pole:DrakeCartPoleEnv",
        )

    def make_env(self):
        return gym.make("DrakeCartPole-v0")

    def test_make_env(self):
        self.make_env()

    def test_sb3_check_env(self):
        """Run stable-baselines's built-in test suite for our env."""
        dut = self.make_env()
        stable_baselines3.common.env_checker.check_env(
            env=dut, warn=True, skip_render_check=True
        )

    # TODO(JoseBarreiros-TRI) Add tests for make_vec_env. In our currently
    # supported versions of `gymnasium` and `stable_baselines3`, stable
    # baselines vector envs do not pass stable baselines' `check_env` tests.

    def test_reset(self):
        # reset(int) sets a deterministic seed.
        dut = self.make_env()
        obs1, _ = dut.reset(seed=7)
        obs2, _ = dut.reset(seed=7)
        self.assertTrue((obs1 == obs2).all())

        # reset() on its own gets a new arbitrary seed.
        dut = self.make_env()
        obs1, _ = dut.reset()
        obs2, _ = dut.reset()
        self.assertFalse((obs1 == obs2).all())

        # The difference when reset() follows reset(seed) is not
        # externally observable, so don't test it.

        # return_options changes the return type.
        (observation, opts) = dut.reset()
        self.assertIsInstance(opts, dict)
        self.assertTrue(dut.observation_space.contains(observation))

    def test_step(self):
        dut = self.make_env()
        dut.reset()
        observation, _, _, _, _ = dut.step(dut.action_space.sample())
        self.assertTrue(dut.observation_space.contains(observation))

    def test_none_spaces_default_to_infinite_box(self):
        """Passing None for vector-valued ports builds ±inf Boxes."""
        size = 3
        builder = DiagramBuilder()
        plant = builder.AddSystem(PassThrough(vector_size=size))
        builder.ExportInput(plant.get_input_port(), "actions")
        builder.ExportOutput(plant.get_output_port(), "observations")
        diagram = builder.Build()
        simulator = Simulator(diagram)

        dut = DrakeGymEnv(
            simulator=simulator,
            time_step=0.1,
            action_space=None,
            observation_space=None,
            reward=lambda system, context: 0.0,
            action_port_id="actions",
            observation_port_id="observations",
        )

        self.assertIsInstance(dut.action_space, gym.spaces.Box)
        self.assertEqual(dut.action_space.shape, (size,))
        self.assertTrue(np.all(np.isneginf(dut.action_space.low)))
        self.assertTrue(np.all(np.isposinf(dut.action_space.high)))

        self.assertIsInstance(dut.observation_space, gym.spaces.Box)
        self.assertEqual(dut.observation_space.shape, (size,))
        self.assertTrue(np.all(np.isneginf(dut.observation_space.low)))
        self.assertTrue(np.all(np.isposinf(dut.observation_space.high)))

    def test_none_spaces_reject_non_vector_ports(self):
        """None is only valid for vector-valued ports."""
        builder = DiagramBuilder()
        system = builder.AddSystem(
            PassThrough(abstract_model_value=Value("model"))
        )
        builder.ExportInput(system.get_input_port(), "actions")
        builder.ExportOutput(system.get_output_port(), "observations")
        diagram = builder.Build()
        simulator = Simulator(diagram)
        # Any concrete Space is fine here; we only exercise the None path.
        dummy_space = gym.spaces.Discrete(1)

        with self.assertRaisesRegex(AssertionError, "action_space must be"):
            DrakeGymEnv(
                simulator=simulator,
                time_step=0.1,
                action_space=None,
                observation_space=dummy_space,
                reward=lambda system, context: 0.0,
                action_port_id="actions",
                observation_port_id="observations",
            )

        with self.assertRaisesRegex(
            AssertionError, "observation_space must be"
        ):
            DrakeGymEnv(
                simulator=simulator,
                time_step=0.1,
                action_space=dummy_space,
                observation_space=None,
                reward=lambda system, context: 0.0,
                action_port_id="actions",
                observation_port_id="observations",
            )

    def _make_pass_through_env(self, **kwargs):
        builder = DiagramBuilder()
        pass_through = builder.AddSystem(PassThrough(vector_size=1))
        builder.ExportInput(pass_through.get_input_port(), "actions")
        builder.ExportOutput(pass_through.get_output_port(), "observations")
        simulator = Simulator(builder.Build())
        return DrakeGymEnv(
            simulator=simulator,
            time_step=0.1,
            action_space=None,
            observation_space=None,
            reward=lambda system, context: 0.0,
            **kwargs,
        )

    def test_info_handler(self):
        """A user-provided info_handler is used by reset() and step()."""

        def info_handler(simulator):
            return {"time": simulator.get_context().get_time()}

        dut = self._make_pass_through_env(info_handler=info_handler)
        _, info = dut.reset(seed=0)
        self.assertEqual(info, {"time": 0.0})
        _, _, _, _, info = dut.step(np.array([0.5]))
        self.assertAlmostEqual(info["time"], 0.1)

    def test_info_handler_default(self):
        """Without an info_handler, the info is an empty dict."""
        dut = self._make_pass_through_env()
        _, info = dut.reset(seed=0)
        self.assertEqual(info, {})
        _, _, _, _, info = dut.step(np.array([0.5]))
        self.assertEqual(info, {})

    def test_info_handler_not_callable(self):
        with self.assertRaisesRegex(ValueError, "info_handler is not callable"):
            self._make_pass_through_env(info_handler="not callable")

    def test_metadata_is_per_instance(self):
        """Constructing an env must not edit gym.Env.metadata (a class
        attribute shared by every Env), nor the metadata of any other env.
        """
        shared_before = copy.deepcopy(gym.Env.metadata)

        builder = DiagramBuilder()
        pass_through = builder.AddSystem(PassThrough(vector_size=1))
        builder.ExportInput(pass_through.get_input_port(), "actions")
        builder.ExportOutput(pass_through.get_output_port(), "observations")
        image = builder.AddSystem(
            PassThrough(abstract_model_value=Value(ImageRgba8U(4, 3)))
        )
        builder.ExportInput(image.get_input_port(), "image_in")
        builder.ExportOutput(image.get_output_port(), "color_image")
        with_rgb = DrakeGymEnv(
            simulator=Simulator(builder.Build()),
            time_step=0.1,
            action_space=None,
            observation_space=None,
            reward=lambda system, context: 0.0,
            render_rgb_port_id="color_image",
        )
        self.assertEqual(
            with_rgb.metadata["render_modes"], ["human", "ansi", "rgb_array"]
        )

        # Building another env leaves the first env's metadata alone.
        without_rgb = self._make_pass_through_env()
        self.assertEqual(
            without_rgb.metadata["render_modes"], ["human", "ansi"]
        )
        self.assertIn("rgb_array", with_rgb.metadata["render_modes"])

        # The metadata shared by all gymnasium envs is unchanged.
        self.assertEqual(gym.Env.metadata, shared_before)
