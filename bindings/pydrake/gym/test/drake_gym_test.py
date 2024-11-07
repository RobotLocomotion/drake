import unittest

import gymnasium as gym
import numpy as np
import stable_baselines3.common.env_checker


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
            entry_point="pydrake.examples.gym.envs.cart_pole:DrakeCartPoleEnv")  # noqa

    def make_env(self):
        return gym.make("DrakeCartPole-v0")

    def test_make_env(self):
        self.make_env()

    def test_sb3_check_env(self):
        """Run stable-baselines's built-in test suite for our env."""
        dut = self.make_env()
        stable_baselines3.common.env_checker.check_env(
            env=dut,
            warn=True,
            skip_render_check=True)

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
