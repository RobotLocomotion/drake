import unittest

import gym
import numpy as np
import stable_baselines3.common.env_checker

from drake_gym.examples.envs.box_flipup.box_flipup import BoxFlipUpEnv

vec_env_available = False
try:
    from stable_baselines3.common.env_util import make_vec_env
    from stable_baselines3.common.vec_env import SubprocVecEnv
    vec_env_availabe = True
except ImportError:
    print("vec_env is not available, so not testing it.")


class DrakeGymTest(unittest.TestCase):
    """
    Test that an DrakeGymEnv satisfies the OpenAI Gym Env specifications as to
    * API https://www.gymlibrary.ml/content/api/#standard-methods, and
    * semantics https://www.gymlibrary.ml/content/environment_creation/

    Not every Gym optimizer algorithm uses every part of the API (for
    instance none use `reset(seed)` as far as I can tell) but we check them
    anyway because if they ever are used the errors will be hard to find.
    """

    @classmethod
    def setUpClass(cls):
        gym.envs.register(
            id="BoxFlipUp-v0",
            entry_point="drake_gym.examples.envs.box_flipup.box_flipup:BoxFlipUpEnv")  # noqa

    def make_env(self):
        # `new_step_api=False` was supposed to be deprecated some time ago
        # but is still required by stable_baselines3.
        return gym.make("BoxFlipUp-v0")

    def test_make_env(self):
        self.make_env()

    # This test is comprehensive enough to make many other tests below
    # redundant, but we can't port it to Drake when the time comes for that
    # without eating `stable_baselines3`'s enormous dependency tree.
    def test_openai_check_env(self):
        """Run OpenAI's built-in test suite for our env."""
        dut = self.make_env()
        stable_baselines3.common.env_checker.check_env(
            env=dut,
            warn=True,
            skip_render_check=True)

    def test_openai_check_vector_env(self):
        if not vec_env_available:
            return
        # Check that we can construct a vector env.
        vector_dut = make_vec_env(
            BoxFlipUpEnv,
            n_envs=2,
            seed=0,
            vec_env_cls=SubprocVecEnv,
            env_kwargs={
                'observations': 'state',
                'time_limit': 5,
            })
        # We should `check_env` here, but in our currently supported versions
        # of `gym` and `stable_baselines3`, stable baselines vector envs do
        # not pass stable baselines' `check_env` tests

    def test_reset(self):
        # reset(int) sets a deterministic seed.
        dut = self.make_env()
        obs1 = dut.reset(seed=7)
        obs2 = dut.reset(seed=7)
        self.assertTrue((obs1 == obs2).all())

        # reset() on its own gets a new arbitrary seed.
        dut = self.make_env()
        obs1 = dut.reset()
        obs2 = dut.reset()
        self.assertFalse((obs1 == obs2).all())

        # The difference when reset() follows reset(seed) is not
        # externally observable, so don't test it.

        # return_options changes the return type.
        (observation, opts) = dut.reset(return_info=True)
        self.assertIsInstance(opts, dict)
        self.assertTrue(dut.observation_space.contains(observation))

    def test_step(self):
        dut = self.make_env()
        dut.reset()
        # TODO(ggould): This uses the old, pseudo-deprecated gym API.
        observation, _, _, _ = dut.step(dut.action_space.sample())
        self.assertTrue(dut.observation_space.contains(observation))


if __name__ == "__main__":
    unittest.main()
