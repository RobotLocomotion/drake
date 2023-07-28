'''
Play a policy for //bindings/pydrake/examples/gym/envs:cart_pole.
'''
import argparse
import os
import sys

import gymnasium as gym
import stable_baselines3
from stable_baselines3.common.env_checker import check_env

from pydrake.geometry import StartMeshcat


def _run_playing(args):

    if args.log_path is None:
        log = args.log_path
    else:
        log = "./rl/tmp/CartPole/play_runs/"

    # Make a version of the env with meshcat.
    meshcat = StartMeshcat()
    env = gym.make(
            "DrakeCartPole-v0",
            meshcat=meshcat,
            time_limit=7,
            debug=args.debug,
            obs_noise=True,
            add_disturbances=True)

    if args.test:
        check_env(env)

    env.simulator.set_target_realtime_rate(1.0)
    max_steps = 1e5 if not args.test else 1e2

    if not args.test:
        assert "drake_internal" not in stable_baselines3.__version__
        from stable_baselines3 import PPO
        model = PPO.load(args.model_path, env, verbose=1, tensorboard_log=log)

    obs, _ = env.reset()
    for _ in range(int(max_steps)):
        if args.test:
            # Plays a random policy.
            action = env.action_space.sample()
        else:
            action, _states = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, info = env.step(action)
        if args.debug:
            # This will play the policy step by step.
            input("Press Enter to continue...")
        env.render()
        if terminated or truncated:
            if args.debug:
                input("The environment will reset. Press Enter to continue...")
            obs, _ = env.reset()


def _bazel_chdir():
    """When using `bazel run`, the current working directory ("cwd") of the
    program is set to a deeply-nested runfiles directory, not the actual cwd.
    In case relative paths are given on the command line, we need to restore
    the original cwd so that those paths resolve correctly.
    """
    original_working_directory = os.environ.get("BUILD_WORKING_DIRECTORY")
    if original_working_directory is not None:
        os.chdir(original_working_directory)


def _main():
    _bazel_chdir()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--test', action='store_true')
    parser.add_argument('--debug', action='store_true')
    parser.add_argument('--model_path', help="path to the policy zip file.")
    parser.add_argument('--log_path', help="path to the logs directory.")
    args = parser.parse_args()

    gym.envs.register(id="DrakeCartPole-v0",
                    entry_point="pydrake.examples.gym.envs.cart_pole:DrakeCartPoleEnv")  # noqa

    _run_playing(args)


if __name__ == '__main__':
    _main()
