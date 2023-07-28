'''
Play a policy for //bindings/pydrake/examples/gym:cart_pole.
'''
import argparse
import sys

import gymnasium as gym
from stable_baselines3.common.env_checker import check_env

from pydrake.geometry import StartMeshcat

full_sb3_available = False
try:
    import stable_baselines3
    if "drake_internal" not in stable_baselines3.__version__:
        from stable_baselines3 import PPO
        full_sb3_available = True
    else:
        print("stable_baselines3 found, but was drake internal")
except ImportError:
    print("stable_baselines3 not found")


def run_playing(args):

    if args.log_path is None:
        log = args.log_path
    else:
        log = "./rl/tmp/Cartpole/play_runs/"

    # Make a version of the env with meshcat.
    meshcat = StartMeshcat()
    env = gym.make(
            "Cartpole-v0",
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--test', action='store_true')
    parser.add_argument('--debug', action='store_true')
    parser.add_argument('--model_path', help="path to the policy zip file.")
    parser.add_argument('--log_path', help="path to the logs directory.")
    args = parser.parse_args()

    gym.envs.register(id="Cartpole-v0",
                    entry_point="pydrake.examples.gym.envs.cart_pole.cart_pole:CartpoleEnv")  # noqa

    run_playing(args)


assert __name__ == '__main__'
main()
