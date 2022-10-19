import argparse
import os
import sys

import gym
# `multiprocessing` also provides this method, but empirically `psutil`'s
# version seems more reliable.
from psutil import cpu_count

sb3_available = False
try:
    import stable_baselines3
    if "drake_internal" not in stable_baselines3.__version__:
        from stable_baselines3 import A2C, PPO
        from stable_baselines3.common.env_util import make_vec_env
        from stable_baselines3.common.vec_env import SubprocVecEnv
        sb3_available = True
    else:
        print("stable_baselines3 found, but was drake internal")
except ImportError:
    print("stable_baselines3 not found")


def main():
    parser = argparse.ArgumentParser(
        description='Train a policy for //drake_gym/examples.envs:box_flipup.')
    parser.add_argument('--test', action='store_true')
    args = parser.parse_args()

    if not sb3_available:
        print("stable_baselines3 not available.")
        return 0 if args.test else 1

    observations = "state"
    time_limit = 10 if not args.test else 0.5
    zip = "data/box_flipup_ppo_{observations}.zip"
    log = "/tmp/ppo_box_flipup/"

    gym.envs.register(
        id="BoxFlipUp-v0",
        entry_point="drake_gym.examples.envs.box_flipup.box_flipup:BoxFlipUpEnv")  # noqa

    num_cpu = int(cpu_count() / 2) if not args.test else 2
    env = make_vec_env("BoxFlipUp-v0",
                       n_envs=num_cpu,
                       seed=0,
                       vec_env_cls=SubprocVecEnv,
                       env_kwargs={
                           'observations': observations,
                           'time_limit': time_limit,
                       })

    if args.test:
        model = PPO('MlpPolicy', env, n_steps=4, n_epochs=2, batch_size=8)
    elif os.path.exists(zip):
        model = PPO.load(zip, env, verbose=1, tensorboard_log=log)
    else:
        model = PPO('MlpPolicy', env, verbose=1, tensorboard_log=log)

    new_log = True
    while True:
        model.learn(total_timesteps=100000 if not args.test else 4,
                    reset_num_timesteps=new_log)
        if args.test:
            break
        model.save(zip)
        new_log = False


if __name__ == '__main__':
    sys.exit(main())
