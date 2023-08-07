"""
Train a policy for //bindings/pydrake/examples/envs:cart_pole
"""
import argparse

import gymnasium as gym
import stable_baselines3
from stable_baselines3.common.env_checker import check_env

from pydrake.geometry import StartMeshcat
from pydrake.examples.gym._bazel_cwd_helpers import bazel_chdir

_full_sb3_available = False
if "drake_internal" not in stable_baselines3.__version__:
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import EvalCallback
    from stable_baselines3.common.env_util import make_vec_env
    from stable_baselines3.common.vec_env import (
        DummyVecEnv,
        SubprocVecEnv,
        VecVideoRecorder,
    )
    import torch as th
    _full_sb3_available = True


def _run_training(config, args):
    env_name = config["env_name"]
    num_env = config["num_workers"]
    time_limit = config["env_time_limit"]
    log_dir = config["local_log_dir"]
    policy_type = config["policy_type"]
    total_timesteps = config["total_timesteps"]
    policy_kwargs = config["policy_kwargs"]
    eval_freq = config["model_save_freq"]
    obs_noise = config["observation_noise"]
    add_disturbances = config["disturbances"]

    if not args.train_single_env:
        env = make_vec_env(env_name,
                           n_envs=num_env,
                           seed=0,
                           vec_env_cls=SubprocVecEnv,
                           env_kwargs={
                               'time_limit': time_limit,
                               'obs_noise': obs_noise,
                               'add_disturbances': add_disturbances,
                           })
    else:
        meshcat = StartMeshcat()
        env = gym.make(env_name,
                       meshcat=meshcat,
                       time_limit=time_limit,
                       debug=args.debug,
                       obs_noise=obs_noise,
                       )
        check_env(env)
        input("Open meshcat (optional). Press Enter to continue...")

    if args.test:
        model = PPO(policy_type, env, n_steps=4, n_epochs=2,
                    batch_size=8, policy_kwargs=policy_kwargs)
    else:
        tensorboard_log = f"{log_dir}runs/test"
        model = PPO(
            policy_type, env, n_steps=int(2048/num_env), n_epochs=10,
            # In SB3, this is the mini-batch size.
            # https://github.com/DLR-RM/stable-baselines3/blob/master/docs/modules/ppo.rst
            batch_size=64*num_env,
            verbose=1,
            tensorboard_log=tensorboard_log,
            policy_kwargs=policy_kwargs)
        print("Open tensorboard (optional) via "
              f"`tensorboard --logdir {tensorboard_log}` "
              "in another terminal.")

    # Separate evaluation env.
    eval_env = gym.make(env_name,
                        time_limit=time_limit,
                        obs_noise=obs_noise,
                        monitoring_camera=True,
                        )
    eval_env = DummyVecEnv([lambda: eval_env])
    # Record a video every n evaluation rollouts.
    n = 1
    eval_env = VecVideoRecorder(
                        eval_env,
                        log_dir+f"videos/test",
                        record_video_trigger=lambda x: x % n == 0,
                        video_length=100)
    # Use deterministic actions for evaluation.
    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path=log_dir+f'eval_logs/test',
        log_path=log_dir+f'eval_logs/test',
        eval_freq=eval_freq,
        deterministic=True,
        render=False)

    model.learn(
        total_timesteps=total_timesteps,
        callback=eval_callback,
    )

    eval_env.close()


def _main():
    bazel_chdir()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--test', action='store_true')
    parser.add_argument('--train_single_env', action='store_true')
    parser.add_argument('--debug', action='store_true')
    parser.add_argument('--log_path', help="path to the logs directory.",
                        default="./rl/tmp/DrakeCartPole/")
    args = parser.parse_args()

    if not _full_sb3_available:
        print("stable_baselines3 found, but was drake internal")
        return 0 if args.test else 1

    if args.test:
        num_env = 2
    elif args.train_single_env:
        num_env = 1
    else:
        num_env = 10

    config = {
        "policy_type": "MlpPolicy",
        "total_timesteps": 5e5 if not args.test else 5,
        "env_name": "DrakeCartPole-v0",
        "num_workers": num_env,
        "env_time_limit": 7 if not args.test else 0.5,
        "local_log_dir": args.log_path,
        "model_save_freq": 1e4,
        "policy_kwargs": {'activation_fn': th.nn.ReLU,
                          'net_arch': {'pi': [64, 64, 64],
                                       'vf': [64, 64, 64]}},
        "observation_noise": True,
        "disturbances": True,
    }

    _run_training(config, args)


gym.envs.register(
    id="DrakeCartPole-v0",
    entry_point=(
        "pydrake.examples.gym.envs.cart_pole:DrakeCartPoleEnv"))

if __name__ == '__main__':
    _main()
