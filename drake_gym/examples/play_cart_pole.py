import argparse
import time

import gym
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env

from pydrake.all import StartMeshcat

parser = argparse.ArgumentParser(
    description='Play a policy for //drake_gym/examples.envs:cart_pole.')
parser.add_argument('--test', action='store_true')
parser.add_argument('--debug', action='store_true')
parser.add_argument('--model_path', help="path to the policy zip file.")
parser.add_argument('--log_path', help="path to the logs directory.")
args = parser.parse_args()

gym.envs.register(id="Cartpole-v0",
                  entry_point="drake_gym.examples.envs.cart_pole.cart_pole:CartpoleEnv")  # noqa

if args.model_path is not None:
    zip = args.model_path
else:
    zip = "./rl/tmp/Cartpole/models/{model_id}/model.zip"

if args.log_path is None:
    log = args.log_path
else:
    log = "./rl/tmp/Cartpole/play_runs/"

if __name__ == '__main__':
    # Make a version of the env with meshcat.
    meshcat = StartMeshcat()
    env = gym.make("Cartpole-v0",
                   meshcat=meshcat,
                   time_limit=7,
                   debug=args.debug,
                   obs_noise=True,
                   add_disturbances=True)

    if args.test:
        check_env(env)

    env.simulator.set_target_realtime_rate(1.0)
    max_num_episodes = 1e5 if args.test else 1e3

    if not args.test:
        model = PPO.load(zip, env, verbose=1, tensorboard_log=log)

    input("Press Enter to continue...")
    obs = env.reset()
    for i in range(100000):
        if args.test:
            # Plays a random policy.
            action = env.action_space.sample()
        else:
            action, _state = model.predict(obs, deterministic=True)
        obs, reward, done, info = env.step(action)
        if args.debug:
            # This will play the policy step by step.
            input("Press Enter to continue...")
        env.render()
        if done:
            input("The environment will reset. Press Enter to continue...")
            obs = env.reset()
            # Wait for meshcat to load the env.
            # TODO(JoseBarreiros-TRI) Replace sleep() with a readiness signal
            # from meshcat.
            time.sleep(0.7)
