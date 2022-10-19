Drake Gym
=========

THIS FEATURE IS EXPERIMENTAL.  Development is ongoing and no guarantees
against deprecation are provided for any file under this directory.

Drake Gym is an implementation of OpenAI's "Gym" interface for reinforcement
learning which uses a Drake `Simulator` as a backend.  The Gym interface
(provided by [the python `gym` module](https://pypi.org/project/gym/) simply
models a time-stepped process with an action space, a reward function, and
some form of state observation.

In order to use a Gym, code must implement a `gym.Env` (in this case the
actual `Simulator` wrapped via `DrakeGymEnv`) and must run the Gym within an
RL engine of some sort (that's usually an algorithm chosen from
[Stable Baselines 3](https://stable-baselines3.readthedocs.io/en/master/index.html)
but could also be from `nevergrad` or some other source of gradient-free
optimizers).

A trivial example of training a 2D box flipper is provided; invoke
it via:

    bazel run //drake_gym/examples:train_box_flipup

Depending on your machine, you should see the reward (`ep_rew_mean`) number
increase slowly from ~500 to ~700 in an hour or so, which means that training
is in fact making a more successful policy.

Another example of training to stabilize a cartpole is provided.
It includes added noise in the observation, added random disturbances,
randomized states (position, velocity), randomized mass,
a monitoring camera for rollout logging, and integration with Wandb.
It can be invoke it via:

    bazel run //drake_gym/examples:train_cart_pole

Depending on your machine, you should see the reward increasing
from 0 to ~140 in about 20 min.

The learned model can be played with:

    bazel run //drake_gym/examples:play_cart_pole -- --model_path {path_to_zip_model_file}

A random policy can be played with:
    bazel run //drake_gym/examples:play_cart_pole -- --test

Notes
-----

 * The overall concept of Drake Gym is discussed in
   [the course notes for Russ's Robot Manipulation course](https://manipulation.csail.mit.edu/rl.html#section1)

 * The code here is borrowed from
   [the repository backing the course notebooks](https://github.com/RussTedrake/manipulation/blob/master/manipulation/drake_gym.py)
   * Where possible this code has been borrowed verbatim; small changes (e.g.
     to bazel rules, namespaces, and so forth) have been required.

 * Although neither Drake Gym itself nor most Envs will have excessive
   dependencies, running a Gym requires an RL engine that may quite
   heavyweight.  Users will need to install `stable_baselines3` themselves,
   ideally in a python virtual environment.
