Drake Gym
=========

THIS FEATURE IS EXPERIMENTAL.  Development is ongoing and no guarantees
against deprecation are provided for any file under this directory.

Drake Gym is an implementation of OpenAI's "Gym" interface for reinforcement
learning which uses a Drake `Simulator` as a backend.  The Gym interface
(provided by [the python `gym` module](https://pypi.org/project/gym/) simply
models a time-stepped process with an action space, a reward function, and
some form of state observation.

A note on dependencies
----------------------

In order to use a Gym, code must implement a `gym.Env` (in this case the
actual `Simulator` wrapped via `DrakeGymEnv`) and must run the Gym within an
RL engine of some sort (that's usually an algorithm chosen from
[Stable Baselines 3](https://stable-baselines3.readthedocs.io/en/master/index.html)
but could also be from `nevergrad` or some other source of gradient-free
optimizers).

Stable Baselines3 iteself is too large and too heavy of a dependency tree for
Drake to require itself; as such you will need to provide it yourself to use
these examples.  If you are going to train drake gym examples on your machine,
you should install Stable Baselines 3 (for instance,
`pip install stable_baselines3` inside of a virtual environment).  The
training examples will not run without it, and drake does not come with it
(Drake does include a subset of `stable_baselines3` for testing purposes, but
not enough to perform training).

Running the examples
--------------------

A trivial example of training a 2D box flipper is provided; invoke
it via:

    bazel run //bindings/pydrake/examples/gym:train_box_flipup

Depending on your machine, you should see the reward (`ep_rew_mean`) number
increase slowly from ~500 to ~700 in an hour or so, which means that training
is in fact making a more successful policy.

Another example of training to stabilize a cartpole is provided.
It includes added noise in the observation, added random disturbances,
randomized states (position, velocity), randomized mass,
a monitoring camera for rollout logging, and integration with Wandb.
It can be invoke it via:

    bazel run //bindings/pydrake/examples/gym:train_cart_pole

Depending on your machine, you should see the reward increasing
from 0 to ~140 in about 20 min.

The learned model can be played with:

    bazel run //bindings/pydrake/examples/gym:play_cart_pole -- \
                --model_path {path_to_zip_model_file}

A random policy can be played with:
    bazel run //bindings/pydrake/examples/gym:play_cart_pole -- --test

Notes
-----

 * The overall concept of Drake Gym is discussed in
   [the course notes for Russ's Robot Manipulation course](https://manipulation.csail.mit.edu/rl.html#section1)

 * The code here is borrowed from
   [the repository backing the course notebooks](https://github.com/RussTedrake/manipulation/blob/master/manipulation/drake_gym.py)
   * Where possible this code has been borrowed verbatim; small changes (e.g.
     to bazel rules, namespaces, and so forth) have been required.
