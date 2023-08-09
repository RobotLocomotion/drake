"""
Drake Gym
=========

THIS FEATURE IS EXPERIMENTAL.
As per our [guidelines](https://drake.mit.edu/stable.html)
for experimental code, development is ongoing and no guarantees against
deprecation are provided for any file under this directory.

Drake Gym is an implementation of Farama's "Gymnasium" interface for
reinforcement learning which uses a Drake `Simulator` as a
backend.  The Gym interface provided
by [the python `gym` module](https://pypi.org/project/gymnasium/)
simply models a time-stepped process with an action space,
a reward function, and some form of state observation.

Note that pydrake.gym is an optional component of pydrake, and will only work
when the gymnasium package is also installed. As such, the DrakeGymEnv and
related code is not available for import as part of `pydrake.all`.

A note on dependencies
----------------------

In order to use a Gym, code must implement a `gymnasium.Env` (in this case the
actual `Simulator` wrapped via `DrakeGymEnv`) and must run the Gym within an
RL engine of some sort (that's usually an algorithm chosen from
[Stable Baselines 3]
(https://stable-baselines3.readthedocs.io/en/master/index.html)
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
"""

from ._drake_gym_env import DrakeGymEnv

__all__ = [x for x in globals() if not x.startswith("_")]
