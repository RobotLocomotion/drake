Drake Gym
=========

Running the examples
--------------------

An example of training to stabilize a cart-pole system is provided.
It includes added noise in the observation, added random disturbances,
randomized states (position, velocity), randomized mass,
and a monitoring camera for rollout logging.

Before trying the following examples, <strong>read the note on dependencies below</strong>.

Training can be invoked it via:

    bazel run //bindings/pydrake/examples/gym:train_cart_pole

Depending on your machine, you should see the reward (`ep_rew_mean`) increasing from 0 to ~120 in about 7 min.

The learned model can be played with:

    bazel run //bindings/pydrake/examples/gym:play_cart_pole -- \
                --model_path {path_to_zip_model_file}

A random policy can be played with:

    bazel run //bindings/pydrake/examples/gym:play_cart_pole -- --test

To visualize, open meshcat by following the link in the prompts. If this link fails, it is likely that your loopback device is broken. Try using http://127.0.0.1:7000 instead.

A note on dependencies
----------------------

In order to use a Gym, code must implement a `gymnasium.Env` (in this case the actual `Simulator` wrapped via `DrakeGymEnv`) and must run the Gym within an RL engine of some sort (that's usually an algorithm chosen from
[Stable Baselines 3](https://stable-baselines3.readthedocs.io/en/master/index.html) but could also be from `nevergrad` or some other source of gradient-free
optimizers).

Stable Baselines3 itself is too large and too heavy of a dependency tree for Drake to require itself; as such you will need to provide it yourself to use these examples.  If you are going to train drake gym examples on your machine, you should install a few dependencies including `stable_baselines3` and `gymnasium` inside of a virtual environment. The training examples will not run without it, and drake does not come with it (Drake does include a subset of `stable_baselines3` for testing purposes, but not enough to perform training). Run the training example via:

      # Create an environment (or use an existing).
      python3 -m venv ~/tmp/drakegym --system-site-packages
      # Activate the environment.
      source ~/tmp/drakegym/bin/activate
      # Install dependencies.
      pip install gymnasium stable_baselines3 tensorboard moviepy
      # From the Drake source tree, run the training.
      cd {your_your_drake_directory}/drake
      export PYTHONPATH=~/tmp/drakegym/lib/python3.10/site-packages
      bazel run //bindings/pydrake/examples/gym:train_cart_pole

Notes
-----

 * The overall concept of Drake Gym is discussed in
   [the course notes for Russ's Robot Manipulation course](https://manipulation.csail.mit.edu/rl.html#section1)

 * The code here is borrowed from
   [the repository backing the course notebooks](https://github.com/RussTedrake/manipulation/blob/f569cd653f35202416e865c42d6825eff9ef2691/manipulation/drake_gym.py)
   * Where possible this code has been borrowed verbatim; small changes (e.g.
     to bazel rules, namespaces, and so forth) have been required.
