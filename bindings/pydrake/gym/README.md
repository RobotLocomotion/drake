Drake Gym
=========

Running the examples
--------------------

An example of training to stabilize a cartpole is provided.
It includes added noise in the observation, added random disturbances,
randomized states (position, velocity), randomized mass,
a monitoring camera for rollout logging, and integration with Wandb.
It can be invoke it via:

    bazel run //bindings/pydrake/examples/gym:train_cart_pole

Depending on your machine, you should see the reward (`ep_rew_mean`) increasing from 0 to ~140 in about 20 min.

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
   [the repository backing the course notebooks](https://github.com/RussTedrake/manipulation/blob/f569cd653f35202416e865c42d6825eff9ef2691/manipulation/drake_gym.py)
   * Where possible this code has been borrowed verbatim; small changes (e.g.
     to bazel rules, namespaces, and so forth) have been required.
