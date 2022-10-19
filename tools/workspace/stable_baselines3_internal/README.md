stable_baselines3_internal
==========================

A local import of `stable_baselines3` that cuts away functions that depend
on python modules that we cannot guarantee are available as drake
dependencies (e.g. `pytorch`)

The resulting `stable_baselines3` workalike is available via::

  deps = [
      "@stable_baselines3_internal//:stable_baselines3",
  ],

and then::

  import stable_baselines3
