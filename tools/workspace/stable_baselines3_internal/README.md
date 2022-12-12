stable_baselines3_internal
==========================

A local import of `stable_baselines3` that *does not* have its dependencies
(such as `pytorch`).  As such, code using this module will have a
`ModuleNotFoundError` error if it attempts to use any functions that depend on 
those modules.

The resulting `stable_baselines3` workalike is available via the bazel::

  deps = [
      "@stable_baselines3_internal//:stable_baselines3",
  ],

and then, in python::

  import stable_baselines3

Code that wants to import the most featureful version of `stable_baselines3`
available (i.e., to use a locally installed `stable_baselines3` if available
and the Drake-provided workalike otherwise) can rely on the presence of the
string `"drake_internal"` in the `__version__` attribute.  For instance::

  sb3_is_fully_featured = False
  try:
      import stable_baselines3
      if "drake_internal" not in stable_baselines3.__version__:
          sb3_is_fully_featured = True
      else:
          print("stable_baselines3 found, but was drake internal")
  except ImportError:
      print("stable_baselines3 not found")
