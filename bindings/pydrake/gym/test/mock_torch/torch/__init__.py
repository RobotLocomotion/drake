# This is a mock version of torch needed for
# stable_baselines3_internal which is used
# for testing.
# This does not affect the functionality of env_checker.check_env()

# ruff: noqa: F401 (unused-import)

from . import distributions, nn, optim

Tensor = None
device = None
