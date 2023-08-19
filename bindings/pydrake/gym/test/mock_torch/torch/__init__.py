# This is a mock version of torch needed for
# stable_baselines3_internal which is used
# for testing.
# This does not affect the functionality of env_checker.check_env()

from . import distributions
from . import nn
from . import optim

Tensor = None
device = None
