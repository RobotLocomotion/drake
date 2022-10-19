import unittest


class StableBaselines3InternalTest(unittest.TestCase):
    """Test that the methods that we use in drake are able to be loaded
    in spite of Drake not having all of SB3's dependencies."""
    def import_test():
        from stable_baselines3.common.env_checker import check_env
        help(check_env)
