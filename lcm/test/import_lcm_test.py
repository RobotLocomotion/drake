import unittest


class ImportLcmTest(unittest.TestCase):
    """This is simple smoke test that we can load and run the LCM class without
    errors from within the Bazel sandbox.
    """

    def test_import(self):
        import lcm
        dut = lcm.LCM()
        self.assertGreater(dut.fileno(), 0)


if __name__ == '__main__':
    unittest.main()
