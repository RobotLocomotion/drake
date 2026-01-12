import json
from pathlib import Path
import subprocess
import unittest

from python import runfiles


class TestCpuCapabilities(unittest.TestCase):
    def setUp(self):
        manifest = runfiles.Create()
        self.device = Path(
            manifest.Rlocation("drake/common/cpu_capabilities_test_device")
        )

    def _check(self, env=None, expect_avx2=True, expect_avx3=True):
        """Runs the test device under the given environment vars, and checks
        that it produced the expected result.
        """
        env = env or {}
        with self.subTest(
            env=env, expect_avx2=expect_avx2, expect_avx3=expect_avx3
        ):
            stdout = subprocess.check_output([self.device], env=env)
            actual = json.loads(stdout)
            self.assertEqual(
                actual, dict(allow_avx2=expect_avx2, allow_avx3=expect_avx3)
            )

    def test_default(self):
        """No environment vars are set."""
        self._check()

    def test_drake_env(self):
        """Only the Drake environment var is set."""
        self._check(
            env=dict(DRAKE_DISABLE_CPU_FEATURES="FMA3"),
            expect_avx2=False,
            expect_avx3=False,
        )
        self._check(
            env=dict(DRAKE_DISABLE_CPU_FEATURES="AVX2"),
            expect_avx2=False,
            expect_avx3=False,
        )
        self._check(
            env=dict(DRAKE_DISABLE_CPU_FEATURES="AVX512F"),
            expect_avx2=True,
            expect_avx3=False,
        )
        self._check(
            env=dict(DRAKE_DISABLE_CPU_FEATURES="MagicPony"),
            expect_avx2=True,
            expect_avx3=True,
        )
        self._check(
            env=dict(DRAKE_DISABLE_CPU_FEATURES="Magic,AVX512F"),
            expect_avx2=True,
            expect_avx3=False,
        )
        self._check(
            env=dict(DRAKE_DISABLE_CPU_FEATURES="AVX512F,Pony"),
            expect_avx2=True,
            expect_avx3=False,
        )

    def test_numpy_env(self):
        """Only the NumPy environment var is set."""
        self._check(
            env=dict(NPY_DISABLE_CPU_FEATURES="FMA3"),
            expect_avx2=False,
            expect_avx3=False,
        )
        self._check(
            env=dict(NPY_DISABLE_CPU_FEATURES="AVX2"),
            expect_avx2=False,
            expect_avx3=False,
        )
        self._check(
            env=dict(NPY_DISABLE_CPU_FEATURES="AVX512F"),
            expect_avx2=True,
            expect_avx3=False,
        )
        self._check(
            env=dict(NPY_DISABLE_CPU_FEATURES="MagicPony"),
            expect_avx2=True,
            expect_avx3=True,
        )
        self._check(
            env=dict(NPY_DISABLE_CPU_FEATURES="Magic,AVX512F"),
            expect_avx2=True,
            expect_avx3=False,
        )
        self._check(
            env=dict(NPY_DISABLE_CPU_FEATURES="AVX512F,Pony"),
            expect_avx2=True,
            expect_avx3=False,
        )

        # NumPy allows tab and space beyond just comma.
        self._check(
            env=dict(NPY_DISABLE_CPU_FEATURES="Magic AVX512F"),
            expect_avx2=True,
            expect_avx3=False,
        )
        self._check(
            env=dict(NPY_DISABLE_CPU_FEATURES="Magic  AVX512F"),
            expect_avx2=True,
            expect_avx3=False,
        )
        self._check(
            env=dict(NPY_DISABLE_CPU_FEATURES="Magic\tAVX512F"),
            expect_avx2=True,
            expect_avx3=False,
        )
        self._check(
            env=dict(NPY_DISABLE_CPU_FEATURES="Magic \t AVX512F"),
            expect_avx2=True,
            expect_avx3=False,
        )

    def test_mixed_env(self):
        """Both the Drake and NumPy environment vars are set.
        Drake always wins.
        """
        self._check(
            env=dict(
                DRAKE_DISABLE_CPU_FEATURES="", NPY_DISABLE_CPU_FEATURES="AVX2"
            ),
            expect_avx2=True,
            expect_avx3=True,
        )
        self._check(
            env=dict(
                DRAKE_DISABLE_CPU_FEATURES="AVX512F",
                NPY_DISABLE_CPU_FEATURES="AVX2",
            ),
            expect_avx2=True,
            expect_avx3=False,
        )
        self._check(
            env=dict(
                DRAKE_DISABLE_CPU_FEATURES="AVX512F",
                NPY_DISABLE_CPU_FEATURES="",
            ),
            expect_avx2=True,
            expect_avx3=False,
        )
