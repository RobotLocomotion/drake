import pydrake.common as mut  # ruff: isort: skip
import pydrake.common._testing as mut_testing  # ruff: isort: skip

import copy
import os
import pickle
import unittest

import numpy as np

from pydrake.autodiffutils import AutoDiffXd
from pydrake.common.test_utilities.pickle_compare import assert_pickle
from pydrake.common.yaml import yaml_dump_typed, yaml_load_typed


class TestCommon(unittest.TestCase):
    def test_drake_demand_throws(self):
        # Drake's assertion errors should turn into SystemExit by default,
        # without the user needing to do anything special.  Here, we trigger a
        # C++ assertion failure from Python and confirm that an exception with
        # an appropriate type and message comes out.
        try:
            mut.trigger_an_assertion_failure()
            self.fail("Did not get a SystemExit")
        except SystemExit as e:
            self.assertTrue(e.code is not None)
            self.assertRegex(
                str(e),
                ".*".join(
                    [
                        "Failure at ",
                        " trigger_an_assertion_failure",
                        " condition 'false' failed",
                    ]
                ),
            )

    def test_find_resource_or_throw(self):
        mut.FindResourceOrThrow("drake/examples/acrobot/Acrobot.urdf")

    def test_sha256(self):
        empty = mut.Sha256()
        not_empty = mut.Sha256.Checksum("Some string")
        self.assertFalse(empty == not_empty)
        self.assertTrue(empty != not_empty)
        self.assertTrue(empty < not_empty)

        str_value = not_empty.to_string()
        not_empty2 = mut.Sha256.Parse(str_value)
        self.assertTrue(not_empty == not_empty2)

        assert_pickle(self, not_empty, lambda sha: sha)

        copy.copy(not_empty)
        copy.deepcopy(not_empty)

    def test_memory_file(self):
        content_bytes = b"Some string"
        hint = "hint"
        ext = ".bob"
        file = mut.MemoryFile(content_bytes, ext, hint)

        self.assertEqual(file.sha256(), mut.Sha256.Checksum(content_bytes))
        self.assertEqual(file.contents(), content_bytes)
        self.assertEqual(file.extension(), ext)
        self.assertEqual(file.filename_hint(), hint)

        assert_pickle(self, file, repr)

        # Check that data pickled as MemoryFile in Drake v1.33.0 can be
        # unpickled in newer versions. The data should produce a MemoryFile
        # identical to `file` above.
        legacy_data = b"\x80\x04\x95l\x00\x00\x00\x00\x00\x00\x00\x8c\x0epydrake.common\x94\x8c\nMemoryFile\x94\x93\x94)\x81\x94}\x94(\x8c\x08contents\x94\x8c\x0bSome string\x94\x8c\textension\x94\x8c\x04.bob\x94\x8c\rfilename_hint\x94\x8c\x04hint\x94ub."  # noqa
        obj = pickle.loads(legacy_data)
        self.assertIsInstance(obj, mut.MemoryFile)
        self.assertEqual(obj.contents(), file.contents())
        self.assertEqual(obj.extension(), file.extension())
        self.assertEqual(obj.filename_hint(), file.filename_hint())

        def string_regex(s):
            """Confirm that the string is surrounded by quotes (either double
            or single; we don't care which, just so long as they match)."""
            return f"""(['"]){s}\\1"""

        representation = repr(file)
        # We know that content_bytes is easily decodable as a string.
        self.assertRegex(
            representation, string_regex(content_bytes.decode("utf-8"))
        )
        self.assertRegex(representation, string_regex(hint))
        self.assertRegex(representation, string_regex(ext))

        copy.copy(file)
        copy.deepcopy(file)

        file = mut.MemoryFile.Make(
            mut.FindResourceOrThrow("drake/examples/acrobot/Acrobot.urdf")
        )
        self.assertEqual(file.extension(), ".urdf")

    def test_memory_file_yaml_serialization(self):
        """Confirms that this can be serialized appropriately. The
        serialization work (with all of its nuances) gets tested elsewhere.
        Here, we're simply testing that the fields get serialized and
        deserialized as expected.
        """
        content = "This is an example of memory file test contents."
        content_b64 = (
            "VGhpcyBpcyBhbiBleGFtcGxlIG9mIG1lbW9yeSBmaWxlIHRlc3QgY29udGVudHMu"
        )
        dut = mut.MemoryFile(content, ".txt", "payload.txt")

        # Serialization.
        dumped = yaml_dump_typed(dut)
        self.assertEqual(
            dumped,
            "contents: !!binary |\n"
            + f"  {content_b64}\n"
            + "extension: .txt\n"
            + "filename_hint: payload.txt\n",
        )

        # Deserialization.
        from_yaml = yaml_load_typed(schema=mut.MemoryFile, data=dumped)
        self.assertEqual(from_yaml.contents(), dut.contents())
        self.assertEqual(from_yaml.extension(), dut.extension())
        self.assertEqual(from_yaml.filename_hint(), dut.filename_hint())
        self.assertEqual(from_yaml.sha256(), dut.sha256())

    def test_parallelism(self):
        # This matches the BUILD.bazel rule for this test program.
        self.assertEqual(os.environ.get("DRAKE_NUM_THREADS"), "2")
        max_num_threads = 2

        # Construction.
        mut.Parallelism()
        mut.Parallelism(parallelize=False)
        mut.Parallelism(num_threads=1)
        mut.Parallelism.Max()

        # Copyable.
        copy.copy(mut.Parallelism())

        # The num_threads() getter.
        self.assertEqual(mut.Parallelism().num_threads(), 1)
        self.assertEqual(mut.Parallelism.Max().num_threads(), max_num_threads)

        # Construction without kwarg names must be careful not to automatically
        # convert between bool and int.
        self.assertEqual(mut.Parallelism(False).num_threads(), 1)
        self.assertEqual(mut.Parallelism(True).num_threads(), max_num_threads)
        self.assertEqual(mut.Parallelism(1).num_threads(), 1)
        self.assertEqual(mut.Parallelism(3).num_threads(), 3)

        # Round-trip repr.
        for rep in ["Parallelism(num_threads=1)", "Parallelism(num_threads=2)"]:
            x = eval(rep, {"Parallelism": mut.Parallelism}, {})
            self.assertEqual(repr(x), rep)

        # Floats are right out.
        with self.assertRaisesRegex(Exception, "types"):
            mut.Parallelism(0.5)

    def test_temp_directory(self):
        temp_dir = mut.temp_directory()
        # We'll simply confirm that the path *starts* with the TEST_TMPDIR and
        # that it exists. We'll assume that it otherwise has the documented
        # properties.
        self.assertTrue(temp_dir.startswith(os.environ.get("TEST_TMPDIR")))
        self.assertTrue(os.path.exists(temp_dir))

    def test_tolerance_type(self):
        # Simply test the spelling
        mut.ToleranceType.kAbsolute
        mut.ToleranceType.kRelative

    def test_random_distribution(self):
        # Simply test the spelling
        mut.RandomDistribution.kUniform
        mut.RandomDistribution.kGaussian
        mut.RandomDistribution.kExponential

    def test_logging(self):
        self.assertTrue(mut._HAVE_SPDLOG)
        self.assertIsInstance(mut._set_log_level(level="unchanged"), str)

    def test_random_generator(self):
        g1 = mut.RandomGenerator()
        self.assertEqual(g1(), 3499211612)
        g2 = mut.RandomGenerator(seed=10)
        self.assertEqual(g2(), 3312796937)

    def test_random_numpy_coordination(self):
        # Verify that multiple numpy generators can be seeded from
        # a single RandomGenerator without duplicating values (as
        # discussed in #12632).
        g = mut.RandomGenerator(123)
        rs1 = np.random.RandomState(seed=g())
        rs1.randint(100)  # discard one
        rs2 = np.random.RandomState(seed=g())
        self.assertNotEqual(rs1.randint(100), rs2.randint(100))

    def test_calc_probability_density(self):
        mut.CalcProbabilityDensity(
            distribution=mut.RandomDistribution.kGaussian,
            x=np.array([0.5, 1.0]),
        )
        mut.CalcProbabilityDensity(
            distribution=mut.RandomDistribution.kGaussian,
            x=np.array([AutoDiffXd(1), AutoDiffXd(2)]),
        )

    def test_assert_is_armed(self):
        self.assertIsInstance(mut.kDrakeAssertIsArmed, bool)

    def test_nice_type_name(self):
        """Tests behavior of ``PyNiceTypeNamePtrOverride`` in module_py.cc."""
        obj = mut_testing.RegisteredType()
        registered_type_py_name = f"{mut_testing.__name__}.RegisteredType"
        registered_type_cc_name = (
            "drake::pydrake::(anonymous)::testing::RegisteredType"
        )
        unregistered_type_cc_name = (
            "drake::pydrake::(anonymous)::testing::UnregisteredType"
        )
        unregistered_derived_type_cc_name = (
            "drake::pydrake::(anonymous)::testing::UnregisteredDerivedType"
        )
        # Type and instance are registered with Python, so it should return the
        # Python type name.
        self.assertEqual(
            mut_testing.get_nice_type_name_cc_registered_instance(obj),
            registered_type_py_name,
        )
        # Type is known, but instance is unregistered, so it should return the
        # C++ type name.
        self.assertEqual(
            mut_testing.get_nice_type_name_cc_unregistered_instance(),
            registered_type_cc_name,
        )
        # Uses raw typeid for a registered type, so it should return the C++
        # type name.
        self.assertEqual(
            mut_testing.get_nice_type_name_cc_typeid(obj),
            registered_type_cc_name,
        )
        # Type and instance are unregistered, so it should return the C++ type
        # name.
        self.assertEqual(
            mut_testing.get_nice_type_name_cc_unregistered_type(),
            unregistered_type_cc_name,
        )
        # Type is unregistered but derived from a registered base type (to
        # mimic Value<> / AbstractValue), and instance is registered. Should
        # return C++ type name.
        base_only_instance = mut_testing.make_cc_unregistered_derived_type()
        self.assertIs(type(base_only_instance), mut_testing.RegisteredType)
        self.assertEqual(
            mut_testing.get_nice_type_name_cc_registered_instance(
                base_only_instance
            ),
            unregistered_derived_type_cc_name,
        )
