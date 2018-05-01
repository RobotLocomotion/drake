import pydrake.systems.scalar_conversion as mut

import copy
import unittest

from pydrake.autodiffutils import AutoDiffXd
from pydrake.symbolic import Expression
from pydrake.systems.framework import LeafSystem_, SystemScalarConverter
from pydrake.util.cpp_template import TemplateClass


@mut.define_convertible_system("Example_")
def Example_(T):

    class ExampleInstantiation(LeafSystem_[T]):
        """Testing example."""

        def _construct(self, value, converter=None):
            LeafSystem_[T].__init__(self, converter)
            self.value = value
            self.copied_from = None

        def _construct_copy(self, other, converter=None):
            LeafSystem_[T].__init__(self, converter)
            self.value = other.value
            self.copied_from = other

    return ExampleInstantiation


Example = Example_[None]


class TestScalarConversion(unittest.TestCase):
    def test_converter_attributes(self):
        conversion_scalars = (
            float, AutoDiffXd, Expression,
        )
        self.assertEqual(
            SystemScalarConverter.SupportedScalars,
            conversion_scalars)
        conversion_pairs = (
            (AutoDiffXd, float),
            (Expression, float),
            (float, AutoDiffXd),
            (Expression, AutoDiffXd),
            (float, Expression),
            (AutoDiffXd, Expression),
        )
        self.assertEqual(
            SystemScalarConverter.SupportedConversionPairs,
            conversion_pairs)

    def test_example_system(self):
        """Tests the Example_ system."""
        # Test template.
        self.assertIsInstance(Example_, TemplateClass)
        self.assertIs(Example_[float], Example)

        # Test parameters.
        param_list = [(T,) for T in SystemScalarConverter.SupportedScalars]
        self.assertEqual(Example_.param_list, param_list)

        # Test private properties (do NOT use these in your code!).
        self.assertEqual(
            tuple(Example_._T_list), SystemScalarConverter.SupportedScalars)
        self.assertEqual(
            tuple(Example_._T_pairs),
            SystemScalarConverter.SupportedConversionPairs)
        converter = Example_._converter
        for T, U in SystemScalarConverter.SupportedConversionPairs:
            self.assertTrue(converter.IsConvertible[T, U]())

        # Test calls that we have available for scalar conversion.
        for T, U in SystemScalarConverter.SupportedConversionPairs:
            system_U = Example_[U](100)
            self.assertIs(system_U.copied_from, None)
            if T == AutoDiffXd:
                method = LeafSystem_[U].ToAutoDiffXd
            elif T == Expression:
                method = LeafSystem_[U].ToSymbolic
            else:
                continue
            system_T = method(system_U)
            self.assertIsInstance(system_T, Example_[T])
            self.assertEqual(system_T.value, 100)
            self.assertIs(system_T.copied_from, system_U)

    def test_define_convertible_system_api(self):
        """Tests more advanced API of `define_convertible_system`, both
        positive and negative tests."""

        def generic_instantiation_func(T):

            class GenericInstantiation(LeafSystem_[T]):
                def _construct(self, converter=None):
                    LeafSystem_[T].__init__(self, converter)

                def _construct_copy(self, other, converter=None):
                    LeafSystem_[T].__init__(self, converter)

            return GenericInstantiation

        # Non-symbolic
        # - Implicit conversion pairs.
        T_list = [float, AutoDiffXd]
        T_pairs_full = [
            (AutoDiffXd, float),
            (float, AutoDiffXd),
        ]
        A = mut.define_convertible_system("A", T_list=T_list)(
            generic_instantiation_func)
        self.assertEqual(A._T_list, T_list)
        self.assertEqual(A._T_pairs, T_pairs_full)

        # - Explicit conversion pairs.
        T_pairs = [
            (float, AutoDiffXd),
        ]
        B = mut.define_convertible_system("B", T_list=T_list, T_pairs=T_pairs)(
            generic_instantiation_func)
        self.assertEqual(B._T_list, T_list)
        self.assertEqual(B._T_pairs, T_pairs)

        # Negative tests.
        # - Not a supported scalar.
        T_list_bad = [int, float]
        with self.assertRaises(AssertionError):
            mut.define_convertible_system("C", T_list=T_list_bad)
        # - Not in original param list.
        T_pairs_bad = [
            (float, Expression),
        ]
        with self.assertRaises(AssertionError):
            mut.define_convertible_system(
                "C", T_list=T_list, T_pairs=T_pairs_bad)
        # - Unsupported conversion.
        T_pairs_unsupported = [
            (float, float),
        ]
        with self.assertRaises(AssertionError):
            mut.define_convertible_system("C", T_pairs=T_pairs_unsupported)

    def test_inheritance(self):

        @mut.define_convertible_system("Child_")
        def Child_(T):

            class ChildInstantiation(Example_[T]):
                def _construct(self, converter=None):
                    Example_[T].__init__(self, 1000, converter=converter)

                def _construct_copy(self, other, converter=None):
                    Example_[T].__init__(self, other, converter=converter)

            return ChildInstantiation

        c_float = Child_[float]()
        self.assertIsInstance(c_float, Child_[float])
        self.assertIsInstance(c_float, Example_[float])
        self.assertEqual(c_float.value, 1000)
        self.assertIs(c_float.copied_from, None)
        c_ad = c_float.ToAutoDiffXd()
        self.assertEqual(c_ad.value, 1000)
        self.assertIs(c_ad.copied_from, c_float)
        self.assertIsInstance(c_ad, Child_[AutoDiffXd])
        self.assertIsInstance(c_ad, Example_[AutoDiffXd])

    def test_bad_class_definitions(self):
        """Tests bad class definitions."""
        bad_init = "Convertible systems should not define"

        # Should not define `__init__`.
        @mut.define_convertible_system("NoInit_")
        def NoInit_(T):

            class NoInitInstantiation(LeafSystem_[T]):
                def __init__(self):
                    pass

                def _construct(self, converter=None):
                    pass

                def _construct_copy(self, converter=None):
                    pass

            return NoInitInstantiation

        with self.assertRaises(RuntimeError) as cm:
            NoInit_[float]
        self.assertIn(bad_init, str(cm.exception))

        # Should define `_construct_copy`.
        @mut.define_convertible_system("NoConstructCopy_")
        def NoConstructCopy_(T):

            class NoConstructCopyInstantiation(LeafSystem_[T]):
                def _construct(self, converter=None):
                    pass

            return NoConstructCopyInstantiation

        with self.assertRaises(RuntimeError) as cm:
            NoConstructCopy_[float]
        self.assertIn(bad_init, str(cm.exception))

        # Should inherit from `LeafSystem_[T]`.
        @mut.define_convertible_system("BadParenting_")
        def BadParenting_(T):

            class BadParentingInstantiation(object):
                def __init__(self):
                    pass

                def _construct(self, converter=None):
                    pass

                def _construct_copy(self, converter=None):
                    pass

            return BadParentingInstantiation

        with self.assertRaises(RuntimeError) as cm:
            BadParenting_[float]
        # N.B. Since we check the class before the instantiation has completed,
        # we will not have the templated name.
        self.assertIn("BadParentingInstantiation", str(cm.exception))
        self.assertIn("LeafSystem_[T]", str(cm.exception))
        self.assertIn("T=float", str(cm.exception))
