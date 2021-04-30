import pydrake.systems.scalar_conversion as mut

import copy
import unittest

from pydrake.autodiffutils import AutoDiffXd
from pydrake.symbolic import Expression
from pydrake.systems.framework import (
    DiagramBuilder,
    LeafSystem_,
    SystemScalarConverter,
)
from pydrake.common.cpp_template import TemplateClass


@mut.TemplateSystem.define("Example_")
def Example_(T):

    class Impl(LeafSystem_[T]):
        """Testing example."""

        def _construct(self, value, converter=None):
            LeafSystem_[T].__init__(self, converter=converter)
            self.value = value
            self.copied_from = None

        def _construct_copy(self, other, converter=None):
            Impl._construct(self, other.value, converter=converter)
            self.copied_from = other

    return Impl


Example = Example_[None]


class TestScalarConversion(unittest.TestCase):
    def test_converter_attributes(self):
        conversion_scalars = (
            float, AutoDiffXd, Expression,
        )
        self.assertTupleEqual(
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
        self.assertTupleEqual(
            SystemScalarConverter.SupportedConversionPairs,
            conversion_pairs)

    def test_example_system(self):
        """Tests the Example_ system."""
        # Test template.
        self.assertIsInstance(Example_, TemplateClass)
        self.assertEqual(
            str(Example_), f"<TemplateSystem {__name__}.Example_>")
        self.assertIs(Example_[float], Example)

        # Test parameters.
        param_list = [(T,) for T in SystemScalarConverter.SupportedScalars]
        self.assertListEqual(Example_.param_list, param_list)

        for T in SystemScalarConverter.SupportedScalars:
            system_T = Example_[T](0)
            self.assertEqual(
                system_T.GetSystemType(),
                f"{__name__}.Example_[{T.__name__}]")

        # Test private properties (do NOT use these in your code!).
        self.assertTupleEqual(
            tuple(Example_._T_list), SystemScalarConverter.SupportedScalars)
        self.assertTupleEqual(
            tuple(Example_._T_pairs),
            SystemScalarConverter.SupportedConversionPairs)
        converter = Example_._converter
        for T, U in SystemScalarConverter.SupportedConversionPairs:
            self.assertTrue(converter.IsConvertible[T, U]())

        # Test calls that we have available for scalar conversion.
        for T, U in SystemScalarConverter.SupportedConversionPairs:
            system_U = Example_[U](100)
            self.assertIs(system_U.copied_from, None)
            system_T = system_U.ToScalarType[T]()
            self.assertIsInstance(system_T, Example_[T])
            self.assertEqual(system_T.value, 100)
            self.assertIs(system_T.copied_from, system_U)
            if T == AutoDiffXd:
                system_ad = system_U.ToAutoDiffXd()
                self.assertIsInstance(system_ad, Example_[T])
                self.assertEqual(system_ad.value, 100)
                self.assertIs(system_ad.copied_from, system_U)
            if T == Expression:
                system_sym = system_U.ToSymbolic()
                self.assertIsInstance(system_sym, Example_[T])
                self.assertEqual(system_sym.value, 100)
                self.assertIs(system_sym.copied_from, system_U)

    def test_example_system_in_diagram(self):
        system_f = Example(value=10)
        system_f.set_name("example")
        builder_f = DiagramBuilder()
        builder_f.AddSystem(system_f)
        diagram_f = builder_f.Build()

        diagram_ad = diagram_f.ToAutoDiffXd()
        system_ad = diagram_ad.GetSubsystemByName(system_f.get_name())
        self.assertIsInstance(system_ad, Example_[AutoDiffXd])
        self.assertIs(system_ad.copied_from, system_f)

    def test_define_convertible_system_api(self):
        """Tests more advanced API of `TemplateSystem.define`, both
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
        A = mut.TemplateSystem.define("A", T_list=T_list)(
            generic_instantiation_func)
        self.assertListEqual(A._T_list, T_list)
        self.assertListEqual(A._T_pairs, T_pairs_full)

        # - Explicit conversion pairs.
        T_pairs = [
            (float, AutoDiffXd),
        ]
        B = mut.TemplateSystem.define("B", T_list=T_list, T_pairs=T_pairs)(
            generic_instantiation_func)
        self.assertListEqual(B._T_list, T_list)
        self.assertListEqual(B._T_pairs, T_pairs)

        # Negative tests.
        # - Not a supported scalar.
        T_list_bad = [int, float]
        with self.assertRaises(AssertionError):
            mut.TemplateSystem.define("C", T_list=T_list_bad)
        # - Not in original `T_list`.
        T_pairs_bad = [
            (float, Expression),
        ]
        with self.assertRaises(AssertionError):
            mut.TemplateSystem.define(
                "C", T_list=T_list, T_pairs=T_pairs_bad)
        # - Unsupported conversion.
        T_pairs_unsupported = [
            (float, float),
        ]
        with self.assertRaises(AssertionError):
            mut.TemplateSystem.define("C", T_pairs=T_pairs_unsupported)

    def test_inheritance(self):

        @mut.TemplateSystem.define("Child_")
        def Child_(T):

            class Impl(Example_[T]):
                def _construct(self, converter=None):
                    Example_[T].__init__(self, 1000, converter=converter)

                def _construct_copy(self, other, converter=None):
                    Example_[T].__init__(self, other, converter=converter)

            return Impl

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

        # Should not define `__init__`.
        @mut.TemplateSystem.define("NoInit_")
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
        self.assertIn(
            "NoInit_[float] defines `__init__`, but should not",
            str(cm.exception))

        # Should define `_construct_copy`.
        @mut.TemplateSystem.define("NoConstructCopy_")
        def NoConstructCopy_(T):

            class NoConstructCopyInstantiation(LeafSystem_[T]):
                def _construct(self, converter=None):
                    pass

            return NoConstructCopyInstantiation

        with self.assertRaises(RuntimeError) as cm:
            NoConstructCopy_[float]
        self.assertIn(
            "NoConstructCopy_[float] does not define `_construct_copy`",
            str(cm.exception))

        # Should inherit from `LeafSystem_[T]`.
        @mut.TemplateSystem.define("BadParenting_")
        def BadParenting_(T):

            class BadParentingInstantiation:
                def __init__(self):
                    pass

                def _construct(self, converter=None):
                    pass

                def _construct_copy(self, converter=None):
                    pass

            return BadParentingInstantiation

        with self.assertRaises(RuntimeError) as cm:
            BadParenting_[float]
        self.assertIn("BadParenting_[float]", str(cm.exception))
        self.assertIn("LeafSystem_[float]", str(cm.exception))
