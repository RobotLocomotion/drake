import pydrake.systems.scalar_conversion as mut

import copy
import unittest

from pydrake.autodiffutils import AutoDiffXd
from pydrake.symbolic import Expression
from pydrake.systems.framework import LeafSystem_, SystemScalarConverter
from pydrake.util.cpp_template import TemplateClass


@TemplateClass.define("DefaultConverted", param_list=LeafSystem_.param_list)
def DefaultConverted(DefaultConverted, param):
    """Defines a simple templated class which defines a default converter."""
    T, = param
    LeafSystem = LeafSystem_[T]
    # N.B. Due to evaluation order on `add_instantiations`, do NOT create
    # converter here. Wait until first class instance is constructed.
    scalar_helper = mut.ScalarHelper(DefaultConverted)

    class DefaultConvertedInstantiation(LeafSystem):
        def __init__(self, *args, **kwargs):
            LeafSystem.__init__(self, scalar_helper.make_converter())
            other = scalar_helper.check_if_copying(*args, **kwargs)
            if other:
                self.value = other.value
                self.copied_from = other
            else:
                self._construct(*args, **kwargs)

        def _construct(self, value):
            self.value = value
            self.copied_from = None

    return DefaultConvertedInstantiation


class TestScalarConversion(unittest.TestCase):
    def test_converter_attributes(self):
        covnersion_scalars = (
            float, AutoDiffXd, Expression,
        )
        self.assertEqual(
            SystemScalarConverter.SupportedScalars,
            covnersion_scalars)
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

    def test_scalar_helper(self):
        scalar_helper = mut.ScalarHelper(DefaultConverted)

        other = DefaultConverted[float](0)
        self.assertIs(scalar_helper.check_if_copying(other), other)
        # Not copying.
        self.assertIs(scalar_helper.check_if_copying(other, True), None)
        self.assertIs(scalar_helper.check_if_copying(), None)

        # Test conversions.
        converter = scalar_helper.make_converter()
        for T, U in SystemScalarConverter.SupportedConversionPairs:
            self.assertTrue(converter.IsConvertible[T, U]())
        # - Test partial conversion.
        subset_pairs = ((AutoDiffXd, float),)
        converter = scalar_helper.make_converter(subset_pairs)
        self.assertTrue(converter.IsConvertible[AutoDiffXd, float]())
        self.assertFalse(converter.IsConvertible[Expression, float]())

    def test_default_type_converter(self):
        # Test calls that we have available for scalar conversion.
        for T, U in SystemScalarConverter.SupportedConversionPairs:
            system_U = DefaultConverted[U](100)
            self.assertIs(system_U.copied_from, None)
            if T == AutoDiffXd:
                method = LeafSystem_[U].ToAutoDiffXd
            elif T == Expression:
                method = LeafSystem_[U].ToSymbolic
            else:
                continue
            system_T = method(system_U)
            self.assertIsInstance(system_T, DefaultConverted[T])
            self.assertEqual(system_T.value, 100)
            self.assertIs(system_T.copied_from, system_U)
