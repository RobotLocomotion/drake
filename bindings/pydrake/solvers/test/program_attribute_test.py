import unittest
from pydrake.solvers import (
    ProgramAttribute,
    ProgramType
)


class TestProgramAttribute(unittest.TestCase):
    def test_program_attribute_enum(self):
        enum_expected = [ProgramAttribute.kGenericCost,
                         ProgramAttribute.kGenericConstraint,
                         ProgramAttribute.kQuadraticCost,
                         ProgramAttribute.kQuadraticConstraint,
                         ProgramAttribute.kLinearCost,
                         ProgramAttribute.kLinearConstraint,
                         ProgramAttribute.kLinearEqualityConstraint,
                         ProgramAttribute.kLinearComplementarityConstraint,
                         ProgramAttribute.kLorentzConeConstraint,
                         ProgramAttribute.kRotatedLorentzConeConstraint,
                         ProgramAttribute.kPositiveSemidefiniteConstraint,
                         ProgramAttribute.kExponentialConeConstraint,
                         ProgramAttribute.kL2NormCost,
                         ProgramAttribute.kBinaryVariable,
                         ProgramAttribute.kCallback,
                         ]


class TestProgramType(unittest.TestCase):
    def test_program_attribute_enum(self):
        enum_expected = [ProgramType.kLP,
                         ProgramType.kQP,
                         ProgramType.kSOCP,
                         ProgramType.kSDP,
                         ProgramType.kGP,
                         ProgramType.kCGP,
                         ProgramType.kMILP,
                         ProgramType.kMIQP,
                         ProgramType.kMISOCP,
                         ProgramType.kMISDP,
                         ProgramType.kQuadraticCostConicConstraint,
                         ProgramType.kNLP,
                         ProgramType.kLCP,
                         ProgramType.kUnknown
                         ]
