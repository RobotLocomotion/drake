import unittest

from pydrake.solvers import ProgramAttribute, ProgramType


class TestProgramAttribute(unittest.TestCase):
    def test_program_attribute_enum(self):
        # This list checks that all the enums exist to ensure that none are
        # deleted by accident.
        ProgramAttribute.kGenericCost  # noqa: B018
        ProgramAttribute.kGenericConstraint  # noqa: B018
        ProgramAttribute.kQuadraticCost  # noqa: B018
        ProgramAttribute.kQuadraticConstraint  # noqa: B018
        ProgramAttribute.kLinearCost  # noqa: B018
        ProgramAttribute.kLinearConstraint  # noqa: B018
        ProgramAttribute.kLinearEqualityConstraint  # noqa: B018
        ProgramAttribute.kLinearComplementarityConstraint  # noqa: B018
        ProgramAttribute.kLorentzConeConstraint  # noqa: B018
        ProgramAttribute.kRotatedLorentzConeConstraint  # noqa: B018
        ProgramAttribute.kPositiveSemidefiniteConstraint  # noqa: B018
        ProgramAttribute.kExponentialConeConstraint  # noqa: B018
        ProgramAttribute.kL2NormCost  # noqa: B018
        ProgramAttribute.kBinaryVariable  # noqa: B018
        ProgramAttribute.kCallback  # noqa: B018


class TestProgramType(unittest.TestCase):
    def test_program_attribute_enum(self):
        # This list checks that all the enums exist to ensure that none are
        # deleted by accident.
        ProgramType.kLP  # noqa: B018
        ProgramType.kQP  # noqa: B018
        ProgramType.kSOCP  # noqa: B018
        ProgramType.kSDP  # noqa: B018
        ProgramType.kGP  # noqa: B018
        ProgramType.kCGP  # noqa: B018
        ProgramType.kMILP  # noqa: B018
        ProgramType.kMIQP  # noqa: B018
        ProgramType.kMISOCP  # noqa: B018
        ProgramType.kMISDP  # noqa: B018
        ProgramType.kQuadraticCostConicConstraint  # noqa: B018
        ProgramType.kNLP  # noqa: B018
        ProgramType.kLCP  # noqa: B018
        ProgramType.kUnknown  # noqa: B018
