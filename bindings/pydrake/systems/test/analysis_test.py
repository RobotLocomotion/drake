import unittest

from pydrake.symbolic import Variable
from pydrake.systems.primitives import SymbolicVectorSystem
from pydrake.systems.analysis import (
    RegionOfAttraction,
    RegionOfAttractionOptions
)


class AnalysisTest(unittest.TestCase):
    def test_region_of_attraction(self):
        x = Variable("x")
        sys = SymbolicVectorSystem(state=[x], dynamics=[-x+x**3])
        context = sys.CreateDefaultContext()
        options = RegionOfAttractionOptions()
        options.lyapunov_candidate = x*x
        options.state_variables = [x]
        V = RegionOfAttraction(system=sys, context=context, options=options)
