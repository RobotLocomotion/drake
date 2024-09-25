"""Eventually this program might eventually grow up to be an actual regression
test for memory leaks, but for now it merely serves to demonstrate such leaks.

Currently, it neither asserts the absence of leaks (i.e., a real test) nor the
presence of leaks (i.e., an expect-fail test) -- instead, it's a demonstration
that we can instrument and observe by hand, to gain traction on the problem.
"""

import dataclasses
import gc
import sys

from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import ConstantVectorSource


@dataclasses.dataclass
class RepetitionDetail:
    """Captures some details of an instrumented run: an iteration counter, and
    the count of allocated memory blocks."""
    i: int
    blocks: int | None = None


def _dut_simple_source():
    """A device under test that creates and destroys a leaf system."""
    source = ConstantVectorSource([1.0])


def _dut_trivial_simulator():
    """A device under test that creates and destroys a simulator that contains
    only a single, simple subsystem."""
    builder = DiagramBuilder()
    builder.AddSystem(ConstantVectorSource([1.0]))
    diagram = builder.Build()
    simulator = Simulator(system=diagram)
    simulator.AdvanceTo(1.0)


def _repeat(*, dut: callable, count: int) -> list[RepetitionDetail]:
    """Returns the details of calling dut() for count times in a row."""
    # Pre-allocate all of our return values.
    details = [RepetitionDetail(i=i) for i in range(count)]
    gc.collect()
    tare_blocks = sys.getallocatedblocks()
    # Call the dut repeatedly, keeping stats as we go.
    for i in range(count):
        dut()
        gc.collect()
        details[i].blocks = sys.getallocatedblocks() - tare_blocks
    return details


def _main():
    # TODO(jwnimmer-tri) Add command line flags for selecting which duts to
    # run, change the count, etc. For now, this just hard-codes some things.
    all_duts = dict([
        (dut.__name__, dut)
        for dut in [
            _dut_simple_source,
            _dut_trivial_simulator,
        ]
    ])
    for name, dut in all_duts.items():
        details = _repeat(dut=dut, count=25)
        print(f"RUNNING: {name[1:]}")
        for x in details:
            print(x)


assert __name__ == "__main__", __name__
sys.exit(_main())
