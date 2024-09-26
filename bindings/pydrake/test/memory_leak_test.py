"""Eventually this program might grow up to be an actual regression test for
memory leaks, but for now it merely serves to demonstrate such leaks.

Currently, it neither asserts the absence of leaks (i.e., a real test) nor the
presence of leaks (i.e., an expect-fail test) -- instead, it's a demonstration
that we can instrument and observe by hand, to gain traction on the problem.
"""

import argparse
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
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--count",
        metavar="N",
        type=int,
        default=25,
        help="Number of iterations to run",
    )
    parser.add_argument(
        "--dut",
        metavar="NAME",
        help="Chooses a device under test; when not given, all DUTs are run.",
    )
    args = parser.parse_args()
    all_duts = dict([
        (dut.__name__[5:], dut)
        for dut in [
            _dut_simple_source,
            _dut_trivial_simulator,
        ]
    ])
    if args.dut:
        run_duts = {args.dut: all_duts[args.dut]}
    else:
        run_duts = all_duts
    for name, dut in run_duts.items():
        details = _repeat(dut=dut, count=args.count)
        print(f"RUNNING: {name}")
        for x in details:
            print(x)


assert __name__ == "__main__", __name__
sys.exit(_main())
