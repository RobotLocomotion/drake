"""
Provides some insight into the ManipulationStation model by printing out the
contents of its (default) Context.
"""

from pydrake.examples.manipulation_station import ManipulationStation

station = ManipulationStation()
station.SetupDefaultStation()
station.Finalize()

context = station.CreateDefaultContext()
print(context)
