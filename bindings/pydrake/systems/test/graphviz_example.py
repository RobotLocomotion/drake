#!/usr/bin/env python

import matplotlib.pyplot as plt

from pydrake.systems.drawing import plot_system_graphviz
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import Adder

builder = DiagramBuilder()
size = 1
adders = [
    builder.AddSystem(Adder(1, size)),
    builder.AddSystem(Adder(1, size)),
]
for i, adder in enumerate(adders):
    adder.set_name("adders[{}]".format(i))
builder.Connect(adders[0].get_output_port(0), adders[1].get_input_port(0))
builder.ExportInput(adders[0].get_input_port(0))
builder.ExportOutput(adders[1].get_output_port(0))

diagram = builder.Build()
diagram.set_name("graphviz_example")

plot_system_graphviz(diagram)
plt.show()
