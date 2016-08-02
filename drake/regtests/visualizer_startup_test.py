#!/usr/bin/env python

"""Tests that DrakeVisualizerApp can be initialized without crashing.
"""

import director.drakevisualizer as vis

app = vis.DrakeVisualizerApp()
app.mainWindow.show()
app.quit()
