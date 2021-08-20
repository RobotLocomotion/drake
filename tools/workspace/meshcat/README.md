# Upgrading meshcat commit

As is typical of GUI-based applications, meshcat is not easily tested via
automated CI processes. As such, when bumping the meshcat (and meshcat-python)
commits, it is _not_ enough to simply let CI run its course; local testing
is also required.

The local testing consists largely of informal prodding: run an application
which exercises meshcat and confirm that is reasonably well behaved.

Required:
  - drake/geometry:meshcat_manual_test
    - See the documentation at the top of that file for the expected behavior.

Possible options:

  - drake/manipulation/util/show_model.py
    - Simply loads a model into the viewer in its default configuration.
    - Build and execute instructions are contained in the Python file itself.
  - drake/manipulation/util/geometry_inspector.py
    - Loads a model and provides sliders to control its degrees of freedom.
    - Build and execute instructions are contained in the Python file itself.
