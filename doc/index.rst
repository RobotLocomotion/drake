.. raw:: html

    <div style="text-align: center; padding: 10px">
        <img src="_images/logo_w_text.jpg" width="400px"/>
    </div>

********
Overview
********

Drake ("dragon" in Middle English) is a C++ toolbox started by the
`Robot Locomotion Group <http://groups.csail.mit.edu/locomotion/>`_ at the MIT Computer Science and Artificial Intelligence Lab (CSAIL).  The :doc:`development team has now
grown significantly </credits>`, with core development led by the `Toyota Research Institute`_.
It is a collection of tools for analyzing the dynamics of our robots and building control systems for them, with a heavy emphasis on optimization-based design/analysis.

While there are an increasing number of simulation tools available for robotics, most of them function like a black box: commands go in, sensors come out.  Drake aims to simulate even very complex dynamics of robots (e.g. including friction, contact, aerodynamics, ...), but always with an emphasis on exposing the structure in the governing equations (sparsity, analytical gradients, polynomial structure, uncertainty quantification, ...) and making this information available for advanced planning, control, and analysis algorithms.  Drake provides an interface to Python to enable rapid-prototyping of new algorithms, and also aims to provide solid open-source implementations for many state-of-the-art algorithms.  Finally, we hope Drake provides many compelling examples that can help people get started and provide much needed benchmarks.   We are excited to accept user contributions to improve the coverage.

We hope you find this tool useful.  Please see :ref:`getting_help` if you wish
to share your comments, questions, success stories, or frustrations.  And please contribute your best bug fixes, features, and examples!

************
Core Library
************

.. raw:: html

    <table align="center">
        <tr>
            <td style="text-align:center" width="220px">
                Modeling Dynamical Systems
            </td>
            <td style="text-align:center"  width="220px">
                Solving Mathematical Programs
            <td style="text-align:center"  width="220px">
                Multibody Kinematics and Dynamics
            </td>
        </tr>
        <tr>
            <td style="text-align:center">
                <img src="_images/systems.svg" width="195px"/>
            </td>
            <td style="text-align:center">
                <img src="_images/mathematical_program.svg" width="150px"/>
            </td>
            <td style="text-align:center">
                <img src="https://github.com/caelan/pddlstream/raw/d0eb256e88b8b5174fbd136a82867fd9e9cebc67/images/drake_kuka.png" width="195px"/>
            </td>
        </tr>
        <tr>
            <td style="text-align:center">
                <a target="_doc" href="https://drake.mit.edu/doxygen_cxx/group__systems.html">doc</a> | <a target="_tutorial" href="https://mybinder.org/v2/gh/RobotLocomotion/drake/nightly-release?filepath=tutorials/dynamical_systems.ipynb">tutorial</a>
            </td>
            <td style="text-align:center">
                <a target="_doc" href="https://drake.mit.edu/doxygen_cxx/group__solvers.html">doc</a> | <a target="_tutorial" href="https://mybinder.org/v2/gh/RobotLocomotion/drake/nightly-release?filepath=tutorials/mathematical_program.ipynb">tutorial</a>
            </td>
            <td style="text-align:center">
                <a target="_doc" href="https://drake.mit.edu/doxygen_cxx/group__multibody.html">doc</a>
            </td>
        </tr>
    </table>

.. N.B. Leave URL for backwards-compatibility.

.. _tutorials-and-examples:

.. _tutorials:

*********
Tutorials
*********

We have Python tutorials that can be previewed and executed as Jupyter
notebooks online with no need for local installation. You can use Binder to
preview and execute the notebooks (but startup time may be long), or you can
use nbviewer to only preview the notebook (where startup time is fast):

.. raw:: html

    <a target="_doc" href="https://mybinder.org/v2/gh/RobotLocomotion/drake/nightly-release?filepath=tutorials">
      <img src="https://mybinder.org/badge_logo.svg"/>
    </a>
    <a target="_doc" href="https://nbviewer.jupyter.org/github/RobotLocomotion/drake/blob/nightly-release/tutorials/">
      <img src="https://img.shields.io/badge/view%20on-nbviewer-brightgreen.svg"/>
    </a>

If you are browsing on nbviewer, you may click on the Binder button
|nbviewer-binder-button| at the top-right of the page.

.. |nbviewer-binder-button| raw:: html

    <img width="15px" height="15px"
        src="https://nbviewer.jupyter.org/static/img/icon-binder-color.png"/>

You may find more information about how to run these locally with Jupyter,
the branch the tutorials use, how they are published to Binder, etc., in
`drake/tutorials/README.md <https://github.com/RobotLocomotion/drake/tree/master/tutorials/README.md>`_.

.. _examples:

********
Examples
********

.. TODO(russt): make this a table with different algorithms, too.

We have a number of use cases demonstrated under `drake/examples in the
the source tree
<https://github.com/RobotLocomotion/drake/tree/master/examples>`_, and
more available through our :doc:`gallery` (contributions welcome!).

We also have a number of `examples of using Drake as a external library
<https://github.com/RobotLocomotion/drake-external-examples>`_
in your own projects, including examples with various build systems and
examples of how you might set up continuous integration.

************
Citing Drake
************

If you would like to cite Drake in your academic publications, we suggest the following BibTeX citation::

	@misc{drake,
	 author = "Russ Tedrake and the Drake Development Team",
	 title = "Drake: Model-based design and verification for robotics",
	 year = 2019,
	 url = "https://drake.mit.edu"
	}


****************
Acknowledgements
****************

The Drake developers would like to acknowledge significant support from the `Toyota Research Institute`_, `DARPA <http://www.darpa.mil/>`_, the `National Science Foundation <https://nsf.gov/>`_, the `Office of Naval Research <http://www.onr.navy.mil/>`_, `Amazon.com <https://www.amazon.com/>`_, and `The MathWorks <http://www.mathworks.com/>`_.

.. _`Toyota Research Institute`: http://tri.global


**********
Next steps
**********

.. N.B. The #:// is meant to force Sphinx to use a relative HTML URL.
   Unfortunately, it is hard to tell it to link to a section (not a page):
   https://github.com/sphinx-doc/sphinx/issues/701
.. TODO(eric): Replace "Tutorials" with binder badge and direct link, or
   replace with actual relative link.

.. toctree::
   :maxdepth: 1

   gallery
   installation
   Tutorials <https://drake.mit.edu/#tutorials-and-examples>
   API Documentation (C++) <doxygen_cxx/index.html#://>
   API Documentation (Python) <pydrake/index.html#://>
   getting_help
   developers
   credits
   GitHub <https://github.com/RobotLocomotion/drake>


********************************************
Using Drake from other Programming Languages
********************************************
.. toctree::
		:maxdepth: 1

		python_bindings
		julia_bindings
