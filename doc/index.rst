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

    <table align="center" style="border-spacing: 50px">
        <tr>
            <td style="text-align:center;vertical-align:top" width="200px" height="150px">
                <a href="https://drake.mit.edu/doxygen_cxx/group__systems.html">Modeling Dynamical Systems</a>
                <p/>
                <img src="_images/systems.svg" width="195px"/>
            </td>
            <td style="text-align:center;vertical-align:top" width="200px">
                <a target="_mathematical_program" href="https://colab.research.google.com/github/RussTedrake/underactuated/blob/master/src/mathematical_program_examples.ipynb">
                Solving Mathematical Programs</a>
                <p/>
                <img src="_images/mathematical_program.svg" width="150px"/>
            </td>
            <td style="text-align:center;vertical-align:top" width="200px">
                <a href="https://drake.mit.edu/doxygen_cxx/group__multibody.html">Multibody Kinematics and Dynamics</a>
                <p/>
                <img src="https://github.com/caelan/pddlstream/raw/d0eb256e88b8b5174fbd136a82867fd9e9cebc67/images/drake_kuka.png" width="195px"/>
            </td>
        </tr>
    </table>

********
Examples
********

.. TODO(russt): make this a table with different algorithms, too.

We have a number of use cases demonstrated in `the examples directory of
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

.. toctree::
   :maxdepth: 1

   installation
   gallery
   getting_help
   API Documentation (C++) <doxygen_cxx/index.html#://>
   API Documentation (Python) <pydrake/index.html#://>
   GitHub <https://github.com/RobotLocomotion/drake>
   developers
   credits


********************************************
Using Drake from other Programming Languages
********************************************
.. toctree::
		:maxdepth: 1

		python_bindings
		julia_bindings
