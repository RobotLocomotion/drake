**************
For Developers
**************

If you have improvements to Drake, send us your pull requests! 

The standard github workflow is to fork the drake repository into your
own github account, push your changes into a branch on that account,
then (when your code is definitely ready) open a `pull request
<https://help.github.com/articles/using-pull-requests/>`_ via the
github website.  Please see the links below for information on code
style, etc, and be prepared to engage in active code review on your
pull request.

Your code will undergo a code review and will run the unit and
regression tests on a number of build servers.  You can run those
tests locally by running ``make test`` from the command line -- note
that there are a lot of computationally demanding tests and this could
run for a few hours depending on your machine.

Your change must include unit tests that protect it against regressions,
and those tests must pass on all platforms supported by Drake.  Please
use the googletest framework, which is already available in the superbuild.

We would like to hear about your success stories if you've used
Drake in your own projects.  Please consider contributing to our :doc:`gallery`
by editing ``gallery.rst`` in the ``drake/doc/`` directory and submitting a pull
request.

Important note: Drake is an open source project licensed under
extremely flexible terms intended to encourage use by anyone, for any
purpose. When you make a contribution to the Drake project, you are
agreeing to do so under those same terms.

Here are some useful links

.. toctree::
	:maxdepth: 1

	code_style_guide
	jenkins
	known_issue
	no_push_to_origin

Here are some tips for using our favorite IDEs for development with Drake

* `CLion <https://github.com/tkoolen/drake/wiki/CLion-setup-(experimental)>`_
* `Eclipse <https://github.com/tkoolen/drake/wiki/Eclipse-setup-(experimental)>`_



