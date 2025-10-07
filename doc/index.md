---
layout: default
title: Model-Based Design and Verification for Robotics
description: A C++ / Python toolbox supported by the Toyota Research Institute.
# ^^^ IMPORTANT NOTE Any changes to the wording of the "description" line above
# must be vetted by TRI legal (at least, insofar as the line still makes
# mention of the Toyota Research Institute).
---

<!--
N.B. We mix HTML and Markdown based on which one is more expressive for the
purpose of the front page (HTML for layout, Markdown for content).
-->

<div class="page-home">

<section class="drake-hero padding">
  <div class="contain">
    <div class="drake-hero-title">
      <h1 class="large">{{ page.title }}</h1>
    </div>
    <p class="drake-hero-intro">
      {{ page.description }}
    </p>
    <div class="drake-hero-buttons">
      <a href="/installation.html" class="button">Install</a>
      <a href="#core" class="button grey">Learn More</a>
    </div>
    <div class="hero-image">
      <img src="{{ site.custom.hero_image | relative_url }}">
    </div>
  </div>
</section>

<section id="core" class="core padding">
  <div class="contain">
    <h2>Core Library</h2>
    <div class="grid grid-3col">
      <div class="core-el">
        <h4>Modeling Dynamical<br> Systems</h4>
        <div class="core-el-buttons">
          <a class="button--text" href="/doxygen_cxx/group__systems.html" target="_blank">API</a><a class="button--text" href="https://deepnote.com/workspace/Drake-0b3b2c53-a7ad-441b-80f8-bf8350752305/project/Tutorials-2b4fc509-aef2-417d-a40d-6071dfed9199/notebook/dynamicalsystems-0da445e96cc24a358022b5075d7b0826" target="_blank">TUTORIAL</a>
        </div>
      </div>
      <div class="core-el">
        <h4>Solving Mathematical<br> Programs</h4>
        <div class="core-el-buttons">
          <a class="button--text" href="/doxygen_cxx/group__solvers.html" target="_blank">API</a><a class="button--text" href="https://deepnote.com/workspace/Drake-0b3b2c53-a7ad-441b-80f8-bf8350752305/project/Tutorials-2b4fc509-aef2-417d-a40d-6071dfed9199/notebook/mathematicalprogram-a901caa611f54d3e9302233a4df1b0af" target="_blank">TUTORIAL</a>
        </div>
      </div>
      <div class="core-el">
        <h4>Multibody Kinematics<br> and Dynamics</h4>
        <div class="core-el-buttons">
          <a class="button--text" href="/doxygen_cxx/group__multibody.html" target="_blank">API</a><a class="button--text" href="https://deepnote.com/workspace/Drake-0b3b2c53-a7ad-441b-80f8-bf8350752305/project/Tutorials-2b4fc509-aef2-417d-a40d-6071dfed9199/notebook/authoringmultibodysimulation-8159e3be6dc048a6a8ab6876017def4b" target="_blank">TUTORIAL</a>
        </div>
      </div>
    </div>
  </div>
</section>

<section class="home-blocks padding">
  <div class="contain markdown-body">
    <div class="home-blocks-grid grid grid-2col grid-wide" markdown="1">

<!-- Begin main content. -->

<article markdown="1">
{% include video-autoplay.html
  url = "https://user-images.githubusercontent.com/26719449/108152577-4d14f300-70a7-11eb-88c5-5da0c39b1e24.mp4"
%}
</article>


<article markdown="1">
## Overview

Drake ("dragon" in Middle English) is a C++ toolbox started by the
[Robot Locomotion Group](http://groups.csail.mit.edu/locomotion/) at the MIT
Computer Science and Artificial Intelligence Lab (CSAIL). The
[development team](/credits.html) has now grown significantly, with core
development led by the [Toyota Research Institute](https://www.tri.global/). It
is a collection of tools for analyzing the dynamics of our robots and building
control systems for them, with a heavy emphasis on optimization-based
design/analysis.

While there are an increasing number of simulation tools available for
robotics, most of them function like a black box: commands go in, sensors come
out. Drake aims to simulate even very complex dynamics of robots (e.g.
including friction, contact, aerodynamics, …), but always with an emphasis on
exposing the structure in the governing equations (sparsity, analytical
gradients, polynomial structure, uncertainty quantification, …) and making this
information available for advanced planning, control, and analysis algorithms.
Drake provides an interface to Python to enable rapid-prototyping of new
algorithms, and also aims to provide solid open-source implementations for many
state-of-the-art algorithms. Finally, we hope Drake provides many compelling
examples that can help people get started and provide much needed benchmarks.
We are excited to accept user contributions to improve the coverage.

You can read more about the vision for Drake in this [blog
post](https://medium.com/toyotaresearch/drake-model-based-design-in-the-age-of-robotics-and-machine-learning-59938c985515).

We hope you find this tool useful. Please see
[Getting Help](/getting_help.html) if you wish to share your comments,
questions, success stories, or frustrations. And please contribute your best
bug fixes, features, and examples!
</article>


<article markdown="1">
## Tutorials

Drake offers Python-based tutorials using Jupyter notebooks. We recommend that
you [view the tutorials online](https://deepnote.com/workspace/Drake-0b3b2c53-a7ad-441b-80f8-bf8350752305/project/Tutorials-2b4fc509-aef2-417d-a40d-6071dfed9199/notebook/index-753e3c9d261247ba9f0eb1d7868c18c8).

Alternatively, to run the tutorials locally via `pip`, refer to
[drake/tutorials/README.md](https://github.com/RobotLocomotion/drake/blob/master/tutorials/README.md).

</article>


<article markdown="1">
## Examples

We have a number of use cases demonstrated under drake/examples in the
[source tree](https://github.com/RobotLocomotion/drake/tree/master/examples),
and more available through our [Drake Gallery](/gallery.html) (contributions
welcome!).

We also have a number of [examples of using Drake as a external library](
https://github.com/RobotLocomotion/drake-external-examples) in your own
projects, including examples with various build systems and examples of how you
might set up continuous integration.
</article>


<article markdown="1">
## Articles

[Drake: Model-based design in the age of robotics and machine learning](https://medium.com/toyotaresearch/drake-model-based-design-in-the-age-of-robotics-and-machine-learning-59938c985515)

[Rethinking Contact Simulation for Robot Manipulation](https://medium.com/toyotaresearch/rethinking-contact-simulation-for-robot-manipulation-434a56b5ec88)

[MIT Underactuated Robotics: Algorithms for Walking, Running, Swimming, Flying, and Manipulation](https://underactuated.csail.mit.edu/)

[MIT Robotic Manipulation: Perception, Planning, and Control](https://manipulation.csail.mit.edu/)

</article>


<article markdown="1">
## Citing Drake

```
@misc{drake,
 author = "Russ Tedrake and the Drake Development Team",
 title = "Drake: Model-based design and verification for robotics",
 year = 2019,
 url = "https://drake.mit.edu"
}
```
</article>


<article markdown="1">
## Acknowledgements

The Drake developers would like to acknowledge significant support from the
[Toyota Research Institute](http://tri.global/),
[DARPA](http://www.darpa.mil/), the
[National Science Foundation](https://nsf.gov/), the
[Office of Naval Research](https://www.nre.navy.mil/),
[Amazon.com](https://www.amazon.com/), and
[The MathWorks](http://www.mathworks.com/).
</article>


<article markdown="1">
## Integrations

[Python](./python_bindings.html)

[LCM](./doxygen_cxx/group__message__passing.html)

[ROS 2](https://github.com/RobotLocomotion/drake-ros)™ (unsupported)

[Julia](./julia_bindings.html) (unsupported)

</article>

<!-- End main content. -->

</div>
</div>
</section>

</div>
