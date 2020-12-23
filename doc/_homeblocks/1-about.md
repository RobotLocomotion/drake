---
title: Overview
---

**Drake ("dragon" in Middle English) is a C++ toolbox started by the
[Robot Locomotion Group](http://groups.csail.mit.edu/locomotion/) at the MIT Computer Science and Artificial Intelligence Lab (CSAIL). The [development team has now
grown significantly](/credits.html), with core development led by the [Toyota Research Institute](https://www.tri.global/).
It is a collection of tools for analyzing the dynamics of our robots and building control systems for them, with a heavy emphasis on optimization-based design/analysis.**

While there are an increasing number of simulation tools available for robotics, most of them function like a black box: commands go in, sensors come out. Drake aims to simulate even very complex dynamics of robots (e.g. including friction, contact, aerodynamics, …), but always with an emphasis on exposing the structure in the governing equations (sparsity, analytical gradients, polynomial structure, uncertainty quantification, …) and making this information available for advanced planning, control, and analysis algorithms. Drake provides an interface to Python to enable rapid-prototyping of new algorithms, and also aims to provide solid open-source implementations for many state-of-the-art algorithms. Finally, we hope Drake provides many compelling examples that can help people get started and provide much needed benchmarks. We are excited to accept user contributions to improve the coverage.

We hope you find this tool useful. Please see [Getting Help](/getting_help.html) if you wish to share your comments, questions, success stories, or frustrations. And please contribute your best bug fixes, features, and examples!
