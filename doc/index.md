---
layout: default
title: Model-Based Design and Verification for Robotics
description: A C++ / Python toolbox supported by the Toyota Research Institute.
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
    <a href="/installation.html" class="button">Install</a>
    <div class="hero-image">
      <img src="{{ site.custom.hero_image | relative_url }}">
    </div>
  </div>
</section>

<section class="core padding">
  <div class="contain">
    <h2>Core Library</h2>
    <div class="grid grid-3col">
      <div class="core-el">
        <h4>Modeling Dynamical<br> Systems</h4>
        <a class="button--text" href="https://drake.mit.edu/doxygen_cxx/group__systems.html" target="_blank">API</a> | <a class="button--text" href="https://mybinder.org/v2/gh/RobotLocomotion/drake/nightly-release?filepath=tutorials/dynamical_systems.ipynb" target="_blank">TUTORIAL</a>
      </div>
      <div class="core-el">
        <h4>Solving Mathematical<br> Programs</h4>
        <a class="button--text" href="https://drake.mit.edu/doxygen_cxx/group__solvers.html" target="_blank">API</a> | <a class="button--text" href="https://mybinder.org/v2/gh/RobotLocomotion/drake/nightly-release?filepath=tutorials/mathematical_program.ipynb" target="_blank">TUTORIAL</a>
      </div>
      <div class="core-el">
        <h4>Multibody Kinematics<br> and Dynamics</h4>
        <a class="button--text" href="https://drake.mit.edu/doxygen_cxx/group__multibody.html" target="_blank">API</a>
      </div>
    </div>
  </div>
</section>

<section class="home-blocks padding">
  <div class="contain markdown-body">
    <div class="home-blocks-grid grid grid-2col grid-wide">
        {% for dev in site.homeblocks %}
<article id="{{ dev.title }}" class="content-container">
{% if dev.display-title != "dont" %}
<h2 class="post__sub_title">{{ dev.title }}</h2>
{% endif %}
{{ dev.content }}
</article>
        {% endfor %}
    </div>
  </div>
</section>

</div>
