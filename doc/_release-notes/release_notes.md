---
layout: default
title: Release Notes
---

<div class="drake-page">

<header class="drake-page-header">
  <div class="contain">
    <h1>{{ page.title }}</h1>
  </div>
</header>

<section class="padding">
  <div class="contain">
<div class="markdown-body" markdown="1">

# Releases

Latest releases are first:

{% for note in site.release-notes reversed %}
{% if note.title != "Release Notes" %}
* <a href="{{ note.url }}.html">{{ note.title }}</a> (released {{ note.released }})
{% endif %}
{% endfor %}

</div>
  </div>
</section>

</div>
