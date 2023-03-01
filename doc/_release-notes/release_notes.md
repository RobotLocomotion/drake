---
layout: default
title: Release Notes
supplemental: true
hidden: true
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
{% if note.supplemental %}
{% else %}
* <a href="{{ note.url }}.html">{{ note.title }}</a> (released {{ note.released }})
{% endif %}
{% endfor %}

{% for page in site.release-notes %}
{% if page.supplemental %}
{% if page.hidden %}
{% else %}
<h3><a href="{{ page.url }}.html">{{ page.title }}</a></h3>
{% endif %}
{% endif %}
{% endfor %}

</div>
  </div>
</section>

</div>
