# Drake Website Preview

To stage changes to Drake Website.

## Stakeholders

* [Eric Cousineau](https://github.com/EricCousineau-TRI/) (TRI) - responsible
on TRI side to incorporate website into the Drake repository, check licensing,
etc.
* [Zack Skynar](https://github.com/devechocharlie) (EchoCharlie) - website
design

## Goal

Put artifacts in as close of a form into the following directory: <br/>
<https://github.com/RobotLocomotion/drake/tree/v0.26.0/doc>

Ensure licensing / derivative works are apparent.

Starting state derived from: <br/>
<https://github.com/jekyll/jekyll/tree/v4.2.0/lib/blank_template>

See `jekyll.LICENSE.md` for Jekyll's licensing terms.

## Tracking Issues

* <https://github.com/RobotLocomotion/drake/issues/14575>

## Relevant Documentation

For documentation regarding client-side technology:

* HTML
    * [W3Schools HTML](https://www.w3schools.com/html/default.asp)
* CSS - Cascading Style Sheets (relying on CSS3)
    * [W3Schools CSS Reference](https://www.w3schools.com/cssref/)
    * [W3Schools CSS Tutorials](https://www.w3schools.com/css/)
    * Advanced features not easily found in above links:
        * [Usage of `@media`](https://www.w3schools.com/css/css3_mediaqueries.asp)
        * [CSS3's Grid Layout, `fr` units](https://www.w3.org/TR/css3-grid-layout/#fr-unit)
* JavaScript (bare basics for this website)
    * [W3Schools JavaScript](https://www.w3schools.com/js/default.asp)

Tools for debugging client-side (HTML, CSS, JavaScript):

* Firefox
    * [Firefox Developer Tools](https://developer.mozilla.org/en-US/docs/Tools)
    * [Open the Page Inspector](https://developer.mozilla.org/en-US/docs/Tools/Page_Inspector/How_to/Open_the_Inspector)
* Chrome / Chromium
    * [Open Chrome DevTools](https://developers.google.com/web/tools/chrome-devtools/open)
    * [Get Started with Viewing And Changing CSS](https://developers.google.com/web/tools/chrome-devtools/css)

All server-side concepts are those supported by Jekyll:

* [Jekyll Documentation](https://jekyllrb.com/docs/), which uses:
    * [Liquid Templating Language](https://shopify.github.io/liquid/basics/introduction/)
    * [Kramdown Flavor of Markdown](https://kramdown.gettalong.org/syntax.html)
* Nuances: <https://jekyllrb.com/tutorials/orderofinterpretation/>

# Caveats

## Using Markdown in HTML tags

Kramdown has the useful feature of being able to use the attribute
`@markdown="1"`. However, it can make things a little weird with indentation,
so be careful and preview often!

For more information: <https://kramdown.gettalong.org/syntax.html#html-blocks>

## Table of Contents

There are some solutions to generating TOC's, with some drawbacks:

### Use Kramdown's `{:toc}` feature.

This is what we use. However, due the
[order in which Jekyll parses things](https://jekyllrb.com/tutorials/orderofinterpretation/), we cannot automagically "wrap" page contents without
doing so explicitly.

However, it's not that bad. :shrug:

Here's an example patch of adding TOC to a page:

```patch
diff --git a/example.md b/example.md
index f33a459..51ba7a4 100644
--- a/example.md
+++ b/example.md
@@ -1,7 +1,11 @@
 ---
 title: Example Page
+layout: page_with_toc
 ---
 
+{% include toc.md %}
+<article class="markdown-body" markdown="1">
+
 # Towards the Start
@@ -39,3 +43,5 @@ # Towards the End
 I have some nice content here.
+
+</article>
```

### Use another plugin

* <https://github.com/allejo/jekyll-toc> is neat, but does some Liquid+Jekyll (and Ruby?) voodoo. Nah for now, maybe later.
    * FWIW They cite the need to repeat stuff as a pain point, so duly noted.
* <https://github.com/toshimaru/jekyll-toc> is a Ruby gem, but has a lot of
logic. Nah for now, maybe later too.
