---
title: Unicode Tips & Tricks
---

We allow our developers to use Unicode in comments and text outputs. It is
especially useful to typeset mathematical symbols and equations. Here is an
[example](https://github.com/RobotLocomotion/drake/blob/0c23a1e73759a5399d8be9213be0c81e005b7744/drake/common/autodiff_overloads.h#L77-L82).

```
// The multivariable chain rule states:
// df/dv_i = (∂f/∂x * dx/dv_i) + (∂f/∂y * dy/dv_i)
// ∂f/∂x is y*x^(y-1)
// ∂f/∂y is (x^y)*ln(x)
```

Here we collect tips and tricks that we have found while using Unicode. Please
consider contributing if you find useful things to share:

* [unicodeit.net](http://www.unicodeit.net/) : A website converts LaTeX
  expressions into corresponding Unicode symbols.
* [latex-input](https://github.com/clarkgrubb/latex-input) : Latex input method for
  macOS and Microsoft Windows.
* [Emacs’ TeX input method](https://www.emacswiki.org/emacs/TeXInputMethod) :
  Emacs' input method which allows users to enter Unicode characters by typing
  their (La)TeX names.
* If you experience a problem rendering Unicode symbols in your system, please
  download and install the following fonts on your machine:
  * [Quivira.ttf](http://www.quivira-font.com/files/Quivira.ttf)
  * [Dejavu Fonts](http://sourceforge.net/projects/dejavu/files/dejavu/2.35/dejavu-fonts-ttf-2.35.tar.bz2)
  * [NotoSans](https://github.com/googlei18n/noto-fonts/blob/master/hinted/NotoSans-Regular.ttc?raw=true)
  * [NotoSansSymbols](https://github.com/googlei18n/noto-fonts/blob/master/unhinted/NotoSansSymbols-Regular.ttf?raw=true)
