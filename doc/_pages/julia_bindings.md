---
title: Using Drake from Julia
---


[Julia](http://julialang.org/) is a relatively new, high-level language designed for fast scientific computing. There are two ways to access Drake's functions from Julia: ``PyCall.jl`` and ``Cxx.jl``.

# With ``PyCall.jl``

The [PyCall.jl](https://github.com/JuliaPy/PyCall.jl) package makes it easy to call Python code from Julia, including translation of basic data types like numbers, strings, and arrays to their Python equivalents. Using ``PyCall.jl``, you can run all of the Drake functions which are exposed in the [Drake Python bindings](/python_bindings.html). For a demonstration of this method, see [this example notebook](https://github.com/rdeits/pydrake-julia/blob/master/pydrake_in_julia.ipynb).

# With ``Cxx.jl``

The [Cxx.jl](https://github.com/Keno/Cxx.jl) package allows Julia to call C++ code directly, and even allows Julia to compile new C++ functions on the fly. Since this method allows direct access to the Drake C++ interface, all of Drake's features are available. However, setting up ``Cxx.jl`` may be more complicated than most Julia packages, and careless programming can easily lead to segmentation faults or other undefined behavior, as with any C++ code. For an example of usage, see [this wiki entry by Twan Koolen](https://github.com/tkoolen/drake/wiki/Running-Drake-code-from-Julia%27s-REPL).
