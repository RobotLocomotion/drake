"""
Generates documentation for `drake.mit.edu`.
"""

from os.path import abspath, dirname

from serve_sphinx_base import main

if __name__ == "__main__":
    input_dir = dirname(abspath(__file__))
    main(input_dir=input_dir, strict=True)
