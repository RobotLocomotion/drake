"""
Generates documentation for `drake.mit.edu`.
"""

from os.path import abspath, dirname

from drake.doc.jekyll_base import gen_main

if __name__ == "__main__":
    input_dir = dirname(abspath(__file__))
    gen_main(input_dir=input_dir)
