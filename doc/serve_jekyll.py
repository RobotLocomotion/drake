"""
Serves documentation for `drake.mit.edu`.
"""

from os.path import abspath, dirname

from drake.doc.jekyll_base import preview_main

if __name__ == "__main__":
    input_dir = dirname(abspath(__file__))
    preview_main(input_dir=input_dir, default_port=8000)
