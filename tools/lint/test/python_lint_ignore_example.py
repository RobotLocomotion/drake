import a
# This should nominally cause an error. However, ignoring E402 in pycodestyle
# via `python_lint` will make it work.
# We also introduce a lack of whitespace for (E226) to ensure our nominal
# defaults are included.
x = 1*3
import b
