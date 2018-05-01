import pydrake.util.compatibility as mut

# TODO(eric.cousineau): Try to reproduce NumPy formatter patch with mock types.
# Even with creating a `MockSymbol` type which returns a `MockFormula` which
# raises an error on `__nonzero__`, I could not get the error relevant to #8729
# to trigger via `np.maximum.reduce` (but could trigger the error via
# `np.max`).
