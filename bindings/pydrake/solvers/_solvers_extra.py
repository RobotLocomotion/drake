def _solver_options_repr(self):
    return f"SolverOptions(options={self.options!r})"


SolverOptions.__repr__ = _solver_options_repr
