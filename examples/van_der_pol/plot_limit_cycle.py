# To be a meaningful unit test, we must render the figure somehow.
# The Agg backend (creating a PNG image) is suitably cross-platform.
# Users should feel free to use a different back-end in their own code.
import os
import webbrowser

os.environ["MPLBACKEND"] = "Agg"  # noqa

# Now that the environment is set up, it's safe to import matplotlib, etc.
import matplotlib.pyplot as plt

from pydrake.examples import VanDerPolOscillator

x = VanDerPolOscillator.CalcLimitCycle()

fig, ax = plt.subplots()
ax.plot(x[0, :], x[1, :], color="k", linewidth=2)
ax.set_xlim([-2.5, 2.5])
ax.set_ylim([-3, 3])
ax.set_xlabel("q")
ax.set_ylabel("qdot")

plt.savefig("plot_limit_cycle.png")
assert os.path.exists("plot_limit_cycle.png")

# Show the figure (but not when testing).
if "TEST_TMPDIR" not in os.environ:
    webbrowser.open_new_tab(url="plot_limit_cycle.png")
