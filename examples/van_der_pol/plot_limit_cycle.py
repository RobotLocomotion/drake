import matplotlib.pyplot as plt

from pydrake.examples.van_der_pol import VanDerPolOscillator

x = VanDerPolOscillator.CalcLimitCycle()

fig, ax = plt.subplots()
ax.plot(x[0, :], x[1, :], color='k', linewidth=2)
ax.set_xlim([-2.5, 2.5])
ax.set_ylim([-3, 3])
ax.set_xlabel('q')
ax.set_ylabel('qdot')

plt.show()
