"""
Produces stiction.png for contact documentation.
"""

import numpy as np
import pylab as plt
import matplotlib.patches as patches


def step5(x):
    '''Python version of MultibodyPlant::StribeckModel::step5 method'''
    x3 = x * x * x
    return x3 * (10 + x * (6 * x - 15))


def stribeck(us, uk, v):
    '''
    Python version of MultibodyPlant::StribeckModel::ComputeFrictionCoefficient
    '''
    u = np.zeros_like(v) + uk
    u[v < 1] = us * step5(v[v < 1])
    mask = (v >= 1) & (v < 3)
    u[mask] = us - (us - uk) * step5((v[mask] - 1) / 2)
    return u


def main():
    x = np.linspace(0, 4, 101)

    us = 0.9
    uk = 0.5
    y = stribeck(us, uk, x)

    f = plt.figure()
    ax = f.add_subplot(111)

    # colored regions
    ax.add_patch(patches.Rectangle((0, 0), 1, 1, alpha=0.1, color=(1, 0, 0)))
    ax.add_patch(patches.Rectangle((1, 0), 2, 1, alpha=0.1, color=(0, 1, 0)))
    ax.add_patch(patches.Rectangle((3, 0), 1, 1, alpha=0.1, color=(1, 1, 0)))

    # vertical lines
    plt.plot((1, 1), (0, 1), 'k', alpha=0.25)
    plt.plot((3, 3), (0, 1), 'k', alpha=0.25)
    # horizontal lines
    plt.plot((0, 4), (us, us), 'k', alpha=0.25)
    plt.plot((0, 4), (uk, uk), 'k', alpha=0.25)

    plt.plot(x, y)
    plt.title('Speed-dependent Coefficient of Friction')
    plt.xlabel('multiples of $v_s$')
    plt.xticks((0, 1, 3))
    plt.yticks((0, uk, us), ('0', r'$\mu_k$', r'$\mu_s$'))
    plt.xlim((0, 4))
    plt.ylim((0, 1))
    plt.ylabel(r'$\mu$')
    image_file = './stribeck.png'
    plt.savefig(image_file)
    print(f"Created: {image_file}")


assert __name__ == "__main__"
main()
