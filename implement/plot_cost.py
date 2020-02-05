from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import numpy as np

vmax = 22
dx = 10
amax = 8
t_min = dx/vmax
w1, w2 = 0.1, 0.1


def cost(v1, v2):
    dt = 2*dx / (v1 + v2)
    a = (v2-v1)/dt
    j = w1 * (dt/t_min) + w2 * abs(a/amax)
    return j


fig = plt.figure()
ax = fig.gca(projection='3d')

# Make data.

vel_grid = np.linspace(1, 22, 22)
X, Y = np.meshgrid(vel_grid, vel_grid)

Z = cost(X, Y)

# Plot the surface.
surf = ax.plot_surface(X, Y, Z, cmap=cm.coolwarm,
                       linewidth=0, antialiased=False)

# Customize the z axis.
ax.set_zlim(-1.01, 1.01)
ax.zaxis.set_major_locator(LinearLocator(10))
ax.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))

# Add a color bar which maps values to colors.
fig.colorbar(surf, shrink=0.5, aspect=5)

plt.show()
