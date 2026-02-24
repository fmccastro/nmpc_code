import matplotlib.pyplot as plt
import numpy as np

"""
    Plot heightmap and normals map
"""

# Create data
x = np.arange(-7, 7, 1e-2)
y = np.arange(-7, 7, 1e-2)
X, Y = np.meshgrid(x, y)

a = 1.0
b = 1.0
c = 0.3

Z = c * np.sin(a * X) * np.cos(b * Y)
#Z = a * X

# Plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

surf = ax.plot_surface(X, Y, Z, cmap='viridis')

ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')

fig.colorbar(surf, ax=ax, shrink=0.5, aspect=10)

plt.show()

nx = -c * a * np.cos(a * X) * np.cos(b * Y)
#nx = -a * np.ones( ( X.shape[0], X.shape[1] ) )

ny = c * b * np.sin(a * X) * np.cos(b * Y)
#ny = np.zeros( ( X.shape[0], X.shape[1] ) )

nz = np.ones( ( X.shape[0], X.shape[1] ) )

norm = np.sqrt(nx**2 + ny**2 + nz**2)
nx /= norm
ny /= norm
nz /= norm

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Surface
ax.plot_surface(X, Y, Z, cmap='viridis', alpha=0.8)

# Downsampling factor
step = 100

# Normal vectors
ax.quiver(
    X[::step, ::step],
    Y[::step, ::step],
    Z[::step, ::step],
    nx[::step, ::step],
    ny[::step, ::step],
    nz[::step, ::step],
    length=0.1,
    normalize=True,
    color='black'
)

ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')

plt.show()