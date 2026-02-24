import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

def generate_curvature_weighted_eight_path(T, N, dt):

    """
        Generate a figure-eight (Lissajous) path with curvature-weighted sampling.

        Args:
            T (float): Total time of the path (used as parametric domain)
            N (int): Number of points for uniform sampling (internal)

        Returns:
            x_ref (ndarray): x reference points
            y_ref (ndarray): y reference points
            yaw_ref (ndarray): heading (yaw) reference
            curvature (ndarray): original curvature values
    """     

    t = np.linspace(0, T, N)

    print("Reference sampling time: ", T / N)
    print("Ratio: ", (T/N) / dt)

    k_x = math.pi / 5
    k_y = k_x
    A = 10.0
    B = 10.0

    x_ref = A * np.cos( k_x * t )
    y_ref = B * np.sin( k_y * t )

    dx_ref = np.gradient(x_ref)
    dy_ref = np.gradient(y_ref)
    yaw_ref = np.arctan2(dy_ref, dx_ref)

    #   Compute curvature
    dx = np.gradient(x_ref, t)
    dy = np.gradient(y_ref, t)

    ddx = np.gradient(dx, t)
    ddy = np.gradient(dy, t)

    numerator = dx * ddy - dy * ddx
    denominator = (dx**2 + dy**2)**1.5 + 1e-8  # Avoid divide by zero

    curvature = numerator / denominator

    print("Maximum curvature: ", max(curvature))
    print("Maximum vehicle curvature: ", math.tan(math.pi/6) / (0.422/2))

    # Normalize curvature to get sampling density
    density = curvature / np.sum(curvature)
    cum_density = np.cumsum(density)
    cum_density = cum_density / cum_density[-1]                         # Normalize to [0, 1]

    # Resample based on curvature
    """from scipy.interpolate import interp1d

    inv_cdf = interp1d(cum_density, t, kind='linear', bounds_error=False, fill_value=(t[0], t[-1]))
    sampled_c = np.linspace(0, 1, N)
    t = inv_cdf(sampled_c)
    x_ref = A * np.cos( k_x * sampled_c )
    y_ref = B * np.sin( k_y * sampled_c )

    dt = np.gradient(t)

    dx = np.gradient(x_ref, t)
    dy = np.gradient(y_ref, t)

    ddx = np.gradient(dx, t)
    ddy = np.gradient(dy, t)

    numerator = dx * ddy - dy * ddx
    denominator = (dx**2 + dy**2)**1.5 + 1e-8  # Avoid divide by zero

    curvature = numerator / denominator

    # Compute yaw reference
    yaw_ref = np.arctan2(np.gradient(y_ref), np.gradient(x_ref))"""

    dist_list = []

    for index in range(N):
        if(index < N - 1):
            dist_list += [ math.sqrt( math.pow(y_ref[index + 1] - y_ref[index], 2) + math.pow(x_ref[index + 1] - x_ref[index], 2) ) ]

    print( "Maximum distance: ", max(dist_list) )
    print( "Minimum distance: ", min(dist_list) )

    print("Ratio distance: ", max(dist_list) / (0.4 * dt) )
    
    # Plot path
    plt.figure(figsize=(6, 6))
    plt.scatter(x_ref, y_ref)
    plt.title("Figure-Eight Path")
    plt.axis('equal')

    plt.figure()
    plt.plot(yaw_ref)
    plt.title("Figure-Eight Path yaw")

    plt.figure()
    plt.plot(curvature)
    plt.title("Path curvature")

    plt.show()
    
    return x_ref, y_ref, yaw_ref