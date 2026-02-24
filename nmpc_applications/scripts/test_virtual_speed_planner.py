import math
import numpy as np
import matplotlib.pyplot as plt

#   Virtual speed functions parameters
L_e = 1e-1
L_k = 1e-1

k_e = 5e0
k_k = 5e-1

x_m_e = 2e0
x_m_k = 1e1

#   Error
error_x = np.arange(0.0, 20.0, 0.01)
v_error = L_e / ( 1 + np.exp( k_e * (error_x - x_m_e) ) )

#   Curvature
curvature_x = np.arange(0.0, 20.0, 0.01)
v_curvature = L_k / ( 1 + np.exp( k_k * (curvature_x - x_m_k) ) )

plt.plot(error_x, v_error, 'b')
plt.plot(curvature_x, v_curvature, 'r')

plt.show()