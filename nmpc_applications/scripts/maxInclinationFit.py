import casadi as ca
import math

"""
    Script to estimate best fit to maximum achievable slopes
"""

k = ca.SX.sym('k')

x = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
y = [0.0, 0.056403, 0.127811, 0.193803, 0.265642, 0.33325, 0.398657, 0.474762]

sum = 0
for i in range( len(x) ):
    sum += ca.power( ca.atan(k * x[i]) - y[i], 2)

nlp = {'x':k, 'f':sum}
S = ca.nlpsol('S', 'ipopt', nlp)

r = S(x0 = 1.0)

x_opt = r['x']

print(x_opt)