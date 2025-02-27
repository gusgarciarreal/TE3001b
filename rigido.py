# Imports
from spatialmath.base import rot2
from sympy import simplify, Matrix
import spatialmath.base as base
import numpy as np

# 2D case
# numeric
R = rot2(0.2)
det = np.linalg.det(R)
print(f"La matriz de rotación es:\n{R} y su determinante es: {det}")
det_RxR = np.linalg.det(R @ R)
print(f"El determinante de R*R es: {det_RxR}")