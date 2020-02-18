import math


SIMULATION_STEP_DURATION = 16
M_PI = 3.14159265358979323846

def ax12_2_deg(X):
    return ((X * 150 / 512) - 150)

def deg_2_ax12(X):
    return ((int)((X + 150) * 512) / 150)

