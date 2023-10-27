import os
import matplotlib.pyplot as plt
import numpy as np
import Filters as kf

data = np.loadtxt('./resource/dollar.txt', np.float64)
x = np.ndarray((2,1), buffer=np.array([0, 0], dtype=np.float64), dtype=np.float64)
A = np.ndarray((2,2), buffer=np.array([1, 1, 0, 1], dtype=np.float64), dtype=np.float64)
P = np.ndarray((2,2), buffer=np.array([5, 0, 0, 5], dtype=np.float64), dtype=np.float64)
R = np.ndarray((2,2), buffer=np.array([5, 0, 0, 5], dtype=np.float64), dtype=np.float64)
C = np.ndarray((1,2), buffer=np.array([1, 0], dtype=np.float64), dtype=np.float64)
Q = np.ndarray((1,1), buffer=np.array([10], dtype=np.float64), dtype=np.float64)
filter = kf.KalmanFilter(x, A, P, R, C, Q)
approx = []
for i in data:
    approx.append(filter.update(np.array([[i]])))
pred = [filter.predict(i) for i in range(20)]
pred = approx + pred
plt.plot(range(len(data)), data, linewidth=1, label='gt')
plt.plot(range(len(pred)), pred, linewidth=1, label='pred')
plt.legend()
plt.show()