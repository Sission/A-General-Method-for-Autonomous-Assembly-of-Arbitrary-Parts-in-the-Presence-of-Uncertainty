import numpy as np

from uncertainty_reduction import *
import time
from datetime import datetime

file_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'rt_data/force_data.csv')
data = np.loadtxt(file_path, delimiter=',', dtype=np.float64, skiprows=1)

force = data[:, 3:9]
uncertainty = data[:, 0:3]
batch = 31
batch_size = int((batch - 1) / 2 - 3)
cluster = int(force.shape[0] / batch)
# for i in range(1000):
#     if uncertainty[30 + 31 * i, :][2] != 0.15:
#         print(uncertainty[30 + 31 * i, :])
#         print(30 + 31 * i + 2)
print(uncertainty.shape)
uncertainty = uncertainty.reshape(cluster, batch, 3)
force = force.reshape(cluster, batch, 6)
# print(uncertainty[1])
ssq = 100
idx1 = 0
idx2 = 0
for i in range(cluster):
    for j in range(batch_size):
        force1 = force[i][j]
        force2 = force[i][-j - 1]
        diff = force1 - force2
        temp = np.sum(diff ** 2)
        if temp < ssq:
            ssq = temp
            idx1 = i
            idx2 = j

print(ssq)
# print(i, j)
uncertainty1 = uncertainty[idx1][idx2]
uncertainty2 = uncertainty[idx1][-idx2 - 1]
print(uncertainty1, uncertainty2)
print(force[idx1][idx2], force[idx1][-idx2 - 1])
# for i in range():
#     pass
# distance = []
# idx = []
# for i in range(len(force)):
#     diff = force[i, :] - force
#     ssq = np.sum(diff ** 2, axis=1)
#     idxs = np.argsort(ssq)[1:10]
#     distance.append(ssq[idxs])
#     idx.append(idxs)
# distance = np.asarray(distance)
# idx = np.asarray(idx)
#
# mini = np.argsort(distance[:, 0])
# pick = mini[4]
# uncertainty1 = data[pick, 0:3]
# uncertainty2 = data[idx[pick, :][0], 0:3]
# print(uncertainty1)
# print(uncertainty2)
