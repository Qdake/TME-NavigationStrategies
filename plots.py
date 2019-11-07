#!/usr/bin/env python

from radarGuidance import *
from wallFollower import *

import time
import random #used for the random choice of a strategy
import sys  
import numpy as np
import math
import pickle

import matplotlib.pyplot as plt

with open('log/61_experience_positions_by_step.pickle', 'rb') as handle:
    unserialized_data = pickle.load(handle)

starting_datas = np.zeros((60,60))
for i in range(10):
    data_i = unserialized_data[i]
    for position in data_i:
        x = int(position[0]) // 10
        y = int(position[1]) // 10
        starting_datas[y][x] += 1

ending_datas = np.zeros((60,60))
for i in range(40,50):
    data_i = unserialized_data[i]
    for position in data_i:
        x = int(position[0]) // 10 
        y = int(position[1]) // 10
        ending_datas[y][x] += 1



plt.imshow(starting_datas)
plt.show()
plt.imshow(ending_datas)
plt.show()


"""
with open('log/400_experience_tiralDurationqlearning.txt', 'rb') as f:
    time_duration_data = f.readlines()

durations = []
for t in time_duration_data:
    durations.append(float(t))

med_durations = []
tmp = []
for t in durations:
    tmp.append(t)
    if len(tmp) > 4:
        med_durations.append(np.mean(np.array(tmp)))
        tmp = []

plt.plot(med_durations)
plt.show()
"""

def norm(a):
    return (a - np.mean(a)) / np.std(a)

window_size = 10
all_durations = []
for exp in range(8):
    with open('log/' + str(401 + exp) + '_experience_tiralDurationqlearning.txt', 'rb') as f:
        time_duration_data = f.readlines()

    durations = []
    for t in time_duration_data:
        durations.append(float(t))

    all_durations.append(durations)


med_durations = []
for i in range(len(all_durations[0])):
    med_durations.append(np.median([all_durations[k][i] for k in range(len(all_durations))]))

mean_durations = []
tmp = []
for t in med_durations:
    tmp.append(t)
    if len(tmp) >= window_size:
        mean_durations.append(np.median(np.array(tmp)))
        tmp = tmp[1:]


#plt.plot(np.array(all_durations[:3]).T)
#plt.show()

#plt.plot(norm(np.array(mean_durations)))



window_size = 10
all_durations = []
for exp in range(8):
    with open('log/' + str(401 + exp) + '_experience_tiralLenqlearning.txt', 'rb') as f:
        time_duration_data = f.readlines()

    durations = []
    for t in time_duration_data:
        durations.append(float(t))

    all_durations.append(durations)


med_durations = []
for i in range(len(all_durations[0])):
    med_durations.append(np.median([all_durations[k][i] for k in range(len(all_durations))]))

mean_durations = []
tmp = []
for t in med_durations:
    tmp.append(t)
    if len(tmp) >= window_size:
        mean_durations.append(np.median(np.array(tmp)))
        tmp = tmp[1:]

"""
plt.plot(np.array(all_durations[:3]).T)
plt.show()
"""
"""
plt.plot(norm(np.array(mean_durations)))
plt.show()
"""

"""
with open('log/401_experience_positions_by_step.pickle', 'rb') as handle:
    unserialized_data = pickle.load(handle)
"""


all_bings = []
for exp in range(8):
    with open('log/' + str(50 + exp) + '_experience_tiralBingsqlearning.txt', 'rb') as f:
        bings_data = f.readlines()


    nbBings = []
    for b in bings_data:
        nbBings.append(float(b))

    all_bings.append(nbBings)


mean_bings = []
for i in range(len(all_bings[0])):
    mean_bings.append(np.mean([all_bings[k][i] for k in range(len(all_bings))]))

rolled_bings = []
tmp = []
for b in mean_bings:
    tmp.append(b)
    if len(tmp) >= window_size:
        rolled_bings.append(np.median(np.array(tmp)))
        tmp = tmp[1:]

#plt.plot(rolled_bings)
#plt.show()