import yaml
import numpy as np
import on_ground
import time
import basic_plot as plt

def get_data_array():
    start = time.time()
    with open('lidar_data.yaml', 'r') as stream:
        rows_to_data_lists = yaml.safe_load(stream)
    end = time.time()
    print(str(end - start), "s to load yaml data")

    start = time.time()
    data = []
    for data_list in rows_to_data_lists.values():
        data.extend(data_list)

    data = np.array(data)
    end = time.time()
    print(str(end - start), "s to make ndarray")
    return data

def make_gd(data, n_ransac_trials, closeness_delta, max_height, max_tilt):
    start = time.time()
    gd = on_ground.GroundDetector(data, n_ransac_trials, closeness_delta, max_height, max_tilt)
    end = time.time()
    print(str(end - start), "s to make GroundDetector object")
    return gd

data = get_data_array()
n_ransac_trials = 100
closeness_delta = 0.2
max_height = 0.25
max_tilt = 0.05
gd = make_gd(data, n_ransac_trials, closeness_delta, max_height, max_tilt)

ground, other = [], []
for p in data:
    if gd.on_ground(p):
        ground.append(p)
    else:
        other.append(p)

plt.plot_two(ground, other, n_ransac_trials, closeness_delta)

