#!/usr/bin/env python3
# Copyright 2022 Joshua Wallace
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
import os
import pickle

import matplotlib.pylab as plt
import numpy as np
import seaborn as sns
from tabulate import tabulate


def getPaths(results):
    paths = []
    for result in results:
        for item in result:
            if 'path' in item:
                paths.append(item['path'])
            else:
                print("Warning: Skipping item with no path")
    return paths


def getTimes(results):
    times = []
    for result in results:
        for item in result:
            if 'planning_time' in item:
                times.append(item['planning_time']['sec'] + item['planning_time']['nanosec'] / 1e9)
            else:
                print("Warning: Skipping item with no planning_time")
    if not times:
        print("Warning: No valid timing data found in results")
    return times


def getMapCoordsFromPaths(paths, resolution):
    coords = []
    for path in paths:
        x = []
        y = []
        for pose in path.poses:
            x.append(pose.pose.position.x / resolution)
            y.append(pose.pose.position.y / resolution)
        coords.append(x)
        coords.append(y)
    return coords


def getPathLength(path):
    path_length = 0
    x_prev = path.poses[0].pose.position.x
    y_prev = path.poses[0].pose.position.y
    for i in range(1, len(path.poses)):
        x_curr = path.poses[i].pose.position.x
        y_curr = path.poses[i].pose.position.y
        path_length = path_length + math.sqrt(
            (x_curr - x_prev) ** 2 + (y_curr - y_prev) ** 2
        )
        x_prev = x_curr
        y_prev = y_curr
    return path_length


def plotResults(costmap, paths):
    coords = getMapCoordsFromPaths(paths, costmap.metadata.resolution)
    data = np.asarray(costmap.data)
    data.resize(costmap.metadata.size_y, costmap.metadata.size_x)
    data = np.where(data <= 253, 0, data)

    plt.figure(3)
    ax = sns.heatmap(data, cmap='Greys', cbar=False)
    for i in range(0, len(coords), 2):
        ax.plot(coords[i], coords[i + 1], linewidth=0.7)
    plt.axis('off')
    ax.set_aspect('equal', 'box')
    plt.show()


def averagePathCost(paths, costmap, num_of_planners):
    coords = getMapCoordsFromPaths(paths, costmap.metadata.resolution)
    data = np.asarray(costmap.data)
    data.resize(costmap.metadata.size_y, costmap.metadata.size_x)

    average_path_costs = []
    for i in range(num_of_planners):
        average_path_costs.append([])

    k = 0
    for i in range(0, len(coords), 2):
        costs = []
        for j in range(len(coords[i])):
            costs.append(data[math.floor(coords[i + 1][j])][math.floor(coords[i][j])])
        average_path_costs[k % num_of_planners].append(sum(costs) / len(costs))
        k += 1

    return average_path_costs


def maxPathCost(paths, costmap, num_of_planners):
    coords = getMapCoordsFromPaths(paths, costmap.metadata.resolution)
    data = np.asarray(costmap.data)
    data.resize(costmap.metadata.size_y, costmap.metadata.size_x)

    max_path_costs = []
    for i in range(num_of_planners):
        max_path_costs.append([])

    k = 0
    for i in range(0, len(coords), 2):
        max_cost = 0
        for j in range(len(coords[i])):
            cost = data[math.floor(coords[i + 1][j])][math.floor(coords[i][j])]
            if max_cost < cost:
                max_cost = cost
        max_path_costs[k % num_of_planners].append(max_cost)
        k += 1

    return max_path_costs


def main():
    print('Read data')
    with open(os.getcwd() + '/results.pickle', 'rb') as f:
        results = pickle.load(f)

    with open(os.getcwd() + '/planners.pickle', 'rb') as f:
        planners = pickle.load(f)

    with open(os.getcwd() + '/costmap.pickle', 'rb') as f:
        costmap = pickle.load(f)

    paths = getPaths(results)
    path_lengths = []

    for path in paths:
        path_lengths.append(getPathLength(path))
    path_lengths = np.asarray(path_lengths)
    total_paths = len(paths)

    path_lengths.resize((int(total_paths / len(planners)), len(planners)))
    path_lengths = path_lengths.transpose()

    times = getTimes(results)
    times_available = len(times) > 0
    if times_available:
        times = np.asarray(times)
        times.resize((int(total_paths / len(planners)), len(planners)))
        times = np.transpose(times)
    else:
        print("Skipping timing-related calculations due to missing data")

    # Costs
    average_path_costs = np.asarray(averagePathCost(paths, costmap, len(planners)))
    max_path_costs = np.asarray(maxPathCost(paths, costmap, len(planners)))

    # Generate table
    planner_table = [
        [
            'Planner',
            'Average path length (m)',
            'Average Time (s)' if times_available else 'Average Time (s) [N/A]',
            'Average cost',
            'Max cost',
        ]
    ]

    for i in range(0, len(planners)):
        planner_table.append(
            [
                planners[i],
                np.average(path_lengths[i]),
                np.average(times[i]) if times_available else 'N/A',
                np.average(average_path_costs[i]),
                np.average(max_path_costs[i]),
            ]
        )

    # Visualize results
    print(tabulate(planner_table))
    plotResults(costmap, paths)


if __name__ == '__main__':
    main()