import numpy as np
import matplotlib.pyplot as plt
import yaml
import csv
import sys

from mpl_toolkits.mplot3d import Axes3D

def dh_transform(a, d, alpha, theta):
    alpha = np.deg2rad(alpha)
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0,              np.sin(alpha),                np.cos(alpha),               d],
        [0,              0,                            0,                           1]
    ])

def forward_kinematics(dh_params, joint_angles):
    T = np.identity(4)
    positions = [T[:3, 3]]

    for i, param in enumerate(dh_params):
        a = param['a']
        d = param['d']
        alpha = param['alpha']
        theta = joint_angles[i]
        A = dh_transform(a, d, alpha, theta)
        T = T @ A
        positions.append(T[:3, 3])

    return np.array(positions)

def plot_fk(init_positions, final_positions, target):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot initial pose (blue)
    ax.plot(init_positions[:, 0], init_positions[:, 1], init_positions[:, 2], '-ob', label="Initial Pose")

    # Plot final pose (red)
    ax.plot(final_positions[:, 0], final_positions[:, 1], final_positions[:, 2], '-or', label="Final Pose")

    # Target position (green dot)
    ax.scatter(target[0], target[1], target[2], c='green', s=80, label="Target", marker='o')

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.legend()
    ax.set_title("Initial vs Final Pose (with Target)")
    ax.grid(True)
    plt.show()

def load_dh_from_yaml(yaml_path):
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    return data['dh_parameters']

def parse_csv_line(line):
    floats = list(map(float, line.strip().split(",")[:-1]))  # drop the algorithm name
    initial_angles = floats[:6]
    target_pos = floats[6:9]
    final_angles = floats[9:15]
    return initial_angles, target_pos, final_angles

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python visualize_fk_csv.py <config.yaml> <data_line>")
        print("Example: python visualize_fk_csv.py config.yaml \"4.78,5.97,0.45,0.50,5.16,2.71,-657.6,1688.0,730.3,1.88,6.17,1.44,1.40,4.82,2.71,PSO\"")
        sys.exit(1)

    yaml_file = sys.argv[1]
    csv_line = sys.argv[2]

    dh_params = load_dh_from_yaml(yaml_file)
    initial, target, final = parse_csv_line(csv_line)

    # Assume angles are in radians
    init_fk = forward_kinematics(dh_params, initial)
    final_fk = forward_kinematics(dh_params, final)

    plot_fk(init_fk, final_fk, target)
