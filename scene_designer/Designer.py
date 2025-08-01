import sys
import yaml
import json
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Slider
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

from designer_modules_cpp import (
    position3D, dh_param, RobotInfo,
    forward_kinematics,
    line_intersects_aabb,
    line_intersects_sphere,
    line_intersects_cylinder,
)

if len(sys.argv) < 3:
    print("Usage: python3 Designer.py <scene_file.json> <dh_file.yaml>")
    sys.exit(1)

FILE_PATH = sys.argv[1]
DH_FILE_PATH = sys.argv[2]
POLL_INTERVAL = 1000  # ms
SCENE_LIMIT = 1000  # constant bounds

def vec_to_pos3D(vec):
    return position3D(vec[0], vec[1], vec[2])

def load_dh_params(file_path):
    with open(file_path, 'r') as f:
        data = yaml.safe_load(f)
    dhpar = []
    for joint in data['dh_parameters']:
        d = dh_param(joint['a'], joint['d'], joint['alpha'])
        dhpar.append(d)
    return dhpar

def draw_cube(ax, position, size, color='blue'):
    x, y, z = position
    dx, dy, dz = size
    vertices = np.array([
        [x, y, z],
        [x+dx, y, z],
        [x+dx, y+dy, z],
        [x, y+dy, z],
        [x, y, z+dz],
        [x+dx, y, z+dz],
        [x+dx, y+dy, z+dz],
        [x, y+dy, z+dz]
    ])
    faces = [
        [vertices[j] for j in [0,1,2,3]],
        [vertices[j] for j in [4,5,6,7]],
        [vertices[j] for j in [0,1,5,4]],
        [vertices[j] for j in [2,3,7,6]],
        [vertices[j] for j in [1,2,6,5]],
        [vertices[j] for j in [0,3,7,4]],
    ]
    poly3d = Poly3DCollection(faces, facecolors=color, edgecolors='k', linewidths=1, alpha=0.6)
    ax.add_collection3d(poly3d)

def draw_sphere(ax, position, radius, color='red'):
    u, v = np.mgrid[0:2*np.pi:24j, 0:np.pi:12j]
    x = radius * np.cos(u) * np.sin(v) + position[0]
    y = radius * np.sin(u) * np.sin(v) + position[1]
    z = radius * np.cos(v) + position[2]
    ax.plot_surface(x, y, z, color=color, alpha=0.6, edgecolor='k')

def draw_cylinder(ax, position, radius, height, color='green'):
    z = np.linspace(position[2], position[2]+height, 30)
    theta = np.linspace(0, 2*np.pi, 30)
    theta_grid, z_grid = np.meshgrid(theta, z)
    x_grid = radius * np.cos(theta_grid) + position[0]
    y_grid = radius * np.sin(theta_grid) + position[1]
    ax.plot_surface(x_grid, y_grid, z_grid, color=color, alpha=0.6, edgecolor='k')
    circle_theta = np.linspace(0, 2*np.pi, 30)
    x_cap = radius * np.cos(circle_theta) + position[0]
    y_cap = radius * np.sin(circle_theta) + position[1]
    z_bottom = np.full_like(x_cap, position[2])
    z_top = np.full_like(x_cap, position[2] + height)
    ax.plot_trisurf(x_cap, y_cap, z_bottom, color=color, alpha=0.6, edgecolor='k')
    ax.plot_trisurf(x_cap, y_cap, z_top, color=color, alpha=0.6, edgecolor='k')

def render_scene(ax, shapes):
    for obj in shapes:
        t = obj["type"]
        pos = obj["position"]
        color = obj.get("color", "gray")
        if t == "cube":
            draw_cube(ax, pos, obj["size"], color=color)
        elif t == "sphere":
            draw_sphere(ax, pos, obj["radius"], color=color)
        elif t == "cylinder":
            draw_cylinder(ax, pos, obj["radius"], obj["height"], color=color)

def check_collision(p1, p2, shapes):
    for obj in shapes:
        t = obj["type"]
        pos = obj["position"]
        if t == "cube":
            size = obj["size"]
            half_size = np.array(size) / 2.0
            min_corner = np.array(pos) - half_size
            max_corner = np.array(pos) + half_size
            if line_intersects_aabb(p1, p2, vec_to_pos3D(min_corner), vec_to_pos3D(max_corner)):
                return True
        elif t == "sphere":
            radius = obj["radius"]
            if line_intersects_sphere(p1, p2, vec_to_pos3D(pos), radius):
                return True
        elif t == "cylinder":
            radius = obj["radius"]
            height = obj["height"]
            if line_intersects_cylinder(p1, p2, vec_to_pos3D(pos), height, radius):
                return True
    return False

def update_plot(val=None):
    ax.clear()
    try:
        with open(FILE_PATH, 'r') as f:
            shapes = json.load(f)
    except Exception as e:
        print("Error loading scene:", e)
        shapes = []
    theta = [np.deg2rad(float(slider.val)) for slider in sliders]
    render_scene(ax, shapes)
    points = forward_kinematics(theta, robot)
    in_collision = False
    for p in range(len(points) - 1):
        in_collision |= check_collision(points[p], points[p+1],shapes)

    # Extract X, Y, Z for plotting
    xs = [pt.x for pt in points]
    ys = [pt.y for pt in points]
    zs = [pt.z for pt in points]

    ax.plot(xs, ys, zs, '-o', lw=2, color='black')
    ax.set_xlim(-SCENE_LIMIT, SCENE_LIMIT)
    ax.set_ylim(-SCENE_LIMIT, SCENE_LIMIT)
    ax.set_zlim(0, SCENE_LIMIT*2)
    ax.set_box_aspect([1, 1, 1])
    ax.set_title("Scene Designer and Robot Visualiser\nIs Colliding: "+str(in_collision), fontsize=14)

    fig.canvas.draw_idle()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

dh_params = load_dh_params(DH_FILE_PATH)
robot = RobotInfo(len(dh_params),dh_params)

sliders = []
slider_ax_start = 0.25
for i in range(robot.dof):
    ax_slider = plt.axes([0.1, slider_ax_start - i*0.04, 0.8, 0.03])
    slider = Slider(ax_slider, f'θ{i+1} (°)', -180, 180, valinit=0)
    slider.on_changed(update_plot)
    sliders.append(slider)

ani = animation.FuncAnimation(fig, lambda frame: update_plot(), interval=POLL_INTERVAL)
update_plot()
plt.show()
