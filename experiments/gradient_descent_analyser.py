import algorithm_modules_cpp as forge
from loaders import load_dh_params, load_scene
from math import dist
import sys
import numpy as np
import time
import matplotlib.pyplot as plt

pop = 100
itr = 200

if len(sys.argv) < 3:
    print("Usage: python analyser.py <scene.json> <dh_params.yaml>")
    sys.exit(1)

scene_file = sys.argv[1]
dh_params_file = sys.argv[2]

robotname = dh_params_file.split('/')[-1].split('.')[0]

robot = forge.RobotInfo()
robot.name = robotname
robot.dh_params = load_dh_params(dh_params_file)
robot.dof = len(robot.dh_params)
robot.scene_objects = load_scene(scene_file)
robot.joint_angle = [0.0] * robot.dof
robot.init_pos = forge.forward_kinematics(robot.joint_angle, robot)[-1]
robot.destination = forge.position3D(100.0, 10.0, 10.0)

gen_pop = forge.generateChromosome(pop, robot.dof)

# Run PSO
outp = forge.socialGroupOptimization(pop, itr, gen_pop, robot, True)

# Extract and plot PSO distance history
distance_history = np.array(outp.distance_history)
plt.figure(figsize=(8, 5))
plt.plot(range(len(distance_history)), distance_history, label='PSO Output', color='blue')
plt.yscale('log')
plt.xlabel('Iteration')
plt.ylabel('Positional Error (mm)')
plt.title('PSO Optimization and Gradient Descent Refinement')
plt.grid(True, which='both', linestyle='--', linewidth=0.5)

# Run Gradient Descent
start_time = time.time()
gd_out = forge.gradientDescent(100, 0.001, outp.best_gene, robot, True)
end_time = time.time()
total_time = end_time - start_time
print(f"Gradient Descent Time: {total_time*1e6:.3f} us")

# Final error after GD
pos = forge.forward_kinematics(gd_out.best_gene, robot)[-1]
pos = [pos.x, pos.y, pos.z]
dest = [robot.destination.x, robot.destination.y, robot.destination.z]
pos_distance = dist(pos, dest)
print(f"Distance to Destination: {pos_distance:.6f} mm")


# Plot final GD refined result as a point
plt.plot([p+len(distance_history) for p in range(len(gd_out.distance_history))], gd_out.distance_history, color='red', label='After Gradient Descent', zorder=5)

# Legend and save
plt.legend()
plt.tight_layout()
plt.savefig("graphs/pso_gd_refinement.png", dpi=300)
plt.show()
