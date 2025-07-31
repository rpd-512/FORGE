import algorithm_modules_cpp as forge
from loaders import load_dh_params, load_scene
from math import dist
import sys
import numpy as np
import time

pop = 200
itr = 200

if len(sys.argv) < 3:
    print("Usage: python analyser.py <scene.json> <dh_params.yaml>")
    sys.exit(1)

scene_file = sys.argv[1]
dh_params_file = sys.argv[2]

gen_pop = forge.generateChromosome(pop, 6)

robot = forge.RobotInfo()
robot.name = "fanuc_m20ia"
robot.dh_params = load_dh_params(dh_params_file)
robot.dof = len(robot.dh_params)
robot.scene_objects = load_scene(scene_file)
robot.joint_angle = [0.0] * robot.dof
robot.init_pos = forge.forward_kinematics(robot.joint_angle, robot)[-1]
robot.destination = forge.position3D(100.0, 100.0, 100.0)

# Define algorithms and their names
algorithms = {
    "Particle Swarm Optimization": forge.particleSwarmOptimization,
    "Genetic Algorithm": forge.geneticAlgorithm,
    "Social Group Optimization": forge.socialGroupOptimization,
    "Teaching Learning Based Optimization": forge.teachingLearningBasedOptimization,
    "Differential Evolution": forge.differentialEvolutionAlgorithm,
}

# Run analyser
def run_analysis(algorithm_func, algo_name, runs=100):
    all_genes = []
    pos_distances = []
    print(f"\n=== {algo_name} ===")
    start_time = time.time()

    for _ in range(runs):
        output = algorithm_func(pop, itr, gen_pop, robot, True)
        gene = output.best_gene
        all_genes.append(gene)
    
    end_time = time.time()
    total_time = end_time - start_time
    avg_time = total_time / runs

    for gene in all_genes:
        pos = forge.forward_kinematics(gene, robot)[-1]
        pos_distance = dist((pos.x, pos.y, pos.z), (robot.destination.x, robot.destination.y, robot.destination.z))
        pos_distances.append(pos_distance)


    genes_array = np.array(all_genes)

    means = np.mean(genes_array, axis=0)
    stds = np.std(genes_array, axis=0)
    mins = np.min(genes_array, axis=0)
    maxs = np.max(genes_array, axis=0)

    mean_position = np.mean(pos_distances)
    std_position = np.std(pos_distances)

    for i in range(6):
        print(f"Joint {i+1}: Mean = {means[i]:.4f}, SD = {stds[i]:.4f}, Range = [{mins[i]:.4f}, {maxs[i]:.4f}]")

    print(f"Total Time: {total_time:.2f} seconds for {runs} runs")
    print(f"Average Time per Run: {avg_time:.4f} seconds")

    print(f"Mean Position Distance: {mean_position:.4f}, SD: {std_position:.4f}")
    print(f"Best Position Distance: {min(pos_distances):.4f}")

# Run for all algorithms
if __name__ == "__main__":
    for algo_name, algo_func in algorithms.items():
        run_analysis(algo_func, algo_name)
