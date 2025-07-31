import algorithm_modules_cpp as forge
from loaders import load_dh_params, load_scene
from math import dist
import sys
import numpy as np
import time
import matplotlib.pyplot as plt
from scipy.stats import norm
from tqdm import tqdm

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


# Define algorithms and their names
algorithms = {
    "Particle Swarm Optimization": forge.particleSwarmOptimization,
    "Genetic Algorithm": forge.geneticAlgorithm,
    "Social Group Optimization": forge.socialGroupOptimization,
    "Teaching Learning Based Optimization": forge.teachingLearningBasedOptimization,
    "Differential Evolution": forge.differentialEvolutionAlgorithm,
}

def run_analysis(algorithm_func, algo_name, runs=1000):
    all_genes = []
    pos_distances = []
    print(f"\n=== {algo_name} ===")
    start_time = time.time()

    for _ in tqdm(range(runs), desc=f"Running {algo_name}"):
        output = algorithm_func(pop, itr, gen_pop, robot, True)
        gene = output.best_gene
        all_genes.append(gene)

    end_time = time.time()
    total_time = end_time - start_time
    avg_time = total_time / runs

    for gene in all_genes:
        pos = forge.forward_kinematics(gene, robot)[-1]
        pos_distance = dist((pos.x, pos.y, pos.z), 
                            (robot.destination.x, robot.destination.y, robot.destination.z))
        pos_distances.append(pos_distance)

    genes_array = np.array(all_genes)

    means = np.mean(genes_array, axis=0)
    stds = np.std(genes_array, axis=0)
    mins = np.min(genes_array, axis=0)
    maxs = np.max(genes_array, axis=0)

    mean_position = np.mean(pos_distances)
    std_position = np.std(pos_distances)
    outp = f"=== {algo_name} Analysis ===\n"
    for i in range(robot.dof):
        outp+='\n'+(f"Joint {i+1}: Mean = {means[i]:.4f}, SD = {stds[i]:.4f}, Range = [{mins[i]:.4f}, {maxs[i]:.4f}]")

    outp+='\n'+(f"Total Time: {total_time:.2f} seconds for {runs} runs")
    outp+='\n'+(f"Average Time per Run: {avg_time:.4f} seconds")

    outp+='\n'+(f"Mean Position Distance: {mean_position:.4f}, SD: {std_position:.4f}")
    outp+='\n'+(f"Best Position Distance: {min(pos_distances):.4f}")
    outp+='\n'+(f"Worst Position Distance: {max(pos_distances):.4f}")
    outp+='\n\n'
    # === Plotting Distributions for Each Joint ===
    fig, axs = plt.subplots(robot.dof, 1, figsize=(8, 2.5 * robot.dof))
    fig.suptitle(f'Joint Angle Distributions - {algo_name}', fontsize=15)
    #with open(f'statistics/analysis.txt', 'a') as f:
    #    f.write(outp)
    for i in range(robot.dof):
        joint_data = genes_array[:, i]
        mu, sigma = means[i], stds[i]

        ax = axs[i] if robot.dof > 1 else axs
        ax.hist(joint_data, bins=20, density=True, alpha=0.6, color='black', edgecolor='white')

        # Overlay Gaussian curve
        xmin, xmax = ax.get_xlim()
        x_vals = np.linspace(xmin, xmax, 100)
        p = norm.pdf(x_vals, mu, sigma)
        ax.plot(x_vals, p, 'blue', linewidth=2)

        ax.set_title(f'\nJoint {i+1}: μ={mu:.2f}, σ={sigma:.2f}', fontsize=10)
        ax.set_xlabel('Angle (rad)')
        ax.set_ylabel('Density')

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.savefig(f'graphs/{algo_name}_joint_distributions.png')

# Run for all algorithms
if __name__ == "__main__":
    with open(f'statistics/analysis.txt', 'w') as f:
        pass
    for algo_name, algo_func in algorithms.items():
        run_analysis(algo_func, algo_name,1000)
        