import csv
import matplotlib.pyplot as plt
def plot_nn(csv_file):
    with open(csv_file, newline='') as f:
        reader = csv.DictReader(f)
        algos = []
        for row in reader:
            algos.append(row["algorithm"])
    
    nn_count = 0
    nn_list = []
    for i, algo in enumerate(algos):
        if(i==10000): break
        if(algo == "NN_Adam"):
            nn_count += 1
        nn_list.append(nn_count/(i+1))

    plt.plot(nn_list)
    plt.title("NN_Adam Proportion Over Time")
    plt.xlabel("Iterations")
    plt.ylabel("Proportion")
    plt.savefig("nn_adam_proportion_kuka_youbot.png")

# Example usage
if __name__ == "__main__":
    plot_nn("kuka_youbot.csv")
