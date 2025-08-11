import csv
from collections import Counter
import matplotlib.pyplot as plt

def count_algorithms(csv_file_path):
    algo_counter = Counter()

    with open(csv_file_path, newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            algo = row['algorithm']
            algo_counter[algo] += 1

    return algo_counter

# Usage
if __name__ == "__main__":
    file_path = "../build/fanuc_m20ia.csv"
    counts = count_algorithms(file_path)

    for algo, count in counts.items():
        print(f"{algo}: {count}")

    labels = list(counts.keys())
    sizes = list(counts.values())

    fig, ax = plt.subplots(figsize=(8, 8))
    wedges, texts, autotexts = ax.pie(
        sizes,
        labels=labels,
        autopct='%1.1f%%',
        startangle=140,
        wedgeprops=dict(edgecolor='white', linewidth=2)  # white border around slices
    )

    # Add white center circle
    centre_circle = plt.Circle((0, 0), 0.70, fc='white')
    ax.add_artist(centre_circle)

    ax.set_title("Algorithm Usage Distribution")
    ax.axis('equal')
    plt.tight_layout()
    plt.show()
