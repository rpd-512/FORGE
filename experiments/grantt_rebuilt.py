import csv
import matplotlib.pyplot as plt

def plot_scores(csv_file):
    runs = []
    scores = []
    rebuilt_markers = []

    # Read CSV ignoring lines starting with '#'
    with open(csv_file, newline='') as f:
        reader = csv.DictReader((row for row in f if not row.startswith('#')))
        prev_rebuilt = False
        for row in reader:
            run = int(row["run"])
            score = float(row[" score"])
            built = row[" built"].strip()

            runs.append(run)
            scores.append(score)

            if built == "Rebuilt" and not prev_rebuilt:
                rebuilt_markers.append((run, score))
                prev_rebuilt = True
            else:
                prev_rebuilt = (built == "Rebuilt")

    # Plot scores
    plt.figure(figsize=(10, 5))
    plt.plot(runs, scores, label="Score", linestyle='-')

    # Plot rebuilt markers
    if rebuilt_markers:
        rebuilt_runs, rebuilt_scores = zip(*rebuilt_markers)
        plt.scatter(rebuilt_runs, rebuilt_scores, color='red', marker='x', s=100, label="Rebuilt")

    plt.xlabel("Run")
    plt.ylabel("Score")
    plt.title("Score vs Run with Rebuilt Markers")
    plt.legend()
    plt.grid(True)
    plt.savefig(csv_file.split("_score.csv")[0] + "_score.png")

# Example usage
if __name__ == "__main__":
    plot_scores("regular_rebuild_score.csv")
    plot_scores("no_rebuild_score.csv")
    plot_scores("balance_rebuild_score.csv")
