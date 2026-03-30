import matplotlib.pyplot as plt
import csv


def load_csv(filename):
    x, y, ids = [], [], []
    try:
        with open(filename, "r") as file:
            reader = csv.DictReader(file)
            for row in reader:
                x.append(float(row["x"]))
                y.append(float(row["y"]))
                if "id" in row:
                    ids.append(row["id"])
    except FileNotFoundError:
        print(f"Warning: Could not find {filename}")
    return x, y, ids


# Load all three datasets
gt_x, gt_y, gt_ids = load_csv("build/ground_truth.csv")
traj_x, traj_y, _ = load_csv("build/trajectory.csv")
slam_x, slam_y, slam_ids = load_csv("build/map.csv")

plt.figure(figsize=(12, 9))

# 1. Plot Ground Truth (The Target)
if gt_x:
    plt.scatter(
        gt_x,
        gt_y,
        facecolors="none",
        edgecolors="black",
        marker="o",
        s=200,
        linewidth=1.5,
        label="GT Cones",
    )

# 2. Plot Odometry Trajectory (The Drifting Car)
if traj_x:
    plt.plot(
        traj_x,
        traj_y,
        color="red",
        linestyle="--",
        linewidth=1.5,
        alpha=0.7,
        label="Odometry Trajectory",
    )

# 3. Plot SLAM Cones
if slam_x:
    plt.scatter(slam_x, slam_y, c="blue", marker="^", s=80, label="SLAM Mapped Cones")
    for i, txt in enumerate(slam_ids):
        plt.annotate(
            txt,
            (slam_x[i] + 0.3, slam_y[i] + 0.3),
            fontsize=9,
            color="blue",
            fontweight="bold",
        )

plt.title("SLAM case study", fontsize=14)
plt.xlabel("X (meters)")
plt.ylabel("Y (meters)")
plt.grid(True)
plt.axis("equal")
plt.legend()

plt.savefig("map_visualization.png", dpi=150)
print("Saved map_visualization.png! Click it to view your results.")
