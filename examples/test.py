import csv
rows = []

num_joints =18
fieldnames = ["timestamp", "px", "py", "pz", "qx", "qy", "qz", "qw"] + \
             [f"joint_{j}" for j in range(num_joints)] + \
             ["extra_0", "extra_1", "extra_2", "extra_3"]

with open("trajectory.csv", "w", newline="") as csvfile:
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    writer.writeheader()
