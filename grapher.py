import sys
from matplotlib import pyplot as plt

graph_names = ["Path", "Linear Velocity", "Angular Velocity"]
graph_axes_name_x = ["X Position (Inches)", "Distance Along Path (Inches)", "Distance Along Path (Inches)"]
graph_axes_name_y = ["Y Position (Inches)", "Velocity (Inches/Second)", "Velocity (Radians/Second)"]

graph_names_amnt = len(graph_names)

xs: list[list[float]] = [[] for _ in range(graph_names_amnt)]
ys: list[list[float]] = [[] for _ in range(graph_names_amnt)]

counter = 0

for line in sys.stdin:
    point = line.strip("\n").split(" ")
    xs[counter % graph_names_amnt].append(float(point[0]))
    ys[counter % graph_names_amnt].append(float(point[1]))
    counter += 1

fig, axs = plt.subplots(1, graph_names_amnt, figsize=(10, 4))

for i in range(graph_names_amnt):
    axs[i].plot(xs[i], ys[i])
    axs[i].set_title(graph_names[i])
    axs[i].set_xlabel(graph_axes_name_x[i])
    axs[i].set_ylabel(graph_axes_name_y[i])
    axs[i].figure.set_figwidth(15)

plt.show()