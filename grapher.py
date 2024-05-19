import sys
from matplotlib import pyplot as plt

graph_names = ["Path", "Linear Velocity", "Angular Velocity"]
graph_names_amnt = len(graph_names)

xs = [[] for _ in range(graph_names_amnt)]
ys = [[] for _ in range(graph_names_amnt)]

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

plt.show()