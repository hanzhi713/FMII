from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import numpy as np
import io

def parse_from_input():
    a = ""
    arr = []
    while True:
        a = input()
        if a == "next":
            break
        b = input()
        arr.append((
            [float(x) for x in a.split(",")],
            [float(x) for x in b.split(",")]
        ))
    return arr

# lines = [[(0, 1), (1, 1)], [(2, 3), (3, 3)], [(1, 2), (1, 3)]]
lines3d = np.array([[(0, 0, 0), (2, 3, 4)]])

fig = plt.figure(figsize=(14, 8))
ax1 = fig.add_subplot(1, 2, 1)
ax1.add_collection(LineCollection(parse_from_input(), colors="blue"))
ax1.add_collection(LineCollection(parse_from_input(), colors="red"))
ax1.add_collection(LineCollection(parse_from_input(), colors="green"))
ax1.autoscale()
ax1.legend(["image", "model", "image on model"])

ax2 = fig.add_subplot(1, 2, 2, projection="3d")
r1 = parse_from_input()
r2 = parse_from_input()
r3 = parse_from_input()
ax2.add_collection(Line3DCollection(r1, colors="blue"))
ax2.add_collection(Line3DCollection(r2, colors="red"))
ax2.add_collection(Line3DCollection(r3, colors="green"))

lines3d = r1 + r2 + r3
lims = np.array([
    np.min(lines3d, axis=(0, 1)),
    np.max(lines3d, axis=(0, 1))
])
ax2.set_xlim(lims[:, 0])
ax2.set_ylim(lims[:, 1])
ax2.set_zlim(lims[:, 2])
ax2.legend(["image", "model", "image on model"])
plt.show()