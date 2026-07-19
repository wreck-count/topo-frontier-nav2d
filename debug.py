import numpy as np
import matplotlib.pyplot as plt
import shapely

data = {'vertices': np.array([[ 3.53553391e+00, -1.35355339e+01],
       [-2.66453526e-15, -1.20710678e+01],
       [-1.46446609e+00, -8.53553391e+00],
       [-1.33226763e-15, -5.00000000e+00],
       [-1.12537636e-15, -5.00000000e+00],
       [-9.18485099e-16, -5.00000000e+00],
       [ 2.50000000e+00, -3.96446609e+00],
       [ 3.96446609e+00, -4.57106781e+00],
       [ 4.57106781e+00, -6.03553391e+00],
       [ 3.53553391e+00, -8.53553391e+00],
       [ 5.00000000e+00, -1.20710678e+01],
       [ 6.03553391e+00, -1.25000000e+01]]),
       'segments': np.array([[ 0,  1],
       [ 1,  2],
       [ 2,  3],
       [ 3,  4],
       [ 4,  5],
       [ 5,  6],
       [ 6,  7],
       [ 7,  8],
       [ 8,  9],
       [ 9, 10],
       [10, 11],
       [11,  0]])}

verts = data['vertices']
segs = data['segments']

new_poly = shapely.Polygon(verts)
new_poly = new_poly.simplify(.001)
assert isinstance(new_poly, shapely.Polygon)
print(new_poly.exterior)

fig, ax = plt.subplots(figsize=(8, 8))

for a, b in segs:
    p1, p2 = verts[a], verts[b]
    ax.plot([p1[0], p2[0]], [p1[1], p2[1]], 'b-', linewidth=1.5, zorder=1)

ax.scatter(verts[:, 0], verts[:, 1], c='red', zorder=2)
for i, (x, y) in enumerate(verts):
    ax.annotate(str(i), (x, y), textcoords="offset points", xytext=(6, 6), fontsize=10, color='black')

ax.set_aspect('equal')
ax.set_title('Boundary polygon passed to triangulate()\n(vertices 3,4,5 are ~duplicate points)')
ax.grid(True, linestyle='--', alpha=0.4)
plt.show()
# out_path = r'C:\Users\gsath\AppData\Local\Temp\claude\c--Users-gsath-OneDrive-Documents-personal-TopoNavSLAM-2D\c9f699a1-9780-44b7-a37b-49b2dbbc4b60\scratchpad\debug_plot.png'
plt.savefig(out_path, dpi=150)
# print('saved to', out_path)
