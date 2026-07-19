"""Visualize/debug an arbitrary WKT geometry (e.g. pasted from a shapely repr or error message).

Breaks a GEOMETRYCOLLECTION (or any geometry) into its primitive parts with
shapely.get_parts, then plots each part in its own color with vertex indices
annotated, and prints a per-part summary (type, coords, length/area).
"""

from typing import Iterator

import shapely
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon
from shapely.geometry.base import BaseGeometry

# WKT = (
#     "GEOMETRYCOLLECTION ("
#     "POLYGON ((3.535533905932737 60, 0 60, 0 63.63961030678928, "
#     "1.7157287525380966 62.928932188134524, 2.751262658470834 60.428932188134524, "
#     "3.7867965644035713 60, 3.535533905932737 60)), "
#     "LINESTRING (-1.8853654595047544e-15 64.28932188134524, "
#     "-1.69277603961185e-15 64.28932188134524))"
# )

WKT = ("POLYGON ((61.64213562373095 61.03553390593274, 65.17766952966369 59.571067811865476, 66.64213562373095 56.03553390593274, 65.17766952966369 52.5, 62.67766952966369 51.46446609406726, 63.10660171779821 52.5, 61.64213562373095 56.03553390593274, 62.071067811865476 57.071067811865476, 61.371968582528396 58.75884265277564, 60.60660171779821 60.60660171779821, 60 60, 60 60.35533905932738, 61.64213562373095 61.03553390593274))")


def flatten_parts(geom: BaseGeometry) -> Iterator[BaseGeometry]:
    """Recursively break a geometry down to its primitive parts (Point/LineString/Polygon/...).

    shapely.get_parts only descends one level, so a GEOMETRYCOLLECTION containing a
    MULTIPOLYGON would otherwise yield a MultiPolygon instead of individual Polygons.
    """
    if hasattr(geom, "geoms"):
        for sub in shapely.get_parts(geom):
            yield from flatten_parts(sub)
    else:
        yield geom


def plot_part(ax, geom, color, label):
    gtype = geom.geom_type

    if gtype == "Polygon":
        exterior = list(geom.exterior.coords)
        xs, ys = zip(*exterior)
        patch = MplPolygon(exterior, closed=True, facecolor=color, edgecolor=color,
                            alpha=0.25, linewidth=2, zorder=1, label=label)
        ax.add_patch(patch)
        ax.plot(xs, ys, "o", color=color, zorder=3, markersize=5)
        for i, (x, y) in enumerate(exterior):
            ax.annotate(str(i), (x, y), textcoords="offset points", xytext=(6, 6),
                        fontsize=8, color=color)
        for ring in geom.interiors:
            ixs, iys = zip(*ring.coords)
            ax.plot(ixs, iys, "--", color=color, linewidth=1.5, zorder=2)

    elif gtype == "LineString":
        coords = list(geom.coords)
        xs, ys = zip(*coords)
        ax.plot(xs, ys, "-o", color=color, linewidth=2, zorder=3, markersize=6, label=label)
        for i, (x, y) in enumerate(coords):
            ax.annotate(str(i), (x, y), textcoords="offset points", xytext=(6, -10),
                        fontsize=8, color=color)

    elif gtype == "Point":
        x, y = geom.x, geom.y
        ax.plot(x, y, "*", color=color, markersize=14, zorder=3, label=label)

    else:
        raise ValueError(f"unexpected non-primitive geometry after flattening: {gtype}")


def visualize_wkt(wkt_str, title=None):
    geom = shapely.from_wkt(wkt_str)
    parts = list(flatten_parts(geom))

    colors = plt.cm.tab10.colors
    fig, ax = plt.subplots(figsize=(8, 8))

    for i, part in enumerate(parts):
        color = colors[i % len(colors)]
        label = f"[{i}] {part.geom_type}"
        print(f"--- part {i}: {part.geom_type} ---")
        print(f"  wkt:    {part.wkt}")
        print(f"  bounds: {part.bounds}")
        if part.geom_type == "Polygon":
            print(f"  area:   {part.area}")
            print(f"  valid:  {part.is_valid}")
        elif part.geom_type == "LineString":
            print(f"  length: {part.length}")
        plot_part(ax, part, color, label)

    ax.set_aspect("equal")
    ax.set_title(title or "GEOMETRYCOLLECTION parts (get_parts)")
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend(loc="best", fontsize=8)
    plt.show()


if __name__ == "__main__":
    visualize_wkt(WKT)
