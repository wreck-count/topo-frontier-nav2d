import random as rnd
import math
import time
import json
import os
from typing import cast, Protocol, Any, Callable
import triangle as tr
from shapely import Polygon, MultiPolygon, MultiLineString, Point, LineString, remove_repeated_points, ops as shape_ops, get_parts
from shapely.geometry.base import BaseGeometry
import networkx as nx
import matplotlib.pyplot as plt
from matplotlib import lines
from matplotlib import animation
from matplotlib.artist import Artist
from matplotlib.collections import LineCollection, PathCollection
from matplotlib.patches import Patch, PathPatch
from matplotlib.path import Path
import numpy as np
import heapq
from grid_index import GridIndex

type TopoId = str
type SearchElement = tuple[TopoId, TopoId]

EPS: float = 1e-3
INF: float = 1e3

def flatten_parts(geometry: BaseGeometry) -> list[BaseGeometry]:
    parts = get_parts(geometry)
    result: list[BaseGeometry] = []
    for p in parts:
        if p.geom_type.startswith('Multi') or p.geom_type == 'GeometryCollection':
            result.extend(flatten_parts(p))
        else:
            result.append(p)
    return result


def _polygon_patch(poly: Polygon, **kwargs: Any) -> PathPatch:
    vertices: list[tuple[float, float]] = []
    codes: list[int] = []
    for ring in [poly.exterior, *poly.interiors]:
        coords = list(ring.coords)
        vertices.extend(coords)
        codes.append(Path.MOVETO)
        codes.extend([Path.LINETO] * (len(coords) - 2))
        codes.append(Path.CLOSEPOLY)
    path = Path(vertices, codes)
    return PathPatch(path, **kwargs)


class Vec2i:
    def __init__(self, x: int, y: int) -> None:
        self.x = x
        self.y = y
    def __sub__(self, other: 'Vec2i') -> 'Vec2i':
        return Vec2i(self.x - other.x, self.y - other.y)
    def __add__(self, other: 'Vec2i') -> 'Vec2i':
        return Vec2i(self.x + other.x, self.y + other.y)

class IdGen:
    __char_choices = '-0123456789#$abcdefghijklmnopqrstuvwxyz'
    @staticmethod
    def gen_random_str(n: int):
        rnd_str = ''.join([IdGen.__char_choices[rnd.randint(0, len(IdGen.__char_choices) - 1)] for _ in range(n)])
        return rnd_str

class Vertex:
    def __init__(self, pos: Point):
        self.pos = Point(pos)
        self.id = IdGen.gen_random_str(5)

class Edge:
    def __init__(self, u: Vertex | TopoId, v: Vertex | TopoId):
        def to_id(x: Vertex | TopoId):
            if isinstance(x, Vertex):
                return x.id
            return x
        self.u = to_id(u)
        self.v = to_id(v)
        self.id = IdGen.gen_random_str(7)
        self.face_count = 0

class Mesh:
    def __init__(self):
        self.position = Point(0, 0)
        self.graph: nx.Graph[TopoId] = nx.Graph()
        self.boundary: set[TopoId] = set()
        self.edge_map: dict[TopoId, Edge] = dict()
        self.e_index: GridIndex[TopoId] = GridIndex()
        self.v_index: GridIndex[TopoId] = GridIndex()

    @property
    def boundary_verts(self):
        b_verts: set[TopoId] = set()
        for be in self.boundary:
            beo = self.get_edge(be)
            b_verts.add(beo.u)
            b_verts.add(beo.v)
        return b_verts

    @property
    def polygon(self):
        if len(self.boundary) == 0:
            return Polygon()
        visited: set[SearchElement] = set()
        def _turn_walk(el: SearchElement) -> list[SearchElement]:
            _el = el
            elem_walk: list[SearchElement] = []
            for _ in range(len(self.boundary) + 1):
                elem_walk.append(_el)
                visited.add(_el)
                u = _el[0]
                e = _el[1]
                eo = self.get_edge(e)
                v = eo.v if u == eo.u else eo.u
                nbs = self.neighbors(u)
                uo = self.get_vert(u)
                vo = self.get_vert(v)
                dir = Point(vo.pos.x - uo.pos.x, vo.pos.y - uo.pos.y)
                ang = math.atan2(dir.x, dir.y)
                if ang < 0:
                    ang += 2 * math.pi
                closest_turn: tuple[SearchElement, float] | None = None
                closest_edge: Edge | None = None
                for nb in nbs:
                    if nb == v:
                        continue
                    teo = self.get_edge(u, nb)
                    tnbo = self.get_vert(nb)
                    tdir = Point(tnbo.pos.x - uo.pos.x, tnbo.pos.y - uo.pos.y)
                    tang = math.atan2(tdir.x, tdir.y)
                    if tang < 0:
                        tang += 2 * math.pi
                    ang_dif = ang - tang
                    if ang_dif < 0:
                        ang_dif += 2 * math.pi
                    if closest_turn is None or closest_turn[1] > ang_dif:
                        closest_turn = ((nb, teo.id), ang_dif)
                        closest_edge = teo
                if closest_turn is None or closest_edge is None or closest_edge.face_count != 1:
                    return []
                _el = closest_turn[0]
                if _el == el:
                    break
            return elem_walk

        walks: list[list[SearchElement]] = []
        for cur_edge in self.boundary:
            eo = self.get_edge(cur_edge)
            el_a = (eo.u, eo.id)
            el_b = (eo.v, eo.id)
            if not el_a in visited:
                w = _turn_walk(el_a)
                if w:
                    walks.append(w)
            if not el_b in visited:
                w = _turn_walk(el_b)
                if w:
                    walks.append(w)

        ext_ring: list[Point] | None = None
        int_rings: list[list[Point]] = []
        for walk in walks:
            ring_pts = [self.get_vert(el[0]).pos for el in walk]
            n = len(ring_pts)
            area = 0
            for i in range(n):
                pt = ring_pts[i]
                npt = ring_pts[(i + 1) % n]
                area += (npt.x - pt.x) * (npt.y + pt.y) / 2.0
            if area < 0:
                ext_ring = ring_pts
            else:
                int_rings.append(ring_pts)
        assert ext_ring is not None
        poly = Polygon(ext_ring, int_rings)
        assert poly.is_valid
        return poly

    @property
    def num_verts(self) -> int:
        return self.graph.number_of_nodes()
    
    @property
    def num_edges(self) -> int:
        return self.graph.number_of_edges()
    
    def verts(self) -> list[Vertex]:
        return [vertData["obj"] for _, vertData in self.graph.nodes(data=True)]

    def edges(self) -> list[Edge]:
        return [edgeData["obj"] for _, _, edgeData in self.graph.edges(data=True)]

    def has_vert(self, u: Point | TopoId):
        if isinstance(u, Point):
            return len([v for v in self.verts() if v.pos.equals(u) > 0]) > 0
        return self.graph.has_node(u)

    def find_vert(self, u: Point | TopoId) -> Vertex | None:
        if isinstance(u, Point):
            nearby_verts = [self.graph.nodes[id]['obj'] for id in self.v_index.query(u.bounds)]
            matches = [v for v in nearby_verts if v.pos.equals_exact(u, EPS)]
            return matches[0] if matches else None
        return self.graph.nodes[u]['obj'] if self.graph.has_node(u) else None

    def get_vert(self, u: Point | TopoId) -> Vertex:
        vert = self.find_vert(u)
        if vert is None:
            raise KeyError(f'no vertex for {u!r}')
        return vert

    def add_vert(self, pos: Point):
        vert = self.find_vert(pos)
        if vert is None:
            new_vert = Vertex(pos)
            self.v_index.insert(new_vert.id, pos.bounds)
            self.graph.add_node(new_vert.id, obj = new_vert)
            return new_vert
        return vert

    def remove_vert(self, u: Point | TopoId) -> bool:
        mu = self.find_vert(u)
        if mu is None:
            return False
        self.graph.remove_node(mu.id)
        self.v_index.remove(mu.id)
        return True

    def has_edge(self, u: Point | TopoId, v: Point | TopoId):
        return self.find_edge(u, v) is not None

    def add_edge(self, u: Point | TopoId, v: Point | TopoId):
        mu = self.find_vert(u)
        mv = self.find_vert(v)
        if mu is None or mv is None:
            raise LookupError('vertices not found in mesh, edge not added')
        e = self.find_edge(mu.id, mv.id)
        if e is None:
            new_edge = Edge(mu, mv) 
            self.e_index.insert(new_edge.id, LineString([mu.pos, mv.pos]).bounds)
            self.edge_map[new_edge.id] = new_edge
            self.graph.add_edge(mu.id, mv.id, obj = new_edge)
            return new_edge
        return e

    def find_edge(self, u: Point | TopoId, v: Point | TopoId | None = None) -> Edge | None:
        if v is None:
            assert isinstance(u, str)
            return self.edge_map.get(u)
        mu = self.find_vert(u)
        mv = self.find_vert(v)
        if mu is None or mv is None:
            return None
        return self.graph[mu.id][mv.id]['obj'] if self.graph.has_edge(mu.id, mv.id) else None

    def get_edge(self, u: Point | TopoId, v: Point | TopoId | None = None) -> Edge:
        edge = self.find_edge(u, v)
        if edge is None:
            raise KeyError(f'no edge for {u!r}, {v!r}')
        return edge

    def remove_edge(self, u: Point | TopoId, v: Point | TopoId | None = None) -> bool:
        me = self.find_edge(u, v)
        if me is None:
            return False
        self.graph.remove_edge(me.u, me.v)
        self.e_index.remove(me.id)
        self.edge_map.pop(me.id)
        return True

    def neighbors(self, u: Point | TopoId):
        mu = self.find_vert(u)
        if mu is not None:
            return [id for id in self.graph.neighbors(mu.id)]
        assert isinstance(u, str)
        me = self.get_edge(u)
        return [self.get_edge(me.u, v).id for v in self.graph.neighbors(me.u)] + [self.get_edge(me.v, v).id for v in self.graph.neighbors(me.v)]

    def find_edges(self, pos: Point):
        def _in_edge(e: Edge):
            mu = self.get_vert(e.u)
            mv = self.get_vert(e.v)
            line = LineString([mu.pos, mv.pos])
            return line.distance(pos) < EPS
        nearby_edges = [self.edge_map[id] for id in self.e_index.query(pos.bounds)]
        return [e for e in nearby_edges if _in_edge(e)]

    def get_closest_vert(self, pos: Point) -> Vertex | None:
        closest_dist = math.inf
        closest_v = None
        for v in self.verts():
            dist = v.pos.distance(pos)
            if dist < closest_dist:
                closest_v = v
                closest_dist = dist
        return closest_v

class Interactable(Protocol):
    def interact_scan(self, scan_polygon: Polygon, ray_start: tuple[float, float]) -> tuple[BaseGeometry, Polygon]:
        ...

class Robot:
    def __init__(self, radius: float, scan_resolution: int = 8):
        self.scan_radius = radius
        self.position: Point = Point(0, 0)
        self.scan_resolution = scan_resolution
        self.mapping = Mesh()
        self.visited: set[TopoId] = set()
        self.obstacle_verts: set[TopoId] = set()
        self.follow_path: tuple[int, list[TopoId]] = (0, [])
        self.last_scan_polys: list[Polygon] = []
        self.scan_log: list[tuple[float, float]] = []
        self.on_scan: Callable[[], None] | None = None
        self.unreachable_frontier_log: list[tuple[TopoId, list[TopoId]]] = []

    def _get_regular_polygon(self, radius: float, centre: Point, point_count: int):
        if point_count < 3:
            raise RuntimeError('points should atleast be 3')
        ang = 0.
        points: list[Point] = []
        for _ in range(point_count):
            p = Point(radius * math.cos(ang) + centre.x, radius * math.sin(ang) + centre.y)
            points.append(p)
            ang += 2 * math.pi / point_count
        return Polygon(points)

    def _constrained_triangulate(self, poly: Polygon, interior_points: list[Point]):
        print('triangulation in')
        bound_pts = np.transpose(poly.exterior.xy)[:-1]
        n = len(bound_pts)
        interior_arr = np.array([(p.x, p.y) for p in interior_points]).reshape(-1, 2)
        verts = np.vstack([bound_pts, interior_arr])
        segments = [(i, (i + 1)% n) for i in range(n)]
        data = {'vertices': verts, 'segments': np.array(segments)}
        result = cast(dict[str, np.ndarray], tr.triangulate(data, 'pqa3.0S5e'))
        verts_out: np.ndarray = result['vertices']
        edges: np.ndarray = result['edges']
        triangles: np.ndarray = result['triangles']
        print('triangulation out')
        return (verts_out, edges, triangles)

    def _map_new_region(self, new_poly: Polygon, obstacle_boundary: BaseGeometry):
        new_poly = remove_repeated_points(new_poly, EPS)
        if not new_poly.is_valid:
            print("Supplied new region is not valid!")
            return False

        # add robot position to set of interior points if it lies inside
        interior_points: list[Point] = []
        if self.position.contains(self.position) and self.position.distance(new_poly.exterior) > EPS:
            interior_points.append(self.position)
            print('interior point in')

        tverts = None
        tedges = None
        ttris = None
        try:
            tverts, tedges, ttris = self._constrained_triangulate(new_poly, interior_points)
        except:
            print('something bad happened during triangulation')
            return
        scan_verts: list[TopoId] = []
        scan_edges: list[TopoId] = []
        for x, y in tverts:
            npoint = Point(x, y)
            redges = self.mapping.find_edges(npoint)
            new_vert = self.mapping.add_vert(npoint)
            scan_verts.append(new_vert.id)
            if len(redges) != 1:
                continue
            old_edge_id = redges[0].id
            mu = redges[0].u
            mv = redges[0].v
            mu_neighbors = set(self.mapping.neighbors(mu))
            mv_neighbors = set(self.mapping.neighbors(mv))
            common_neighbors = [id for id in mu_neighbors.intersection(mv_neighbors)]
            if len(common_neighbors) > 1:
                print('debug place in')
            nearest_v = None
            nearest_dist = math.inf
            for cv in common_neighbors:
                cvert = self.mapping.get_vert(cv)
                dist = cvert.pos.distance(npoint)
                if dist < nearest_dist:
                    nearest_dist = dist
                    nearest_v = cv

            assert nearest_v is not None
            connects = [mu, mv, nearest_v]

            self.mapping.remove_edge(old_edge_id)
            self.mapping.boundary.discard(old_edge_id)
            for v in connects:
                edge = self.mapping.add_edge(new_vert.id, v)
                if v == nearest_v:
                    edge.face_count = 2
                else:
                    edge.face_count = 1
                scan_edges.append(edge.id)

        # add other edges
        for ti, tj in tedges:
            edge = self.mapping.add_edge(scan_verts[ti], scan_verts[tj])
            scan_edges.append(edge.id)

        for tri in ttris: 
            id_pairs: list[tuple[TopoId, TopoId]] = [(scan_verts[tri[i]], scan_verts[tri[(i + 1) % 3]]) for i in range(3)]
            for pr in id_pairs:
                redge = self.mapping.get_edge(pr[0], pr[1])
                redge.face_count += 1
        
        for e in scan_edges:
            eo = self.mapping.find_edge(e)
            if eo is None:
                continue
            if eo.face_count == 1:
                self.mapping.boundary.add(e)
            else:
                self.mapping.boundary.discard(e)

        for scan_vert in scan_verts:
            vo = self.mapping.get_vert(scan_vert)
            if obstacle_boundary.distance(vo.pos) < 10 * EPS:
                self.obstacle_verts.add(vo.id)

        print('boundary out')

    def perform_scan(self, world: Interactable):
        self.scan_log.append((self.position.x, self.position.y))
        if self.on_scan is not None:
            self.on_scan()
        actual_scan_polygon = self._get_regular_polygon(self.scan_radius, self.position, self.scan_resolution)
        scan_interaction = world.interact_scan(actual_scan_polygon, (self.position.x, self.position.y))
        polygon_map = self.mapping.polygon
        new_region_result = Polygon.difference(scan_interaction[1], polygon_map)
        new_regions = [p for p in flatten_parts(new_region_result) if isinstance(p, Polygon)]
        print('new regions')
        for new_poly in new_regions:
            print(new_poly)
        self.last_scan_polys = list(new_regions)
        for new_poly in new_regions:
            self._map_new_region(new_poly, scan_interaction[0])
       
    def __a_star(self, src: TopoId, des: TopoId):
        path: list[TopoId] = []
        if src == des:
            return [src]
        def _heuristic(v: TopoId):
            return self.mapping.get_vert(v).pos.distance(self.mapping.get_vert(des).pos)
        
        dist: dict[TopoId, float] = dict()
        hdist: dict[TopoId, float] = dict()
        parent: dict[TopoId, TopoId] = dict()
        pq: list[tuple[float, TopoId]] = []
        
        heapq.heappush(pq, (_heuristic(src), src))
        dist[src] = 0
        hdist[src] = _heuristic(src)
        parent[src] = src

        while pq:
            h, v = heapq.heappop(pq)
            if v == des:
                while True:
                    path.append(v)
                    if v == parent[v]:
                        break
                    v = parent[v]
                path.reverse()
                return path
            if v in hdist and hdist[v] < h:
                continue
            for nb in self.mapping.neighbors(v):
                edge_cost = 1 + (INF if nb in self.obstacle_verts else 0)
                if nb not in dist or dist[nb] > dist[v] + edge_cost:
                    parent[nb] = v
                    dist[nb] = dist[v] + edge_cost
                    hdist[nb] = dist[nb] + _heuristic(nb)
                    heapq.heappush(pq, (hdist[nb], nb))

        return path
        
    def process(self, world: Interactable):
        if len(self.mapping.boundary_verts) == 0:
            self.perform_scan(world)
            return
        cur_vert = self.mapping.find_vert(self.position)
        if cur_vert is None:
            return
        if cur_vert.id in self.mapping.boundary_verts:
            self.perform_scan(world)
            return
        path_idx, path = self.follow_path
        if path_idx + 1 >= len(path):
            frontier = self.mapping.boundary_verts.difference(self.obstacle_verts)
            unreachable: list[TopoId] = []
            for dest_id in frontier:
                path_to_nxt = self.__a_star(cur_vert.id, dest_id)
                if path_to_nxt:
                    if unreachable:
                        print(f'WARNING: frontier vertices unreachable from {cur_vert.id}, skipped: {unreachable}')
                        self.unreachable_frontier_log.append((cur_vert.id, list(unreachable)))
                    self.follow_path = (0, path_to_nxt)
                    return
                unreachable.append(dest_id)
            if unreachable:
                print(f'WARNING: ALL frontier vertices unreachable from {cur_vert.id}: {unreachable}')
                self.unreachable_frontier_log.append((cur_vert.id, list(unreachable)))
            return
        path_idx += 1
        self.follow_path = (path_idx, path)
        next_vert_id = path[path_idx]
        self.position = self.mapping.get_vert(next_vert_id).pos


class Maze:
    def __init__(self, bounds: tuple[Vec2i, Vec2i], zoom: float = 15, walls: list[LineString] | None = None) -> None:
        self.bounds = bounds
        self.zoom = zoom
        if walls is not None:
            self.walls = walls
        else:
            self.walls = []
            self.__gen_maze(bounds)

    def world_bounds(self) -> tuple[float, float, float, float]:
        b0, b1 = self.bounds
        return (b0.x * self.zoom, b0.y * self.zoom, b1.x * self.zoom, b1.y * self.zoom)

    def world_center(self) -> Point:
        minx, miny, maxx, maxy = self.world_bounds()
        return Point((minx + maxx) / 2, (miny + maxy) / 2)

    def to_dict(self) -> dict[str, Any]:
        b0, b1 = self.bounds
        return {
            'bounds': [[b0.x, b0.y], [b1.x, b1.y]],
            'zoom': self.zoom,
            'walls': [list(w.coords) for w in self.walls],
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> 'Maze':
        (x0, y0), (x1, y1) = data['bounds']
        walls = [LineString(coords) for coords in data['walls']]
        return cls((Vec2i(x0, y0), Vec2i(x1, y1)), data['zoom'], walls=walls)

    def _wall(self, a: Vec2i, b: Vec2i) -> LineString:
        return LineString([[a.x * self.zoom, a.y * self.zoom], [b.x * self.zoom, b.y * self.zoom]])

    def _interacting_clip_lines(self, wall: LineString, ray_start: tuple[float, float]):
        clines: list[LineString] = [wall]
        for bpoint in wall.boundary.geoms:
            dir = (bpoint.x - ray_start[0], bpoint.y - ray_start[1])
            inf_point = Point(1000 * dir[0] + bpoint.x, 1000 * dir[1] + bpoint.y)
            inf_line = LineString([bpoint, inf_point])
            clines.append(inf_line)
        clip_lines = shape_ops.linemerge(clines)
        assert isinstance(clip_lines, LineString)
        return clip_lines
    
    def _construct_infinite_poly(self, wall: LineString, ray_start: tuple[float, float]):
        clip_lines = self._interacting_clip_lines(wall, ray_start)
        assert isinstance(clip_lines, LineString)
        return Polygon(clip_lines)
        
    def interact_scan(self, scan_polygon: Polygon, ray_start: tuple[float, float]) -> tuple[BaseGeometry, Polygon]:
        interacting_walls: list[LineString] = [wall for wall in self.walls if scan_polygon.intersects(wall)]
        if len(interacting_walls) == 0:
            return (LineString(), scan_polygon)
        merge_result = shape_ops.linemerge(interacting_walls)
        interaction_boundary = scan_polygon.intersection(merge_result)
        flat = flatten_parts(interaction_boundary)
        fwalls = [geom for geom in flat if isinstance(geom, LineString)]
        scan_region = scan_polygon
        for wall in fwalls:
            split_lines = self._interacting_clip_lines(wall, ray_start)
            split_result = shape_ops.split(scan_region, split_lines)
            print('Number of splits:', len(split_result.geoms))
            for split_geom in split_result.geoms:
                if isinstance(split_geom, Polygon):
                    if split_geom.contains(Point(ray_start)):
                        scan_region = split_geom
                        break
                else:
                    print('FOUND NON-POLYGON SPLIT', split_geom)
        assert scan_region.is_valid
        return (interaction_boundary, scan_region)

    def __gen_maze(self, bounds: tuple[Vec2i, Vec2i]):
        q: list[tuple[Vec2i, int]] = []
        # populate boundary values
        steps = bounds[1] - bounds[0]
        vis = [[False] * (steps.y + 1) for _ in range(steps.x + 1)]
        def add_to_vis(coord: Vec2i):
            vis_coord = coord - bounds[0]
            vis[vis_coord.x][vis_coord.y] = True

        def is_vis(coord: Vec2i):
            vis_coord = coord - bounds[0]
            if (vis_coord.x < 0 or vis_coord.x > steps.x):
                return True
            if (vis_coord.y < 0 or vis_coord.y > steps.y):
                return True
            return vis[vis_coord.x][vis_coord.y]

        prev_top: Vec2i | None = None
        prev_bottom: Vec2i | None = None
        for i in range(steps.x + 1):
            curx = bounds[0].x + i
            c1 = Vec2i(curx, bounds[1].y)
            c2 = Vec2i(curx, bounds[0].y)
            if prev_top is not None and prev_bottom is not None:
                self.walls.append(self._wall(prev_top, c1))
                self.walls.append(self._wall(prev_bottom, c2))
            prev_top, prev_bottom = c1, c2
            if not is_vis(c1):
                q.append((c1, 0))
                add_to_vis(c1)
            if not is_vis(c2):
                q.append((c2, 0))
                add_to_vis(c2)

        prev_left: Vec2i | None = None
        prev_right: Vec2i | None = None
        for i in range(steps.y + 1):
            cury = bounds[0].y + i
            c1 = Vec2i(bounds[0].x, cury)
            c2 = Vec2i(bounds[1].x, cury)
            if prev_left is not None and prev_right is not None:
                self.walls.append(self._wall(prev_left, c1))
                self.walls.append(self._wall(prev_right, c2))
            prev_left, prev_right = c1, c2
            if not is_vis(c1):
                q.append((c1, 0))
                add_to_vis(c1)
            if not is_vis(c2):
                q.append((c2, 0))
                add_to_vis(c2)

        dx = [-1, 0, 1, 0]
        dy = [0, 1, 0, -1]
        while len(q):
            ridx = rnd.randint(0, len(q) - 1)
            cur_coord, depth = q[ridx]
            q.pop(ridx)
            new_coords = [c for c in [cur_coord + Vec2i(dx[j], dy[j]) for j in range(4)] if not is_vis(c)]
            prob = 1.
            p = rnd.random()
            while (len(new_coords) > 0 and p < prob):
                eidx = rnd.randint(0, len(new_coords) - 1)
                new_coord = new_coords[eidx]
                new_coords.pop(eidx)
                self.walls.append(self._wall(cur_coord, new_coord))
                q.append((new_coord, depth + 1))
                add_to_vis(new_coord)
                prob /= (depth + 1)
                p = rnd.random()

    def show(self):
        return


def save_recording(maze: Maze, robot: Robot, path: str) -> None:
    data: dict[str, Any] = {
        'maze': maze.to_dict(),
        'scan_radius': robot.scan_radius,
        'scan_resolution': robot.scan_resolution,
        'scan_log': [list(p) for p in robot.scan_log],
    }
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    with open(path, 'w') as f:
        json.dump(data, f, indent=2)


def load_recording(path: str) -> tuple[Maze, Robot]:
    with open(path, 'r') as f:
        data = json.load(f)
    maze = Maze.from_dict(data['maze'])
    robot = Robot(data.get('scan_radius', 5.0), data.get('scan_resolution', 8))
    for x, y in data['scan_log']:
        robot.position = Point(x, y)
        robot.perform_scan(maze)
    return maze, robot


class Simulator:
    def __init__(self, robot_position: Point, obstacles: list[Interactable]) -> None:
        self.robot = Robot(5.0)
        self.obstacles = obstacles
        self.robot.position = robot_position
        self.paused = True
        self.fig, self.ax = plt.subplots()
        self.fig.subplots_adjust(right=0.78)
        self._blink_polys: list[Polygon] = []
        self._blink_frames_left: int = 0
        self._focus_polys: list[Polygon] = []
        self._blink_patches: list[PathPatch] = []
        self.fig.canvas.mpl_connect('button_press_event', self.on_pick)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        self.robot.on_scan = self._auto_save_recording
        self._init_artists()

    def _auto_save_recording(self) -> None:
        for obstacle in self.obstacles:
            if isinstance(obstacle, Maze):
                save_recording(obstacle, self.robot, 'tests/fixtures/_live_recovery.json')
                break

    def interact_scan(self, scan_polygon: Polygon, ray_start: tuple[float, float]) -> tuple[BaseGeometry, Polygon]:
        fscan_polygon = scan_polygon
        interacted_boundaries: list[BaseGeometry] = []
        for obstacle in self.obstacles:
            interaction_result = obstacle.interact_scan(fscan_polygon, ray_start)
            interacted_boundaries.append(interaction_result[0])
            fscan_polygon = interaction_result[1]
        interacted_boundary = shape_ops.unary_union(interacted_boundaries)
        return (interacted_boundary, fscan_polygon)

    def process(self):
        if self._blink_frames_left == 0 and not self.paused:
            self.robot.process(self)

    def run(self, tick_seconds: float = 0.5, max_ticks: int | None = None):
        tick = 0
        while max_ticks is None or tick < max_ticks:
            self.process()
            tick += 1
            time.sleep(tick_seconds)

    def _init_artists(self):
        """Build the scene once. The maze walls, the legend and the axes never
        change, and the mesh needs two collections rather than one Line2D per
        edge -- rebuilding ~1k artists per frame is what made big maps crawl."""
        wall_segments: list[list[tuple[float, ...]]] = []
        for obstacle in self.obstacles:
            if isinstance(obstacle, Maze):
                wall_segments.extend([list(wall.coords) for wall in obstacle.walls])
        self.ax.add_collection(LineCollection(wall_segments, colors='black', linewidths=1.5, zorder=1))

        self._edge_lines = LineCollection([], colors='tab:blue', linewidths=0.5, zorder=2)
        self._frontier_lines = LineCollection([], colors='tab:red', linewidths=0.5, zorder=3)
        self.ax.add_collection(self._edge_lines)
        self.ax.add_collection(self._frontier_lines)

        def _dots(size: float, color: str, zorder: int) -> PathCollection:
            return self.ax.scatter([], [], s=size, color=color, picker=True, zorder=zorder)

        self._vert_dots = _dots(10, 'tab:blue', 4)
        self._boundary_dots = _dots(20, 'tab:orange', 5)
        self._obstacle_dots = _dots(20, 'brown', 6)
        self._robot_dot = _dots(40, 'tab:red', 7)

        legend_handles = [
            lines.Line2D([0], [0], color='black', linewidth=1.5, label='Maze wall'),
            lines.Line2D([0], [0], color='tab:red', linewidth=0.5, label='Frontier edge'),
            lines.Line2D([0], [0], color='tab:blue', linewidth=0.5, label='Mesh edge'),
            lines.Line2D([0], [0], marker='o', color='w', markerfacecolor='tab:blue', markersize=6, label='Vertex'),
            lines.Line2D([0], [0], marker='o', color='w', markerfacecolor='tab:orange', markersize=6, label='Boundary vertex'),
            lines.Line2D([0], [0], marker='o', color='w', markerfacecolor='brown', markersize=6, label='Obstacle vertex'),
            lines.Line2D([0], [0], marker='o', color='w', markerfacecolor='tab:red', markersize=8, label='Robot'),
            Patch(facecolor='yellow', alpha=0.4, label='Latest scan'),
        ]
        self.ax.legend(handles=legend_handles, loc='upper left', bbox_to_anchor=(1.02, 1.0), fontsize='small', borderaxespad=0)
        self.ax.set_aspect('equal')
        view_bounds: list[float] | None = None
        for obstacle in self.obstacles:
            if isinstance(obstacle, Maze):
                wminx, wminy, wmaxx, wmaxy = obstacle.world_bounds()
                if view_bounds is None:
                    view_bounds = [wminx, wminy, wmaxx, wmaxy]
                else:
                    view_bounds[0] = min(view_bounds[0], wminx)
                    view_bounds[1] = min(view_bounds[1], wminy)
                    view_bounds[2] = max(view_bounds[2], wmaxx)
                    view_bounds[3] = max(view_bounds[3], wmaxy)
        if view_bounds is None:
            view_bounds = list(self.robot.position.bounds)
        minx, miny, maxx, maxy = view_bounds
        pad = 5
        self.ax.set_xlim(minx - pad, maxx + pad)
        self.ax.set_ylim(miny - pad, maxy + pad)

    def _dynamic_artists(self) -> list[Artist]:
        """Everything _draw touches. Returned to FuncAnimation so blitting can
        redraw just these over a cached background, skipping the legend and tick
        text relayout that otherwise dominates every frame."""
        return [self._edge_lines, self._frontier_lines, self._vert_dots,
                self._boundary_dots, self._obstacle_dots, self._robot_dot,
                *self._blink_patches]

    def _offsets(self, points: list[Point]) -> np.ndarray:
        if not points:
            return np.empty((0, 2))
        return np.array([[p.x, p.y] for p in points])

    def _draw(self) -> list[Artist]:
        for patch in self._blink_patches:
            patch.remove()
        self._blink_patches = []
        if self._blink_frames_left > 0 and self._blink_polys:
            if self._blink_frames_left % 2 == 0:
                for poly in self._blink_polys:
                    if poly.is_empty:
                        continue
                    patch = _polygon_patch(poly, facecolor='yellow', alpha=0.4, zorder=0)
                    self.ax.add_patch(patch)
                    self._blink_patches.append(patch)
            self._blink_frames_left -= 1

        mesh = self.robot.mapping
        edge_segments: list[tuple[tuple[float, float], tuple[float, float]]] = []
        frontier_segments: list[tuple[tuple[float, float], tuple[float, float]]] = []
        for e in mesh.edges():
            mu = mesh.get_vert(e.u)
            mv = mesh.get_vert(e.v)
            segment = ((mu.pos.x, mu.pos.y), (mv.pos.x, mv.pos.y))
            if e.id in mesh.boundary:
                frontier_segments.append(segment)
            else:
                edge_segments.append(segment)
        self._edge_lines.set_segments(edge_segments)
        self._frontier_lines.set_segments(frontier_segments)

        self._vert_dots.set_offsets(self._offsets([v.pos for v in mesh.verts()]))
        self._boundary_dots.set_offsets(self._offsets([mesh.get_vert(id).pos for id in mesh.boundary_verts]))
        self._obstacle_dots.set_offsets(self._offsets([mesh.get_vert(id).pos for id in self.robot.obstacle_verts]))
        self._robot_dot.set_offsets(self._offsets([self.robot.position]))
        return self._dynamic_artists()

    def animate(self, interval_ms: float = 500, max_ticks: int | None = None):
        self._draw()
        ticks = 0
        def update(_frame: int) -> list[Artist]:
            nonlocal ticks
            if max_ticks is not None and ticks >= max_ticks:
                return self._dynamic_artists()
            self.process()
            if self.robot.last_scan_polys:
                self._blink_polys = self.robot.last_scan_polys
                self._blink_frames_left = 6
                self._focus_polys = self.robot.last_scan_polys
                self.robot.last_scan_polys = []
            ticks += 1
            return self._draw()
        self._anim = animation.FuncAnimation(self.fig, update, interval=interval_ms, cache_frame_data=False, blit=True)
        plt.show()

    def on_pick(self, event):
        x: float = float(event.xdata)
        y: float = float(event.ydata)
        if x and y:
            v = self.robot.mapping.get_closest_vert(Point(x, y))
            if v is not None:
                self.robot.position = v.pos

    def on_key_press(self, event):
        print('key', event.key)
        if event.key == ' ':
            self.paused = not self.paused
        elif event.key == 's':
            for obstacle in self.obstacles:
                if isinstance(obstacle, Maze):
                    path = f'tests/fixtures/recording_{int(time.time())}.json'
                    save_recording(obstacle, self.robot, path)
                    print('saved recording to', path)
                    break


def visualize_recording(path: str, interval_ms: float = 500) -> Simulator:
    with open(path, 'r') as f:
        data = json.load(f)
    maze = Maze.from_dict(data['maze'])
    scan_log = [(x, y) for x, y in data['scan_log']]
    sim = Simulator(Point(*scan_log[0]), [maze])
    sim.robot.on_scan = None
    sim.robot.scan_radius = data.get('scan_radius', 5.0)
    sim.robot.scan_resolution = data.get('scan_resolution', 8)
    live_process = sim.process
    step = {'i': 0}
    def replay_step() -> None:
        if step['i'] >= len(scan_log):
            live_process()
            return
        sim.robot.position = Point(*scan_log[step['i']])
        sim.robot.perform_scan(maze)
        step['i'] += 1
    sim.process = replay_step

    sim.animate(interval_ms=interval_ms)
    return sim
