# Filename: HelperFunctions.py
import os
import copy
import time
import json
import random
import math

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib import rcParams
import pickle

from tqdm import tqdm
from numbers import Number

from matplotlib import rc, animation
from matplotlib.gridspec import GridSpec
from matplotlib.animation import FuncAnimation, FFMpegWriter
from matplotlib.patches import Polygon as MplPolygon, Circle as MplCircle

from shapely import affinity
from shapely.affinity import translate, rotate
from shapely.geometry import Polygon, Point, LineString, MultiPolygon, MultiPoint
from shapely.geometry.base import BaseGeometry

from IPMobileRobotCollisionChecker import MobileRobotCollisionChecker
from IPMultiMobileRobotCollisionChecker import MultiMobileRobotCollisionChecker

from IPython.display import clear_output, HTML, display
from IPython import get_ipython
get_ipython().run_line_magic('config', "InlineBackend.figure_format = 'retina'")

class SceneBuilder:
    """
    Baut eine Planungs-Szene mit beliebigen Hindernissen und Roboter-Basisgeometrien auf.
    """
    def __init__(self):
        self.obstacles: dict[str, BaseGeometry] = {}
        self.robots:    dict[str, BaseGeometry] = {}

    def add_obstacle(self, name: str, geom: BaseGeometry) -> None:
        self.obstacles[name] = geom

    def add_robot(self, name: str, base_geom: BaseGeometry) -> None:
        self.robots[name] = base_geom

    def get_obstacles(self) -> dict[str, BaseGeometry]:
        return self.obstacles

    def get_robot_base(self, name: str) -> BaseGeometry:
        return self.robots[name]


def transform_geometry(base: BaseGeometry, x: float, y: float, theta: float) -> BaseGeometry:
    """
    Rotate and translate a Shapely geometry.
    :param base: Base geometry.
    :param x: x-offset.
    :param y: y-offset.
    :param theta: rotation angle in radians.
    :return: transformed geometry.
    """
    deg = math.degrees(theta)
    rotated = affinity.rotate(base, deg, origin=(0, 0))
    return affinity.translate(rotated, xoff=x, yoff=y)


def generate_random_polygon_shape(num_vertices: int = 8,
                                  concavity: float = 0.3) -> BaseGeometry:
    """
    Erzeuge zufälliges Polygon mit gegebenen Eckenzahl und Konkavität.
    """
    angles = sorted(random.uniform(0, 2 * math.pi) for _ in range(num_vertices))
    points = []
    for ang in angles:
        r = random.uniform(0.3, 1.0)
        if random.random() < concavity:
            r *= random.uniform(0.4, 0.8)
        points.append((r * math.cos(ang), r * math.sin(ang)))
    poly = Polygon(points)
    if not poly.is_valid or poly.area == 0:
        poly = MultiPoint(points).convex_hull
    return poly


def generate_letter_shape(letter: str, size: float = 1.0) -> BaseGeometry:
    """
    Generiert einfache Buchstabenformen (I, T, K) als Polygone.
    """
    w = size * 0.2
    h = size
    letter = letter.upper()
    if letter == 'I':
        return Polygon([(-w/2, 0), (w/2, 0), (w/2, h), (-w/2, h)])
    if letter == 'T':
        top = Polygon([(-h/2, h), (h/2, h), (h/2, h-w), (-h/2, h-w)])
        stem = Polygon([(-w/2, 0), (w/2, 0), (w/2, h-w), (-w/2, h-w)])
        return top.union(stem)
    if letter == 'K':
        vert = Polygon([(-w/2, 0), (w/2, 0), (w/2, h), (-w/2, h)])
        tri1 = Polygon([(w/2, h/2), (h/2, h), (h/2, h/2 + w)])
        tri2 = Polygon([(w/2, h/2), (h/2, 0), (h/2, w)])
        return vert.union(tri1).union(tri2)
    # Fallback
    return generate_random_polygon_shape()


def generate_robot_shape(index: int) -> BaseGeometry:
    """
    Erzeugt eine zufällige Roboter-Basisgeometrie basierend auf dem Index.
    """
    if index == 0:
        base = generate_letter_shape('K', size=random.uniform(0.5, 1.5))
    elif index == 1:
        base = generate_letter_shape('I', size=random.uniform(0.5, 1.5))
    elif index == 2:
        base = generate_letter_shape('T', size=random.uniform(0.5, 1.5))
    else:
        choice = random.random()
        if choice < 0.3:
            base = generate_random_polygon_shape(num_vertices=random.randint(5, 8), concavity=0.2)
        elif choice < 0.6:
            base = generate_random_polygon_shape(num_vertices=random.randint(8, 15), concavity=0.5)
        else:
            base = generate_letter_shape(random.choice(['K', 'I', 'T']), size=random.uniform(0.5, 1.5))
    scale = random.uniform(0.5, 2.0)
    return affinity.scale(base, xfact=scale, yfact=scale, origin=(0, 0))


def plot_geometries(ax, geometries, facecolor='none', edgecolor='blue', alpha=1.0, linewidth=1.5):
    """
    Plotte eine Liste von Shapely-Geometrien.
    """
    for geom in geometries:
        gtype = geom.geom_type
        if gtype == 'Polygon':
            xs, ys = geom.exterior.xy
            ax.fill(xs, ys, fc=facecolor, ec=edgecolor, alpha=alpha, lw=linewidth)
        elif gtype == 'MultiPolygon':
            for part in geom.geoms:
                xs, ys = part.exterior.xy
                ax.fill(xs, ys, fc=facecolor, ec=edgecolor, alpha=alpha, lw=linewidth)


def update_scene(scene: SceneBuilder, n_robots: int):
    """
    Erzeugt n_robots zufällige Roboter in einer bestehenden SceneBuilder-Szene
    und plottet diese zusammen mit den Hindernissen.
    """
    clear_output(wait=True)
    scene.robots.clear()
    configs = []
    geoms = []
    for i in range(n_robots):
        shape = generate_robot_shape(i)
        name = f'robot_{i+1}'
        scene.add_robot(name, shape)
        x, y = random.uniform(0, 20), random.uniform(0, 20)
        theta = random.uniform(0, 2 * math.pi)
        configs.append((name, (x, y, theta)))
        geoms.append(transform_geometry(shape, x, y, theta))
    geoms += list(scene.get_obstacles().values())
    minx = min(g.bounds[0] for g in geoms)
    miny = min(g.bounds[1] for g in geoms)
    maxx = max(g.bounds[2] for g in geoms)
    maxy = max(g.bounds[3] for g in geoms)
    margin = max(maxx - minx, maxy - miny) * 0.05

    fig, ax = plt.subplots(figsize=(6, 6))
    plot_geometries(ax, scene.get_obstacles().values(), facecolor='gray', edgecolor='black', alpha=0.6)
    robot_geoms = [transform_geometry(scene.get_robot_base(name), *cfg) for name, cfg in configs]
    plot_geometries(ax, robot_geoms)
    ax.set_aspect('equal')
    ax.set_xlim(minx - margin, maxx + margin)
    ax.set_ylim(miny - margin, maxy + margin)
    ax.set_title(f'Szene mit {n_robots} zufälligen Robotern')
    plt.show()


def plot_collision_tests(robot_shape: BaseGeometry,
                         scene: dict[str, BaseGeometry],
                         checker,
                         test_positions: list[tuple[list[float], str]],
                         limits: list[list[float]] = [[0,20],[0,20]],
                         figsize_per_subplot: tuple[int,int] = (5,5)):
    """
    Führt Kollisionstests an mehreren Positionen durch und visualisiert sie.
    :param robot_shape: Basisgeometrie des Roboters (zentrisch um Ursprung).
    :param scene: Dict von Hindernisnamen zu Geometrien.
    :param checker: Instanz von MobileRobotCollisionChecker.
    :param test_positions: Liste von Tupeln ([x,y,theta], titel).
    :param limits: [x_limits, y_limits] für die Achsen.
    :param figsize_per_subplot: Größe jedes Subplots.
    """
    n = len(test_positions)
    fig, axs = plt.subplots(1, n, figsize=(figsize_per_subplot[0] * n,
                                           figsize_per_subplot[1]))
    for ax, (pos, title) in zip(axs, test_positions):
        xlim, ylim = limits
        ax.set_title(f"{title} @ {pos}")
        ax.set_xlim(*xlim)
        ax.set_ylim(*ylim)
        ax.set_aspect('equal')
        # Hindernisse zeichnen
        plot_geometries(ax, scene.values(), facecolor='red', edgecolor='black', alpha=0.5)
        # Roboter transformieren und zeichnen
        robot_trans = transform_geometry(robot_shape, pos[0], pos[1],
                                         math.radians(pos[2]) if isinstance(pos[2], (int,float)) else pos[2])
        in_collision = checker.pointInCollision(pos)
        color = 'green' if not in_collision else 'orange'
        text_result = "Kollisionsfrei" if not in_collision else "Kollision"
        plot_geometries(ax, [robot_trans], facecolor=color, alpha=0.6)
        ax.plot(pos[0], pos[1], 'ko')
        ax.text(0.5, -0.1,
                f"Checker: {text_result}",
                transform=ax.transAxes,
                fontsize=12, ha='center', va='top')
    plt.tight_layout()
    plt.show()

def transform_robot(pos, robot_shape, use_radians=False):
    x, y = pos[0], pos[1]
    theta = pos[2] if len(pos) == 3 else 0
    r = rotate(robot_shape, theta, origin='centroid', use_radians=use_radians)
    return translate(r, xoff=x, yoff=y)

def interpolate_path(path, steps_per_segment=10):
    interp_path = []
    for i in range(len(path) - 1):
        p1 = np.array(path[i])
        p2 = np.array(path[i + 1])
        for j in range(steps_per_segment):
            alpha = j / steps_per_segment
            interp = (1 - alpha) * p1[:2] + alpha * p2[:2]
            if len(p1) == 3:
                dtheta = (p2[2] - p1[2] + 180) % 360 - 180
                theta = (p1[2] + alpha * dtheta) % 360
                interp_path.append(np.array([interp[0], interp[1], theta]))
            else:
                interp_path.append(interp)
    interp_path.append(np.array(path[-1]))
    return interp_path
def interpolate_path_equal_speed(path, total_steps=100):
    """
    Interpoliert den Pfad mit gleichmäßiger Geschwindigkeit.

    Args:
        path: Liste von Konfigurationen [(x, y, theta)], [(x, y)], etc.
        total_steps: Anzahl der Gesamt-Interpolationspunkte.
    """
    path = np.array(path)
    interp_path = []

    # Berechne die Abstände zwischen den Punkten
    distances = np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1)
    cumulative_distances = np.insert(np.cumsum(distances), 0, 0)
    total_distance = cumulative_distances[-1]

    # Gleichmäßig verteilte Ziellängen
    target_distances = np.linspace(0, total_distance, total_steps)

    # Interpolieren
    for d in target_distances:
        # Finde das Segment
        idx = np.searchsorted(cumulative_distances, d, side='right') - 1
        idx = min(idx, len(path) - 2)  # Grenze absichern

        # Lokale Interpolation innerhalb des Segments
        segment_start = path[idx]
        segment_end = path[idx + 1]
        segment_length = cumulative_distances[idx + 1] - cumulative_distances[idx]
        if segment_length == 0:
            alpha = 0
        else:
            alpha = (d - cumulative_distances[idx]) / segment_length

        # Interpolation für Position
        interp_pos = (1 - alpha) * segment_start[:2] + alpha * segment_end[:2]

        if len(segment_start) == 3:
            # Interpolation für Winkel (korrekte Behandlung von Sprüngen über 360°)
            dtheta = (segment_end[2] - segment_start[2] + 180) % 360 - 180
            theta = (segment_start[2] + alpha * dtheta) % 360
            interp_path.append(np.array([interp_pos[0], interp_pos[1], theta]))
        else:
            interp_path.append(interp_pos)

    return interp_path

def compute_prm_path(planner_cls, collision_checker, start, goal, config):
    planner = planner_cls(collision_checker)
    try:
        path_ids = planner.planPath([start], [goal], config)
        return path_ids, planner.graph
    except Exception as e:
        print(f"❌ Fehler bei Pfadplanung mit {planner_cls.__name__}: {e}")
        return [], planner.graph

def plot_configuration_space(ax, graph, path, start, goal, dof, collision_checker, skip_edge_length=False):
    if dof == 3:
        for u, v in graph.edges():
            p1, p2 = graph.nodes[u]['pos'], graph.nodes[v]['pos']
            ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], color='gray', linewidth=0.3)
        pts = np.array([graph.nodes[n]['pos'] for n in graph.nodes()])
        ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], c='k', s=2)

        # Pfad einzeichnen und Kantenlängen dranschreiben
        path_xyz = np.array(path)
        ax.plot(path_xyz[:, 0], path_xyz[:, 1], path_xyz[:, 2], 'b-', linewidth=2)
        for p1, p2 in zip(path_xyz[:-1], path_xyz[1:]):
            length = np.linalg.norm(p2 - p1)
            midpoint = (p1 + p2) / 2
            if not skip_edge_length:
                ax.text(midpoint[0], midpoint[1], midpoint[2], f"{length:.2f}", fontsize=8, color='red')

        ax.scatter(start[0], start[1], start[2], c='g', s=50, marker='o')
        ax.scatter(goal[0], goal[1], goal[2], c='r', s=50, marker='^')
        robot_dot = ax.scatter([], [], [], c='r', s=30)

    else:
        ax.set_xlim(collision_checker.limits[0])
        ax.set_ylim(collision_checker.limits[1])
        for u, v in graph.edges():
            p1, p2 = graph.nodes[u]['pos'], graph.nodes[v]['pos']
            ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color='gray', linewidth=0.3)
        pts = np.array([graph.nodes[n]['pos'] for n in graph.nodes()])
        ax.plot(pts[:, 0], pts[:, 1], 'k.', markersize=2)

        # Pfad einzeichnen und Kantenlängen dranschreiben
        pa = np.array(path)
        ax.plot(pa[:, 0], pa[:, 1], 'b-', linewidth=2)
        for p1, p2 in zip(pa[:-1], pa[1:]):
            length = np.linalg.norm(p2 - p1)
            midpoint = (p1 + p2) / 2
            if not skip_edge_length:
                ax.text(midpoint[0], midpoint[1], f"{length:.2f}", fontsize=8, color='red')

        ax.scatter(start[0], start[1], c='g', s=50)
        ax.scatter(goal[0], goal[1], c='r', s=50)
        robot_dot, = ax.plot([], [], 'ro', markersize=6)
    return robot_dot


def plot_work_space(ax, scene, robot_shape, start, goal, collision_checker):
    limits = collision_checker.limits
    ax.set_xlim(limits[0])
    ax.set_ylim(limits[1])
    for obs in scene.values():
        x, y = obs.exterior.xy
        ax.fill(x, y, color='red', alpha=0.5)
    start_poly = transform_robot(start, robot_shape)
    goal_poly = transform_robot(goal, robot_shape)
    ax.fill(*start_poly.exterior.xy, color='green', alpha=0.4)
    ax.fill(*goal_poly.exterior.xy, color='blue', alpha=0.4)
    patch = plt.Polygon(np.array(start_poly.exterior.coords), closed=True, fc='orange', alpha=0.8)
    ax.add_patch(patch)
    return patch

def make_animation(fig, init_func, animate_func, n_frames, interval=100):
    return animation.FuncAnimation(
        fig, animate_func, frames=n_frames,
        init_func=init_func, blit=False, interval=interval, repeat=False
    )


def make_init_func(robot_patch, robot_dot, coord1, coord2, start, robot_shape, dof):
    def init():
        transformed = transform_robot(start, robot_shape)
        robot_patch.set_xy(np.array(transformed.exterior.coords))
        if dof == 3:
            robot_dot._offsets3d = ([], [], [])
        else:
            robot_dot.set_data([], [])
        coord1.set_text('')
        coord2.set_text('')
        return robot_patch, robot_dot, coord1, coord2
    return init


def make_animate_func(robot_patch, robot_dot, coord1, coord2, interp, robot_shape, dof):
    def animate(i):
        pos = interp[min(i, len(interp) - 1)]
        transformed = transform_robot(pos, robot_shape)
        robot_patch.set_xy(np.array(transformed.exterior.coords))
        if dof == 3:
            robot_dot._offsets3d = ([pos[0]], [pos[1]], [pos[2]])
        else:
            robot_dot.set_data([pos[0]], [pos[1]])
        txt = f"x={pos[0]:.2f}, y={pos[1]:.2f}" + (f", θ={pos[2]:.2f}" if dof == 3 else "")
        coord1.set_text(txt)
        coord2.set_text(txt)
        return robot_patch, robot_dot, coord1, coord2
    return animate

def path_length(path):
    return sum(np.linalg.norm(np.array(path[i+1]) - np.array(path[i])) for i in range(len(path)-1))

def run_benchmark(planner_cls, planner_name, config, benchmarks, max_attempts=10):
    results = []
    print(f"🔍 Starte Benchmarks für {planner_name}...")
    for idx, benchmark in enumerate(tqdm(benchmarks, desc=f"{planner_name} Benchmarks", leave=False)):
        start = benchmark.startList[0]
        goal  = benchmark.goalList[0]
        dof   = len(start)
        path_ids = []
        t0 = time.time()
        for attempt in range(1, max_attempts + 1):
            path_ids, graph = compute_prm_path(planner_cls, benchmark.collisionChecker, start, goal, config)
            if path_ids:
                break
        t1 = time.time()
        duration = t1 - t0
        if not path_ids:
            results.append({
                'Benchmark': idx,
                'Planner': planner_name,
                'Time [s]': None,
                'Roadmap Size': len(graph.nodes),
                'Path Points': 0,
                'Path Length': None,
                'Path': None,
                'Graph': graph,
                'CollisionChecker': benchmark.collisionChecker,
                'Start': start,
                'Goal': goal
            })
            continue
        path = [graph.nodes[n]['pos'] for n in path_ids]
        results.append({
            'Benchmark': idx,
            'Planner': planner_name,
            'Time [s]': round(duration, 3),
            'Roadmap Size': len(graph.nodes),
            'Path Points': len(path),
            'Path Length': round(path_length(path), 3),
            'Path': path,
            'Graph': graph,
            'CollisionChecker': benchmark.collisionChecker,
            'Start': start,
            'Goal': goal
        })
    return results

def run_benchmark_adaptive_multi_try_sampling(planner_cls, planner_name, config, benchmarks, 
                                              max_attempts=10, max_scalings=5, scale_factor=1.5,
                                              params_output_file='found_params.json', multi_robot=False):
    """
    Führt Benchmarks durch und versucht bei Fehlschlag adaptive Skalierung.
    Speichert initiale Parameter jedes Verfahrens und alle erfolgreichen Läufe
    in einer JSON-Datei, ohne vorhandene Einträge zu überschreiben.
    """

    def _scale_config(cfg, factor):
        new_cfg = copy.deepcopy(cfg)
        if isinstance(cfg, dict):
            for k, v in cfg.items():
                if isinstance(v, Number) and not isinstance(v, bool):
                    scaled = v * factor
                    new_cfg[k] = int(scaled) if type(v) is int else scaled
        elif hasattr(new_cfg, '__dict__'):
            for attr, val in vars(new_cfg).items():
                if isinstance(val, Number) and not isinstance(val, bool):
                    scaled = val * factor
                    setattr(new_cfg, attr, int(scaled) if type(val) is int else scaled)
        else:
            raise TypeError(f"Cannot scale config of type {type(cfg)}")
        return new_cfg

    def _cfg_to_dict(cfg):
        if isinstance(cfg, dict):
            return cfg.copy()
        elif hasattr(cfg, '__dict__'):
            return {k: v for k, v in vars(cfg).items() if isinstance(v, (int, float, str, bool, list, dict))}
        else:
            return {}

    # Lade oder initialisiere found_params
    found_params = {}
    if os.path.exists(params_output_file):
        try:
            with open(params_output_file, 'r') as f:
                raw = json.load(f)
            if isinstance(raw, list):
                grouped = {}
                for entry in raw:
                    pl = entry.get('Planner', 'Unknown')
                    grouped.setdefault(pl, {
                        'initialConfig': None,
                        'runs': []
                    })
                    if grouped[pl]['initialConfig'] is None:
                        grouped[pl]['initialConfig'] = entry.get('Parameters', {})
                    grouped[pl]['runs'].append({
                        'Benchmark': entry.get('Benchmark'),
                        'Parameters': entry.get('Parameters', {})
                    })
                found_params = grouped
            elif isinstance(raw, dict):
                found_params = raw
        except Exception:
            found_params = {}

    if planner_name not in found_params:
        found_params[planner_name] = {
            'initialConfig': _cfg_to_dict(config),
            'runs': []
        }

    results = []
    new_runs = []
    print(f"🔍 Starte Benchmarks für {planner_name}...")

    for idx, bm in enumerate(tqdm(benchmarks, desc=f"{planner_name} Benchmarks", leave=False)):
        
        if multi_robot:
            # flatten start, goal lists, sum up dof list to create one large configuration space
            start = [number for coords in bm.startList for number in coords]
            goal = [number for coords in bm.goalList for number in coords]
        else:
            start = bm.startList[0]
            goal = bm.goalList[0]
        
        found = False
        graph = None
        path_ids = []
        used_cfg = None
        duration = None

        # 1) Versuche mit Original-Config
        for _ in range(max_attempts):
            t0 = time.time()
            try:
                path_ids, graph = compute_prm_path(planner_cls, bm.collisionChecker, start, goal, config)
                # 🛡️ Prüfen, ob Pfad und Graph valide sind
                if not path_ids or graph is None or len(graph.nodes) == 0:
                    path_ids = None
                    graph = None
                    continue
                found = True
                used_cfg = config
                duration = round(time.time() - t0, 3)
                break
            except Exception as e:
                print(f"❌ {planner_name}: Exception während Pfadsuche: {e}")
                graph = None
                path_ids = None
                continue

        # 2) Adaptive Skalierung
        if not found:
            for scale_iter in range(1, max_scalings + 1):
                print("apply adaptive scaling", scale_iter, "/", max_scalings+1, "to benchmark", idx)
                scaled_cfg = _scale_config(config, scale_factor ** scale_iter)
                for _ in range(max_attempts):
                    t0 = time.time()
                    try:
                        path_ids, graph = compute_prm_path(planner_cls, bm.collisionChecker, start, goal, scaled_cfg)
                        if not path_ids or graph is None or len(graph.nodes) == 0:
                            path_ids = None
                            graph = None
                            continue
                        found = True
                        used_cfg = scaled_cfg
                        duration = round(time.time() - t0, 3)
                        break
                    except Exception as e:
                        print(f"❌ {planner_name}: Exception während Skalierungsversuch: {e}")
                        graph = None
                        path_ids = None
                        continue
                if found:
                    break

        # Dokumentiere erfolgreichen Lauf oder Fehler
        if found and used_cfg is not None and path_ids and len(graph.nodes) > 0:
            new_runs.append({
                'Benchmark': idx,
                'Parameters': _cfg_to_dict(used_cfg)
            })
            path = [graph.nodes[n]['pos'] for n in path_ids]
            results.append({
                'Benchmark': idx,
                'Planner': planner_name,
                'Time [s]': duration,
                'Roadmap Size': len(graph.nodes),
                'Path Points': len(path),
                'Path Length': round(path_length(path), 3),
                'Path': path,
                'Graph': graph,
                'CollisionChecker': bm.collisionChecker,
                'Start': start,
                'Goal': goal,
                'Error': None
            })
        else:
            results.append({
                'Benchmark': idx,
                'Planner': planner_name,
                'Time [s]': None,
                'Roadmap Size': len(graph.nodes) if graph else 0,
                'Path Points': 0,
                'Path Length': None,
                'Path': None,
                'Graph': graph,
                'CollisionChecker': bm.collisionChecker,
                'Start': start,
                'Goal': goal,
                'Error': "Kein Pfad gefunden oder leerer Graph"
            })

    if new_runs:
        found_params[planner_name]['runs'].extend(new_runs)
        try:
            with open(params_output_file, 'w') as f:
                json.dump(found_params, f, indent=4)
            print(f"✅ '{params_output_file}' aktualisiert: {len(new_runs)} neue Läufe für '{planner_name}'.")
        except Exception as e:
            print(f"⚠️ Fehler beim Schreiben der Parameter-Datei: {e}")
    else:
        print(f"ℹ️ Keine neuen erfolgreichen Läufe für '{planner_name}'.")

    return results

def animate_saved_result(results, selected_benchmark, selected_planner,
                         steps_per_segment=3, save_path: str = None, fps: int = 30):
    """
    Animiert den gespeicherten Pfad.
    Zeigt die Animation immer im Notebook und speichert optional als MP4 mit Fortschrittsanzeige.
    """
    plt.rcParams.update({
        'figure.dpi': 100,
        'savefig.dpi': 100,
        'font.size': 12,
        'lines.antialiased': True,
        'patch.antialiased': True
    })

    # Suche passenden Benchmark und Planner
    match = next(
        (r for r in results if r['Benchmark'] == selected_benchmark and r['Planner'] == selected_planner), None
    )
    if not match or match['Path'] is None:
        print("❌ Kein Pfad gefunden für diese Auswahl.")
        return

    path = match['Path']
    graph = match['Graph']
    start = match['Start']
    goal = match['Goal']
    collision_checker = match['CollisionChecker']
    robot_shape = collision_checker.robot_shape
    scene = collision_checker.scene
    dof = len(start)

    # Interpolierter Pfad mit gleichmäßiger Geschwindigkeit
    total_steps = steps_per_segment * len(path) * 10
    interp = interpolate_path_equal_speed(path, total_steps=total_steps)

    fig = plt.figure(figsize=(10, 5), dpi=100)
    ax1 = fig.add_subplot(1, 2, 1, projection='3d') if dof == 3 else fig.add_subplot(1, 2, 1)
    ax1.set_title(f'Konfigurationsraum ({selected_planner})')

    coord1 = (ax1.text2D(0.02, 0.95, '', transform=ax1.transAxes, fontsize=10,
                         verticalalignment='top', bbox=dict(facecolor='white', alpha=0.7))
              if dof == 3
              else ax1.text(0.02, 0.95, '', transform=ax1.transAxes, fontsize=10,
                            verticalalignment='top', bbox=dict(facecolor='white', alpha=0.7)))
    ax2 = fig.add_subplot(1, 2, 2)
    ax2.set_title(f'Arbeitsraum ({selected_planner})')
    coord2 = ax2.text(0.02, 0.95, '', transform=ax2.transAxes, fontsize=10,
                      verticalalignment='top', bbox=dict(facecolor='white', alpha=0.7))

    robot_dot = plot_configuration_space(ax1, graph, path, start, goal, dof, collision_checker)
    robot_patch = plot_work_space(ax2, scene, robot_shape, start, goal, collision_checker)

    init = make_init_func(robot_patch, robot_dot, coord1, coord2, start, robot_shape, dof)
    animate_func = make_animate_func(robot_patch, robot_dot, coord1, coord2, interp, robot_shape, dof)

    ani = animation.FuncAnimation(fig, animate_func, init_func=init,
                                  frames=len(interp), interval=1000 // fps)

    # 📺 Zeige Animation im Notebook
    from IPython.display import HTML
    display(HTML(ani.to_jshtml()))

    # 💾 Zusätzlich speichern mit Fortschrittsanzeige
    if save_path:
        try:
            Writer = animation.writers['ffmpeg']
            writer = Writer(fps=fps, metadata=dict(artist='PRM'), bitrate=3000)

            print(f"💾 Speichere Animation nach {save_path}...")
            with tqdm(total=len(interp), desc="🎞 Rendering Frames") as pbar:
                def progress_callback(current_frame, total_frames):
                    pbar.update(1)
                ani.save(save_path, writer=writer, dpi=100,
                         progress_callback=progress_callback)
            print(f"✅ Video gespeichert unter: {save_path}")

        except Exception as e:
            print(f"⚠️ Fehler beim Speichern mit ffmpeg: {e}")

    plt.close(fig)


def animate_robot_scene(
    num_obstacles=20,
    num_waypoints=15,
    obstacle_types=("box", "circle", "polygon"),
    area_size=20,
    margin=2,
    edge_band=3,
    robot_size_scale=1.0,
    animation_speed=80,
    embed_limit_mb=100,
    robot_speed=0.2,
    num_robots=1,
    skip_collision_free_start=False
):
    """
    Erstellt und animiert eine Roboterfahrt mit konstanter Geschwindigkeit.

    Args:
        num_obstacles (int): Anzahl der Hindernisse.
        num_waypoints (int): Anzahl der Wegpunkte.
        obstacle_types (tuple): Hindernistypen ("box", "circle", "polygon").
        area_size (float): Größe der Szene (quadratisch).
        margin (float): Abstand zur Szenenkante für Hindernisse.
        edge_band (float): Breite des Randbereichs für Wegpunkte.
        robot_size_scale (float): Skaliert die Robotergröße.
        animation_speed (int): Intervall zwischen Frames (ms).
        embed_limit_mb (int): Max Größe der Notebook-Animation.
        robot_speed (float): Geschwindigkeit des Roboters (Einheiten pro Frame).
    """
    mpl.rcParams['animation.embed_limit'] = embed_limit_mb

    # Roboter: Strichmännchen
    def create_stickman(scale=1.0):
        head = Point(0, 0.8).buffer(0.25 * scale)
        body = LineString([(0, -0.5 * scale), (0, 0.5 * scale)]).buffer(0.1 * scale)
        arms = LineString([(-0.5 * scale, 0.4 * scale), (0.5 * scale, 0.4 * scale)]).buffer(0.08 * scale)
        leg_left = LineString([(0, -0.5 * scale), (-0.4 * scale, -1.2 * scale)]).buffer(0.08 * scale)
        leg_right = LineString([(0, -0.5 * scale), (0.4 * scale, -1.2 * scale)]).buffer(0.08 * scale)
        return head.union(body).union(arms).union(leg_left).union(leg_right)

    robot_shapes = []
    for i in range(num_robots):
        robot_shapes.append(create_stickman(scale=robot_size_scale))

    # Szene mit nicht überlappenden Hindernissen
    scene = {}
    np.random.seed(42)
    existing_obstacles = []

    def is_overlapping(new_geom, existing):
        return any(new_geom.intersects(obj.buffer(0.2)) for obj in existing)

    i = 0
    while len(scene) < num_obstacles and i < num_obstacles * 10:
        choice = np.random.rand()
        if choice < 0.4 and "box" in obstacle_types:
            x, y = np.random.uniform(margin, area_size - margin), np.random.uniform(margin, area_size - margin)
            w, h = np.random.uniform(1, 3), np.random.uniform(1, 3)
            new_obj = Polygon([(x, y), (x + w, y), (x + w, y + h), (x, y + h)])
        elif choice < 0.7 and "circle" in obstacle_types:
            cx, cy = np.random.uniform(margin, area_size - margin), np.random.uniform(margin, area_size - margin)
            r = np.random.uniform(0.5, 1.5)
            new_obj = Point(cx, cy).buffer(r)
        elif "polygon" in obstacle_types:
            num_vertices = np.random.randint(3, 8)
            angle = np.linspace(0, 2 * np.pi, num_vertices, endpoint=False)
            radius = np.random.uniform(0.5, 1.5, size=num_vertices)
            x = np.cos(angle) * radius
            y = np.sin(angle) * radius
            new_obj = Polygon(np.column_stack((x, y)))
            new_obj = rotate(new_obj, np.random.uniform(0, 360), origin=(0, 0))
            new_obj = translate(new_obj,
                                xoff=np.random.uniform(margin, area_size - margin),
                                yoff=np.random.uniform(margin, area_size - margin))
        else:
            i += 1
            continue

        if not is_overlapping(new_obj, existing_obstacles):
            scene[f"obstacle_{len(scene)}"] = new_obj
            existing_obstacles.append(new_obj)
        i += 1

    # Kollisionschecker
    single_checkers = []
    for i in range(num_robots):
        single_checkers.append(MobileRobotCollisionChecker(robot_shapes[i], scene, limits=[[0, area_size], [0, area_size], [0, 360]]))

    dofs = [3] * num_robots  # Jeder Roboter hat 3 DOFs (x, y, theta)
    multi_checker = MultiMobileRobotCollisionChecker(num_robots, robot_shapes, scene, dofs, sum(dofs), limits=[[0, area_size], [0, area_size], [0, 360]])

    # Wegpunkte nur im Randbereich generieren
    def generate_random_x_y_theta():
        side = np.random.choice(["top", "bottom", "left", "right"])
        if side == "top":
            x = np.random.uniform(margin, area_size - margin)
            y = np.random.uniform(area_size - edge_band, area_size - margin)
        elif side == "bottom":
            x = np.random.uniform(margin, area_size - margin)
            y = np.random.uniform(margin, edge_band)
        elif side == "left":
            x = np.random.uniform(margin, edge_band)
            y = np.random.uniform(margin, area_size - margin)
        else:  # right
            x = np.random.uniform(area_size - edge_band, area_size - margin)
            y = np.random.uniform(margin, area_size - margin)
        theta = np.random.uniform(0, 360)
        return x, y, theta
    
    waypoints = []
    for _ in range(num_waypoints):
        for _ in range(50):  # max. 50 Versuche pro Wegpunkt
            
            pos = []
            for i in range(num_robots):
                x, y, theta = generate_random_x_y_theta()
                pos.extend([x, y, theta])
                
            if not skip_collision_free_start:
                if not multi_checker.pointInCollision(pos):
                    waypoints.append(pos)
                    break
            else:
                waypoints.append(pos)
                break

    # Trajektorie mit konstanter Geschwindigkeit
    trajectory = []
    for i in range(len(waypoints) - 1):
        p1 = np.array(waypoints[i])
        p2 = np.array(waypoints[i + 1])
        distance = np.linalg.norm(p2[:2] - p1[:2])
        num_steps = max(int(distance / robot_speed), 1)

        for t in np.linspace(0, 1, num_steps):
            pos = (1 - t) * p1 + t * p2
            for j in range(num_robots):
                theta_idx = j * 3 + 2
                θ1 = p1[theta_idx]
                θ2 = p2[theta_idx]
                dθ = ((θ2 - θ1 + 180) % 360) - 180
                θ = (θ1 + t * dθ) % 360
                pos[theta_idx] = θ

            trajectory.append(pos.tolist())

    # Plot vorbereiten
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_aspect('equal')
    ax.set_xlim(multi_checker.limits[0])
    ax.set_ylim(multi_checker.limits[1])
    ax.set_title("Roboterfahrt")

    # Hindernisse zeichnen
    for obstacle in scene.values():
        if isinstance(obstacle, Polygon):
            x, y = obstacle.exterior.xy
            patch = MplPolygon(np.column_stack((x, y)), closed=True,
                               facecolor='lightgray', edgecolor='black', alpha=0.7)
            ax.add_patch(patch)
        elif isinstance(obstacle, Point):
            circ = MplCircle((obstacle.centroid.x, obstacle.centroid.y), obstacle.bounds[2]/2,
                             facecolor='lightgray', edgecolor='black', alpha=0.7)
            ax.add_patch(circ)

    robot_patches = []  # Platzhalter für Roboterteile

    # Animationsfunktion
    def update(frame):
        nonlocal robot_patches
        for patch in robot_patches:
            patch.remove()
        robot_patches = []
        
        pose = trajectory[frame]  # z. B. [x1, y1, θ1, x2, y2, θ2, ..., xN, yN, θN]
        collisions = []
        for i in range(num_robots):
            idx = i * 3
            subpose = pose[idx:idx+3]
            
            # Add other robots to the scene for collision checking
            for j in range(num_robots):
                if j == i:
                    continue  # don't include self
                jdx = j * 3
                other_pose = pose[jdx:jdx+3]
                other_shape = robot_shapes[j]
                transformed_other = transform_robot(other_pose, other_shape)
                single_checkers[i].scene[f"robot_{j}"] = transformed_other
            
            collision = single_checkers[i].pointInCollision(subpose)
            
            # Entferne Roboter wieder aus der szene
            keys_to_remove = [key for key in single_checkers[i].scene if key.startswith("robot_")]
            for key in keys_to_remove:
                del single_checkers[i].scene[key]
            
            collisions.append(collision)
            color = 'red' if collision else 'green'

            shape = robot_shapes[i]
            transformed_robot = transform_robot(subpose, shape)

            if isinstance(transformed_robot, Polygon):
                x, y = transformed_robot.exterior.xy
                patch = MplPolygon(np.column_stack((x, y)), closed=True,
                                facecolor=color, edgecolor='black', alpha=0.6)
                ax.add_patch(patch)
                robot_patches.append(patch)

        ax.set_title("Kollision" if any(collisions) else "Frei")

    writer = FFMpegWriter(fps=30)  # 30 Bilder pro Sekunde

    # Animation erstellen und speichern
    anim = FuncAnimation(fig, update, frames=len(trajectory), interval=animation_speed, repeat=False)

    # Speichern als MP4
    anim.save('robot_animation.mp4', writer=writer)
    plt.close(fig)  # ❌ Figure-Handle schließen, damit kein zusätzliches Standbild angezeigt wird
    return HTML(anim.to_jshtml())


def visualize_params_custom_layout(json_file: str, total_benchmarks: int = 30):
    # --- JSON laden ---
    with open(json_file, 'r') as f:
        data = json.load(f)

    records = []
    for planner, info in data.items():
        for run in info.get('runs', []):
            b = run['Benchmark'] + 1
            rec = {'Planner': planner, 'Benchmark': b}
            rec.update(run['Parameters'])
            records.append(rec)

    df = pd.DataFrame(records)
    params = [c for c in df.columns if c not in ('Planner', 'Benchmark')]

    # --- Layout: feste Struktur ---
    row_layout = [2, 4, 1, 2]
    total_plots = len(params)

    # Wenn mehr Plots als im festen Layout vorhanden sind
    while total_plots > sum(row_layout):
        row_layout.insert(-1, 4)  # Füge weitere Zeilen mit 4 Plots vor der letzten Zeile ein

    max_cols = max(row_layout)
    fig = plt.figure(figsize=(4 * max_cols, 3 * len(row_layout)))
    gs = GridSpec(len(row_layout), max_cols, figure=fig)

    # --- Plots zeichnen ---
    plot_i = 0
    handles_labels = {}  # für die globale Legende
    for row_i, cols_in_row in enumerate(row_layout):
        start_col = (max_cols - cols_in_row) // 2  # Zentrierung
        for col_i in range(cols_in_row):
            if plot_i >= total_plots:
                break
            ax = fig.add_subplot(gs[row_i, start_col + col_i])
            param = params[plot_i]
            for planner in df['Planner'].unique():
                sub = df[df['Planner'] == planner]
                line, = ax.plot(
                    sub['Benchmark'],
                    sub[param],
                    marker='o',
                    linestyle='None',
                    label=planner
                )
                # Handles und Labels für die globale Legende sammeln
                if planner not in handles_labels:
                    handles_labels[planner] = line
            ax.set_title(param)
            ax.set_xlabel('Benchmark')
            ax.set_ylabel('Wert')
            ax.set_xticks(range(1, total_benchmarks + 1, 5))
            ax.set_xlim(1, total_benchmarks)
            ax.grid(False)
            plot_i += 1

    # --- Globale Legende ---
    fig.legend(
        handles_labels.values(),
        handles_labels.keys(),
        loc='lower center',
        ncol=len(handles_labels),
        fontsize='small',
        bbox_to_anchor=(0.5, -0.02)
    )

    plt.tight_layout()
    plt.subplots_adjust(bottom=0.1)  # Platz für die Legende schaffen
    plt.show()



class PlannerRunner:
    def __init__(self, planner_class, config, name="Planner"):
        self.planner_class = planner_class
        self.config = config
        self.name = name

    def run_benchmarks(self, bench_list, max_attempts=10,
                       fps=30, steps_per_segment=5, save_animation=False, animation_dir="./animations"):
        for idx, benchmark in enumerate(bench_list):
            start = benchmark.startList[0]
            goal = benchmark.goalList[0]
            dof = len(start)
            path_ids = []

            # Suche Pfad mit mehreren Versuchen
            for attempt in tqdm(range(1, max_attempts + 1), desc=f"{self.name} Benchmark {idx+1}/{len(bench_list)}"):
                path_ids, graph = compute_prm_path(
                    self.planner_class, benchmark.collisionChecker, start, goal, self.config
                )
                if path_ids:
                    print(f"{self.name} Benchmark {idx}: Pfad gefunden nach {attempt} Versuchen.")
                    break
            else:
                print(f"{self.name} Benchmark {idx}: Kein Pfad nach {max_attempts} Versuchen.")
                continue

            # Animation anzeigen oder speichern
            self.visualize(graph, path_ids, start, goal, dof, benchmark,
                           fps=fps, steps_per_segment=steps_per_segment,
                           save=save_animation, animation_dir=animation_dir, benchmark_idx=idx)

    def visualize(self, graph, path_ids, start, goal, dof, benchmark,
                  fps=30, steps_per_segment=5, save=False, animation_dir="./animations", benchmark_idx=0):
        path = [graph.nodes[n]['pos'] for n in path_ids]
        total_steps = steps_per_segment * len(path)
        interp = interpolate_path_equal_speed(path, total_steps=total_steps)

        fig = plt.figure(figsize=(14, 7))
        if dof == 3:
            ax1 = fig.add_subplot(1, 2, 1, projection='3d')
            coord1 = ax1.text2D(0.02, 0.95, '', transform=ax1.transAxes, fontsize=12,
                                verticalalignment='top', bbox=dict(facecolor='white', alpha=0.7))
        else:
            ax1 = fig.add_subplot(1, 2, 1)
            coord1 = ax1.text(0.02, 0.95, '', transform=ax1.transAxes, fontsize=12,
                              verticalalignment='top', bbox=dict(facecolor='white', alpha=0.7))
        ax1.set_title(f'Konfigurationsraum ({self.name})')

        ax2 = fig.add_subplot(1, 2, 2)
        ax2.set_title(f'Arbeitsraum ({self.name})')
        coord2 = ax2.text(0.02, 0.95, '', transform=ax2.transAxes, fontsize=12,
                          verticalalignment='top', bbox=dict(facecolor='white', alpha=0.7))

        robot_dot = plot_configuration_space(ax1, graph, path, start, goal, dof, benchmark.collisionChecker)
        robot_patch = plot_work_space(
            ax2, benchmark.collisionChecker.scene, benchmark.collisionChecker.robot_shape,
            start, goal, benchmark.collisionChecker
        )

        init = make_init_func(robot_patch, robot_dot, coord1, coord2, start, benchmark.collisionChecker.robot_shape, dof)
        animate = make_animate_func(robot_patch, robot_dot, coord1, coord2, interp, benchmark.collisionChecker.robot_shape, dof)

        ani = make_animation(fig, init, animate, len(interp), interval=1000 // fps)

        if save:
            os.makedirs(animation_dir, exist_ok=True)
            file_name = f"{self.name}_Benchmark_{benchmark_idx}.mp4"
            save_path = os.path.join(animation_dir, file_name)
            print(f"Speichere Animation nach {save_path}...")
            writer = animation.FFMpegWriter(fps=fps)
            ani.save(save_path, writer=writer, dpi=200)
            print(f"Animation gespeichert: {save_path}")
        else:
            display(HTML(ani.to_jshtml()))

        plt.close(fig)



def generate_animations(results_file, benchmark_idx, planners,
                        save_dir="animations", fps=30,
                        steps_per_segment=5, save=True, all_planners=True, selected_idx=0):

    rcParams['animation.embed_limit'] = 2000

    # Ergebnisse laden
    with open(results_file, 'rb') as f:
        results = pickle.load(f)

    os.makedirs(save_dir, exist_ok=True)

    if all_planners:
        for planner in planners:
            save_path = os.path.join(save_dir, f"{planner}_benchmark_{benchmark_idx}.mp4")
            print(f"Erzeuge Animation für {planner} (Benchmark {benchmark_idx}) und speichere als {save_path}...")
            animate_saved_result(
                results,
                benchmark_idx,
                planner,
                save_path=save_path,
                fps=fps,
                steps_per_segment=steps_per_segment
            )
            print(f"Animation gespeichert unter: {save_path}")
    else:
        selected_planner = planners[selected_idx]
        if save:
            save_path = os.path.join(save_dir, f"{selected_planner}_benchmark_{benchmark_idx}.mp4")
            print(f"Erzeuge Animation für {selected_planner} (Benchmark {benchmark_idx}) und speichere als {save_path}...")
            animate_saved_result(
                results,
                benchmark_idx,
                selected_planner,
                save_path=save_path,
                fps=fps,
                steps_per_segment=steps_per_segment
            )
            print(f"Animation gespeichert unter: {save_path}")
        else:
            print(f"Zeige Animation für {selected_planner} (Benchmark {benchmark_idx})...")
            animate_saved_result(
                results,
                benchmark_idx,
                selected_planner,
                fps=fps,
                steps_per_segment=steps_per_segment
            )

    print("Fertig.")


def plot_benchmark_summary(results_csv: str):
    """
    Lädt die Benchmark-Ergebnisse und erstellt:
    - Eine Übersichtstabelle mit Min/Max-Highlighting.
    - Balkendiagramme für Zeit, Roadmap-Größe, Pfadpunkte und Pfadlänge.
    """
    import pandas as pd
    import matplotlib.pyplot as plt
    import seaborn as sns

    # --- Daten laden ---
    df_results = pd.read_csv(results_csv)

    # --- Mittelwerte pro Planner berechnen ---
    pivot_avg = df_results.groupby("Planner").mean(numeric_only=True).round(3)
    pivot_avg = pivot_avg[['Time [s]', 'Roadmap Size', 'Path Points', 'Path Length']]

    # --- Min/Max-Highlighting für Tabelle ---
    def highlight_min_max(df):
        return df.style.apply(lambda x: [
            'background-color: green' if v == x.min() else
            'background-color: coral' if v == x.max() else ''
            for v in x
        ], axis=0)

    print("Durchschnittswerte je Planungsverfahren:")
    styled_table = highlight_min_max(pivot_avg).format("{:.3f}")
    display(styled_table)

    # --- Plots vorbereiten ---
    metrics = ['Time [s]', 'Roadmap Size', 'Path Points', 'Path Length']
    titles = ['Suchzeit (Sekunden)', 'Größe der Roadmap (Knoten)',
              'Anzahl Punkte im Pfad', 'Pfadlänge (euklidisch)']

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    axes = axes.flatten()

    unique_planners = df_results['Planner'].unique()
    palette = dict(zip(unique_planners, sns.color_palette("Set2", n_colors=len(unique_planners))))

    for i, (metric, title) in enumerate(zip(metrics, titles)):
        sns.barplot(
            data=df_results,
            x='Planner',
            y=metric,
            hue='Planner',
            palette=palette,
            dodge=False,
            ax=axes[i]
        )

        axes[i].set_title(title)
        axes[i].set_xlabel("Planungsverfahren")
        axes[i].set_ylabel(title)
        axes[i].grid(True, linestyle='--', alpha=0.6)
        
        if i != 0:
            legend = axes[i].get_legend()
            if legend is not None:
                legend.remove()

    # Gemeinsame Legende unten hinzufügen
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='lower center', ncol=len(unique_planners), frameon=False)
    plt.tight_layout(rect=[0, 0.05, 1, 1]) 
    plt.show()

def plot_benchmark_metrics(results_csv: str):
    """
    Plottet pro Benchmark und Planungsverfahren die Metriken:
    - Time [s] (logarithmisch)
    - Roadmap Size
    - Path Points
    - Path Length
    """
    import pandas as pd
    import numpy as np
    import matplotlib.pyplot as plt

    # --- CSV laden ---
    df_results = pd.read_csv(results_csv)

    metrics = ['Time [s]', 'Roadmap Size', 'Path Points', 'Path Length']

    # Planner in fester Reihenfolge 
    all_planners = ['BasicPRM', 'LazyPRM', 'VisPRM', 'RRTSimple']
    planners_in_data = [p for p in all_planners if p in df_results['Planner'].unique()]

    # Benchmarks 1-basiert
    benchmarks = sorted(df_results['Benchmark'].unique())
    benchmarks_1based = [b + 1 for b in benchmarks]

    # Feste Farben für Planner
    planner_colors = {
        'BasicPRM': 'green',
        'LazyPRM': 'orange',
        'RRTSimple': 'blue',
        'VisPRM': 'purple'
    }

    # --- Subplot-Layout ---
    n_cols = 2
    n_rows = int(np.ceil(len(metrics) / n_cols))
    fig, axes = plt.subplots(n_rows, n_cols, figsize=(16, 6 * n_rows))
    axes = axes.flatten()

    for idx, metric in enumerate(metrics):
        ax = axes[idx]
        x = np.arange(len(benchmarks))
        width = 0.8 / len(planners_in_data)  # Balkenbreite für alle Planner

        for i, planner in enumerate(planners_in_data):
            values = []
            for benchmark in benchmarks:
                subset = df_results[
                    (df_results['Planner'] == planner) &
                    (df_results['Benchmark'] == benchmark)
                ]
                mean_value = subset[metric].mean() if not subset.empty else 0.001
                values.append(mean_value)

            offset = x + (i - len(planners_in_data)/2) * width
            ax.bar(offset, values, width=width, label=planner, color=planner_colors.get(planner, None))

        ax.set_title(f'{metric} pro Benchmark und Planer', fontsize=11)
        ax.set_xlabel('Benchmark', fontsize=9)
        ax.set_ylabel(metric, fontsize=9)
        ax.set_xticks(x)
        ax.set_xticklabels(benchmarks_1based, rotation=45, ha='right', fontsize=8)

        if metric == 'Time [s]':
            ax.set_yscale('log')
            ax.set_ylim(1e-1, 100)  # Anpassbarer Bereich
            ax.set_ylabel('Time [s] (logarithmisch)', fontsize=9)

        ax.grid(True, linestyle='--', alpha=0.5)

    # Überzählige Achsen löschen
    for i in range(len(metrics), len(axes)):
        fig.delaxes(axes[i])

    # Gemeinsame Legende unten
    handles, labels = ax.get_legend_handles_labels()
    fig.legend(handles, labels, loc='lower center', ncol=len(planners_in_data),
               fontsize=9, frameon=False)

    plt.tight_layout(rect=[0, 0.05, 1, 1])  # Platz für Legende unten
    plt.show()
