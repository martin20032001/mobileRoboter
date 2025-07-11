# Filename: HelperFunction.py
from shapely.geometry.base import BaseGeometry
from shapely import affinity
import random
import math
from shapely.geometry import Polygon, MultiPoint, Point
import matplotlib.pyplot as plt
from IPython.display import clear_output
import time
from tqdm.notebook import tqdm
import matplotlib.pyplot as plt
import numpy as np
from IPython.display import display, HTML
from matplotlib import rc
import os

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


    # HelperFunction.py

import numpy as np
from shapely.affinity import translate, rotate
import matplotlib.pyplot as plt
from matplotlib import animation

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

def compute_prm_path(planner_cls, collision_checker, start, goal, config):
    planner = planner_cls(collision_checker)
    try:
        path_ids = planner.planPath([start], [goal], config)
        return path_ids, planner.graph
    except Exception as e:
        print(f"❌ Fehler bei Pfadplanung mit {planner_cls.__name__}: {e}")
        return [], planner.graph

def plot_configuration_space(ax, graph, path, start, goal, dof, collision_checker):
    if dof == 3:
        for u, v in graph.edges():
            p1, p2 = graph.nodes[u]['pos'], graph.nodes[v]['pos']
            ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], color='gray', linewidth=0.3)
        pts = np.array([graph.nodes[n]['pos'] for n in graph.nodes()])
        ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], c='k', s=2)
        path_xyz = np.array(path)
        ax.plot(path_xyz[:, 0], path_xyz[:, 1], path_xyz[:, 2], 'b-', linewidth=2)
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
        pa = np.array(path)
        ax.plot(pa[:, 0], pa[:, 1], 'b-', linewidth=2)
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


import copy
import time
import json
import os
from tqdm import tqdm
from numbers import Number


def run_benchmark_adaptive_multi_try_sampling(planner_cls, planner_name, config, benchmarks, 
                                              max_attempts=10, max_scalings=5, scale_factor=1.5,
                                              params_output_file='found_params.json'):
    """
    Führt Benchmarks durch und versucht bei Fehlschlag adaptive Skalierung.
    Speichert initiale Parameter jedes Verfahrens und alle erfolgreichen Läufe
    in einer JSON-Datei, ohne vorhandene Einträge zu überschreiben.

    Args:
        planner_cls: Planer-Klasse für compute_prm_path.
        planner_name (str): Name des Planers (für Ausgabe).
        config: Konfigurationsobjekt (dict oder Objekt) mit numerischen Parametern.
        benchmarks (list): Liste von Benchmark-Objekten.
        max_attempts (int): Versuche je Parameter-Set.
        max_scalings (int): Max. Skalierungsdurchläufe bei Fehlschlag.
        scale_factor (float): Faktor pro Skalierungsschritt.
        params_output_file (str): Pfad zur JSON-Datei für Parameter und Läufe.

    Returns:
        list of dict: Ergebnisse pro Benchmark.
    """

    def _scale_config(cfg, factor):
        """
        Tiefe Kopie von cfg, bei der:
        - aus int-Vorlagen nach Skalierung wieder int wird
        - float-Vorlagen float bleiben
        """
        new_cfg = copy.deepcopy(cfg)
        if isinstance(cfg, dict):
            for k, v in cfg.items():
                if isinstance(v, Number) and not isinstance(v, bool):
                    scaled = v * factor
                    # wenn original ein int war, zurück zu int casten
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
            # Umwandlung der alten Liste in neues Dict-Format
            if isinstance(raw, list):
                grouped = {}
                for entry in raw:
                    pl = entry.get('Planner', 'Unknown')
                    if pl not in grouped:
                        grouped[pl] = {'initialConfig': None, 'runs': []}
                    if grouped[pl]['initialConfig'] is None:
                        grouped[pl]['initialConfig'] = entry.get('Parameters', {})
                    grouped[pl]['runs'].append({'Benchmark': entry.get('Benchmark'), 'Parameters': entry.get('Parameters', {})})
                found_params = grouped
            elif isinstance(raw, dict):
                found_params = raw
        except Exception:
            found_params = {}
    # Sicherstellen, dass found_params ein Dict ist
    if not isinstance(found_params, dict):
        found_params = {}
    found_params = {}
    if os.path.exists(params_output_file):
        try:
            with open(params_output_file, 'r') as f:
                raw = json.load(f)
            # Liste in dict umwandeln, falls nötig
            if isinstance(raw, list):
                grouped = {}
                for entry in raw:
                    pl = entry.get('Planner', 'Unknown')
                    grouped.setdefault(pl, {
                        'initialConfig': None,
                        'runs': []
                    })
                    # falls initialConfig noch nicht gesetzt
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

    # Sicherstellen, dass Struktur für diesen Planner vorhanden ist
    if planner_name not in found_params:
        found_params[planner_name] = {
            'initialConfig': _cfg_to_dict(config),
            'runs': []
        }

    results = []
    new_runs = []
    print(f"🔍 Starte Benchmarks für {planner_name}...")

    for idx, bm in enumerate(tqdm(benchmarks, desc=f"{planner_name} Benchmarks", leave=False)):
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
            path_ids, graph = compute_prm_path(planner_cls, bm.collisionChecker, start, goal, config)
            t1 = time.time()
            if path_ids:
                found = True
                used_cfg = config
                duration = round(t1 - t0, 3)
                break

        # 2) Adaptive Skalierung
        if not found:
            for scale_iter in range(1, max_scalings + 1):
                scaled_cfg = _scale_config(config, scale_factor ** scale_iter)
                for _ in range(max_attempts):
                    t0 = time.time()
                    path_ids, graph = compute_prm_path(planner_cls, bm.collisionChecker, start, goal, scaled_cfg)
                    t1 = time.time()
                    if path_ids:
                        found = True
                        used_cfg = scaled_cfg
                        duration = round(t1 - t0, 3)
                        break
                if found:
                    break

        # Dokumentiere erfolgreichen Lauf
        if found and used_cfg is not None:
            new_runs.append({
                'Benchmark': idx,
                'Parameters': _cfg_to_dict(used_cfg)
            })

        # Ergebnisse erfassen
        if not found:
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
                'Goal': goal
            })
        else:
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
                'Goal': goal
            })

    # Neue Läufe hinzufügen und speichern
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
                        steps_per_segment=3, save_path: str = None, fps: int = 10):

    match = next((r for r in results if r['Benchmark'] == selected_benchmark and r['Planner'] == selected_planner), None)
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
    interp = interpolate_path(path, steps_per_segment=steps_per_segment)

    fig = plt.figure(figsize=(14, 7))
    ax1 = fig.add_subplot(1, 2, 1, projection='3d') if dof == 3 else fig.add_subplot(1, 2, 1)
    ax1.set_title(f'Konfigurationsraum ({selected_planner})')
    coord1 = (ax1.text2D(0.02, 0.95, '', transform=ax1.transAxes, fontsize=12, verticalalignment='top',
                        bbox=dict(facecolor='white', alpha=0.7)) if dof == 3
            else ax1.text(0.02, 0.95, '', transform=ax1.transAxes, fontsize=12, verticalalignment='top',
                            bbox=dict(facecolor='white', alpha=0.7)))

    ax2 = fig.add_subplot(1, 2, 2)
    ax2.set_title(f'Arbeitsraum ({selected_planner})')
    coord2 = ax2.text(0.02, 0.95, '', transform=ax2.transAxes, fontsize=12,
                    verticalalignment='top', bbox=dict(facecolor='white', alpha=0.7))

    robot_dot = plot_configuration_space(ax1, graph, path, start, goal, dof, collision_checker)
    robot_patch = plot_work_space(ax2, scene, robot_shape, start, goal, collision_checker)

    init = make_init_func(robot_patch, robot_dot, coord1, coord2, start, robot_shape, dof)
    animate = make_animate_func(robot_patch, robot_dot, coord1, coord2, interp, robot_shape, dof)

    ani = make_animation(fig, init, animate, len(interp), interval=1000 // fps)

    if save_path:
        print(f"💾 Speichere Animation nach {save_path}...")
        Writer = animation.writers['ffmpeg']
        writer = Writer(fps=fps, metadata=dict(artist='PRM'), bitrate=1800)
        ani.save(save_path, writer=writer)
        print("✅ Video gespeichert.")

    else:
        display(HTML(ani.to_jshtml()))

    plt.close(fig)





import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Polygon as MplPolygon, Circle as MplCircle
from shapely.geometry import Polygon, Point, LineString, MultiPolygon
from shapely.affinity import rotate, translate
import matplotlib as mpl
from IPython.display import HTML
from IPMobileRobotCollisionChecker import MobileRobotCollisionChecker
from HelperFunction import transform_robot

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
    robot_speed=0.2
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

    robot_shape = create_stickman(scale=robot_size_scale)

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
    checker = MobileRobotCollisionChecker(robot_shape, scene, limits=[[0, area_size], [0, area_size], [0, 360]])

    # Wegpunkte nur im Randbereich generieren
    waypoints = []
    for _ in range(num_waypoints):
        for _ in range(50):  # max. 50 Versuche pro Wegpunkt
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
            if not checker.pointInCollision([x, y, theta]):
                waypoints.append([x, y, theta])
                break

    # Trajektorie mit konstanter Geschwindigkeit
    trajectory = []
    for i in range(len(waypoints) - 1):
        p1, p2 = np.array(waypoints[i]), np.array(waypoints[i + 1])
        distance = np.linalg.norm(p2[:2] - p1[:2])
        num_steps = max(int(distance / robot_speed), 1)
        for t in np.linspace(0, 1, num_steps):
            pos = (1 - t) * p1 + t * p2
            pos[2] %= 360  # Winkel normalisieren
            trajectory.append(pos.tolist())

    # Plot vorbereiten
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_aspect('equal')
    ax.set_xlim(checker.limits[0])
    ax.set_ylim(checker.limits[1])
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

    robot_patch = []  # Platzhalter für Roboterteile

    # Animationsfunktion
    def update(frame):
        nonlocal robot_patch
        for patch in robot_patch:
            patch.remove()
        pose = trajectory[frame]
        collision = checker.pointInCollision(pose)
        transformed_robot = transform_robot(pose, robot_shape)
        color = 'red' if collision else 'green'
        patches = []
        if isinstance(transformed_robot, Polygon):
            x, y = transformed_robot.exterior.xy
            patch = MplPolygon(np.column_stack((x, y)), closed=True,
                               facecolor=color, edgecolor='black', alpha=0.6)
            ax.add_patch(patch)
            patches.append(patch)
        elif isinstance(transformed_robot, MultiPolygon):
            for poly in transformed_robot.geoms:
                x, y = poly.exterior.xy
                patch = MplPolygon(np.column_stack((x, y)), closed=True,
                                   facecolor=color, edgecolor='black', alpha=0.6)
                ax.add_patch(patch)
                patches.append(patch)
        robot_patch = patches
        ax.set_title("Kollision" if collision else "Frei")

    # Animation
    # Animation
    anim = FuncAnimation(fig, update, frames=len(trajectory), interval=animation_speed, repeat=False)

    plt.close(fig)  # ❌ Figure-Handle schließen, damit kein zusätzliches Standbild angezeigt wird
    return HTML(anim.to_jshtml())


def visualize_params_custom_layout(json_file: str, total_benchmarks: int = 30):
    """
    Lädt die JSON-Datei und erstellt für jeden Parameter eine Grafik in einem
    benutzerdefinierten Layout:
    - Erste Zeile: 2 Plots
    - Zweite Zeile: 4 Plots
    - Dritte Zeile: 2 Plots
    - Vierte Zeile: 1 Plot (zentriert)

    Grid-Linien werden entfernt, die X-Achse zeigt Beschriftungen in 5er-Schritten,
    und es werden nur Marker ohne Verbindungs-Linien gezeichnet.
    Eine gemeinsame Legende wird einmal unterhalb des gesamten Plots angezeigt.
    """
    import json
    import matplotlib.pyplot as plt
    import pandas as pd
    from matplotlib.gridspec import GridSpec

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
