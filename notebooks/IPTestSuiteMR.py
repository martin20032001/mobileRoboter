from shapely.geometry import box, Point, LineString, Polygon
from shapely.affinity import translate
from IPBenchmark import Benchmark
from IPMobileRobotCollisionChecker import MobileRobotCollisionChecker
from HelperFunction import generate_random_polygon_shape
import random

# --- Roboterdefinitionen ---
robots = {
    "L_Robot": Polygon([(0, 0), (0, 1), (1, 1.5), (1, 1), (1.5, 1), (0, 0)]),
    "H_Robot": Polygon([
        (1.6, 0.0), (1.7, 1.1), (2.2, 1.1), (2.2, 1.5), (1.7, 1.5), (1.6, 2.0),
        (1.2, 2.0), (1.2, 1.5), (0.6, 1.5), (0.6, 1.3), (1.2, 1.3), (1.3, 0.0), (1.6, 0.0)
    ]),
    "U_Robot": Polygon([
        (0, 0), (0, 2), (0.5, 2), (0.5, 0.5), (1.5, 0.5), (1.5, 3), (2, 3), (2, 0)
    ]),
    "T_Robot": Polygon([
        (0, 0), (0, 1), (0.4, 1), (0.4, 1.6), (1.6, 1.6), (1.6, 1), (2, 1), (2, 0),
        (1.6, 0), (1.6, 0.6), (0.4, 0.6), (0.4, 0)
    ]),
    "C_Robot": Polygon([
        (0.5, 0), (0.5, 0.6), (0.1, 0.6), (0.1, 1.4), (0.6, 1.4), (0.6, 2), (1.8, 2),
        (1.8, 1.6), (1.2, 1.6), (1.2, 1.2), (2, 1.2), (2, 0.8), (1.2, 0.8),
        (1.2, 0.4), (1.8, 0.4), (1.8, 0), (0.5, 0)
    ]),
    "R_Robot": Polygon([
        (0, 2), (3, 2), (3, 1.4), (1.8, 1.4), (1.8, 0.8), (2.5, 0.8), (2.5, 0),
        (0.5, 0), (0.5, 0.8), (1.2, 0.8), (1.2, 1.4), (0, 1.4), (0, 2)
    ])
}

# --- Umgebungsdefinitionen ---
scenes = {
    "scene1": {
        "wall1": box(6, 0, 8, 16),
        "wall3": box(12, 0, 14, 16)
    },
    "scene2": {
        "block1": box(5, 3, 7, 5),
        "block3": box(12, 0, 13, 10),
        "block4": box(5, 15, 7, 17),
        "block5": box(12, 20, 13, 15),
        "arc1": Point(5, 8).buffer(2).difference(Point(6, 9).buffer(1.5)),
        "line_obstacle": LineString([(15, 3), (17, 6)]).buffer(0.4)
    },
    "scene3": {
        f"diag{i}": box(i, i, i + 0.8, i + 0.8) for i in range(4, 18, 2)
    },
    "scene4": {
        "wall_left": box(5, 0, 6, 12),
        "wall_right": box(12, 8, 13, 20),
        "block1": box(7, 3, 9, 5),
        "block3": box(14, 4, 16, 6),
        "round_obstacle": Point(7, 15).buffer(1.5),
        "diagonal_bar": LineString([(4, 16), (9, 11)]).buffer(0.3),
        "narrow_passage": box(16, 10, 17, 14)
    },
    "scene5": {
        f"mine_{row}_{col}": box(6 + col * 5, 6 + row * 5, 7 + col * 5, 7 + row * 5)
        for row in range(4) for col in range(4)
    }
}

# --- Start- und Zielpositionen ---
positions = {
    "scene1": ([[1, 1]], [[16, 1]], [[1, 1, 0]], [[16, 1, 180]]),
    "scene2": ([[1, 1]], [[18, 4]], [[1, 1, 0]], [[18, 4, 300]]),
    "scene3": ([[2.7, 0.5]], [[18, 18]], [[2.7, 0.5, 0]], [[18, 18, 100]]),
    "scene4": ([[1, 1]], [[16, 16]], [[1, 1, 0]], [[16, 16, 70]]),
    "scene5": ([[3, 3]], [[23, 23]], [[3, 3, 0]], [[23, 23, 245]])
}

# --- Benchmarks erzeugen ---
benchList = []
for i, scene_name in enumerate(scenes):
    scene = scenes[scene_name]
    start2d, goal2d, start3d, goal3d = positions[scene_name]
    limits2d = [[0, 27], [0, 27]] if scene_name == "scene5" else [[0, 24], [0, 24]] if scene_name == "scene3" else [[0, 22], [0, 20]]
    limits3d = limits2d + [[0, 360]]
    
    for name, robot in robots.items():
        is3d = name.startswith(("T_", "C_", "R_"))
        dof = 3 if is3d else 2
        limits = limits3d if dof == 3 else limits2d
        start = start3d if dof == 3 else start2d
        goal = goal3d if dof == 3 else goal2d
        label = f"{name.replace('_', '-')}-Roboter ({dof}DOF) in {scene_name}"
        bench_name = f"{dof}DOF_{name}_{scene_name}"
        checker = MobileRobotCollisionChecker(robot, scene, limits)
        benchList.append(Benchmark(bench_name, checker, start, goal, label, dof))

# --- Funktion zum Erzeugen zufälliger Benchmarks ---
def create_random_benchmark(name_prefix="random", dof=2, seed=None):
    if seed is not None:
        random.seed(seed)

    # Roboterform
    robot_shape = generate_random_polygon_shape(
        num_vertices=random.randint(5, 12),
        concavity=random.uniform(0.3, 0.6)
    )
    robot_name = f"AsymRobot_{random.randint(1000, 9999)}"

    # Hindernisse
    scene = {}
    for i in range(random.randint(6, 12)):
        choice = random.random()
        if choice < 0.3:
            x, y = random.uniform(1, 20), random.uniform(1, 20)
            w, h = random.uniform(1.5, 4), random.uniform(1.5, 4)
            scene[f"box_{i}"] = box(x, y, x + w, y + h)
        elif choice < 0.6:
            cx, cy = random.uniform(2, 20), random.uniform(2, 20)
            r = random.uniform(0.8, 2.0)
            scene[f"circle_{i}"] = Point(cx, cy).buffer(r)
        elif choice < 0.85:
            x1, y1 = random.uniform(0, 22), random.uniform(0, 22)
            x2, y2 = x1 + random.uniform(1, 4), y1 + random.uniform(1, 4)
            scene[f"line_{i}"] = LineString([(x1, y1), (x2, y2)]).buffer(0.4)
        else:
            x_off, y_off = random.uniform(2, 18), random.uniform(2, 18)
            shape = generate_random_polygon_shape(
                num_vertices=random.randint(5, 8),
                concavity=random.uniform(0.2, 0.4)
            )
            scene[f"poly_{i}"] = translate(shape.buffer(0), xoff=x_off, yoff=y_off)

    # Kollisionschecker
    limits = [[0, 22], [0, 22]] + ([[0, 360]] if dof == 3 else [])
    checker = MobileRobotCollisionChecker(robot_shape, scene, limits)

    # Start- und Zielkonfigurationen
    def sample_free_position():
        for _ in range(1000):
            x = random.uniform(limits[0][0], limits[0][1])
            y = random.uniform(limits[1][0], limits[1][1])
            pos = [x, y] if dof == 2 else [x, y, random.uniform(0, 360)]
            if not checker.pointInCollision(pos):
                return pos
        raise RuntimeError("⚠️ Keine gültige Start-/Zielposition gefunden.")

    start = [sample_free_position()]
    goal = [sample_free_position()]
    bench_name = f"{dof}DOF_{robot_name}_{name_prefix}"
    description = f"{robot_name} ({dof}DOF) in kreativer Szene"
    return Benchmark(bench_name, checker, start, goal, description, dof)
