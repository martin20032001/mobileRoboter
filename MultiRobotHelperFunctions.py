import networkx as nx
import random
import math
import os
import numpy as np

import matplotlib as mpl
import matplotlib.pyplot as plt

from matplotlib import animation
from tqdm import tqdm
from HelperFunctions import translate, rotate, transform_robot, interpolate_path_equal_speed, make_animation, plot_configuration_space, compute_prm_path
from dependencies.IPTestSuiteMR import scenes, robots
from dependencies.IPBenchmark import Benchmark
from IPMultiMobileRobotCollisionChecker import MultiMobileRobotCollisionChecker

from IPython import get_ipython
from IPython.display import HTML, display
get_ipython().run_line_magic('config', "InlineBackend.figure_format = 'retina'")

def generate_valid_positions(collision_checker, num_positions, xlim, ylim):
    
    min_distance = 5
    positions = []
    obstacles = collision_checker.scene.values()
    
    while len(positions) < num_positions:
        x = random.uniform(xlim[0], xlim[1])
        y = random.uniform(ylim[0], ylim[1])
        theta = random.uniform(0, 360)
        
        if not all(math.hypot(x - px, y - py) >= min_distance for px, py, _ in positions):
            # check if new generated spot is away from already generated positions,
            # TODO maybe implement a more complex approach (using real collision checks? prohibit overlapping of multiple starts and multiple goals, but start<->goal can be overlapping)
            continue
        
        # Assure that every robot can be placed at x,y without collision
        translated_shapes = [
            translate(rotate(robot_shape, theta, origin='centroid', use_radians=False), xoff=x, yoff=y)
            for robot_shape in collision_checker.robot_shapes
        ]

        is_valid = all(
            not any(translated_shape.intersects(obstacle) for obstacle in obstacles)
            for translated_shape in translated_shapes
        )

        if is_valid:
            positions.append([x, y, theta])
                
    return positions

def split_combined_graph(G_combined, robot_dofs):
    
    num_robots = len(robot_dofs)
    graph_list = [nx.Graph() for _ in range(num_robots)]

    for node_id, data in G_combined.nodes(data=True):
        full_config = np.asarray(data['pos'])
        idx = 0
        for r, dof in enumerate(robot_dofs):
            sub_pos = tuple(full_config[idx:idx + dof])
            # sub_pos = tuple(full_config[idx:idx + 2])
            graph_list[r].add_node(sub_pos, pos=sub_pos)
            idx += dof

    for u, v in G_combined.edges:
        u_cfg = np.asarray(G_combined.nodes[u]['pos'])
        v_cfg = np.asarray(G_combined.nodes[v]['pos'])

        idx = 0
        for r, dof in enumerate(robot_dofs):
            u_r = tuple(u_cfg[idx:idx + dof])
            v_r = tuple(v_cfg[idx:idx + dof])

            # Safely add both nodes with 'pos' if not already present
            if u_r not in graph_list[r]:
                graph_list[r].add_node(u_r, pos=u_r)
            if v_r not in graph_list[r]:
                graph_list[r].add_node(v_r, pos=v_r)

            if u_r != v_r:
                graph_list[r].add_edge(u_r, v_r)
            idx += dof

    return graph_list

def interpolate_path_combined(path, robot_dofs, steps_per_segment=10):
    interp_path = []  # Will be a list of lists: [[robot1_cfg, robot2_cfg, ...], ...]

    for i in range(len(path) - 1):
        p1_combined = np.array(path[i])
        p2_combined = np.array(path[i + 1])
        
        for j in range(steps_per_segment):
            alpha = j / steps_per_segment
            current_frame = []
            current_idx = 0

            for dof_count in robot_dofs:
                p1_robot_segment = p1_combined[current_idx: current_idx + dof_count]
                p2_robot_segment = p2_combined[current_idx: current_idx + dof_count]

                interp = (1 - alpha) * p1_robot_segment[:2] + alpha * p2_robot_segment[:2]
                if dof_count == 3:
                    dtheta = (p2_robot_segment[2] - p1_robot_segment[2] + 180) % 360 - 180
                    theta = (p1_robot_segment[2] + alpha * dtheta) % 360
                    robot_cfg = np.array([interp[0], interp[1], theta])
                else:
                    robot_cfg = np.array([interp[0], interp[1]])
                
                current_frame.append(robot_cfg)

                current_idx += dof_count

            interp_path.append(current_frame)

    # Add final point
    final_frame = []
    current_idx = 0
    for dof_count in robot_dofs:
        robot_cfg = path[-1][current_idx: current_idx + dof_count]
        final_frame.append(np.array(robot_cfg))
        current_idx += dof_count
    interp_path.append(final_frame)
    
    return interp_path  # shape: [T][num_robots][dof]

def plot_combined_work_space(ax, scene, robot_shapes, start, goal, collision_checker):
    limits = collision_checker.limits
    ax.set_xlim(0, limits[0][1] + 2)
    ax.set_ylim(0, limits[1][1] + 2)
    ax.set_aspect("equal", adjustable="box")

    patches = []
    
    for obs in scene.values():
        x, y = obs.exterior.xy
        ax.fill(x, y, color='red', alpha=0.5)
    
    colors = mpl.colormaps["tab10"].colors
    for i, robot_shape in enumerate(robot_shapes):
        start_poly = transform_robot(start[i], robot_shape, use_radians=False)
        goal_poly = transform_robot(goal[i], robot_shape, use_radians=False)
        ax.fill(*start_poly.exterior.xy, color=colors[i], alpha=0.2)
        ax.fill(*goal_poly.exterior.xy, color=colors[i], alpha=0.2)
        patch = plt.Polygon(np.array(start_poly.exterior.coords), closed=True, fc=colors[i], alpha=0.8)
        ax.add_patch(patch)
        patches.append(patch)
    
    return patches


def make_multi_init_func(robot_patches, robot_dots, coord_texts1, coord_texts2, starts, robot_shapes, dof_list):
    def init():
        for i in range(len(robot_patches)):
            transformed = transform_robot(starts[i], robot_shapes[i])
            robot_patches[i].set_xy(np.array(transformed.exterior.coords))
            if dof_list[i] == 3:
                robot_dots[i]._offsets3d = ([], [], [])
            else:
                robot_dots[i].set_data([], [])
            # coord_texts1[i].set_text('')
            # coord_texts2[i].set_text('')
        return robot_patches, robot_dots, coord_texts1, coord_texts2
    return init

def make_multi_animate_func(robot_patches, robot_dots, coord_texts1, coord_texts2, robot_trajs, robot_shapes, dof_list):
    def animate(i):
        for r in range(len(robot_patches)):
            pos = robot_trajs[min(i, len(robot_trajs) - 1)][r]
            transformed = transform_robot(pos, robot_shapes[r], use_radians=False)
            robot_patches[r].set_xy(np.array(transformed.exterior.coords))
            if dof_list[r] == 3:
                robot_dots[r]._offsets3d = ([pos[0]], [pos[1]], [pos[2]])
            else:
                robot_dots[r].set_data([pos[0]], [pos[1]])
            txt = f"x={pos[0]:.2f}, y={pos[1]:.2f}" + (f", θ={pos[2]:.2f}" if dof_list[r] == 3 else "")
            # coord_texts1[r].set_text(txt)
            # coord_texts2[r].set_text(txt)
        return robot_patches, robot_dots, coord_texts1, coord_texts2
    return animate


def visualize_multi_robots(graph, path_ids, robot_dofs, num_robots, collisionChecker, benchmark, path):
    if path == None:
        path = [graph.nodes[n]['pos'] for n in path_ids]
    # print("gefundener Pfad:", path)
    
    # Interp: list of lists containing [robot1_cfg, robot2_cfg, ...] for each interpolation step
    interp = interpolate_path_combined(path, robot_dofs)
    
    # path_list: list of lists containing [start_cfg, ..., end_cfg] for each robot
    num_robots = len(interp[0])
    path_list = [[] for _ in range(num_robots)]
    for frame in interp:
        for i, robot_cfg in enumerate(frame):
            path_list[i].append(robot_cfg)
    
    # graph_list: list of graphs containing configuration spaces for each robot
    graph_list = split_combined_graph(graph, robot_dofs)
    
    fig = plt.figure(figsize=(8 * 2, 4 * num_robots))

    ax_workspace = fig.add_subplot(num_robots, 2, 2)  # Shared workspace on right
    ax_workspace.set_title('Arbeitsraum')

    ax_cspaces = []
    coord_texts1 = []
    coord_texts2 = []
    robot_dots = []

    for i in range(num_robots):
        ax = fig.add_subplot(num_robots, 2, 2 * i + 1, projection="3d" if robot_dofs[i] == 3 else None)
        ax.set_title(f'Konfigurationsraum Roboter {i}')
        ax_cspaces.append(ax)

        robot_dot = plot_configuration_space(ax, graph_list[i], path_list[i], benchmark.startList[i], benchmark.goalList[i],
                                             robot_dofs[i], collisionChecker, True)
        robot_dots.append(robot_dot)

    robot_patches = plot_combined_work_space(ax_workspace, collisionChecker.scene, collisionChecker.robot_shapes,
                                             benchmark.startList, benchmark.goalList, collisionChecker)

    init_func = make_multi_init_func(robot_patches, robot_dots, coord_texts1, coord_texts2, benchmark.startList,
                                     collisionChecker.robot_shapes, robot_dofs)
    animate_func = make_multi_animate_func(robot_patches, robot_dots, coord_texts1, coord_texts2, interp,
                                           collisionChecker.robot_shapes, robot_dofs)

    ani = make_animation(fig, init_func, animate_func, len(interp))
    
    return fig, ani

def animate_saved_result_multi_robots(multiRobotBenchList, results, selected_benchmark, selected_planner,
                         steps_per_segment=3, save_path: str = None, fps: int = 30):
    """
    Animiert den gespeicherten Pfad mit Fortschrittsanzeige und hoher Auflösung.
    """
    # Retina-Optimierung (falls im Notebook)
    """ipython = get_ipython()
    if ipython is not None:
        ipython.run_line_magic('matplotlib', 'inline')
        ipython.run_line_magic('config', "InlineBackend.figure_format = 'retina'")""" 

    plt.rcParams.update({
        'figure.dpi': 300,
        'savefig.dpi': 300,
        'font.size': 14,
        'lines.antialiased': True,
        'patch.antialiased': True
    })

    match = next((r for r in results if r['Benchmark'] == selected_benchmark and r['Planner'] == selected_planner), None)
    if not match or match['Path'] is None:
        print("❌ Kein Pfad gefunden für diese Auswahl.")
        return

    path = match['Path']
    graph = match['Graph']
    start = match['Start']
    goal = match['Goal']
    collision_checker = match['CollisionChecker']
    
    fig, ani = visualize_multi_robots(graph=graph, path_ids=None, robot_dofs=multiRobotBenchList[selected_benchmark].level,
                                      num_robots=collision_checker.num_robots, collisionChecker=collision_checker,
                                      benchmark=multiRobotBenchList[selected_benchmark], path=path)

    if save_path:
        print(f"💾 Speichere Animation nach {save_path}...")
        Writer = animation.writers['ffmpeg']
        writer = Writer(fps=fps, metadata=dict(artist='PRM'), bitrate=12000)

        # Fortschrittsanzeige beim Rendern
        with tqdm(total=1000, desc="🎞 Rendering Frames") as pbar:
            def progress_callback(current_frame, total_frames):
                pbar.update(1)
            ani.save(save_path, writer=writer, dpi=300,
                     progress_callback=progress_callback)
        print("✅ Video gespeichert unter:", save_path)
    else:
        display(HTML(ani.to_jshtml()))

    plt.close(fig)

def build_random_benchmarks(num_robots):
    multiRobotBenchList = []
    for i, scene_name in enumerate(scenes):
        scene = scenes[scene_name]
            
        limits2d = [[0, 27], [0, 27]] if scene_name == "scene5" else [[0, 24], [0, 24]] if scene_name == "scene3" else [[0, 22], [0, 20]]
        limits3d = limits2d + [[0, 360]]
        
        selected_robots = [random.choice(list(robots.items())) for _ in range(num_robots)]
        robot_names = [robot[0] for robot in selected_robots]
        robot_polygons = [robot[1] for robot in selected_robots]
        robot_dofs = [3 if name.startswith(("T_", "C_", "R_")) else 2 for name in robot_names] # e.g. [3, 3, 3] if all robots are 3dof
        
        limits_full = [limits3d if name.startswith(("T_", "C_", "R_")) else limits2d for name in robot_names]
        limits_flattened = [pair for sublist in limits_full for pair in sublist]
        
        # print("selected robots", robot_names, "in scene", scene_name, "with limits", limits_flattened)
        
        checker = MultiMobileRobotCollisionChecker(num_robots, robot_polygons, scene, robot_dofs, sum(robot_dofs), limits_flattened)
        valid_positions = generate_valid_positions(checker, 2*num_robots, limits2d[0], limits2d[1])
        
        start = [valid_positions[2*i] if dof == 3 else valid_positions[2*i][:2] for i, dof in enumerate(robot_dofs)]
        goal  = [valid_positions[2*i+1] if dof == 3 else valid_positions[2*i+1][:2] for i, dof in enumerate(robot_dofs)]
                                                                            
        label = f"{num_robots} Roboter ({robot_dofs}DOF) in {scene_name}"
        bench_name = f"{scene_name}"
        multiRobotBenchList.append(Benchmark(bench_name, checker, start, goal, label, robot_dofs))  
    return multiRobotBenchList

def get_predefined_benchmarks():
    multiRobotBenchList = []
    
    # Robots: L H U / T C R
    
    # Benchmarks 1 - 5: 2 Robots in each scene
    dofs = [2, 3]

    limits = [[0, 22], [0, 20], [0, 22], [0, 20], [0, 360]]
    robot_shapes = [robots["U_Robot"], robots["R_Robot"]]
    startList =[[2, 5], [18, 3, 180]]
    goalList = [[17, 11], [2, 10, 60]]

    checker = MultiMobileRobotCollisionChecker(num_robots=len(dofs), robot_shapes=robot_shapes, scene=scenes["scene1"], dofs=dofs, dim=sum(dofs), limits=limits)
    benchmark = Benchmark(name="Benchmark 1", collisionChecker=checker, startList=startList,
                            goalList=goalList, description="2 Roboter (2DoF und 3DoF) in Szene 1", level=dofs)
    multiRobotBenchList.append(benchmark)

    limits = [[0, 22], [0, 20], [0, 22], [0, 20], [0, 360]]
    robot_shapes = [robots["H_Robot"], robots["T_Robot"]]
    startList =[[1, 1], [2, 16, 90]]
    goalList = [[19, 10], [15, 12, 0]]

    checker = MultiMobileRobotCollisionChecker(num_robots=len(dofs), robot_shapes=robot_shapes, scene=scenes["scene2"], dofs=dofs, dim=sum(dofs), limits=limits)
    benchmark = Benchmark(name="Benchmark 2", collisionChecker=checker, startList=startList,
                            goalList=goalList, description="2 Roboter (2DoF und 3DoF) in Szene 2", level=dofs)
    multiRobotBenchList.append(benchmark)
    
    limits = [[0, 22], [0, 20], [0, 22], [0, 20], [0, 360]]
    robot_shapes = [robots["U_Robot"], robots["H_Robot"]]
    startList =[[3.1, 2], [6, 8, 0]]
    goalList = [[15.1, 14], [11.6, 10.1, 270]]

    checker = MultiMobileRobotCollisionChecker(num_robots=len(dofs), robot_shapes=robot_shapes, scene=scenes["scene3"], dofs=dofs, dim=sum(dofs), limits=limits)
    benchmark = Benchmark(name="Benchmark 3", collisionChecker=checker, startList=startList,
                            goalList=goalList, description="2 Roboter (2DoF und 3DoF) in Szene 3", level=dofs)
    multiRobotBenchList.append(benchmark)
    
    limits = [[0, 22], [0, 20], [0, 22], [0, 20], [0, 360]]
    robot_shapes = [robots["R_Robot"], robots["L_Robot"]]
    startList =[[7, 9], [1, 1, 0]]
    goalList = [[14.5, 15], [18, 8, 270]]

    checker = MultiMobileRobotCollisionChecker(num_robots=len(dofs), robot_shapes=robot_shapes, scene=scenes["scene4"], dofs=dofs, dim=sum(dofs), limits=limits)
    benchmark = Benchmark(name="Benchmark 4", collisionChecker=checker, startList=startList,
                            goalList=goalList, description="2 Roboter (2DoF und 3DoF) in Szene 4", level=dofs)
    multiRobotBenchList.append(benchmark)
    
    limits = [[0, 27], [0, 27], [0, 27], [0, 27], [0, 360]]
    robot_shapes = [robots["C_Robot"], robots["T_Robot"]]
    startList =[[3.5, 3.5], [8, 18, 70]]
    goalList = [[23, 23], [17.5, 9.3, 90]]

    checker = MultiMobileRobotCollisionChecker(num_robots=len(dofs), robot_shapes=robot_shapes, scene=scenes["scene5"], dofs=dofs, dim=sum(dofs), limits=limits)
    benchmark = Benchmark(name="Benchmark 5", collisionChecker=checker, startList=startList,
                            goalList=goalList, description="2 Roboter (2DoF und 3DoF) in Szene 5", level=dofs)
    multiRobotBenchList.append(benchmark)

    # Benchmarks 6 - 10: 3 Robots in each scene
    limits = [[0, 22], [0, 20], [0, 22], [0, 20], [0, 360], [0, 22], [0, 20], [0, 360]]
    dofs = [2, 3, 3]

    robot_shapes = [robots["U_Robot"], robots["R_Robot"], robots["C_Robot"]]
    startList =[[2, 5], [18, 3, 180], [9, 2, 180]]
    goalList = [[17, 11], [2, 10, 60], [9, 17, 270]]

    checker = MultiMobileRobotCollisionChecker(num_robots=len(dofs), robot_shapes=robot_shapes, scene=scenes["scene1"], dofs=dofs, dim=sum(dofs), limits=limits)
    benchmark = Benchmark(name="Benchmark 6", collisionChecker=checker, startList=startList,
                            goalList=goalList, description="3 Roboter (zwei 3DoF und ein 2DoF) in Szene 1", level=dofs)
    multiRobotBenchList.append(benchmark)
    
    limits = [[0, 22], [0, 20], [0, 22], [0, 20], [0, 360], [0, 22], [0, 20], [0, 360]]
    robot_shapes = [robots["H_Robot"], robots["T_Robot"], robots["U_Robot"]]
    startList =[[1, 1], [2, 16, 90], [17, 6, 120]]
    goalList = [[19, 10], [15, 12, 0], [6.5, 7.3, 120]]

    checker = MultiMobileRobotCollisionChecker(num_robots=len(dofs), robot_shapes=robot_shapes, scene=scenes["scene2"], dofs=dofs, dim=sum(dofs), limits=limits)
    benchmark = Benchmark(name="Benchmark 7", collisionChecker=checker, startList=startList,
                            goalList=goalList, description="3 Roboter (zwei 3DoF und ein 2DoF) in Szene 2", level=dofs)
    multiRobotBenchList.append(benchmark)
    
    limits = [[0, 22], [0, 20], [0, 22], [0, 20], [0, 360], [0, 22], [0, 20], [0, 360]]
    robot_shapes = [robots["U_Robot"], robots["H_Robot"], robots["L_Robot"]]
    startList =[[3.1, 2], [6, 8, 0], [7, 13, 0]]
    goalList = [[15.1, 14], [11.6, 10.1, 270], [15, 5, 70]]

    checker = MultiMobileRobotCollisionChecker(num_robots=len(dofs), robot_shapes=robot_shapes, scene=scenes["scene3"], dofs=dofs, dim=sum(dofs), limits=limits)
    benchmark = Benchmark(name="Benchmark 8", collisionChecker=checker, startList=startList,
                            goalList=goalList, description="3 Roboter (zwei 3DoF und ein 2DoF) in Szene 3", level=dofs)
    multiRobotBenchList.append(benchmark)
    
    limits = [[0, 22], [0, 20], [0, 22], [0, 20], [0, 360], [0, 22], [0, 20], [0, 360]]
    robot_shapes = [robots["R_Robot"], robots["L_Robot"], robots["H_Robot"]]
    startList =[[7, 9], [1, 1, 0], [18, 3, 0]]
    goalList = [[14.5, 15], [18, 8, 270], [8.5, 17, 180]]

    checker = MultiMobileRobotCollisionChecker(num_robots=len(dofs), robot_shapes=robot_shapes, scene=scenes["scene4"], dofs=dofs, dim=sum(dofs), limits=limits)
    benchmark = Benchmark(name="Benchmark 9", collisionChecker=checker, startList=startList,
                            goalList=goalList, description="3 Roboter (zwei 3DoF und ein 2DoF) in Szene 4", level=dofs)
    multiRobotBenchList.append(benchmark)
    
    limits = [[0, 27], [0, 27], [0, 27], [0, 27], [0, 360], [0, 27], [0, 27], [0, 360]]
    robot_shapes = [robots["C_Robot"], robots["T_Robot"], robots["U_Robot"]]
    startList =[[3.5, 3.5], [8, 18, 70], [10.2, 3.6, 0]]
    goalList = [[23, 23], [17.5, 9.3, 90], [15, 22.2, 180]]

    checker = MultiMobileRobotCollisionChecker(num_robots=len(dofs), robot_shapes=robot_shapes, scene=scenes["scene5"], dofs=dofs, dim=sum(dofs), limits=limits)
    benchmark = Benchmark(name="Benchmark 10", collisionChecker=checker, startList=startList,
                            goalList=goalList, description="3 Roboter (zwei 3DoF und ein 2DoF) in Szene 5", level=dofs)
    multiRobotBenchList.append(benchmark)
    
    return multiRobotBenchList

def visualize_multi_robot_benchmarks(multiRobotBenchList): 
    n = len(multiRobotBenchList)
    cols = 5
    rows = math.ceil(n / cols)

    fig, axes = plt.subplots(rows, cols, figsize=(cols * 5, rows * 5))
    axes = axes.flatten()

    for idx, benchmark in enumerate(multiRobotBenchList):
        ax = axes[idx]
        checker = benchmark.collisionChecker
        scene = checker.scene
        robot_shapes = checker.robot_shapes
        limits = checker.limits
        start = benchmark.startList
        goal = benchmark.goalList
        num_robots = len(checker.dofs)

        ax.set_xlim(0, limits[0][1] + 2)
        ax.set_ylim(0, limits[1][1] + 2)
        ax.set_aspect("equal")
        ax.set_title(f"{benchmark.name}", fontsize=9)

        # Hindernisse
        for obs in scene.values():
            if hasattr(obs, 'exterior'):
                x, y = obs.exterior.xy
                ax.fill(x, y, color="red", alpha=0.5)
        
        colors = mpl.colormaps["tab10"].colors
        for i in range(num_robots):
            
            # Startposition
            if len(start[i]) == 3:
                r_start = translate(rotate(robot_shapes[i], start[i][2], origin="centroid"), xoff=start[i][0], yoff=start[i][1])
            else:
                r_start = translate(robot_shapes[i], xoff=start[i][0], yoff=start[i][1])
                
            ax.fill(*r_start.exterior.xy, color=colors[i], alpha=0.7)
            ax.text(start[i][0], start[i][1], "Start", ha="center", va="center", fontsize=7)

            # Zielposition
            if len(goal[i]) == 3:
                r_goal = translate(rotate(robot_shapes[i], goal[i][2], origin="centroid"), xoff=goal[i][0], yoff=goal[i][1])
            else:
                r_goal = translate(robot_shapes[i], xoff=goal[i][0], yoff=goal[i][1])
            ax.fill(*r_goal.exterior.xy, color=colors[i], alpha=0.3)
            ax.text(goal[i][0], goal[i][1], "Ziel", ha="center", va="center", fontsize=7)

            # Verbindung Start-Ziel
            ax.plot(
                [start[i][0], goal[i][0]],
                [start[i][1], goal[i][1]],
                linestyle="--",
                color=colors[i],
                linewidth=0.8
            )

    # Leere Achsen entfernen
    for i in range(n, len(axes)):
        fig.delaxes(axes[i])

    plt.tight_layout()
    plt.show()

class MultiRobotPlannerRunner:
    def __init__(self, planner_class, config, name="Planner"):
        self.planner_class = planner_class
        self.config = config
        self.name = name        

    def run_benchmarks(self, bench_list, max_attempts=10,
                       fps=30, steps_per_segment=5, save_animation=False, animation_dir="./animations"):
        for idx, benchmark in enumerate(bench_list):
            collisionChecker = benchmark.collisionChecker
        
            # flatten start, goal lists, sum up dof list to create one large configuration space
            start = [number for coords in benchmark.startList for number in coords]
            goal = [number for coords in benchmark.goalList for number in coords]
            robot_dofs = benchmark.level # TODO maybe rename to dofs
            dof = sum(benchmark.level) # TODO maybe rename to dofs
            num_robots = collisionChecker.num_robots
            
            path_ids = []

            for attempt in tqdm(range(1, max_attempts + 1), desc=f"{self.name} Benchmark {idx+1}/{len(bench_list)}"):
                path_ids, graph = compute_prm_path(self.planner_class, collisionChecker, start, goal, self.config)
                if path_ids:
                    print(f"{self.name} Benchmark {idx}: Pfad gefunden nach {attempt} Versuchen.")
                    break
            if not path_ids:
                print(f"{self.name} Benchmark {idx}: Kein Pfad nach {max_attempts} Versuchen.")
                continue

            fig, ani = visualize_multi_robots(graph, path_ids, robot_dofs, num_robots, collisionChecker, benchmark, path=None)
    
            if save_animation:
                os.makedirs(animation_dir, exist_ok=True)
                file_name = f"{self.name}_MultiBenchmark_{idx}.mp4"
                save_path = os.path.join(animation_dir, file_name)
                print(f"Speichere Animation nach {save_path}...")
                writer = animation.FFMpegWriter(fps=fps)
                ani.save(save_path, writer=writer, dpi=200)
                print(f"Animation gespeichert: {save_path}")
            else:
                display(HTML(ani.to_jshtml()))

            plt.close(fig)