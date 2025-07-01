from IPBenchmark import Benchmark
from shapely.geometry import box, Point, LineString, Polygon
from IPMobileRobotCollisionChecker import MobileRobotCollisionChecker

# Roboterdefinitionen
robots = {
    "L_Robot": Polygon([(0, 0), (0, 1), (1, 1.5), (1, 1), (1.5, 1), (0, 0)]),
"H_Robot": Polygon([
    (1.6, 0.0), (1.7, 1.1),       # rechte Körperkante
    (2.2, 1.1), (2.2, 1.5),       # rechter Arm
    (1.7, 1.5), (1.6, 2.0),       # rechte Schulter
    (1.2, 2.0), (1.2, 1.5),       # linke Schulter
    (0.6, 1.5), (0.6, 1.3),       # linker Arm
    (1.2, 1.3), (1.3, 0.0),       # linke Körperkante
    (1.6, 0.0)                    # zurück zum Start
]),



    "U_Robot": Polygon([
        (0, 0), (0, 2), (0.5, 2), (0.5, 0.5),
        (1.5, 0.5), (1.5, 3), (2, 3), (2, 0)

        
    ]),
    "T_Robot": Polygon([
        (0, 0), (0, 1), (0.4, 1), (0.4, 1.6), (1.6, 1.6),
        (1.6, 1), (2, 1), (2, 0), (1.6, 0), (1.6, 0.6),
        (0.4, 0.6), (0.4, 0)
    ]),
    "C_Robot": Polygon([
        (0.5, 0), (0.5, 0.6), (0.1, 0.6), (0.1, 1.4),
        (0.6, 1.4), (0.6, 2), (1.8, 2), (1.8, 1.6),
        (1.2, 1.6), (1.2, 1.2), (2, 1.2), (2, 0.8),
        (1.2, 0.8), (1.2, 0.4), (1.8, 0.4), (1.8, 0), (0.5, 0)
    ]),
    "R_Robot": Polygon([
        (0, 2), (3, 2), (3, 1.4), (1.8, 1.4),
        (1.8, 0.8), (2.5, 0.8), (2.5, 0), (0.5, 0),
        (0.5, 0.8), (1.2, 0.8), (1.2, 1.4), (0, 1.4), (0, 2)
    ])
}

# Umgebungsdefinitionen
scenes = {
    "scene1": {"wall1": box(6, 0, 8, 16), "wall3": box(12, 0, 14, 16)},
    "scene2": {
        "block1": box(5, 3, 7, 5),
        "block2": box(9, 2, 11, 4),
        "block3": box(13, 10, 15, 20),
        "block4": box(17, 2, 19, 4),
        "arc1": Point(7, 7).buffer(2).difference(Point(5, 5).buffer(1.5)),
        "line_obstacle": LineString([(15, 3.5), (18, 5)]).buffer(0.4)
    },
    "scene3": {f"diag{i}": box(i, i, i + 0.8, i + 0.8) for i in range(4, 18, 2)},
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


# Szene 1
start1 = [[1, 1]]
goal1 = [[16, 1]]
start1_3d = [[1, 1, 0]]
goal1_3d = [[16, 1, 180]]

# Szene 2
start2 = [[1, 1]]
goal2 = [[16, 14]]
start2_3d = [[1, 1, 0]]
goal2_3d = [[16, 14, 300]]

# Szene 3
start3 = [[2.7, 0.5]]
goal3 = [[18, 18]]
start3_3d = [[2.7, 0.5, 0]]
goal3_3d = [[18, 18, 100]]

# Szene 4
start4 = [[1, 1]]
goal4 = [[16, 16]]
start4_3d = [[1, 1, 0]]
goal4_3d = [[16, 16, 70]]

# Szene 5 (verschoben)
start5 = [[3, 3]]
goal5 = [[23, 23]]
start5_3d = [[3, 3, 0]]
goal5_3d = [[23, 23, 245]]

benchList = []

# Szene 1
benchList.append(Benchmark("2DOF_L_scene1", MobileRobotCollisionChecker(robots["L_Robot"], scenes["scene1"], [[0, 22], [0, 20]]), start1, goal1, "L-Roboter (2DOF) in scene1", 2))
benchList.append(Benchmark("2DOF_H_scene1", MobileRobotCollisionChecker(robots["H_Robot"], scenes["scene1"], [[0, 22], [0, 20]]), start1, goal1, "H-Roboter (2DOF) in scene1", 2))
benchList.append(Benchmark("2DOF_U_scene1", MobileRobotCollisionChecker(robots["U_Robot"], scenes["scene1"], [[0, 22], [0, 20]]), start1, goal1, "U-Roboter (2DOF) in scene1", 2))
benchList.append(Benchmark("3DOF_T_scene1", MobileRobotCollisionChecker(robots["T_Robot"], scenes["scene1"], [[0, 22], [0, 20], [0, 360]]), start1_3d, goal1_3d, "T-Roboter (3DOF) in scene1", 3))
benchList.append(Benchmark("3DOF_C_scene1", MobileRobotCollisionChecker(robots["C_Robot"], scenes["scene1"], [[0, 22], [0, 20], [0, 360]]), start1_3d, goal1_3d, "C-Roboter (3DOF) in scene1", 3))
benchList.append(Benchmark("3DOF_R_scene1", MobileRobotCollisionChecker(robots["R_Robot"], scenes["scene1"], [[0, 22], [0, 20], [0, 360]]), start1_3d, goal1_3d, "R-Roboter (3DOF) in scene1", 3))

# Szene 2
benchList.append(Benchmark("2DOF_L_scene2", MobileRobotCollisionChecker(robots["L_Robot"], scenes["scene2"], [[0, 22], [0, 20]]), start2, goal2, "L-Roboter (2DOF) in scene2", 2))
benchList.append(Benchmark("2DOF_H_scene2", MobileRobotCollisionChecker(robots["H_Robot"], scenes["scene2"], [[0, 22], [0, 20]]), start2, goal2, "H-Roboter (2DOF) in scene2", 2))
benchList.append(Benchmark("2DOF_U_scene2", MobileRobotCollisionChecker(robots["U_Robot"], scenes["scene2"], [[0, 22], [0, 20]]), start2, goal2, "U-Roboter (2DOF) in scene2", 2))
benchList.append(Benchmark("3DOF_T_scene2", MobileRobotCollisionChecker(robots["T_Robot"], scenes["scene2"], [[0, 22], [0, 20], [0, 360]]), start2_3d, goal2_3d, "T-Roboter (3DOF) in scene2", 3))
benchList.append(Benchmark("3DOF_C_scene2", MobileRobotCollisionChecker(robots["C_Robot"], scenes["scene2"], [[0, 22], [0, 20], [0, 360]]), start2_3d, goal2_3d, "C-Roboter (3DOF) in scene2", 3))
benchList.append(Benchmark("3DOF_R_scene2", MobileRobotCollisionChecker(robots["R_Robot"], scenes["scene2"], [[0, 22], [0, 20], [0, 360]]), start2_3d, goal2_3d, "R-Roboter (3DOF) in scene2", 3))

# Szene 3
benchList.append(Benchmark("2DOF_L_scene3", MobileRobotCollisionChecker(robots["L_Robot"], scenes["scene3"], [[0, 24], [0, 24]]), start3, goal3, "L-Roboter (2DOF) in scene3", 2))
benchList.append(Benchmark("2DOF_H_scene3", MobileRobotCollisionChecker(robots["H_Robot"], scenes["scene3"], [[0, 24], [0, 24]]), start3, goal3, "H-Roboter (2DOF) in scene3", 2))
benchList.append(Benchmark("2DOF_U_scene3", MobileRobotCollisionChecker(robots["U_Robot"], scenes["scene3"], [[0, 24], [0, 24]]), start3, goal3, "U-Roboter (2DOF) in scene3", 2))
benchList.append(Benchmark("3DOF_T_scene3", MobileRobotCollisionChecker(robots["T_Robot"], scenes["scene3"], [[0, 24], [0, 24], [0, 360]]), start3_3d, goal3_3d, "T-Roboter (3DOF) in scene3", 3))
benchList.append(Benchmark("3DOF_C_scene3", MobileRobotCollisionChecker(robots["C_Robot"], scenes["scene3"], [[0, 24], [0, 24], [0, 360]]), start3_3d, goal3_3d, "C-Roboter (3DOF) in scene3", 3))
benchList.append(Benchmark("3DOF_R_scene3", MobileRobotCollisionChecker(robots["R_Robot"], scenes["scene3"], [[0, 24], [0, 24], [0, 360]]), start3_3d, goal3_3d, "R-Roboter (3DOF) in scene3", 3))

# Szene 4
benchList.append(Benchmark("2DOF_L_scene4", MobileRobotCollisionChecker(robots["L_Robot"], scenes["scene4"], [[0, 20], [0, 20]]), start4, goal4, "L-Roboter (2DOF) in scene4", 2))
benchList.append(Benchmark("2DOF_H_scene4", MobileRobotCollisionChecker(robots["H_Robot"], scenes["scene4"], [[0, 20], [0, 20]]), start4, goal4, "H-Roboter (2DOF) in scene4", 2))
benchList.append(Benchmark("2DOF_U_scene4", MobileRobotCollisionChecker(robots["U_Robot"], scenes["scene4"], [[0, 20], [0, 20]]), start4, goal4, "U-Roboter (2DOF) in scene4", 2))
benchList.append(Benchmark("3DOF_T_scene4", MobileRobotCollisionChecker(robots["T_Robot"], scenes["scene4"], [[0, 20], [0, 20], [0, 360]]), start4_3d, goal4_3d, "T-Roboter (3DOF) in scene4", 3))
benchList.append(Benchmark("3DOF_C_scene4", MobileRobotCollisionChecker(robots["C_Robot"], scenes["scene4"], [[0, 20], [0, 20], [0, 360]]), start4_3d, goal4_3d, "C-Roboter (3DOF) in scene4", 3))
benchList.append(Benchmark("3DOF_R_scene4", MobileRobotCollisionChecker(robots["R_Robot"], scenes["scene4"], [[0, 20], [0, 20], [0, 360]]), start4_3d, goal4_3d, "R-Roboter (3DOF) in scene4", 3))

# Szene 5
benchList.append(Benchmark("2DOF_L_scene5", MobileRobotCollisionChecker(robots["L_Robot"], scenes["scene5"], [[2, 27], [2, 27]]), start5, goal5, "L-Roboter (2DOF) in scene5", 2))
benchList.append(Benchmark("2DOF_H_scene5", MobileRobotCollisionChecker(robots["H_Robot"], scenes["scene5"], [[2, 27], [2, 27]]), start5, goal5, "H-Roboter (2DOF) in scene5", 2))
benchList.append(Benchmark("2DOF_U_scene5", MobileRobotCollisionChecker(robots["U_Robot"], scenes["scene5"], [[2, 27], [2, 27]]), start5, goal5, "U-Roboter (2DOF) in scene5", 2))
benchList.append(Benchmark("3DOF_T_scene5", MobileRobotCollisionChecker(robots["T_Robot"], scenes["scene5"], [[2, 27], [2, 27], [0, 360]]), start5_3d, goal5_3d, "T-Roboter (3DOF) in scene5", 3))
benchList.append(Benchmark("3DOF_C_scene5", MobileRobotCollisionChecker(robots["C_Robot"], scenes["scene5"], [[2, 27], [2, 27], [0, 360]]), start5_3d, goal5_3d, "C-Roboter (3DOF) in scene5", 3))
benchList.append(Benchmark("3DOF_R_scene5", MobileRobotCollisionChecker(robots["R_Robot"], scenes["scene5"], [[2, 27], [2, 27], [0, 360]]), start5_3d, goal5_3d, "R-Roboter (3DOF) in scene5", 3))

