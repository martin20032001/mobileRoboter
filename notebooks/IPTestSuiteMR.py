from IPBenchmark import Benchmark
from shapely.geometry import box, Polygon, Point, LineString
from shapely.affinity import rotate
from IPMobileRobotCollisionChecker import MobileRobotCollisionChecker

benchList = []

# --- Benchmark 1: 2DOF, Roboter ---
L_robot = Polygon([(0, 0), (0, 1), (1, 1.5), (1, 1), (1.5, 1), (0, 0)])
scene1 = {
    "wall1": box(3, 0, 4, 10),
    #"wall2": box(6, 2, 7, 15),
    "wall3": box(9, 0, 10, 10)
}
checker1 = MobileRobotCollisionChecker(L_robot, scene1, limits=[[0, 15], [0, 12]])
benchList.append(Benchmark(
    "2DOF_L_Labyrinth",
    checker1,
    [[1, 1]],
    [[13, 9]],
    "Roboter durch enge Passagen",
    level=2
))

# --- Benchmark 2: 2DOF, asymmetrischer H-förmiger Roboter ---
H_robot = Polygon([(1,1), (0,3), (1,3), (1.5,2), (2.5,2), (2.5,3), (3.5,3), (4,0), (2.5,0), (2.5,1), (1.5,1), (1,0)])
scene2 = {
    "block1": box(5, 3, 7, 5),
    "block2": box(9, 2, 11, 4),
    "block3": box(13, 10, 15, 15),
    "block4": box(17, 2, 19, 4),
    
    "arc1": Point(7, 7).buffer(2).difference(Point(5, 5).buffer(1.5)),
    
    "line_obstacle": LineString([(15, 3.5), (18, 5)]).buffer(0.4)
}
checker2 = MobileRobotCollisionChecker(H_robot, scene2, limits=[[0, 26], [0, 14]])
benchList.append(Benchmark(
    "2DOF_H_BlockField",
    checker2,
    [[0, 0]],
    [[21, 5]],
    "Asymmetrischer H-Roboter durch versetzte Hindernissstrukturen",
    level=2
))




# --- Benchmark 3: 2DOF, U-förmiger Roboter durch diagonales Feld ---
U_robot = Polygon([(0, 0), (0, 2), (0.5, 2), (0.5, 0.5), (1.5, 0.5), (1.5, 2), (2, 2), (2, 0)])
scene3 = {
    f"diag{i}": box(i, i, i+0.8, i+0.8) for i in range(4, 18, 2)
}
checker3 = MobileRobotCollisionChecker(U_robot, scene3, limits=[[0, 20], [0, 20]])
benchList.append(Benchmark(
    "2DOF_U_Diagonal",
    checker3,
    [[3.4, 3]],
    [[15.4, 15]],
    "U-Roboter durch diagonales Minenfeld",
    level=2
))



# --- Benchmark 4: 3DOF, L-förmiger Roboter durch 90°-Kurve ---
scene4 = {
    #"left": box(4, 0, 5, 12),
    #"top": box(4, 12, 12, 13),
    #"right": box(11, 0, 12, 12)
}
checker4 = MobileRobotCollisionChecker(L_robot, scene4, limits=[[0, 20], [0, 20], [0, 360]])
benchList.append(Benchmark(
    "3DOF_L_Bend",
    checker4,
    [[2, 1, 0]],
    [[18, 1, 90]],
    "L-Roboter muss durch 90°-Kurve manövrieren",
    level=3
))

# --- Benchmark 5: 3DOF, T-förmiger Roboter durch asymmetrisches Labyrinth ---


U_robot = Polygon([
    (0, 0), (0, 2), (0.5, 2), (0.5, 0.5),
    (1.5, 0.5), (1.5, 2), (2, 2), (2, 0)
])
scene5 = {
    f"diag{i}": box(i, i, i + 0.8, i + 0.8)
    for i in range(4, 18, 2)
}
checker5 = MobileRobotCollisionChecker(
    U_robot,
    scene5,
    limits=[[0, 22], [0, 22], [0, 360]]  # → 3DOF: x, y, theta
)
benchList.append(Benchmark(
    "3DOF_U_Diagonal",
    checker5,
    [[3.4, 3.0, 0]],         # Start: 0 Grad
    [[15.4, 16.0, 180]],     # Ziel: 180 Grad Rotation
    "U-Roboter (3DOF) durch diagonales Minenfeld mit 180° Zielrotation",
    level=2
))