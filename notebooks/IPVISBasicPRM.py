# coding: utf-8

"""
This code is part of the course "Introduction to robot path planning" (Author: Bjoern Hein).

License is based on Creative Commons: Attribution-NonCommercial 4.0 International (CC BY-NC 4.0) (pls. check: http://creativecommons.org/licenses/by-nc/4.0/)
"""

import matplotlib.pyplot as plt
import networkx as nx
from matplotlib import animation
from IPython.display import HTML, display
import numpy as np


def basicPRMVisualize(planner, solution, ax = None, nodeSize = 100):
    """ Draw graph, obstacles and solution in a axis environment of matplotib.
    """
    # get a list of positions of all nodes by returning the content of the attribute 'pos'
    graph = planner.graph
    collChecker = planner._collisionChecker

    collChecker.drawObstacles(ax)
    
    pos = nx.get_node_attributes(graph,'pos')
    
    # draw graph (nodes colorized by degree)
    nx.draw_networkx_nodes(graph, pos,  cmap=plt.cm.Blues, ax = ax, node_size=nodeSize)
    nx.draw_networkx_edges(graph,pos,
                                ax = ax
                                 )
    Gcc = sorted(nx.connected_components(graph), key=len, reverse=True)
    G0=graph.subgraph(Gcc[0])# = largest connected component

    # how largest connected component
    nx.draw_networkx_edges(G0,pos,
                               edge_color='b',
                               width=3.0, ax = ax
                            )

    
    # draw nodes based on solution path
    Gsp = nx.subgraph(graph,solution)
    nx.draw_networkx_nodes(Gsp,pos,
                            node_size=nodeSize*1.5,
                             node_color='g',  ax = ax)
        
    # draw edges based on solution path
    nx.draw_networkx_edges(Gsp,pos,alpha=0.8,edge_color='g',width=10,  ax = ax)
        
    # draw start and goal
    if "start" in graph.nodes(): 
        nx.draw_networkx_nodes(graph,pos,nodelist=["start"],
                                   node_size=nodeSize*1.5,
                                   node_color='#00dd00',  ax = ax)
        nx.draw_networkx_labels(graph,pos,labels={"start": "S"},  ax = ax)


    if "goal" in graph.nodes():
        nx.draw_networkx_nodes(graph,pos,nodelist=["goal"],
                                   node_size=nodeSize*1.5,
                                   node_color='#DD0000',  ax = ax)
        nx.draw_networkx_labels(graph,pos,labels={"goal": "G"},  ax = ax)

def basicPRMVisualize3D(planner, solution, ax = None, nodeSize = 100):
    """ Draw graph, obstacles and solution in a axis environment of matplotib.
    """
    # get a list of positions of all nodes by returning the content of the attribute 'pos'
    graph = planner.graph
    collChecker = planner._collisionChecker

    collChecker.drawObstacles(ax)
    
    graph_positions = nx.get_node_attributes(graph,'pos')

    for node_index, node_position in graph_positions.items():
        ax.plot(node_position[0], node_position[1], node_position[2], 'bo')

        edges = graph.edges(node_index)

        for edge in edges:
            if(hash(edge[1]) > hash(edge[0])):
                continue

            edge_start = graph_positions[edge[0]]
            edge_end = graph_positions[edge[1]]

            ax.plot([edge_start[0], edge_end[0]], [edge_start[1], edge_end[1]],[edge_start[2], edge_end[2]], ':b')

    solution_graph = nx.subgraph(graph,solution)

    solution_graph_positions = nx.get_node_attributes(solution_graph,'pos')

    for node_index, node_position in solution_graph_positions.items():
        ax.plot(node_position[0], node_position[1], node_position[2], 'ro')

        edges = solution_graph.edges(node_index)

        for edge in edges:
            if(hash(edge[1]) > hash(edge[0])):
                continue

            edge_start = graph_positions[edge[0]]
            edge_end = graph_positions[edge[1]]

            ax.plot([edge_start[0], edge_end[0]], [edge_start[1], edge_end[1]],[edge_start[2], edge_end[2]], '-r')


def animatePRM3D(planner, solution, interpolated_path, title="3D Animation", interval=100):
    graph = planner.graph
    collChecker = planner._collisionChecker

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title(title)

    # --- Hindernisse zeichnen (angenommen, drawObstacles(ax) unterstützt 3D) ---
    if hasattr(collChecker, "drawObstacles"):
        collChecker.drawObstacles(ax)

    # --- Graph darstellen ---
    graph_positions = nx.get_node_attributes(graph, 'pos')
    for node_index, node_position in graph_positions.items():
        ax.plot([node_position[0]], [node_position[1]], [node_position[2]], 'bo', markersize=2)
        for edge in graph.edges(node_index):
            if hash(edge[1]) > hash(edge[0]):
                continue
            p1 = graph_positions[edge[0]]
            p2 = graph_positions[edge[1]]
            ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], ':b', linewidth=0.5)

    # --- Lösungspfad darstellen ---
    for i in range(len(solution) - 1):
        p1 = graph.nodes[solution[i]]["pos"]
        p2 = graph.nodes[solution[i + 1]]["pos"]
        ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], '-r', linewidth=2)

    # --- Roboter-Objekt: Kugel/Punkt/Marker ---
    robot_marker, = ax.plot([], [], [], 'ro', markersize=6)

    # --- Textanzeige ---
    coord_text = ax.text2D(0.05, 0.95, "", transform=ax.transAxes)

    # --- Limits setzen (z. B. über die Szenenlimits oder automatisch) ---
    all_pos = np.array(list(graph_positions.values()))
    ax.set_xlim(np.min(all_pos[:, 0]) - 5, np.max(all_pos[:, 0]) + 5)
    ax.set_ylim(np.min(all_pos[:, 1]) - 5, np.max(all_pos[:, 1]) + 5)
    ax.set_zlim(np.min(all_pos[:, 2]) - 5, np.max(all_pos[:, 2]) + 5)

    def init():
        robot_marker.set_data([], [])
        robot_marker.set_3d_properties([])
        coord_text.set_text("")
        return robot_marker, coord_text

    def animate(i):
        i = min(i, len(interpolated_path) - 1)
        pos = interpolated_path[i]
        robot_marker.set_data([pos[0]], [pos[1]])
        robot_marker.set_3d_properties([pos[2]])
        coord_text.set_text(f"x = {pos[0]:.2f}, y = {pos[1]:.2f}, θ = {pos[2]:.2f}")
        return robot_marker, coord_text

    ani = animation.FuncAnimation(
        fig, animate, init_func=init, frames=len(interpolated_path),
        interval=interval, blit=True
    )
    display(HTML(ani.to_jshtml()))
    plt.close(fig)
