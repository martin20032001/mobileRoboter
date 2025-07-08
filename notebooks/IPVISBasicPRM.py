# coding: utf-8

"""
This code is part of the course "Introduction to robot path planning" (Author: Bjoern Hein).

License is based on Creative Commons: Attribution-NonCommercial 4.0 International (CC BY-NC 4.0) (pls. check: http://creativecommons.org/licenses/by-nc/4.0/)
"""

import matplotlib.pyplot as plt
import networkx as nx


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

def visualize_nodes(ax, node_positions, color):
    for node_index, node_position in node_positions.items():
        ax.plot(node_position[0], node_position[1], node_position[2], color)

def visualize_edges(ax, node_positions, edges, color):
    for edge in edges:
        node_position_start = node_positions[edge[0]]
        node_position_end = node_positions[edge[1]]

        ax.plot([node_position_start[0], node_position_end[0]], [node_position_start[1], node_position_end[1]],[node_position_start[2], node_position_end[2]], color)



def basicPRMVisualize3D(planner, solution, ax):
    """ Draw graph, obstacles and solution in a axis environment of matplotib.
    """
    # get a list of positions of all nodes by returning the content of the attribute 'pos'
    graph = planner.graph
    
    graph_positions = nx.get_node_attributes(graph,'pos')
    graph_edges = graph.edges()

    visualize_nodes(ax, graph_positions, 'bo')
    visualize_edges(ax, graph_positions, graph_edges, 'b--')

    solution_graph = nx.subgraph(graph,solution)

    solution_positions = nx.get_node_attributes(solution_graph,'pos')
    solution_edges = solution_graph.edges()

    visualize_nodes(ax, solution_positions, 'ro')
    visualize_edges(ax, solution_positions, solution_edges, 'r-')