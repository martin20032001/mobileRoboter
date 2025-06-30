# coding: utf-8

"""
This code is part of the course "Introduction to robot path planning" (Author: Bjoern Hein).

License is based on Creative Commons: Attribution-NonCommercial 4.0 International (CC BY-NC 4.0) (pls. check: http://creativecommons.org/licenses/by-nc/4.0/)
"""

import matplotlib.pyplot as plt
import networkx as nx

def rrtPRMVisualize(planner, solution, ax = None, nodeSize = 300):
    """ Draw graph, obstacles and solution in a axis environment of matplotib.
    """
    graph = planner.graph
    collChecker = planner._collisionChecker
    pos = nx.get_node_attributes(graph,'pos')
    # draw graph
    
    collChecker.drawObstacles(ax)
    nx.draw_networkx_nodes(graph, pos, ax = ax, cmap=plt.cm.Blues, node_size=nodeSize)
    nx.draw_networkx_edges(graph, pos, ax = ax)
     

    
    
    # draw nodes based on solution path
    Gsp = nx.subgraph(graph,solution)
    nx.draw_networkx_nodes(Gsp,pos,
                            node_size=nodeSize,
                             node_color='g',  ax = ax)
        
    # draw edges based on solution path
    nx.draw_networkx_edges(Gsp,pos,alpha=0.8,edge_color='g',width=3.0,  ax = ax)
        
    # draw start and goal
    if "start" in graph.nodes(): 
        nx.draw_networkx_nodes(graph,pos,nodelist=["start"],
                                   node_size=nodeSize,
                                   node_color='#00dd00',  ax = ax)
        nx.draw_networkx_labels(graph,pos,labels={"start": "S"},  ax = ax)


    if "goal" in graph.nodes():
        nx.draw_networkx_nodes(graph,pos,nodelist=["goal"],
                                   node_size=nodeSize,
                                   node_color='#DD0000',  ax = ax)
        nx.draw_networkx_labels(graph,pos,labels={"goal": "G"},  ax = ax)

def rrtPRMVisualize(planner, solution, ax = None, nodeSize = 300):
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

