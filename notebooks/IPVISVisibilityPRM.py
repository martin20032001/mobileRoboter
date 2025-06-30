# coding: utf-8

"""
This code is part of the course 'Innovative Programmiermethoden für Industrieroboter' (Author: Bjoern Hein). It is based on the slides given during the course, so please **read the information in theses slides first**

License is based on Creative Commons: Attribution-NonCommercial 4.0 International (CC BY-NC 4.0) (pls. check: http://creativecommons.org/licenses/by-nc/4.0/)
"""

import networkx as nx

import networkx as nx
def visibilityPRMVisualize(planner, solution, ax = None, nodeSize = 300):
    # get a list of positions of all nodes by returning the content of the attribute 'pos'
    graph = planner.graph
    statsHandler = planner.statsHandler
    collChecker = planner._collisionChecker
    pos = nx.get_node_attributes(graph,'pos')
    color = nx.get_node_attributes(graph,'color')

    collChecker.drawObstacles(ax)
    
    if statsHandler:
        statPos = nx.get_node_attributes(statsHandler.graph,'pos')
        nx.draw_networkx_nodes(statsHandler.graph, pos=statPos, alpha=0.2,node_size=nodeSize)
        nx.draw_networkx_edges(statsHandler.graph, pos=statPos, alpha=0.2,edge_color='y')
        
    # draw graph 
    nx.draw_networkx_nodes(graph, pos, ax = ax, nodelist=list(color.keys()), node_color=list(color.values()), node_size=nodeSize)
    nx.draw_networkx_edges(graph, pos, ax = ax)
    
    Gcc = sorted(nx.connected_components(graph), key=len, reverse=True)
    G0=graph.subgraph(Gcc[0])# = largest connected component

    # how largest connected component
    nx.draw_networkx_edges(G0,pos,
                               edge_color='b',
                               width=2.0, ax = ax
                            )
    
    # get nodes based on solution path
    Gsp = nx.subgraph(graph,solution)

    # draw edges based on solution path
    nx.draw_networkx_edges(Gsp,pos,alpha=0.8,edge_color='g',width=5.0, label="Solution Path")
        
    # draw start and goal
    # draw start and goal
    if "start" in graph.nodes(): 
        nx.draw_networkx_nodes(graph,pos,nodelist=["start"],
                                   node_size=nodeSize,
                                   node_color='#00dd00',  ax = ax)
        nx.draw_networkx_labels(graph,pos,labels={"start": "S"},  ax = ax)
    if "goal" in graph.nodes():
        nx.draw_networkx_nodes(graph,pos,nodelist=["goal"],
                                   node_size=nodeSize,
                                   node_color='#dd0000',  ax = ax)
        nx.draw_networkx_labels(graph,pos,labels={"goal": "G"},  ax = ax)

def visibilityPRMVisualize3D(planner, solution, ax = None, nodeSize = 100):
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