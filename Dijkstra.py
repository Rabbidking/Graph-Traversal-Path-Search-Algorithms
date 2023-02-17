import heapq
import networkx as nx
import matplotlib.pyplot as plt


def Dijkstra(graph, start, end):
    # initialize distances and previous nodes
    distances = {node: float('inf') for node in graph}
    distances[start] = 0
    previous_nodes = {node: None for node in graph}

    # create a priority queue and add the start node
    priority_queue = [(0, start)]

    while len(priority_queue) > 0:
        # get the node with the smallest distance from the start node
        current_distance, current_node = heapq.heappop(priority_queue)

        # if we have found the end node, we can stop searching
        if current_node == end:
            break

        # update the distances and previous nodes of neighboring nodes
        for neighbor, weight in graph[current_node].items():
            distance = current_distance + weight
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous_nodes[neighbor] = current_node
                heapq.heappush(priority_queue, (distance, neighbor))

    # reconstruct the shortest path
    path = []
    node = end
    while node is not None:
        path.append(node)
        node = previous_nodes[node]
    path.reverse()

    return path, distances[end]


# define the graph
graph = {
    'Arad': {'Zerind': 75, 'Sibiu': 140, 'Timisoara': 118},
    'Zerind': {'Arad': 75, 'Oradea': 71},
    'Oradea': {'Zerind': 71, 'Sibiu': 151},
    'Sibiu': {'Arad': 140, 'Oradea': 151, 'Fagaras': 99, 'Rimnicu Vilcea': 80},
    'Timisoara': {'Arad': 118, 'Lugoj': 111},
    'Lugoj': {'Timisoara': 111, 'Mehadia': 70},
    'Mehadia': {'Lugoj': 70, 'Drobeta': 75},
    'Drobeta': {'Mehadia': 75, 'Craiova': 120},
    'Craiova': {'Drobeta': 120, 'Rimnicu Vilcea': 146, 'Pitesti': 138},
    'Rimnicu Vilcea': {'Sibiu': 80, 'Craiova': 146, 'Pitesti': 97},
    'Fagaras': {'Sibiu': 99, 'Bucharest': 211},
    'Pitesti': {'Rimnicu Vilcea': 97, 'Craiova': 138, 'Bucharest': 101},
    'Bucharest': {'Fagaras': 211, 'Pitesti': 101, 'Giurgiu': 90, 'Urziceni': 85},
    'Giurgiu': {'Bucharest': 90},
    'Urziceni': {'Bucharest': 85, 'Hirsova': 98, 'Vaslui': 142},
    'Hirsova': {'Urziceni': 98, 'Eforie': 86},
    'Eforie': {'Hirsova': 86},
    'Vaslui': {'Urziceni': 142, 'Iasi': 92},
    'Iasi': {'Vaslui': 92, 'Neamt': 87},
    'Neamt': {'Iasi': 87}
}

# define the start and end nodes
start_node = 'Arad'
end_node = 'Bucharest'

# run Dijkstra's algorithm to find the shortest path and distance
shortest_path, distance = Dijkstra(graph, start_node, end_node)
print(f'The shortest path from {start_node} to {end_node} is {shortest_path} with a distance of {distance} km.')

# create a NetworkX graph object
G = nx.Graph(graph)

# create a dictionary of node colors for the visual component
node_colors = {node: 'white' for node in graph}
node_colors[start_node] = 'green'
node_colors[end_node] = 'red'

# create a dictionary of edge colors for the visual component
edge_colors = {edge: 'black' for edge in G.edges()}

# Draw graph with shortest path
pos = nx.spring_layout(G)
nx.draw_networkx_nodes(G, pos, node_size=500)
nx.draw_networkx_edges(G, pos, width=1, edge_color='gray')
nx.draw_networkx_labels(G, pos, font_size=12, font_family='sans-serif')
edge_labels = nx.get_edge_attributes(G, 'weight')
nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_size=10, font_family='sans-serif')
path_edges = list(zip(shortest_path, shortest_path[1:]))
nx.draw_networkx_nodes(G, pos, nodelist=shortest_path, node_color='r', node_size=500)
nx.draw_networkx_edges(G, pos, edgelist=path_edges, edge_color='r', width=2)
plt.axis('off')
plt.show()
