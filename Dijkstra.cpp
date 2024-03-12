//
// Copyright Caiden Sanders - All Rights Reserved
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// Proprietary and confidential.
//
// Written by Caiden Sanders <work.caidensanders@gmail.com>, March 11, 2024.
//

#include <iostream>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

const int INF = std::numeric_limits<int>::max();

/**
 * @brief Implements Dijkstra's algorithm to find the shortest paths from a
 *        source vertex to all other vertices in a graph.
 *
 * Dijkstra's algorithm is a greedy algorithm that works on weighted graphs.
 * It finds the shortest path from a source vertex to all other vertices in the
 * graph by iteratively selecting the vertex with the minimum distance and
 * updating the distance of its neighbors if a shorter path is found.
 *
 * Time Complexity: O((V + E) * log V), where V is the number of vertices and
 *                  E is the number of edges in the graph.
 * Space Complexity: O(V + E) where V isthe number of vertices and E is the
 *                  number of edges in the graph.
 *
 * @param graph The weighted graph represented as an adjacency list.
 * @param source The source vertex from which to find the shortest paths.
 * @param distance A reference to the vector that will store the shortest
 *                 distance from the source vertex to all other vertices.
 * @param previous A reference to a vector that will store the previous vertex
 *                 in the shortest path from the source vertex to all other
 *                 vertices.
 */
void dijkstra(std::vector<std::vector<std::pair<int, int>>>& graph, int source,
              std::vector<int>& distance, std::vector<int>& previous) {
    int n = graph.size();     // Number of vertices in the graph.
    distance.assign(n, INF);  // Initialize all distances to infinity (max).
    previous.assign(
        n, -1);  // Initialize previous vertex to -1 (no previous vertex).
    distance[source] = 0;  // Distance from source to itself is 0.

    // Priority queue to store vertices and their distances from the source.
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>,
                        std::greater<std::pair<int, int>>>
        pq;
    pq.emplace(0, source);  // Insert the source vertex with distance 0.

    while (!pq.empty()) {         // While there are vertices to process.
        int u = pq.top().second;  // Get the vertex with the minimum distance.
        pq.pop();                 // Remove the vertex from the priority queue.

        // Iterative over the neighbors of the extracted vertex.
        for (const auto& neighbor : graph[u]) {
            int v = neighbor.first;        // Neighbor vertex.
            int weight = neighbor.second;  // Weight of the edge (u, v).

            // If a shorter path to v is found, updated the distance and
            // previous vertex.
            if (distance[v] > distance[u] + weight) {
                distance[v] = distance[u] + weight;
                previous[v] = u;
                pq.emplace(distance[v], v);  // Insert the updated distance.
            }
        }
    }
}

int main() {
    int n, m, source;

    std::cout << "Enter the number of vertices: ";
    std::cin >> n;

    std::cout << "Enter the number of edges: ";
    std::cin >> m;

    std::cout << "Enter the source vertex: ";
    std::cin >> source;

    std::vector<std::vector<std::pair<int, int>>> graph(n);

    std::cout << "Enter the edges (u v weight):\n";
    for (int i = 0; i < m; i++) {
        int u, v, weight;
        std::cin >> u >> v >> weight;
        graph[u].emplace_back(v, weight);
        graph[v].emplace_back(u, weight);
    }

    std::vector<int> distance, previous;
    dijkstra(graph, source, distance, previous);

    std::cout << "Shortest distance from source " << source << ":\n";
    for (int i = 0; i < n; i++) {
        std::cout << "Vertex " << i << ": "
                  << (distance[i] == INF ? -1 : distance[i]) << "\n";
    }

    return 0;
}