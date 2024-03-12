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
 * @brief Implements Prim's algorithm to find the Minimum Spanning Tree (MST) of
 *        a weighted, connected, and undirected graph.
 *
 * Prim's algorithm is a greedy algorithm that finds the MST of a weighted,
 * connected, and undirected graph. It starts with an arbitrary vertex and
 * iteratively adds the edge with the minimum weight that connects a vertex in
 * the MST to a vertex not in the MST, until all vertices are included in the
 * MST.
 *
 * Time Complexity: O((V + E) * log V), where V is the number of vertices and
 *                  E is the number of edges in the graph.
 * Space Complexity: O(V + E), where V is the number of vertices and E is the
 *                  number of edges in the graph.
 *
 * @param graph The weighted graph represented as an adjacency list.
 * @param mst The vector to store the edges of the Minimum Spanning Tree.
 */
void prim(std::vector<std::vector<std::pair<int, int>>>& graph,
          std::vector<std::pair<int, std::pair<int, int>>>& mst) {
    int n = graph.size();          // Number of vertices in the graph.
    std::vector<int> key(n, INF);  // Initialize all keys to infinity (max).
    std::vector<int> parent(n,
                            -1);  // Initialize all parents to -1 (no parent).
    std::vector<bool> inMST(n,
                            false);  // Initialize all vertices as not in MST.

    key[0] = 0;  // Start with vertex 0 and key 0 to begin the MST.

    // Priority queue to store vertices and their keys (weights).
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>,
                        std::greater<std::pair<int, int>>>
        pq;
    pq.emplace(0, 0);  // Insert the source vertex with key 0.

    while (!pq.empty()) {         // While there are vertices to process.
        int u = pq.top().second;  // Get the vertex with the minimum key.
        pq.pop();                 // Remove the vertex from the priority queue.

        inMST[u] = true;  // Mark the extracted vertex as part of the MST.

        // Iterative over the neighbors of the extracted vertex.
        for (const auto& neighbor : graph[u]) {
            int v = neighbor.first;        // Neighbor vertex.
            int weight = neighbor.second;  // Weight of the edge (u, v).

            // If the neighbor is not in the MST and the weight is smaller than
            // the current key, update the key and parent.
            if (!inMST[v] && weight < key[v]) {
                key[v] = weight;
                parent[v] = u;
                pq.emplace(key[v], v);  // Insert the updated vertex.
            }
        }
    }

    // Construct the MST from the parent array and the edges.
    for (int i = 1; i < n; i++) {
        mst.emplace_back(key[i], std::make_pair(i, parent[i]));
    }
}

int main() {
    int n, m;

    std::cout << "Enter the number of vertices: ";
    std::cin >> n;

    std::cout << "Enter the number of edges: ";
    std::cin >> m;

    std::vector<std::vector<std::pair<int, int>>> graph(n);
    std::vector<std::pair<int, std::pair<int, int>>>
        edges;  // Vector to store the edges of the MST.

    std::cout << "Enter the edges (u v weight):\n";
    for (int i = 0; i < m; i++) {
        int u, v, weight;
        std::cin >> u >> v >> weight;
        graph[u].emplace_back(v, weight);
        graph[v].emplace_back(u, weight);
    }

    prim(graph, edges);

    std::cout << "Minimum Spanning Tree:\n";
    for (const auto& edge : edges) {
        std::cout << "(" << edge.second.first << ", " << edge.second.second
                  << ") : " << edge.first << "\n";
    }

    return 0;
}