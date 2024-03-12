function dijkstra(graph, source) {
    dist[source] = 0

    for each vertex v in graph {
        if V != source {
            dist[v] = Infinity
        }

        prev[v] = undefined
    }

    Q = graph.vertices

    while Q is not empty {
        u = vertex in Q with smallest dist[u]
        remove u from Q

        for each neighbor v of u {
            alt = dist[u] + length(u, v)

            if alt < dist[v] {
                dist[v] = alt
                prev[v] = u
            }
        }
    }

    return [dist, prev];
}