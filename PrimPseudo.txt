function prim(graph, source) {
    MST = {}
    visited = { source }
    edges = graph.edges
    edges = sort(edges, by weight)

    while visited != graph.vertices {
        edge = first_unvisited_edge(edges)
        MST.add(edge)
        visited.add(other_vertex(edge))
    }

    return MST
}