class Solution {
  public:
    vector<int> dijkstra(int V, vector<vector<int>> &edges, int src) {
        // Step 1: Create adjacency list
        vector<pair<int, int>> adj[V];  // {neighbor, weight}
        for (auto &e : edges) {
            int u = e[0], v = e[1], w = e[2];
            adj[u].push_back({v, w});
            adj[v].push_back({u, w}); // undirected graph
        }

        // Step 2: Initialize distance array with infinity
        vector<int> dist(V, 1e9);
        dist[src] = 0;

        // Step 3: Min-heap to get minimum distance node
        priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
        pq.push({0, src}); // {distance, node}

        // Step 4: Dijkstra’s algorithm
        while (!pq.empty()) {
            int d = pq.top().first;
            int node = pq.top().second;
            pq.pop();

            // Skip if this distance is not the latest
            if (d > dist[node]) continue;

            // Traverse neighbors
            for (auto &nbr : adj[node]) {
                int next = nbr.first;
                int edgeWeight = nbr.second;

                // Relaxation step
                if (dist[node] + edgeWeight < dist[next]) {
                    dist[next] = dist[node] + edgeWeight;
                    pq.push({dist[next], next});
                }
            }
        }

        return dist;
    }
};