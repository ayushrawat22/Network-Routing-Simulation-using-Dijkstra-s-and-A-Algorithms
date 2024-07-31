#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <cmath>
#include <chrono>
using namespace std;

#define INF numeric_limits<int>::max() // Infinity value for representing absence of edge

// Structure to represent a router
struct Router {
    int id;
    vector<pair<int, int>> connections; // <destination, weight>
};

// Class to represent a network
class Network {
    int V; // Number of routers
    vector<Router> routers;

public:
    // Constructor
    Network(int v) : V(v) {
        routers.resize(V);
        for (int i = 0; i < V; ++i)
            routers[i].id = i;
    }

    // Function to add a connection between routers
    void addConnection(int from, int to, int weight) {
        routers[from].connections.push_back({to, weight});
        routers[to].connections.push_back({from, weight}); // Assuming bidirectional connections
    }

    // Euclidean distance heuristic function for A* algorithm
    double heuristic(int src, int dest) {
        // Assuming the router coordinates are given
        double x1 = routers[src].id;
        double y1 = 0; // Dummy y-coordinate for simplicity
        double x2 = routers[dest].id;
        double y2 = 0; // Dummy y-coordinate for simplicity
        return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
    }

    // A* algorithm to find shortest paths from source to all other routers
    vector<int> aStar(int src, int dest) {
        vector<int> dist(V, INF);
        vector<bool> visited(V, false);
        dist[src] = 0;

        priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> pq;
        pq.push({0, src});

        while (!pq.empty()) {
            int u = pq.top().second;
            pq.pop();

            if (u == dest)
                break;

            if (visited[u])
                continue;

            visited[u] = true;

            for (const auto& neighbor : routers[u].connections) {
                int v = neighbor.first;
                int weight = neighbor.second;

                if (!visited[v] && dist[u] != INF && dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                    double f = dist[v] + heuristic(v, dest); // f(n) = g(n) + h(n)
                    pq.push({f, v});
                }
            }
        }

        return dist;
    }

    // Function to access the connections of a router
    vector<pair<int, int>> getRouterConnections(int routerId) const {
        return routers[routerId].connections;
    }

    // Dijkstra's algorithm to find shortest paths from source to all other routers
    vector<int> dijkstra(int src) {
        vector<int> dist(V, INF);
        vector<bool> visited(V, false);
        dist[src] = 0;

        priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
        pq.push({0, src});

        while (!pq.empty()) {
            int u = pq.top().second;
            pq.pop();

            if (visited[u])
                continue;

            visited[u] = true;

            for (const auto& neighbor : routers[u].connections) {
                int v = neighbor.first;
                int weight = neighbor.second;

                if (!visited[v] && dist[u] != INF && dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                    pq.push({dist[v], v});
                }
            }
        }

        return dist;
    }

    // Function to simulate packet transmission and collect performance metrics for A* algorithm
    void simulatePacketTransmissionAStar(int src, int dest) {
        auto start = chrono::high_resolution_clock::now();

        vector<int> shortestDistances = aStar(src, dest);
        int shortestPathLength = shortestDistances[dest];

        auto end = chrono::high_resolution_clock::now();
        chrono::duration<double> elapsed_seconds = end - start;
        double transmissionTime = elapsed_seconds.count();

        int totalPackets = 0;
        int successfulPackets = 0;

        if (shortestPathLength == INF) {
            cout << "No path available from router " << src << " to router " << dest << endl;
            return;
        }

        cout << "Shortest path from router " << src << " to router " << dest << " (A* algorithm) is: ";
        int currentRouter = dest;
        vector<int> shortestPath;
        shortestPath.push_back(currentRouter);
        while (currentRouter != src) {
            for (const auto& neighbor : getRouterConnections(currentRouter)) {
                int neighborRouter = neighbor.first;
                int weight = neighbor.second;
                if (shortestDistances[currentRouter] == shortestDistances[neighborRouter] + weight) {
                    shortestPath.push_back(neighborRouter);
                    currentRouter = neighborRouter;
                    break;
                }
            }
        }
        for (int i = shortestPath.size() - 1; i >= 0; --i)
            cout << shortestPath[i] << " ";
        cout << endl;
        cout << "Total cost: " << shortestPathLength << endl;

        // Simulate packet transmission and calculate performance metrics
        cout << "Simulating packet transmission...\n";
        for (int i = 0; i < 100; ++i) { // Simulate transmission of 100 packets
            ++totalPackets;
            // Assuming packet transmission is successful if the destination router is reachable
            if (shortestPathLength != INF)
                ++successfulPackets;
        }

        cout << "Packet Delivery Ratio: " << (double)successfulPackets / totalPackets << endl;
        cout << "End-to-End Delay: " << transmissionTime << " seconds" << endl;
        cout << "Throughput: " << successfulPackets / transmissionTime << " packets/second" << endl;
    }
};

int main() {
    int numRouters, numConnections;
    cout << "Enter the number of routers: ";
    cin >> numRouters;
    cout << "Enter the number of connections: ";
    cin >> numConnections;

    Network network(numRouters);

    cout << "Enter connections (source, destination, weight):\n";
    for (int i = 0; i < numConnections; ++i) {
        int from, to, weight;
        cin >> from >> to >> weight;
        network.addConnection(from, to, weight);
    }

    int sourceRouter, destinationRouter;
    cout << "Enter source router: ";
    cin >> sourceRouter;
    cout << "Enter destination router: ";
    cin >> destinationRouter;

    cout << "\nDijkstra's Algorithm:\n";
    vector<int> dijkstraDistances = network.dijkstra(sourceRouter);
    int dijkstraShortestPathLength = dijkstraDistances[destinationRouter];
    cout << "Shortest path from router " << sourceRouter << " to router " << destinationRouter << " is: ";
    if (dijkstraShortestPathLength == INF)
        cout << "No path available\n";
    else {
        // Print shortest path
        int currentRouter = destinationRouter;
        vector<int> dijkstraShortestPath;
        dijkstraShortestPath.push_back(currentRouter);
        while (currentRouter != sourceRouter) {
            for (const auto& neighbor : network.getRouterConnections(currentRouter)) {
                int neighborRouter = neighbor.first;
                int weight = neighbor.second;
                if (dijkstraDistances[currentRouter] == dijkstraDistances[neighborRouter] + weight) {
                    dijkstraShortestPath.push_back(neighborRouter);
                    currentRouter = neighborRouter;
                    break;
                }
            }
        }
        for (int i = dijkstraShortestPath.size() - 1; i >= 0; --i)
            cout << dijkstraShortestPath[i] << " ";
        cout << "\nTotal cost: " << dijkstraShortestPathLength << endl;
    }

    cout << "\nA* Algorithm:\n";
    network.simulatePacketTransmissionAStar(sourceRouter, destinationRouter);

    return 0;
}
