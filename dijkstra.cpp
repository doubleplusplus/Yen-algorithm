// dijkstra algorithm
// https://www.geeksforgeeks.org/dijkstras-shortest-path-algorithm-using-priority_queue-stl/?ref=lbp
#include "stdc++.h"
#include <iostream>

using namespace std; 

// iPair ==>  Integer Pair 
typedef pair<int, int> iPair; 

// This class represents a directed graph using 
// adjacency list representation 
class Graph 
{ 
    int V;    // No. of vertices 
    // In a weighted graph, we need to store vertex 
    // and weight pair for every edge 
    list< pair<int, int> > *adj; 
  
public: 
    Graph(int V) {
        this->V = V;
        adj = new list<iPair> [V];
    } // Constructor  
    // function to add an edge to graph 
    void addEdge(int u, int v, int w) {
        adj[u].push_back(make_pair(v, w)); 
        adj[v].push_back(make_pair(u, w)); 
    } 

    list<int> findPath(int parent[], int s, int d){
        list<int> path;
        path.push_back(d);
        int u = parent[d];
        path.push_front(u);
        while (u != s){
            int temp = parent[u];
            u = temp;
            path.push_front(temp);
        }
        return path;
    }
    // prints shortest path from s 
    list<int> dijkstra(int src, int des){
            // Create a priority queue to store vertices that 
        // are being preprocessed. This is weird syntax in C++. 
        // Refer below link for details of this syntax 
        // https://www.geeksforgeeks.org/implement-min-heap-using-stl/ 
        priority_queue< iPair, vector <iPair> , greater<iPair> > pq; 
    
        // Create a vector for distances and initialize all 
        // distances as infinite (INF) 
        vector<int> dist(V, INT_MAX); 
    
        // Insert source itself in priority queue and initialize 
        // its distance as 0. 
        pq.push(make_pair(0, src)); 
        dist[src] = 0; 

        // Parent array to store shortest path
        int parent[V];
        parent[0] = -1;
    
        /* Looping till priority queue becomes empty (or all 
        distances are not finalized) */
        while (!pq.empty()) 
        { 
            // The first vertex in pair is the minimum distance 
            // vertex, extract it from priority queue. 
            // vertex label is stored in second of pair (it 
            // has to be done this way to keep the vertices 
            // sorted distance (distance must be first item 
            // in pair) 
            int u = pq.top().second; 
            pq.pop(); 
    
            // 'i' is used to get all adjacent vertices of a vertex 
            list< pair<int, int> >::iterator i; 
            for (i = adj[u].begin(); i != adj[u].end(); ++i) 
            { 
                // Get vertex label and weight of current adjacent 
                // of u. 
                int v = (*i).first; 
                int weight = (*i).second; 
    
                //  If there is shorted path to v through u. 
                if (dist[v] > dist[u] + weight) 
                { 
                    // Update parent of v. Update distance of v
                    parent[v] = u;
                    dist[v] = dist[u] + weight; 
                    pq.push(make_pair(dist[v], v)); 
                } 
            } 
        } 
        list<int> path = findPath(parent, src, des);
        return path;
    } 
}; 

void showlist(list <int> g) { 
    list <int> :: iterator it; 
    for(it = g.begin(); it != g.end(); ++it) 
        cout << *it << ' '; 
} 

// Driver program to test methods of graph class 
int main() 
{ 
    // create the graph 
    int V = 9; 
    Graph g(V); 
  
    // edge between u, v. The third int is weight
    g.addEdge(0, 1, 4); 
    g.addEdge(0, 7, 8); 
    g.addEdge(1, 2, 8); 
    g.addEdge(1, 7, 11); 
    g.addEdge(2, 3, 7); 
    g.addEdge(2, 8, 2); 
    g.addEdge(2, 5, 4); 
    g.addEdge(3, 4, 9); 
    g.addEdge(3, 5, 14); 
    g.addEdge(4, 5, 10); 
    g.addEdge(5, 6, 2); 
    g.addEdge(6, 7, 1); 
    g.addEdge(6, 8, 6); 
    g.addEdge(7, 8, 7); 
  
    list<int> path = g.dijkstra(0, 4); 
    showlist(path);
    return 0; 
} 