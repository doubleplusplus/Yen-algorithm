// yen's algorithm
// in which Dijkstra algorithm is adapted from:
// https://www.geeksforgeeks.org/dijkstras-shortest-path-algorithm-using-priority_queue-stl/?ref=lbp
#include "stdc++.h"
#include <iostream>

using namespace std; 

// iPair ==>  Integer Pair 
typedef pair<int, int> iPair; 
typedef pair<int, vector<int>> vPair;

// This class represents a directed graph using 
// adjacency list representation 
class Graph 
{ 
public:
    int V;    // No. of vertices 
    // In a weighted graph, we need to store vertex 
    // and weight pair for every edge 
    list< pair<int, int> > *adj; 
    vector<int> dist;  // to store distance from source
 
    Graph(int V) {
        this->V = V;
        adj = new list<iPair> [V];        
    } // Constructor  
    Graph(const Graph &rhs) { /* copy construction from rhs*/ 
        V = rhs.V;
        adj = rhs.adj; 
        dist = rhs.dist;
    }

    // function to add an edge to graph 
    void addEdge(int u, int v, int w) {
        adj[u].push_back(make_pair(v, w)); 
        adj[v].push_back(make_pair(u, w)); 
    } 

    void delEdge(int u, int v){
    // Traversing through the first vector list 
    // and removing the second element from it 
        for ( auto it = adj[u].begin(); it != adj[u].end(); it++) { 
            if (it->first == v) { 
                it = adj[u].erase(it); 
                break; 
            } 
        } 
        // Traversing through the second vector list 
        // and removing the first element from it 
        for ( auto it = adj[v].begin(); it != adj[v].end(); it++) { 
            if (it->first == u) { 
                it = adj[v].erase(it); 
                break; 
            } 
        } 
    }

    vector<int> findPath(int parent[], int s, int d){
        vector<int> path;
        path.push_back(d);
        int u = parent[d];
        path.insert(path.begin(), u); 
        while (u != s){
            int temp = parent[u];
            u = temp;
            path.insert(path.begin(), temp);
        }
        return path;
    }
    // prints shortest path from s 
    vector<int> dijkstra(int src, int des){
            // Create a priority queue to store vertices that 
        // are being preprocessed. This is weird syntax in C++. 
        // Refer below link for details of this syntax 
        // https://www.geeksforgeeks.org/implement-min-heap-using-stl/ 
        priority_queue< iPair, vector <iPair> , greater<iPair> > pq; 
    
        // Create a vector for distances and initialize all 
        // distances as infinite (INF) 
        //vector<int> dist(V, INT_MAX); 
        dist.reserve(V);
        for (int i=0; i<V; i++){
            dist[i] = 100000;
        }
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
        vector<int> path = findPath(parent, src, des);
        return path;
    } 
}; 

void showlist(vector<int> g) { 
    for(auto it = g.begin(); it != g.end(); ++it) 
        cout << *it << ' '; 
} 

// helper function to slice vector from X to Y, inclusive
vector<int> slicing(vector<int>& arr, int X, int Y) {  
    // Starting and Ending iterators 
    auto start = arr.begin() + X; 
    auto end = arr.begin() + Y + 1; 
  
    // To store the sliced vector 
    vector<int> result(Y - X + 1); 
  
    // Copy vector using copy function() 
    copy(start, end, result.begin()); 
  
    // Return the final sliced vector 
    return result; 
} 

// psudocode: https://en.wikipedia.org/wiki/Yen%27s_algorithm
vector<vector<int>> yen(Graph g, int s, int d, int K) {
    // Determine the shortest path from the s to the d
    vector<vector<int>> A;
    //Graph g_copy = g;
    // apply dijkstra
    vector<int> path = g.dijkstra(s, d);

    A.push_back(path);
    // Initialize the set to store the potential kth shortest path.
    priority_queue<vPair, vector<vPair>, greater<vPair>> B;
    
    for (int k=1; k<K; k++){ 
        // The spur node ranges from the first node to the next to last node in the previous k-shortest path
        for (int i=0; i<=A[k-1].size()-2; i++){
            Graph g_copy = g;
            // Spur node is retrieved from the previous k-shortest path, k − 1
            int spurNode = A[k-1][i];
            int root_dis = g.dist[spurNode];
            // The sequence of nodes from the source to the spur node of the previous k-shortest path
            vector<int> rootPath = slicing(A[k-1], 0, i);
            // for each path p in A
            for (int j = 0; j < A.size(); j++) { 
                vector<int> p = A[j];
                if (rootPath == slicing(p, 0, i)) {   
                    // Remove the links that are part of the previous shortest paths which share the same root path
                    g_copy.delEdge(p[i], p[i+1]);
                }
            }
            
            // for each node rootPathNode in rootPath except spurNode:
            // remove rootPathNode from Graph;
            //for (int i=0; i<rootPath.size()-1; i++){               
            //}
            // Calculate the spur path from the spur node to the sink.
            
            vector<int> spurPath = g_copy.dijkstra(spurNode, d);
            int spur_dis = g_copy.dist[d];
            
            // Entire path is made up of the root path and spur path
            vector<int> totalPath = rootPath;
            totalPath.insert(totalPath.end(), spurPath.begin()+1, spurPath.end());
            // Add the potential k-shortest path to the heap
            //if (totalPath not in B):
            //   B.append(totalPath);
            if (B.empty()){
                B.push(make_pair(root_dis+spur_dis, totalPath));
            }
            priority_queue<vPair, vector<vPair>, greater<vPair>> temp;         
            bool found = false;
            while (!B.empty()) {
                vPair dis_path = B.top(); 
                temp.push(dis_path);
                B.pop();     
                if (dis_path.second == totalPath & !found){
                    found = true;
                }   
            }
            if (!found) {
                temp.push(make_pair(root_dis+spur_dis, totalPath));
            }
            B = temp;
            // Add back the edges and nodes that were removed from the graph
            //g_copy = g; // already happens in the begiging of this loop
        }
        if (B.empty()){
            // This handles the case of there being no spur paths, or no spur paths left.
            // This could happen if the spur paths have already been exhausted (added to A), 
            // or there are no spur paths at all - such as when both the source and sink vertices 
            // lie along a "dead end".
            break;
        }
        // Add the lowest cost path becomes the k-shortest path
        A.push_back(B.top().second);
        // In fact we should rather use shift since we are removing the first element
        B.pop();
    }
    return A;
}

// Driver program to test methods of graph class 
int main() 
{ 
    // create the graph 
    int V = 9; 
    Graph g(V); 
    // the picture of g is in the link of line 3
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
  
    /*vector<int> path = g.dijkstra(0, 4); 
    showlist(path);
    cout << "\n";
    int cost = g.dist[4];
    cout << cost << endl;
    */
    vector<vector<int>> A;
    A = yen(g, 0, 4, 8);
    for(auto it = A.begin(); it != A.end(); ++it) {
        showlist(*it); 
        cout << "\n";
    }
    return 0; 
} 
