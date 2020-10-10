/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package Algorithm;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Vector;
import static javafx.scene.input.KeyCode.V;

/**
 *
 * @author lanfouf
 */

public abstract class AbstractGraph<V> implements Graphe<V> {
    protected List<V> vertices = new ArrayList<V>();
    protected List<List<Integer>> neighbors= new ArrayList<List<Integer>>(); 
    protected List<Edge> edgelist= new ArrayList<>(); 
    protected  List<Integer> source=new ArrayList<>();
    protected  List<Integer> dest=new ArrayList<>();
    
    //-------KRuskal--------------
    protected  List<Integer> Kruskalsource=new ArrayList<>();
    protected  List<Integer> Kruskaldest=new ArrayList<>();
    protected  List<Integer> Kruskalcout=new ArrayList<>();
    //--------Prim-----------------
    protected  List<Integer> Primsource=new ArrayList<>();
    protected  List<Integer> Primdest=new ArrayList<>();
    protected  List<Integer> Primcout=new ArrayList<>();
    //---------Dijkstra-------------
    
    protected  List<Integer> dijkstraVER=new ArrayList<>();
    protected  List<Integer> dijkstraCOU=new ArrayList<>();
    
    //---------bell-man------------
    protected  List<Integer> belmanVER=new ArrayList<>();
    protected  List<Integer> belmanCOU=new ArrayList<>();
  
    
    
    protected  List<Integer> sourcedfs=new ArrayList<>();
    protected  List<Integer> destdfs=new ArrayList<>();
    protected  List<Integer> dfsres=new ArrayList<>();
    protected List<Edge> oldedges= new ArrayList<>(); 

    public List<Integer> getDijkstraVER() {
        return dijkstraVER;
    }

    public List<Integer> getDijkstraCOU() {
        return dijkstraCOU;
    }

    public List<Integer> getBelmanVER() {
        return belmanVER;
    }

    public List<Integer> getBelmanCOU() {
        return belmanCOU;
    }

    public List<Integer> getPrimsource() {
        return Primsource;
    }

    public List<Integer> getPrimdest() {
        return Primdest;
    }

    public List<Integer> getPrimcout() {
        return Primcout;
    }

    public List<Integer> getKruskalsource() {
        return Kruskalsource;
    }

    public List<Integer> getKruskaldest() {
        return Kruskaldest;
    }

    public List<Integer> getKruskalcout() {
        return Kruskalcout;
    }

    public List<Integer> getSourcedfs() {
        return sourcedfs;
    }

    public List<Integer> getDestdfs() {
        return destdfs;
    }

    public List<Integer> getDfsres() {
        return dfsres;
    }
    

    public List<Integer> getSource() {
        return source;
    }

    public List<Integer> getDest() {
        return dest;
    }
    
    // graphe vide 
    public AbstractGraph() {
    }
    //creation de graphe par tableu
    protected AbstractGraph(int[][] edges, V[] vertices) {
    for (int i = 0; i < vertices.length; i++)
    this.vertices.add(vertices[i]);
    createAdjacencyLists(edges, vertices.length);
    }
    /** Construct a graph from edges and vertices stored in List */
    protected AbstractGraph(List<Edge> edges, List<V> vertices) {
    for (int i = 0; i < vertices.size(); i++)
    this.vertices.add(vertices.get(i));
    createAdjacencyLists(edges, vertices.size());
    }
    /** Construct a graph for integer vertices 0, 1, 2 and edge list */
    protected AbstractGraph(List<Edge> edges, int numberOfVertices) {
    for (int i = 0; i < numberOfVertices; i++)
    vertices.add((V)(new Integer(i))); // vertices is {0, 1, ...}
    createAdjacencyLists(edges, numberOfVertices);
    }
    /** Construct a graph from integer vertices 0, 1, and edge array */
    protected AbstractGraph(int[][] edges, int numberOfVertices) {
    for (int i = 0; i < numberOfVertices; i++)
    vertices.add((V)(new Integer(i))); // vertices is {0, 1, ...}
    createAdjacencyLists(edges, numberOfVertices);
    }
    /** Create adjacency lists for each vertex */
    private void createAdjacencyLists(int[][] edges, int numberOfVertices) {
     // Create a linked list
    for (int i = 0; i < numberOfVertices; i++) {
    neighbors.add(new ArrayList<Integer>());
    }

    for (int i = 0; i < edges.length; i++) {
    int u = edges[i][0];
    int v = edges[i][1];
    neighbors.get(u).add(v);
        }
    }
     private void createAdjacencyLists(List<Edge> edges, int numberOfVertices) {
    // Create a linked list for each vertex
    for (int i = 0; i < numberOfVertices; i++) {
        neighbors.add(new ArrayList<Integer>());
        }
    
    for (Edge edge: edges) {
    neighbors.get(edge.getU()).add(edge.getV());
         }
        
         for (int i = 0; i < edges.size(); i++) {
             this.oldedges.add(new Edge(edges.get(i).getU(), edges.get(i).getV(),edges.get(i).getCout()));
             
             
         }
    
    }
      @Override /** Return the vertices in the graph */
  public List<V> getVertices() {
    return vertices;
  }

  @Override /** Return the object for the specified vertex */
  public V getVertex(int index) {
    return vertices.get(index);
  }
@Override /** Return the index for the specified vertex object */
  public int getIndex(V v) {
    return vertices.indexOf(v);
  }
  @Override /** Return the neighbors of the specified vertex */
public List<Integer> getNeighbors(int index) {
 return neighbors.get(index);
 }
@Override /** Return the degree for a specified vertex */
 public int getDegree(int v) {
 return neighbors.get(v).size();
 }
 @Override /** Return the number of vertices in the graph */
 public int getSize() {
 return vertices.size();
 }
//This code is contributed by Aakash Hasija 
 @Override /** Print the edges */
 public void printEdges() {
 for (int u = 0; u < neighbors.size(); u++) {
 System.out.print(getVertex(u) + " (" + u + "): ");
 for (int j = 0; j < neighbors.get(u).size(); j++) {
 System.out.print("(" + u + ", " +
 neighbors.get(u).get(j) + ") ");
}
 System.out.println();
 }
 }


 @Override /** Clear graph */
 public void clear() {
 vertices.clear();
 neighbors.clear();
 }
 @Override /** Add a vertex to the graph */
 public void addVertex(V vertex) {
 vertices.add(vertex);
 neighbors.add(new ArrayList<Integer>());
 }

 @Override /** Add an edge to the graph */
 public void addEdge(int u, int v) {
 neighbors.get(u).add(v);
 neighbors.get(v).add(u);
 }


 //-------------------------------bfs---------------------------------------
public List<Integer> BFS(int s) 
    { this.source.clear();
    this.dest.clear();
        // Mark all the vertices as not visited(By default 
        // set as false) 
        int V=this.vertices.size();
        boolean visited[] = new boolean[V]; 
        List<Integer> ar =new ArrayList<>();
  
        // Create a queue for BFS 
        LinkedList<Integer> queue = new LinkedList<Integer>(); 
  
        // Mark the current node as visited and enqueue it 
        visited[s]=true; 
        queue.add(s); 
  
        while (queue.size() != 0) 
        { 
            // Dequeue a vertex from queue and print it 
            s = queue.poll(); 
            
            //System.out.print(((Vertex)vertices.get(s)).getNom());
            ar.add(((Vertex)vertices.get(s)).getNom());
            
  
            // Get all adjacent vertices of the dequeued vertex s 
            // If a adjacent has not been visited, then mark it 
            // visited and enqueue it 
            Iterator<Integer> i = neighbors.get(s).listIterator(); 
            while (i.hasNext()) 
            { 
                int n = i.next(); 
                if (!visited[n]) 
                { 
                    visited[n] = true; 
                    queue.add(n); 
                    
                    source.add(s);
                    dest.add(n);
                } 
            } 
        }
        
        return ar;
    } 


//---------------------------------Dfs-----------------------
 void DFSUtil(int v,boolean visited[]) 
    { 
        // Mark the current node as visited and print it 
        visited[v] = true; 
//        System.out.print(v+" "); 
//        System.out.print(vertices.get(v));
        this.dfsres.add(v);
  
        // Recur for all the vertices adjacent to this vertex 
        Iterator<Integer> i = neighbors.get(v).listIterator(); 
        while (i.hasNext()) 
        { 
            int n = i.next(); 
            if (!visited[n]) {
                this.sourcedfs.add(n);
            this.destdfs.add(v);
                DFSUtil(n, visited); 
            
            }
                
//            System.out.println(n+"--"+v);
            
        } 
    } 
 
 public void DFS(int v) 
    { 
        // Mark all the vertices as not visited(set as 
        // false by default in java) 
        int V=this.vertices.size();
        boolean visited[] = new boolean[V]; 
  
        // Call the recursive helper function to print DFS traversal 
        DFSUtil(v, visited); 
    } 
//------------------kruskal--------------------------------
 class subset 
    { 
        int parent, rank; 
    }; 
 
 // A utility function to find set of an element i 
    // (uses path compression technique) 
    int find(AbstractGraph.subset subsets[], int i) 
    { 
        // find root and make root as parent of i (path compression) 
        if (subsets[i].parent != i) 
            subsets[i].parent = find(subsets, subsets[i].parent); 
  
        return subsets[i].parent; 
    } 
// A function that does union of two sets of x and y 
    // (uses union by rank) 
    void Union(AbstractGraph.subset subsets[], int x, int y) 
    { 
        int xroot = find(subsets, x); 
        int yroot = find(subsets, y); 
  
        // Attach smaller rank tree under root of high rank tree 
        // (Union by Rank) 
        if (subsets[xroot].rank < subsets[yroot].rank) 
            subsets[xroot].parent = yroot; 
        else if (subsets[xroot].rank > subsets[yroot].rank) 
            subsets[yroot].parent = xroot; 
  
        // If ranks are same, then make one as root and increment 
        // its rank by one 
        else
        { 
            subsets[yroot].parent = xroot; 
            subsets[xroot].rank++; 
        } 
    } 
  public void KruskalMST(Edge[] edg) 
    { int V=this.vertices.size();
        Edge result[] = new Edge[V];  // Tnis will store the resultant MST 
        int e = 0;  // An index variable, used for result[] 
        int i = 0;  // An index variable, used for sorted edges 
        for (i=0; i<V; ++i) 
            result[i] = new Edge(); 
  
        // Step 1:  Sort all the edges in non-decreasing order of their 
        // weight.  If we are not allowed to change the given graph, we 
        // can create a copy of array of edges 
        Arrays.sort(edg); 
  
        // Allocate memory for creating V ssubsets 
        AbstractGraph.subset subsets[] = new AbstractGraph.subset[V]; 
        for(i=0; i<V; ++i) 
            subsets[i]=new AbstractGraph.subset(); 
  
        // Create V subsets with single elements 
        for (int v = 0; v < V; ++v) 
        { 
            subsets[v].parent = v; 
            subsets[v].rank = 0; 
        } 
  
        i = 0;  // Index used to pick next edge 
  
        // Number of edges to be taken is equal to V-1 
        while (e < V - 1) 
        { 
            // Step 2: Pick the smallest edge. And increment  
            // the index for next iteration 
            Edge next_edge = new Edge(); 
            next_edge = edg[i++]; 
  
            int x = find(subsets, next_edge.getU()); 
            int y = find(subsets, next_edge.getV()); 
  
            // If including this edge does't cause cycle, 
            // include it in result and increment the index  
            // of result for next edge 
            if (x != y) 
            { 
                result[e++] = next_edge; 
                Union(subsets, x, y); 
            } 
            // Else discard the next_edge 
        } 
  
        // print the contents of result[] to display 
        // the built MST 
//        System.out.println("Following are the edges in " +  
//                                     "the constructed MST"); 
        for (i = 0; i < e; ++i) {
//            System.out.println(result[i].getU()+" -- " +  
//                   result[i].getV()+" == " + result[i].getCout());
        this.Kruskalsource.add(result[i].getU());
        this.Kruskaldest.add(result[i].getV());
        this.Kruskalcout.add(result[i].getCout());
        }
             
    }
 //-------------------------Prim --------------------------
  int minKey(int key[], Boolean mstSet[]) 
    { 
        // Initialize min value 
        int min = Integer.MAX_VALUE, min_index=-1; 
        int V=this.vertices.size();
        for (int v = 0; v < V; v++) 
            if (mstSet[v] == false && key[v] < min) 
            { 
                min = key[v]; 
                min_index = v; 
            } 
  
        return min_index; 
    } 
  
  void printMST(int parent[], int n, int graph[][]) 
    {   int V=this.vertices.size();
//        System.out.println("Edge \tWeight"); 
        for (int i = 1; i < V; i++){
//        System.out.println(parent[i]+" - "+ i+"\t"+ 
//                            graph[i][parent[i]]);
        this.Primsource.add(parent[i]);
        this.Primdest.add(i);
        this.Primcout.add(graph[i][parent[i]]);
        } 
             
    } 
  
      public void primMST(int graph[][]) 
    { 
        int V=this.vertices.size();
        // Array to store constructed MST 
        int parent[] = new int[V]; 
  
        // Key values used to pick minimum weight edge in cut 
        int key[] = new int [V]; 
  
        // To represent set of vertices not yet included in MST 
        Boolean mstSet[] = new Boolean[V]; 
  
        // Initialize all keys as INFINITE 
        for (int i = 0; i < V; i++) 
        { 
            key[i] = Integer.MAX_VALUE; 
            mstSet[i] = false; 
        } 
  
        // Always include first 1st vertex in MST. 
        key[0] = 0;     // Make key 0 so that this vertex is 
                        // picked as first vertex 
        parent[0] = -1; // First node is always root of MST 
  
        // The MST will have V vertices 
        for (int count = 0; count < V-1; count++) 
        { 
            // Pick thd minimum key vertex from the set of vertices 
            // not yet included in MST 
            int u = minKey(key, mstSet); 
  
            // Add the picked vertex to the MST Set 
            mstSet[u] = true; 
  
            // Update key value and parent index of the adjacent 
            // vertices of the picked vertex. Consider only those 
            // vertices which are not yet included in MST 
            for (int v = 0; v < V; v++) 
  
                // graph[u][v] is non zero only for adjacent vertices of m 
                // mstSet[v] is false for vertices not yet included in MST 
                // Update the key only if graph[u][v] is smaller than key[v] 
                if (graph[u][v]!=0 && mstSet[v] == false && 
                    graph[u][v] < key[v]) 
                { 
                    parent[v] = u; 
                    key[v] = graph[u][v]; 
                } 
        } 
  
        // print the constructed MST 
        printMST(parent, V, graph); 
    } 
  
 //-----------------------Dijkstra---------------------------------
       int minDistance(int dist[], Boolean sptSet[]) 
    { 
        // Initialize min value 
        int V=this.vertices.size();
        int min = Integer.MAX_VALUE, min_index=-1; 
  
        for (int v = 0; v < V; v++) 
            if (sptSet[v] == false && dist[v] <= min) 
            { 
                min = dist[v]; 
                min_index = v; 
            } 
  
        return min_index; 
    } 
       
      // A utility function to print the constructed distance array 
    void printSolution(int dist[], int n) 
    { int V=this.vertices.size();
        System.out.println("Vertex   Distance from Source"); 
        for (int i = 0; i < V; i++){
        System.out.println(i+" tt "+dist[i]); 
        this.dijkstraVER.add(i);
        this.dijkstraCOU.add(dist[i]);
        } 
            
    } 
  public  void dijkstra(int graph[][], int src) 
    { int V=this.vertices.size();
        int dist[] = new int[V]; // The output array. dist[i] will hold 
                                 // the shortest distance from src to i 
  
        // sptSet[i] will true if vertex i is included in shortest 
        // path tree or shortest distance from src to i is finalized 
        Boolean sptSet[] = new Boolean[V]; 
  
        // Initialize all distances as INFINITE and stpSet[] as false 
        for (int i = 0; i < V; i++) 
        { 
            dist[i] = Integer.MAX_VALUE; 
            sptSet[i] = false; 
        } 
  
        // Distance of source vertex from itself is always 0 
        dist[src] = 0; 
  
        // Find shortest path for all vertices 
        for (int count = 0; count < V-1; count++) 
        { 
            // Pick the minimum distance vertex from the set of vertices 
            // not yet processed. u is always equal to src in first 
            // iteration. 
            int u = minDistance(dist, sptSet); 
  
            // Mark the picked vertex as processed 
            sptSet[u] = true; 
  
            // Update dist value of the adjacent vertices of the 
            // picked vertex. 
            for (int v = 0; v < V; v++){
            
            // Update dist[v] only if is not in sptSet, there is an 
                // edge from u to v, and total weight of path from src to 
                // v through u is smaller than current value of dist[v] 
                if (!sptSet[v] && graph[u][v]!=0 && 
                        dist[u] != Integer.MAX_VALUE && 
                        dist[u]+graph[u][v] < dist[v]){
                dist[v] = dist[u] + graph[u][v];
                    System.out.println(u+" ----> "+v );
                } 
                    
            
            } 
  
                
                    
        } 
  
        // print the constructed distance array 
        printSolution(dist, V); 
    } 
 //---------------------bellman ford--------------------------
   // A utility function used to print the solution 
    void printArr(int dist[], int V) 
    { 
        System.out.println("Vertex   Distance from Source"); 
        for (int i=0; i<V; ++i) {
        System.out.println(i+"\t\t"+dist[i]);
        this.belmanVER.add(i);
        this.belmanCOU.add(dist[i]);
        }
             
    } 
     public void BellmanFord(Edge edj[],int src) 
    { 
        int V = this.vertices.size(), E = edj.length; 
        int dist[] = new int[V]; 
  
        // Step 1: Initialize distances from src to all other 
        // vertices as INFINITE 
        for (int i=0; i<V; ++i) 
            dist[i] = Integer.MAX_VALUE; 
        dist[src] = 0; 
  
        // Step 2: Relax all edges |V| - 1 times. A simple 
        // shortest path from src to any other vertex can 
        // have at-most |V| - 1 edges 
        for (int i=1; i<V; ++i) 
        { 
            for (int j=0; j<E; ++j) 
            { 
                int u = edj[j].getU(); 
                int v = edj[j].getV(); 
                int weight = edj[j].getCout();
                if (dist[u]!=Integer.MAX_VALUE && 
                    dist[u]+weight<dist[v]) 
                    dist[v]=dist[u]+weight; 
            } 
        } 
  
        // Step 3: check for negative-weight cycles.  The above 
        // step guarantees shortest distances if graph doesn't 
        // contain negative weight cycle. If we get a shorter 
        //  path, then there is a cycle. 
        for (int j=0; j<E; ++j) 
        { 
            int u = edj[j].getU(); 
            int v = edj[j].getV(); 
            int weight = edj[j].getCout();  
            if (dist[u] != Integer.MAX_VALUE && 
                dist[u]+weight < dist[v]) 
              System.out.println("Graph contains negative weight cycle"); 
        } 
        printArr(dist, V); 
    }
     
    //----------------------------------------Ford-Fulkerson-----------------
     boolean bfs(int rGraph[][], int s, int t, int parent[]) 
    { 
        // Create a visited array and mark all vertices as not 
        // visited 
        int V = this.vertices.size();
        boolean visited[] = new boolean[V]; 
        for(int i=0; i<V; ++i) 
            visited[i]=false; 
  
        // Create a queue, enqueue source vertex and mark 
        // source vertex as visited 
        LinkedList<Integer> queue = new LinkedList<Integer>(); 
        queue.add(s); 
        visited[s] = true; 
        parent[s]=-1; 
  
        // Standard BFS Loop 
        while (queue.size()!=0) 
        { 
            int u = queue.poll(); 
  
            for (int v=0; v<V; v++) 
            { 
                if (visited[v]==false && rGraph[u][v] > 0) 
                { 
                    queue.add(v); 
                    parent[v] = u; 
                    visited[v] = true; 
                } 
            } 
        } 
  
        // If we reached sink in BFS starting from source, then 
        // return true, else false 
        return (visited[t] == true); 
    } 
     public int fordFulkerson(int graph[][], int s, int t) 
    { 
        int u, v; 
  
        // Create a residual graph and fill the residual graph 
        // with given capacities in the original graph as 
        // residual capacities in residual graph 
  
        // Residual graph where rGraph[i][j] indicates 
        // residual capacity of edge from i to j (if there 
        // is an edge. If rGraph[i][j] is 0, then there is 
        // not) 
        int V = this.vertices.size();
        int rGraph[][] = new int[V][V]; 
  
        for (u = 0; u < V; u++) 
            for (v = 0; v < V; v++) 
                rGraph[u][v] = graph[u][v]; 
  
        // This array is filled by BFS and to store path 
        int parent[] = new int[V]; 
  
        int max_flow = 0;  // There is no flow initially 
  
        // Augment the flow while tere is path from source 
        // to sink 
        while (bfs(rGraph, s, t, parent)) 
        { 
            // Find minimum residual capacity of the edhes 
            // along the path filled by BFS. Or we can say 
            // find the maximum flow through the path found. 
            int path_flow = Integer.MAX_VALUE; 
            for (v=t; v!=s; v=parent[v]) 
            { 
                u = parent[v]; 
                path_flow = Math.min(path_flow, rGraph[u][v]); 
            } 
  
            // update residual capacities of the edges and 
            // reverse edges along the path 
            for (v=t; v != s; v=parent[v]) 
            { 
                u = parent[v]; 
                rGraph[u][v] -= path_flow; 
                rGraph[v][u] += path_flow; 
            } 
  
            // Add path flow to overall flow 
            max_flow += path_flow; 
        } 
  
        // Return the overall flow 
        return max_flow; 
    }
     
     //-----------------------------warshall--------------------
     
    final static int INF = 99999; 
  
    /**
     *
     * @param graph
     * @return
     */
    public int[][] floydWarshall(int graph[][]) 
    { int V=graph[0].length;
        int dist[][] = new int[V][V]; 
        int i, j, k; 
  
        /* Initialize the solution matrix same as input graph matrix. 
           Or we can say the initial values of shortest distances 
           are based on shortest paths considering no intermediate 
           vertex. */
        for (i = 0; i < V; i++) 
            for (j = 0; j < V; j++) 
                dist[i][j] = graph[i][j]; 
  
        /* Add all vertices one by one to the set of intermediate 
           vertices. 
          ---> Before start of an iteration, we have shortest 
               distances between all pairs of vertices such that 
               the shortest distances consider only the vertices in 
               set {0, 1, 2, .. k-1} as intermediate vertices. 
          ----> After the end of an iteration, vertex no. k is added 
                to the set of intermediate vertices and the set 
                becomes {0, 1, 2, .. k} */
        for (k = 0; k < V; k++) 
        { 
            // Pick all vertices as source one by one 
            for (i = 0; i < V; i++) 
            { 
                // Pick all vertices as destination for the 
                // above picked source 
                for (j = 0; j < V; j++) 
                { 
                    // If vertex k is on the shortest path from 
                    // i to j, then update the value of dist[i][j] 
                    if (dist[i][k] + dist[k][j] < dist[i][j]) 
                        dist[i][j] = dist[i][k] + dist[k][j]; 
                } 
            } 
        } 
        return dist;
  
        
         
    }
    
    
    }

    