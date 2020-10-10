/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package Algorithm;

import java.util.List;


public class UnweightedGraph<V> extends AbstractGraph<V>{

  
    public UnweightedGraph() {
 }

 /** Construct a graph from edges and vertices stored in arrays */
public UnweightedGraph(int[][] edges, V[] vertices) {
 super(edges, vertices);
 }

 /** Construct a graph from edges and vertices stored in List */
 public UnweightedGraph(List<Edge> edges, List<V> vertices) {
 super(edges, vertices);
 }

 /** Construct a graph for integer vertices 0, 1, 2 and edge list */
 public UnweightedGraph(List<Edge> edges, int numberOfVertices) {
 super(edges, numberOfVertices);
}
/** Construct a graph from integer vertices 0, 1, and edge array */
 public UnweightedGraph(int[][] edges, int numberOfVertices) {
 super(edges, numberOfVertices);
 }

    

    
    
}
