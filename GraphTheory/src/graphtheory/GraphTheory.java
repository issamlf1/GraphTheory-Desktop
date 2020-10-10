/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package graphtheory;

import java.util.ArrayList;
import java.util.List;
import Algorithm.*;


/**
 *
 * @author lanfouf
 */
public class GraphTheory {

    /**
     * @param args the command line arguments
     */
    public static void main(String[] args) {
        List<Vertex> list_nou =new ArrayList<>();
        
//        list_nou.add(new Vertex("0"));
//        list_nou.add(new Vertex("0"));
//        list_nou.add(new Vertex("1"));
//        list_nou.add(new Vertex("2"));
//        list_nou.add(new Vertex("3"));
//        list_nou.add(new Vertex("5"));
        
        
        
        
        
        
        
        


 List<Edge> liste_edges =new ArrayList<>();
 liste_edges.add(new Edge(0, 1,-1));
 liste_edges.add(new Edge(0, 2,4));
 liste_edges.add(new Edge(1, 2,3));
 liste_edges.add(new Edge(1,3,2));
 liste_edges.add(new Edge(1, 4,2));
 liste_edges.add(new Edge(3,2,5));
 liste_edges.add(new Edge(3,1,1));
 liste_edges.add(new Edge(4, 3,-3));

 
 

 

         int graphee [][]=new int[list_nou.size()][list_nou.size()];
         for (int i = 0; i < 10; i++) {
             for (int j = 0; j < 10; j++) {
                 graphee[i][j]=0;
             }
        }
        for (int i = 0; i < list_nou.size(); i++) {
            for (int j = 0; j < list_nou.size(); j++) {
                for (int k = 0; k < liste_edges.size(); k++) {
                    if (i==liste_edges.get(k).getU() && j==liste_edges.get(k).getV()) {
                    graphee[i][j]=liste_edges.get(k).getCout();
                    
                }
                }
  
            }
        }
        for (int i = 0; i < 10; i++) {
            for (int j = 0; j < 10; j++) {
                if (graphee[i][j]!=1 && i!=j) {
                    graphee[i][j]=99999;
                }
            }
        }
        
        for (int i = 0; i < graphee.length; i++) {
            for (int j = 0; j < graphee[i].length; j++) {
                System.out.print(graphee[i][j]);
            }System.out.println();
        }
  
  
  
 Edge[] edg=new Edge[liste_edges.size()];
        for (int i = 0; i < liste_edges.size(); i++) {
             edg[i]= liste_edges.get(i);
            
        }

Graphe<Vertex> graph1 = new UnweightedGraph<Vertex>(liste_edges, list_nou);
//        System.out.println(graph1.getSize());
//        System.out.println(graph1.getVertex(0));
//        System.out.println(graph1.getIndex(list_nou.get(0)));
        
//        graph1.printEdges();
//        graph1.BFS(0);
//        graph1.DFS(0);
//        graph1.KruskalMST(edg);
//        graph1.primMST(graphee);
                                   
 
//graph1.dijkstra(graphee, 0);
//graph1.BellmanFord(edg, 0);

int grapha[][] =new int[][] { {0, 16, 13, 0, 0, 0}, 
                                     {0, 0, 10, 12, 0, 0}, 
                                     {0, 4, 0, 0, 14, 0}, 
                                     {0, 0, 9, 0, 0, 20}, 
                                     {0, 0, 0, 7, 0, 4}, 
                                     {0, 0, 0, 0, 0, 0} 
                                   };

        System.out.println("le flow max est : "+graph1.fordFulkerson(grapha, 0, 5));
    }
    
}
