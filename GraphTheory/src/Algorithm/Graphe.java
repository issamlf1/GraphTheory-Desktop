/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package Algorithm;

import java.util.List;

/**
 *
 * @author lanfouf
 * 
 */

public interface Graphe <V>{
    //retourne le nombre des noeuds
    public int getSize();
    //retourne tout les noeuds du graphe 
    public java.util.List<V> getVertices();
    //retourne le noued d'indice index
    public V getVertex(int index);
    //retourn l'indice du noeud
    public int getIndex(V v);
    //retourne les voisin du noeud de l'indice index
    public java.util.List<Integer> getNeighbors(int index);
    //retourn le degree du noued 
    public int getDegree(int v);
    //afficher les aretes
    public void printEdges();
    //supprimer graphe
    public void clear();
    //ajouter un noeud
    public void addVertex(V vertex);
    //ajouter un arret
    public void addEdge(int u, int v);
     
     public void primMST(int graph[][]);
      public void KruskalMST(Edge[] edg) ;
      public  void dijkstra(int graph[][], int src);
      public void BellmanFord(Edge edj[],int src);
      public int fordFulkerson(int graph[][], int s, int t) ;
    //dfs
//    public AbstractGraph<V>.Tree dfs(int v);
//    //bfs
//    public AbstractGraph<V>.Tree bfs(int v);
    public List<Integer> BFS(int s);
    public void DFS(int v);
    public List<Integer> getDfsres();
    public List<Integer> getSourcedfs();
    public List<Integer> getDestdfs();
    public List<Integer> getDest();
    public List<Integer> getSource();
    
    public List<Integer> getKruskalsource();
    public List<Integer> getKruskaldest();
    public List<Integer> getKruskalcout();
    
    public List<Integer> getPrimsource();
    public List<Integer> getPrimdest();
    public List<Integer> getPrimcout();
    
    public List<Integer> getDijkstraVER();
    public List<Integer> getDijkstraCOU();
    
    public List<Integer> getBelmanVER();
    public List<Integer> getBelmanCOU();
    
    
    public int[][] floydWarshall(int graph[][]);
}
