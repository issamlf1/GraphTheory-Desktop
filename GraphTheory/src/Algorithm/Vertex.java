/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package Algorithm;

import java.awt.geom.Ellipse2D;
import java.util.List;

/**
 *
 * @author lanfouf
 */
public class Vertex {
    private int x;
    private int y;
    
    private int nom;
    private Ellipse2D nodes;

    public int getX() {
        return x;
    }

    public void setX(int x) {
        this.x = x;
    }

    public int getY() {
        return y;
    }

    public void setY(int y) {
        this.y = y;
    }

   

    public Vertex(int nom) {
        this.nom = nom;
    }

    public Vertex(int nom, Ellipse2D nodes) {
        this.nom = nom;
        this.nodes = nodes;
    }

    public Ellipse2D getNodes() {
        return nodes;
    }

    public void setNodes(Ellipse2D nodes) {
        this.nodes = nodes;
    }

    public int getNom() {
        return nom;
    }

    public void setNom(int nom) {
        this.nom = nom;
    }
    

    @Override
    public String toString() {
        return nom+"\t";
    }
   
    
}

