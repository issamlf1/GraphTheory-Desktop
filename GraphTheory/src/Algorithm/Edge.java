/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */





package Algorithm;

import java.awt.geom.Line2D;

/**
 *
 * @author lanfouf
 */
public class Edge implements Comparable<Edge> {
    private int u;
    private int v;
    private int cout;
    private Line2D line ;

    public Edge(int u, int v,int cout) {
        this.u = u;
        this.v = v;
        this.cout=cout;
    }

    public Edge(int u, int v) {
        this.u = u;
        this.v = v;
        this.cout=1;
    }

    public Edge() {
    }
    

    public int getU() {
        return u;
    }

    public void setU(int u) {
        this.u = u;
    }

    public int getV() {
        return v;
    }

    public void setV(int v) {
        this.v = v;
    }

    public int getCout() {
        return cout;
    }

    public void setCout(int cout) {
        this.cout = cout;
    }

    public Line2D getLine() {
        return line;
    }

    public void setLine(Line2D line) {
        this.line = line;
    }

    
    @Override
    public int compareTo(Edge compareEdge) {
        return this.cout-compareEdge.cout;  
    }

    @Override
    public boolean equals(Object obj) {
        return (this.u==((Edge)obj).u && this.v==((Edge)obj).v)||(this.u==((Edge)obj).v && this.v==((Edge)obj).u);
    }

    @Override
    public Object clone() throws CloneNotSupportedException {
        return super.clone(); //To change body of generated methods, choose Tools | Templates.
    }

    @Override
    public String toString() {
        return "("+u+","+v+")"+" : "+cout;
    }
    
    
}

