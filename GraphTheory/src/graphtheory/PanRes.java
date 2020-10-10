/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package graphtheory;

import Algorithm.Edge;
import Algorithm.Vertex;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.FontMetrics;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Point;
import java.awt.Rectangle;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.geom.AffineTransform;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.util.ArrayList;
import java.util.List;
import javax.swing.JPanel;

/**
 *
 * @author lanfouf
 */
public class PanRes  extends JPanel{
     private int radius = 50;
        private String text = "stack";
        private List<Vertex> v;
        private List<Edge> e;
        private Boolean bool;

        private List<Ellipse2D> nodes;

        private Ellipse2D dragged;
        private Point offset;

        public PanRes() {
       
        }

    PanRes(List<Vertex> v, List<Edge> e,Boolean bool) {
        this.v=v;
        this.e=e;
        this.bool=bool;
             nodes = new ArrayList<>(25);

            nodes.add(new Ellipse2D.Float(50 - (radius / 2), 100 - (radius / 2), radius, radius));
            nodes.add(new Ellipse2D.Float(350 - (radius / 2), 100 - (radius / 2), radius, radius));

            this.addMouseListener(new MouseAdapter() {
                @Override
                public void mousePressed(MouseEvent e) {
//
for (int i = 0; i < v.size(); i++) {
            Ellipse2D get = v.get(i).getNodes();
            if (get.contains(e.getPoint())) {

//                            System.out.println("Clicked...");
                            dragged = get;
                            // Adjust for the different between the top/left corner of the
                            // node and the point it was clicked...
                            offset = new Point(get.getBounds().x - e.getX(), get.getBounds().y - e.getY());
                            // Highlight the clicked node
                            repaint();
                            break;

                        }

        }

//
//                    for (Ellipse2D node : nodes) {
//
//                        if (node.contains(e.getPoint())) {
//
//                            System.out.println("Clicked...");
//                            dragged = node;
//                            // Adjust for the different between the top/left corner of the
//                            // node and the point it was clicked...
//                            offset = new Point(node.getBounds().x - e.getX(), node.getBounds().y - e.getY());
//                            // Highlight the clicked node
//                            repaint();
//                            break;
//
//                        }
//
//                    }

                }

                @Override
                public void mouseReleased(MouseEvent e) {
                    // Erase the "click" highlight
                    if (dragged != null) {
                        repaint();
                    }
                    dragged = null;
                    offset = null;
                }
            });

            addMouseMotionListener(new MouseAdapter() {
                @Override
                public void mouseDragged(MouseEvent e) {
                
                     if (dragged != null && offset != null) {
                        // Adjust the position of the drag point to allow for the
                        // click point offset
                        Point to = e.getPoint();
                        to.x += offset.x;
                        to.y += offset.y;

                        // Modify the position of the node...
                        Rectangle bounds = dragged.getBounds();
                        bounds.setLocation(to);
                        dragged.setFrame(bounds);

                        repaint();
                    }

                }
            });
    }

    public List<Vertex> getV() {
        return v;
    }

    public void setV(List<Vertex> v) {
        this.v = v;
    }

    public List<Edge> getE() {
        return e;
    }

    public void setE(List<Edge> e) {
        this.e = e;
    }

        @Override
        public Dimension getPreferredSize() {
            return new Dimension(400, 400);
        }

        @Override
        protected void paintComponent(Graphics g) {
            // declaration
                 super.paintComponent(g);
        Graphics2D g2d = (Graphics2D)g.create();
        //dessiner les arret
        if (this.bool==true) {
          for (int i = 0; i < this.e.size(); i++) {
            Edge get = this.e.get(i);
            int u=get.getU();
            int v=get.getV();
            Ellipse2D get1 = null;
            Ellipse2D get2 = null;
            for (int j = 0; j < this.v.size(); j++) {
                if (u==this.v.get(j).getNom()) {
                    get1=this.v.get(j).getNodes();
                }
                if (v==this.v.get(j).getNom()) {
                    get2=this.v.get(j).getNodes();
                }    
            }
            Line2D line = new Line2D.Double(get1.getCenterX(), get1.getCenterY(), get2.getCenterX(),get2.getCenterY() );
            get.setLine(line);
            g2d.draw(line);
            
        }  
        }else {
        
             for (int i = 0; i < this.e.size(); i++) {
            Edge get = this.e.get(i);
            int u=get.getU();
            int v=get.getV();
            Ellipse2D get1 = null;
            Ellipse2D get2 = null;
            for (int j = 0; j < this.v.size(); j++) {
                if (u==this.v.get(j).getNom()) {
                    get1=this.v.get(j).getNodes();
                }
                if (v==this.v.get(j).getNom()) {
                    get2=this.v.get(j).getNodes();
                }    
            }   
            int x1=(int) get1.getCenterX();
            int y1=(int)get1.getCenterY();
            int x2=(int)get2.getCenterX();
            int y2=(int)get2.getCenterY();
            
            Graphics2D g3d = (Graphics2D) g.create();

                double dx = x2 - x1, dy = y2 - y1;
                double angle = Math.atan2(dy, dx);
                int len = (int) Math.sqrt(dx*dx + dy*dy);
                AffineTransform at = AffineTransform.getTranslateInstance(x1, y1);
                at.concatenate(AffineTransform.getRotateInstance(angle));
                g3d.transform(at);

                // Draw horizontal arrow starting in (0, 0)
                
                
                Line2D line =new Line2D.Double(x1, y1, x2,y2);
                Line2D line1 =new Line2D.Double(0, 20, len, 0);
                get.setLine(line);
                g3d.draw(line1);
                g3d.fillPolygon(new int[] {len, len-40, len-40, len},
                              new int[] {0, -4, 4, 0}, 4);
            
             
        }
        }
        
        //dessiner vertex
        for (int i = 0; i < this.v.size(); i++) {
            Vertex geto = this.v.get(i);
            g2d.setColor(Color.yellow);
            g2d.fill(this.v.get(i).getNodes());
            g2d.draw(this.v.get(i).getNodes());
            this.v.get(i).setX((int) this.v.get(i).getNodes().getCenterX());
            this.v.get(i).setY((int) this.v.get(i).getNodes().getCenterY());
            //dessiner cout
            g2d.setColor(Color.BLUE);
            FontMetrics fm = g.getFontMetrics();
                int textWidth = fm.stringWidth(String.valueOf(this.v.get(i).getNom()) );
                int x = this.v.get(i).getNodes().getBounds().x;
                int y = this.v.get(i).getNodes().getBounds().y;
                int width = this.v.get(i).getNodes().getBounds().width;
                int height = this.v.get(i).getNodes().getBounds().height;
                g.drawString(String.valueOf(this.v.get(i).getNom()),x + ((width - textWidth)) / 2,y + ((height - fm.getHeight()) / 2) + fm.getAscent());
                

            
            
        }
        if (this.bool==true) {
            for (int i = 0; i < this.e.size(); i++) {
            
            Edge get = this.e.get(i);
            FontMetrics fo = g.getFontMetrics();
            g.setColor(Color.RED);
            g.setFont(new Font("SansSerif", Font.BOLD, 15));
                int textWidth = fo.stringWidth(String.valueOf(get.getCout()) );
                int x = get.getLine().getBounds().x;
                int y = get.getLine().getBounds().y;
                int width = get.getLine().getBounds().width;
                int height = get.getLine().getBounds().height;
                g.drawString(String.valueOf(get.getCout()),x + ((width - textWidth)) / 2,y + ((height - fo.getHeight()) / 2) + fo.getAscent());
        }
        }
        if (this.bool==false) {
            for (int i = 0; i < this.e.size(); i++) {
            
            Edge get = this.e.get(i);
            FontMetrics fo = g.getFontMetrics();
            g.setColor(Color.RED);
            g.setFont(new Font("SansSerif", Font.BOLD, 15));
                int textWidth = fo.stringWidth(String.valueOf(get.getCout()) );
                int x = get.getLine().getBounds().x;
                int y = get.getLine().getBounds().y;
                int width = get.getLine().getBounds().width;
                int height = get.getLine().getBounds().height;
                g.drawString(String.valueOf(get.getCout()),x + ((width - textWidth)) / 2,y + ((height - fo.getHeight()) / 2) + fo.getAscent());
//                System.out.println(String.valueOf(get.getCout()));
        }
        }
        
        
        g2d.dispose();
    }

    
    
}
