package frc.robot.app;

import javax.swing.*;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.File;
import javax.imageio.ImageIO;
import java.util.ArrayList;
import java.util.List;

public class FRCPathVisualizer extends JPanel {

    private static final double FIELD_WIDTH_METERS = 16.54;
    private static final double FIELD_HEIGHT_METERS = 8.21;
    private static final int WINDOW_WIDTH = 1000;
    private static final int WINDOW_HEIGHT = 500;

    private BufferedImage backgroundImage;
    private List<FieldPoint> path;

    public FRCPathVisualizer() {
        try {
            backgroundImage = ImageIO.read(new File("field.png"));
        } catch (Exception e) {
            System.out.println("שגיאה: לא ניתן למצוא את הקובץ field.png באותה תיקייה!");
        }

       
        path = new ArrayList<>();
        path.add(new FieldPoint(1.0, 1.0));
        path.add(new FieldPoint(4.0, 2.5));
        path.add(new FieldPoint(8.0, 4.0));
        path.add(new FieldPoint(12.0, 2.5));
        path.add(new FieldPoint(15.0, 7.0));
    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        if (backgroundImage != null) {
            g2d.drawImage(backgroundImage, 0, 0, WINDOW_WIDTH, WINDOW_HEIGHT, null);
        } else {
            g2d.setColor(Color.LIGHT_GRAY);
            g2d.fillRect(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);
        }

        g2d.setColor(Color.RED);
        g2d.setStroke(new BasicStroke(3));
        for (int i = 0; i < path.size() - 1; i++) {
            int x1 = meterToPixelX(path.get(i).x);
            int y1 = meterToPixelY(path.get(i).y);
            int x2 = meterToPixelX(path.get(i + 1).x);
            int y2 = meterToPixelY(path.get(i + 1).y);
            g2d.drawLine(x1, y1, x2, y2);
        }

        
        for (FieldPoint point : path) {
            int x = meterToPixelX(point.x);
            int y = meterToPixelY(point.y);

          
            g2d.setColor(Color.YELLOW);
            g2d.fillOval(x - 6, y - 6, 12, 12);
            g2d.setColor(Color.BLACK);
            g2d.setStroke(new BasicStroke(1.5f));
            g2d.drawOval(x - 6, y - 6, 12, 12);
        }
    }

    private int meterToPixelX(double metersX) {
        return (int) ((metersX / FIELD_WIDTH_METERS) * WINDOW_WIDTH);
    }

    private int meterToPixelY(double metersY) {
        return (int) ((metersY / FIELD_HEIGHT_METERS) * WINDOW_HEIGHT);
    }

    static class FieldPoint {
        double x, y;
        public FieldPoint(double x, double y) { this.x = x; this.y = y; }
    }

    public static void main(String[] args) {
        JFrame frame = new JFrame("FRC Autonomous Path Visualizer (Swing)");
        FRCPathVisualizer panel = new FRCPathVisualizer();
        frame.add(panel);
        frame.setSize(WINDOW_WIDTH, WINDOW_HEIGHT + 38);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLocationRelativeTo(null);
        frame.setResizable(false);
        frame.setVisible(true);
    }
}