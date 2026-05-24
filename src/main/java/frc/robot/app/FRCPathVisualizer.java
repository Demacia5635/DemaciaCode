import javax.swing.*;
import java.awt.*;
import java.awt.event.MouseEvent;
import java.awt.event.MouseMotionAdapter;
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
    private static final int POINT_RADIUS = 8;

    private BufferedImage backgroundImage;
    private List<FieldPoint> path;

    public FRCPathVisualizer() {
        // טעינת תמונת הרקע
        try {
            backgroundImage = ImageIO.read(new File("field.png"));
        } catch (Exception e) {
            System.out.println("שגיאה: לא ניתן למצוא את הקובץ field.png, מציג רקע אפור.");
        }

        // הגדרת נקודות המסלול (X, Y במטרים, זוית נוכחית ברדיאנים, והאם פונים שמאלה)
        path = new ArrayList<>();
        path.add(new FieldPoint(2.0, 2.0, 0.0, false));       // נקודת התחלה
        path.add(new FieldPoint(6.0, 3.5, 0.4, true));        // פנייה שמאלה
        path.add(new FieldPoint(10.0, 5.0, -0.2, false));     // פנייה ימינה
        path.add(new FieldPoint(14.0, 3.0, 0.0, false));

        // האזנה לתנועת העכבר
        addMouseMotionListener(new MouseMotionAdapter() {
            @Override
            public void mouseMoved(MouseEvent e) {
                boolean foundPoint = false;

                for (int i = 0; i < path.size(); i++) {
                    FieldPoint point = path.get(i);
                    int pixelX = meterToPixelX(point.x);
                    int pixelY = meterToPixelY(point.y);

                    double distance = Math.hypot(e.getX() - pixelX, e.getY() - pixelY);
                    if (distance <= POINT_RADIUS) {
                        
                        // ביצוע החישובים שביקשת עבור הנקודה הנוכחית ביחס לבאה אחריה
                        String infoHtml = "<html><b>מידע על הנקודה:</b><br>";
                        if (i < path.size() - 1) {
                            PathResult result = calculateRobotPath(point, path.get(i + 1));
                            infoHtml += String.format("X: %.2fm, Y: %.2fm<br>", point.x, point.y);
                            infoHtml += String.format("<b>מהירות (v):</b> %.2f m/s<br>", result.v);
                            infoHtml += String.format("<b>כיוון (h):</b> %.2f deg<br>", Math.toDegrees(result.h));
                            infoHtml += String.format("<b>מרחק (dist):</b> %.2f m", result.dist);
                        } else {
                            infoHtml += String.format("נקודת סיום המסלול<br>X: %.2fm, Y: %.2fm", point.x, point.y);
                        }

                        setToolTipText(infoHtml);
                        foundPoint = true;
                        break;
                    }
                }

                if (!foundPoint) {
                    setToolTipText(null);
                }
            }
        });
        
        ToolTipManager.sharedInstance().setInitialDelay(0);
        
        // הדפסת החישובים לטרמינל להרצה ראשונית
        printPathCalculations();
    }

    /**
     * פונקציית החישוב המרכזית לפי הנוסחאות שלך
     */
    private PathResult calculateRobotPath(FieldPoint p1, FieldPoint p2) {
        // הגדרת קבועים לחישוב (רדיוס סיבוב ופרמטרים של המגרש)
        double r = 1.0; // רדיוס סיבוב מוגדר כ-1 מטר למשל
        
        // מרחק בין הנקודות d
        double d = Math.hypot(p2.x - p1.x, p2.y - p1.y);
        if (d < r) d = r + 0.1; // הגנה מתמטית מפני חלוקה באפס ב-asin

        // זוית בין p1 ל-p2
        double angleP1ToP2 = Math.atan2(p2.y - p1.y, p2.x - p1.x);

        // חישוב הזווית b לפי כיוון הפנייה
        double b;
        if (p1.turnLeft) {
            // במידה ופונים שמאלה
            b = Math.asin(r / d) + angleP1ToP2;
        } else {
            // במידה ופונים ימינה
            b = angleP1ToP2 - Math.asin(r / d);
        }

        // חישוב כיוון (Heading) לפי הנוסחה: h = 2 * currentAngle + baseVector.angle()
        // נתייחס ל-b כזווית הווקטור הבסיסי הנוצר מהחישוב
        double heading = 2 * p1.currentAngle + b;

        // חישוב מרחק נסיעה הכולל (dist) - שילוב של קשת (Arch) וקווים ישרים במטרים
        double deltaA = Math.abs(heading - p1.currentAngle);
        double dist = r * deltaA + (Math.sqrt(d * d - r * r)); // שילוב פיתגורס וקשת המעגל לפי הנוסחה

        // חישוב מהירות (v) לפי פרופיל טרפזואידי פשוט (Trapezoidal Velocity Profile)
        // המהירות נקבעת לפי המרחק (מאיצה, נשארת קבועה, ומאיטה לקראת העצירה)
        double maxVelocity = 3.0; // מטרים לשנייה (מהירות מקסימלית של רובוט FRC ממוצע)
        double v = Math.min(maxVelocity, Math.sqrt(2 * 1.5 * dist)); // חישוב קינמטי בסיסי לטרפז

        return new PathResult(v, heading, dist);
    }

    private void printPathCalculations() {
        System.out.println("--- חישובי מסלול רובוט ---");
        for (int i = 0; i < path.size() - 1; i++) {
            PathResult res = calculateRobotPath(path.get(i), path.get(i + 1));
            System.out.printf("מנקודה %d ל-%d -> מהירות: %.2f, כיוון: %.2f מעלות, מרחק: %.2f מטרים\n", 
                    i, i+1, res.v, Math.toDegrees(res.h), res.dist);
        }
    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        // 1. ציור הרקע
        if (backgroundImage != null) {
            g2d.drawImage(backgroundImage, 0, 0, WINDOW_WIDTH, WINDOW_HEIGHT, null);
        } else {
            g2d.setColor(Color.LIGHT_GRAY);
            g2d.fillRect(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);
        }

        // 2. ציור קווי המסלול
        g2d.setColor(Color.RED);
        g2d.setStroke(new BasicStroke(3));
        for (int i = 0; i < path.size() - 1; i++) {
            int x1 = meterToPixelX(path.get(i).x);
            int y1 = meterToPixelY(path.get(i).y);
            int x2 = meterToPixelX(path.get(i + 1).x);
            int y2 = meterToPixelY(path.get(i + 1).y);
            g2d.drawLine(x1, y1, x2, y2);
        }

        // 3. ציור נקודות הציון
        for (FieldPoint point : path) {
            int x = meterToPixelX(point.x);
            int y = meterToPixelY(point.y);

            g2d.setColor(point.turnLeft ? Color.CYAN : Color.YELLOW); // צבע שונה לפנייה שמאלה/ימינה
            g2d.fillOval(x - POINT_RADIUS, y - POINT_RADIUS, POINT_RADIUS * 2, POINT_RADIUS * 2);
            
            g2d.setColor(Color.BLACK);
            g2d.setStroke(new BasicStroke(1.5f));
            g2d.drawOval(x - POINT_RADIUS, y - POINT_RADIUS, POINT_RADIUS * 2, POINT_RADIUS * 2);
        }
    }

    private int meterToPixelX(double metersX) {
        return (int) ((metersX / FIELD_WIDTH_METERS) * WINDOW_WIDTH);
    }

    private int meterToPixelY(double metersY) {
        return (int) ((metersY / FIELD_HEIGHT_METERS) * WINDOW_HEIGHT);
    }

    // אובייקט נקודה מורחב המכיל נתוני כיוון ופניות
    static class FieldPoint {
        double x, y;
        double currentAngle; // הזווית הנוכחית של הרובוט ברדיאנים
        boolean turnLeft;    // האם הפנייה הבאה היא שמאלה או ימינה

        public FieldPoint(double x, double y, double currentAngle, boolean turnLeft) {
            this.x = x;
            this.y = y;
            this.currentAngle = currentAngle;
            this.turnLeft = turnLeft;
        }
    }

    // אובייקט המחזיר את הטאפל שביקשת (v, h, dist)
    static class PathResult {
        double v;    // מהירות (Velocity)
        double h;    // כיוון (Heading/Angle)
        double dist; // מרחק (Distance)

        public PathResult(double v, double h, double dist) {
            this.v = v;
            this.h = h;
            this.dist = dist;
        }
    }

    public static void main(String[] args) {
        JFrame frame = new JFrame("FRC Path Tracker with Math Logic");
        FRCPathVisualizer panel = new FRCPathVisualizer();
        frame.add(panel);
        frame.setSize(WINDOW_WIDTH, WINDOW_HEIGHT + 38);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLocationRelativeTo(null);
        frame.setResizable(false);
        frame.setVisible(true);
    }
}