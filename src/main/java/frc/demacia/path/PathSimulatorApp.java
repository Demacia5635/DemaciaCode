package frc.demacia.path;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.demacia.utils.log.LogManager;

// JavaFX Imports for the embedded simulation app
import javafx.animation.AnimationTimer;
import javafx.application.Application;
import javafx.scene.Scene;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.layout.StackPane;
import javafx.scene.paint.Color;
import javafx.stage.Stage;

public class demaciaTrajectory extends Application {
    private List<Pose2d> demaciaPathPoint;
    private List<PointPair<Pose2d>> pointPairs;
    private List<Pose2d> pathPoint;
    
    private List<CircleCalculator> allCircles;
    private List<Leg> allLegs;
    
    private double radius = 3.0;

    // Static variables required for the JavaFX Application context launch
    private static List<Pose2d> staticWaypoints = null;
    private static Pose2d simulatedRobotPose;
    private static int currentSimStage = 1;
    private static double tCircleSim = 0.0;

    /**
     * Default constructor required by JavaFX runtime launch mechanism.
     */
    public demaciaTrajectory() {
        // Required for JavaFX launch, do not remove
    }

    public demaciaTrajectory(List<Pose2d> demaciaPathPoint) {
        this.demaciaPathPoint = demaciaPathPoint;
        this.pointPairs = new ArrayList<>();
        this.pathPoint = new ArrayList<>();
        this.allCircles = new ArrayList<>();
        this.allLegs = new ArrayList<>();
        
        // 1. Generate unique combinations of pairs
        for (int i = 0; i < demaciaPathPoint.size() - 1; i++) {
            for (int j = i + 1; j < demaciaPathPoint.size(); j++) {
                PointPair<Pose2d> pointPair = new PointPair<>(demaciaPathPoint.get(i), demaciaPathPoint.get(j));
                this.pointPairs.add(pointPair);
            }
        }

        // 2. Compute turn circles for middle waypoints
        for (int i = 1; i < demaciaPathPoint.size() - 1; i++) {
            Translation2d from = demaciaPathPoint.get(i - 1).getTranslation();
            Translation2d mid = demaciaPathPoint.get(i).getTranslation();
            Translation2d to = demaciaPathPoint.get(i + 1).getTranslation();
            
            CircleCalculator circle = CircleCalculator.calculateCircleCenter(from, to, mid, this.radius);
            allCircles.add(circle);
        }

        // 3. Build straight legs between circles
        for (int i = 1; i < allCircles.size(); i++) {
            CircleCalculator c1 = allCircles.get(i - 1);
            CircleCalculator c2 = allCircles.get(i);
            
            Circle startCircle = new Circle(c1.center, c1.radius, c1.isLeftTurn);
            Circle endCircle = new Circle(c2.center, c2.radius, c2.isLeftTurn);
            
            Leg leg = new Leg(startCircle, endCircle);
            allLegs.add(leg);
            
            this.pathPoint.add(new Pose2d(leg.getStart(), demaciaPathPoint.get(i - 1).getRotation()));
            this.pathPoint.add(new Pose2d(leg.getEnd(), demaciaPathPoint.get(i).getRotation()));
        }
    }

    /**
     * Evaluates where the robot is and computes linear/rotational target ChassisSpeeds.
     */
    public ChassisSpeeds getChassisSpeeds(Pose2d currentRobotPose) {
        Object location = checkRobotLocation(currentRobotPose);
        
        double targetVelocity = 0;
        Rotation2d targetHeading = currentRobotPose.getRotation(); // Fallback to current heading

        if (location instanceof Circle) {
            Circle currentCircle = (Circle) location;
            
            currentCircle.updateDistance(currentRobotPose);
            targetVelocity = currentCircle.getVel();
            
            Translation2d baseVector = currentCircle.center.minus(currentRobotPose.getTranslation());
            targetHeading = currentCircle.getTargetHeading(currentRobotPose.getRotation(), baseVector);
            
        } else if (location instanceof Leg) {
            Leg currentLeg = (Leg) location;
            
            currentLeg.updateDistance(currentRobotPose);
            targetVelocity = currentLeg.getVel();
            
            Translation2d baseVector = currentLeg.getEnd().minus(currentRobotPose.getTranslation());
            targetHeading = currentLeg.getTargetHeading(currentRobotPose.getRotation(), baseVector);
        } else {
            // Off-track: safe stop
            return new ChassisSpeeds(0, 0, 0);
        }

        // Turn scalar speed and rotation into linear components (X and Y velocities)
        double vx = targetVelocity * targetHeading.getCos();
        double vy = targetVelocity * targetHeading.getSin();
        
        // P-control for angular velocity targeting the desired tracking heading
        double headingError = targetHeading.minus(currentRobotPose.getRotation()).getRadians();
        double omega = headingError * 4.0; 

        return new ChassisSpeeds(vx, vy, omega);
    }

    public Object checkRobotLocation(Pose2d currentRobotPose) {
        for (Leg leg : allLegs) {
            if (leg.getStartCircle().isPointInside(currentRobotPose)) {
                return leg.getStartCircle(); 
            }
            if (leg.getEndCircle().isPointInside(currentRobotPose)) {
                return leg.getEndCircle();   
            }
        }

        for (Leg leg : allLegs) {
            if (isRobotNearLine(currentRobotPose, leg)) {
                return leg;
            }
        }
        return null;
    }

    private boolean isRobotNearLine(Pose2d robotPose, Leg leg) {
        double x = robotPose.getX();
        double y = robotPose.getY();
        double x1 = leg.getStartCircle().center.getX();
        double y1 = leg.getStartCircle().center.getY();
        double x2 = leg.getEndCircle().center.getX();
        double y2 = leg.getEndCircle().center.getY();

        double num = Math.abs((y2 - y1) * x - (x2 - x1) * y + x2 * y1 - y2 * x1);
        double den = Math.sqrt(Math.pow(y2 - y1, 2) + Math.pow(x2 - x1, 2));
        return (num / den) <= 0.2; 
    }

    public Pose2d getPointLestPose(){
        return this.demaciaPathPoint.get(demaciaPathPoint.size() - 1);
    }

    public List<Pose2d> getDemaciaPathPoint() {
        return demaciaPathPoint;
    }

    // =========================================================================
    // EMBEDDED JAVAFX SIMULATION APP LOGIC
    // =========================================================================

    @Override
    public void start(Stage primaryStage) {
        final double width = 800;
        final double height = 600;

        // Use default test points if no static configuration was passed before launch
        if (staticWaypoints == null) {
            staticWaypoints = List.of(
                new Pose2d(100, 500, new Rotation2d()),
                new Pose2d(400, 150, new Rotation2d()),
                new Pose2d(700, 450, new Rotation2d())
            );
        }

        // Initialize simulation math parameters (scaled to pixels)
        double visualRadius = 80.0; 
        CircleCalculator circleCalc = CircleCalculator.calculateCircleCenter(
                staticWaypoints.get(0).getTranslation(),
                staticWaypoints.get(2).getTranslation(),
                staticWaypoints.get(1).getTranslation(),
                visualRadius
        );

        Translation2d tangentStart = LegCalculator.pointToCircleTangent(staticWaypoints.get(0), circleCalc.center, circleCalc.radius, circleCalc.isLeftTurn);
        Translation2d tangentEnd = LegCalculator.pointToCircleTangent(staticWaypoints.get(2), circleCalc.center, circleCalc.radius, !circleCalc.isLeftTurn);

        simulatedRobotPose = staticWaypoints.get(0);

        Canvas canvas = new Canvas(width, height);
        GraphicsContext gc = canvas.getGraphicsContext2D();

        // Main 60FPS UI refresh loop
        AnimationTimer timer = new AnimationTimer() {
            @Override
            public void handle(long now) {
                // Physics update
                double speed = 3.0;
                if (currentSimStage == 1) {
                    double dx = tangentStart.getX() - simulatedRobotPose.getX();
                    double dy = tangentStart.getY() - simulatedRobotPose.getY();
                    if (Math.hypot(dx, dy) < speed) {
                        currentSimStage = 2;
                    } else {
                        Rotation2d heading = new Rotation2d(Math.atan2(dy, dx));
                        simulatedRobotPose = new Pose2d(simulatedRobotPose.getX() + heading.getCos() * speed, simulatedRobotPose.getY() + heading.getSin() * speed, heading);
                    }
                } else if (currentSimStage == 2) {
                    tCircleSim += 0.015;
                    if (tCircleSim >= 1.0) {
                        currentSimStage = 3;
                        tCircleSim = 1.0;
                    }
                    double startAngle = Math.atan2(tangentStart.getY() - circleCalc.center.getY(), tangentStart.getX() - circleCalc.center.getX());
                    double endAngle = Math.atan2(tangentEnd.getY() - circleCalc.center.getY(), tangentEnd.getX() - circleCalc.center.getX());
                    double diffAngle = endAngle - startAngle;
                    while (diffAngle > Math.PI) diffAngle -= 2 * Math.PI;
                    while (diffAngle < -Math.PI) diffAngle += 2 * Math.PI;

                    double currentArcAngle = startAngle + diffAngle * tCircleSim;
                    Rotation2d heading = new Rotation2d(currentArcAngle + (circleCalc.isLeftTurn ? -Math.PI / 2 : Math.PI / 2));
                    simulatedRobotPose = new Pose2d(circleCalc.center.getX() + circleCalc.radius * Math.cos(currentArcAngle), circleCalc.center.getY() + circleCalc.radius * Math.sin(currentArcAngle), heading);
                } else if (currentSimStage == 3) {
                    Translation2d target = staticWaypoints.get(2).getTranslation();
                    double dx = target.getX() - simulatedRobotPose.getX();
                    double dy = target.getY() - simulatedRobotPose.getY();
                    if (Math.hypot(dx, dy) > speed) {
                        Rotation2d heading = new Rotation2d(Math.atan2(dy, dx));
                        simulatedRobotPose = new Pose2d(simulatedRobotPose.getX() + heading.getCos() * speed, simulatedRobotPose.getY() + heading.getSin() * speed, heading);
                    } else {
                        currentSimStage = 1;
                        tCircleSim = 0.0;
                        simulatedRobotPose = staticWaypoints.get(0);
                    }
                }

                // Render Canvas
                gc.setFill(Color.web("#1e1e2e"));
                gc.fillRect(0, 0, width, height);

                // Guide lines
                gc.setStroke(Color.web("#45475a"));
                gc.setLineWidth(1.5);
                gc.strokeLine(staticWaypoints.get(0).getX(), staticWaypoints.get(0).getY(), staticWaypoints.get(1).getX(), staticWaypoints.get(1).getY());
                gc.strokeLine(staticWaypoints.get(1).getX(), staticWaypoints.get(1).getY(), staticWaypoints.get(2).getX(), staticWaypoints.get(2).getY());

                // Waypoint markers
                for (int i = 0; i < staticWaypoints.size(); i++) {
                    Pose2d p = staticWaypoints.get(i);
                    gc.setFill(Color.web("#f38ba8"));
                    gc.fillOval(p.getX() - 6, p.getY() - 6, 12, 12);
                    gc.setFill(Color.WHITE);
                    gc.fillText("P" + i, p.getX() + 12, p.getY() - 6);
                }

                // Turn Circle Guide
                gc.setStroke(Color.rgb(166, 227, 161, 0.3));
                gc.strokeOval(circleCalc.center.getX() - circleCalc.radius, circleCalc.center.getY() - circleCalc.radius, circleCalc.radius * 2, circleCalc.radius * 2);

                // Valid Generated Path (Yellow)
                gc.setStroke(Color.web("#f9e2af"));
                gc.setLineWidth(3);
                gc.strokeLine(staticWaypoints.get(0).getX(), staticWaypoints.get(0).getY(), tangentStart.getX(), tangentStart.getY());
                gc.strokeLine(tangentEnd.getX(), tangentEnd.getY(), staticWaypoints.get(2).getX(), staticWaypoints.get(2).getY());

                // Draw Simulated Robot
                gc.save();
                gc.translate(simulatedRobotPose.getX(), simulatedRobotPose.getY());
                gc.rotate(Math.toDegrees(simulatedRobotPose.getRotation().getRadians()));
                gc.setFill(Color.web("#89b4fa"));
                gc.fillOval(-12, -12, 24, 24);
                gc.setStroke(Color.WHITE);
                gc.strokeLine(0, 0, 20, 0);
                gc.restore();
            }
        };
        timer.start();

        primaryStage.setTitle("Demacia Path Simulator - Embedded");
        primaryStage.setScene(new Scene(new StackPane(canvas), width, height));
        primaryStage.show();
    }

    /**
     * Local executable main function to easily test and launch the visual simulator window.
     */
    public static void main(String[] args) {
        // Mock default Waypoint positions for standalone app runtime execution
        staticWaypoints = List.of(
            new Pose2d(100, 500, new Rotation2d()),
            new Pose2d(400, 150, new Rotation2d()),
            new Pose2d(700, 450, new Rotation2d())
        );
        launch(args);
    }
}